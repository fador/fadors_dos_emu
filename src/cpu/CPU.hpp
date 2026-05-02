#pragma once
#include <array>
#include <cstdint>
#include <vector>

namespace fador::memory {
class MemoryBus;
}

namespace fador::cpu {

enum RegIndex { EAX = 0, ECX, EDX, EBX, ESP, EBP, ESI, EDI };

enum Reg16Index {
  AX = 0,
  CX = 1,
  DX = 2,
  BX = 3,
  SP = 4,
  BP = 5,
  SI = 6,
  DI = 7
};

enum Reg8Index {
  AL = 0,
  CL = 1,
  DL = 2,
  BL = 3,
  AH = 4,
  CH = 5,
  DH = 6,
  BH = 7
};

enum SegRegIndex { ES = 0, CS, SS, DS, FS, GS };

typedef RegIndex Reg32Index;

// EFLAGS bits
static constexpr uint32_t FLAG_CARRY = 0x0001;
static constexpr uint32_t FLAG_PARITY = 0x0004;
static constexpr uint32_t FLAG_AUX = 0x0010;
static constexpr uint32_t FLAG_ZERO = 0x0040;
static constexpr uint32_t FLAG_SIGN = 0x0080;
static constexpr uint32_t FLAG_TRAP = 0x0100;
static constexpr uint32_t FLAG_INTERRUPT = 0x0200;
static constexpr uint32_t FLAG_DIRECTION = 0x0400;
static constexpr uint32_t FLAG_OVERFLOW = 0x0800;

struct DescriptorRegister {
  uint16_t limit;
  uint32_t base;
};

class CPU {
public:
  CPU();
  void reset();

  uint32_t getReg32(uint8_t index) const;
  void setReg32(uint8_t index, uint32_t value);
  uint16_t getReg16(uint8_t index) const;
  void setReg16(uint8_t index, uint16_t value);
  uint8_t getReg8(uint8_t index) const;
  void setReg8(uint8_t index, uint8_t value);

  uint16_t getSegReg(uint8_t index) const { return m_segRegs[index % 6]; }
  void setSegReg(uint8_t index, uint16_t value) {
    m_segRegs[index % 6] = value;
  }

  uint32_t getSegBase(uint8_t index) const { return m_segBase[index % 6]; }
  void setSegBase(uint8_t index, uint32_t value) {
    m_segBase[index % 6] = value;
  }

  bool is32BitCode() const { return m_is32BitCode; }
  void setIs32BitCode(bool val) { m_is32BitCode = val; }
  bool is32BitStack() const { return m_is32BitStack; }
  void setIs32BitStack(bool val) { m_is32BitStack = val; }

  // HLE frame tracking for interrupt dispatch.
  // Stores the IRET frame location so the 0F FF handler can find it
  // even if client thunks modify ESP between the push and the handler.
  struct HLEFrame {
    bool is32;
    uint32_t framePhysAddr; // Physical address of IRET frame in memory
    uint32_t frameSP;       // SP/ESP value at push time (for restoring)
    bool stackIs32;         // Whether stack was 32-bit at push time
    uint8_t vector;         // Interrupt vector that created this frame
    bool dpmiStackSwitch = false; // Reflection stack was used
    uint32_t origESP = 0;        // Caller's original ESP before switch
    uint16_t origSS = 0;         // Caller's original SS before switch
    uint32_t origEIP = 0;        // Caller's original EIP before switch
    uint16_t origCS = 0;         // Caller's original CS before switch
    uint32_t origEFLAGS = 0;     // Caller's original EFLAGS before switch
    uint16_t origDS = 0;          // Caller's original DS before switch
    uint16_t origES = 0;          // Caller's original ES before switch
    bool dispatchedToPM = false;  // Thunk dispatched to PM handler
    // GPRs + extra segments saved at thunk→PM dispatch, restored at PM-OK
    uint32_t savedEAX = 0, savedEBX = 0, savedECX = 0, savedEDX = 0;
    uint32_t savedESI = 0, savedEDI = 0, savedEBP = 0;
    uint16_t savedFS = 0, savedGS = 0;
  };

  bool isLastInt32() const {
    return m_hleStack.empty() ? false : m_hleStack.back().is32;
  }
  const HLEFrame &lastHLEFrame() const { return m_hleStack.back(); }
  HLEFrame &lastHLEFrameMut() { return m_hleStack.back(); }
  void pushHLEFrame(bool is32, uint8_t vector = 0xFF) {
    m_hleStack.push_back({is32, 0, 0, false, vector, false, 0, 0});
  }
  void popHLEFrame() {
    if (!m_hleStack.empty())
      m_hleStack.pop_back();
  }
  size_t hleStackSize() const { return m_hleStack.size(); }
  const HLEFrame &hleFrameAt(size_t i) const { return m_hleStack[i]; }

  // Pop an HLE frame whose framePhysAddr matches the given address.
  // Used by IRET to clean up frames for interrupts handled entirely
  // by thunks (no forwarding to the 0F FF stub).
  // Returns true if a frame was found and popped.
  bool popHLEFrameByPhysAddr(uint32_t physAddr) {
    auto f = popAndGetHLEFrameByPhysAddr(physAddr);
    return f.framePhysAddr != 0;
  }

  // Pop an HLE frame whose framePhysAddr matches the given address and
  // return a copy. Returns a frame with framePhysAddr==0 if not found.
  HLEFrame popAndGetHLEFrameByPhysAddr(uint32_t physAddr) {
    // First try exact match (fast path)
    for (auto it = m_hleStack.rbegin(); it != m_hleStack.rend(); ++it) {
      if (it->framePhysAddr == physAddr) {
        HLEFrame result = *it;
        m_hleStack.erase(std::prev(it.base()));
        return result;
      }
    }

    // Fallback: tolerate small offsets between recorded frame address
    // and observed IRET/SP-derived address. Small differences can occur
    // when privilege-change stack words are pushed or when stack
    // width interpretation varies; prefer not to disturb frames that
    // are clearly distinct (use a small tolerance).
    constexpr uint32_t TOLERANCE = 12; // bytes
    for (auto it = m_hleStack.rbegin(); it != m_hleStack.rend(); ++it) {
      uint32_t a = it->framePhysAddr;
      uint32_t b = physAddr;
      if (a >= b) {
        if (a - b <= TOLERANCE) {
          HLEFrame result = *it;
          m_hleStack.erase(std::prev(it.base()));
          return result;
        }
      } else {
        if (b - a <= TOLERANCE) {
          HLEFrame result = *it;
          m_hleStack.erase(std::prev(it.base()));
          return result;
        }
      }
    }
    return {false, 0, 0, false, 0xFF, false, 0, 0, 0, 0};
  }

  // Pop a dpmiStackSwitch frame whose origCS:origEIP matches the given
  // CS:EIP.  Used as a fallback when physical-address matching fails
  // because the DOS/4GW thunk rearranges its internal stack.
  HLEFrame popDpmiFrameByCSEIP(uint16_t cs, uint32_t eip) {
    // Exact 32-bit match
    for (auto it = m_hleStack.rbegin(); it != m_hleStack.rend(); ++it) {
      if (it->origCS == cs && it->origEIP == eip && cs != 0) {
        HLEFrame result = *it;
        m_hleStack.erase(std::prev(it.base()));
        return result;
      }
    }
    // Fallback: tolerate 16-bit EIP truncation. A 16-bit IRET on a
    // 32-bit frame restores only the low 16 bits of EIP (the high 16
    // bits come from the zero-extension of the 16-bit pop). Match when
    // CS is identical and the low 16 bits of both EIPs agree.
    for (auto it = m_hleStack.rbegin(); it != m_hleStack.rend(); ++it) {
      if (it->origCS == cs && cs != 0 &&
          (it->origEIP & 0xFFFF) == (eip & 0xFFFF) &&
          it->origEIP != eip) {
        HLEFrame result = *it;
        m_hleStack.erase(std::prev(it.base()));
        return result;
      }
    }
    return {false, 0, 0, false, 0xFF, false, 0, 0, 0, 0};
  }

  // Peek at the most recent undispatched dpmiStackSwitch frame whose
  // vector is a hardware IRQ (0x08-0x0F master, 0x70-0x77 slave).
  // Non-destructive: returns a copy without removing from the stack.
  // Used to detect thunk → PM handler dispatch.
  HLEFrame peekUndispatchedHwIrqFrame() const {
    for (auto it = m_hleStack.rbegin(); it != m_hleStack.rend(); ++it) {
      if (it->dpmiStackSwitch && !it->dispatchedToPM) {
        uint8_t v = it->vector;
        bool isHwIrq = (v >= 0x08 && v <= 0x0F) || (v >= 0x70 && v <= 0x77);
        if (isHwIrq) return *it;
      }
    }
    return {false, 0, 0, false, 0xFF, false, 0, 0, 0, 0};
  }

  // Find the outermost (first) dpmiStackSwitch frame on the HLE stack.
  // Returns the application's original SS:ESP before any reflection.
  // Used to give the PM handler the app's stack when a HW IRQ fires
  // inside a thunk (nested reflection).
  const HLEFrame *peekOutermostDpmiFrame() const {
    for (auto it = m_hleStack.begin(); it != m_hleStack.end(); ++it) {
      if (it->dpmiStackSwitch) return &(*it);
    }
    return nullptr;
  }

  // Mark the most recent undispatched dpmiStackSwitch hw IRQ frame as
  // dispatched to PM handler (in-place).  Also saves current GPRs + FS/GS
  // so they can be restored when the PM handler's IRET is intercepted.
  bool markDpmiFrameDispatched() {
    for (auto it = m_hleStack.rbegin(); it != m_hleStack.rend(); ++it) {
      if (it->dpmiStackSwitch && !it->dispatchedToPM) {
        uint8_t v = it->vector;
        bool isHwIrq = (v >= 0x08 && v <= 0x0F) || (v >= 0x70 && v <= 0x77);
        if (isHwIrq) {
          it->dispatchedToPM = true;
          it->savedEAX = m_regs[EAX]; it->savedEBX = m_regs[EBX];
          it->savedECX = m_regs[ECX]; it->savedEDX = m_regs[EDX];
          it->savedESI = m_regs[ESI]; it->savedEDI = m_regs[EDI];
          it->savedEBP = m_regs[EBP];
          it->savedFS = m_segRegs[FS]; it->savedGS = m_segRegs[GS];
          return true;
        }
      }
    }
    return false;
  }

  // Pop the most recent dpmiStackSwitch frame that was dispatched to a
  // PM handler.  Used to intercept the PM handler's IRET (which may use
  // the wrong operand size) and restore full state.
  HLEFrame popDpmiFrameByDispatchedPM() {
    for (auto it = m_hleStack.rbegin(); it != m_hleStack.rend(); ++it) {
      if (it->dpmiStackSwitch && it->dispatchedToPM) {
        HLEFrame result = *it;
        m_hleStack.erase(std::prev(it.base()));
        return result;
      }
    }
    return {false, 0, 0, false, 0xFF, false, 0, 0, 0, 0};
  }

  // Peek (non-destructive) at the most recent dispatchedToPM frame.
  const HLEFrame *peekDpmiFrameByDispatchedPM() const {
    for (auto it = m_hleStack.rbegin(); it != m_hleStack.rend(); ++it) {
      if (it->dpmiStackSwitch && it->dispatchedToPM)
        return &(*it);
    }
    return nullptr;
  }

  // Find and remove the most recent HLE frame for the given vector.
  // Returns a copy of the frame, or a default frame if not found.
  // Skips frames with dpmiStackSwitch=true — those belong to the outer
  // interrupt dispatch and should only be consumed by the IRET handler.
  HLEFrame popHLEFrameForVector(uint8_t vector) {
    for (auto it = m_hleStack.rbegin(); it != m_hleStack.rend(); ++it) {
      if (it->vector == vector && !it->dpmiStackSwitch) {
        HLEFrame result = *it;
        // Erase all frames from this one to the end (orphans above it)
        m_hleStack.erase(std::prev(it.base()), m_hleStack.end());
        return result;
      }
    }
    // Not found — return default
    return {false, 0, 0, false, vector, false, 0, 0, 0, 0, 0};
  }

  // Peek at the most recent HLE frame for a given vector (non-destructive).
  // Skips dpmiStackSwitch frames (see popHLEFrameForVector).
  HLEFrame peekHLEFrameForVector(uint8_t vector) const {
    for (auto it = m_hleStack.rbegin(); it != m_hleStack.rend(); ++it) {
      if (it->vector == vector && !it->dpmiStackSwitch)
        return *it;
    }
    return {false, 0, 0, false, vector, false, 0, 0, 0, 0, 0};
  }

  // Stack operations
  void push16(uint16_t value);
  void push32(uint32_t value);
  uint16_t pop16();
  uint32_t pop32();

  uint64_t getCycles() const { return m_cycles; }
  const uint64_t &getCyclesRef() const { return m_cycles; }
  void addCycles(uint32_t c) { m_cycles += c; }

  uint32_t getEIP() const { return m_eip; }
  void setEIP(uint32_t val);

  uint32_t getInstructionStartEIP() const { return m_instructionStartEIP; }
  void setInstructionStartEIP(uint32_t val) { m_instructionStartEIP = val; }

  uint32_t getEFLAGS() const { return m_eflags; }
  void setEFLAGS(uint32_t val) { m_eflags = val; }

  uint32_t getCR(uint8_t index) const { return m_cr[index & 7]; }
  void setCR(uint8_t index, uint32_t val) { m_cr[index & 7] = val; }

  uint32_t getDR(uint8_t index) const { return m_dr[index & 7]; }
  void setDR(uint8_t index, uint32_t val) { m_dr[index & 7] = val; }

  DescriptorRegister getGDTR() const { return m_gdtr; }
  void setGDTR(DescriptorRegister val) { m_gdtr = val; }
  DescriptorRegister getIDTR() const { return m_idtr; }
  void setIDTR(DescriptorRegister val) { m_idtr = val; }
  DescriptorRegister getLDTR() const { return m_ldtr; }
  void setLDTR(DescriptorRegister val) { m_ldtr = val; }
  DescriptorRegister getTR() const { return m_tr; }
  void setTR(DescriptorRegister val) { m_tr = val; }
  uint16_t getLDTRSelector() const { return m_ldtrSelector; }
  void setLDTRSelector(uint16_t val) { m_ldtrSelector = val; }
  uint16_t getTRSelector() const { return m_trSelector; }
  void setTRSelector(uint16_t val) { m_trSelector = val; }

  void setMemoryBus(fador::memory::MemoryBus *memory) { m_memory = memory; }

private:
  uint64_t m_cycles{0};
  fador::memory::MemoryBus *m_memory{nullptr};
  std::array<uint32_t, 8> m_regs{};    // EAX, ECX, EDX, EBX, ESP, EBP, ESI, EDI
  std::array<uint16_t, 6> m_segRegs{}; // ES, CS, SS, DS, FS, GS
  std::array<uint32_t, 6> m_segBase{}; // Cached bases for PM/RM
  bool m_is32BitCode{false};           // CS D/B bit status
  bool m_is32BitStack{false};          // SS B bit status
  uint32_t m_eip{0};
  uint32_t m_instructionStartEIP{0};
  uint32_t m_eflags{0x00000002}; // bit 1 is always reserved as 1

  std::array<uint32_t, 8> m_cr{};
  std::array<uint32_t, 8> m_dr{};

  DescriptorRegister m_gdtr{0xFFFF, 0};
  DescriptorRegister m_idtr{0xFFFF, 0};
  DescriptorRegister m_ldtr{0xFFFF, 0};
  DescriptorRegister m_tr{0xFFFF, 0};
  uint16_t m_ldtrSelector{0};
  uint16_t m_trSelector{0};

  std::vector<HLEFrame> m_hleStack;
};

} // namespace fador::cpu
