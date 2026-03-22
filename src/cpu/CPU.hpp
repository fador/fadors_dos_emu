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
  };

  bool isLastInt32() const {
    return m_hleStack.empty() ? false : m_hleStack.back().is32;
  }
  const HLEFrame &lastHLEFrame() const { return m_hleStack.back(); }
  HLEFrame &lastHLEFrameMut() { return m_hleStack.back(); }
  void pushHLEFrame(bool is32, uint8_t vector = 0xFF) {
    m_hleStack.push_back({is32, 0, 0, false, vector});
  }
  void popHLEFrame() {
    if (!m_hleStack.empty())
      m_hleStack.pop_back();
  }
  size_t hleStackSize() const { return m_hleStack.size(); }

  // Pop an HLE frame whose framePhysAddr matches the given address.
  // Used by IRET to clean up frames for interrupts handled entirely
  // by thunks (no forwarding to the 0F FF stub).
  // Returns true if a frame was found and popped.
  bool popHLEFrameByPhysAddr(uint32_t physAddr) {
    for (auto it = m_hleStack.rbegin(); it != m_hleStack.rend(); ++it) {
      if (it->framePhysAddr == physAddr) {
        m_hleStack.erase(std::prev(it.base()));
        return true;
      }
    }
    return false;
  }

  // Find and remove the most recent HLE frame for the given vector.
  // Returns a copy of the frame, or a default frame if not found.
  HLEFrame popHLEFrameForVector(uint8_t vector) {
    for (auto it = m_hleStack.rbegin(); it != m_hleStack.rend(); ++it) {
      if (it->vector == vector) {
        HLEFrame result = *it;
        // Erase all frames from this one to the end (orphans above it)
        m_hleStack.erase(std::prev(it.base()), m_hleStack.end());
        return result;
      }
    }
    // Not found — return default
    return {false, 0, 0, false, vector};
  }

  // Peek at the most recent HLE frame for a given vector (non-destructive).
  HLEFrame peekHLEFrameForVector(uint8_t vector) const {
    for (auto it = m_hleStack.rbegin(); it != m_hleStack.rend(); ++it) {
      if (it->vector == vector)
        return *it;
    }
    return {false, 0, 0, false, vector};
  }

  // Stack operations
  void push16(uint16_t value);
  void push32(uint32_t value);
  uint16_t pop16();
  uint32_t pop32();

  uint64_t getCycles() const { return m_cycles; }
  void addCycles(uint32_t c) { m_cycles += c; }

  uint32_t getEIP() const { return m_eip; }
  void setEIP(uint32_t val);

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
