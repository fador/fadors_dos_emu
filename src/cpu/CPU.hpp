#pragma once
#include "../memory/MemoryBus.hpp"
#include "x87/FPU.hpp"
#include "CpuModel.hpp"
#include "LazyFlags.hpp"
#include <array>
#include <cstdint>
#include <functional>
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

static constexpr uint16_t FPU_CONTROL_DEFAULT = 0x037F;
static constexpr uint16_t FPU_STATUS_IE = 0x0001;
static constexpr uint16_t FPU_STATUS_DE = 0x0002;
static constexpr uint16_t FPU_STATUS_ZE = 0x0004;
static constexpr uint16_t FPU_STATUS_OE = 0x0008;
static constexpr uint16_t FPU_STATUS_UE = 0x0010;
static constexpr uint16_t FPU_STATUS_PE = 0x0020;
static constexpr uint16_t FPU_STATUS_SF = 0x0040;
static constexpr uint16_t FPU_STATUS_ES = 0x0080;
static constexpr uint16_t FPU_STATUS_C0 = 0x0100;
static constexpr uint16_t FPU_STATUS_C1 = 0x0200;
static constexpr uint16_t FPU_STATUS_C2 = 0x0400;
static constexpr uint16_t FPU_STATUS_TOP_MASK = 0x3800;
static constexpr uint16_t FPU_STATUS_C3 = 0x4000;
static constexpr uint16_t FPU_STATUS_BUSY = 0x8000;
static constexpr uint16_t FPU_EXCEPTION_STATUS_MASK =
  FPU_STATUS_IE | FPU_STATUS_DE | FPU_STATUS_ZE |
  FPU_STATUS_OE | FPU_STATUS_UE | FPU_STATUS_PE;

enum FPUTagValue : uint8_t {
  FPU_TAG_VALID = 0,
  FPU_TAG_ZERO = 1,
  FPU_TAG_SPECIAL = 2,
  FPU_TAG_EMPTY = 3,
};

struct DescriptorRegister {
  uint16_t limit;
  uint32_t base;
};

class CPU {
public:
  explicit CPU(CpuModelId model = kDefaultCpuModel);
  void reset();

  // CPU model: identification, model-gated instructions, EFLAGS/CR0/CR4
  // bits and the cycle-cost table.  setModel() keeps the register state
  // (except CR0 bits the new model lacks); reset() applies the model's
  // power-on signature in EDX.
  void setModel(CpuModelId id);
  const CpuModel &model() const { return *m_model; }
  bool hasFeature(uint32_t featureBits) const {
    return (m_features & featureBits) == featureBits;
  }
  // EFLAGS bits that exist on this model (bit 1 excluded).
  uint32_t eflagsMask() const { return m_model->eflagsMask; }

  // Guest clock: approximate core clock cycles executed so far, weighted by
  // the model's per-instruction costs.  Monotonic; never reset.
  uint64_t cycles() const { return m_guestCycles; }
  void addGuestCycles(uint32_t c) { m_guestCycles += c; }
  // Per-opcode cost table (see CpuModel.hpp); decoders keep a copy and
  // refresh it when the segment-state version changes.
  const uint8_t *cycleTable() const { return m_cycleTable.data(); }
  uint32_t memOperandCycles() const { return m_memOperandCycles; }
  uint32_t mulExtraCycles() const { return m_mulExtraCycles; }
  uint32_t divExtraCycles() const { return m_divExtraCycles; }
  // Time-stamp counter (RDTSC, MSR 0x10): cycles() plus a WRMSR offset.
  uint64_t readTSC() const { return m_guestCycles + m_tscOffset; }
  void writeTSC(uint64_t value) { m_tscOffset = value - m_guestCycles; }

  // Register accessors are defined inline: they are on every instruction's
  // hot path.
  uint32_t getReg32(uint8_t index) const { return m_regs[index & 7]; }
  void setReg32(uint8_t index, uint32_t value) { m_regs[index & 7] = value; }
  uint16_t getReg16(uint8_t index) const {
    return static_cast<uint16_t>(m_regs[index & 7]);
  }
  void setReg16(uint8_t index, uint16_t value) {
    uint32_t &reg = m_regs[index & 7];
    reg = (reg & 0xFFFF0000u) | value;
  }
  uint8_t getReg8(uint8_t index) const {
    // 0-3 = AL,CL,DL,BL; 4-7 = AH,CH,DH,BH
    return static_cast<uint8_t>(m_regs[index & 3] >> ((index & 4) ? 8 : 0));
  }
  void setReg8(uint8_t index, uint8_t value) {
    uint32_t &reg = m_regs[index & 3];
    if (index & 4)
      reg = (reg & 0xFFFF00FFu) | (static_cast<uint32_t>(value) << 8);
    else
      reg = (reg & 0xFFFFFF00u) | value;
  }

  uint16_t getSegReg(uint8_t index) const { return m_segRegs[index % 6]; }
  void setSegReg(uint8_t index, uint16_t value) {
    uint8_t seg = index % 6;
    m_segRegs[seg] = value;
    if (!(m_cr[0] & 1) || (m_eflags & 0x00020000)) {
      // Real/V86 mode: the base is simply selector*16, so keep the cached
      // base coherent immediately (HLE services read getSegBase()).
      m_segBase[seg] = static_cast<uint32_t>(value) << 4;
      m_dirtySegmentMask =
          static_cast<uint8_t>(m_dirtySegmentMask & ~(1u << seg));
    } else {
      m_dirtySegmentMask |= static_cast<uint8_t>(1u << seg);
    }
    ++m_segmentStateVersion;
  }

  void loadSegment(SegRegIndex index, uint16_t value);

  uint32_t getSegBase(uint8_t index) const { return m_segBase[index % 6]; }
  void setSegBase(uint8_t index, uint32_t value) {
    uint8_t seg = index % 6;
    uint8_t mask = static_cast<uint8_t>(1u << seg);
    m_segBase[seg] = value;
    m_dirtySegmentMask = static_cast<uint8_t>(m_dirtySegmentMask & ~mask);
    ++m_segmentStateVersion;
  }

  bool is32BitCode() const { return m_is32BitCode; }
  void setIs32BitCode(bool val) {
    m_is32BitCode = val;
    if (!val) {
      m_eip &= 0xFFFF;
    }
  }
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
    uint32_t returnFrameBytes = 0; // Total bytes to discard on HLE return
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
    bool gprsPreSaved = false;     // GPRs were saved at stack-switch time
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
    HLEFrame frame{};
    frame.is32 = is32;
    frame.vector = vector;
    m_hleStack.push_back(frame);
  }
  void popHLEFrame() {
    if (!m_hleStack.empty())
      m_hleStack.pop_back();
  }
  size_t hleStackSize() const { return m_hleStack.size(); }
  const HLEFrame &hleFrameAt(size_t i) const { return m_hleStack[i]; }
  const std::vector<HLEFrame>& getHLEStack() const { return m_hleStack; }
  void setHLEStack(const std::vector<HLEFrame>& stack) { m_hleStack = stack; }

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
    HLEFrame empty{};
    empty.vector = 0xFF;
    return empty;
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
    HLEFrame empty{};
    empty.vector = 0xFF;
    return empty;
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
    HLEFrame empty{};
    empty.vector = 0xFF;
    return empty;
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
          if (!it->gprsPreSaved) {
            it->savedEAX = m_regs[EAX]; it->savedEBX = m_regs[EBX];
            it->savedECX = m_regs[ECX]; it->savedEDX = m_regs[EDX];
            it->savedESI = m_regs[ESI]; it->savedEDI = m_regs[EDI];
            it->savedEBP = m_regs[EBP];
            it->savedFS = m_segRegs[FS]; it->savedGS = m_segRegs[GS];
          }
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

  // Host-side tick counter advanced by the main loop (one per instruction
  // plus idle ticks); see cycles() for the model-weighted guest clock.
  uint64_t getCycles() const { return m_cycles; }
  const uint64_t &getCyclesRef() const { return m_cycles; }
  void addCycles(uint32_t c) { m_cycles += c; }

  uint64_t getSegmentStateVersion() const { return m_segmentStateVersion; }
  uint8_t getDirtySegmentMask() const { return m_dirtySegmentMask; }
  bool hardwareInterruptsEnabled() const {
    return (m_eflags & FLAG_INTERRUPT) != 0 && m_interruptShadow == 0;
  }
  // STI, MOV SS and POP SS hold off interrupts until the next instruction
  // has run: the decoder ends the shadow when it starts that instruction.
  void beginInterruptShadow() { m_interruptShadow = 1; }
  void endInterruptShadow() { m_interruptShadow = 0; }
  bool interruptShadowActive() const { return m_interruptShadow != 0; }

  uint32_t getEIP() const { return m_eip; }
  void setEIP(uint32_t val) { m_eip = m_is32BitCode ? val : (val & 0xFFFF); }

  uint32_t getInstructionStartEIP() const { return m_instructionStartEIP; }
  void setInstructionStartEIP(uint32_t val) { m_instructionStartEIP = val; }
  uint16_t getInstructionStartCS() const { return m_instructionStartCS; }
  void setInstructionStartCS(uint16_t val) { m_instructionStartCS = val; }
  uint16_t getPrevInstructionCS() const { return m_prevInstructionCS; }
  void setPrevInstructionCS(uint16_t val) { m_prevInstructionCS = val; }

  // EFLAGS.  The arithmetic flags (OF SF ZF AF PF CF) of the last ALU
  // instruction are computed lazily: the decoder records the operation and
  // its operands (setLazyFlags) and they are folded into EFLAGS only when
  // something reads them.  getEFLAGS() always returns the architectural
  // value; setEFLAGS() replaces it (and drops the pending record).
  uint32_t getEFLAGS() const {
    if (m_lazy.kind != LazyFlags::None) [[unlikely]]
      materializeFlags();
    return m_eflags;
  }
  void setEFLAGS(uint32_t val) {
    m_lazy.kind = LazyFlags::None;
    m_eflags = val;
  }
  // Bits that are never lazy (IF, DF, TF, IOPL, NT, RF, VM, AC, ...) can be
  // read without folding the arithmetic flags in.
  uint32_t getEFLAGSNonArith() const { return m_eflags & ~LazyFlags::kArithMask; }
  bool directionFlag() const { return (m_eflags & FLAG_DIRECTION) != 0; }

  // Records the result of an ALU operation whose six arithmetic flags are
  // a function of (kind, size, dst, src, res); see LazyFlags.  For Inc/Dec
  // the carry flag is first folded into m_eflags (INC/DEC preserve CF).
  void setLazyFlags(LazyFlags::Kind kind, uint32_t msb, uint32_t dst,
                    uint32_t src, uint32_t res) {
    if (kind == LazyFlags::Inc || kind == LazyFlags::Dec) {
      const uint32_t cf = carryFlag() ? FLAG_CARRY : 0u;
      m_eflags = (m_eflags & ~FLAG_CARRY) | cf;
    }
    m_lazy.kind = kind;
    m_lazy.msb = msb;
    m_lazy.dst = dst;
    m_lazy.src = src;
    m_lazy.res = res;
  }
  bool carryFlag() const {
    if (m_lazy.kind == LazyFlags::None || m_lazy.kind >= LazyFlags::Inc)
      return (m_eflags & FLAG_CARRY) != 0;
    return m_lazy.carry();
  }
  bool zeroFlag() const {
    if (m_lazy.kind == LazyFlags::None)
      return (m_eflags & FLAG_ZERO) != 0;
    return m_lazy.res == 0;
  }
  // Jcc/SETcc/CMOVcc condition (low four bits of the opcode).
  bool testCondition(uint8_t cond) const;
  // Snapshot of EFLAGS including a pending lazy record (instruction restart
  // after #PF restores it without materializing anything).
  struct FlagsState {
    uint32_t eflags;
    LazyFlags lazy;
  };
  FlagsState saveFlagsState() const { return {m_eflags, m_lazy}; }
  void restoreFlagsState(const FlagsState &s) {
    m_eflags = s.eflags;
    m_lazy = s.lazy;
  }

  uint32_t getCR(uint8_t index) const { return m_cr[index & 7]; }
  // True while the guest itself owns protected mode (it set CR0.PE with
  // MOV CR0/LMSW, e.g. a DOS extender in raw or VCPI mode): descriptor
  // privilege levels, call gates, TSS stack switches and exceptions then
  // follow the 386 rules.  Under the emulator's own DPMI host this stays
  // false and the HLE dispatch paths are used.
  bool nativeProtection() const { return m_nativeProtection; }
  void setNativeProtection(bool on) { m_nativeProtection = on; }

  void setCR(uint8_t index, uint32_t val) {
    index &= 7;
    m_cr[index] = val;
    if (index == 0 && !(val & 1))
      m_nativeProtection = false;
    if (m_memory && (index == 0 || index == 3 || index == 4))
      m_memory->setPaging((m_cr[0] & 0x80000001u) == 0x80000001u, m_cr[3],
                          (m_cr[4] & 0x10u) != 0);
  }

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

  // x87 FPU: state and semantics live in x87::FPU (src/cpu/x87).  The
  // accessors below are kept for tests, the debugger and HLE code.
  x87::FPU &fpu() { return m_fpu; }
  const x87::FPU &fpu() const { return m_fpu; }
  void resetFPU() { m_fpu.reset(); }
  uint16_t getFPUControlWord() const { return m_fpu.controlWord(); }
  void setFPUControlWord(uint16_t val) { m_fpu.setControlWord(val); }
  uint16_t getFPUStatusWord() const { return m_fpu.statusWord(); }
  void setFPUStatusWord(uint16_t val) { m_fpu.setStatusWord(val); }
  void setFPUStatusBits(uint16_t mask) { m_fpu.setStatusBits(mask); }
  void clearFPUStatusBits(uint16_t mask) { m_fpu.clearStatusBits(mask); }
  uint16_t getPendingFPUExceptionBits() const { return m_fpu.pendingExceptions(); }
  bool hasPendingFPUException() const { return m_fpu.pendingExceptions() != 0; }
  uint16_t getFPUTagWord() const { return m_fpu.tagWord(); }
  void setFPUTagWord(uint16_t val) { m_fpu.setTagWord(val); }
  uint8_t getFPUTop() const { return m_fpu.top(); }
  void setFPUTop(uint8_t top) { m_fpu.setTop(top); }
  uint32_t getFPUInstructionPointer() const { return m_fpu.fip; }
  void setFPUInstructionPointer(uint32_t val) { m_fpu.fip = val; }
  uint16_t getFPUInstructionSelector() const { return m_fpu.fcs; }
  void setFPUInstructionSelector(uint16_t val) { m_fpu.fcs = val; }
  uint16_t getFPULastOpcode() const { return m_fpu.fop; }
  void setFPULastOpcode(uint16_t val) {
    m_fpu.fop = static_cast<uint16_t>(val & 0x07FF);
  }
  uint32_t getFPUDataPointer() const { return m_fpu.fdp; }
  void setFPUDataPointer(uint32_t val) { m_fpu.fdp = val; }
  uint16_t getFPUDataSelector() const { return m_fpu.fds; }
  void setFPUDataSelector(uint16_t val) { m_fpu.fds = val; }
  bool isFPURegisterEmpty(uint8_t logicalIndex) const {
    return m_fpu.isEmpty(logicalIndex & 7);
  }
  // Double views of the 80-bit registers (rounded to nearest).
  double getFPURegister(uint8_t logicalIndex) const {
    return x87::toDouble(m_fpu.st(logicalIndex & 7));
  }
  void setFPURegister(uint8_t logicalIndex, double value) {
    m_fpu.setSt(logicalIndex & 7, x87::fromDouble(value));
  }
  x87::Float80 getFPURegister80(uint8_t logicalIndex) const {
    return m_fpu.st(logicalIndex & 7);
  }
  void setFPURegister80(uint8_t logicalIndex, const x87::Float80 &value) {
    m_fpu.setSt(logicalIndex & 7, value);
  }
  void freeFPURegister(uint8_t logicalIndex) { m_fpu.freeReg(logicalIndex & 7); }
  bool pushFPU(double value) { return m_fpu.push(x87::fromDouble(value)); }
  bool popFPU() { return m_fpu.pop(); }
  // FERR# output: called once per unmasked x87 error when CR0.NE = 0 (the
  // PC wires it to IRQ13 on the slave PIC).  Keeps cpu/ free of hw/.
  void setFPUErrorCallback(std::function<void()> callback) {
    m_fpuErrorCallback = std::move(callback);
  }
  void signalFPUError() {
    if (m_fpuErrorCallback)
      m_fpuErrorCallback();
  }
  bool fpuErrorSignalled() const { return m_fpu.ferrSignalled(); }
  void setFPUErrorSignalled(bool v) { m_fpu.setFerrSignalled(v); }

  void setMemoryBus(fador::memory::MemoryBus *memory) { m_memory = memory; }

private:
  uint64_t m_cycles{0};
  uint64_t m_guestCycles{0};
  uint32_t m_memOperandCycles{0};
  fador::memory::MemoryBus *m_memory{nullptr};
  std::array<uint32_t, 8> m_regs{};    // EAX, ECX, EDX, EBX, ESP, EBP, ESI, EDI
  std::array<uint16_t, 6> m_segRegs{}; // ES, CS, SS, DS, FS, GS
  std::array<uint32_t, 6> m_segBase{}; // Cached bases for PM/RM
  bool m_is32BitCode{false};           // CS D/B bit status
  bool m_is32BitStack{false};          // SS B bit status
  uint64_t m_segmentStateVersion{1};
  uint8_t m_dirtySegmentMask{0};
  uint8_t m_interruptShadow{0};
  uint32_t m_eip{0};
  uint32_t m_instructionStartEIP{0};
  uint16_t m_instructionStartCS{0xF000};
  uint16_t m_prevInstructionCS{0xF000};
  mutable uint32_t m_eflags{0x00000002}; // bit 1 is always reserved as 1
  mutable LazyFlags m_lazy{};
  void materializeFlags() const {
    m_eflags = (m_eflags & ~m_lazy.definedMask()) | m_lazy.flags(m_eflags);
    m_lazy.kind = LazyFlags::None;
  }

  std::array<uint32_t, 8> m_cr{};
  bool m_nativeProtection{false};
  std::array<uint32_t, 8> m_dr{};

  DescriptorRegister m_gdtr{0xFFFF, 0};
  DescriptorRegister m_idtr{0xFFFF, 0};
  DescriptorRegister m_ldtr{0xFFFF, 0};
  DescriptorRegister m_tr{0xFFFF, 0};
  uint16_t m_ldtrSelector{0};
  uint16_t m_trSelector{0};
  x87::FPU m_fpu;
  std::function<void()> m_fpuErrorCallback;

  std::vector<HLEFrame> m_hleStack;

  // CPU model (kept after the hot register state).
  const CpuModel *m_model{nullptr};
  uint32_t m_features{0};
  uint32_t m_mulExtraCycles{0};
  uint32_t m_divExtraCycles{0};
  uint64_t m_tscOffset{0};
  CycleTable m_cycleTable{};
};

inline bool CPU::testCondition(uint8_t cond) const {
  bool r;
  if (m_lazy.kind == LazyFlags::None) {
    const uint32_t f = m_eflags;
    const bool of = (f & FLAG_OVERFLOW) != 0, sf = (f & FLAG_SIGN) != 0;
    const bool zf = (f & FLAG_ZERO) != 0, cf = (f & FLAG_CARRY) != 0;
    switch ((cond >> 1) & 7) {
    case 0: r = of; break;
    case 1: r = cf; break;
    case 2: r = zf; break;
    case 3: r = cf || zf; break;
    case 4: r = sf; break;
    case 5: r = (f & FLAG_PARITY) != 0; break;
    case 6: r = sf != of; break;
    default: r = zf || (sf != of); break;
    }
  } else {
    switch ((cond >> 1) & 7) {
    case 0: r = m_lazy.overflow(); break;
    case 1: r = carryFlag(); break;
    case 2: r = m_lazy.zero(); break;
    case 3: r = m_lazy.zero() || carryFlag(); break;
    case 4: r = m_lazy.sign(); break;
    case 5: r = m_lazy.parity(); break;
    case 6: r = m_lazy.sign() != m_lazy.overflow(); break;
    default: r = m_lazy.zero() || (m_lazy.sign() != m_lazy.overflow()); break;
    }
  }
  return r != ((cond & 1) != 0);
}

// Stack helpers (inline: used by PUSH/POP/CALL/RET on every call)
inline void CPU::push16(uint16_t value) {
  uint32_t stackBase = m_segBase[SS];
  if (m_is32BitStack) {
    uint32_t esp = getReg32(ESP) - 2;
    setReg32(ESP, esp);
    if (m_memory)
      m_memory->lwrite16(stackBase + esp, value);
  } else {
    uint16_t sp = getReg16(SP) - 2;
    setReg16(SP, sp);
    if (m_memory)
      m_memory->lwrite16(stackBase + sp, value);
  }
}

inline void CPU::push32(uint32_t value) {
  uint32_t stackBase = m_segBase[SS];
  if (m_is32BitStack) {
    uint32_t esp = getReg32(ESP) - 4;
    setReg32(ESP, esp);
    if (m_memory)
      m_memory->lwrite32(stackBase + esp, value);
  } else {
    uint16_t sp = getReg16(SP) - 4;
    setReg16(SP, sp);
    if (m_memory)
      m_memory->lwrite32(stackBase + sp, value);
  }
}

inline uint16_t CPU::pop16() {
  uint16_t value = 0;
  uint32_t stackBase = m_segBase[SS];
  if (m_is32BitStack) {
    uint32_t esp = getReg32(ESP);
    if (m_memory)
      value = m_memory->lread16(stackBase + esp);
    setReg32(ESP, esp + 2);
  } else {
    uint16_t sp = getReg16(SP);
    if (m_memory)
      value = m_memory->lread16(stackBase + sp);
    setReg16(SP, sp + 2);
  }
  return value;
}

inline uint32_t CPU::pop32() {
  uint32_t value = 0;
  uint32_t stackBase = m_segBase[SS];
  if (m_is32BitStack) {
    uint32_t esp = getReg32(ESP);
    if (m_memory)
      value = m_memory->lread32(stackBase + esp);
    setReg32(ESP, esp + 4);
  } else {
    uint16_t sp = getReg16(SP);
    if (m_memory)
      value = m_memory->lread32(stackBase + sp);
    setReg16(SP, sp + 4);
  }
  return value;
}

} // namespace fador::cpu
