#pragma once

#include "../memory/MemoryBus.hpp"
#include "CPU.hpp"
#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <tuple>

namespace fador::hw {
class IOBus;
class BIOS;
class DOS;
} // namespace fador::hw

namespace fador::cpu {

struct ModRM {
  uint8_t mod;
  uint8_t reg;
  uint8_t rm;
};

struct SIB {
  uint8_t scale;
  uint8_t index;
  uint8_t base;
};

class InstructionDecoder;

// One predecoded instruction of the fast path (see FastPath.cpp).  Entries
// live in a two-way set-associative cache indexed by the linear address of
// the first byte.  An entry is used only while the bytes in memory still
// equal `bytes` (checked before every execution, so self-modifying code,
// code loaded over old code and remapped pages need no invalidation) and
// the code segment's default size matches `code32`.  A null handler marks
// an instruction the legacy decoder executes.
struct DecodedInsn {
  using Handler = void (*)(InstructionDecoder &, const DecodedInsn &);
  static constexpr uint32_t kNoEntry = 0xFFFFFFFFu;
  static constexpr uint8_t kX87 = 1;

  uint32_t linear = kNoEntry; // tag: CS base + EIP of the first byte
  uint8_t code32 = 0;         // CS.D the entry was decoded for
  uint8_t len = 0;            // bytes, prefixes included (1..16)
  uint8_t reg = 0;            // ModRM.reg, or the register in the opcode
  uint8_t rm = 0;             // ModRM.rm (register operand)
  Handler handler = nullptr;
  uint64_t bytes[2] = {0, 0}; // instruction bytes, zero-padded
  uint8_t flags = 0;          // kX87: run only without a pending x87 error
  uint8_t spare[3] = {0, 0, 0};
  uint32_t disp = 0;          // memory operand displacement / branch offset
  uint32_t imm = 0;           // immediate operand
  uint32_t baseMask = 0;      // memory operand: offset = disp
  uint32_t indexMask = 0;     //   + (reg[base] & baseMask)
  uint32_t addrMask = 0;      //   + ((reg[index] & indexMask) << scale)
  uint8_t base = 0;           //   then & addrMask
  uint8_t index = 0;
  uint8_t scale = 0;
  uint8_t seg = 0;            // segment of the memory operand
  uint16_t cycles = 0;        // guest cycles charged (opcode + operand)
  uint8_t op = 0;             // opcode (second byte after 0F)
  uint8_t aux = 0;            // handler-specific (condition, group op)

  // 16-bit code: an instruction whose bytes run past IP FFFF is left to
  // the legacy decoder.
  bool wraps(uint32_t eip) const { return !code32 && eip + len > 0x10000u; }
  // True when the 16 bytes at `p` start with this instruction's bytes.
  bool matches(const uint8_t *p) const {
    struct Masks {
      uint64_t m[17][2];
      constexpr Masks() : m() {
        for (unsigned len = 0; len <= 16; ++len)
          for (unsigned i = 0; i < len; ++i)
            m[len][i / 8] |= 0xFFull << (8 * (i % 8));
      }
    };
    static constexpr Masks kMasks{};
    uint64_t w0, w1;
    std::memcpy(&w0, p, 8);
    std::memcpy(&w1, p + 8, 8);
    return (((w0 ^ bytes[0]) & kMasks.m[len][0]) |
            ((w1 ^ bytes[1]) & kMasks.m[len][1])) == 0;
  }
};

static_assert(sizeof(DecodedInsn) <= 64, "keep a decoded instruction in one cache line");

// Provides helper functions for fetching CPU instructions and processing
// prefixes
class InstructionDecoder {
public:
  InstructionDecoder(CPU &cpu, memory::MemoryBus &memory, hw::IOBus &iobus,
                     hw::BIOS &bios, hw::DOS &dos);

  // Main execution loop single step
  void step();

  void syncSegments();

  // Inject a hardware interrupt (always pushes FLAGS/CS/IP, unlike
  // triggerInterrupt which may take the HLE fast-path).
  void injectHardwareInterrupt(uint8_t vector);

  // Calculates effective address based on ModRM and SIB (public for testing)
  uint32_t getEffectiveAddress16(const ModRM &modrm);
  uint32_t getEffectiveAddress32(const ModRM &modrm);

  // Protected Mode segment loading (public for DPMI segment cache reload)
  void loadSegment(SegRegIndex seg, uint16_t selector);

  // Runs up to `maxInstructions` instructions (each one as step() followed
  // by CPU::addCycles(1)), stopping early after the instruction that makes
  // the DOS program terminate.  Returns the number executed.
  uint64_t run(uint64_t maxInstructions);

  // The predecoded fast path (on by default).  Turning it off makes every
  // instruction go through the legacy decoder; both must give identical
  // results (tests/test_interpreter_fast_paths.cpp, tools/cpu_fuzz).
  void setFastPathEnabled(bool on) { m_fastPathEnabled = on; }
  bool fastPathEnabled() const { return m_fastPathEnabled; }
  // Drops every predecoded instruction (tests; never needed for
  // correctness since entries are validated against memory).
  void flushDecodedCache();

private:
  CPU &m_cpu;
  memory::MemoryBus &m_memory;
  hw::IOBus &m_iobus;
  hw::BIOS &m_bios;
  hw::DOS &m_dos;

  friend struct FastOps;
  // Two-way set-associative: set = low bits of the linear address, so
  // sequential code uses consecutive sets.
  static constexpr uint32_t kDecodedSets = 8192; // power of two
  std::unique_ptr<DecodedInsn[]> m_decoded =
      std::make_unique<DecodedInsn[]>(kDecodedSets * 2);
  DecodedInsn *findDecoded(uint32_t lin, uint8_t code32) {
    DecodedInsn *set = &m_decoded[(lin & (kDecodedSets - 1)) * 2];
    if (set[0].linear == lin && set[0].code32 == code32) [[likely]]
      return set;
    if (set[1].linear == lin && set[1].code32 == code32)
      return set + 1;
    return nullptr;
  }
  // Slot for a new entry: the set's older entry is evicted.
  DecodedInsn &newDecoded(uint32_t lin) {
    DecodedInsn *set = &m_decoded[(lin & (kDecodedSets - 1)) * 2];
    set[1] = set[0];
    return set[0];
  }
  bool m_fastPathEnabled = true;
  bool tryFastPath(uint32_t startEIP);
  uint64_t runFastBurst(uint64_t maxInstructions);
  void decodeFast(DecodedInsn &d, uint32_t linear, const uint8_t *code,
                  bool code32);

  // Prefixes state for current instruction
  uint32_t m_instrStartEIP;
  uint32_t m_faultCount = 0; // raiseException() calls (REP loops stop)
  uint32_t m_interruptCount = 0; // interrupts/exceptions delivered
  bool m_rfLoaded = false;        // the instruction loaded EFLAGS.RF = 1
  bool m_hasPrefix66; // Operand size override
  bool m_hasPrefix67; // Address size override
  bool m_hasRepnz;
  bool m_hasRepz;
  uint8_t m_segmentOverride; // SegRegIndex or 0xFF for none
  uint32_t m_segBase[6];     // Cached bases for ES, CS, SS, DS, FS, GS
  uint64_t m_cachedSegStateVersion;

  // App-level PM interrupt vectors captured from INT 31h/0205h calls
  // that are intercepted by DOS extender thunks before reaching the DPMI host.
  // When a DOS extender (e.g. DOS/4GW) hooks INT 31h and intercepts 0205h
  // calls from the application, the DPMI host never sees the app's handler.
  // We capture the original CX:EDX at the point of the INT 31h instruction.
  struct AppPMVector {
    uint16_t selector = 0;
    uint32_t offset = 0;
    bool valid = false;
  };
  std::array<AppPMVector, 256> m_appPMVectors{};

  // Caching for EA to avoid double-fetching displacement in RMW instructions
  uint32_t m_currentEA;
  uint32_t m_currentOffset; // Holds isolated offset (critical for 0x8D LEA)
  uint8_t m_currentEASegment;
  bool m_eaResolved;

  // Copy of the CPU model's cycle-cost table (see CpuModel.hpp), refreshed
  // by syncSegments() when CPU::setModel() changes the model.
  CycleTable m_cycleTable{};
  const CpuModel *m_cycleTableModel{nullptr};

  // Fetches next byte(s) from memory through CS:EIP (inline: hot path)
  uint8_t fetch8() {
    const uint32_t eip = m_cpu.getEIP();
    const uint8_t val = m_memory.lfetch8(m_segBase[CS] + eip);
    m_cpu.setEIP(eip + 1);
    return val;
  }
  uint16_t fetch16() {
    const uint32_t eip = m_cpu.getEIP();
    const uint16_t val = m_memory.lfetch16(m_segBase[CS] + eip);
    m_cpu.setEIP(eip + 2);
    return val;
  }
  uint32_t fetch32() {
    const uint32_t eip = m_cpu.getEIP();
    const uint32_t val = m_memory.lfetch32(m_segBase[CS] + eip);
    m_cpu.setEIP(eip + 4);
    return val;
  }
  void syncSegmentCacheIfNeeded();
  bool tryFastRepMovs(uint32_t elementSize);
  bool tryFastRepStos(uint32_t elementSize);
  bool tryFastRepString(uint8_t opcode);
  bool currentOperandSize32() const;
  bool dos4gwReflectionStack(uint32_t &hostSP) const;

public:
  ModRM decodeModRM(uint8_t byte);
private:
  SIB decodeSIB(uint8_t byte);

  // Helpers to resolve operand values
  uint32_t readModRM32(const ModRM &modrm);
  uint16_t readModRM16(const ModRM &modrm);
  uint8_t readModRM8(const ModRM &modrm);

  void writeModRM32(const ModRM &modrm, uint32_t value);
  void writeModRM16(const ModRM &modrm, uint16_t value);
  void writeModRM8(const ModRM &modrm, uint8_t value);
  // x87 (InstructionDecoderFPU.cpp)
  void executeFPUOpcode(uint8_t opcode);
  void executeFPUInstruction(uint8_t opcode, uint8_t modrmByte, ModRM modrm,
                             uint32_t addr);
  void executeFPUPaged(uint8_t opcode, uint8_t modrmByte, ModRM modrm,
                       uint32_t addr);
  void executeFPUOperands(uint8_t opcode, uint8_t modrmByte, bool hasMem,
                          uint32_t addr);
  void fpuEnvironment(uint8_t opcode, uint8_t reg, uint32_t addr);
  void fpuTenByte(uint8_t opcode, uint8_t reg, uint32_t addr);
  bool fpuCheckPendingError();
  void fpuWait();
  x87::ImageMode fpuImageMode() const;
  void fpuReadBytes(uint32_t address, uint8_t *out, size_t n);
  void fpuWriteBytes(uint32_t address, const uint8_t *in, size_t n);

  // General ALU operation evaluator covering Math and Logic EFLAGS
  uint32_t aluOp(uint8_t op, uint32_t dest, uint32_t src, int size);

  void setMulOverflow(bool overflow);

  // INC/DEC helpers that preserve CF (per x86 spec)
  uint32_t incOp(uint32_t val, int size);
  uint32_t decOp(uint32_t val, int size);

  // Shift/rotate group: op = ModRM.reg (ROL ROR RCL RCR SHL SHR SAL SAR)
  uint32_t shiftRotate(uint8_t op, uint32_t val, uint8_t c, int size);

  // Checks branch/set conditions
  bool checkCondition(uint8_t cond);

  // Instruction Handlers
  void executeOpcode(uint8_t opcode);
  void executeOpcode0F(uint8_t opcode); // Two-byte opcodes

  // Interrupt and Exception handling
  void triggerInterrupt(uint8_t vector);

  // 386 protection model for guest-owned protected mode (Protection.cpp)
  uint8_t currentCPL() const;
  bool readDescriptor(uint16_t selector, uint32_t &low, uint32_t &high) const;
  bool tssStack(uint8_t dpl, uint16_t &ss, uint32_t &esp) const;
  void setStackPointer(uint32_t value);
  void pushSized(bool is32, uint32_t value);
  uint32_t popSized(bool is32);
  // Faults: EIP is rewound to the faulting instruction and, in native
  // protected mode, the error code is pushed where the CPU would.
  void raiseException(uint8_t vector, uint32_t errorCode = 0);
  void nativeInterrupt(uint8_t vector, bool software, bool pushError,
                       uint32_t errorCode);
  void nativeIret(bool op32);
  void nativeFarTransfer(uint16_t selector, uint32_t offset, bool isCall,
                         bool op32);
  void nativeRetf(uint16_t release, bool op32);
  void stepInstruction();
  // step() without the debug checks: one instruction, undone and restarted
  // after #PF when paging is on.
  void stepChecked();
  // Debug registers and single-stepping (Protection.cpp): instruction
  // breakpoints (faults, suppressed by RF), data breakpoints and TF
  // single-step (traps after the instruction).  Only used while
  // CPU::debugStepNeeded().
  void stepDebug();
  void deliverDebugException(uint32_t dr6Bits);
  // MOV DRn privilege/alias/general-detect checks; maps DR4/DR5 to
  // DR6/DR7.  Returns false when an exception was raised.
  bool debugRegisterAccess(uint8_t &dr);
  uint32_t readDebugRegister(uint8_t dr) const;
  void writeDebugRegister(uint8_t dr, uint32_t value);
  // LLDT / LTR under guest-owned protection (type, presence, busy bit).
  void nativeLoadLDT(uint16_t selector);
  void nativeLoadTR(uint16_t selector);

  // CPU model rules (Protection.cpp).
  // Raises #UD and returns false when the model lacks the feature bits.
  bool requireFeature(uint32_t featureBits);
  // EFLAGS bits that POPF (iret=false) or IRET (iret=true) may change at
  // the current privilege level, before operand-size masking.
  uint32_t flagsWriteMask(bool iret) const;
  // Merges `value` into EFLAGS under `mask` (low word only for 16-bit
  // operands), keeping bit 1 set and nonexistent bits clear.
  void writeFlags(uint32_t value, uint32_t mask, bool op32);
  // IOPL-sensitive instructions (CLI/STI/PUSHF/POPF in V86) under guest-owned
  // protection: raises #GP(0) and returns false when CPL > IOPL.
  bool checkIOPL();
  // Privileged instructions (LGDT, LIDT, LLDT, LTR, LMSW, CLTS, MOV
  // CRn/DRn, INVD, WBINVD, INVLPG, HLT, RDMSR/WRMSR) under guest-owned
  // protection: #GP(0) and false at CPL > 0 (V86 mode is CPL 3).
  bool requireCPL0();
  // IN/OUT/INS/OUTS: with CPL > IOPL or in V86 mode the TSS I/O permission
  // bitmap must clear every bit of ports port..port+size-1; otherwise
  // #GP(0) and false.
  bool checkIOPermission(uint16_t port, uint32_t size);
  bool ioBitmapAllows(uint16_t port, uint32_t size) const;
  void executeCPUID();
  bool readMSR(uint32_t index, uint64_t &value) const;
  bool writeMSR(uint32_t index, uint64_t value);

  struct Descriptor {
    uint32_t base;
    uint32_t limit;
    bool is32Bit;
    bool isPresent;
    uint8_t dpl;
    uint8_t type;
    bool isSystem;
  };

  Descriptor decodeDescriptor(uint32_t low, uint32_t high);
};

inline bool InstructionDecoder::tryFastPath(uint32_t startEIP) {
  // Off while CPU debug logging is on, so --trace=cpu output is complete.
  if (!m_fastPathEnabled || (utils::enabledCategories & utils::CAT_CPU))
    [[unlikely]]
    return false;
  const uint32_t lin = m_segBase[CS] + startEIP;
  const uint8_t code32 = m_cpu.is32BitCode() ? 1 : 0;
  DecodedInsn *e = findDecoded(lin, code32);
  const uint8_t *p;
  if (e != nullptr) [[likely]] {
    if (e->handler == nullptr)
      return false;
    p = m_memory.codeWindow(lin);
    if (p == nullptr) [[unlikely]]
      return false;
    if (!e->matches(p)) [[unlikely]] {
      decodeFast(*e, lin, p, code32 != 0);
      if (e->handler == nullptr)
        return false;
    }
  } else {
    p = m_memory.codeWindow(lin);
    if (p == nullptr)
      return false;
    e = &newDecoded(lin);
    decodeFast(*e, lin, p, code32 != 0);
    if (e->handler == nullptr)
      return false;
  }
  const DecodedInsn &d = *e;
  if (d.wraps(startEIP) ||
      ((d.flags & DecodedInsn::kX87) &&
       (m_cpu.fpu().errorPending() || m_cpu.x87Gated())))
    [[unlikely]]
    return false;
  m_cpu.setEIP(startEIP + d.len);
  m_cpu.addGuestCycles(d.cycles);
  d.handler(*this, d);
  return true;
}

// ALU helpers are inline: the predecoded fast path (FastPath.cpp) and the
// legacy decoder share them.
inline uint32_t InstructionDecoder::aluOp(uint8_t op, uint32_t dest, uint32_t src,
                                   int size) {
  // The six arithmetic flags are recorded lazily (see LazyFlags.hpp).
  const uint32_t msb = 1u << (size - 1);
  const uint32_t mask = (msb << 1) - 1u;
  const uint32_t d = dest & mask;
  const uint32_t s = src & mask;
  uint32_t r;
  LazyFlags::Kind kind;
  switch (op & 7) {
  case 0: // ADD
    r = d + s;
    kind = LazyFlags::Add;
    break;
  case 2: // ADC
    if (m_cpu.carryFlag()) {
      r = d + s + 1;
      kind = LazyFlags::AddC;
    } else {
      r = d + s;
      kind = LazyFlags::Add;
    }
    break;
  case 3: // SBB
    if (m_cpu.carryFlag()) {
      r = d - s - 1;
      kind = LazyFlags::SubB;
    } else {
      r = d - s;
      kind = LazyFlags::Sub;
    }
    break;
  case 5: // SUB
  case 7: // CMP
    r = d - s;
    kind = LazyFlags::Sub;
    break;
  case 1: // OR
    r = d | s;
    kind = LazyFlags::Logic;
    break;
  case 4: // AND
    r = d & s;
    kind = LazyFlags::Logic;
    break;
  default: // 6: XOR
    r = d ^ s;
    kind = LazyFlags::Logic;
    break;
  }
  r &= mask;
  m_cpu.setLazyFlags(kind, msb, d, s, r);
  return r;
}

// INC/DEC preserve CF (setLazyFlags folds the current CF into EFLAGS).
inline uint32_t InstructionDecoder::incOp(uint32_t val, int size) {
  const uint32_t msb = 1u << (size - 1);
  const uint32_t mask = (msb << 1) - 1u;
  const uint32_t d = val & mask;
  const uint32_t r = (d + 1) & mask;
  m_cpu.setLazyFlags(LazyFlags::Inc, msb, d, 1, r);
  return r;
}

inline uint32_t InstructionDecoder::decOp(uint32_t val, int size) {
  const uint32_t msb = 1u << (size - 1);
  const uint32_t mask = (msb << 1) - 1u;
  const uint32_t d = val & mask;
  const uint32_t r = (d - 1) & mask;
  m_cpu.setLazyFlags(LazyFlags::Dec, msb, d, 1, r);
  return r;
}

} // namespace fador::cpu