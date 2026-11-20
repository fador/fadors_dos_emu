// x87 instruction decoding (opcodes D8..DF and FWAIT): operand fetch,
// memory access, instruction/operand pointer tracking and exception
// delivery.  The arithmetic and stack semantics live in x87::FPU.

#include "InstructionDecoder.hpp"
#include "../utils/Logger.hpp"

#include <array>
#include <cstring>

#if defined(_MSC_VER)
#define FADOR_FPU_COLD __declspec(noinline)
#define FADOR_FPU_INLINE __forceinline
#define FADOR_FPU_HOT
#else
#define FADOR_FPU_COLD __attribute__((noinline, cold))
#define FADOR_FPU_INLINE inline __attribute__((always_inline))
// The hot dispatcher passes small locals to inline asm by address, which
// would otherwise make -fstack-protector-strong instrument every x87
// instruction.
#if defined(__clang__) || (defined(__GNUC__) && __GNUC__ >= 11)
#define FADOR_FPU_HOT __attribute__((no_stack_protector))
#else
#define FADOR_FPU_HOT
#endif
#endif

namespace fador::cpu {

using x87::Arith;
using x87::Const;
using x87::Float80;
using x87::ImageMode;
using x87::MemFmt;

namespace {

constexpr uint32_t kCR0_NE = 0x20;

// Instructions that do not check for pending unmasked exceptions.
constexpr bool nonWaiting(uint8_t opcode, uint8_t modrmByte) {
  if (modrmByte < 0xC0) {
    const uint8_t reg = (modrmByte >> 3) & 7;
    // FNSTENV, FNSTCW, FNSAVE, FNSTSW m16
    return (opcode == 0xD9 || opcode == 0xDD) && (reg == 6 || reg == 7);
  }
  // FNENI, FNDISI, FNCLEX, FNINIT, FNSETPM, FNSTSW AX
  return (opcode == 0xDB && modrmByte >= 0xE0 && modrmByte <= 0xE4) ||
         (opcode == 0xDF && modrmByte == 0xE0);
}

// Control instructions leave FIP/FCS/FOP/FDP/FDS alone.
constexpr bool control(uint8_t opcode, uint8_t modrmByte) {
  if (modrmByte < 0xC0)
    return (opcode == 0xD9 || opcode == 0xDD) && ((modrmByte >> 3) & 7) >= 4;
  return (opcode == 0xDB && modrmByte >= 0xE0 && modrmByte <= 0xE4) ||
         (opcode == 0xDF && modrmByte == 0xE0);
}

// Bit 0 = non-waiting, bit 1 = control, indexed by (opcode & 7) << 8 | ModRM.
constexpr std::array<uint8_t, 2048> kFpuClass = [] {
  std::array<uint8_t, 2048> t{};
  for (int op = 0; op < 8; ++op)
    for (int m = 0; m < 256; ++m)
      t[(op << 8) | m] = static_cast<uint8_t>(
          (nonWaiting(static_cast<uint8_t>(0xD8 + op), static_cast<uint8_t>(m)) ? 1 : 0) |
          (control(static_cast<uint8_t>(0xD8 + op), static_cast<uint8_t>(m)) ? 2 : 0));
  return t;
}();

constexpr Arith kArith[8] = {Arith::Add, Arith::Mul, Arith::Add, Arith::Add,
                             Arith::Sub, Arith::SubR, Arith::Div, Arith::DivR};
// DC/DE register forms: ST(i) is the destination, so the reversed forms swap.
constexpr Arith kArithRev[8] = {Arith::Add, Arith::Mul, Arith::Add, Arith::Add,
                                Arith::SubR, Arith::Sub, Arith::DivR, Arith::Div};

} // namespace

ImageMode InstructionDecoder::fpuImageMode() const {
  const bool realLayout =
      !(m_cpu.getCR(0) & 1) || (m_cpu.getEFLAGS() & 0x00020000u);
  if (currentOperandSize32())
    return realLayout ? ImageMode::Real32 : ImageMode::Prot32;
  return realLayout ? ImageMode::Real16 : ImageMode::Prot16;
}

// Called before a waiting x87 instruction (and FWAIT).  Returns true if the
// instruction must not execute because #MF was raised.
bool InstructionDecoder::fpuCheckPendingError() {
  if (!m_cpu.fpu().errorPending()) {
    m_cpu.setFPUErrorSignalled(false);
    return false;
  }
  if (m_cpu.getCR(0) & kCR0_NE) {
    raiseException(16); // #MF, a fault at the waiting instruction
    return true;
  }
  // PC/AT style: FERR# raises IRQ13 once per error; the instruction then
  // proceeds as if IGNNE# were asserted (the handler sees the exception
  // flags, which stay set until FNCLEX/FNINIT).
  if (!m_cpu.fpuErrorSignalled()) {
    m_cpu.setFPUErrorSignalled(true);
    m_cpu.signalFPUError();
  }
  return false;
}

void InstructionDecoder::fpuWait() { fpuCheckPendingError(); }

void InstructionDecoder::fpuReadBytes(uint32_t address, uint8_t *out, size_t n) {
  size_t i = 0;
  for (; i + 4 <= n; i += 4) {
    const uint32_t v = m_memory.lread32(address + static_cast<uint32_t>(i));
    std::memcpy(out + i, &v, 4);
  }
  for (; i < n; ++i)
    out[i] = m_memory.lread8(address + static_cast<uint32_t>(i));
}

void InstructionDecoder::fpuWriteBytes(uint32_t address, const uint8_t *in, size_t n) {
  size_t i = 0;
  for (; i + 4 <= n; i += 4) {
    uint32_t v;
    std::memcpy(&v, in + i, 4);
    m_memory.lwrite32(address + static_cast<uint32_t>(i), v);
  }
  for (; i < n; ++i)
    m_memory.lwrite8(address + static_cast<uint32_t>(i), in[i]);
}

// 10-byte memory operands: FLD/FSTP m80 (DB /5, /7), FBLD/FBSTP (DF /4, /6).
FADOR_FPU_COLD void InstructionDecoder::fpuTenByte(uint8_t opcode, uint8_t reg,
                                                   uint32_t addr) {
  x87::FPU &fpu = m_cpu.fpu();
  uint8_t b[10];
  if (opcode == 0xDB && reg == 5) {
    fpuReadBytes(addr, b, 10);
    fpu.fldValue(x87::loadFloat80(b));
  } else if (opcode == 0xDB) {
    Float80 v;
    if (fpu.fst80(v, true)) {
      x87::storeFloat80(b, v);
      fpuWriteBytes(addr, b, 10);
    }
  } else if (reg == 4) {
    fpuReadBytes(addr, b, 10);
    fpu.fbld(b);
  } else if (fpu.fbstp(b)) {
    fpuWriteBytes(addr, b, 10);
  }
}

FADOR_FPU_INLINE void InstructionDecoder::executeFPUInstruction(uint8_t opcode,
                                                               uint8_t modrmByte,
                                                               ModRM modrm, uint32_t addr) {
  x87::FPU &fpu = m_cpu.fpu();
  const int i = modrm.rm;
  auto read64 = [&](uint32_t a) {
    return static_cast<uint64_t>(m_memory.lread32(a)) |
           (static_cast<uint64_t>(m_memory.lread32(a + 4)) << 32);
  };
  auto write64 = [&](uint32_t a, uint64_t v) {
    m_memory.lwrite32(a, static_cast<uint32_t>(v));
    m_memory.lwrite32(a + 4, static_cast<uint32_t>(v >> 32));
  };
  auto unimplemented = [&]() {
    LOG_WARN("Unimplemented x87 opcode ", std::hex, static_cast<int>(opcode), " ",
             static_cast<int>(modrmByte));
  };

  switch (opcode) {
  case 0xD8:
  case 0xDC:
  case 0xDA:
  case 0xDE: {
    if (modrm.mod != 3) {
      // D8: m32fp  DA: m32int  DC: m64fp  DE: m16int
      const auto fmt = static_cast<MemFmt>((opcode - 0xD8) >> 1);
      uint64_t raw = 0;
      switch (fmt) {
      case MemFmt::F64:
        raw = read64(addr);
        break;
      case MemFmt::I16:
        raw = m_memory.lread16(addr);
        break;
      default:
        raw = m_memory.lread32(addr);
        break;
      }
      if (modrm.reg == 2 || modrm.reg == 3)
        fpu.fcomMem(fmt, raw, modrm.reg == 3);
      else
        fpu.arithMem(kArith[modrm.reg], fmt, raw);
      return;
    }
    if (opcode == 0xD8) {
      if (modrm.reg == 2 || modrm.reg == 3)
        fpu.fcomReg(i, false, modrm.reg == 3 ? 1 : 0);
      else
        fpu.arithReg(kArith[modrm.reg], 0, i, false);
      return;
    }
    if (opcode == 0xDC) {
      if (modrm.reg == 2 || modrm.reg == 3) // FCOM2 / FCOMP3 aliases
        fpu.fcomReg(i, false, modrm.reg == 3 ? 1 : 0);
      else
        fpu.arithReg(kArithRev[modrm.reg], i, 0, false);
      return;
    }
    if (opcode == 0xDE) {
      if (modrm.reg == 2) { // FCOMP5 alias
        fpu.fcomReg(i, false, 1);
      } else if (modrm.reg == 3) {
        if (modrmByte == 0xD9)
          fpu.fcomReg(1, false, 2); // FCOMPP
        else
          unimplemented();
      } else {
        fpu.arithReg(kArithRev[modrm.reg], i, 0, true);
      }
      return;
    }
    // DA register forms: FCMOVcc and FUCOMPP
    {
      const uint32_t fl = m_cpu.getEFLAGS();
      switch (modrm.reg) {
      case 0:
        fpu.fcmov((fl & FLAG_CARRY) != 0, i);
        return;
      case 1:
        fpu.fcmov((fl & FLAG_ZERO) != 0, i);
        return;
      case 2:
        fpu.fcmov((fl & (FLAG_CARRY | FLAG_ZERO)) != 0, i);
        return;
      case 3:
        fpu.fcmov((fl & FLAG_PARITY) != 0, i);
        return;
      default:
        if (modrmByte == 0xE9)
          fpu.fcomReg(1, true, 2); // FUCOMPP
        else
          unimplemented();
        return;
      }
    }
  }

  case 0xD9: {
    if (modrm.mod != 3) {
      switch (modrm.reg) {
      case 0:
        fpu.fldF32(m_memory.lread32(addr));
        return;
      case 2:
      case 3: {
        uint32_t v = 0;
        if (fpu.fstF32(v, modrm.reg == 3))
          m_memory.lwrite32(addr, v);
        return;
      }
      case 4: // FLDENV
        fpuEnvironment(opcode, modrm.reg, addr);
        return;
      case 5: // FLDCW
        fpu.setControlWord(m_memory.lread16(addr));
        return;
      case 6: // FNSTENV
        fpuEnvironment(opcode, modrm.reg, addr);
        return;
      case 7: // FNSTCW
        m_memory.lwrite16(addr, fpu.controlWord());
        return;
      default:
        unimplemented();
        return;
      }
    }
    switch (modrm.reg) {
    case 0:
      fpu.fldReg(i);
      return;
    case 1:
      fpu.fxch(i);
      return;
    case 2:
      if (modrmByte == 0xD0) // FNOP
        return;
      unimplemented();
      return;
    case 3: // FSTP1 (reserved alias)
      fpu.fstp1(i);
      return;
    default:
      break;
    }
    switch (modrmByte) {
    case 0xE0:
      fpu.fchs();
      return;
    case 0xE1:
      fpu.fabs();
      return;
    case 0xE4:
      fpu.ftst();
      return;
    case 0xE5:
      fpu.fxam();
      return;
    case 0xE8:
    case 0xE9:
    case 0xEA:
    case 0xEB:
    case 0xEC:
    case 0xED:
    case 0xEE:
      fpu.fldConst(static_cast<Const>(modrmByte - 0xE8));
      return;
    case 0xF6:
      fpu.fdecstp();
      return;
    case 0xF7:
      fpu.fincstp();
      return;
    default:
      if (x87::stackOpInputs(modrmByte) != 0) {
        fpu.stackOp(modrmByte);
        return;
      }
      unimplemented();
      return;
    }
  }

  case 0xDB: {
    if (modrm.mod != 3) {
      switch (modrm.reg) {
      case 0:
        fpu.fildValue(static_cast<int32_t>(m_memory.lread32(addr)));
        return;
      case 2:
      case 3: {
        int64_t v = 0;
        if (fpu.fist(32, v, modrm.reg == 3))
          m_memory.lwrite32(addr, static_cast<uint32_t>(v));
        return;
      }
      case 5: // FLD m80
      case 7: // FSTP m80
        fpuTenByte(opcode, modrm.reg, addr);
        return;
      default:
        unimplemented();
        return;
      }
    }
    const uint32_t fl = m_cpu.getEFLAGS();
    switch (modrm.reg) {
    case 0:
      fpu.fcmov((fl & FLAG_CARRY) == 0, i);
      return;
    case 1:
      fpu.fcmov((fl & FLAG_ZERO) == 0, i);
      return;
    case 2:
      fpu.fcmov((fl & (FLAG_CARRY | FLAG_ZERO)) == 0, i);
      return;
    case 3:
      fpu.fcmov((fl & FLAG_PARITY) == 0, i);
      return;
    case 5:
      m_cpu.setEFLAGS(fpu.fcomi(i, true, false, fl)); // FUCOMI
      return;
    case 6:
      m_cpu.setEFLAGS(fpu.fcomi(i, false, false, fl)); // FCOMI
      return;
    default:
      break;
    }
    switch (modrmByte) {
    case 0xE0: // FNENI   (8087 only; a no-op on 287 and later)
    case 0xE1: // FNDISI  (8087 only)
    case 0xE4: // FNSETPM (287 only)
      return;
    case 0xE2:
      fpu.fnclex();
      return;
    case 0xE3:
      fpu.reset();
      return;
    default:
      unimplemented();
      return;
    }
  }

  case 0xDD: {
    if (modrm.mod != 3) {
      switch (modrm.reg) {
      case 0:
        fpu.fldF64(read64(addr));
        return;
      case 2:
      case 3: {
        uint64_t v = 0;
        if (fpu.fstF64(v, modrm.reg == 3))
          write64(addr, v);
        return;
      }
      case 4: // FRSTOR
      case 6: // FNSAVE
        fpuEnvironment(opcode, modrm.reg, addr);
        return;
      case 7: // FNSTSW m16
        m_memory.lwrite16(addr, fpu.statusWord());
        return;
      default:
        unimplemented();
        return;
      }
    }
    switch (modrm.reg) {
    case 0:
      fpu.ffree(i);
      return;
    case 1: // FXCH4 alias
      fpu.fxch(i);
      return;
    case 2:
      fpu.fstReg(i, false);
      return;
    case 3:
      fpu.fstReg(i, true);
      return;
    case 4:
      fpu.fcomReg(i, true, 0); // FUCOM
      return;
    case 5:
      fpu.fcomReg(i, true, 1); // FUCOMP
      return;
    default:
      unimplemented();
      return;
    }
  }

  case 0xDF: {
    if (modrm.mod != 3) {
      switch (modrm.reg) {
      case 0:
        fpu.fildValue(static_cast<int16_t>(m_memory.lread16(addr)));
        return;
      case 2:
      case 3: {
        int64_t v = 0;
        if (fpu.fist(16, v, modrm.reg == 3))
          m_memory.lwrite16(addr, static_cast<uint16_t>(v));
        return;
      }
      case 4: // FBLD
        fpuTenByte(opcode, modrm.reg, addr);
        return;
      case 5:
        fpu.fildValue(static_cast<int64_t>(read64(addr)));
        return;
      case 6: // FBSTP
        fpuTenByte(opcode, modrm.reg, addr);
        return;
      case 7: {
        int64_t v = 0;
        if (fpu.fist(64, v, true))
          write64(addr, static_cast<uint64_t>(v));
        return;
      }
      default:
        unimplemented();
        return;
      }
    }
    switch (modrm.reg) {
    case 0: // FFREEP (undocumented): free ST(i), then pop
      fpu.ffreep(i);
      return;
    case 1: // FXCH7 alias
      fpu.fxch(i);
      return;
    case 2: // FSTP8 (reserved alias)
    case 3: // FSTP9 (reserved alias)
      fpu.fstReg(i, true);
      return;
    case 4:
      if (modrmByte == 0xE0) { // FNSTSW AX
        m_cpu.setReg16(AX, fpu.statusWord());
        return;
      }
      unimplemented();
      return;
    case 5:
      m_cpu.setEFLAGS(fpu.fcomi(i, true, true, m_cpu.getEFLAGS())); // FUCOMIP
      return;
    case 6:
      m_cpu.setEFLAGS(fpu.fcomi(i, false, true, m_cpu.getEFLAGS())); // FCOMIP
      return;
    default:
      unimplemented();
      return;
    }
  }
  default:
    LOG_WARN("Unhandled x87 opcode 0x", std::hex, static_cast<int>(opcode));
    return;
  }
}

FADOR_FPU_HOT void InstructionDecoder::executeFPUOpcode(uint8_t opcode) {
  const uint8_t modrmByte = fetch8();
  const ModRM modrm{static_cast<uint8_t>(modrmByte >> 6),
                    static_cast<uint8_t>((modrmByte >> 3) & 7),
                    static_cast<uint8_t>(modrmByte & 7)};
  x87::FPU &fpu = m_cpu.fpu();
  const uint8_t cls = kFpuClass[((opcode & 7u) << 8) | modrmByte];

  if (fpu.errorPending() | m_cpu.fpuErrorSignalled()) [[unlikely]] {
    if (!fpu.errorPending())
      m_cpu.setFPUErrorSignalled(false); // FERR# re-arms once the error is cleared
    else if (!(cls & 1) && fpuCheckPendingError())
      return;
  }

  const bool hasMem = modrm.mod != 3;
  uint32_t addr = 0;
  if (hasMem)
    addr = m_hasPrefix67 ? getEffectiveAddress32(modrm) : getEffectiveAddress16(modrm);

  if (!(cls & 2)) [[likely]] {
    fpu.fip = m_instrStartEIP;
    fpu.fcs = m_cpu.getSegReg(CS);
    fpu.fop = static_cast<uint16_t>(((opcode & 0x07u) << 8) | modrmByte);
    if (hasMem) {
      fpu.fdp = m_currentOffset;
      fpu.fds = m_cpu.getSegReg(static_cast<SegRegIndex>(m_currentEASegment));
    }
  }

  if (hasMem && m_memory.pagingEnabled()) [[unlikely]] {
    executeFPUPaged(opcode, modrmByte, modrm, addr);
    return;
  }
  executeFPUInstruction(opcode, modrmByte, modrm, addr);
}

// With paging, a memory access to a not-present page is undone and
// restarted after #PF: keep the FPU state so it can be rolled back too.
FADOR_FPU_COLD void InstructionDecoder::executeFPUPaged(uint8_t opcode, uint8_t modrmByte,
                                         ModRM modrm, uint32_t addr) {
  x87::FPU &fpu = m_cpu.fpu();
  const x87::FPU saved = fpu;
  executeFPUInstruction(opcode, modrmByte, modrm, addr);
  if (m_memory.pageFaultPending())
    fpu = saved;
}

FADOR_FPU_COLD void InstructionDecoder::fpuEnvironment(uint8_t opcode, uint8_t reg, uint32_t addr) {
  x87::FPU &fpu = m_cpu.fpu();
  std::array<uint8_t, 108> image{};
  const ImageMode mode = fpuImageMode();
  const size_t envSize = x87::envImageSize(mode);
  if (opcode == 0xD9 && reg == 4) { // FLDENV
    fpuReadBytes(addr, image.data(), envSize);
    fpu.loadEnv(image.data(), mode);
  } else if (opcode == 0xD9) { // FNSTENV: store, then mask all exceptions
    fpu.storeEnv(image.data(), mode);
    fpuWriteBytes(addr, image.data(), envSize);
    fpu.setControlWord(static_cast<uint16_t>(fpu.controlWord() | x87::SW_EXC));
  } else if (reg == 4) { // FRSTOR
    fpuReadBytes(addr, image.data(), envSize + 80);
    fpu.loadEnv(image.data(), mode);
    fpu.loadRegs(image.data() + envSize);
  } else { // FNSAVE: store, then FNINIT
    fpu.storeEnv(image.data(), mode);
    fpu.storeRegs(image.data() + envSize);
    fpuWriteBytes(addr, image.data(), envSize + 80);
    fpu.reset();
  }
}

} // namespace fador::cpu
