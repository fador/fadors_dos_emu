#pragma once

// x87 FPU model: architectural state (80-bit register stack, control,
// status and tag words, last instruction/operand pointers) and the
// instruction-level semantics that are independent of instruction decoding
// and memory access (stack faults, masked/unmasked exception responses,
// condition codes, environment/state images).  Values are computed by the
// back end selected in X87Math.hpp.
//
// The decoder (InstructionDecoderFPU.cpp) reads/writes guest memory and
// calls into this class; exception delivery (#MF / FERR# -> IRQ13) is driven
// by the decoder from errorPending().

#include "Float80.hpp"
#include "HostX87Inline.hpp"
#include "X87Math.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace fador::cpu::x87 {

// Layout of FSTENV/FLDENV (and the first part of FSAVE/FRSTOR) images.
enum class ImageMode : uint8_t { Real16, Prot16, Real32, Prot32 };

constexpr size_t envImageSize(ImageMode m) {
  return (m == ImageMode::Real32 || m == ImageMode::Prot32) ? 28 : 14;
}
constexpr size_t stateImageSize(ImageMode m) { return envImageSize(m) + 80; }

class FPU {
public:
  static constexpr uint16_t kDefaultControlWord = 0x037F;

  FPU() { reset(); }

  // FNINIT
  void reset();

  // ---- control / status / tag words -------------------------------------
  uint16_t controlWord() const { return m_cw; }
  void setControlWord(uint16_t cw) {
    m_cw = cw;
    refreshSummary();
  }
  // Status word including TOP.
  uint16_t statusWord() const {
    return static_cast<uint16_t>((m_sw & ~SW_TOP) | (m_top << 11));
  }
  void setStatusWord(uint16_t sw) {
    m_top = static_cast<uint8_t>((sw >> 11) & 7);
    m_sw = static_cast<uint16_t>(sw & ~SW_TOP);
    refreshSummary();
  }
  void setStatusBits(uint16_t bits) {
    m_sw = static_cast<uint16_t>(m_sw | (bits & ~SW_TOP));
    refreshSummary();
  }
  void clearStatusBits(uint16_t bits) {
    m_sw = static_cast<uint16_t>(m_sw & ~(bits & ~SW_TOP));
    refreshSummary();
  }
  // Tag word computed from the register contents (as FSTENV stores it).
  uint16_t tagWord() const;
  // Only empty (11) / non-empty matters, as on the 387 and later.
  void setTagWord(uint16_t tw);

  uint8_t top() const { return m_top; }
  void setTop(uint8_t top) { m_top = static_cast<uint8_t>(top & 7); }

  // Unmasked exception flags (the ES summary bit).
  uint16_t pendingExceptions() const {
    return static_cast<uint16_t>(m_sw & ~m_cw & SW_EXC);
  }
  bool errorPending() const { return (m_sw & SW_ES) != 0; }

  // ---- registers (logical index i = ST(i)) --------------------------------
  bool isEmpty(int i) const { return (m_emptyMask >> phys(i)) & 1; }
  Float80 st(int i) const { return loadFields(m_regs[phys(i)]); }
  void setSt(int i, const Float80 &v) {
    const int p = phys(i);
    storeWhole(m_regs[p], v);
    m_emptyMask = static_cast<uint8_t>(m_emptyMask & ~(1u << p));
  }
  void freeReg(int i) { m_emptyMask = static_cast<uint8_t>(m_emptyMask | (1u << phys(i))); }
  Float80 physicalReg(int p) const { return loadFields(m_regs[p & 7]); }
  bool isPhysicalEmpty(int p) const { return (m_emptyMask >> (p & 7)) & 1; }

  // Raw push/pop with stack-fault detection (no masked responses); used by
  // tests and HLE code.  Return false on overflow/underflow.
  bool push(const Float80 &v);
  bool pop();

  // ---- last instruction / operand pointers ------------------------------
  uint32_t fip = 0;
  uint16_t fcs = 0;
  uint16_t fop = 0;
  uint32_t fdp = 0;
  uint16_t fds = 0;

  // ---- instruction semantics ----------------------------------------------
  // Loads (push).  The m32/m64 forms raise IE (SNaN) / DE (denormal).
  void fldValue(const Float80 &v); // FLD m80, constants, exact values
  void fldReg(int i) {
    if (!isEmpty(i) && !pushBlocked()) [[likely]] {
      const Float80 v = st(i);
      pushRaw(v);
      m_sw = static_cast<uint16_t>(m_sw & ~SW_C1);
      return;
    }
    fldRegSlow(i);
  }
  void fldF32(uint32_t raw) {
    const uint32_t e = (raw >> 23) & 0xFF;
    if (e != 0 && e != 0xFF && !pushBlocked()) [[likely]] { // exact, no exceptions
      pushRaw(Float80::make((raw >> 31) != 0, static_cast<uint16_t>(e - 127 + 16383),
                            kIntegerBit | (static_cast<uint64_t>(raw & 0x7FFFFF) << 40)));
      m_sw = static_cast<uint16_t>(m_sw & ~SW_C1);
      return;
    }
    fldF32Slow(raw);
  }
  void fldF64(uint64_t raw) {
    const uint32_t e = static_cast<uint32_t>((raw >> 52) & 0x7FF);
    if (e != 0 && e != 0x7FF && !pushBlocked()) [[likely]] {
      pushRaw(Float80::make((raw >> 63) != 0, static_cast<uint16_t>(e - 1023 + 16383),
                            kIntegerBit | ((raw & 0xFFFFFFFFFFFFFull) << 11)));
      m_sw = static_cast<uint16_t>(m_sw & ~SW_C1);
      return;
    }
    fldF64Slow(raw);
  }
  void fildValue(int64_t v);
  void fbld(const uint8_t *bcd10);
  void fldConst(Const c) {
    if (!pushBlocked()) [[likely]] {
      if (c == Const::Zero || c == Const::One) {
        pushRaw(c == Const::Zero ? kPosZero : kOne);
        m_sw = static_cast<uint16_t>(m_sw & ~SW_C1);
        return;
      }
    }
    fldConstSlow(c);
  }

  // Stores.  Return true if the memory operand must be written.
  bool fstF32(uint32_t &out, bool pop);
  bool fstF64(uint64_t &out, bool pop) {
    if (!isEmpty(0)) [[likely]] {
      const Float80 &v = m_regs[phys(0)];
      const uint64_t mant = v.mant;
      const uint16_t se = v.signExp;
      const uint16_t e = static_cast<uint16_t>(se & 0x7FFF);
      // Normal result well inside the double range: round the significand
      // to 53 bits with RC here (cheaper than any x87 round trip).
      if (e > 16383 - 1022 && e < 16383 + 1023 && (mant >> 63)) [[likely]] {
        uint64_t keep = mant >> 11;
        uint32_t exp = e;
        const uint32_t drop = static_cast<uint32_t>(mant & 0x7FF);
        bool up = false;
        if (drop) {
          switch ((m_cw >> CW_RC_SHIFT) & 3) {
          case 0:
            up = drop > 0x400 || (drop == 0x400 && (keep & 1));
            break;
          case 1:
            up = (se >> 15) != 0;
            break;
          case 2:
            up = (se >> 15) == 0;
            break;
          default:
            break;
          }
          if (up && (++keep >> 53)) {
            keep >>= 1;
            ++exp;
          }
        }
        out = (static_cast<uint64_t>(se >> 15) << 63) |
              (static_cast<uint64_t>(exp - 16383 + 1023) << 52) |
              (keep & 0xFFFFFFFFFFFFFull);
        update(drop ? static_cast<uint16_t>(SW_PE | (up ? SW_C1 : 0)) : 0, SW_C1);
        if (pop)
          popRaw();
        return true;
      }
    }
    return fstF64Slow(out, pop);
  }
  bool fst80(Float80 &out, bool pop);
  bool fist(int bits, int64_t &out, bool pop);
  bool fbstp(uint8_t *bcd10);
  void fstReg(int i, bool pop) {
    if (!isEmpty(0)) [[likely]] {
      setSt(i, st(0));
      m_sw = static_cast<uint16_t>(m_sw & ~SW_C1);
      if (pop)
        popRaw();
      return;
    }
    fstRegSlow(i, pop);
  }
  // Reserved alias D9 D8+i (FSTP1): like FSTP ST(i), but an empty ST0 is
  // just popped without a stack fault.
  void fstp1(int i);

  // ST(dst) = ST(dst) op ST(src) [, pop]
  void arithReg(Arith op, int dst, int src, bool pop) {
#if FADOR_X87_HOST
    if (!isEmpty(dst) && !isEmpty(src)) [[likely]] {
      uint16_t flags = knownFlags();
      if (host::fastReady(m_cw, flags)) [[likely]] {
        host::fastArith(op, &m_regs[phys(dst)], &m_regs[phys(src)], flags);
        update(flags, SW_C1);
        if (pop)
          popRaw();
        return;
      }
    }
#endif
    arithRegSlow(op, dst, src, pop);
  }
  // ST0 = ST0 op mem
  void arithMem(Arith op, MemFmt fmt, uint64_t raw) {
#if FADOR_X87_HOST
    if (!isEmpty(0)) [[likely]] {
      uint16_t flags = knownFlags();
      if (host::fastReady(m_cw, flags)) [[likely]] {
        host::fastArithMem(op, &m_regs[phys(0)], fmt, &raw, flags);
        update(flags, SW_C1);
        return;
      }
    }
#endif
    arithMemSlow(op, fmt, raw);
  }

  // FCOM/FCOMP/FCOMPP/FUCOM*: compare ST0 with ST(i), then pop `pops` times.
  void fcomReg(int i, bool unordered, int pops);
  void fcomMem(MemFmt fmt, uint64_t raw, bool pop);
  // FCOMI/FCOMIP/FUCOMI/FUCOMIP: returns the updated EFLAGS.
  uint32_t fcomi(int i, bool unordered, bool pop, uint32_t eflags);
  void ftst();
  void fxam();
  void fchs();
  void fabs();
  void fxch(int i) {
    if (!isEmpty(0) && !isEmpty(i)) [[likely]] {
      const int p0 = phys(0), pi = phys(i);
      const Float80 a = loadFields(m_regs[p0]);
      const Float80 b = loadFields(m_regs[pi]);
      storeWhole(m_regs[p0], b);
      storeWhole(m_regs[pi], a);
      m_sw = static_cast<uint16_t>(m_sw & ~SW_C1);
      return;
    }
    fxchSlow(i);
  }
  void ffree(int i) {
    freeReg(i);
    update(0, SW_C1);
  }
  // FFREEP (undocumented DF C0+i): free ST(i), then pop.
  void ffreep(int i) {
    freeReg(i);
    popRaw();
    update(0, SW_C1);
  }
  void fincstp();
  void fdecstp();
  void fcmov(bool condition, int i);
  // D9 E0..FF register-stack operations that go through the back end
  // (F2XM1 ... FCOS).
  void stackOp(uint8_t modrm);
  void fnclex();

  // ---- environment / state images ------------------------------------------
  void storeEnv(uint8_t *out, ImageMode mode) const;
  void loadEnv(const uint8_t *in, ImageMode mode);
  // ST(0)..ST(7) as 8 x 10 bytes (FSAVE/FRSTOR register area).
  void storeRegs(uint8_t *out80) const;
  void loadRegs(const uint8_t *in80);

private:
  int phys(int i) const { return (m_top + i) & 7; }

  // Out-of-line parts of the inline fast paths above (stack faults,
  // special values, rounding).
  void fldF32Slow(uint32_t raw);
  void fldF64Slow(uint64_t raw);
  bool fstF64Slow(uint64_t &out, bool pop);
  void fxchSlow(int i);
  void fstRegSlow(int i, bool pop);
  void fldRegSlow(int i);
  void arithRegSlow(Arith op, int dst, int src, bool pop);
  void arithMemSlow(Arith op, MemFmt fmt, uint64_t raw);
  void fldConstSlow(Const c);

  void refreshSummary() {
    if (m_sw & ~m_cw & SW_EXC)
      m_sw = static_cast<uint16_t>(m_sw | SW_ES | SW_B);
    else
      m_sw = static_cast<uint16_t>(m_sw & ~(SW_ES | SW_B));
  }

  // Merges exception flags (sticky) and replaces the condition bits in
  // `ccMask` with those in `flags`.
  void update(uint16_t flags, uint16_t ccMask) {
    const uint16_t added = static_cast<uint16_t>(flags & SW_EXC & ~m_sw);
    m_sw = static_cast<uint16_t>((m_sw & ~ccMask) | (flags & ccMask) |
                                 (flags & (SW_EXC | SW_SF)));
    if (added) // ES/B can only change when exception flags are added
      refreshSummary();
  }
  // True if an unmasked IE/DE/ZE in `flags` suppresses the result.
  bool suppressed(uint16_t flags) const {
    return (flags & ~m_cw & (SW_IE | SW_DE | SW_ZE)) != 0;
  }
  bool ieMasked() const { return (m_cw & SW_IE) != 0; }
  // Masked sticky exception flags already set: passed to the back end,
  // which may report them again (see HostX87.cpp).
  uint16_t knownFlags() const { return static_cast<uint16_t>(m_sw & m_cw & SW_EXC); }

  // Stack faults: set IE|SF and C1 (1 = overflow, 0 = underflow); return
  // true if the masked response should be applied.
  bool stackUnderflow() {
    update(SW_IE | SW_SF, SW_C1);
    return ieMasked();
  }
  bool stackOverflow() {
    update(SW_IE | SW_SF | SW_C1, SW_C1);
    return ieMasked();
  }
  // Push target (ST7) not empty?
  bool pushBlocked() const { return !isEmpty(7); }
  void pushRaw(const Float80 &v) {
    m_top = static_cast<uint8_t>((m_top - 1) & 7);
    setSt(0, v);
  }
  void popRaw() {
    freeReg(0);
    m_top = static_cast<uint8_t>((m_top + 1) & 7);
  }
  // Common FLD tail: handles the stack overflow / masked response.
  void pushChecked(const Float80 &v, uint16_t flags);

  std::array<Float80, 8> m_regs{};
  uint8_t m_emptyMask = 0xFF;
  uint8_t m_top = 0;
  uint16_t m_cw = kDefaultControlWord;
  uint16_t m_sw = 0; // without TOP
};

} // namespace fador::cpu::x87
