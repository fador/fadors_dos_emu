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
  Float80 st(int i) const { return m_regs[phys(i)]; }
  void setSt(int i, const Float80 &v) {
    const int p = phys(i);
    m_regs[p] = v;
    m_emptyMask = static_cast<uint8_t>(m_emptyMask & ~(1u << p));
  }
  void freeReg(int i) { m_emptyMask = static_cast<uint8_t>(m_emptyMask | (1u << phys(i))); }
  Float80 physicalReg(int p) const { return m_regs[p & 7]; }
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
  void fldReg(int i);
  void fldF32(uint32_t raw);
  void fldF64(uint64_t raw);
  void fildValue(int64_t v);
  void fbld(const uint8_t *bcd10);
  void fldConst(Const c);

  // Stores.  Return true if the memory operand must be written.
  bool fstF32(uint32_t &out, bool pop);
  bool fstF64(uint64_t &out, bool pop);
  bool fst80(Float80 &out, bool pop);
  bool fist(int bits, int64_t &out, bool pop);
  bool fbstp(uint8_t *bcd10);
  void fstReg(int i, bool pop);
  // Reserved alias D9 D8+i (FSTP1): like FSTP ST(i), but an empty ST0 is
  // just popped without a stack fault.
  void fstp1(int i);

  // ST(dst) = ST(dst) op ST(src) [, pop]
  void arithReg(Arith op, int dst, int src, bool pop);
  // ST0 = ST0 op mem
  void arithMem(Arith op, MemFmt fmt, uint64_t raw);

  // FCOM/FCOMP/FCOMPP/FUCOM*: compare ST0 with ST(i), then pop `pops` times.
  void fcomReg(int i, bool unordered, int pops);
  void fcomMem(MemFmt fmt, uint64_t raw, bool pop);
  // FCOMI/FCOMIP/FUCOMI/FUCOMIP: returns the updated EFLAGS.
  uint32_t fcomi(int i, bool unordered, bool pop, uint32_t eflags);
  void ftst();
  void fxam();
  void fchs();
  void fabs();
  void fxch(int i);
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

  void refreshSummary() {
    if (m_sw & ~m_cw & SW_EXC)
      m_sw = static_cast<uint16_t>(m_sw | SW_ES | SW_B);
    else
      m_sw = static_cast<uint16_t>(m_sw & ~(SW_ES | SW_B));
  }

  // Merges exception flags (sticky) and replaces the condition bits in
  // `ccMask` with those in `flags`.
  void update(uint16_t flags, uint16_t ccMask) {
    m_sw = static_cast<uint16_t>((m_sw & ~ccMask) | (flags & ccMask) |
                                 (flags & (SW_EXC | SW_SF)));
    refreshSummary();
  }
  // True if an unmasked IE/DE/ZE in `flags` suppresses the result.
  bool suppressed(uint16_t flags) const {
    return (flags & ~m_cw & (SW_IE | SW_DE | SW_ZE)) != 0;
  }
  bool ieMasked() const { return (m_cw & SW_IE) != 0; }

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
