#pragma once

// Inline fast paths of the host x87 back end (see HostX87.cpp), used by the
// FPU for the hottest operations while the guest masks every exception (the
// normal case: no rebiased results, no suppressed results, so the host
// stack depth after each instruction is fixed and no flag checks are needed
// between the instruction and its stores).
//
// The host control word and the host's sticky exception flags are tracked
// in thread-local variables so neither needs reading per operation; FNCLEX
// (slow) only runs when the host has a flag the guest has not.

#include "X87Math.hpp"

#if FADOR_X87_HOST

#if defined(_MSC_VER)
#define FADOR_X87_HOST_INLINE __forceinline
#else
#define FADOR_X87_HOST_INLINE inline __attribute__((always_inline))
#endif

namespace fador::cpu::x87::host {

// Host x87 control word / sticky exception flags as last set or observed by
// the back end (the emulator does no other long double arithmetic).
inline thread_local uint16_t t_hostCW = 0x037F;
inline thread_local uint16_t t_hostFlags = 0;

// Gets the host ready for a fast-path operation under guest control word
// `cw`; `known` = the guest's masked sticky flags.  Returns false if the
// generic (checked) kernels must be used.
FADOR_X87_HOST_INLINE bool fastReady(uint16_t cw, uint16_t known) {
  if ((cw & SW_EXC) != SW_EXC) [[unlikely]]
    return false;
  if (t_hostCW != cw) [[unlikely]] {
    asm volatile("fldcw %0" : : "m"(cw));
    t_hostCW = cw;
  }
  if (t_hostFlags & ~known & SW_EXC) [[unlikely]] {
    asm volatile("fnclex");
    t_hostFlags = 0;
  }
  return true;
}

// The asm statements only touch their listed memory operands.
#define FADOR_X87_CLOBBERS                                                     \
  "st", "st(1)", "st(2)", "st(3)", "st(4)", "st(5)", "st(6)", "st(7)"

// *dst = *dst op *src (both in the register file).  ST1 = dst, ST0 = src,
// then the popping form DE xx leaves the result alone in ST0:
// DE C1 FADDP, DE C9 FMULP, DE E9 FSUBP (ST1-ST0), DE E1 FSUBRP (ST0-ST1),
// DE F9 FDIVP (ST1/ST0), DE F1 FDIVRP (ST0/ST1).
FADOR_X87_HOST_INLINE void fastArith(Arith op, Float80 *dst, const Float80 *src, uint16_t &sw) {
  uint16_t s;
#define FADOR_X87_FAST_ARITH(byte)                                             \
  asm volatile("fldt %0\n\t"                                                   \
               "fldt %2\n\t"                                                   \
               ".byte 0xDE, " #byte "\n\t"                                     \
               "fnstsw %%ax\n\t"                                               \
               "fstpt %0"                                                      \
               : "+m"(*dst), "=a"(s)                                           \
               : "m"(*src)                                                     \
               : FADOR_X87_CLOBBERS)
  switch (op) {
  case Arith::Add: FADOR_X87_FAST_ARITH(0xC1); break;
  case Arith::Mul: FADOR_X87_FAST_ARITH(0xC9); break;
  case Arith::Sub: FADOR_X87_FAST_ARITH(0xE9); break;
  case Arith::SubR: FADOR_X87_FAST_ARITH(0xE1); break;
  case Arith::Div: FADOR_X87_FAST_ARITH(0xF9); break;
  default: FADOR_X87_FAST_ARITH(0xF1); break;
  }
#undef FADOR_X87_FAST_ARITH
  t_hostFlags = static_cast<uint16_t>(s & SW_EXC);
  sw = static_cast<uint16_t>(sw | (s & (SW_EXC | SW_CC)));
}

// *dst = *dst op memory operand.
FADOR_X87_HOST_INLINE void fastArithMem(Arith op, Float80 *dst, MemFmt fmt, const uint64_t *raw,
                         uint16_t &sw) {
  uint16_t s;
#define FADOR_X87_FAST_MEM(insn)                                               \
  asm volatile("fldt %0\n\t" insn " %2\n\t"                                    \
               "fnstsw %%ax\n\t"                                               \
               "fstpt %0"                                                      \
               : "+m"(*dst), "=a"(s)                                           \
               : "m"(*raw)                                                     \
               : FADOR_X87_CLOBBERS)
#define FADOR_X87_FAST_MEM_SET(a, m, s_, sr, d, dr)                            \
  switch (op) {                                                                \
  case Arith::Add: FADOR_X87_FAST_MEM(a); break;                               \
  case Arith::Mul: FADOR_X87_FAST_MEM(m); break;                               \
  case Arith::Sub: FADOR_X87_FAST_MEM(s_); break;                              \
  case Arith::SubR: FADOR_X87_FAST_MEM(sr); break;                             \
  case Arith::Div: FADOR_X87_FAST_MEM(d); break;                               \
  default: FADOR_X87_FAST_MEM(dr); break;                                      \
  }
  switch (fmt) {
  case MemFmt::F32:
    FADOR_X87_FAST_MEM_SET("fadds", "fmuls", "fsubs", "fsubrs", "fdivs", "fdivrs")
    break;
  case MemFmt::F64:
    FADOR_X87_FAST_MEM_SET("faddl", "fmull", "fsubl", "fsubrl", "fdivl", "fdivrl")
    break;
  case MemFmt::I32:
    FADOR_X87_FAST_MEM_SET("fiaddl", "fimull", "fisubl", "fisubrl", "fidivl", "fidivrl")
    break;
  default:
    FADOR_X87_FAST_MEM_SET("fiadds", "fimuls", "fisubs", "fisubrs", "fidivs", "fidivrs")
    break;
  }
#undef FADOR_X87_FAST_MEM_SET
#undef FADOR_X87_FAST_MEM
  t_hostFlags = static_cast<uint16_t>(s & SW_EXC);
  sw = static_cast<uint16_t>(sw | (s & (SW_EXC | SW_CC)));
}

// Compare *a with *b (FCOM / FUCOM rules); sets C0/C2/C3 and IE/DE.
FADOR_X87_HOST_INLINE void fastCompare(const Float80 *a, const Float80 *b, bool unordered,
                        uint16_t &sw) {
  uint16_t s;
  if (unordered)
    asm volatile("fldt %2\n\tfldt %1\n\tfucompp\n\tfnstsw %%ax"
                 : "=a"(s) : "m"(*a), "m"(*b) : FADOR_X87_CLOBBERS);
  else
    asm volatile("fldt %2\n\tfldt %1\n\tfcompp\n\tfnstsw %%ax"
                 : "=a"(s) : "m"(*a), "m"(*b) : FADOR_X87_CLOBBERS);
  t_hostFlags = static_cast<uint16_t>(s & SW_EXC);
  sw = static_cast<uint16_t>(sw | (s & (SW_EXC | SW_CC)));
}

// FSQRT / FRNDINT in place.
FADOR_X87_HOST_INLINE void fastUnary(uint8_t modrm, Float80 *v, uint16_t &sw) {
  uint16_t s;
  if (modrm == 0xFA)
    asm volatile("fldt %0\n\tfsqrt\n\tfnstsw %%ax\n\tfstpt %0"
                 : "+m"(*v), "=a"(s) : : FADOR_X87_CLOBBERS);
  else
    asm volatile("fldt %0\n\tfrndint\n\tfnstsw %%ax\n\tfstpt %0"
                 : "+m"(*v), "=a"(s) : : FADOR_X87_CLOBBERS);
  t_hostFlags = static_cast<uint16_t>(s & SW_EXC);
  sw = static_cast<uint16_t>(sw | (s & (SW_EXC | SW_CC)));
}

// FIST/FISTP (bits = 16/32/64), FST/FSTP m32/m64 to `out`.
FADOR_X87_HOST_INLINE void fastStoreInt(const Float80 *v, int bits, int64_t &out, uint16_t &sw) {
  uint16_t s;
  if (bits == 16) {
    int16_t r;
    asm volatile("fldt %2\n\tfistps %0\n\tfnstsw %%ax"
                 : "=m"(r), "=a"(s) : "m"(*v) : FADOR_X87_CLOBBERS);
    out = r;
  } else if (bits == 32) {
    int32_t r;
    asm volatile("fldt %2\n\tfistpl %0\n\tfnstsw %%ax"
                 : "=m"(r), "=a"(s) : "m"(*v) : FADOR_X87_CLOBBERS);
    out = r;
  } else {
    int64_t r;
    asm volatile("fldt %2\n\tfistpll %0\n\tfnstsw %%ax"
                 : "=m"(r), "=a"(s) : "m"(*v) : FADOR_X87_CLOBBERS);
    out = r;
  }
  t_hostFlags = static_cast<uint16_t>(s & SW_EXC);
  sw = static_cast<uint16_t>(sw | (s & (SW_EXC | SW_CC)));
}

FADOR_X87_HOST_INLINE void fastStoreF64(const Float80 *v, uint64_t &out, uint16_t &sw) {
  uint16_t s;
  asm volatile("fldt %2\n\tfstpl %0\n\tfnstsw %%ax"
               : "=m"(out), "=a"(s) : "m"(*v) : FADOR_X87_CLOBBERS);
  t_hostFlags = static_cast<uint16_t>(s & SW_EXC);
  sw = static_cast<uint16_t>(sw | (s & (SW_EXC | SW_CC)));
}

FADOR_X87_HOST_INLINE void fastStoreF32(const Float80 *v, uint32_t &out, uint16_t &sw) {
  uint16_t s;
  asm volatile("fldt %2\n\tfstps %0\n\tfnstsw %%ax"
               : "=m"(out), "=a"(s) : "m"(*v) : FADOR_X87_CLOBBERS);
  t_hostFlags = static_cast<uint16_t>(s & SW_EXC);
  sw = static_cast<uint16_t>(sw | (s & (SW_EXC | SW_CC)));
}

#undef FADOR_X87_CLOBBERS

} // namespace fador::cpu::x87::host

#endif // FADOR_X87_HOST
