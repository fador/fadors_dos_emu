// Host x87 back end (GCC/Clang on x86 / x86-64): each guest operation is
// executed by the host's own x87 unit with the guest's control word, so the
// 80-bit results, condition codes and exception flags are exactly those of
// a real x87.  Operands are loaded with FLD TBYTE (which never raises an
// exception) and results are stored with FSTP TBYTE.
//
// Every kernel follows the same protocol: record the host status word,
// load the operands, run the instruction, record the status word, then pop
// however many values the instruction left on the host stack (derived from
// TOP), so the host x87 stack is always empty again on return.
//
// FNCLEX is very slow on some hosts (~70 ns when flags are set), so the
// host's sticky exception flags are only cleared when they would make the
// result ambiguous: before the operation if the host has a flag the guest
// does not (the caller passes the guest's masked sticky flags in `sw`), and
// after it if an unmasked exception was raised (it would otherwise trap at
// the host's next x87 instruction).  Flags reported back may therefore
// include ones the guest already has, which is harmless as they are sticky.
//
// The host control word is only reloaded when the guest's changes; while
// the guest masks every exception (the common case) the host keeps that
// control word between operations.  Nothing else in the emulator computes
// with long double, so the host precision/rounding setting is harmless.

#include "HostX87Inline.hpp"

#if FADOR_X87_HOST

#include <cstring>

namespace fador::cpu::x87::host {

namespace {

// 10-byte extended value in a 16-byte slot, deliberately left
// uninitialised (no zeroing on every call).
struct Raw80 {
  uint64_t mant;
  uint16_t signExp;
  uint16_t pad[3];
};

struct alignas(16) Frame {
  Raw80 in0;       // 0   ST0 operand
  Raw80 in1;       // 16  ST1 operand
  Raw80 out[3];    // 32, 48, 64: host ST0, ST1, ST2 after the instruction
  uint8_t mem[16]; // 80  memory operand / memory result
  uint16_t sw0;    // 96  status word before
  uint16_t sw;     // 98  status word after
  uint16_t depth;  // 100 values left on the host stack
  uint16_t known;  // 102 guest's masked sticky flags (in)
  uint16_t cw;     // 104 guest control word (in)
};
static_assert(sizeof(Float80) == 16 && sizeof(Raw80) == 16);

inline void put(Raw80 &r, const Float80 &v) {
#if defined(__SSE2__)
  _mm_storeu_si128(reinterpret_cast<__m128i *>(&r),
                   _mm_set_epi64x(static_cast<long long>(v.signExp),
                                  static_cast<long long>(v.mant)));
#else
  r.mant = v.mant;
  r.signExp = v.signExp;
#endif
}

// In-place arithmetic: the operands are loaded straight from the FPU
// register file and the result is stored straight back (no copies, and the
// 10-byte FSTP/FLD pairs forward from the store buffer).  On an unmasked
// IE/DE/ZE the x87 leaves the destination unchanged, so storing it back is
// harmless.
struct InPlace {
  const void *a;   // 0  destination operand (ST0 or ST(i))
  const void *b;   // 8  source operand (register or memory)
  void *out;       // 16 = a
  uint16_t sw0;    // 24
  uint16_t sw;     // 26
  uint16_t known;  // 28
  uint16_t cw;     // 30
};


#define FADOR_X87_BEGIN                                                        \
  "fnstsw 96(%[f])\n\t"                                                        \
  "movzwl 102(%[f]), %k[a]\n\t"                                                \
  "notl %k[a]\n\t"                                                             \
  "andw 96(%[f]), %w[a]\n\t"                                                   \
  "testl $0x3f, %k[a]\n\t"                                                     \
  "jz 0f\n\t"                                                                  \
  "fnclex\n\t"                                                                 \
  "fnstsw 96(%[f])\n"                                                          \
  "0:\n\t"

#define FADOR_X87_END                                                          \
  "fnstsw 98(%[f])\n\t"                                                        \
  "movzwl 104(%[f]), %k[a]\n\t"                                                \
  "notl %k[a]\n\t"                                                             \
  "andw 98(%[f]), %w[a]\n\t"                                                   \
  "testl $0x3f, %k[a]\n\t"                                                     \
  "jz 1f\n\t"                                                                  \
  "fnclex\n"                                                                   \
  "1:\n\t"                                                                     \
  "movzwl 96(%[f]), %k[a]\n\t"                                                 \
  "shrl $11, %k[a]\n\t"                                                        \
  "movzwl 98(%[f]), %k[b]\n\t"                                                 \
  "shrl $11, %k[b]\n\t"                                                        \
  "subl %k[b], %k[a]\n\t"                                                      \
  "andl $7, %k[a]\n\t"                                                         \
  "movw %w[a], 100(%[f])\n\t"                                                  \
  "lea 32(%[f]), %[p]\n"                                                       \
  "2:\n\t"                                                                     \
  "testl %k[a], %k[a]\n\t"                                                     \
  "jz 3f\n\t"                                                                  \
  "fstpt (%[p])\n\t"                                                           \
  "add $16, %[p]\n\t"                                                          \
  "decl %k[a]\n\t"                                                             \
  "jmp 2b\n"                                                                   \
  "3:\n\t"

#define FADOR_X87_LOAD1 "fldt 0(%[f])\n\t"
#define FADOR_X87_LOAD2 "fldt 16(%[f])\n\t fldt 0(%[f])\n\t"

#define FADOR_X87_KERNEL(name, body)                                           \
  [[gnu::noinline]] void name(Frame *f) {                                      \
    uintptr_t a_, b_;                                                          \
    void *p_;                                                                  \
    asm volatile(FADOR_X87_BEGIN body FADOR_X87_END                            \
                 : [a] "=&r"(a_), [b] "=&r"(b_), [p] "=&r"(p_)                 \
                 : [f] "r"(f)                                                  \
                 : "memory", "cc", "st", "st(1)", "st(2)", "st(3)", "st(4)",   \
                   "st(5)", "st(6)", "st(7)");                                 \
  }

#define FADOR_X87_IP_BEGIN                                                     \
  "fnstsw 24(%[f])\n\t"                                                        \
  "movzwl 28(%[f]), %k[a]\n\t"                                                 \
  "notl %k[a]\n\t"                                                             \
  "andw 24(%[f]), %w[a]\n\t"                                                   \
  "testl $0x3f, %k[a]\n\t"                                                     \
  "jz 0f\n\t"                                                                  \
  "fnclex\n"                                                                   \
  "0:\n\t"

#define FADOR_X87_IP_END                                                       \
  "fnstsw 26(%[f])\n\t"                                                        \
  "movzwl 30(%[f]), %k[a]\n\t"                                                 \
  "notl %k[a]\n\t"                                                             \
  "andw 26(%[f]), %w[a]\n\t"                                                   \
  "testl $0x3f, %k[a]\n\t"                                                     \
  "jz 1f\n\t"                                                                  \
  "fnclex\n"                                                                   \
  "1:\n\t"                                                                    \
  "mov 16(%[f]), %[p]\n\t"                                                     \
  "fstpt (%[p])\n\t"

#define FADOR_X87_IP_KERNEL(name, body, tail)                                  \
  [[gnu::noinline]] void name(InPlace *f) {                                    \
    uintptr_t a_;                                                              \
    void *p_;                                                                  \
    asm volatile(FADOR_X87_IP_BEGIN body FADOR_X87_IP_END tail                 \
                 : [a] "=&r"(a_), [p] "=&r"(p_)                                \
                 : [f] "r"(f)                                                  \
                 : "memory", "cc", "st", "st(1)", "st(2)", "st(3)", "st(4)",   \
                   "st(5)", "st(6)", "st(7)");                                 \
  }

// Register forms: load b then a (ST0 = a, ST1 = b), op ST0,ST1, store ST0,
// then drop b.
#define FADOR_X87_IP_REG(name, byte)                                           \
  FADOR_X87_IP_KERNEL(name, "mov 8(%[f]), %[p]\n\t fldt (%[p])\n\t"           \
                            "mov 0(%[f]), %[p]\n\t fldt (%[p])\n\t"           \
                            ".byte 0xD8, " #byte "\n\t",                    \
                      "fstp %%st(0)\n\t")
// Memory forms: ST0 = a op [b]
#define FADOR_X87_IP_MEM(name, insn)                                           \
  FADOR_X87_IP_KERNEL(name, "mov 0(%[f]), %[p]\n\t fldt (%[p])\n\t"           \
                            "mov 8(%[f]), %[p]\n\t " insn " (%[p])\n\t", "")

FADOR_X87_IP_REG(ipAdd, 0xC1)
FADOR_X87_IP_REG(ipMul, 0xC9)
FADOR_X87_IP_REG(ipSub, 0xE1)
FADOR_X87_IP_REG(ipSubR, 0xE9)
FADOR_X87_IP_REG(ipDiv, 0xF1)
FADOR_X87_IP_REG(ipDivR, 0xF9)
FADOR_X87_IP_MEM(ipF32Add, "fadds")
FADOR_X87_IP_MEM(ipF32Mul, "fmuls")
FADOR_X87_IP_MEM(ipF32Sub, "fsubs")
FADOR_X87_IP_MEM(ipF32SubR, "fsubrs")
FADOR_X87_IP_MEM(ipF32Div, "fdivs")
FADOR_X87_IP_MEM(ipF32DivR, "fdivrs")
FADOR_X87_IP_MEM(ipF64Add, "faddl")
FADOR_X87_IP_MEM(ipF64Mul, "fmull")
FADOR_X87_IP_MEM(ipF64Sub, "fsubl")
FADOR_X87_IP_MEM(ipF64SubR, "fsubrl")
FADOR_X87_IP_MEM(ipF64Div, "fdivl")
FADOR_X87_IP_MEM(ipF64DivR, "fdivrl")
FADOR_X87_IP_MEM(ipI32Add, "fiaddl")
FADOR_X87_IP_MEM(ipI32Mul, "fimull")
FADOR_X87_IP_MEM(ipI32Sub, "fisubl")
FADOR_X87_IP_MEM(ipI32SubR, "fisubrl")
FADOR_X87_IP_MEM(ipI32Div, "fidivl")
FADOR_X87_IP_MEM(ipI32DivR, "fidivrl")
FADOR_X87_IP_MEM(ipI16Add, "fiadds")
FADOR_X87_IP_MEM(ipI16Mul, "fimuls")
FADOR_X87_IP_MEM(ipI16Sub, "fisubs")
FADOR_X87_IP_MEM(ipI16SubR, "fisubrs")
FADOR_X87_IP_MEM(ipI16Div, "fidivs")
FADOR_X87_IP_MEM(ipI16DivR, "fidivrs")

// Register arithmetic: ST0 = a, ST1 = b; D8 C1/C9/E1/E9/F1/F9 = op ST0,ST1.
FADOR_X87_KERNEL(kAdd, FADOR_X87_LOAD2 ".byte 0xD8, 0xC1\n\t")
FADOR_X87_KERNEL(kMul, FADOR_X87_LOAD2 ".byte 0xD8, 0xC9\n\t")
FADOR_X87_KERNEL(kSub, FADOR_X87_LOAD2 ".byte 0xD8, 0xE1\n\t")
FADOR_X87_KERNEL(kSubR, FADOR_X87_LOAD2 ".byte 0xD8, 0xE9\n\t")
FADOR_X87_KERNEL(kDiv, FADOR_X87_LOAD2 ".byte 0xD8, 0xF1\n\t")
FADOR_X87_KERNEL(kDivR, FADOR_X87_LOAD2 ".byte 0xD8, 0xF9\n\t")

// Memory arithmetic: ST0 op m.  Written out explicitly (AT&T suffixes:
// s = m32fp / m16int, l = m64fp / m32int).
FADOR_X87_KERNEL(kF32Add, FADOR_X87_LOAD1 "fadds 80(%[f])\n\t")
FADOR_X87_KERNEL(kF32Mul, FADOR_X87_LOAD1 "fmuls 80(%[f])\n\t")
FADOR_X87_KERNEL(kF32Sub, FADOR_X87_LOAD1 "fsubs 80(%[f])\n\t")
FADOR_X87_KERNEL(kF32SubR, FADOR_X87_LOAD1 "fsubrs 80(%[f])\n\t")
FADOR_X87_KERNEL(kF32Div, FADOR_X87_LOAD1 "fdivs 80(%[f])\n\t")
FADOR_X87_KERNEL(kF32DivR, FADOR_X87_LOAD1 "fdivrs 80(%[f])\n\t")
FADOR_X87_KERNEL(kF64Add, FADOR_X87_LOAD1 "faddl 80(%[f])\n\t")
FADOR_X87_KERNEL(kF64Mul, FADOR_X87_LOAD1 "fmull 80(%[f])\n\t")
FADOR_X87_KERNEL(kF64Sub, FADOR_X87_LOAD1 "fsubl 80(%[f])\n\t")
FADOR_X87_KERNEL(kF64SubR, FADOR_X87_LOAD1 "fsubrl 80(%[f])\n\t")
FADOR_X87_KERNEL(kF64Div, FADOR_X87_LOAD1 "fdivl 80(%[f])\n\t")
FADOR_X87_KERNEL(kF64DivR, FADOR_X87_LOAD1 "fdivrl 80(%[f])\n\t")
FADOR_X87_KERNEL(kI32Add, FADOR_X87_LOAD1 "fiaddl 80(%[f])\n\t")
FADOR_X87_KERNEL(kI32Mul, FADOR_X87_LOAD1 "fimull 80(%[f])\n\t")
FADOR_X87_KERNEL(kI32Sub, FADOR_X87_LOAD1 "fisubl 80(%[f])\n\t")
FADOR_X87_KERNEL(kI32SubR, FADOR_X87_LOAD1 "fisubrl 80(%[f])\n\t")
FADOR_X87_KERNEL(kI32Div, FADOR_X87_LOAD1 "fidivl 80(%[f])\n\t")
FADOR_X87_KERNEL(kI32DivR, FADOR_X87_LOAD1 "fidivrl 80(%[f])\n\t")
FADOR_X87_KERNEL(kI16Add, FADOR_X87_LOAD1 "fiadds 80(%[f])\n\t")
FADOR_X87_KERNEL(kI16Mul, FADOR_X87_LOAD1 "fimuls 80(%[f])\n\t")
FADOR_X87_KERNEL(kI16Sub, FADOR_X87_LOAD1 "fisubs 80(%[f])\n\t")
FADOR_X87_KERNEL(kI16SubR, FADOR_X87_LOAD1 "fisubrs 80(%[f])\n\t")
FADOR_X87_KERNEL(kI16Div, FADOR_X87_LOAD1 "fidivs 80(%[f])\n\t")
FADOR_X87_KERNEL(kI16DivR, FADOR_X87_LOAD1 "fidivrs 80(%[f])\n\t")

// Compares (ST0 vs ST1 / memory), popping everything they load.
FADOR_X87_KERNEL(kCom, FADOR_X87_LOAD2 ".byte 0xDE, 0xD9\n\t")  // FCOMPP
FADOR_X87_KERNEL(kUcom, FADOR_X87_LOAD2 ".byte 0xDA, 0xE9\n\t") // FUCOMPP
FADOR_X87_KERNEL(kComF32, FADOR_X87_LOAD1 "fcomps 80(%[f])\n\t")
FADOR_X87_KERNEL(kComF64, FADOR_X87_LOAD1 "fcompl 80(%[f])\n\t")
FADOR_X87_KERNEL(kComI32, FADOR_X87_LOAD1 "ficompl 80(%[f])\n\t")
FADOR_X87_KERNEL(kComI16, FADOR_X87_LOAD1 "ficomps 80(%[f])\n\t")

// Loads and stores
FADOR_X87_KERNEL(kLdF32, "flds 80(%[f])\n\t")
FADOR_X87_KERNEL(kLdF64, "fldl 80(%[f])\n\t")
FADOR_X87_KERNEL(kLdBCD, "fbld 80(%[f])\n\t")
FADOR_X87_KERNEL(kStF32, FADOR_X87_LOAD1 "fstps 80(%[f])\n\t")
FADOR_X87_KERNEL(kStF64, FADOR_X87_LOAD1 "fstpl 80(%[f])\n\t")
FADOR_X87_KERNEL(kStI16, FADOR_X87_LOAD1 "fistps 80(%[f])\n\t")
FADOR_X87_KERNEL(kStI32, FADOR_X87_LOAD1 "fistpl 80(%[f])\n\t")
FADOR_X87_KERNEL(kStI64, FADOR_X87_LOAD1 "fistpll 80(%[f])\n\t")
FADOR_X87_KERNEL(kStBCD, FADOR_X87_LOAD1 "fbstp 80(%[f])\n\t")

// D9 E0..FF register-stack group
#define FADOR_X87_STACK1(name, byte)                                           \
  FADOR_X87_KERNEL(name, FADOR_X87_LOAD1 ".byte 0xD9, " #byte "\n\t")
#define FADOR_X87_STACK2(name, byte)                                           \
  FADOR_X87_KERNEL(name, FADOR_X87_LOAD2 ".byte 0xD9, " #byte "\n\t")
FADOR_X87_STACK1(kChs, 0xE0)
FADOR_X87_STACK1(kAbs, 0xE1)
FADOR_X87_STACK1(kF2xm1, 0xF0)
FADOR_X87_STACK2(kYl2x, 0xF1)
FADOR_X87_STACK1(kPtan, 0xF2)
FADOR_X87_STACK2(kPatan, 0xF3)
FADOR_X87_STACK1(kXtract, 0xF4)
FADOR_X87_STACK2(kPrem1, 0xF5)
FADOR_X87_STACK2(kPrem, 0xF8)
FADOR_X87_STACK2(kYl2xp1, 0xF9)
FADOR_X87_STACK1(kSqrt, 0xFA)
FADOR_X87_STACK1(kSincos, 0xFB)
FADOR_X87_STACK1(kRndint, 0xFC)
FADOR_X87_STACK2(kScale, 0xFD)
FADOR_X87_STACK1(kSin, 0xFE)
FADOR_X87_STACK1(kCos, 0xFF)

using Kernel = void (*)(Frame *);

// Runs a kernel with the guest control word in effect.  `known` = the
// guest's masked sticky exception flags (see the protocol above).
inline void run(Kernel k, Frame &f, uint16_t cw, uint16_t known) {
  const bool allMasked = (cw & SW_EXC) == SW_EXC;
  if (t_hostCW != cw) [[unlikely]] {
    // Sticky host flags that the new control word unmasks would become
    // pending (and trap): clear them first.  FNCLEX is slow, so only then.
    if (!allMasked) {
      uint16_t hsw;
      asm volatile("fnstsw %0" : "=m"(hsw));
      if (hsw & ~cw & SW_EXC)
        asm volatile("fnclex");
    }
    asm volatile("fldcw %0" : : "m"(cw));
    t_hostCW = cw;
  }
  f.cw = cw;
  f.known = static_cast<uint16_t>(known & cw & SW_EXC);
  k(&f);
  // The kernel cleared the host flags if an unmasked one was raised.
  t_hostFlags = (f.sw & ~cw & SW_EXC) ? 0 : static_cast<uint16_t>(f.sw & SW_EXC);
  if (!allMasked) {
    // Never leave unmasked exceptions enabled in the host between calls.
    const uint16_t safe = static_cast<uint16_t>(cw | SW_EXC);
    asm volatile("fldcw %0" : : "m"(safe));
    t_hostCW = safe;
  }
}

inline void runInPlace(void (*k)(InPlace *), InPlace &f, uint16_t cw, uint16_t known) {
  const bool allMasked = (cw & SW_EXC) == SW_EXC;
  if (t_hostCW != cw) [[unlikely]] {
    if (!allMasked) {
      uint16_t hsw;
      asm volatile("fnstsw %0" : "=m"(hsw));
      if (hsw & ~cw & SW_EXC)
        asm volatile("fnclex");
    }
    asm volatile("fldcw %0" : : "m"(cw));
    t_hostCW = cw;
  }
  f.cw = cw;
  f.known = static_cast<uint16_t>(known & cw & SW_EXC);
  k(&f);
  t_hostFlags = (f.sw & ~cw & SW_EXC) ? 0 : static_cast<uint16_t>(f.sw & SW_EXC);
  if (!allMasked) {
    const uint16_t safe = static_cast<uint16_t>(cw | SW_EXC);
    asm volatile("fldcw %0" : : "m"(safe));
    t_hostCW = safe;
  }
}

inline Float80 result(const Raw80 &v) {
  // Field-wise copy: a 16-byte load of the 10 bytes FSTP just stored would
  // defeat store forwarding.
  return Float80(v.mant, v.signExp);
}

constexpr uint16_t kResultFlags = SW_EXC | SW_CC;

Kernel arithKernel(Arith op) {
  switch (op) {
  case Arith::Add:
    return kAdd;
  case Arith::Mul:
    return kMul;
  case Arith::Sub:
    return kSub;
  case Arith::SubR:
    return kSubR;
  case Arith::Div:
    return kDiv;
  default:
    return kDivR;
  }
}

Kernel memArithKernel(Arith op, MemFmt fmt) {
  static constexpr Kernel table[4][8] = {
      {kF32Add, kF32Mul, nullptr, nullptr, kF32Sub, kF32SubR, kF32Div, kF32DivR},
      {kI32Add, kI32Mul, nullptr, nullptr, kI32Sub, kI32SubR, kI32Div, kI32DivR},
      {kF64Add, kF64Mul, nullptr, nullptr, kF64Sub, kF64SubR, kF64Div, kF64DivR},
      {kI16Add, kI16Mul, nullptr, nullptr, kI16Sub, kI16SubR, kI16Div, kI16DivR},
  };
  return table[static_cast<int>(fmt) & 3][static_cast<int>(op) & 7];
}

} // namespace

Float80 arith(Arith op, const Float80 &a, const Float80 &b, uint16_t cw,
              uint16_t &sw) {
  Frame f;
  put(f.in0, a);
  put(f.in1, b);
  run(arithKernel(op), f, cw, sw);
  sw |= f.sw & kResultFlags;
  return result(f.out[0]);
}

void arithTo(Arith op, Float80 *dst, const Float80 &src, uint16_t cw,
             uint16_t &sw) {
  static constexpr void (*table[8])(InPlace *) = {ipAdd, ipMul, nullptr, nullptr,
                                                  ipSub, ipSubR, ipDiv, ipDivR};
  InPlace f{dst, &src, dst, 0, 0, 0, 0};
  runInPlace(table[static_cast<int>(op) & 7], f, cw, sw);
  sw |= f.sw & kResultFlags;
}

void arithMemTo(Arith op, Float80 *dst, MemFmt fmt, uint64_t raw, uint16_t cw,
                uint16_t &sw) {
  static constexpr void (*table[4][8])(InPlace *) = {
      {ipF32Add, ipF32Mul, nullptr, nullptr, ipF32Sub, ipF32SubR, ipF32Div, ipF32DivR},
      {ipI32Add, ipI32Mul, nullptr, nullptr, ipI32Sub, ipI32SubR, ipI32Div, ipI32DivR},
      {ipF64Add, ipF64Mul, nullptr, nullptr, ipF64Sub, ipF64SubR, ipF64Div, ipF64DivR},
      {ipI16Add, ipI16Mul, nullptr, nullptr, ipI16Sub, ipI16SubR, ipI16Div, ipI16DivR},
  };
  InPlace f{dst, &raw, dst, 0, 0, 0, 0};
  runInPlace(table[static_cast<int>(fmt) & 3][static_cast<int>(op) & 7], f, cw, sw);
  sw |= f.sw & kResultFlags;
}

Float80 arithMem(Arith op, const Float80 &a, MemFmt fmt, uint64_t raw,
                 uint16_t cw, uint16_t &sw) {
  Frame f;
  put(f.in0, a);
  std::memcpy(f.mem, &raw, sizeof(raw));
  run(memArithKernel(op, fmt), f, cw, sw);
  sw |= f.sw & kResultFlags;
  return result(f.out[0]);
}

void compare(const Float80 &a, const Float80 &b, bool unordered, uint16_t &sw) {
  Frame f;
  put(f.in0, a);
  put(f.in1, b);
  run(unordered ? kUcom : kCom, f, static_cast<uint16_t>(t_hostCW | SW_EXC), sw);
  sw |= f.sw & kResultFlags;
}

void compareMem(const Float80 &a, MemFmt fmt, uint64_t raw, uint16_t &sw) {
  static constexpr Kernel table[4] = {kComF32, kComI32, kComF64, kComI16};
  Frame f;
  put(f.in0, a);
  std::memcpy(f.mem, &raw, sizeof(raw));
  run(table[static_cast<int>(fmt) & 3], f, static_cast<uint16_t>(t_hostCW | SW_EXC), sw);
  sw |= f.sw & kResultFlags;
}

Float80 loadF32(uint32_t raw, uint16_t cw, uint16_t &sw) {
  Frame f;
  std::memcpy(f.mem, &raw, sizeof(raw));
  run(kLdF32, f, cw, sw);
  sw |= f.sw & kResultFlags;
  return result(f.out[0]);
}

Float80 loadF64(uint64_t raw, uint16_t cw, uint16_t &sw) {
  Frame f;
  std::memcpy(f.mem, &raw, sizeof(raw));
  run(kLdF64, f, cw, sw);
  sw |= f.sw & kResultFlags;
  return result(f.out[0]);
}

Float80 loadBCD(const uint8_t *bcd10) {
  Frame f;
  std::memcpy(f.mem, bcd10, 10);
  run(kLdBCD, f, static_cast<uint16_t>(t_hostCW | SW_EXC), 0);
  return result(f.out[0]);
}

uint32_t storeF32(const Float80 &v, uint16_t cw, uint16_t &sw) {
  Frame f;
  put(f.in0, v);
  run(kStF32, f, cw, sw);
  sw |= f.sw & kResultFlags;
  uint32_t r;
  std::memcpy(&r, f.mem, sizeof(r));
  return r;
}

uint64_t storeF64(const Float80 &v, uint16_t cw, uint16_t &sw) {
  Frame f;
  put(f.in0, v);
  run(kStF64, f, cw, sw);
  sw |= f.sw & kResultFlags;
  uint64_t r;
  std::memcpy(&r, f.mem, sizeof(r));
  return r;
}

int64_t storeInt(const Float80 &v, int bits, uint16_t cw, uint16_t &sw) {
  Frame f;
  put(f.in0, v);
  run(bits == 16 ? kStI16 : bits == 32 ? kStI32 : kStI64, f, cw, sw);
  sw |= f.sw & kResultFlags;
  if (bits == 16) {
    int16_t r;
    std::memcpy(&r, f.mem, sizeof(r));
    return r;
  }
  if (bits == 32) {
    int32_t r;
    std::memcpy(&r, f.mem, sizeof(r));
    return r;
  }
  int64_t r;
  std::memcpy(&r, f.mem, sizeof(r));
  return r;
}

void storeBCD(const Float80 &v, uint8_t *bcd10, uint16_t cw, uint16_t &sw) {
  Frame f;
  put(f.in0, v);
  run(kStBCD, f, cw, sw);
  sw |= f.sw & kResultFlags;
  std::memcpy(bcd10, f.mem, 10);
}

int stackOp(uint8_t modrm, int nIn, const Float80 *in, Float80 *out,
            uint16_t cw, uint16_t &sw) {
  Kernel k = nullptr;
  switch (modrm) {
  case 0xE0: k = kChs; break;
  case 0xE1: k = kAbs; break;
  case 0xF0: k = kF2xm1; break;
  case 0xF1: k = kYl2x; break;
  case 0xF2: k = kPtan; break;
  case 0xF3: k = kPatan; break;
  case 0xF4: k = kXtract; break;
  case 0xF5: k = kPrem1; break;
  case 0xF8: k = kPrem; break;
  case 0xF9: k = kYl2xp1; break;
  case 0xFA: k = kSqrt; break;
  case 0xFB: k = kSincos; break;
  case 0xFC: k = kRndint; break;
  case 0xFD: k = kScale; break;
  case 0xFE: k = kSin; break;
  case 0xFF: k = kCos; break;
  default:
    for (int i = 0; i < nIn; ++i)
      out[i] = in[i];
    return nIn;
  }
  Frame f;
  put(f.in0, in[0]);
  if (nIn > 1)
    put(f.in1, in[1]);
  run(k, f, cw, sw);
  sw |= f.sw & kResultFlags;
  const int depth = static_cast<int>(f.depth);
  for (int i = 0; i < depth && i < 3; ++i)
    out[i] = result(f.out[i]);
  return depth;
}

void restoreHostControlWord() {
  const uint16_t cw = 0x037F;
  asm volatile("fnclex\n\tfldcw %0" : : "m"(cw));
  t_hostCW = cw;
  t_hostFlags = 0;
}

} // namespace fador::cpu::x87::host

#endif // FADOR_X87_HOST
