// Host x87 back end (GCC/Clang on x86 / x86-64): each guest operation is
// executed by the host's own x87 unit with the guest's control word, so the
// 80-bit results, condition codes and exception flags are exactly those of
// a real x87.  Operands are loaded with FLD TBYTE (which never raises an
// exception) and results are stored with FSTP TBYTE.
//
// Every kernel follows the same protocol: record the host status word,
// clear stale host exception flags, load the operands, run the instruction,
// record the status word, clear the flags again (so an unmasked exception
// can never trap in the host), then pop however many values the
// instruction left on the host stack (derived from TOP), so the host x87
// stack is always empty again on return.
//
// The host control word is only reloaded when the guest's changes; while
// the guest masks every exception (the common case) the host keeps that
// control word between operations.  Nothing else in the emulator computes
// with long double, so the host precision/rounding setting is harmless.

#include "X87Math.hpp"

#if FADOR_X87_HOST

#include <cstring>

namespace fador::cpu::x87::host {

namespace {

struct alignas(16) Frame {
  Float80 in0;     // 0   ST0 operand
  Float80 in1;     // 16  ST1 operand
  Float80 out[3];  // 32, 48, 64: host ST0, ST1, ST2 after the instruction
  uint8_t mem[16]; // 80  memory operand / memory result
  uint16_t sw0;    // 96  status word before
  uint16_t sw;     // 98  status word after
  uint32_t depth;  // 100 values left on the host stack
};
static_assert(sizeof(Float80) == 16);

thread_local uint16_t t_hostCW = 0x037F;

#define FADOR_X87_BEGIN                                                        \
  "fnstsw 96(%[f])\n\t"                                                        \
  "testb $0x3f, 96(%[f])\n\t"                                                  \
  "jz 0f\n\t"                                                                  \
  "fnclex\n"                                                                   \
  "0:\n\t"

#define FADOR_X87_END                                                          \
  "fnstsw 98(%[f])\n\t"                                                        \
  "testb $0x3f, 98(%[f])\n\t"                                                  \
  "jz 1f\n\t"                                                                  \
  "fnclex\n"                                                                   \
  "1:\n\t"                                                                     \
  "movzwl 96(%[f]), %k[a]\n\t"                                                 \
  "shrl $11, %k[a]\n\t"                                                        \
  "movzwl 98(%[f]), %k[b]\n\t"                                                 \
  "shrl $11, %k[b]\n\t"                                                        \
  "subl %k[b], %k[a]\n\t"                                                      \
  "andl $7, %k[a]\n\t"                                                         \
  "movl %k[a], 100(%[f])\n\t"                                                  \
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

// Runs a kernel with the guest control word in effect.
inline void run(Kernel k, Frame &f, uint16_t cw) {
  const bool allMasked = (cw & SW_EXC) == SW_EXC;
  if (t_hostCW != cw) {
    // The kernels leave no exception flags behind, so loading a control word
    // that unmasks exceptions cannot raise a pending one here.
    asm volatile("fnclex\n\tfldcw %0" : : "m"(cw));
    t_hostCW = cw;
  }
  k(&f);
  if (!allMasked) {
    // Never leave unmasked exceptions enabled in the host between calls.
    const uint16_t safe = static_cast<uint16_t>(cw | SW_EXC);
    asm volatile("fldcw %0" : : "m"(safe));
    t_hostCW = safe;
  }
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
  f.in0 = a;
  f.in1 = b;
  run(arithKernel(op), f, cw);
  sw |= f.sw & kResultFlags;
  return f.out[0];
}

Float80 arithMem(Arith op, const Float80 &a, MemFmt fmt, uint64_t raw,
                 uint16_t cw, uint16_t &sw) {
  Frame f;
  f.in0 = a;
  std::memcpy(f.mem, &raw, sizeof(raw));
  run(memArithKernel(op, fmt), f, cw);
  sw |= f.sw & kResultFlags;
  return f.out[0];
}

void compare(const Float80 &a, const Float80 &b, bool unordered, uint16_t &sw) {
  Frame f;
  f.in0 = a;
  f.in1 = b;
  run(unordered ? kUcom : kCom, f, t_hostCW | SW_EXC);
  sw |= f.sw & kResultFlags;
}

void compareMem(const Float80 &a, MemFmt fmt, uint64_t raw, uint16_t &sw) {
  static constexpr Kernel table[4] = {kComF32, kComI32, kComF64, kComI16};
  Frame f;
  f.in0 = a;
  std::memcpy(f.mem, &raw, sizeof(raw));
  run(table[static_cast<int>(fmt) & 3], f, t_hostCW | SW_EXC);
  sw |= f.sw & kResultFlags;
}

Float80 loadF32(uint32_t raw, uint16_t cw, uint16_t &sw) {
  Frame f;
  std::memcpy(f.mem, &raw, sizeof(raw));
  run(kLdF32, f, cw);
  sw |= f.sw & kResultFlags;
  return f.out[0];
}

Float80 loadF64(uint64_t raw, uint16_t cw, uint16_t &sw) {
  Frame f;
  std::memcpy(f.mem, &raw, sizeof(raw));
  run(kLdF64, f, cw);
  sw |= f.sw & kResultFlags;
  return f.out[0];
}

Float80 loadBCD(const uint8_t *bcd10) {
  Frame f;
  std::memcpy(f.mem, bcd10, 10);
  run(kLdBCD, f, t_hostCW | SW_EXC);
  return f.out[0];
}

uint32_t storeF32(const Float80 &v, uint16_t cw, uint16_t &sw) {
  Frame f;
  f.in0 = v;
  run(kStF32, f, cw);
  sw |= f.sw & kResultFlags;
  uint32_t r;
  std::memcpy(&r, f.mem, sizeof(r));
  return r;
}

uint64_t storeF64(const Float80 &v, uint16_t cw, uint16_t &sw) {
  Frame f;
  f.in0 = v;
  run(kStF64, f, cw);
  sw |= f.sw & kResultFlags;
  uint64_t r;
  std::memcpy(&r, f.mem, sizeof(r));
  return r;
}

int64_t storeInt(const Float80 &v, int bits, uint16_t cw, uint16_t &sw) {
  Frame f;
  f.in0 = v;
  run(bits == 16 ? kStI16 : bits == 32 ? kStI32 : kStI64, f, cw);
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
  f.in0 = v;
  run(kStBCD, f, cw);
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
  f.in0 = in[0];
  if (nIn > 1)
    f.in1 = in[1];
  run(k, f, cw);
  sw |= f.sw & kResultFlags;
  const int depth = static_cast<int>(f.depth);
  for (int i = 0; i < depth && i < 3; ++i)
    out[i] = f.out[i];
  return depth;
}

void restoreHostControlWord() {
  const uint16_t cw = 0x037F;
  asm volatile("fnclex\n\tfldcw %0" : : "m"(cw));
  t_hostCW = cw;
}

} // namespace fador::cpu::x87::host

#endif // FADOR_X87_HOST
