#pragma once

// Value-level x87 arithmetic back ends.
//
// Two implementations with an identical interface:
//   soft::  portable software 80-bit arithmetic (always compiled; used on
//           hosts without a native x87, e.g. MSVC where long double is
//           double, or ARM64)
//   host::  GCC/Clang on x86/x86-64: the operation is executed by the host's
//           own x87 unit with the guest's control word, so results and
//           exception flags are exact by construction
// `active` aliases the one selected at compile time (FADOR_X87_FORCE_SOFT
// forces the soft path; see the FADOR_SOFT_X87 CMake option).
//
// Conventions for every function:
//   cw   guest control word (precision control, rounding control, masks).
//        Masks only matter for the rebiased results of unmasked overflow /
//        underflow; the caller (FPU) applies the "no result" rule for
//        unmasked IE/DE/ZE itself.
//   sw   in: the guest's masked sticky exception flags (may be 0); out: ORed
//        with the exception flags (IE DE ZE OE UE PE) and the condition
//        bits the operation defines (C1 = "rounded up" for rounding
//        operations, C0..C3 for compares / FPREM, C2 for out-of-range trig).
//        Only bits the operation defines are meaningful; the caller masks.
// The functions never see empty registers: stack faults are handled by the
// FPU before calling them.

#include "Float80.hpp"

#include <cstdint>

#if !defined(FADOR_X87_FORCE_SOFT)
#define FADOR_X87_FORCE_SOFT 0
#endif

#if !FADOR_X87_FORCE_SOFT && !defined(_MSC_VER) &&                              \
    (defined(__GNUC__) || defined(__clang__)) &&                               \
    (defined(__x86_64__) || defined(__i386__))
#define FADOR_X87_HOST 1
#else
#define FADOR_X87_HOST 0
#endif

namespace fador::cpu::x87 {

inline constexpr uint16_t SW_IE = 0x0001;
inline constexpr uint16_t SW_DE = 0x0002;
inline constexpr uint16_t SW_ZE = 0x0004;
inline constexpr uint16_t SW_OE = 0x0008;
inline constexpr uint16_t SW_UE = 0x0010;
inline constexpr uint16_t SW_PE = 0x0020;
inline constexpr uint16_t SW_SF = 0x0040;
inline constexpr uint16_t SW_ES = 0x0080;
inline constexpr uint16_t SW_C0 = 0x0100;
inline constexpr uint16_t SW_C1 = 0x0200;
inline constexpr uint16_t SW_C2 = 0x0400;
inline constexpr uint16_t SW_TOP = 0x3800;
inline constexpr uint16_t SW_C3 = 0x4000;
inline constexpr uint16_t SW_B = 0x8000;
inline constexpr uint16_t SW_EXC = 0x003F;
inline constexpr uint16_t SW_CC = SW_C0 | SW_C1 | SW_C2 | SW_C3;

inline constexpr uint16_t CW_PC_SHIFT = 8;
inline constexpr uint16_t CW_RC_SHIFT = 10;

// Two-operand arithmetic: values match the ModRM reg field of D8 /r.
//   Add: a+b   Mul: a*b   Sub: a-b   SubR: b-a   Div: a/b   DivR: b/a
enum class Arith : uint8_t { Add = 0, Mul = 1, Sub = 4, SubR = 5, Div = 6, DivR = 7 };

// Memory operand formats of the D8/DA/DC/DE arithmetic group.
enum class MemFmt : uint8_t { F32 = 0, I32 = 1, F64 = 2, I16 = 3 };

// FLDPI etc. (index = ModRM - 0xE8 of D9 E8..EE)
enum class Const : uint8_t { One = 0, L2T, L2E, Pi, LG2, LN2, Zero };

#define FADOR_X87_BACKEND_API                                                    \
  Float80 arith(Arith op, const Float80 &a, const Float80 &b, uint16_t cw,     \
                uint16_t &sw);                                                 \
  /* a = ST0, the memory operand is the second operand */                      \
  Float80 arithMem(Arith op, const Float80 &a, MemFmt fmt, uint64_t raw,       \
                   uint16_t cw, uint16_t &sw);                                 \
  /* In place: *dst = *dst op src, left unchanged on an unmasked IE/DE/ZE      \
     (the FPU's hot path; dst points into the register file). */               \
  void arithTo(Arith op, Float80 *dst, const Float80 &src, uint16_t cw,        \
               uint16_t &sw);                                                  \
  void arithMemTo(Arith op, Float80 *dst, MemFmt fmt, uint64_t raw,            \
                  uint16_t cw, uint16_t &sw);                                  \
  /* sets C0/C2/C3; unordered = FUCOM rules (IE only for SNaN) */              \
  void compare(const Float80 &a, const Float80 &b, bool unordered,             \
               uint16_t &sw);                                                  \
  void compareMem(const Float80 &a, MemFmt fmt, uint64_t raw, uint16_t &sw);   \
  Float80 loadF32(uint32_t raw, uint16_t cw, uint16_t &sw);                    \
  Float80 loadF64(uint64_t raw, uint16_t cw, uint16_t &sw);                    \
  Float80 loadBCD(const uint8_t *bcd10);                                       \
  uint32_t storeF32(const Float80 &v, uint16_t cw, uint16_t &sw);              \
  uint64_t storeF64(const Float80 &v, uint16_t cw, uint16_t &sw);              \
  /* bits = 16/32/64; returns the integer indefinite on invalid */             \
  int64_t storeInt(const Float80 &v, int bits, uint16_t cw, uint16_t &sw);     \
  void storeBCD(const Float80 &v, uint8_t *bcd10, uint16_t cw, uint16_t &sw);  \
  /* D9 E0..FF register-stack group (FCHS..FCOS).  in[0] = ST0, in[1] =        \
     ST1 (when nIn == 2).  Returns the number of values left (out[0] = new    \
     ST0); e.g. FPATAN 2 -> 1, FPTAN 1 -> 2. */                                \
  int stackOp(uint8_t modrm, int nIn, const Float80 *in, Float80 *out,         \
              uint16_t cw, uint16_t &sw);

namespace soft {
FADOR_X87_BACKEND_API
Float80 loadConst(Const c, uint16_t cw);
} // namespace soft

#if FADOR_X87_HOST
namespace host {
FADOR_X87_BACKEND_API
// Restores the host control word the host back end may have left set.
void restoreHostControlWord();
} // namespace host
namespace active = host;
#else
namespace active = soft;
#endif

#undef FADOR_X87_BACKEND_API

// Number of stack inputs a D9 E0..FF operation reads (0 = not a stack op).
constexpr int stackOpInputs(uint8_t modrm) {
  switch (modrm) {
  case 0xE0: // FCHS
  case 0xE1: // FABS
  case 0xF0: // F2XM1
  case 0xF2: // FPTAN
  case 0xF4: // FXTRACT
  case 0xFA: // FSQRT
  case 0xFB: // FSINCOS
  case 0xFC: // FRNDINT
  case 0xFE: // FSIN
  case 0xFF: // FCOS
    return 1;
  case 0xF1: // FYL2X
  case 0xF3: // FPATAN
  case 0xF5: // FPREM1
  case 0xF8: // FPREM
  case 0xF9: // FYL2XP1
  case 0xFD: // FSCALE
    return 2;
  default:
    return 0;
  }
}

// Number of stack values a D9 E0..FF operation leaves on success.
constexpr int stackOpOutputs(uint8_t modrm) {
  switch (modrm) {
  case 0xF1:
  case 0xF3:
  case 0xF9:
    return 1;
  case 0xF2:
  case 0xF4:
  case 0xFB:
    return 2;
  default:
    return stackOpInputs(modrm);
  }
}

// Condition bits an operation defines (the others keep their old value).
// FPREM/FPREM1 with a NaN result (invalid operands) only clear C1 and C2.
constexpr uint16_t stackOpDefinedCC(uint8_t modrm, bool nanResult = false) {
  switch (modrm) {
  case 0xF5:
  case 0xF8:
    return nanResult ? static_cast<uint16_t>(SW_C1 | SW_C2)
                     : SW_CC; // quotient bits + C2
  case 0xF2:
  case 0xFB:
  case 0xFE:
  case 0xFF:
    return SW_C1 | SW_C2; // trig: C2 = operand out of range
  default:
    return SW_C1;
  }
}

} // namespace fador::cpu::x87
