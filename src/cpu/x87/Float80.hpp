#pragma once

// 80-bit x87 double extended-precision value and helpers shared by the x87
// FPU model and its arithmetic back ends.
//
// Layout matches the in-memory m80fp format: a 64-bit significand with an
// explicit integer bit (J, bit 63) followed by a 16-bit sign/exponent word.
// The struct is 16 bytes (padded), so the first 10 bytes can be loaded by
// the host's FLD TBYTE directly.

#include <cstdint>
#include <cstring>

namespace fador::cpu::x87 {

struct Float80 {
  uint64_t mant = 0;     // significand, bit 63 = explicit integer bit
  uint16_t signExp = 0;  // bit 15 = sign, bits 0..14 = biased exponent
  uint16_t pad[3] = {0, 0, 0};

  constexpr Float80() = default;
  constexpr Float80(uint64_t m, uint16_t se) : mant(m), signExp(se) {}

  constexpr bool sign() const { return (signExp & 0x8000u) != 0; }
  constexpr uint16_t exp() const { return static_cast<uint16_t>(signExp & 0x7FFFu); }
  constexpr bool integerBit() const { return (mant >> 63) != 0; }

  constexpr bool operator==(const Float80 &o) const {
    return mant == o.mant && signExp == o.signExp;
  }
  constexpr bool operator!=(const Float80 &o) const { return !(*this == o); }

  static constexpr Float80 make(bool sign, uint16_t exp, uint64_t mant) {
    return Float80(mant, static_cast<uint16_t>((sign ? 0x8000u : 0u) | (exp & 0x7FFFu)));
  }
};
static_assert(sizeof(Float80) == 16);

inline constexpr uint64_t kIntegerBit = 0x8000000000000000ull;
inline constexpr uint64_t kQuietBit = 0x4000000000000000ull;
inline constexpr uint16_t kExpBias = 16383;
inline constexpr uint16_t kExpMax = 0x7FFF;

inline constexpr Float80 kPosZero{0, 0};
inline constexpr Float80 kNegZero{0, 0x8000};
inline constexpr Float80 kPosInf{kIntegerBit, 0x7FFF};
inline constexpr Float80 kNegInf{kIntegerBit, 0xFFFF};
// "Real indefinite": the default QNaN produced by masked invalid operations.
inline constexpr Float80 kIndefinite{0xC000000000000000ull, 0xFFFF};
inline constexpr Float80 kOne{kIntegerBit, 0x3FFF};

enum class Class : uint8_t {
  Zero,
  Denormal,     // includes pseudo-denormals (exp 0, J=1)
  Normal,
  Infinity,
  QNaN,
  SNaN,
  Unsupported,  // unnormals, pseudo-NaNs, pseudo-infinities
};

constexpr Class classify(const Float80 &v) {
  const uint16_t e = v.exp();
  if (e == 0) {
    if (v.mant == 0)
      return Class::Zero;
    return Class::Denormal;
  }
  if (!v.integerBit())
    return Class::Unsupported;
  if (e == kExpMax) {
    if ((v.mant << 1) == 0)
      return Class::Infinity;
    return (v.mant & kQuietBit) ? Class::QNaN : Class::SNaN;
  }
  return Class::Normal;
}

constexpr bool isNaN(const Float80 &v) {
  return v.exp() == kExpMax && v.integerBit() && (v.mant << 1) != 0;
}
constexpr bool isSNaN(const Float80 &v) {
  return isNaN(v) && (v.mant & kQuietBit) == 0;
}
constexpr bool isZero(const Float80 &v) { return v.exp() == 0 && v.mant == 0; }
constexpr bool isInf(const Float80 &v) {
  return v.exp() == kExpMax && v.mant == kIntegerBit;
}
constexpr bool isUnsupported(const Float80 &v) {
  return v.exp() != 0 && !v.integerBit();
}
constexpr bool isDenormal(const Float80 &v) {
  return v.exp() == 0 && v.mant != 0;
}

// FPU tag for a non-empty register (FSTENV/FSAVE compute these from the
// register contents): 0 = valid, 1 = zero, 2 = special.
constexpr uint8_t tagFor(const Float80 &v) {
  const uint16_t e = v.exp();
  if (e == 0)
    return v.mant == 0 ? 1 : 2;
  if (e == kExpMax || !v.integerBit())
    return 2;
  return 0;
}

inline Float80 loadFloat80(const uint8_t *bytes) {
  Float80 v;
  std::memcpy(&v.mant, bytes, 8);
  std::memcpy(&v.signExp, bytes + 8, 2);
  return v;
}

inline void storeFloat80(uint8_t *bytes, const Float80 &v) {
  std::memcpy(bytes, &v.mant, 8);
  std::memcpy(bytes + 8, &v.signExp, 2);
}

// Exact conversions used by the double-based convenience accessors
// (debugger, tests).  fromDouble is exact; toDouble rounds to nearest-even.
Float80 fromDouble(double value);
double toDouble(const Float80 &value);

} // namespace fador::cpu::x87
