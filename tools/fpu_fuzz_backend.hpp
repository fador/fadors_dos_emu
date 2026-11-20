#pragma once

// Value-level differential test of the soft x87 back end against the host
// x87 back end (used by tools/fpu_fuzz.cpp --backend).  Compares results and
// the defined status bits operation by operation with operands biased
// towards special values, rounding boundaries and exponent extremes.

#include "cpu/x87/X87Math.hpp"

#include <cinttypes>
#include <cstdio>
#include <map>
#include <random>
#include <string>

namespace fpu_fuzz {

using namespace fador::cpu::x87;

class ValueGen {
public:
  explicit ValueGen(uint64_t seed) : m_rng(seed) {}

  uint64_t r64() { return m_rng(); }
  uint32_t r32() { return static_cast<uint32_t>(m_rng()); }
  uint32_t below(uint32_t n) { return static_cast<uint32_t>(m_rng() % n); }

  uint64_t mantissa() {
    switch (below(8)) {
    case 0:
      return kIntegerBit; // power of two
    case 1:
      return ~0ull; // all ones
    case 2:
      return kIntegerBit | (r64() & 0xFFFFFFFFFF000000ull); // 40 bits
    case 3:
      return kIntegerBit | (r64() & 0xFFFFFF0000000000ull); // float-like
    case 4:
      return kIntegerBit | (r64() & 0xFFFFFFFFFFFFF800ull); // double-like
    case 5:
      return kIntegerBit | (1ull << below(63)) | (r64() & 0xFF); // tie-ish
    default:
      return kIntegerBit | r64();
    }
  }

  uint16_t exponent() {
    switch (below(10)) {
    case 0:
      return static_cast<uint16_t>(1 + below(70)); // near denormal range
    case 1:
      return static_cast<uint16_t>(0x7FFE - below(70)); // near overflow
    case 2:
      return static_cast<uint16_t>(16383 - 126 - 40 + below(80)); // float edge
    case 3:
      return static_cast<uint16_t>(16383 - 1022 - 70 + below(140)); // double edge
    case 4:
      return static_cast<uint16_t>(16383 + 60 + below(10)); // near 2^63
    default:
      return static_cast<uint16_t>(16383 - 40 + below(80));
    }
  }

  Float80 value() {
    const bool sign = below(2) != 0;
    switch (below(20)) {
    case 0:
      return Float80::make(sign, 0, 0);
    case 1: // denormal
      return Float80::make(sign, 0, r64() >> (1 + below(63)));
    case 2: // pseudo-denormal
      return Float80::make(sign, 0, kIntegerBit | (r64() >> 1));
    case 3:
      return Float80::make(sign, 0x7FFF, kIntegerBit);
    case 4: // QNaN
      return Float80::make(sign, 0x7FFF, 0xC000000000000000ull | (below(2) ? r64() >> 2 : 0));
    case 5: { // SNaN
      uint64_t m = kIntegerBit | ((r64() >> 2) | 1);
      return Float80::make(sign, 0x7FFF, m);
    }
    case 6: // unsupported: unnormal / pseudo-NaN / pseudo-inf
      if (below(2))
        return Float80::make(sign, static_cast<uint16_t>(1 + below(0x7FFE)), r64() >> 1);
      return Float80::make(sign, 0x7FFF, r64() >> 1);
    case 7: { // small integer
      int64_t v = static_cast<int64_t>(below(2000)) - 1000;
      return fromIntValue(v);
    }
    case 8: // x.5 values (rounding ties)
      return Float80::make(sign, static_cast<uint16_t>(16383 + below(20)),
                           kIntegerBit | (1ull << (62 - below(20))));
    default:
      return Float80::make(sign, exponent(), mantissa());
    }
  }

  static Float80 fromIntValue(int64_t v) {
    if (v == 0)
      return kPosZero;
    const bool s = v < 0;
    uint64_t m = s ? 0 - static_cast<uint64_t>(v) : static_cast<uint64_t>(v);
    int sh = 0;
    while (!(m >> 63)) {
      m <<= 1;
      ++sh;
    }
    return Float80::make(s, static_cast<uint16_t>(16383 + 63 - sh), m);
  }

  // A value near `v` (for cancellation / comparison of nearby numbers).
  Float80 near(const Float80 &v) {
    Float80 r = v;
    switch (below(4)) {
    case 0:
      r.mant ^= 1ull << below(64);
      break;
    case 1:
      r.signExp ^= 0x8000;
      break;
    case 2:
      r.signExp = static_cast<uint16_t>((r.signExp & 0x8000) | ((r.exp() + below(3) - 1) & 0x7FFF));
      break;
    default:
      break;
    }
    return r;
  }

  uint16_t controlWord(bool allowUnmasked) {
    uint16_t cw = 0x0040; // reserved bit 6 is set on real hardware
    static const uint16_t pcs[4] = {0, 2, 3, 3};
    cw |= static_cast<uint16_t>(pcs[below(4)] << 8);
    cw |= static_cast<uint16_t>(below(4) << 10);
    uint16_t masks = 0x3F;
    if (allowUnmasked && below(4) == 0)
      masks = static_cast<uint16_t>(r32() & 0x3F);
    return static_cast<uint16_t>(cw | masks);
  }

  uint64_t memRaw(MemFmt fmt) {
    switch (fmt) {
    case MemFmt::F32: {
      switch (below(8)) {
      case 0:
        return r32() & 0x807FFFFFu; // denormal/zero
      case 1:
        return (r32() & 0x807FFFFFu) | 0x7F800000u; // inf/NaN
      default:
        return r32();
      }
    }
    case MemFmt::F64: {
      switch (below(8)) {
      case 0:
        return r64() & 0x800FFFFFFFFFFFFFull;
      case 1:
        return (r64() & 0x800FFFFFFFFFFFFFull) | 0x7FF0000000000000ull;
      default: {
        // exponent biased towards the middle
        uint64_t v = r64();
        if (below(2))
          v = (v & 0x800FFFFFFFFFFFFFull) |
              (static_cast<uint64_t>(1023 - 60 + below(120)) << 52);
        return v;
      }
      }
    }
    case MemFmt::I32:
      return below(2) ? r32() : static_cast<uint32_t>(static_cast<int32_t>(below(2000)) - 1000);
    default:
      return below(2) ? (r32() & 0xFFFF) : static_cast<uint16_t>(static_cast<int16_t>(below(200)) - 100);
    }
  }

private:
  std::mt19937_64 m_rng;
};

struct BackendStats {
  std::map<std::string, std::pair<uint64_t, uint64_t>> perOp; // runs, fails
  std::map<std::string, std::string> firstFailure;
  uint64_t total = 0, failures = 0;

  void record(const std::string &name, bool ok, const std::string &detail) {
    auto &s = perOp[name];
    ++s.first;
    ++total;
    if (!ok) {
      ++s.second;
      ++failures;
      if (!firstFailure.count(name))
        firstFailure[name] = detail;
    }
  }
};

inline std::string hex80(const Float80 &v) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%04X:%016" PRIX64, v.signExp, v.mant);
  return buf;
}

// Transcendental results from the soft back end are approximations: accept
// them if they are within `ulps` units in the last place of the host's.
inline bool closeEnough(const Float80 &a, const Float80 &b, uint64_t ulps) {
  if (a == b)
    return true;
  if (a.sign() != b.sign())
    return isZero(a) && isZero(b);
  if (isNaN(a) || isNaN(b) || isInf(a) || isInf(b))
    return false;
  // Compare as integers on a common exponent scale.
  const int ea = a.exp() == 0 ? 1 : a.exp();
  const int eb = b.exp() == 0 ? 1 : b.exp();
  if (ea != eb) {
    if (ea - eb == 1 || eb - ea == 1) {
      const Float80 &hi = ea > eb ? a : b;
      const Float80 &lo = ea > eb ? b : a;
      // hi.mant near 2^63 and lo.mant near 2^64
      const uint64_t lhs = hi.mant - kIntegerBit;     // hi above 2^e
      const uint64_t rhs = (~lo.mant) + 1;            // lo below 2^e (x2 scale)
      return lhs / 1 + rhs / 2 <= ulps;
    }
    return false;
  }
  const uint64_t d = a.mant > b.mant ? a.mant - b.mant : b.mant - a.mant;
  return d <= ulps;
}

} // namespace fpu_fuzz
