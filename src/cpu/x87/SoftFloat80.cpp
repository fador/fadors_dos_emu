// Portable software implementation of the x87 value operations (see
// X87Math.hpp).  Basic arithmetic, square root, conversions, FRNDINT,
// FSCALE, FXTRACT and FPREM/FPREM1 are bit-exact: they round once from an
// exact (or sticky-extended) intermediate in all four rounding modes, honour
// precision control, denormalise tiny results (tininess is detected after
// rounding, as on x86), rebias results for unmasked overflow/underflow, and
// follow the x87 NaN propagation and invalid-operand rules.  Transcendental
// functions (F2XM1, FYL2X, FYL2XP1, FPTAN, FPATAN, FSIN, FCOS, FSINCOS) use
// the host's long double (or double) maths library with the x87 special-value
// and domain rules applied on top, so their last bits can differ from a real
// x87.

#include "X87Math.hpp"
#include "U128.hpp"

#include <bit>
#include <cfloat>
#include <cmath>
#include <cstring>

namespace fador::cpu::x87 {

namespace {

constexpr int32_t kBias = 16383;

enum Rounding : int { RN = 0, RD = 1, RU = 2, RZ = 3 };

inline int roundingOf(uint16_t cw) { return (cw >> CW_RC_SHIFT) & 3; }

inline int precisionOf(uint16_t cw) {
  switch ((cw >> CW_PC_SHIFT) & 3) {
  case 0:
    return 24;
  case 2:
    return 53;
  default:
    return 64; // 3 = extended; 1 is reserved and behaves as extended
  }
}

inline bool masked(uint16_t cw, uint16_t flag) { return (cw & flag) != 0; }

inline Float80 pack(bool sign, int32_t exp, uint64_t mant) {
  return Float80::make(sign, static_cast<uint16_t>(exp), mant);
}

inline Float80 quietNaN(Float80 v) {
  v.mant |= kQuietBit;
  return v;
}

// x87 NaN propagation (SDM Table 4-7): an SNaN raises IE; a QNaN operand is
// preferred over an SNaN; between two NaNs of the same kind the larger
// significand wins, ties go to the positive one.
Float80 propagateNaN(const Float80 &a, const Float80 &b, uint16_t &sw) {
  const bool aN = isNaN(a), bN = isNaN(b);
  const bool aS = aN && isSNaN(a), bS = bN && isSNaN(b);
  if (aS || bS)
    sw |= SW_IE;
  if (aN && bN) {
    if (aS != bS)
      return aS ? quietNaN(b) : quietNaN(a);
    if (a.mant != b.mant)
      return quietNaN(a.mant > b.mant ? a : b);
    return quietNaN(a.signExp < b.signExp ? a : b);
  }
  return quietNaN(aN ? a : b);
}

Float80 propagateNaN1(const Float80 &a, uint16_t &sw) {
  if (isSNaN(a))
    sw |= SW_IE;
  return quietNaN(a);
}

// Reports a denormal operand.  Returns true if the denormal-operand
// exception is unmasked: the x87 then stops before computing anything.
inline bool denormalStop(bool denormal, uint16_t cw, uint16_t &sw) {
  if (!denormal)
    return false;
  sw |= SW_DE;
  return !masked(cw, SW_DE);
}

Float80 invalid(uint16_t &sw) {
  sw |= SW_IE;
  return kIndefinite;
}

// Finite non-zero value unpacked to a normalised 64-bit significand:
// value = sig * 2^(exp - kBias - 63), bit 63 of sig set.  exp may be < 1.
struct Unpacked {
  bool sign;
  int32_t exp;
  uint64_t sig;
};

inline Unpacked unpack(const Float80 &v) {
  int32_t e = v.exp();
  if (e == 0)
    e = 1; // denormals and pseudo-denormals share exponent 1
  const int shift = std::countl_zero(v.mant);
  return {v.sign(), e - shift, v.mant << shift};
}

// Drops `drop` low bits of v with rounding.  Sets inexact / roundedUp.
U128 roundBits(U128 v, int drop, int rc, bool sign, bool &inexact, bool &up) {
  inexact = false;
  up = false;
  if (drop <= 0)
    return v;
  U128 kept, rem, half;
  if (drop >= 128) {
    kept = {0, 0};
    rem = v;
    half = {0, 0}; // half > rem always (2^(drop-1) >= 2^127 > v only if drop>128)
  } else {
    kept = shr128(v, drop);
    rem = sub128(v, shl128(kept, drop));
    half = shl128(U128{0, 1}, drop - 1);
  }
  inexact = !rem.isZero();
  if (!inexact)
    return kept;
  bool inc = false;
  switch (rc) {
  case RN:
    if (drop > 128) {
      inc = false;
    } else if (drop == 128) {
      // half = 2^127
      const U128 h{0x8000000000000000ull, 0};
      inc = rem > h || (rem == h && (kept.lo & 1));
    } else {
      inc = rem > half || (rem == half && (kept.lo & 1));
    }
    break;
  case RD:
    inc = sign;
    break;
  case RU:
    inc = !sign;
    break;
  default:
    break;
  }
  if (inc) {
    kept = add128(kept, U128{0, 1});
    up = true;
  }
  return kept;
}

// Result of rounding to a destination format, in extended-biased terms.
struct Rounded {
  bool sign;
  int32_t exp;   // biased extended exponent (normal), or 0 = zero/denormal
  uint64_t mant; // 64-bit significand (explicit J bit for normals)
  bool inf;
};

// Rounds sig128 (bit 127 set) * 2^(exp - kBias - 127) to `prec` significand
// bits for a format with normal exponents eMin..eMax (extended-biased) and
// `fmtPrec` significand bits.  Applies the x87 overflow/underflow rules.
// rebias = whether unmasked OE/UE produce a rebiased result (register
// destinations); stores to memory report the exception and the caller does
// not store.
Rounded roundPackGeneric(bool sign, int32_t exp, U128 sig, int prec,
                         int fmtPrec, int32_t eMin, int32_t eMax,
                         int32_t rebiasAmount, uint16_t cw, uint16_t &sw) {
  const int rc = roundingOf(cw);
  bool inexact = false, up = false;
  U128 r = roundBits(sig, 128 - prec, rc, sign, inexact, up);
  int32_t e = exp;
  const bool carry = prec == 64 ? r.hi != 0 : (r.lo >> prec) != 0;
  if (carry) { // rounded up to 2^prec
    r = {0, 1ull << (prec - 1)};
    ++e;
  }
  uint64_t m = r.lo << (64 - prec);

  if (e > eMax) {
    if (!masked(cw, SW_OE) && rebiasAmount == 0) {
      sw |= SW_OE; // memory destination: nothing is stored
      return {sign, e, m, false};
    }
    if (!masked(cw, SW_OE) && e - rebiasAmount > eMax) {
      // Beyond the rebias range: infinity, whatever the rounding mode.
      sw |= SW_OE | SW_PE | SW_C1;
      return {sign, 0, 0, true};
    }
    if (!masked(cw, SW_OE)) {
      sw |= SW_OE;
      if (inexact)
        sw |= SW_PE;
      if (up)
        sw |= SW_C1;
      return {sign, e - rebiasAmount, m, false};
    }
    sw |= SW_OE | SW_PE;
    const bool toInf = rc == RN || (rc == RU && !sign) || (rc == RD && sign);
    if (toInf) {
      sw |= SW_C1;
      return {sign, 0, 0, true};
    }
    const uint64_t maxMant = ~0ull << (64 - prec);
    return {sign, eMax, maxMant, false};
  }

  if (e < eMin) { // tiny (detected after rounding)
    if (!masked(cw, SW_UE) && rebiasAmount == 0) {
      sw |= SW_UE; // memory destination: nothing is stored
      return {sign, e, m, false};
    }
    if (!masked(cw, SW_UE) && e + rebiasAmount < eMin) {
      // Beyond the rebias range: a zero.
      sw |= SW_UE | SW_PE;
      return {sign, 0, 0, false};
    }
    if (!masked(cw, SW_UE)) {
      sw |= SW_UE;
      if (inexact)
        sw |= SW_PE;
      if (up)
        sw |= SW_C1;
      return {sign, e + rebiasAmount, m, false};
    }
    // Denormalise from the unrounded value.
    const int32_t shift = eMin - exp; // >= 1
    // Precision control rounds at a fixed bit of the stored significand,
    // so a denormalised result keeps fewer than `prec` significant bits.
    int32_t lsbPos = 64 - prec;
    if (lsbPos < 64 - fmtPrec)
      lsbPos = 64 - fmtPrec;
    const int64_t drop = 64 + static_cast<int64_t>(shift) + lsbPos;
    bool inex2 = false, up2 = false;
    U128 d = roundBits(sig, drop > 1000 ? 1000 : static_cast<int>(drop), rc,
                       sign, inex2, up2);
    uint64_t dm = d.lo << lsbPos;
    if (inex2)
      sw |= SW_PE | SW_UE;
    if (!masked(cw, SW_UE))
      sw |= SW_UE; // beyond the rebias range
    if (up2)
      sw |= SW_C1;
    if (dm >> 63)
      return {sign, eMin, dm, false}; // rounded up to the smallest normal
    return {sign, 0, dm, false};
  }

  if (inexact)
    sw |= SW_PE;
  if (up)
    sw |= SW_C1;
  return {sign, e, m, false};
}

inline Float80 toExtended(const Rounded &r) {
  if (r.inf)
    return pack(r.sign, 0x7FFF, kIntegerBit);
  if (r.mant == 0)
    return pack(r.sign, 0, 0);
  if (r.exp == 0)
    return pack(r.sign, 0, r.mant);
  return pack(r.sign, r.exp & 0x7FFF, r.mant);
}

constexpr int32_t kRebias = 24576;

// Round to the extended format with precision `prec`.
inline Float80 roundPack(bool sign, int32_t exp, U128 sig, int prec,
                         uint16_t cw, uint16_t &sw) {
  return toExtended(
      roundPackGeneric(sign, exp, sig, prec, 64, 1, 0x7FFE, kRebias, cw, sw));
}

// Normalises sig128 (non-zero) so that bit 127 is set and rounds.
inline Float80 normRoundPack(bool sign, int32_t exp, U128 sig, int prec,
                             uint16_t cw, uint16_t &sw) {
  const int s = clz128(sig);
  return roundPack(sign, exp - s, shl128(sig, s), prec, cw, sw);
}

// ---------------------------------------------------------------------------
// Memory-format conversions to extended (exact).  SNaNs stay signalling so
// the arithmetic NaN rules see the original operand; `denormal` reports a
// denormal source.

Float80 fromF32Raw(uint32_t raw, bool &denormal) {
  denormal = false;
  const bool sign = (raw >> 31) != 0;
  const uint32_t e = (raw >> 23) & 0xFF;
  const uint32_t f = raw & 0x7FFFFF;
  if (e == 0xFF)
    return pack(sign, 0x7FFF, kIntegerBit | (static_cast<uint64_t>(f) << 40));
  if (e == 0) {
    if (f == 0)
      return pack(sign, 0, 0);
    denormal = true;
    const int s = std::countl_zero(static_cast<uint64_t>(f));
    // value = f * 2^-149 ; normalised: sig = f << s, exp = ...
    const uint64_t sig = static_cast<uint64_t>(f) << s;
    const int32_t exp = kBias - 149 + 63 - s;
    return pack(sign, exp, sig);
  }
  return pack(sign, static_cast<int32_t>(e) - 127 + kBias,
              kIntegerBit | (static_cast<uint64_t>(f) << 40));
}

Float80 fromF64Raw(uint64_t raw, bool &denormal) {
  denormal = false;
  const bool sign = (raw >> 63) != 0;
  const uint32_t e = static_cast<uint32_t>((raw >> 52) & 0x7FF);
  const uint64_t f = raw & 0xFFFFFFFFFFFFFull;
  if (e == 0x7FF)
    return pack(sign, 0x7FFF, kIntegerBit | (f << 11));
  if (e == 0) {
    if (f == 0)
      return pack(sign, 0, 0);
    denormal = true;
    const int s = std::countl_zero(f);
    return pack(sign, kBias - 1074 + 63 - s, f << s);
  }
  return pack(sign, static_cast<int32_t>(e) - 1023 + kBias, kIntegerBit | (f << 11));
}

Float80 fromInt(int64_t v) {
  if (v == 0)
    return kPosZero;
  const bool sign = v < 0;
  const uint64_t mag = sign ? 0 - static_cast<uint64_t>(v) : static_cast<uint64_t>(v);
  const int s = std::countl_zero(mag);
  return pack(sign, kBias + 63 - s, mag << s);
}

Float80 memOperand(MemFmt fmt, uint64_t raw, bool &denormal) {
  denormal = false;
  switch (fmt) {
  case MemFmt::F32:
    return fromF32Raw(static_cast<uint32_t>(raw), denormal);
  case MemFmt::F64:
    return fromF64Raw(raw, denormal);
  case MemFmt::I32:
    return fromInt(static_cast<int32_t>(static_cast<uint32_t>(raw)));
  default:
    return fromInt(static_cast<int16_t>(static_cast<uint16_t>(raw)));
  }
}

// ---------------------------------------------------------------------------
// Arithmetic

Float80 addSub(const Float80 &a, const Float80 &b, bool negateB, bool aDenormal,
               bool bDenormal, uint16_t cw, uint16_t &sw) {
  if (isUnsupported(a) || isUnsupported(b))
    return invalid(sw);
  if (isNaN(a) || isNaN(b))
    return propagateNaN(a, b, sw);
  const bool aSign = a.sign();
  const bool bSign = b.sign() != negateB;
  const bool denormal =
      isDenormal(a) || isDenormal(b) || aDenormal || bDenormal;
  if (isInf(a) || isInf(b)) {
    if (isInf(a) && isInf(b) && aSign != bSign)
      return invalid(sw);
    if (denormalStop(denormal, cw, sw))
      return a;
    return isInf(a) ? pack(aSign, 0x7FFF, kIntegerBit)
                    : pack(bSign, 0x7FFF, kIntegerBit);
  }
  if (denormalStop(denormal, cw, sw))
    return a;
  const int prec = precisionOf(cw);
  const bool aZero = isZero(a), bZero = isZero(b);
  if (aZero && bZero) {
    if (aSign == bSign)
      return pack(aSign, 0, 0);
    return pack(roundingOf(cw) == RD, 0, 0);
  }
  if (aZero || bZero) {
    const Unpacked u = unpack(aZero ? b : a);
    return roundPack(aZero ? bSign : aSign, u.exp, U128{u.sig, 0}, prec, cw, sw);
  }
  Unpacked ua = unpack(a), ub = unpack(b);
  ua.sign = aSign;
  ub.sign = bSign;
  if (ua.exp < ub.exp || (ua.exp == ub.exp && ua.sig < ub.sig))
    std::swap(ua, ub);
  const int32_t d = ua.exp - ub.exp;
  const U128 A{ua.sig, 0};
  const U128 B = shr128Jam(U128{ub.sig, 0}, d > 200 ? 200 : static_cast<int>(d));
  if (ua.sign == ub.sign) {
    U128 s = add128(A, B);
    int32_t e = ua.exp;
    if (s < A) { // carry out of bit 127
      s = shr128Jam(s, 1);
      s.hi |= 0x8000000000000000ull;
      ++e;
    }
    return roundPack(ua.sign, e, s, prec, cw, sw);
  }
  const U128 diff = sub128(A, B);
  if (diff.isZero())
    return pack(roundingOf(cw) == RD, 0, 0);
  return normRoundPack(ua.sign, ua.exp, diff, prec, cw, sw);
}

Float80 mul(const Float80 &a, const Float80 &b, bool bDenormal, uint16_t cw,
            uint16_t &sw) {
  if (isUnsupported(a) || isUnsupported(b))
    return invalid(sw);
  if (isNaN(a) || isNaN(b))
    return propagateNaN(a, b, sw);
  const bool sign = a.sign() != b.sign();
  const bool denormal = isDenormal(a) || isDenormal(b) || bDenormal;
  if (isInf(a) || isInf(b)) {
    if (isZero(a) || isZero(b))
      return invalid(sw);
    if (denormalStop(denormal, cw, sw))
      return a;
    return pack(sign, 0x7FFF, kIntegerBit);
  }
  if (denormalStop(denormal, cw, sw))
    return a;
  if (isZero(a) || isZero(b))
    return pack(sign, 0, 0);
  const Unpacked ua = unpack(a), ub = unpack(b);
  U128 p = mul64x64(ua.sig, ub.sig);
  int32_t e = ua.exp + ub.exp - kBias + 1;
  if (!(p.hi >> 63)) {
    p = shl128(p, 1);
    --e;
  }
  return roundPack(sign, e, p, precisionOf(cw), cw, sw);
}

Float80 div(const Float80 &a, const Float80 &b, bool aDenormal, bool bDenormal,
            uint16_t cw, uint16_t &sw) {
  if (isUnsupported(a) || isUnsupported(b))
    return invalid(sw);
  if (isNaN(a) || isNaN(b))
    return propagateNaN(a, b, sw);
  const bool sign = a.sign() != b.sign();
  const bool aDen = isDenormal(a) || aDenormal;
  const bool bDen = isDenormal(b) || bDenormal;
  if (isInf(a)) {
    if (isInf(b))
      return invalid(sw);
    if (denormalStop(bDen, cw, sw))
      return a;
    return pack(sign, 0x7FFF, kIntegerBit);
  }
  if (isInf(b)) {
    if (denormalStop(aDen, cw, sw))
      return a;
    return pack(sign, 0, 0);
  }
  if (isZero(b)) {
    if (isZero(a))
      return invalid(sw);
    sw |= SW_ZE; // takes priority over a denormal dividend
    return pack(sign, 0x7FFF, kIntegerBit);
  }
  if (denormalStop(aDen || bDen, cw, sw))
    return a;
  if (isZero(a))
    return pack(sign, 0, 0);
  const Unpacked ua = unpack(a), ub = unpack(b);
  uint64_t hi, lo;
  int32_t e;
  if (ua.sig >= ub.sig) {
    hi = ua.sig >> 1;
    lo = ua.sig << 63;
    e = ua.exp - ub.exp + kBias;
  } else {
    hi = ua.sig;
    lo = 0;
    e = ua.exp - ub.exp + kBias - 1;
  }
  uint64_t r1 = 0, r2 = 0;
  const uint64_t q1 = div128by64(hi, lo, ub.sig, r1);
  const uint64_t q2 = div128by64(r1, 0, ub.sig, r2);
  U128 q{q1, q2};
  if (r2 != 0)
    q.lo |= 1;
  return roundPack(sign, e, q, precisionOf(cw), cw, sw);
}

// Integer square root of a 128-bit value: floor(sqrt(x)) and remainder.
uint64_t isqrt128(U128 x, U128 &remainder) {
  U128 rem{0, 0};
  uint64_t root = 0;
  for (int i = 0; i < 64; ++i) {
    rem = shl128(rem, 2);
    rem.lo |= x.hi >> 62;
    x = shl128(x, 2);
    root <<= 1;
    const U128 test = add128(shl128(U128{0, root}, 1), U128{0, 1});
    if (rem >= test) {
      rem = sub128(rem, test);
      root |= 1;
    }
  }
  remainder = rem;
  return root;
}

Float80 sqrtOp(const Float80 &a, uint16_t cw, uint16_t &sw) {
  if (isUnsupported(a))
    return invalid(sw);
  if (isNaN(a))
    return propagateNaN1(a, sw);
  if (isZero(a))
    return a;
  if (a.sign())
    return invalid(sw);
  if (isInf(a))
    return a;
  if (denormalStop(isDenormal(a), cw, sw))
    return a;
  const Unpacked u = unpack(a);
  const int32_t k = u.exp - kBias - 63;
  U128 x;
  int32_t e;
  if ((k & 1) == 0) {
    x = {u.sig, 0};
    e = (k - 64) / 2 + kBias + 63;
  } else {
    x = {u.sig >> 1, u.sig << 63};
    e = (k - 63) / 2 + kBias + 63;
  }
  U128 rem;
  const uint64_t r = isqrt128(x, rem);
  uint64_t frac = 0;
  if (rem > U128{0, r})
    frac |= 0x8000000000000000ull;
  if (!rem.isZero())
    frac |= 1;
  return roundPack(false, e, U128{r, frac}, precisionOf(cw), cw, sw);
}

// FRNDINT: round to an integral value with RC (always 64-bit precision).
Float80 roundToInt(const Float80 &a, uint16_t cw, uint16_t &sw) {
  if (isUnsupported(a))
    return invalid(sw);
  if (isNaN(a))
    return propagateNaN1(a, sw);
  if (isZero(a) || isInf(a))
    return a;
  if (denormalStop(isDenormal(a), cw, sw))
    return a;
  const Unpacked u = unpack(a);
  const int32_t fracBits = kBias + 63 - u.exp;
  if (fracBits <= 0)
    return a;
  bool inexact = false, up = false;
  const U128 r = roundBits(U128{0, u.sig}, fracBits > 200 ? 200 : fracBits,
                           roundingOf(cw), u.sign, inexact, up);
  if (inexact)
    sw |= SW_PE;
  if (up)
    sw |= SW_C1;
  if (r.isZero())
    return pack(u.sign, 0, 0);
  // r <= 2^64 - 1 + 1: the carry case (2^64) only happens for fracBits == 0.
  const int s = std::countl_zero(r.lo);
  return pack(u.sign, kBias + 63 - s, r.lo << s);
}

// Exact rounding of a value to an integer for FIST/FBSTP.  Returns false if
// the magnitude is >= 2^64 after rounding.
bool roundToInteger(const Float80 &a, uint16_t cw, uint64_t &mag, bool &inexact,
                    bool &up) {
  inexact = false;
  up = false;
  if (isZero(a)) {
    mag = 0;
    return true;
  }
  const Unpacked u = unpack(a);
  const int32_t fracBits = kBias + 63 - u.exp;
  if (fracBits < 0)
    return false;
  const U128 r = roundBits(U128{0, u.sig}, fracBits > 200 ? 200 : fracBits,
                           roundingOf(cw), u.sign, inexact, up);
  if (r.hi != 0)
    return false;
  mag = r.lo;
  return true;
}

int64_t storeIntImpl(const Float80 &a, int bits, uint16_t cw, uint16_t &sw) {
  const int64_t indefinite = bits == 16   ? static_cast<int64_t>(-32768)
                             : bits == 32 ? static_cast<int64_t>(INT32_MIN)
                                          : INT64_MIN;
  if (isUnsupported(a) || isNaN(a) || isInf(a)) {
    sw |= SW_IE;
    return indefinite;
  }
  uint64_t mag = 0;
  bool inexact = false, up = false;
  if (!roundToInteger(a, cw, mag, inexact, up)) {
    sw |= SW_IE;
    return indefinite;
  }
  const uint64_t limit = 1ull << (bits - 1);
  if (a.sign() ? mag > limit : mag >= limit) {
    sw |= SW_IE;
    return indefinite;
  }
  if (inexact)
    sw |= SW_PE;
  if (up)
    sw |= SW_C1;
  return a.sign() ? static_cast<int64_t>(0 - mag) : static_cast<int64_t>(mag);
}

// Store to a binary32/binary64 memory format.
template <int FracBits, int ExpBits>
uint64_t storeBinary(const Float80 &a, uint16_t cw, uint16_t &sw) {
  constexpr int prec = FracBits + 1;
  constexpr int32_t fmtBias = (1 << (ExpBits - 1)) - 1;
  constexpr uint64_t expMax = (1ull << ExpBits) - 1;
  const uint64_t signBit = a.sign() ? 1ull << (FracBits + ExpBits) : 0;
  auto encode = [&](uint64_t e, uint64_t f) {
    return signBit | (e << FracBits) | f;
  };
  if (isUnsupported(a)) {
    sw |= SW_IE;
    return (1ull << (FracBits + ExpBits)) | encode(expMax, 1ull << (FracBits - 1));
  }
  if (isNaN(a)) {
    if (isSNaN(a))
      sw |= SW_IE;
    const uint64_t f = ((a.mant | kQuietBit) << 1) >> (64 - FracBits);
    return encode(expMax, f);
  }
  if (isInf(a))
    return encode(expMax, 0);
  if (isZero(a))
    return signBit;
  const Unpacked u = unpack(a);
  const int32_t eMin = kBias - fmtBias + 1;
  const int32_t eMax = kBias + fmtBias;
  const Rounded r = roundPackGeneric(u.sign, u.exp, U128{u.sig, 0}, prec, prec,
                                     eMin, eMax, 0, cw, sw);
  if (r.inf)
    return encode(expMax, 0);
  const uint64_t frac = (r.mant << 1) >> (64 - FracBits);
  if (r.exp == 0)
    return encode(0, r.mant >> (64 - prec)); // denormal (or zero)
  int64_t e = static_cast<int64_t>(r.exp) - kBias + fmtBias;
  // Unmasked over/underflow: the exponent is out of range and the caller
  // does not store; produce something deterministic anyway.
  if (e <= 0 || e >= static_cast<int64_t>(expMax))
    e = static_cast<int64_t>(static_cast<uint64_t>(e) & expMax);
  return encode(static_cast<uint64_t>(e), frac);
}

// ---------------------------------------------------------------------------
// FSCALE, FXTRACT, FPREM, FPREM1

Float80 scaleOp(const Float80 &a, const Float80 &b, uint16_t cw, uint16_t &sw) {
  if (isUnsupported(a) || isUnsupported(b))
    return invalid(sw);
  if (isNaN(a) || isNaN(b))
    return propagateNaN(a, b, sw);
  const bool aDen = isDenormal(a), bDen = isDenormal(b);
  if (isInf(b)) {
    if (b.sign()) { // scale by -inf
      if (isInf(a))
        return invalid(sw);
      if (denormalStop(aDen, cw, sw))
        return a;
      return pack(a.sign(), 0, 0);
    }
    if (isZero(a))
      return invalid(sw);
    if (denormalStop(aDen, cw, sw))
      return a;
    return pack(a.sign(), 0x7FFF, kIntegerBit);
  }
  if (denormalStop(aDen || bDen, cw, sw))
    return a;
  if (isZero(a) || isInf(a))
    return a;
  if (isZero(b)) { // scale by zero: unchanged
    if (a.exp() == 0 && a.integerBit())
      return pack(a.sign(), 1, a.mant);
    return a;
  }
  // n = trunc(b), clamped (anything beyond +-2^17 over/underflows anyway).
  int64_t n = 0;
  if (!isZero(b)) {
    const Unpacked ub = unpack(b);
    const int32_t unb = ub.exp - kBias; // value in [2^unb, 2^(unb+1))
    if (unb >= 20) {
      n = 1 << 20;
    } else if (unb >= 0) {
      n = static_cast<int64_t>(ub.sig >> (63 - unb));
    }
    if (ub.sign)
      n = -n;
  }
  const Unpacked ua = unpack(a);
  return roundPack(ua.sign, static_cast<int32_t>(ua.exp + n), U128{ua.sig, 0},
                   64, cw, sw);
}

int xtractOp(const Float80 &a, Float80 *out, uint16_t cw, uint16_t &sw) {
  if (isUnsupported(a)) {
    sw |= SW_IE;
    out[0] = kIndefinite;
    out[1] = kIndefinite;
    return 2;
  }
  if (isNaN(a)) {
    const Float80 q = propagateNaN1(a, sw);
    out[0] = q;
    out[1] = q;
    return 2;
  }
  if (isZero(a)) {
    sw |= SW_ZE;
    out[0] = a;
    out[1] = kNegInf;
    return 2;
  }
  if (isInf(a)) {
    out[0] = a;
    out[1] = kPosInf;
    return 2;
  }
  if (denormalStop(isDenormal(a), cw, sw)) {
    out[0] = a;
    return 1;
  }
  const Unpacked u = unpack(a);
  out[0] = pack(u.sign, kBias, u.sig);
  out[1] = fromInt(static_cast<int64_t>(u.exp) - kBias);
  return 2;
}

Float80 premOp(const Float80 &a, const Float80 &b, bool ieee, uint16_t cw,
               uint16_t &sw) {
  if (isUnsupported(a) || isUnsupported(b))
    return invalid(sw);
  if (isNaN(a) || isNaN(b))
    return propagateNaN(a, b, sw);
  if (isInf(a) || isZero(b))
    return invalid(sw);
  if (denormalStop(isDenormal(a) || isDenormal(b), cw, sw))
    return a;
  if (isZero(a))
    return a; // quotient 0: C0/C1/C3 = 0, C2 = 0
  if (isInf(b)) { // the dividend is returned (pseudo-denormals normalised)
    if (a.exp() == 0 && a.integerBit())
      return pack(a.sign(), 1, a.mant);
    return a;
  }
  const Unpacked ua = unpack(a), ub = unpack(b);
  const int32_t d = ua.exp - ub.exp;
  if (d >= 64) {
    // Partial remainder: reduce the exponent by n in [32, 63].
    const int n = static_cast<int>((d & 31) | 32);
    uint64_t r = 0;
    // (ua.sig * 2^n) mod ub.sig
    const uint64_t hi0 = ua.sig >> (64 - n);
    const uint64_t lo0 = ua.sig << n;
    const uint64_t hi = hi0 % ub.sig;
    const uint64_t q = div128by64(hi, lo0, ub.sig, r);
    (void)q;
    sw |= SW_C2; // incomplete: C0, C1 and C3 are cleared
    if (r == 0)
      return pack(ua.sign, 0, 0);
    // remainder value = r * 2^(ua.exp - n - kBias - 63)
    return normRoundPack(ua.sign, ua.exp - n, U128{r, 0}, 64, cw, sw);
  }
  uint64_t q = 0;
  uint64_t r = 0;
  int32_t rExp = ub.exp; // remainder scale: value = r * 2^(rExp - kBias - 63)
  bool sign = ua.sign;
  if (d < 0) {
    q = 0;
    r = ua.sig;
    rExp = ua.exp;
    if (ieee && d == -1) {
      // |a| in [|b|/4, |b|): round to nearest if |a| > |b|/2
      // compare a.sig * 2^-1 with b.sig/2  <=>  a.sig > b.sig
      if (ua.sig > ub.sig) {
        // |a| - |b|, with a scaled at ub.exp - 1
        const U128 diff = sub128(U128{ub.sig, 0}, U128{ua.sig >> 1, ua.sig << 63});
        sw |= SW_C1; // q = 1
        return normRoundPack(!sign, ub.exp, diff, 64, cw, sw);
      }
    }
  } else {
    const int dd = static_cast<int>(d);
    const uint64_t hi = dd == 0 ? 0 : ua.sig >> (64 - dd);
    const uint64_t lo = ua.sig << dd;
    q = div128by64(hi, lo, ub.sig, r);
    if (ieee) {
      // round the quotient to nearest even
      const bool gt = r > ub.sig - r; // r > b/2
      const bool eq = r == ub.sig - r;
      if (gt || (eq && (q & 1))) {
        ++q;
        r = ub.sig - r;
        sign = !sign;
      }
    }
  }
  if (q & 4)
    sw |= SW_C0;
  if (q & 2)
    sw |= SW_C3;
  if (q & 1)
    sw |= SW_C1;
  if (r == 0)
    return pack(ua.sign, 0, 0);
  return normRoundPack(sign, rExp, U128{r, 0}, 64, cw, sw);
}

// ---------------------------------------------------------------------------
// Transcendentals via the host maths library.

using LD = long double;

LD toLD(const Float80 &v) {
  if (isZero(v))
    return v.sign() ? -0.0L : 0.0L;
  const Unpacked u = unpack(v);
  LD r = std::ldexp(static_cast<LD>(u.sig), u.exp - kBias - 63);
  return u.sign ? -r : r;
}

// Finite non-zero value split as m * 2^e with |m| in [1, 2), so values far
// outside the host's long double range (or double, on MSVC) keep their
// precision.
struct Split {
  LD m;
  int32_t e;
};

Split split(const Float80 &v) {
  const Unpacked u = unpack(v);
  const LD m = std::ldexp(static_cast<LD>(u.sig), -63);
  return {u.sign ? -m : m, u.exp - kBias};
}

// Rounds x * 2^extraExp to extended precision (64 bits, RC) and reports PE.
Float80 fromLD(LD x, int64_t extraExp, uint16_t cw, uint16_t &sw) {
  if (std::isnan(x))
    return invalid(sw);
  if (std::isinf(x))
    return pack(std::signbit(x), 0x7FFF, kIntegerBit);
  if (x == 0)
    return pack(std::signbit(x), 0, 0);
  int e = 0;
  const LD m = std::frexp(std::fabs(x), &e); // [0.5, 1)
  const LD scaled = std::ldexp(m, 64);
  uint64_t sig = 0;
  const LD two63 = std::ldexp(static_cast<LD>(1), 63);
  if (scaled >= two63) {
    // Convert via two halves so a 64-bit double extended converts exactly
    // and a narrower long double does not overflow int conversions.
    const LD hiPart = std::floor(scaled / two63);
    sig = static_cast<uint64_t>(hiPart) << 63;
    sig |= static_cast<uint64_t>(scaled - hiPart * two63);
  } else {
    sig = static_cast<uint64_t>(scaled);
  }
  const LD rest = scaled - static_cast<LD>(sig);
  const U128 s{sig, rest != 0 ? 1u : 0u};
  sw |= SW_PE;
  int64_t exp = static_cast<int64_t>(e) - 1 + kBias + extraExp;
  if (exp > 0x7FFF + kRebias)
    exp = 0x7FFF + kRebias;
  if (exp < -kRebias - 128)
    exp = -kRebias - 128;
  return normRoundPack(std::signbit(x), static_cast<int32_t>(exp), s, 64, cw, sw);
}

constexpr LD kLn2 = 0.693147180559945309417232121458176568L;
constexpr LD kPi = 3.14159265358979323846264338327950288L;

// Out-of-range check for FSIN/FCOS/FSINCOS/FPTAN: |x| >= 2^63.
bool trigOutOfRange(const Float80 &a) {
  return !isZero(a) && !isDenormal(a) && a.exp() >= kBias + 63;
}

// Argument reduction as the x87 does it: modulo pi/2 with the internal
// 66-bit approximation of pi (0x3243F6A8885A308D3 * 2^-64), exactly.
// Returns r = |x| - k * pi66/2 and k mod 4.  Requires 0.5 <= |x| < 2^63.
struct Reduced {
  LD r;
  int quadrant;
};

Reduced reducePi66(const Float80 &a) {
  const U128 m{0x3, 0x243F6A8885A308D3ull};
  const Unpacked u = unpack(a);
  const int sh = u.exp - kBias + 2; // X = |x| * 2^65 = sig << sh, sh in [1, 64]
  U128 x = shl128(U128{0, u.sig}, sh);
  uint64_t k = 0;
  for (int i = 62; i >= 0; --i) {
    const U128 d = shl128(m, i);
    if (x >= d) {
      x = sub128(x, d);
      k |= 1ull << i;
    }
  }
  bool negative = false;
  if (shl128(x, 1) >= m) { // round the quotient to nearest
    x = sub128(m, x);
    negative = true;
    ++k;
  }
  const LD two64 = std::ldexp(static_cast<LD>(1), 64);
  LD r = (static_cast<LD>(x.hi) * two64 + static_cast<LD>(x.lo));
  r = std::ldexp(r, -65);
  return {negative ? -r : r, static_cast<int>(k & 3)};
}

int trigOp(uint8_t modrm, const Float80 &a, Float80 *out, uint16_t cw,
           uint16_t &sw) {
  const bool pushes = modrm == 0xF2 || modrm == 0xFB;
  if (isUnsupported(a) || isInf(a)) {
    sw |= SW_IE;
    out[0] = kIndefinite;
    out[1] = kIndefinite;
    return pushes ? 2 : 1;
  }
  if (isNaN(a)) {
    const Float80 q = propagateNaN1(a, sw);
    out[0] = q;
    out[1] = q;
    return pushes ? 2 : 1;
  }
  if (trigOutOfRange(a)) {
    sw |= SW_C2;
    out[0] = a;
    return 1;
  }
  if (denormalStop(isDenormal(a), cw, sw)) {
    out[0] = a;
    return 1;
  }
  if (isZero(a)) {
    switch (modrm) {
    case 0xFE:
      out[0] = a;
      return 1;
    case 0xFF:
      out[0] = kOne;
      return 1;
    default: // FPTAN: tan(0) then 1; FSINCOS: sin(0) then cos(0)
      out[0] = kOne;
      out[1] = a;
      return 2;
    }
  }
  // Tiny arguments: sin(x) = tan(x) = x, cos(x) = 1, rounded.
  const bool tiny = a.exp() < kBias - 40;
  const Split sa = split(a);
  LD sinV = 0, cosV = 1, tanV = 0;
  if (!tiny) {
    const LD sign = a.sign() ? -1.0L : 1.0L;
    LD r = std::fabs(toLD(a));
    int q = 0;
    if (a.exp() >= kBias - 1) {
      const Reduced red = reducePi66(a);
      r = red.r;
      q = red.quadrant;
    }
    const LD sr = std::sin(r), cr = std::cos(r);
    static constexpr int sinSel[4] = {0, 1, 2, 3}; // sin, cos, -sin, -cos
    const LD vals[4] = {sr, cr, -sr, -cr};
    sinV = sign * vals[sinSel[q]];
    cosV = vals[(q + 1) & 3];
    tanV = sign * ((q & 1) ? -cr / sr : sr / cr);
  }
  auto sinOf = [&]() {
    return tiny ? fromLD(sa.m, sa.e, cw, sw) : fromLD(sinV, 0, cw, sw);
  };
  auto cosOf = [&]() { return fromLD(cosV, 0, cw, sw); };
  auto tanOf = [&]() {
    return tiny ? fromLD(sa.m, sa.e, cw, sw) : fromLD(tanV, 0, cw, sw);
  };
  switch (modrm) {
  case 0xFE:
    out[0] = sinOf();
    return 1;
  case 0xFF:
    out[0] = cosOf();
    return 1;
  case 0xF2:
    out[1] = tanOf();
    out[0] = kOne;
    return 2;
  default:
    out[1] = sinOf();
    out[0] = cosOf();
    return 2;
  }
}

// FPATAN: atan(ST1 / ST0) -> ST1, pop.  x = ST0, y = ST1.
Float80 patanOp(const Float80 &x, const Float80 &y, uint16_t cw, uint16_t &sw) {
  if (isUnsupported(x) || isUnsupported(y))
    return invalid(sw);
  if (isNaN(x) || isNaN(y))
    return propagateNaN(x, y, sw);
  if (denormalStop(isDenormal(x) || isDenormal(y), cw, sw))
    return x;
  if (isZero(y) || isInf(x) || isInf(y) || isZero(x)) {
    // Exact special cases follow atan2 (signed zeros / infinities).
    const LD r = std::atan2(isInf(y) ? (y.sign() ? -1.0L : 1.0L) * HUGE_VALL
                                     : (isZero(y) ? (y.sign() ? -0.0L : 0.0L)
                                                  : (y.sign() ? -1.0L : 1.0L)),
                            isInf(x) ? (x.sign() ? -1.0L : 1.0L) * HUGE_VALL
                                     : (isZero(x) ? (x.sign() ? -0.0L : 0.0L)
                                                  : (x.sign() ? -1.0L : 1.0L)));
    if (r == 0)
      return pack(std::signbit(r), 0, 0);
    return fromLD(r, 0, cw, sw);
  }
  const Split sx = split(x), sy = split(y);
  const int32_t d = sy.e - sx.e;
  if (d > 80)
    return fromLD(std::copysign(kPi / 2, sy.m), 0, cw, sw);
  if (d < -80) {
    if (sx.m > 0)
      return fromLD(sy.m / sx.m, d, cw, sw); // atan(t) = t for tiny t
    return fromLD(std::copysign(kPi, sy.m), 0, cw, sw);
  }
  return fromLD(std::atan2(std::ldexp(sy.m, d), sx.m), 0, cw, sw);
}

// FYL2X: ST1 * log2(ST0) -> ST1, pop.  x = ST0, y = ST1.
Float80 yl2xOp(const Float80 &x, const Float80 &y, uint16_t cw, uint16_t &sw) {
  if (isUnsupported(x) || isUnsupported(y))
    return invalid(sw);
  if (isNaN(x) || isNaN(y))
    return propagateNaN(x, y, sw);
  if (x.sign() && !isZero(x))
    return invalid(sw); // log of a negative number
  const bool xOne = x == kOne;
  if (isZero(x)) {
    if (isZero(y))
      return invalid(sw);
    if (!isInf(y))
      sw |= SW_ZE; // takes priority over a denormal ST1
    return pack(!y.sign(), 0x7FFF, kIntegerBit);
  }
  if (isInf(x)) {
    if (isZero(y))
      return invalid(sw);
    if (denormalStop(isDenormal(y), cw, sw))
      return x;
    return pack(y.sign(), 0x7FFF, kIntegerBit);
  }
  const bool xBelowOne = x.exp() < kBias;
  if (isInf(y)) {
    if (xOne)
      return invalid(sw);
    if (denormalStop(isDenormal(x), cw, sw))
      return x;
    return pack(y.sign() != xBelowOne, 0x7FFF, kIntegerBit);
  }
  if (denormalStop(isDenormal(x) || isDenormal(y), cw, sw))
    return x;
  if (xOne)
    return pack(y.sign(), 0, 0);
  if (isZero(y))
    return pack(y.sign() != xBelowOne, 0, 0);
  const Split sx = split(x), sy = split(y);
  LD l2;
  if (sx.e == 0 || sx.e == -1) // near 1: x - 1 is exact, avoid cancellation
    l2 = std::log1p(std::ldexp(sx.m, sx.e) - 1.0L) / kLn2;
  else
    l2 = std::log2(sx.m) + static_cast<LD>(sx.e);
  return fromLD(sy.m * l2, sy.e, cw, sw);
}

// FYL2XP1: ST1 * log2(ST0 + 1) -> ST1, pop.  x = ST0, y = ST1.
Float80 yl2xp1Op(const Float80 &x, const Float80 &y, uint16_t cw, uint16_t &sw) {
  if (isUnsupported(x) || isUnsupported(y))
    return invalid(sw);
  if (isNaN(x) || isNaN(y))
    return propagateNaN(x, y, sw);
  if (isInf(x)) {
    if (x.sign() || isZero(y))
      return invalid(sw);
    if (denormalStop(isDenormal(y), cw, sw))
      return x;
    return pack(y.sign(), 0x7FFF, kIntegerBit);
  }
  if (isZero(x) && isInf(y))
    return invalid(sw);
  if (denormalStop(isDenormal(x) || isDenormal(y), cw, sw))
    return x;
  if (isZero(x))
    return pack(x.sign() != y.sign(), 0, 0);
  if (isInf(y))
    return pack(x.sign() != y.sign(), 0x7FFF, kIntegerBit);
  if (isZero(y))
    return pack(x.sign() != y.sign(), 0, 0);
  const Split sx = split(x), sy = split(y);
  if (x.sign() && sx.e >= 0) { // x <= -1: outside the domain, x is returned
    sw |= SW_PE;
    return x;
  }
  if (sx.e < -60)
    return fromLD(sy.m * sx.m / kLn2, static_cast<int64_t>(sy.e) + sx.e, cw, sw);
  return fromLD(sy.m * (std::log1p(std::ldexp(sx.m, sx.e)) / kLn2), sy.e, cw, sw);
}

Float80 f2xm1Op(const Float80 &a, uint16_t cw, uint16_t &sw) {
  if (isUnsupported(a))
    return invalid(sw);
  if (isNaN(a))
    return propagateNaN1(a, sw);
  if (isZero(a))
    return a;
  if (isInf(a))
    return a.sign() ? pack(true, kBias, kIntegerBit) : a; // -1 or +inf
  if (denormalStop(isDenormal(a), cw, sw))
    return a;
  const Split s = split(a);
  if (s.e > 0 || (s.e == 0 && std::fabs(s.m) > 1)) { // |x| > 1: unchanged
    sw |= SW_PE;
    return a;
  }
  if (s.e < -60)
    return fromLD(s.m * kLn2, s.e, cw, sw);
  return fromLD(std::expm1(std::ldexp(s.m, s.e) * kLn2), 0, cw, sw);
}

} // namespace

// ---------------------------------------------------------------------------
// Public soft back end

namespace soft {

Float80 arith(Arith op, const Float80 &a, const Float80 &b, uint16_t cw,
              uint16_t &sw) {
  switch (op) {
  case Arith::Add:
    return addSub(a, b, false, false, false, cw, sw);
  case Arith::Sub:
    return addSub(a, b, true, false, false, cw, sw);
  case Arith::SubR:
    return addSub(b, a, true, false, false, cw, sw);
  case Arith::Mul:
    return mul(a, b, false, cw, sw);
  case Arith::Div:
    return div(a, b, false, false, cw, sw);
  default:
    return div(b, a, false, false, cw, sw);
  }
}

Float80 arithMem(Arith op, const Float80 &a, MemFmt fmt, uint64_t raw,
                 uint16_t cw, uint16_t &sw) {
  bool den = false;
  const Float80 b = memOperand(fmt, raw, den);
  switch (op) {
  case Arith::Add:
    return addSub(a, b, false, false, den, cw, sw);
  case Arith::Sub:
    return addSub(a, b, true, false, den, cw, sw);
  case Arith::SubR:
    return addSub(b, a, true, den, false, cw, sw);
  case Arith::Mul:
    return mul(a, b, den, cw, sw);
  case Arith::Div:
    return div(a, b, false, den, cw, sw);
  default:
    return div(b, a, den, false, cw, sw);
  }
}

void compare(const Float80 &a, const Float80 &b, bool unordered, uint16_t &sw) {
  constexpr uint16_t kUnordered = SW_C0 | SW_C2 | SW_C3;
  if (isUnsupported(a) || isUnsupported(b)) {
    sw |= SW_IE | kUnordered;
    return;
  }
  if (isNaN(a) || isNaN(b)) {
    if (!unordered || isSNaN(a) || isSNaN(b))
      sw |= SW_IE;
    sw |= kUnordered;
    return;
  }
  if (isDenormal(a) || isDenormal(b))
    sw |= SW_DE;
  const bool aZero = isZero(a), bZero = isZero(b);
  if (aZero && bZero) {
    sw |= SW_C3;
    return;
  }
  // Compare as sign-magnitude; normalise denormals for the magnitude order.
  auto key = [](const Float80 &v, bool zero) -> U128 {
    if (zero)
      return {0, 0};
    if (isInf(v))
      return {0x7FFF + 200, 0};
    const Unpacked u = unpack(v);
    // exp may be < 1 for denormals: bias it into the positive range
    return {static_cast<uint64_t>(static_cast<int64_t>(u.exp) + 100), u.sig};
  };
  const U128 ka = key(a, aZero), kb = key(b, bZero);
  const bool sa = !aZero && a.sign(), sb = !bZero && b.sign();
  int cmp = 0; // -1 a<b, 0 eq, 1 a>b
  if (sa != sb) {
    cmp = sa ? -1 : 1;
  } else {
    const int mag = ka < kb ? -1 : (ka == kb ? 0 : 1);
    cmp = sa ? -mag : mag;
  }
  if (cmp < 0)
    sw |= SW_C0;
  else if (cmp == 0)
    sw |= SW_C3;
}

void compareMem(const Float80 &a, MemFmt fmt, uint64_t raw, uint16_t &sw) {
  bool den = false;
  const Float80 b = memOperand(fmt, raw, den);
  uint16_t local = 0;
  compare(a, b, false, local);
  if (den && !(local & SW_IE))
    local |= SW_DE;
  sw |= local;
}

Float80 loadF32(uint32_t raw, uint16_t cw, uint16_t &sw) {
  bool den = false;
  Float80 v = fromF32Raw(raw, den);
  if (denormalStop(den, cw, sw))
    return v;
  if (isNaN(v))
    v = propagateNaN1(v, sw);
  return v;
}

Float80 loadF64(uint64_t raw, uint16_t cw, uint16_t &sw) {
  bool den = false;
  Float80 v = fromF64Raw(raw, den);
  if (denormalStop(den, cw, sw))
    return v;
  if (isNaN(v))
    v = propagateNaN1(v, sw);
  return v;
}

Float80 loadBCD(const uint8_t *bcd) {
  uint64_t v = 0;
  for (int i = 8; i >= 0; --i) {
    v = v * 10 + ((bcd[i] >> 4) & 0xF);
    v = v * 10 + (bcd[i] & 0xF);
  }
  const bool neg = (bcd[9] & 0x80) != 0;
  if (v == 0)
    return neg ? kNegZero : kPosZero;
  const int s = std::countl_zero(v);
  return pack(neg, kBias + 63 - s, v << s);
}

uint32_t storeF32(const Float80 &v, uint16_t cw, uint16_t &sw) {
  return static_cast<uint32_t>(storeBinary<23, 8>(v, cw, sw));
}

uint64_t storeF64(const Float80 &v, uint16_t cw, uint16_t &sw) {
  return storeBinary<52, 11>(v, cw, sw);
}

int64_t storeInt(const Float80 &v, int bits, uint16_t cw, uint16_t &sw) {
  return storeIntImpl(v, bits, cw, sw);
}

void storeBCD(const Float80 &v, uint8_t *bcd, uint16_t cw, uint16_t &sw) {
  auto indefinite = [&]() {
    sw |= SW_IE;
    std::memset(bcd, 0, 10);
    bcd[7] = 0xC0;
    bcd[8] = 0xFF;
    bcd[9] = 0xFF;
  };
  if (isUnsupported(v) || isNaN(v) || isInf(v)) {
    indefinite();
    return;
  }
  uint64_t mag = 0;
  bool inexact = false, up = false;
  if (!roundToInteger(v, cw, mag, inexact, up) || mag > 999999999999999999ull) {
    indefinite();
    return;
  }
  if (inexact)
    sw |= SW_PE;
  if (up)
    sw |= SW_C1;
  for (int i = 0; i < 9; ++i) {
    const uint8_t lo = static_cast<uint8_t>(mag % 10);
    mag /= 10;
    const uint8_t hi = static_cast<uint8_t>(mag % 10);
    mag /= 10;
    bcd[i] = static_cast<uint8_t>(lo | (hi << 4));
  }
  bcd[9] = v.sign() ? 0x80 : 0x00;
}

int stackOp(uint8_t modrm, int nIn, const Float80 *in, Float80 *out,
            uint16_t cw, uint16_t &sw) {
  (void)nIn;
  switch (modrm) {
  case 0xE0: // FCHS
    out[0] = Float80::make(!in[0].sign(), in[0].exp(), in[0].mant);
    return 1;
  case 0xE1: // FABS
    out[0] = Float80::make(false, in[0].exp(), in[0].mant);
    return 1;
  case 0xF0:
    out[0] = f2xm1Op(in[0], cw, sw);
    return 1;
  case 0xF1:
    out[0] = yl2xOp(in[0], in[1], cw, sw);
    return 1;
  case 0xF3:
    out[0] = patanOp(in[0], in[1], cw, sw);
    return 1;
  case 0xF4:
    return xtractOp(in[0], out, cw, sw);
  case 0xF5:
  case 0xF8:
    out[0] = premOp(in[0], in[1], modrm == 0xF5, cw, sw);
    out[1] = in[1];
    return 2;
  case 0xF9:
    out[0] = yl2xp1Op(in[0], in[1], cw, sw);
    return 1;
  case 0xFA:
    out[0] = sqrtOp(in[0], cw, sw);
    return 1;
  case 0xFC:
    out[0] = roundToInt(in[0], cw, sw);
    return 1;
  case 0xFD:
    out[0] = scaleOp(in[0], in[1], cw, sw);
    out[1] = in[1];
    return 2;
  case 0xF2:
  case 0xFB:
  case 0xFE:
  case 0xFF:
    return trigOp(modrm, in[0], out, cw, sw);
  default:
    out[0] = in[0];
    if (nIn > 1)
      out[1] = in[1];
    return nIn;
  }
}

Float80 loadConst(Const c, uint16_t cw) {
  const int rc = roundingOf(cw);
  const bool downOrChop = rc == RD || rc == RZ;
  switch (c) {
  case Const::One:
    return kOne;
  case Const::L2T:
    return Float80(0xD49A784BCD1B8AFEull + (rc == RU ? 1 : 0), 0x4000);
  case Const::L2E:
    return Float80(0xB8AA3B295C17F0BCull - (downOrChop ? 1 : 0), 0x3FFF);
  case Const::Pi:
    return Float80(0xC90FDAA22168C235ull - (downOrChop ? 1 : 0), 0x4000);
  case Const::LG2:
    return Float80(0x9A209A84FBCFF799ull - (downOrChop ? 1 : 0), 0x3FFD);
  case Const::LN2:
    return Float80(0xB17217F7D1CF79ACull - (downOrChop ? 1 : 0), 0x3FFE);
  default:
    return kPosZero;
  }
}

} // namespace soft

// ---------------------------------------------------------------------------

Float80 fromDouble(double value) {
  uint64_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));
  bool den = false;
  return fromF64Raw(raw, den);
}

double toDouble(const Float80 &value) {
  uint16_t sw = 0;
  const uint64_t raw = soft::storeF64(value, 0x037F, sw);
  double d = 0;
  std::memcpy(&d, &raw, sizeof(d));
  return d;
}

} // namespace fador::cpu::x87
