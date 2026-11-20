#pragma once

// Minimal unsigned 128-bit arithmetic for the software x87 back end.
// Uses the compiler's __int128 where available; the portable fallback (used
// by MSVC) can be forced with FADOR_X87_PORTABLE_U128 so it is testable on
// any host.

#include <bit>
#include <cstdint>

#if !defined(FADOR_X87_PORTABLE_U128)
#if defined(__SIZEOF_INT128__)
#define FADOR_X87_PORTABLE_U128 0
#else
#define FADOR_X87_PORTABLE_U128 1
#endif
#endif

namespace fador::cpu::x87 {

#if !FADOR_X87_PORTABLE_U128
__extension__ typedef unsigned __int128 NativeU128;
#endif

struct U128 {
  uint64_t hi = 0;
  uint64_t lo = 0;

  constexpr bool isZero() const { return (hi | lo) == 0; }
  constexpr bool operator==(const U128 &o) const { return hi == o.hi && lo == o.lo; }
  constexpr bool operator<(const U128 &o) const {
    return hi < o.hi || (hi == o.hi && lo < o.lo);
  }
  constexpr bool operator>(const U128 &o) const { return o < *this; }
  constexpr bool operator>=(const U128 &o) const { return !(*this < o); }
  constexpr bool operator<=(const U128 &o) const { return !(o < *this); }
};

constexpr U128 add128(U128 a, U128 b) {
  U128 r{a.hi + b.hi, a.lo + b.lo};
  if (r.lo < a.lo)
    ++r.hi;
  return r;
}

constexpr U128 sub128(U128 a, U128 b) {
  U128 r{a.hi - b.hi, a.lo - b.lo};
  if (a.lo < b.lo)
    --r.hi;
  return r;
}

constexpr U128 shl128(U128 a, int n) {
  if (n <= 0)
    return a;
  if (n >= 128)
    return {0, 0};
  if (n >= 64)
    return {a.lo << (n - 64), 0};
  return {(a.hi << n) | (a.lo >> (64 - n)), a.lo << n};
}

constexpr U128 shr128(U128 a, int n) {
  if (n <= 0)
    return a;
  if (n >= 128)
    return {0, 0};
  if (n >= 64)
    return {0, a.hi >> (n - 64)};
  return {a.hi >> n, (a.lo >> n) | (a.hi << (64 - n))};
}

// Shift right, OR-ing every bit shifted out into bit 0 ("sticky").
constexpr U128 shr128Jam(U128 a, int n) {
  if (n <= 0)
    return a;
  if (n >= 128)
    return {0, a.isZero() ? 0u : 1u};
  U128 r = shr128(a, n);
  const U128 back = shl128(r, n);
  if (!(back == a))
    r.lo |= 1;
  return r;
}

constexpr int clz128(U128 a) {
  if (a.hi)
    return std::countl_zero(a.hi);
  if (a.lo)
    return 64 + std::countl_zero(a.lo);
  return 128;
}

// Portable versions (always compiled so tests can check them on any host).
inline U128 mul64x64Portable(uint64_t a, uint64_t b) {
  const uint64_t aL = a & 0xFFFFFFFFu, aH = a >> 32;
  const uint64_t bL = b & 0xFFFFFFFFu, bH = b >> 32;
  const uint64_t ll = aL * bL, lh = aL * bH, hl = aH * bL, hh = aH * bH;
  const uint64_t mid = (ll >> 32) + (lh & 0xFFFFFFFFu) + (hl & 0xFFFFFFFFu);
  const uint64_t lo = (ll & 0xFFFFFFFFu) | (mid << 32);
  const uint64_t hi = hh + (lh >> 32) + (hl >> 32) + (mid >> 32);
  return {hi, lo};
}

// (hi:lo) / d with hi < d: Hacker's Delight divlu (normalised long division
// with 32-bit digits).  Returns the quotient, stores the remainder.
inline uint64_t div128by64Portable(uint64_t hi, uint64_t lo, uint64_t d,
                                   uint64_t &rem) {
  constexpr uint64_t b = 1ull << 32;
  const int s = std::countl_zero(d);
  d <<= s;
  const uint64_t vn1 = d >> 32, vn0 = d & 0xFFFFFFFFu;
  const uint64_t un32 = s == 0 ? hi : (hi << s) | (lo >> (64 - s));
  const uint64_t un10 = lo << s;
  const uint64_t un1 = un10 >> 32, un0 = un10 & 0xFFFFFFFFu;
  uint64_t q1 = un32 / vn1;
  uint64_t rhat = un32 - q1 * vn1;
  while (q1 >= b || q1 * vn0 > b * rhat + un1) {
    --q1;
    rhat += vn1;
    if (rhat >= b)
      break;
  }
  const uint64_t un21 = un32 * b + un1 - q1 * d;
  uint64_t q0 = un21 / vn1;
  rhat = un21 - q0 * vn1;
  while (q0 >= b || q0 * vn0 > b * rhat + un0) {
    --q0;
    rhat += vn1;
    if (rhat >= b)
      break;
  }
  rem = (un21 * b + un0 - q0 * d) >> s;
  return q1 * b + q0;
}

inline U128 mul64x64(uint64_t a, uint64_t b) {
#if !FADOR_X87_PORTABLE_U128
  const NativeU128 p = static_cast<NativeU128>(a) * b;
  return {static_cast<uint64_t>(p >> 64), static_cast<uint64_t>(p)};
#else
  return mul64x64Portable(a, b);
#endif
}

inline uint64_t div128by64(uint64_t hi, uint64_t lo, uint64_t d, uint64_t &rem) {
#if !FADOR_X87_PORTABLE_U128
  const NativeU128 n = (static_cast<NativeU128>(hi) << 64) | lo;
  rem = static_cast<uint64_t>(n % d);
  return static_cast<uint64_t>(n / d);
#else
  return div128by64Portable(hi, lo, d, rem);
#endif
}

} // namespace fador::cpu::x87
