// Portable reference kernels.  These define the exact results every SIMD
// variant has to reproduce; keep them simple and obviously correct, but
// not needlessly slow (they are the fallback on hosts without SIMD).

#include "KernelsInternal.hpp"

#include <array>
#include <bit>
#include <cmath>
#include <cstring>

namespace fador::utils::simd::detail::scalar {

namespace {

// Byte b of a planar plane → 8 bytes (in memory order, pixel 0 first),
// each 0 or 1: bit 7 is the leftmost pixel.
constexpr std::array<uint64_t, 256> makeSpreadTable() {
  std::array<uint64_t, 256> t{};
  for (unsigned b = 0; b < 256; ++b) {
    std::array<uint8_t, 8> bytes{};
    for (unsigned k = 0; k < 8; ++k)
      bytes[k] = static_cast<uint8_t>((b >> (7 - k)) & 1u);
    t[b] = std::bit_cast<uint64_t>(bytes);
  }
  return t;
}
constexpr auto kSpread = makeSpreadTable();

inline uint32_t expand5(uint32_t v) { return (v << 3) | (v >> 2); }
inline uint32_t expand6(uint32_t v) { return (v << 2) | (v >> 4); }

} // namespace

void paletteToArgb(uint32_t *dst, const uint8_t *src, size_t n,
                   const uint32_t *palette) {
  size_t i = 0;
  for (; i + 4 <= n; i += 4) {
    const uint32_t a = palette[src[i + 0]];
    const uint32_t b = palette[src[i + 1]];
    const uint32_t c = palette[src[i + 2]];
    const uint32_t d = palette[src[i + 3]];
    dst[i + 0] = a;
    dst[i + 1] = b;
    dst[i + 2] = c;
    dst[i + 3] = d;
  }
  for (; i < n; ++i)
    dst[i] = palette[src[i]];
}

void paletteToArgbX2(uint32_t *dst, const uint8_t *src, size_t n,
                     const uint32_t *palette) {
  for (size_t i = 0; i < n; ++i) {
    const uint32_t v = palette[src[i]];
    dst[2 * i] = v;
    dst[2 * i + 1] = v;
  }
}

void interleave4(uint8_t *dst, const uint8_t *const planes[4], size_t n) {
  const uint8_t *p0 = planes[0], *p1 = planes[1], *p2 = planes[2],
                *p3 = planes[3];
  const size_t groups = n / 4;
  for (size_t g = 0; g < groups; ++g) {
    dst[4 * g + 0] = p0[g];
    dst[4 * g + 1] = p1[g];
    dst[4 * g + 2] = p2[g];
    dst[4 * g + 3] = p3[g];
  }
  for (size_t i = groups * 4; i < n; ++i)
    dst[i] = planes[i & 3][i >> 2];
}

void planarToIndex(uint8_t *dst, const uint8_t *const planes[4], size_t n) {
  const uint8_t *p0 = planes[0], *p1 = planes[1], *p2 = planes[2],
                *p3 = planes[3];
  auto group = [&](size_t j) {
    return kSpread[p0[j]] | (kSpread[p1[j]] << 1) | (kSpread[p2[j]] << 2) |
           (kSpread[p3[j]] << 3);
  };
  const size_t full = n / 8;
  for (size_t j = 0; j < full; ++j) {
    const uint64_t v = group(j);
    std::memcpy(dst + 8 * j, &v, 8);
  }
  if (const size_t rest = n % 8) {
    const uint64_t v = group(full);
    std::memcpy(dst + 8 * full, &v, rest);
  }
}

void rgb555ToArgb(uint32_t *dst, const uint8_t *src, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    const uint32_t p = src[2 * i] | (uint32_t(src[2 * i + 1]) << 8);
    const uint32_t r = (p >> 10) & 0x1F, g = (p >> 5) & 0x1F, b = p & 0x1F;
    dst[i] = 0xFF000000u | (expand5(r) << 16) | (expand5(g) << 8) | expand5(b);
  }
}

void rgb565ToArgb(uint32_t *dst, const uint8_t *src, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    const uint32_t p = src[2 * i] | (uint32_t(src[2 * i + 1]) << 8);
    const uint32_t r = (p >> 11) & 0x1F, g = (p >> 5) & 0x3F, b = p & 0x1F;
    dst[i] = 0xFF000000u | (expand5(r) << 16) | (expand6(g) << 8) | expand5(b);
  }
}

void rgb24ToArgb(uint32_t *dst, const uint8_t *src, size_t n) {
  for (size_t i = 0; i < n; ++i)
    dst[i] = 0xFF000000u | (uint32_t(src[3 * i + 2]) << 16) |
             (uint32_t(src[3 * i + 1]) << 8) | src[3 * i];
}

void rgb32ToArgb(uint32_t *dst, const uint8_t *src, size_t n) {
  for (size_t i = 0; i < n; ++i)
    dst[i] = 0xFF000000u | (uint32_t(src[4 * i + 2]) << 16) |
             (uint32_t(src[4 * i + 1]) << 8) | src[4 * i];
}

// The float kernels keep the multiply and the add in separate statements
// so no compiler contracts them into an FMA (the SIMD variants do not
// fuse either, and the results must match bit for bit).
void mixAddF32(float *dst, const float *src, size_t n, float gain) {
  for (size_t i = 0; i < n; ++i) {
    const float t = src[i] * gain;
    dst[i] = dst[i] + t;
  }
}

void s16ToF32(float *dst, const uint8_t *src, size_t n, float gain) {
  for (size_t i = 0; i < n; ++i) {
    const auto v = static_cast<int16_t>(src[2 * i] | (src[2 * i + 1] << 8));
    dst[i] = static_cast<float>(v) * gain;
  }
}

void u8ToF32(float *dst, const uint8_t *src, size_t n, float gain) {
  for (size_t i = 0; i < n; ++i) {
    const float v = static_cast<float>(src[i]) - 128.0f;
    dst[i] = v * gain;
  }
}

float f32ToS16(int16_t *dst, const float *src, size_t n) {
  float peak = 0.0f;
  for (size_t i = 0; i < n; ++i) {
    float v = src[i];
    v = v > -1.0f ? v : -1.0f; // also maps NaN to -1
    v = v < 1.0f ? v : 1.0f;
    const float a = std::fabs(v);
    peak = a > peak ? a : peak;
    dst[i] = static_cast<int16_t>(std::lrint(v * 32767.0f));
  }
  return peak;
}

void fillPattern32(uint8_t *dst, size_t bytes, uint32_t pattern) {
  std::array<uint8_t, 8> pb{};
  for (size_t i = 0; i < 8; ++i)
    pb[i] = static_cast<uint8_t>(pattern >> (8 * (i & 3)));
  if (pb[0] == pb[1] && pb[0] == pb[2] && pb[0] == pb[3]) {
    std::memset(dst, pb[0], bytes);
    return;
  }
  size_t i = 0;
  for (; i + 8 <= bytes; i += 8)
    std::memcpy(dst + i, pb.data(), 8);
  for (; i < bytes; ++i)
    dst[i] = pb[i & 3];
}

} // namespace fador::utils::simd::detail::scalar

namespace fador::utils::simd::detail {

void installScalar(Kernels &k) {
  k.paletteToArgb = scalar::paletteToArgb;
  k.paletteToArgbX2 = scalar::paletteToArgbX2;
  k.interleave4 = scalar::interleave4;
  k.planarToIndex = scalar::planarToIndex;
  k.rgb555ToArgb = scalar::rgb555ToArgb;
  k.rgb565ToArgb = scalar::rgb565ToArgb;
  k.rgb24ToArgb = scalar::rgb24ToArgb;
  k.rgb32ToArgb = scalar::rgb32ToArgb;
  k.mixAddF32 = scalar::mixAddF32;
  k.s16ToF32 = scalar::s16ToF32;
  k.u8ToF32 = scalar::u8ToF32;
  k.f32ToS16 = scalar::f32ToS16;
  k.fillPattern32 = scalar::fillPattern32;
}

} // namespace fador::utils::simd::detail
