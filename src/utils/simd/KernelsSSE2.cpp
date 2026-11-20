// SSE2 kernels (x86-64 baseline; the target attribute only matters for
// 32-bit x86 builds).

#include "KernelsInternal.hpp"

#ifdef FADOR_SIMD_X86

#include <emmintrin.h>
#if defined(_MSC_VER) && !defined(__clang__)
#include <intrin.h>
#endif

#include <cstring>

namespace fador::utils::simd::detail {

namespace {

#define SSE2_FN FADOR_TARGET("sse2")

SSE2_FN void interleave4SSE2(uint8_t *dst, const uint8_t *const planes[4],
                             size_t n) {
  const uint8_t *p0 = planes[0], *p1 = planes[1], *p2 = planes[2],
                *p3 = planes[3];
  size_t g = 0; // plane byte index
  for (; (g + 16) * 4 <= n; g += 16) {
    const __m128i a = _mm_loadu_si128(reinterpret_cast<const __m128i *>(p0 + g));
    const __m128i b = _mm_loadu_si128(reinterpret_cast<const __m128i *>(p1 + g));
    const __m128i c = _mm_loadu_si128(reinterpret_cast<const __m128i *>(p2 + g));
    const __m128i d = _mm_loadu_si128(reinterpret_cast<const __m128i *>(p3 + g));
    const __m128i ab0 = _mm_unpacklo_epi8(a, b), ab1 = _mm_unpackhi_epi8(a, b);
    const __m128i cd0 = _mm_unpacklo_epi8(c, d), cd1 = _mm_unpackhi_epi8(c, d);
    auto *out = reinterpret_cast<__m128i *>(dst + 4 * g);
    _mm_storeu_si128(out + 0, _mm_unpacklo_epi16(ab0, cd0));
    _mm_storeu_si128(out + 1, _mm_unpackhi_epi16(ab0, cd0));
    _mm_storeu_si128(out + 2, _mm_unpacklo_epi16(ab1, cd1));
    _mm_storeu_si128(out + 3, _mm_unpackhi_epi16(ab1, cd1));
  }
  const uint8_t *rest[4] = {p0 + g, p1 + g, p2 + g, p3 + g};
  scalar::interleave4(dst + 4 * g, rest, n - 4 * g);
}

// 8 RGB555/565 pixels (16-bit lanes) → 8 ARGB pixels.
template <bool k565>
SSE2_FN inline void hiColor8(uint32_t *dst, __m128i p) {
  const __m128i m5 = _mm_set1_epi16(0x1F);
  __m128i r, g, b;
  if constexpr (k565) {
    r = _mm_srli_epi16(p, 11);
    g = _mm_and_si128(_mm_srli_epi16(p, 5), _mm_set1_epi16(0x3F));
    g = _mm_or_si128(_mm_slli_epi16(g, 2), _mm_srli_epi16(g, 4));
  } else {
    r = _mm_and_si128(_mm_srli_epi16(p, 10), m5);
    g = _mm_and_si128(_mm_srli_epi16(p, 5), m5);
    g = _mm_or_si128(_mm_slli_epi16(g, 3), _mm_srli_epi16(g, 2));
  }
  b = _mm_and_si128(p, m5);
  r = _mm_or_si128(_mm_slli_epi16(r, 3), _mm_srli_epi16(r, 2));
  b = _mm_or_si128(_mm_slli_epi16(b, 3), _mm_srli_epi16(b, 2));
  const __m128i gb = _mm_or_si128(_mm_slli_epi16(g, 8), b);
  const __m128i ar = _mm_or_si128(r, _mm_set1_epi16(static_cast<short>(0xFF00)));
  _mm_storeu_si128(reinterpret_cast<__m128i *>(dst), _mm_unpacklo_epi16(gb, ar));
  _mm_storeu_si128(reinterpret_cast<__m128i *>(dst + 4),
                   _mm_unpackhi_epi16(gb, ar));
}

SSE2_FN void rgb555SSE2(uint32_t *dst, const uint8_t *src, size_t n) {
  size_t i = 0;
  for (; i + 8 <= n; i += 8)
    hiColor8<false>(dst + i, _mm_loadu_si128(
                                 reinterpret_cast<const __m128i *>(src + 2 * i)));
  scalar::rgb555ToArgb(dst + i, src + 2 * i, n - i);
}

SSE2_FN void rgb565SSE2(uint32_t *dst, const uint8_t *src, size_t n) {
  size_t i = 0;
  for (; i + 8 <= n; i += 8)
    hiColor8<true>(dst + i, _mm_loadu_si128(
                                reinterpret_cast<const __m128i *>(src + 2 * i)));
  scalar::rgb565ToArgb(dst + i, src + 2 * i, n - i);
}

SSE2_FN void rgb32SSE2(uint32_t *dst, const uint8_t *src, size_t n) {
  const __m128i alpha = _mm_set1_epi32(static_cast<int>(0xFF000000u));
  size_t i = 0;
  for (; i + 8 <= n; i += 8) {
    const __m128i a = _mm_loadu_si128(reinterpret_cast<const __m128i *>(src + 4 * i));
    const __m128i b =
        _mm_loadu_si128(reinterpret_cast<const __m128i *>(src + 4 * i + 16));
    _mm_storeu_si128(reinterpret_cast<__m128i *>(dst + i), _mm_or_si128(a, alpha));
    _mm_storeu_si128(reinterpret_cast<__m128i *>(dst + i + 4),
                     _mm_or_si128(b, alpha));
  }
  scalar::rgb32ToArgb(dst + i, src + 4 * i, n - i);
}

SSE2_FN void mixAddSSE2(float *dst, const float *src, size_t n, float gain) {
  const __m128 g = _mm_set1_ps(gain);
  size_t i = 0;
  for (; i + 8 <= n; i += 8) {
    const __m128 s0 = _mm_mul_ps(_mm_loadu_ps(src + i), g);
    const __m128 s1 = _mm_mul_ps(_mm_loadu_ps(src + i + 4), g);
    _mm_storeu_ps(dst + i, _mm_add_ps(_mm_loadu_ps(dst + i), s0));
    _mm_storeu_ps(dst + i + 4, _mm_add_ps(_mm_loadu_ps(dst + i + 4), s1));
  }
  scalar::mixAddF32(dst + i, src + i, n - i, gain);
}

SSE2_FN void s16ToF32SSE2(float *dst, const uint8_t *src, size_t n,
                          float gain) {
  const __m128 g = _mm_set1_ps(gain);
  size_t i = 0;
  for (; i + 8 <= n; i += 8) {
    const __m128i v = _mm_loadu_si128(reinterpret_cast<const __m128i *>(src + 2 * i));
    const __m128i lo = _mm_srai_epi32(_mm_unpacklo_epi16(v, v), 16);
    const __m128i hi = _mm_srai_epi32(_mm_unpackhi_epi16(v, v), 16);
    _mm_storeu_ps(dst + i, _mm_mul_ps(_mm_cvtepi32_ps(lo), g));
    _mm_storeu_ps(dst + i + 4, _mm_mul_ps(_mm_cvtepi32_ps(hi), g));
  }
  scalar::s16ToF32(dst + i, src + 2 * i, n - i, gain);
}

SSE2_FN void u8ToF32SSE2(float *dst, const uint8_t *src, size_t n,
                         float gain) {
  const __m128 g = _mm_set1_ps(gain);
  const __m128 bias = _mm_set1_ps(128.0f);
  const __m128i zero = _mm_setzero_si128();
  size_t i = 0;
  for (; i + 16 <= n; i += 16) {
    const __m128i v = _mm_loadu_si128(reinterpret_cast<const __m128i *>(src + i));
    const __m128i w0 = _mm_unpacklo_epi8(v, zero), w1 = _mm_unpackhi_epi8(v, zero);
    const __m128i d[4] = {_mm_unpacklo_epi16(w0, zero), _mm_unpackhi_epi16(w0, zero),
                          _mm_unpacklo_epi16(w1, zero), _mm_unpackhi_epi16(w1, zero)};
    for (int k = 0; k < 4; ++k)
      _mm_storeu_ps(dst + i + 4 * k,
                    _mm_mul_ps(_mm_sub_ps(_mm_cvtepi32_ps(d[k]), bias), g));
  }
  scalar::u8ToF32(dst + i, src + i, n - i, gain);
}

SSE2_FN float f32ToS16SSE2(int16_t *dst, const float *src, size_t n) {
  const __m128 lo = _mm_set1_ps(-1.0f), hi = _mm_set1_ps(1.0f);
  const __m128 scale = _mm_set1_ps(32767.0f);
  const __m128 absMask = _mm_castsi128_ps(_mm_set1_epi32(0x7FFFFFFF));
  __m128 peak = _mm_setzero_ps();
  size_t i = 0;
  for (; i + 8 <= n; i += 8) {
    // max(v, -1) / min(v, 1) return the second operand for NaN, exactly
    // like the scalar comparisons.
    __m128 a = _mm_min_ps(_mm_max_ps(_mm_loadu_ps(src + i), lo), hi);
    __m128 b = _mm_min_ps(_mm_max_ps(_mm_loadu_ps(src + i + 4), lo), hi);
    peak = _mm_max_ps(peak, _mm_and_ps(a, absMask));
    peak = _mm_max_ps(peak, _mm_and_ps(b, absMask));
    const __m128i ia = _mm_cvtps_epi32(_mm_mul_ps(a, scale));
    const __m128i ib = _mm_cvtps_epi32(_mm_mul_ps(b, scale));
    _mm_storeu_si128(reinterpret_cast<__m128i *>(dst + i), _mm_packs_epi32(ia, ib));
  }
  float lanes[4];
  _mm_storeu_ps(lanes, peak);
  float p = scalar::f32ToS16(dst + i, src + i, n - i);
  for (float l : lanes)
    p = l > p ? l : p;
  return p;
}

SSE2_FN void fillPattern32SSE2(uint8_t *dst, size_t bytes, uint32_t pattern) {
  if (bytes < 64 || ((pattern ^ (pattern >> 8)) & 0x00FFFFFFu) == 0) {
    scalar::fillPattern32(dst, bytes, pattern); // small or memset-able
    return;
  }
  const __m128i v = _mm_set1_epi32(static_cast<int>(pattern));
  size_t i = 0;
  for (; i + 64 <= bytes; i += 64) {
    auto *o = reinterpret_cast<__m128i *>(dst + i);
    _mm_storeu_si128(o + 0, v);
    _mm_storeu_si128(o + 1, v);
    _mm_storeu_si128(o + 2, v);
    _mm_storeu_si128(o + 3, v);
  }
  for (; i + 16 <= bytes; i += 16)
    _mm_storeu_si128(reinterpret_cast<__m128i *>(dst + i), v);
  // i is a multiple of 4, so the pattern phase is unchanged for the tail.
  scalar::fillPattern32(dst + i, bytes - i, pattern);
}

// Enhanced REP STOSB/MOVSB CPUs: the microcoded string store writes whole
// cache lines and beats 16/32-byte vector stores about 3x from ~1 KiB up
// (measured on Sapphire Rapids: 16 KiB in 82 ns vs 262 ns with AVX2).
SSE2_FN void fillPattern32Erms(uint8_t *dst, size_t bytes, uint32_t pattern) {
  if (bytes < 1024 || ((pattern ^ (pattern >> 8)) & 0x00FFFFFFu) == 0) {
    fillPattern32SSE2(dst, bytes, pattern);
    return;
  }
  const size_t dwords = bytes / 4;
#if defined(_MSC_VER) && !defined(__clang__)
  __stosd(reinterpret_cast<unsigned long *>(dst), pattern, dwords);
#else
  void *d = dst;
  size_t c = dwords;
  __asm__ volatile("rep stosl" : "+D"(d), "+c"(c) : "a"(pattern) : "memory");
#endif
  scalar::fillPattern32(dst + 4 * dwords, bytes - 4 * dwords, pattern);
}

#undef SSE2_FN

} // namespace

void installErms(Kernels &k) { k.fillPattern32 = fillPattern32Erms; }

void installSSE2(Kernels &k) {
  k.interleave4 = interleave4SSE2;
  k.rgb555ToArgb = rgb555SSE2;
  k.rgb565ToArgb = rgb565SSE2;
  k.rgb32ToArgb = rgb32SSE2;
  k.mixAddF32 = mixAddSSE2;
  k.s16ToF32 = s16ToF32SSE2;
  k.u8ToF32 = u8ToF32SSE2;
  k.f32ToS16 = f32ToS16SSE2;
  k.fillPattern32 = fillPattern32SSE2;
}

} // namespace fador::utils::simd::detail

#endif // FADOR_SIMD_X86
