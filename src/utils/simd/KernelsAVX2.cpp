// AVX2 kernels.  Compiled per function with target("avx2"); only called
// when CPUID and XGETBV report AVX2 with OS support for the YMM state.

#include "KernelsInternal.hpp"

#ifdef FADOR_SIMD_X86

#include <immintrin.h>

namespace fador::utils::simd::detail {

namespace {

#define AVX2_FN FADOR_TARGET("avx2")

AVX2_FN inline __m256i loadu256(const void *p) {
  return _mm256_loadu_si256(static_cast<const __m256i *>(p));
}
AVX2_FN inline void storeu256(void *p, __m256i v) {
  _mm256_storeu_si256(static_cast<__m256i *>(p), v);
}

// Not here on purpose (measured with --bench=render-*, see README):
//  * 8bpp palette lookup: VPGATHERDD was only ~1.2x faster than the scalar
//    loop on Sapphire Rapids and is several times slower on CPUs with the
//    Gather Data Sampling microcode mitigation and on older AMD cores.
//  * plane interleave, 24bpp and 32bpp: no faster than the SSE2 / SSE4
//    variants (memory bound), so the AVX2 level inherits those.

// 16 bytes per plane → 128 indices, 32 per step (see planarToIndexSSE4).
AVX2_FN void planarToIndexAVX2(uint8_t *dst, const uint8_t *const planes[4],
                               size_t n) {
  const __m256i bitMask = _mm256_set1_epi64x(0x0102040810204080LL);
  const __m256i weight[4] = {_mm256_set1_epi8(1), _mm256_set1_epi8(2),
                             _mm256_set1_epi8(4), _mm256_set1_epi8(8)};
  // Step k covers plane bytes 4k..4k+3: lane 0 = bytes 4k, 4k+1 (x8),
  // lane 1 = bytes 4k+2, 4k+3 (x8).
  const __m256i shufBase = _mm256_setr_epi8(
      0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
      2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3);
  size_t j = 0;
  for (; (j + 16) * 8 <= n; j += 16) {
    __m256i v[4];
    for (int p = 0; p < 4; ++p)
      v[p] = _mm256_broadcastsi128_si256(
          _mm_loadu_si128(reinterpret_cast<const __m128i *>(planes[p] + j)));
    for (int k = 0; k < 4; ++k) {
      const __m256i shuf =
          _mm256_add_epi8(shufBase, _mm256_set1_epi8(static_cast<char>(4 * k)));
      __m256i acc = _mm256_setzero_si256();
      for (int p = 0; p < 4; ++p) {
        const __m256i e = _mm256_and_si256(_mm256_shuffle_epi8(v[p], shuf), bitMask);
        acc = _mm256_or_si256(
            acc, _mm256_and_si256(_mm256_cmpeq_epi8(e, bitMask), weight[p]));
      }
      storeu256(dst + 8 * j + 32 * k, acc);
    }
  }
  const uint8_t *rest[4] = {planes[0] + j, planes[1] + j, planes[2] + j,
                            planes[3] + j};
  scalar::planarToIndex(dst + 8 * j, rest, n - 8 * j);
}

// 16 RGB555/565 pixels → ARGB.
template <bool k565>
AVX2_FN inline void hiColor16(uint32_t *dst, __m256i p) {
  const __m256i m5 = _mm256_set1_epi16(0x1F);
  __m256i r, g, b;
  if constexpr (k565) {
    r = _mm256_srli_epi16(p, 11);
    g = _mm256_and_si256(_mm256_srli_epi16(p, 5), _mm256_set1_epi16(0x3F));
    g = _mm256_or_si256(_mm256_slli_epi16(g, 2), _mm256_srli_epi16(g, 4));
  } else {
    r = _mm256_and_si256(_mm256_srli_epi16(p, 10), m5);
    g = _mm256_and_si256(_mm256_srli_epi16(p, 5), m5);
    g = _mm256_or_si256(_mm256_slli_epi16(g, 3), _mm256_srli_epi16(g, 2));
  }
  b = _mm256_and_si256(p, m5);
  r = _mm256_or_si256(_mm256_slli_epi16(r, 3), _mm256_srli_epi16(r, 2));
  b = _mm256_or_si256(_mm256_slli_epi16(b, 3), _mm256_srli_epi16(b, 2));
  const __m256i gb = _mm256_or_si256(_mm256_slli_epi16(g, 8), b);
  const __m256i ar =
      _mm256_or_si256(r, _mm256_set1_epi16(static_cast<short>(0xFF00)));
  const __m256i lo = _mm256_unpacklo_epi16(gb, ar); // 0..3  | 8..11
  const __m256i hi = _mm256_unpackhi_epi16(gb, ar); // 4..7  | 12..15
  storeu256(dst, _mm256_permute2x128_si256(lo, hi, 0x20));
  storeu256(dst + 8, _mm256_permute2x128_si256(lo, hi, 0x31));
}

AVX2_FN void rgb555AVX2(uint32_t *dst, const uint8_t *src, size_t n) {
  size_t i = 0;
  for (; i + 16 <= n; i += 16)
    hiColor16<false>(dst + i, loadu256(src + 2 * i));
  scalar::rgb555ToArgb(dst + i, src + 2 * i, n - i);
}

AVX2_FN void rgb565AVX2(uint32_t *dst, const uint8_t *src, size_t n) {
  size_t i = 0;
  for (; i + 16 <= n; i += 16)
    hiColor16<true>(dst + i, loadu256(src + 2 * i));
  scalar::rgb565ToArgb(dst + i, src + 2 * i, n - i);
}

AVX2_FN void mixAddAVX2(float *dst, const float *src, size_t n, float gain) {
  const __m256 g = _mm256_set1_ps(gain);
  size_t i = 0;
  for (; i + 16 <= n; i += 16) {
    const __m256 s0 = _mm256_mul_ps(_mm256_loadu_ps(src + i), g);
    const __m256 s1 = _mm256_mul_ps(_mm256_loadu_ps(src + i + 8), g);
    _mm256_storeu_ps(dst + i, _mm256_add_ps(_mm256_loadu_ps(dst + i), s0));
    _mm256_storeu_ps(dst + i + 8, _mm256_add_ps(_mm256_loadu_ps(dst + i + 8), s1));
  }
  scalar::mixAddF32(dst + i, src + i, n - i, gain);
}

AVX2_FN void s16ToF32AVX2(float *dst, const uint8_t *src, size_t n,
                          float gain) {
  const __m256 g = _mm256_set1_ps(gain);
  size_t i = 0;
  for (; i + 16 <= n; i += 16) {
    const __m128i a = _mm_loadu_si128(reinterpret_cast<const __m128i *>(src + 2 * i));
    const __m128i b =
        _mm_loadu_si128(reinterpret_cast<const __m128i *>(src + 2 * i + 16));
    _mm256_storeu_ps(dst + i,
                     _mm256_mul_ps(_mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(a)), g));
    _mm256_storeu_ps(dst + i + 8,
                     _mm256_mul_ps(_mm256_cvtepi32_ps(_mm256_cvtepi16_epi32(b)), g));
  }
  scalar::s16ToF32(dst + i, src + 2 * i, n - i, gain);
}

AVX2_FN void u8ToF32AVX2(float *dst, const uint8_t *src, size_t n, float gain) {
  const __m256 g = _mm256_set1_ps(gain);
  const __m256 bias = _mm256_set1_ps(128.0f);
  size_t i = 0;
  for (; i + 16 <= n; i += 16) {
    const __m128i v = _mm_loadu_si128(reinterpret_cast<const __m128i *>(src + i));
    const __m256 a = _mm256_cvtepi32_ps(_mm256_cvtepu8_epi32(v));
    const __m256 b = _mm256_cvtepi32_ps(_mm256_cvtepu8_epi32(_mm_srli_si128(v, 8)));
    _mm256_storeu_ps(dst + i, _mm256_mul_ps(_mm256_sub_ps(a, bias), g));
    _mm256_storeu_ps(dst + i + 8, _mm256_mul_ps(_mm256_sub_ps(b, bias), g));
  }
  scalar::u8ToF32(dst + i, src + i, n - i, gain);
}

AVX2_FN float f32ToS16AVX2(int16_t *dst, const float *src, size_t n) {
  const __m256 lo = _mm256_set1_ps(-1.0f), hi = _mm256_set1_ps(1.0f);
  const __m256 scale = _mm256_set1_ps(32767.0f);
  const __m256 absMask = _mm256_castsi256_ps(_mm256_set1_epi32(0x7FFFFFFF));
  __m256 peak = _mm256_setzero_ps();
  size_t i = 0;
  for (; i + 16 <= n; i += 16) {
    const __m256 a = _mm256_min_ps(_mm256_max_ps(_mm256_loadu_ps(src + i), lo), hi);
    const __m256 b =
        _mm256_min_ps(_mm256_max_ps(_mm256_loadu_ps(src + i + 8), lo), hi);
    peak = _mm256_max_ps(peak, _mm256_and_ps(a, absMask));
    peak = _mm256_max_ps(peak, _mm256_and_ps(b, absMask));
    const __m256i ia = _mm256_cvtps_epi32(_mm256_mul_ps(a, scale));
    const __m256i ib = _mm256_cvtps_epi32(_mm256_mul_ps(b, scale));
    // packs works per 128-bit lane: a0-3 b0-3 | a4-7 b4-7 → reorder.
    const __m256i packed =
        _mm256_permute4x64_epi64(_mm256_packs_epi32(ia, ib), 0xD8);
    storeu256(dst + i, packed);
  }
  float lanes[8];
  _mm256_storeu_ps(lanes, peak);
  float p = scalar::f32ToS16(dst + i, src + i, n - i);
  for (float l : lanes)
    p = l > p ? l : p;
  return p;
}

AVX2_FN void fillPattern32AVX2(uint8_t *dst, size_t bytes, uint32_t pattern) {
  if (bytes < 128 || ((pattern ^ (pattern >> 8)) & 0x00FFFFFFu) == 0) {
    scalar::fillPattern32(dst, bytes, pattern);
    return;
  }
  const __m256i v = _mm256_set1_epi32(static_cast<int>(pattern));
  size_t i = 0;
  for (; i + 128 <= bytes; i += 128) {
    storeu256(dst + i, v);
    storeu256(dst + i + 32, v);
    storeu256(dst + i + 64, v);
    storeu256(dst + i + 96, v);
  }
  for (; i + 32 <= bytes; i += 32)
    storeu256(dst + i, v);
  scalar::fillPattern32(dst + i, bytes - i, pattern);
}

#undef AVX2_FN

} // namespace

void installAVX2(Kernels &k) {
  k.planarToIndex = planarToIndexAVX2;
  k.rgb555ToArgb = rgb555AVX2;
  k.rgb565ToArgb = rgb565AVX2;
  k.mixAddF32 = mixAddAVX2;
  k.s16ToF32 = s16ToF32AVX2;
  k.u8ToF32 = u8ToF32AVX2;
  k.f32ToS16 = f32ToS16AVX2;
  k.fillPattern32 = fillPattern32AVX2;
}

} // namespace fador::utils::simd::detail

#endif // FADOR_SIMD_X86
