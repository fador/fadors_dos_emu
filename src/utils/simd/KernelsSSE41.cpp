// SSSE3 / SSE4.1 kernels ("sse4" level): byte shuffles (PSHUFB).

#include "KernelsInternal.hpp"

#ifdef FADOR_SIMD_X86

#include <immintrin.h>

namespace fador::utils::simd::detail {

namespace {

#define SSE4_FN FADOR_TARGET("ssse3,sse4.1")

// 16 bytes per plane → 128 colour indices.  Each pixel byte is the plane
// byte broadcast with PSHUFB, tested against its bit and weighted by the
// plane number.
SSE4_FN void planarToIndexSSE4(uint8_t *dst, const uint8_t *const planes[4],
                               size_t n) {
  const __m128i bitMask = _mm_setr_epi8(
      static_cast<char>(0x80), 0x40, 0x20, 0x10, 8, 4, 2, 1,
      static_cast<char>(0x80), 0x40, 0x20, 0x10, 8, 4, 2, 1);
  const __m128i weight[4] = {_mm_set1_epi8(1), _mm_set1_epi8(2),
                             _mm_set1_epi8(4), _mm_set1_epi8(8)};
  size_t j = 0; // plane byte index
  for (; (j + 16) * 8 <= n; j += 16) {
    __m128i v[4];
    for (int p = 0; p < 4; ++p)
      v[p] = _mm_loadu_si128(reinterpret_cast<const __m128i *>(planes[p] + j));
    for (int k = 0; k < 8; ++k) {
      // bytes 2k (x8) and 2k+1 (x8)
      const __m128i shuf = _mm_add_epi8(
          _mm_setr_epi8(0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1),
          _mm_set1_epi8(static_cast<char>(2 * k)));
      __m128i acc = _mm_setzero_si128();
      for (int p = 0; p < 4; ++p) {
        const __m128i e = _mm_and_si128(_mm_shuffle_epi8(v[p], shuf), bitMask);
        acc = _mm_or_si128(acc, _mm_and_si128(_mm_cmpeq_epi8(e, bitMask),
                                              weight[p]));
      }
      _mm_storeu_si128(reinterpret_cast<__m128i *>(dst + 8 * j + 16 * k), acc);
    }
  }
  const uint8_t *rest[4] = {planes[0] + j, planes[1] + j, planes[2] + j,
                            planes[3] + j};
  scalar::planarToIndex(dst + 8 * j, rest, n - 8 * j);
}

SSE4_FN void rgb24SSE4(uint32_t *dst, const uint8_t *src, size_t n) {
  const __m128i shuf = _mm_setr_epi8(0, 1, 2, -1, 3, 4, 5, -1, 6, 7, 8, -1,
                                     9, 10, 11, -1);
  const __m128i alpha = _mm_set1_epi32(static_cast<int>(0xFF000000u));
  size_t i = 0;
  // Each 16-byte load covers 4 pixels plus 4 spare bytes, so stop while
  // the spare bytes are still inside the source row.
  for (; i + 18 <= n; i += 16)
    for (int k = 0; k < 4; ++k) {
      const __m128i v = _mm_loadu_si128(
          reinterpret_cast<const __m128i *>(src + 3 * (i + 4 * k)));
      _mm_storeu_si128(reinterpret_cast<__m128i *>(dst + i + 4 * k),
                       _mm_or_si128(_mm_shuffle_epi8(v, shuf), alpha));
    }
  for (; i + 6 <= n; i += 4) {
    const __m128i v = _mm_loadu_si128(reinterpret_cast<const __m128i *>(src + 3 * i));
    _mm_storeu_si128(reinterpret_cast<__m128i *>(dst + i),
                     _mm_or_si128(_mm_shuffle_epi8(v, shuf), alpha));
  }
  scalar::rgb24ToArgb(dst + i, src + 3 * i, n - i);
}

#undef SSE4_FN

} // namespace

void installSSE41(Kernels &k) {
  k.planarToIndex = planarToIndexSSE4;
  k.rgb24ToArgb = rgb24SSE4;
}

} // namespace fador::utils::simd::detail

#endif // FADOR_SIMD_X86
