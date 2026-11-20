// ARM64 Advanced SIMD kernels (NEON is part of the ARMv8-A baseline, so no
// target attribute is needed).

#include "KernelsInternal.hpp"

#ifdef FADOR_SIMD_NEON

#include <arm_neon.h>

namespace fador::utils::simd::detail {

namespace {

void interleave4NEON(uint8_t *dst, const uint8_t *const planes[4], size_t n) {
  size_t g = 0;
  for (; (g + 16) * 4 <= n; g += 16) {
    uint8x16x4_t v;
    v.val[0] = vld1q_u8(planes[0] + g);
    v.val[1] = vld1q_u8(planes[1] + g);
    v.val[2] = vld1q_u8(planes[2] + g);
    v.val[3] = vld1q_u8(planes[3] + g);
    vst4q_u8(dst + 4 * g, v);
  }
  const uint8_t *rest[4] = {planes[0] + g, planes[1] + g, planes[2] + g,
                            planes[3] + g};
  scalar::interleave4(dst + 4 * g, rest, n - 4 * g);
}

template <bool k565>
inline void hiColor8(uint32_t *dst, uint16x8_t p) {
  const uint16x8_t m5 = vdupq_n_u16(0x1F);
  uint16x8_t r, g;
  if constexpr (k565) {
    r = vshrq_n_u16(p, 11);
    g = vandq_u16(vshrq_n_u16(p, 5), vdupq_n_u16(0x3F));
    g = vorrq_u16(vshlq_n_u16(g, 2), vshrq_n_u16(g, 4));
  } else {
    r = vandq_u16(vshrq_n_u16(p, 10), m5);
    g = vandq_u16(vshrq_n_u16(p, 5), m5);
    g = vorrq_u16(vshlq_n_u16(g, 3), vshrq_n_u16(g, 2));
  }
  uint16x8_t b = vandq_u16(p, m5);
  r = vorrq_u16(vshlq_n_u16(r, 3), vshrq_n_u16(r, 2));
  b = vorrq_u16(vshlq_n_u16(b, 3), vshrq_n_u16(b, 2));
  const uint16x8_t gb = vorrq_u16(vshlq_n_u16(g, 8), b);
  const uint16x8_t ar = vorrq_u16(r, vdupq_n_u16(0xFF00));
  const uint16x8x2_t z = vzipq_u16(gb, ar);
  vst1q_u32(dst, vreinterpretq_u32_u16(z.val[0]));
  vst1q_u32(dst + 4, vreinterpretq_u32_u16(z.val[1]));
}

void rgb555NEON(uint32_t *dst, const uint8_t *src, size_t n) {
  size_t i = 0;
  for (; i + 8 <= n; i += 8)
    hiColor8<false>(dst + i, vreinterpretq_u16_u8(vld1q_u8(src + 2 * i)));
  scalar::rgb555ToArgb(dst + i, src + 2 * i, n - i);
}

void rgb565NEON(uint32_t *dst, const uint8_t *src, size_t n) {
  size_t i = 0;
  for (; i + 8 <= n; i += 8)
    hiColor8<true>(dst + i, vreinterpretq_u16_u8(vld1q_u8(src + 2 * i)));
  scalar::rgb565ToArgb(dst + i, src + 2 * i, n - i);
}

void rgb24NEON(uint32_t *dst, const uint8_t *src, size_t n) {
  size_t i = 0;
  for (; i + 16 <= n; i += 16) {
    const uint8x16x3_t bgr = vld3q_u8(src + 3 * i);
    uint8x16x4_t out;
    out.val[0] = bgr.val[0];
    out.val[1] = bgr.val[1];
    out.val[2] = bgr.val[2];
    out.val[3] = vdupq_n_u8(0xFF);
    vst4q_u8(reinterpret_cast<uint8_t *>(dst + i), out);
  }
  scalar::rgb24ToArgb(dst + i, src + 3 * i, n - i);
}

void rgb32NEON(uint32_t *dst, const uint8_t *src, size_t n) {
  const uint32x4_t alpha = vdupq_n_u32(0xFF000000u);
  size_t i = 0;
  for (; i + 4 <= n; i += 4)
    vst1q_u32(dst + i,
              vorrq_u32(vreinterpretq_u32_u8(vld1q_u8(src + 4 * i)), alpha));
  scalar::rgb32ToArgb(dst + i, src + 4 * i, n - i);
}

void mixAddNEON(float *dst, const float *src, size_t n, float gain) {
  const float32x4_t g = vdupq_n_f32(gain);
  size_t i = 0;
  for (; i + 4 <= n; i += 4) {
    const float32x4_t s = vmulq_f32(vld1q_f32(src + i), g); // no FMA: the
    vst1q_f32(dst + i, vaddq_f32(vld1q_f32(dst + i), s));    // scalar has none
  }
  scalar::mixAddF32(dst + i, src + i, n - i, gain);
}

void s16ToF32NEON(float *dst, const uint8_t *src, size_t n, float gain) {
  const float32x4_t g = vdupq_n_f32(gain);
  size_t i = 0;
  for (; i + 8 <= n; i += 8) {
    const int16x8_t v = vreinterpretq_s16_u8(vld1q_u8(src + 2 * i));
    vst1q_f32(dst + i,
              vmulq_f32(vcvtq_f32_s32(vmovl_s16(vget_low_s16(v))), g));
    vst1q_f32(dst + i + 4,
              vmulq_f32(vcvtq_f32_s32(vmovl_s16(vget_high_s16(v))), g));
  }
  scalar::s16ToF32(dst + i, src + 2 * i, n - i, gain);
}

void u8ToF32NEON(float *dst, const uint8_t *src, size_t n, float gain) {
  const float32x4_t g = vdupq_n_f32(gain);
  const float32x4_t bias = vdupq_n_f32(128.0f);
  size_t i = 0;
  for (; i + 8 <= n; i += 8) {
    const uint16x8_t w = vmovl_u8(vld1_u8(src + i));
    const float32x4_t a = vcvtq_f32_u32(vmovl_u16(vget_low_u16(w)));
    const float32x4_t b = vcvtq_f32_u32(vmovl_u16(vget_high_u16(w)));
    vst1q_f32(dst + i, vmulq_f32(vsubq_f32(a, bias), g));
    vst1q_f32(dst + i + 4, vmulq_f32(vsubq_f32(b, bias), g));
  }
  scalar::u8ToF32(dst + i, src + i, n - i, gain);
}

float f32ToS16NEON(int16_t *dst, const float *src, size_t n) {
  const float32x4_t lo = vdupq_n_f32(-1.0f), hi = vdupq_n_f32(1.0f);
  const float32x4_t scale = vdupq_n_f32(32767.0f);
  float32x4_t peak = vdupq_n_f32(0.0f);
  size_t i = 0;
  for (; i + 8 <= n; i += 8) {
    // FMAXNM/FMINNM return the number when one operand is NaN, matching
    // the scalar "NaN → -1" clamp.
    const float32x4_t a = vminnmq_f32(vmaxnmq_f32(vld1q_f32(src + i), lo), hi);
    const float32x4_t b =
        vminnmq_f32(vmaxnmq_f32(vld1q_f32(src + i + 4), lo), hi);
    peak = vmaxq_f32(peak, vmaxq_f32(vabsq_f32(a), vabsq_f32(b)));
    const int32x4_t ia = vcvtnq_s32_f32(vmulq_f32(a, scale));
    const int32x4_t ib = vcvtnq_s32_f32(vmulq_f32(b, scale));
    vst1q_s16(dst + i, vcombine_s16(vqmovn_s32(ia), vqmovn_s32(ib)));
  }
  const float p = scalar::f32ToS16(dst + i, src + i, n - i);
  const float v = vmaxvq_f32(peak);
  return v > p ? v : p;
}

void fillPattern32NEON(uint8_t *dst, size_t bytes, uint32_t pattern) {
  if (bytes < 64 || ((pattern ^ (pattern >> 8)) & 0x00FFFFFFu) == 0) {
    scalar::fillPattern32(dst, bytes, pattern);
    return;
  }
  const uint8x16_t v = vreinterpretq_u8_u32(vdupq_n_u32(pattern));
  size_t i = 0;
  for (; i + 64 <= bytes; i += 64) {
    vst1q_u8(dst + i, v);
    vst1q_u8(dst + i + 16, v);
    vst1q_u8(dst + i + 32, v);
    vst1q_u8(dst + i + 48, v);
  }
  for (; i + 16 <= bytes; i += 16)
    vst1q_u8(dst + i, v);
  scalar::fillPattern32(dst + i, bytes - i, pattern);
}

} // namespace

void installNEON(Kernels &k) {
  k.interleave4 = interleave4NEON;
  k.rgb555ToArgb = rgb555NEON;
  k.rgb565ToArgb = rgb565NEON;
  k.rgb24ToArgb = rgb24NEON;
  k.rgb32ToArgb = rgb32NEON;
  k.mixAddF32 = mixAddNEON;
  k.s16ToF32 = s16ToF32NEON;
  k.u8ToF32 = u8ToF32NEON;
  k.f32ToS16 = f32ToS16NEON;
  k.fillPattern32 = fillPattern32NEON;
}

} // namespace fador::utils::simd::detail

#endif // FADOR_SIMD_NEON
