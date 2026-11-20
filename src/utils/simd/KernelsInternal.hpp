#pragma once

// Internal to src/utils/simd: per-ISA installers and target macros.

#include "Simd.hpp"

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) ||            \
    defined(_M_IX86)
#define FADOR_SIMD_X86 1
#endif

#if defined(__aarch64__) || defined(_M_ARM64)
#define FADOR_SIMD_NEON 1
#endif

// Per-function ISA selection: GCC and Clang compile a function for the
// named extension without raising the ISA of the rest of the program.
// MSVC accepts every intrinsic without flags.
#if defined(__GNUC__) || defined(__clang__)
#define FADOR_TARGET(isa) __attribute__((target(isa)))
#else
#define FADOR_TARGET(isa)
#endif

namespace fador::utils::simd::detail {

// Scalar reference kernels (KernelsScalar.cpp).  SIMD variants call them
// for the tail elements that do not fill a whole vector.
namespace scalar {
void paletteToArgb(uint32_t *dst, const uint8_t *src, size_t n,
                   const uint32_t *palette);
void paletteToArgbX2(uint32_t *dst, const uint8_t *src, size_t n,
                     const uint32_t *palette);
void interleave4(uint8_t *dst, const uint8_t *const planes[4], size_t n);
void planarToIndex(uint8_t *dst, const uint8_t *const planes[4], size_t n);
void rgb555ToArgb(uint32_t *dst, const uint8_t *src, size_t n);
void rgb565ToArgb(uint32_t *dst, const uint8_t *src, size_t n);
void rgb24ToArgb(uint32_t *dst, const uint8_t *src, size_t n);
void rgb32ToArgb(uint32_t *dst, const uint8_t *src, size_t n);
void mixAddF32(float *dst, const float *src, size_t n, float gain);
void s16ToF32(float *dst, const uint8_t *src, size_t n, float gain);
void u8ToF32(float *dst, const uint8_t *src, size_t n, float gain);
float f32ToS16(int16_t *dst, const float *src, size_t n);
void fillPattern32(uint8_t *dst, size_t bytes, uint32_t pattern);
} // namespace scalar

// Each installer overwrites the entries it accelerates; the dispatcher
// starts from the scalar table and applies the installers in ISA order
// (SSE2 → SSE4 → AVX2), so a level inherits everything below it.
void installScalar(Kernels &k);
#ifdef FADOR_SIMD_X86
void installSSE2(Kernels &k);
void installSSE41(Kernels &k);
void installAVX2(Kernels &k);
// Feature-based override on top of an x86 level: REP STOSD fills on CPUs
// with ERMS (enhanced REP MOVSB/STOSB).
void installErms(Kernels &k);
#endif
#ifdef FADOR_SIMD_NEON
void installNEON(Kernels &k);
#endif

} // namespace fador::utils::simd::detail
