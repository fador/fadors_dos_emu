#pragma once

// Host-SIMD kernels with run-time dispatch.
//
// Every kernel has a portable scalar reference implementation
// (KernelsScalar.cpp) and optional SSE2 / SSE4 / AVX2 / NEON variants
// (Kernels<ISA>.cpp) that are compiled with per-function target attributes,
// so the build itself stays at the baseline ISA.  The active table is chosen
// once (bestSimdLevel(), --host-simd=..., or the FADOR_HOST_SIMD environment
// variable) and every variant must produce bit-identical output to the
// scalar one (tests/test_host_simd.cpp checks this on random inputs).
//
// Byte pointers need no alignment; uint32_t / float / int16_t pointers only
// their natural alignment (no 16/32-byte alignment is assumed anywhere).
// Pixel/sample counts may be any value, including 0.  Source and
// destination ranges must not overlap.

#include "../HostCpu.hpp"

#include <cstddef>
#include <cstdint>

namespace fador::utils::simd {

struct Kernels {
  // ── Video ──────────────────────────────────────────────────────────
  // dst[i] = palette[src[i]]                        (8bpp indexed → ARGB)
  void (*paletteToArgb)(uint32_t *dst, const uint8_t *src, size_t n,
                        const uint32_t *palette);
  // dst[2i] = dst[2i+1] = palette[src[i]]  (writes 2n pixels; doubled width)
  void (*paletteToArgbX2)(uint32_t *dst, const uint8_t *src, size_t n,
                          const uint32_t *palette);
  // Unchained 256-colour (Mode X / chain-4) plane interleave:
  //   dst[i] = planes[i & 3][i >> 2]                (reads ceil(n/4) bytes)
  void (*interleave4)(uint8_t *dst, const uint8_t *const planes[4], size_t n);
  // 16-colour planar → one colour index per byte:
  //   dst[i] = sum_p ((planes[p][i >> 3] >> (7 - (i & 7))) & 1) << p
  void (*planarToIndex)(uint8_t *dst, const uint8_t *const planes[4],
                        size_t n);
  // VBE direct colour → 0xFFRRGGBB (sources are little-endian pixels)
  void (*rgb555ToArgb)(uint32_t *dst, const uint8_t *src, size_t n);
  void (*rgb565ToArgb)(uint32_t *dst, const uint8_t *src, size_t n);
  void (*rgb24ToArgb)(uint32_t *dst, const uint8_t *src, size_t n);  // B,G,R
  void (*rgb32ToArgb)(uint32_t *dst, const uint8_t *src, size_t n);  // B,G,R,x

  // ── Audio (n = number of floats / samples) ─────────────────────────
  // dst[i] = dst[i] + src[i] * gain
  void (*mixAddF32)(float *dst, const float *src, size_t n, float gain);
  // dst[i] = float(int16le(src + 2i)) * gain
  void (*s16ToF32)(float *dst, const uint8_t *src, size_t n, float gain);
  // dst[i] = (float(src[i]) - 128) * gain                 (unsigned 8-bit)
  void (*u8ToF32)(float *dst, const uint8_t *src, size_t n, float gain);
  // v = clamp(src[i], -1, 1) (NaN → -1); dst[i] = lrint(v * 32767).
  // Returns max |v| over the block (0 for n == 0).
  float (*f32ToS16)(int16_t *dst, const float *src, size_t n);

  // ── Memory ─────────────────────────────────────────────────────────
  // Repeats the little-endian 4-byte pattern over dst[0..bytes): byte i
  // is (pattern >> (8 * (i & 3))) & 0xFF.  Used by REP STOSB/W/D.
  void (*fillPattern32)(uint8_t *dst, size_t bytes, uint32_t pattern);
};

// The active kernel table.  Selected on first use from FADOR_HOST_SIMD
// (when set) or the best level the host supports.
const Kernels &kernels();

// Kernel table of a specific level, or nullptr when the host cannot run
// it.  For tests and benchmarks.
const Kernels *kernelsFor(SimdLevel level);

// Selects the active level; an unsupported level falls back to the best
// supported one below it.  Returns the level actually selected.
SimdLevel setActiveLevel(SimdLevel level);
SimdLevel activeLevel();

// One-time startup hook: applies an explicit request ("auto", "avx2", ...;
// empty = FADOR_HOST_SIMD or auto) and logs the host features and the
// chosen level.  Returns false for an unknown level name.
bool initHostSimd(const char *request);

// ── VGA helpers built on the kernels ─────────────────────────────────
// Plane offsets wrap at 64 KiB like the VGA's CRTC address counter.
// `planes` point at the start of the four 64 KiB planes.

// Chain-4 / unchained 256-colour scanline: pixel i of the line comes from
// plane (phase + i) & 3 at offset start + ((phase + i) >> 2).
void vgaUnchainedRow(uint8_t *dst, const uint8_t *const planes[4],
                     uint32_t start, uint32_t phase, size_t n);
// 16-colour planar scanline starting at bit 7 of byte `start`.
void vgaPlanarRow(uint8_t *dst, const uint8_t *const planes[4],
                  uint32_t start, size_t n);

} // namespace fador::utils::simd
