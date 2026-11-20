#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace fador::utils {

// Instruction-set features of the machine the emulator runs on (not of
// the emulated CPU).  Detected once, on first use, with CPUID/XGETBV on
// x86 and from the target architecture on ARM64.  Used to pick the
// host-SIMD kernels in utils/simd (frame conversion, audio, bulk fills).
struct HostCpuFeatures {
  std::string vendor; // "GenuineIntel", "AuthenticAMD", "ARM", ...
  std::string brand;  // processor brand string when available

  // x86 (all false elsewhere).  avx/avx2/avx512* are only set when the
  // operating system also saves the wider register state (XGETBV).
  bool sse2 = false;
  bool sse3 = false;
  bool ssse3 = false;
  bool sse41 = false;
  bool sse42 = false;
  bool popcnt = false;
  bool avx = false;
  bool avx2 = false;
  bool fma = false;
  bool bmi1 = false;
  bool bmi2 = false;
  bool avx512f = false;
  bool avx512bw = false;
  bool erms = false; // enhanced REP MOVSB/STOSB
  bool fsrm = false; // fast short REP MOVSB

  // ARM64: Advanced SIMD is part of the baseline.
  bool neon = false;
};

// Dispatch levels, lowest to highest.  A level is usable when the host
// has every feature it needs (see simdLevelSupported).
enum class SimdLevel : uint8_t {
  Scalar = 0, // portable C++ only
  SSE2,       // x86-64 baseline
  SSE41,      // SSSE3 + SSE4.1 (option name "sse4")
  AVX2,       // AVX2 (with OS support for the YMM state)
  NEON,       // ARM64 Advanced SIMD
};

const HostCpuFeatures &hostCpuFeatures();

// Space-separated list of the detected features, e.g. "sse2 ssse3 avx2".
std::string hostCpuFeatureString();

bool simdLevelSupported(SimdLevel level);
SimdLevel bestSimdLevel();

// "scalar", "sse2", "sse4", "avx2", "neon".
const char *simdLevelName(SimdLevel level);

// Parses a --host-simd / FADOR_HOST_SIMD value.  "auto" yields
// bestSimdLevel(); unknown names yield std::nullopt.
std::optional<SimdLevel> parseSimdLevel(std::string_view name);

} // namespace fador::utils
