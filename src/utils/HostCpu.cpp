#include "HostCpu.hpp"

#include <array>
#include <initializer_list>
#include <utility>

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) ||            \
    defined(_M_IX86)
#define FADOR_HOST_X86 1
#if defined(_MSC_VER) && !defined(__clang__)
#include <immintrin.h>
#include <intrin.h>
#else
#include <cpuid.h>
#endif
#endif

namespace fador::utils {

namespace {

#ifdef FADOR_HOST_X86
struct CpuidRegs {
  uint32_t eax = 0, ebx = 0, ecx = 0, edx = 0;
};

CpuidRegs cpuid(uint32_t leaf, uint32_t subleaf) {
  CpuidRegs r;
#if defined(_MSC_VER) && !defined(__clang__)
  int regs[4] = {};
  __cpuidex(regs, static_cast<int>(leaf), static_cast<int>(subleaf));
  r.eax = static_cast<uint32_t>(regs[0]);
  r.ebx = static_cast<uint32_t>(regs[1]);
  r.ecx = static_cast<uint32_t>(regs[2]);
  r.edx = static_cast<uint32_t>(regs[3]);
#else
  unsigned a = 0, b = 0, c = 0, d = 0;
  if (__get_cpuid_count(leaf, subleaf, &a, &b, &c, &d)) {
    r.eax = a;
    r.ebx = b;
    r.ecx = c;
    r.edx = d;
  }
#endif
  return r;
}

uint32_t maxLeaf(uint32_t base) { return cpuid(base, 0).eax; }

// XCR0: which register states the OS saves on context switches.
uint64_t readXcr0() {
#if defined(_MSC_VER) && !defined(__clang__)
  return _xgetbv(0);
#else
  uint32_t lo = 0, hi = 0;
  __asm__ volatile("xgetbv" : "=a"(lo), "=d"(hi) : "c"(0));
  return (static_cast<uint64_t>(hi) << 32) | lo;
#endif
}

bool bit(uint32_t v, int n) { return ((v >> n) & 1u) != 0; }

// Appends the bytes of CPUID registers (vendor / brand string pieces).
void appendRegs(std::string &s, std::initializer_list<uint32_t> regs) {
  for (uint32_t v : regs)
    for (int i = 0; i < 4; ++i) {
      const char c = static_cast<char>((v >> (8 * i)) & 0xFF);
      if (c != 0)
        s += c;
    }
}

HostCpuFeatures detect() {
  HostCpuFeatures f;
  const uint32_t top = maxLeaf(0);
  const CpuidRegs v = cpuid(0, 0);
  appendRegs(f.vendor, {v.ebx, v.edx, v.ecx});

  if (top >= 1) {
    const CpuidRegs l1 = cpuid(1, 0);
    f.sse2 = bit(l1.edx, 26);
    f.sse3 = bit(l1.ecx, 0);
    f.ssse3 = bit(l1.ecx, 9);
    f.fma = bit(l1.ecx, 12);
    f.sse41 = bit(l1.ecx, 19);
    f.sse42 = bit(l1.ecx, 20);
    f.popcnt = bit(l1.ecx, 23);
    const bool osxsave = bit(l1.ecx, 27);
    const bool avxCpu = bit(l1.ecx, 28);
    uint64_t xcr0 = osxsave ? readXcr0() : 0;
    const bool ymmState = (xcr0 & 0x6) == 0x6;     // XMM + YMM
    const bool zmmState = (xcr0 & 0xE6) == 0xE6;   // + opmask, ZMM
    f.avx = avxCpu && ymmState;
    f.fma = f.fma && f.avx;
    if (top >= 7) {
      const CpuidRegs l7 = cpuid(7, 0);
      f.bmi1 = bit(l7.ebx, 3);
      f.avx2 = f.avx && bit(l7.ebx, 5);
      f.bmi2 = bit(l7.ebx, 8);
      f.erms = bit(l7.ebx, 9);
      f.avx512f = zmmState && bit(l7.ebx, 16);
      f.avx512bw = f.avx512f && bit(l7.ebx, 30);
      f.fsrm = bit(l7.edx, 4);
    }
  }

  if (maxLeaf(0x80000000u) >= 0x80000004u) {
    for (uint32_t leaf = 0x80000002u; leaf <= 0x80000004u; ++leaf) {
      const CpuidRegs r = cpuid(leaf, 0);
      appendRegs(f.brand, {r.eax, r.ebx, r.ecx, r.edx});
    }
    const auto first = f.brand.find_first_not_of(' ');
    f.brand = first == std::string::npos ? std::string() : f.brand.substr(first);
    while (!f.brand.empty() && f.brand.back() == ' ')
      f.brand.pop_back();
  }
#if defined(__x86_64__) || defined(_M_X64)
  f.sse2 = true; // architectural on x86-64
#endif
  return f;
}
#else
HostCpuFeatures detect() {
  HostCpuFeatures f;
#if defined(__aarch64__) || defined(_M_ARM64)
  f.vendor = "ARM";
  f.neon = true;
#else
  f.vendor = "unknown";
#endif
  return f;
}
#endif

} // namespace

const HostCpuFeatures &hostCpuFeatures() {
  static const HostCpuFeatures features = detect();
  return features;
}

std::string hostCpuFeatureString() {
  const auto &f = hostCpuFeatures();
  const std::array<std::pair<bool, const char *>, 16> list = {{
      {f.sse2, "sse2"},
      {f.sse3, "sse3"},
      {f.ssse3, "ssse3"},
      {f.sse41, "sse4.1"},
      {f.sse42, "sse4.2"},
      {f.popcnt, "popcnt"},
      {f.avx, "avx"},
      {f.avx2, "avx2"},
      {f.fma, "fma"},
      {f.bmi1, "bmi1"},
      {f.bmi2, "bmi2"},
      {f.avx512f, "avx512f"},
      {f.avx512bw, "avx512bw"},
      {f.erms, "erms"},
      {f.fsrm, "fsrm"},
      {f.neon, "neon"},
  }};
  std::string s;
  for (const auto &[present, name] : list) {
    if (!present)
      continue;
    if (!s.empty())
      s += ' ';
    s += name;
  }
  return s.empty() ? std::string("none") : s;
}

bool simdLevelSupported(SimdLevel level) {
  const auto &f = hostCpuFeatures();
  switch (level) {
  case SimdLevel::Scalar:
    return true;
  case SimdLevel::SSE2:
    return f.sse2;
  case SimdLevel::SSE41:
    return f.sse2 && f.ssse3 && f.sse41;
  case SimdLevel::AVX2:
    return f.sse2 && f.ssse3 && f.sse41 && f.avx2;
  case SimdLevel::NEON:
    return f.neon;
  }
  return false;
}

SimdLevel bestSimdLevel() {
  for (SimdLevel l : {SimdLevel::AVX2, SimdLevel::SSE41, SimdLevel::SSE2,
                      SimdLevel::NEON})
    if (simdLevelSupported(l))
      return l;
  return SimdLevel::Scalar;
}

const char *simdLevelName(SimdLevel level) {
  switch (level) {
  case SimdLevel::Scalar:
    return "scalar";
  case SimdLevel::SSE2:
    return "sse2";
  case SimdLevel::SSE41:
    return "sse4";
  case SimdLevel::AVX2:
    return "avx2";
  case SimdLevel::NEON:
    return "neon";
  }
  return "?";
}

std::optional<SimdLevel> parseSimdLevel(std::string_view name) {
  if (name == "auto" || name.empty())
    return bestSimdLevel();
  if (name == "scalar" || name == "none" || name == "off")
    return SimdLevel::Scalar;
  if (name == "sse2")
    return SimdLevel::SSE2;
  if (name == "sse4" || name == "sse4.1" || name == "sse41")
    return SimdLevel::SSE41;
  if (name == "avx2")
    return SimdLevel::AVX2;
  if (name == "neon")
    return SimdLevel::NEON;
  return std::nullopt;
}

} // namespace fador::utils
