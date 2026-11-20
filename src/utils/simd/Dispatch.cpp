#include "KernelsInternal.hpp"

#include "../Logger.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdlib>
#include <string>

namespace fador::utils::simd {

namespace {

constexpr size_t kLevels = 5;

struct Tables {
  std::array<Kernels, kLevels> table{};
  std::array<bool, kLevels> usable{};
};

Tables buildTables() {
  Tables t;
  for (size_t i = 0; i < kLevels; ++i) {
    const auto level = static_cast<SimdLevel>(i);
    Kernels &k = t.table[i];
    detail::installScalar(k);
    t.usable[i] = simdLevelSupported(level);
#ifdef FADOR_SIMD_X86
    if (level == SimdLevel::SSE2 || level == SimdLevel::SSE41 ||
        level == SimdLevel::AVX2)
      detail::installSSE2(k);
    if (level == SimdLevel::SSE41 || level == SimdLevel::AVX2)
      detail::installSSE41(k);
    if (level == SimdLevel::AVX2)
      detail::installAVX2(k);
    if (level != SimdLevel::Scalar && level != SimdLevel::NEON &&
        hostCpuFeatures().erms)
      detail::installErms(k);
#else
    if (level != SimdLevel::Scalar && level != SimdLevel::NEON)
      t.usable[i] = false;
#endif
#ifdef FADOR_SIMD_NEON
    if (level == SimdLevel::NEON)
      detail::installNEON(k);
#else
    if (level == SimdLevel::NEON)
      t.usable[i] = false;
#endif
  }
  return t;
}

const Tables &tables() {
  static const Tables t = buildTables();
  return t;
}

std::atomic<const Kernels *> g_active{nullptr};
std::atomic<SimdLevel> g_level{SimdLevel::Scalar};

SimdLevel usableAtOrBelow(SimdLevel level) {
  const auto &t = tables();
  if (t.usable[static_cast<size_t>(level)])
    return level;
  if (level == SimdLevel::NEON)
    return SimdLevel::Scalar;
  for (int i = static_cast<int>(level); i >= 0; --i)
    if (i != static_cast<int>(SimdLevel::NEON) && t.usable[static_cast<size_t>(i)])
      return static_cast<SimdLevel>(i);
  return SimdLevel::Scalar;
}

SimdLevel levelFromEnvironment() {
  if (const char *env = std::getenv("FADOR_HOST_SIMD")) {
    if (auto l = parseSimdLevel(env))
      return *l;
    LOG_WARN("Ignoring unknown FADOR_HOST_SIMD value '", env, "'");
  }
  return bestSimdLevel();
}

const Kernels &selectDefault() {
  setActiveLevel(levelFromEnvironment());
  return *g_active.load(std::memory_order_acquire);
}

} // namespace

const Kernels &kernels() {
  const Kernels *k = g_active.load(std::memory_order_acquire);
  if (k == nullptr) [[unlikely]]
    return selectDefault();
  return *k;
}

const Kernels *kernelsFor(SimdLevel level) {
  const auto &t = tables();
  const auto i = static_cast<size_t>(level);
  return i < kLevels && t.usable[i] ? &t.table[i] : nullptr;
}

SimdLevel setActiveLevel(SimdLevel level) {
  const SimdLevel chosen = usableAtOrBelow(level);
  g_level.store(chosen, std::memory_order_relaxed);
  g_active.store(&tables().table[static_cast<size_t>(chosen)],
                 std::memory_order_release);
  return chosen;
}

SimdLevel activeLevel() {
  (void)kernels();
  return g_level.load(std::memory_order_relaxed);
}

bool initHostSimd(const char *request) {
  SimdLevel wanted;
  std::string source = "auto";
  if (request != nullptr && *request != '\0') {
    auto l = parseSimdLevel(request);
    if (!l)
      return false;
    wanted = *l;
    source = std::string("--host-simd=") + request;
  } else {
    wanted = levelFromEnvironment();
    if (std::getenv("FADOR_HOST_SIMD"))
      source = "FADOR_HOST_SIMD";
  }
  const SimdLevel chosen = setActiveLevel(wanted);
  const auto &f = hostCpuFeatures();
  std::string cpu = f.vendor;
  if (!f.brand.empty())
    cpu += " (" + f.brand + ")";
  if (chosen != wanted)
    LOG_WARN("Host SIMD level ", simdLevelName(wanted),
             " is not supported here; using ", simdLevelName(chosen));
  LOG_INFO("Host CPU: ", cpu, " features: ", hostCpuFeatureString(),
           "; SIMD kernels: ", simdLevelName(chosen), " (", source, ")");
  return true;
}

void vgaUnchainedRow(uint8_t *dst, const uint8_t *const planes[4],
                     uint32_t start, uint32_t phase, size_t n) {
  start += phase >> 2;
  phase &= 3;
  size_t i = 0;
  // Leading pixels until the line reaches plane 0.
  while (phase != 0 && i < n) {
    dst[i++] = planes[phase][start & 0xFFFF];
    if (++phase == 4) {
      phase = 0;
      ++start;
    }
  }
  const Kernels &k = kernels();
  while (i < n) {
    const uint32_t off = start & 0xFFFF;
    const size_t pix = std::min<size_t>(n - i, (0x10000u - off) * size_t{4});
    const uint8_t *p[4] = {planes[0] + off, planes[1] + off, planes[2] + off,
                           planes[3] + off};
    k.interleave4(dst + i, p, pix);
    i += pix;
    start += static_cast<uint32_t>(pix / 4);
  }
}

void vgaPlanarRow(uint8_t *dst, const uint8_t *const planes[4], uint32_t start,
                  size_t n) {
  const Kernels &k = kernels();
  uint32_t off = start & 0xFFFF;
  size_t i = 0;
  while (i < n) {
    const size_t pix = std::min<size_t>(n - i, (0x10000u - off) * size_t{8});
    const uint8_t *p[4] = {planes[0] + off, planes[1] + off, planes[2] + off,
                           planes[3] + off};
    k.planarToIndex(dst + i, p, pix);
    i += pix;
    off = 0;
  }
}

} // namespace fador::utils::simd
