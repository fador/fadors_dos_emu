#include "HostSimdBench.hpp"

#include "../hw/VGAController.hpp"
#include "../memory/MemoryBus.hpp"
#include "../utils/HostCpu.hpp"
#include "../utils/simd/Simd.hpp"
#include "FrameRenderer.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <random>
#include <vector>

namespace fador::ui {

namespace {

using utils::SimdLevel;
namespace simd = utils::simd;

constexpr SimdLevel kLevels[] = {SimdLevel::Scalar, SimdLevel::SSE2,
                                 SimdLevel::SSE41, SimdLevel::AVX2,
                                 SimdLevel::NEON};

// Times `fn` at every supported level and prints one row per level.  The
// levels take turns over several rounds and the fastest batch of each is
// kept, so a busy shared host affects all levels alike.  `units` is the
// work per call (pixels, samples, bytes), `unitName` its throughput label.
void runLevels(const char *bench, const char *variant, double units,
               const char *unitName, const std::function<void()> &fn) {
  using clock = std::chrono::steady_clock;
  auto timeBatch = [&](size_t iters) {
    const auto t0 = clock::now();
    for (size_t i = 0; i < iters; ++i)
      fn();
    return std::chrono::duration<double, std::nano>(clock::now() - t0).count() /
           static_cast<double>(iters);
  };
  const SimdLevel saved = simd::activeLevel();
  std::vector<SimdLevel> levels;
  for (SimdLevel level : kLevels)
    if (simd::kernelsFor(level))
      levels.push_back(level);

  // Batch size: ~5 ms of scalar work.
  simd::setActiveLevel(SimdLevel::Scalar);
  size_t iters = 1;
  while (timeBatch(iters) * static_cast<double>(iters) < 5e6 &&
         iters < (size_t{1} << 30))
    iters *= 2;

  std::vector<double> best(levels.size(), 1e300);
  for (int round = 0; round < 15; ++round)
    for (size_t i = 0; i < levels.size(); ++i) {
      simd::setActiveLevel(levels[i]);
      best[i] = std::min(best[i], timeBatch(iters));
    }
  for (size_t i = 0; i < levels.size(); ++i)
    std::printf("%-14s %-10s %-7s %12.1f ns/call %10.1f M%s/s   x%.2f\n", bench,
                variant, utils::simdLevelName(levels[i]), best[i],
                units / best[i] * 1e3, unitName, best[0] / best[i]);
  simd::setActiveLevel(saved);
}

struct VgaRig {
  memory::MemoryBus memory;
  hw::VGAController vga{memory};
  FrameRenderer frame{memory, vga};

  explicit VgaRig(uint8_t mode, bool chain4) {
    memory.setVGA(&vga);
    std::mt19937 rng(1);
    uint8_t *dac = memory.dacPalette();
    for (uint32_t i = 0; i < memory::MemoryBus::DAC_SIZE; ++i)
      dac[i] = static_cast<uint8_t>(rng() & 0x3F);
    seq(4, 0x06); // unchained while filling the planes
    for (int p = 0; p < 4; ++p) {
      seq(2, static_cast<uint8_t>(1u << p));
      for (uint32_t off = 0; off < 0x10000; ++off)
        vga.planeWrite8(off, static_cast<uint8_t>(rng()));
    }
    seq(2, 0x0F);
    seq(4, chain4 ? 0x0E : 0x06);
    memory.write8(0x449, mode);
  }
  void seq(uint8_t index, uint8_t value) {
    vga.write8(0x3C4, index);
    vga.write8(0x3C5, value);
  }
};

void benchRender(const char *name, uint8_t mode, bool chain4) {
  VgaRig rig(mode, chain4);
  rig.frame.render(false);
  const double pixels = static_cast<double>(rig.frame.width()) * rig.frame.height();
  runLevels(name, "frame", pixels, "pix", [&] { rig.frame.render(false); });
}

void benchVbe() {
  constexpr int kW = 640, kH = 480;
  std::vector<uint8_t> fb(static_cast<size_t>(kW) * kH * 4);
  std::mt19937 rng(2);
  for (auto &b : fb)
    b = static_cast<uint8_t>(rng());
  std::vector<uint32_t> out(static_cast<size_t>(kW) * kH);
  std::vector<uint32_t> pal(256);
  for (auto &p : pal)
    p = static_cast<uint32_t>(rng());
  struct Variant {
    const char *name;
    int bytesPP;
  };
  for (const Variant v : {Variant{"8bpp", 1}, Variant{"15bpp", 2},
                          Variant{"16bpp", 2}, Variant{"24bpp", 3},
                          Variant{"32bpp", 4}}) {
    const size_t pitch = static_cast<size_t>(kW) * v.bytesPP;
    runLevels("render-vbe", v.name, kW * kH, "pix", [&] {
      const auto &k = simd::kernels();
      for (int y = 0; y < kH; ++y) {
        const uint8_t *src = fb.data() + y * pitch;
        uint32_t *dst = out.data() + static_cast<size_t>(y) * kW;
        switch (v.bytesPP) {
        case 1:
          k.paletteToArgb(dst, src, kW, pal.data());
          break;
        case 2:
          if (v.name[0] == '1' && v.name[1] == '5')
            k.rgb555ToArgb(dst, src, kW);
          else
            k.rgb565ToArgb(dst, src, kW);
          break;
        case 3:
          k.rgb24ToArgb(dst, src, kW);
          break;
        default:
          k.rgb32ToArgb(dst, src, kW);
          break;
        }
      }
    });
  }
}

void benchAudio() {
  // One mixer period: 4096 stereo frames.
  constexpr size_t kSamples = 8192;
  std::mt19937 rng(3);
  std::uniform_real_distribution<float> dist(-1.2f, 1.2f);
  std::vector<float> a(kSamples), b(kSamples);
  for (auto &v : a)
    v = dist(rng);
  for (auto &v : b)
    v = dist(rng);
  std::vector<uint8_t> raw(kSamples * 2);
  for (auto &v : raw)
    v = static_cast<uint8_t>(rng());
  std::vector<int16_t> pcm(kSamples);
  float sink = 0.0f;
  runLevels("audio-mix", "mix-add", kSamples, "smp",
            [&] { simd::kernels().mixAddF32(b.data(), a.data(), kSamples, 1e-6f); });
  runLevels("audio-mix", "s16->f32", kSamples, "smp", [&] {
    simd::kernels().s16ToF32(b.data(), raw.data(), kSamples, 1.0f / 32768.0f);
  });
  runLevels("audio-mix", "u8->f32", kSamples, "smp", [&] {
    simd::kernels().u8ToF32(b.data(), raw.data(), kSamples, 1.0f / 128.0f);
  });
  runLevels("audio-mix", "f32->s16", kSamples, "smp", [&] {
    sink += simd::kernels().f32ToS16(pcm.data(), a.data(), kSamples);
  });
  if (sink < 0.0f)
    std::printf("unreachable\n");
}

void benchFill() {
  // One REP STOSD batch of the decoder (4096 dwords) and a 64 KiB block.
  for (size_t bytes : {size_t{16384}, size_t{65536}}) {
    std::vector<uint8_t> buf(bytes + 64);
    const char *label = bytes == 16384 ? "16K-dword" : "64K-dword";
    runLevels("mem-fill", label, static_cast<double>(bytes), "B", [&] {
      simd::kernels().fillPattern32(buf.data() + 4, bytes, 0x12345678u);
    });
  }
}

} // namespace

const char *hostSimdBenchmarkList() {
  return "render-13h, render-modex, render-planar, render-vbe, audio-mix, "
         "mem-fill";
}

bool isHostSimdBenchmark(const std::string &name) {
  return name == "render-13h" || name == "render-modex" ||
         name == "render-planar" || name == "render-vbe" ||
         name == "audio-mix" || name == "mem-fill";
}

bool runHostSimdBenchmark(const std::string &name) {
  std::printf("Host CPU features: %s\n", utils::hostCpuFeatureString().c_str());
  if (name == "render-13h")
    benchRender("render-13h", 0x13, true);
  else if (name == "render-modex")
    benchRender("render-modex", 0x13, false);
  else if (name == "render-planar")
    benchRender("render-planar", 0x12, false);
  else if (name == "render-vbe")
    benchVbe();
  else if (name == "audio-mix")
    benchAudio();
  else if (name == "mem-fill")
    benchFill();
  else
    return false;
  return true;
}

} // namespace fador::ui
