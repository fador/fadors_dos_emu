// Host-SIMD kernels: every level the host supports must match the scalar
// reference bit for bit (and the scalar reference must match a naive
// per-element model) on random data, odd sizes and unaligned pointers.

#include "test_framework.hpp"
#include "utils/HostCpu.hpp"
#include "utils/simd/Simd.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <string>
#include <vector>

using namespace fador::utils;
using namespace fador::utils::simd;

namespace {

constexpr SimdLevel kAllLevels[] = {SimdLevel::Scalar, SimdLevel::SSE2,
                                    SimdLevel::SSE41, SimdLevel::AVX2,
                                    SimdLevel::NEON};

// Sizes around every vector width and unroll factor used by the kernels.
std::vector<size_t> testSizes() {
  std::vector<size_t> s;
  for (size_t n = 0; n <= 70; ++n)
    s.push_back(n);
  for (size_t n : {95, 96, 97, 127, 128, 129, 130, 255, 256, 257, 319, 320,
                   321, 511, 640, 641, 799, 800, 1023, 1031})
    s.push_back(n);
  return s;
}

constexpr size_t kGuard = 64;
constexpr uint8_t kGuardByte = 0xA5;

// A byte buffer with guard bytes after `offset + size` so overruns of the
// kernels' stores are caught.
struct Buffer {
  std::vector<uint8_t> raw;
  size_t offset;
  Buffer(size_t bytes, size_t off) : raw(off + bytes + kGuard, kGuardByte), offset(off) {}
  uint8_t *data() { return raw.data() + offset; }
  bool guardIntact(size_t bytes) const {
    for (size_t i = 0; i < offset; ++i)
      if (raw[i] != kGuardByte)
        return false;
    for (size_t i = offset + bytes; i < raw.size(); ++i)
      if (raw[i] != kGuardByte)
        return false;
    return true;
  }
};

// Input copy in a heap block that ends right after its last byte (placed
// `off` bytes into the block), so a kernel reading past its input runs
// into the allocator's redzone under ASan.
template <typename T> struct Tight {
  std::vector<T> buf;
  size_t off;
  Tight(const T *from, size_t count, size_t o) : buf(o + count), off(o) {
    if (count)
      std::memcpy(buf.data() + o, from, count * sizeof(T));
  }
  const T *data() const { return buf.data() + off; }
};

uint32_t u32At(const uint8_t *p, size_t i) {
  uint32_t v;
  std::memcpy(&v, p + 4 * i, 4);
  return v;
}

std::string where(SimdLevel l, size_t n, size_t off) {
  return std::string(simdLevelName(l)) + " n=" + std::to_string(n) +
         " off=" + std::to_string(off);
}

#define REQUIRE_MSG(cond, msg)                                                 \
  do {                                                                         \
    if (!(cond))                                                               \
      throw std::runtime_error(std::string("REQUIRE failed: " #cond " [") +    \
                               (msg) + "] at " __FILE__ ":" +                  \
                               std::to_string(__LINE__));                      \
  } while (0)

// ── Naive models (independent of KernelsScalar.cpp) ──────────────────
uint32_t model555(uint16_t p) {
  uint32_t r = (p >> 10) & 0x1F, g = (p >> 5) & 0x1F, b = p & 0x1F;
  return 0xFF000000u | (((r << 3) | (r >> 2)) << 16) |
         (((g << 3) | (g >> 2)) << 8) | ((b << 3) | (b >> 2));
}
uint32_t model565(uint16_t p) {
  uint32_t r = (p >> 11) & 0x1F, g = (p >> 5) & 0x3F, b = p & 0x1F;
  return 0xFF000000u | (((r << 3) | (r >> 2)) << 16) |
         (((g << 2) | (g >> 4)) << 8) | ((b << 3) | (b >> 2));
}

} // namespace

TEST_CASE("HostSimd: level names, parsing and availability", "[simd]") {
  REQUIRE(parseSimdLevel("scalar") == SimdLevel::Scalar);
  REQUIRE(parseSimdLevel("sse2") == SimdLevel::SSE2);
  REQUIRE(parseSimdLevel("sse4") == SimdLevel::SSE41);
  REQUIRE(parseSimdLevel("avx2") == SimdLevel::AVX2);
  REQUIRE(parseSimdLevel("neon") == SimdLevel::NEON);
  REQUIRE(parseSimdLevel("auto") == bestSimdLevel());
  REQUIRE(!parseSimdLevel("avx9000").has_value());
  for (SimdLevel l : kAllLevels)
    REQUIRE(parseSimdLevel(simdLevelName(l)) == l);

  REQUIRE(simdLevelSupported(SimdLevel::Scalar));
  REQUIRE(simdLevelSupported(bestSimdLevel()));
  REQUIRE(kernelsFor(SimdLevel::Scalar) != nullptr);
  REQUIRE(!hostCpuFeatureString().empty());
  const auto &f = hostCpuFeatures();
  if (f.avx2)
    REQUIRE(f.avx);
#if defined(__x86_64__) || defined(_M_X64)
  REQUIRE(f.sse2);
  REQUIRE(simdLevelSupported(SimdLevel::SSE2));
#endif

  // Forcing a level: unsupported requests fall back, never fail.
  const SimdLevel saved = activeLevel();
  REQUIRE(setActiveLevel(SimdLevel::Scalar) == SimdLevel::Scalar);
  REQUIRE(&kernels() == kernelsFor(SimdLevel::Scalar));
  for (SimdLevel l : kAllLevels) {
    const SimdLevel got = setActiveLevel(l);
    REQUIRE(simdLevelSupported(got));
    if (simdLevelSupported(l))
      REQUIRE(got == l);
  }
  setActiveLevel(saved);
  REQUIRE(activeLevel() == saved);
}

TEST_CASE("HostSimd: video kernels match the scalar reference", "[simd]") {
  std::mt19937 rng(1234);
  std::array<uint32_t, 256> pal{};
  for (auto &p : pal)
    p = rng();
  std::vector<uint8_t> src(4 * 2048 + 64);
  for (auto &b : src)
    b = static_cast<uint8_t>(rng());

  for (SimdLevel level : kAllLevels) {
    const Kernels *k = kernelsFor(level);
    if (!k)
      continue;
    for (size_t n : testSizes())
      for (size_t off : {size_t{0}, size_t{1}, size_t{3}}) {
        const uint8_t *s = src.data() + off;
        const std::string ctx = where(level, n, off);

        { // 8bpp palette
          Buffer out(4 * n, 4 * off);
          const Tight<uint8_t> in(s, n, off);
          k->paletteToArgb(reinterpret_cast<uint32_t *>(out.data()), in.data(),
                           n, pal.data());
          for (size_t i = 0; i < n; ++i)
            REQUIRE_MSG(u32At(out.data(), i) == pal[s[i]], ctx);
          REQUIRE_MSG(out.guardIntact(4 * n), ctx);
        }
        { // 8bpp palette, doubled
          Buffer out(8 * n, 4 * off);
          const Tight<uint8_t> in(s, n, off);
          k->paletteToArgbX2(reinterpret_cast<uint32_t *>(out.data()),
                             in.data(), n, pal.data());
          for (size_t i = 0; i < 2 * n; ++i)
            REQUIRE_MSG(u32At(out.data(), i) == pal[s[i / 2]], ctx);
          REQUIRE_MSG(out.guardIntact(8 * n), ctx);
        }
        // Planes at different (odd) distances from each other
        const uint8_t *planes[4] = {s, s + 509, s + 1021 + off, s + 1777};
        { // Mode X interleave
          Buffer out(n, off);
          const size_t bytes = (n + 3) / 4;
          const Tight<uint8_t> t0(planes[0], bytes, off), t1(planes[1], bytes, 1),
              t2(planes[2], bytes, 2), t3(planes[3], bytes, 0);
          const uint8_t *tight[4] = {t0.data(), t1.data(), t2.data(), t3.data()};
          k->interleave4(out.data(), tight, n);
          for (size_t i = 0; i < n; ++i)
            REQUIRE_MSG(out.data()[i] == planes[i & 3][i >> 2], ctx);
          REQUIRE_MSG(out.guardIntact(n), ctx);
        }
        { // 16-colour planar
          Buffer out(n, off);
          const size_t bytes = (n + 7) / 8;
          const Tight<uint8_t> t0(planes[0], bytes, off), t1(planes[1], bytes, 1),
              t2(planes[2], bytes, 2), t3(planes[3], bytes, 0);
          const uint8_t *tight[4] = {t0.data(), t1.data(), t2.data(), t3.data()};
          k->planarToIndex(out.data(), tight, n);
          for (size_t i = 0; i < n; ++i) {
            uint8_t idx = 0;
            for (int p = 0; p < 4; ++p)
              if (planes[p][i >> 3] & (0x80 >> (i & 7)))
                idx |= static_cast<uint8_t>(1 << p);
            REQUIRE_MSG(out.data()[i] == idx, ctx);
          }
          REQUIRE_MSG(out.guardIntact(n), ctx);
        }
        { // 15/16/24/32 bpp
          Buffer o555(4 * n, 4 * off), o565(4 * n, 4 * off),
              o24(4 * n, 4 * off), o32(4 * n, 4 * off);
          const Tight<uint8_t> in16(s, 2 * n, off), in24(s, 3 * n, off),
              in32(s, 4 * n, off);
          k->rgb555ToArgb(reinterpret_cast<uint32_t *>(o555.data()), in16.data(), n);
          k->rgb565ToArgb(reinterpret_cast<uint32_t *>(o565.data()), in16.data(), n);
          k->rgb24ToArgb(reinterpret_cast<uint32_t *>(o24.data()), in24.data(), n);
          k->rgb32ToArgb(reinterpret_cast<uint32_t *>(o32.data()), in32.data(), n);
          for (size_t i = 0; i < n; ++i) {
            const auto p16 = static_cast<uint16_t>(s[2 * i] | (s[2 * i + 1] << 8));
            REQUIRE_MSG(u32At(o555.data(), i) == model555(p16), ctx);
            REQUIRE_MSG(u32At(o565.data(), i) == model565(p16), ctx);
            const uint32_t c24 = 0xFF000000u | (uint32_t(s[3 * i + 2]) << 16) |
                                 (uint32_t(s[3 * i + 1]) << 8) | s[3 * i];
            REQUIRE_MSG(u32At(o24.data(), i) == c24, ctx);
            const uint32_t c32 = 0xFF000000u | (uint32_t(s[4 * i + 2]) << 16) |
                                 (uint32_t(s[4 * i + 1]) << 8) | s[4 * i];
            REQUIRE_MSG(u32At(o32.data(), i) == c32, ctx);
          }
          REQUIRE_MSG(o555.guardIntact(4 * n) && o565.guardIntact(4 * n) &&
                          o24.guardIntact(4 * n) && o32.guardIntact(4 * n),
                      ctx);
        }
      }
  }
}

TEST_CASE("HostSimd: audio kernels match the scalar reference bit for bit",
          "[simd]") {
  std::mt19937 rng(99);
  std::uniform_real_distribution<float> wide(-1.6f, 1.6f);
  const size_t maxN = 1100;
  std::vector<float> a(maxN + 8), b(maxN + 8), conv(maxN + 8);
  for (auto &v : a)
    v = wide(rng);
  for (auto &v : b)
    v = wide(rng);
  // Specials for the float → int16 clamp / rounding path.
  const float specials[] = {std::numeric_limits<float>::quiet_NaN(),
                            -std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::infinity(),
                            -std::numeric_limits<float>::infinity(),
                            -0.0f, 0.0f, 1.0f, -1.0f, 1.0000001f, -1.0000001f,
                            0.5f / 32767.0f, 1.5f / 32767.0f, -2.5f / 32767.0f,
                            1e-30f, -1e-30f, 0.99999994f};
  for (size_t i = 0; i < conv.size(); ++i)
    conv[i] = (i % 3 == 0) ? specials[(i / 3) % std::size(specials)]
                           : wide(rng);
  std::vector<uint8_t> bytes(2 * maxN + 16);
  for (auto &v : bytes)
    v = static_cast<uint8_t>(rng());

  const Kernels *ref = kernelsFor(SimdLevel::Scalar);
  auto sameBits = [](const float *x, const float *y, size_t n) {
    return std::memcmp(x, y, n * sizeof(float)) == 0;
  };

  // The scalar reference against the documented formulas.
  {
    std::vector<int16_t> out(maxN);
    const float peak = ref->f32ToS16(out.data(), conv.data(), maxN);
    float expectPeak = 0.0f;
    for (size_t i = 0; i < maxN; ++i) {
      float v = conv[i];
      if (std::isnan(v))
        v = -1.0f;
      v = std::fmin(std::fmax(v, -1.0f), 1.0f);
      expectPeak = std::fmax(expectPeak, std::fabs(v));
      REQUIRE(out[i] == static_cast<int16_t>(std::lrint(v * 32767.0f)));
    }
    REQUIRE(peak == expectPeak);
    int16_t half[3];
    const float halves[3] = {0.5f / 32767.0f, 1.5f / 32767.0f, -2.5f / 32767.0f};
    ref->f32ToS16(half, halves, 3);
    // v * 32767 lands near the .5 boundaries; whatever float the product
    // is, the result must be lrint of it.
    for (int i = 0; i < 3; ++i)
      REQUIRE(half[i] == static_cast<int16_t>(std::lrint(halves[i] * 32767.0f)));
    std::vector<float> f(16);
    const uint8_t u8[2] = {0, 255};
    ref->u8ToF32(f.data(), u8, 2, 1.0f / 128.0f);
    REQUIRE(f[0] == -1.0f && f[1] == 127.0f / 128.0f);
    const uint8_t s16[4] = {0x00, 0x80, 0xFF, 0x7F};
    ref->s16ToF32(f.data(), s16, 2, 1.0f / 32768.0f);
    REQUIRE(f[0] == -1.0f && f[1] == 32767.0f / 32768.0f);
  }

  for (SimdLevel level : kAllLevels) {
    const Kernels *k = kernelsFor(level);
    if (!k || level == SimdLevel::Scalar)
      continue;
    for (size_t n : testSizes()) {
      if (n > maxN)
        continue;
      for (size_t off : {size_t{0}, size_t{1}, size_t{3}}) {
        const std::string ctx = where(level, n, off);
        const float gain = 0.25f + 0.01f * static_cast<float>(n % 37);
        { // mix-add
          std::vector<float> d1(b.begin() + off, b.begin() + off + n + 8);
          std::vector<float> d2 = d1;
          const Tight<float> in(a.data() + off, n, off);
          ref->mixAddF32(d1.data(), in.data(), n, gain);
          k->mixAddF32(d2.data(), in.data(), n, gain);
          REQUIRE_MSG(sameBits(d1.data(), d2.data(), n + 8), ctx);
        }
        { // int16 / uint8 → float
          std::vector<float> r(n + 8, 7.0f), t(n + 8, 7.0f);
          const Tight<uint8_t> in16(bytes.data() + off, 2 * n, off);
          const Tight<uint8_t> in8(bytes.data() + off, n, off);
          ref->s16ToF32(r.data(), in16.data(), n, gain);
          k->s16ToF32(t.data(), in16.data(), n, gain);
          REQUIRE_MSG(sameBits(r.data(), t.data(), n + 8), ctx);
          ref->u8ToF32(r.data(), in8.data(), n, gain);
          k->u8ToF32(t.data(), in8.data(), n, gain);
          REQUIRE_MSG(sameBits(r.data(), t.data(), n + 8), ctx);
        }
        { // float → int16 with clamp and peak
          std::vector<int16_t> r(n + 8, 0x5A5A), t(n + 8, 0x5A5A);
          const Tight<float> in(conv.data() + off, n, off);
          const float pr = ref->f32ToS16(r.data(), in.data(), n);
          const float pt = k->f32ToS16(t.data(), in.data(), n);
          REQUIRE_MSG(r == t, ctx);
          REQUIRE_MSG(std::memcmp(&pr, &pt, sizeof(float)) == 0, ctx);
        }
      }
    }
  }
}

TEST_CASE("HostSimd: pattern fill matches the scalar reference", "[simd]") {
  std::mt19937 rng(7);
  const uint32_t patterns[] = {0x00000000u, 0xFFFFFFFFu, 0x5A5A5A5Au,
                               0x12345678u, 0x0000ABCDu, 0xABCDABCDu,
                               0x00FF00FFu, static_cast<uint32_t>(rng()),
                               static_cast<uint32_t>(rng())};
  std::vector<size_t> sizes = testSizes();
  for (size_t n : {4095, 4096, 4097, 16384, 16387})
    sizes.push_back(n);
  for (SimdLevel level : kAllLevels) {
    const Kernels *k = kernelsFor(level);
    if (!k)
      continue;
    for (uint32_t pat : patterns)
      for (size_t n : sizes)
        for (size_t off : {size_t{0}, size_t{1}, size_t{2}, size_t{7}}) {
          Buffer out(n, off);
          k->fillPattern32(out.data(), n, pat);
          for (size_t i = 0; i < n; ++i)
            REQUIRE_MSG(out.data()[i] == static_cast<uint8_t>(pat >> (8 * (i & 3))),
                        where(level, n, off));
          REQUIRE_MSG(out.guardIntact(n), where(level, n, off));
        }
  }
}

TEST_CASE("HostSimd: VGA scanline helpers wrap at 64 KiB", "[simd]") {
  std::mt19937 rng(42);
  std::array<std::vector<uint8_t>, 4> plane;
  for (auto &p : plane) {
    p.resize(0x10000);
    for (auto &b : p)
      b = static_cast<uint8_t>(rng());
  }
  const uint8_t *planes[4] = {plane[0].data(), plane[1].data(), plane[2].data(),
                              plane[3].data()};
  const SimdLevel saved = activeLevel();
  for (SimdLevel level : kAllLevels) {
    if (!simdLevelSupported(level))
      continue;
    setActiveLevel(level);
    for (uint32_t start : {0u, 5u, 0xFFF0u, 0xFFFFu, 0x1FFF8u, 0x2345u})
      for (size_t n : {size_t{0}, size_t{1}, size_t{7}, size_t{320}, size_t{321},
                       size_t{640}, size_t{1024}})
        for (uint32_t phase = 0; phase < 6; ++phase) {
          const std::string ctx = where(level, n, start) + " phase=" +
                                  std::to_string(phase);
          std::vector<uint8_t> out(n + 1, 0xEE);
          vgaUnchainedRow(out.data(), planes, start, phase, n);
          for (size_t i = 0; i < n; ++i) {
            const uint32_t q = phase + static_cast<uint32_t>(i);
            REQUIRE_MSG(out[i] == plane[q & 3][(start + (q >> 2)) & 0xFFFF], ctx);
          }
          REQUIRE_MSG(out[n] == 0xEE, ctx);
          if (phase != 0)
            continue;
          vgaPlanarRow(out.data(), planes, start, n);
          for (size_t i = 0; i < n; ++i) {
            const uint32_t off = (start + static_cast<uint32_t>(i >> 3)) & 0xFFFF;
            uint8_t idx = 0;
            for (int p = 0; p < 4; ++p)
              if (plane[p][off] & (0x80 >> (i & 7)))
                idx |= static_cast<uint8_t>(1 << p);
            REQUIRE_MSG(out[i] == idx, ctx);
          }
          REQUIRE_MSG(out[n] == 0xEE, ctx);
        }
  }
  setActiveLevel(saved);
}

// ── Integration: audio consumers of the kernels ──────────────────────

#include "hw/DMA8237.hpp"
#include "hw/audio/SoundBlaster.hpp"
#include "hw/audio/WavWriter.hpp"
#include "memory/MemoryBus.hpp"

#include <filesystem>
#include <fstream>

TEST_CASE("HostSimd: WavWriter converts blocks with clamp, rounding and peak",
          "[simd][audio]") {
  const auto path = std::filesystem::temp_directory_path() /
                    ("fador_simd_wav_" + std::to_string(std::random_device{}()) +
                     ".wav");
  // More than one 4096-sample conversion block.
  std::vector<float> samples(5000);
  for (size_t i = 0; i < samples.size(); ++i)
    samples[i] = std::sin(static_cast<float>(i) * 0.01f) * 0.8f;
  samples[1] = 1.5f;   // clamps to +32767
  samples[2] = -3.0f;  // clamps to -32767
  samples[3] = 0.25f;
  samples[4999] = -0.5f;
  {
    fador::hw::audio::WavWriter wav;
    REQUIRE(wav.open(path.string(), 44100, 2));
    wav.write(samples.data(), 3);
    wav.write(samples.data() + 3, samples.size() - 3);
    REQUIRE(wav.framesWritten() == samples.size() / 2);
    REQUIRE(wav.peak() == 1.0f);
  }
  std::ifstream in(path, std::ios::binary);
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(in)),
                             std::istreambuf_iterator<char>());
  in.close();
  std::filesystem::remove(path);
  REQUIRE(bytes.size() == 44 + samples.size() * 2);
  auto sampleAt = [&](size_t i) {
    return static_cast<int16_t>(bytes[44 + 2 * i] | (bytes[45 + 2 * i] << 8));
  };
  REQUIRE(sampleAt(1) == 32767);
  REQUIRE(sampleAt(2) == -32767);
  REQUIRE(sampleAt(3) == static_cast<int16_t>(std::lrint(0.25f * 32767.0f)));
  REQUIRE(sampleAt(4999) == static_cast<int16_t>(std::lrint(-0.5f * 32767.0f)));
  for (size_t i = 5; i < 4999; i += 97)
    REQUIRE(sampleAt(i) == static_cast<int16_t>(std::lrint(samples[i] * 32767.0f)));
}

TEST_CASE("HostSimd: SoundBlaster output queue mixes, pads and drops frames",
          "[simd][audio]") {
  fador::memory::MemoryBus memory;
  fador::hw::DMA8237 dma;
  fador::hw::audio::SoundBlaster sb(memory, dma, 44100.0f);
  sb.write8(0x226, 0x01);
  sb.write8(0x226, 0x00);
  sb.read8(0x22A);
  sb.write8(0x22C, 0x10); // direct DAC output
  sb.write8(0x22C, 0xFF);

  // Time-driven: 10 frames are produced, 16 requested → 6 padded in front.
  sb.advance(10.5 / 44100.0);
  std::vector<float> buf(2 * 16, 0.25f);
  sb.generateSamples(buf.data(), 16);
  for (size_t i = 0; i < 12; ++i)
    REQUIRE(buf[i] == 0.25f);
  const float level = buf[12];
  REQUIRE(level > 0.25f);
  for (size_t i = 12; i < buf.size(); ++i)
    REQUIRE(buf[i] == level);

  // A long backlog is capped: 4410 frames of latency stay queued.
  sb.advance(1.0);
  buf.assign(2 * 16, 0.0f);
  sb.generateSamples(buf.data(), 16);
  REQUIRE(buf.front() != 0.0f);
  std::vector<float> rest(2 * 5000, 0.0f);
  sb.generateSamples(rest.data(), 5000);
  // 4410 frames left → the first 590 frames are padding.
  REQUIRE(rest[2 * 589] == 0.0f && rest[2 * 589 + 1] == 0.0f);
  REQUIRE(rest[2 * 590] != 0.0f && rest.back() != 0.0f);
  buf.assign(2 * 4, 0.5f);
  sb.generateSamples(buf.data(), 4); // queue drained: nothing added
  for (float v : buf)
    REQUIRE(v == 0.5f);
}

// ── Integration: REP STOS fast path (MemoryBus::lfillPattern) ────────

#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "hw/BIOS.hpp"
#include "hw/DOS.hpp"
#include "hw/IOBus.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIC8259.hpp"
#include "hw/PIT8254.hpp"

namespace {

struct StosRun {
  std::vector<uint8_t> memory; // 0x20000..0x5FFFF after the run
  uint32_t ecx = 0, edi = 0, eip = 0;
  int steps = 0;
  uint64_t guestCycles = 0;
};

// Runs one string instruction at 0000:0100 until it completes.  With
// `slowPath` the bus traces writes, which disables the fill fast path.
StosRun runStos(const std::vector<uint8_t> &code, bool slowPath, uint16_t es,
                uint32_t edi, uint32_t ecx, uint32_t eax, bool df) {
  using namespace fador;
  cpu::CPU cpu;
  memory::MemoryBus mem;
  hw::IOBus iobus;
  hw::KeyboardController kbd;
  hw::PIT8254 pit;
  hw::DOS dos(cpu, mem);
  hw::PIC8259 pic(true);
  hw::BIOS bios(cpu, mem, kbd, pit, pic);
  bios.initialize();
  dos.initialize();
  cpu::InstructionDecoder decoder(cpu, mem, iobus, bios, dos);
  for (uint32_t a = 0x20000; a < 0x60000; ++a)
    mem.write8(a, static_cast<uint8_t>(a * 7 + (a >> 9)));
  for (size_t i = 0; i < code.size(); ++i)
    mem.write8(0x100 + static_cast<uint32_t>(i), code[i]);
  cpu.setSegReg(cpu::CS, 0);
  cpu.setSegReg(cpu::DS, 0);
  cpu.setSegReg(cpu::SS, 0);
  cpu.setSegReg(cpu::ES, es);
  cpu.setReg32(cpu::EDI, edi);
  cpu.setReg32(cpu::ECX, ecx);
  cpu.setReg32(cpu::EAX, eax);
  cpu.setEFLAGS(df ? (cpu.getEFLAGS() | cpu::FLAG_DIRECTION)
                   : (cpu.getEFLAGS() & ~cpu::FLAG_DIRECTION));
  cpu.setEIP(0x100);
  mem.setWriteTracing(slowPath);
  StosRun r;
  const uint64_t startCycles = cpu.cycles();
  while (cpu.getEIP() == 0x100 && r.steps < 100000) {
    decoder.step();
    ++r.steps;
  }
  mem.setWriteTracing(false);
  for (uint32_t a = 0x20000; a < 0x60000; ++a)
    r.memory.push_back(mem.read8(a));
  r.ecx = cpu.getReg32(cpu::ECX);
  r.edi = cpu.getReg32(cpu::EDI);
  r.eip = cpu.getEIP();
  r.guestCycles = cpu.cycles() - startCycles;
  return r;
}

} // namespace

TEST_CASE("HostSimd: REP STOS fast path matches the element-wise loop",
          "[simd][cpu]") {
  struct Case {
    const char *name;
    std::vector<uint8_t> code;
    uint16_t es;
    uint32_t edi, ecx, eax;
    bool df;
  };
  const uint32_t hiEcx = 0xABCD0000u; // upper half must survive 16-bit forms
  const Case cases[] = {
      {"rep stosb", {0xF3, 0xAA}, 0x2000, 0x0003, hiEcx | 5000, 0x11223344, false},
      {"rep stosw odd di", {0xF3, 0xAB}, 0x2000, 0x0001, hiEcx | 0x1001, 0x5566, false},
      {"rep stosd", {0xF3, 0x66, 0xAB}, 0x2100, 0x0002, hiEcx | 9000, 0xDEADBEEF, false},
      {"rep stosd addr32", {0xF3, 0x67, 0x66, 0xAB}, 0x0000, 0x30001, 9999, 0xCAFEF00D, false},
      {"rep stosb addr32 byte", {0xF3, 0x67, 0xAA}, 0x0000, 0x20000, 0x12345, 0x7F, false},
      {"repne stosw", {0xF2, 0xAB}, 0x3000, 0x0100, hiEcx | 77, 0xA5A5, false},
      {"di wraps", {0xF3, 0xAB}, 0x2000, 0xFFF0, hiEcx | 0x20, 0x1234, false},
      {"df set", {0xF3, 0xAA}, 0x2000, 0x8000, hiEcx | 300, 0x42, true},
      {"zero count", {0xF3, 0x66, 0xAB}, 0x2000, 0x0010, hiEcx, 0x99, false},
  };
  for (const Case &c : cases) {
    const StosRun slow = runStos(c.code, true, c.es, c.edi, c.ecx, c.eax, c.df);
    const StosRun fast = runStos(c.code, false, c.es, c.edi, c.ecx, c.eax, c.df);
    REQUIRE_MSG(slow.eip != 0x100, c.name);
    REQUIRE_MSG(fast.memory == slow.memory, c.name);
    REQUIRE_MSG(fast.ecx == slow.ecx, c.name);
    REQUIRE_MSG(fast.edi == slow.edi, c.name);
    REQUIRE_MSG(fast.eip == slow.eip, c.name);
    REQUIRE_MSG(fast.steps == slow.steps, c.name); // same batching
    // The bulk fill charges the per-element cycles of the element loop.
    REQUIRE_MSG(fast.guestCycles == slow.guestCycles, c.name);
  }
}
