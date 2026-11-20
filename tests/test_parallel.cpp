// Parallel port, printer file, Covox and Disney Sound Source.

#include "test_framework.hpp"
#include "hw/ParallelPort.hpp"
#include "hw/audio/LptDac.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace fador;

namespace {

constexpr uint16_t kBase = 0x378;

struct LptEnv {
  uint64_t now = 5'000'000;
  hw::ParallelPort port{kBase};
  int irqs = 0;
  LptEnv() {
    port.setTimeSource([this] { return now; });
    port.setIrqCallback([this] { ++irqs; });
  }
  uint8_t in(uint8_t reg) { return port.read8(kBase + reg); }
  void out(uint8_t reg, uint8_t v) { port.write8(kBase + reg, v); }
};

bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) < eps; }

} // namespace

TEST_CASE("LPT registers and status polarity", "[lpt]") {
  LptEnv e;
  // Nothing attached: inputs float high.  BUSY high reads as bit 7 = 0,
  // ACK# / ERROR# high (inactive), paper end and select high.
  REQUIRE(e.in(1) == 0x7F);
  e.out(0, 0xA5);
  REQUIRE(e.in(0) == 0xA5);
  e.out(2, 0x0C);
  REQUIRE(e.in(2) == 0xEC);
  e.out(1, 0x00); // read-only
  REQUIRE(e.in(1) == 0x7F);
}

TEST_CASE("LPT file printer: strobe, ACK pulse and IRQ", "[lpt]") {
  const std::string path = "T_LPT_OUT.TMP";
  {
    LptEnv e;
    bool present = false;
    std::unique_ptr<hw::ParallelDevice> device;
    std::string error;
    REQUIRE(hw::makeParallelDevice("file:" + path, present, device, error));
    REQUIRE(present);
    e.port.setDevice(std::move(device));
    // Ready printer: not busy, ACK# high, paper, selected, no error.
    REQUIRE(e.in(1) == 0xDF);
    e.out(2, 0x1C); // IRQ enabled
    e.out(0, 'O');
    e.out(2, 0x1D); // STROBE
    e.out(2, 0x1C);
    REQUIRE(e.in(1) == 0x9F); // ACK# low during the pulse
    REQUIRE(e.irqs == 0);
    e.now += hw::ParallelPort::kAckPulseNs;
    e.port.advanceTo(e.now);
    REQUIRE(e.irqs == 1); // end of the ACK pulse
    REQUIRE(e.in(1) == 0xDF);
    e.out(2, 0x0C); // IRQ disabled
    e.out(0, 'K');
    e.out(2, 0x0D);
    e.out(2, 0x0C);
    e.now += hw::ParallelPort::kAckPulseNs;
    REQUIRE(e.in(1) == 0xDF);
    REQUIRE(e.irqs == 1);
    e.port.poll(e.now);
  }
  std::FILE *f = std::fopen(path.c_str(), "rb");
  REQUIRE(f != nullptr);
  char buf[8] = {};
  const size_t n = std::fread(buf, 1, sizeof buf, f);
  std::fclose(f);
  std::remove(path.c_str());
  REQUIRE(std::string(buf, n) == "OK");
}

TEST_CASE("LPT device specs are parsed", "[lpt]") {
  bool present = false;
  std::unique_ptr<hw::ParallelDevice> device;
  std::string error;
  REQUIRE(hw::makeParallelDevice("none", present, device, error));
  REQUIRE((present && !device));
  REQUIRE(hw::makeParallelDevice("off", present, device, error));
  REQUIRE(!present);
  REQUIRE(hw::makeParallelDevice("covox", present, device, error));
  REQUIRE(device->describe() == "covox");
  REQUIRE(hw::makeParallelDevice("disney", present, device, error));
  REQUIRE(device->describe() == "disney");
  REQUIRE(!hw::makeParallelDevice("speaker", present, device, error));
}

TEST_CASE("Covox renders data port writes on emulated time", "[lpt][covox][audio]") {
  LptEnv e;
  auto covox = std::make_unique<hw::audio::CovoxDac>();
  auto *dac = covox.get();
  dac->renderer().setDcBlock(false);
  e.port.setDevice(std::move(covox));
  REQUIRE(e.in(1) == 0x7F); // status lines are not connected

  constexpr double kFrameNs = 1e9 / 44100.0;
  // Silent before the first write.
  std::vector<float> buf(2 * 64, 0.0f);
  e.port.generateSamples(buf.data(), 64);
  for (float v : buf)
    REQUIRE(v == 0.0f);

  // Full-scale write in the middle of the next frame: that frame averages
  // the two levels, the following frames hold the new one.
  const uint64_t frameStart = e.now;
  e.now = frameStart + static_cast<uint64_t>(kFrameNs / 2);
  e.out(0, 0xFF);
  e.now = frameStart + static_cast<uint64_t>(4 * kFrameNs);
  std::fill(buf.begin(), buf.end(), 0.0f);
  e.port.generateSamples(buf.data(), 4);
  const float full = 0.5f * 127.0f / 128.0f; // gain 0.5
  REQUIRE(near(buf[0], full / 2, 2e-3f));
  REQUIRE(buf[0] == buf[1]); // mono on both channels
  REQUIRE(near(buf[2], full));
  REQUIRE(near(buf[6], full));

  // Writes faster than the output rate are averaged (PWM-style DAC use).
  const uint64_t t = e.now;
  for (int i = 0; i < 8; ++i) {
    e.now = t + static_cast<uint64_t>(i * kFrameNs / 8);
    e.out(0, (i & 1) ? 0xC0 : 0x40);
  }
  e.now = t + static_cast<uint64_t>(kFrameNs);
  std::fill(buf.begin(), buf.end(), 0.0f);
  e.port.generateSamples(buf.data(), 1);
  REQUIRE(near(buf[0], 0.0f, 2e-3f));
}

TEST_CASE("Covox output is AC coupled", "[lpt][covox][audio]") {
  LptEnv e;
  e.port.setDevice(std::make_unique<hw::audio::CovoxDac>());
  e.out(0, 0xFF);
  std::vector<float> buf(2 * 44100, 0.0f);
  e.now += 1'000'000'000;
  e.port.generateSamples(buf.data(), 44100);
  REQUIRE(buf[0] > 0.4f);                   // the step comes through
  REQUIRE(std::fabs(buf.back()) < 1e-3f);   // a constant level decays
}

TEST_CASE("Disney Sound Source FIFO drains at 7 kHz", "[lpt][disney][audio]") {
  LptEnv e;
  e.now = 0;
  auto dss = std::make_unique<hw::audio::DisneySoundSource>();
  auto *d = dss.get();
  d->renderer().setDcBlock(false);
  e.port.setDevice(std::move(dss));

  // Detection sequence used by games: select, fill the FIFO with more
  // than 16 samples, check ACK (bit 6) shows "full", wait until it has
  // drained a few samples and check that bit 6 cleared.
  e.out(2, 0x04);
  REQUIRE((e.in(1) & 0x40) == 0);
  for (int i = 0; i < 20; ++i) {
    e.out(0, static_cast<uint8_t>(0x80 + 4 * i));
    e.out(2, 0x0C);
    e.out(2, 0x04); // SELECT IN pulse latches the sample
  }
  REQUIRE(d->fifoLevel(e.now) == 16);
  REQUIRE((e.in(1) & 0x40) == 0x40);
  // Writes without a SELECT IN pulse (or a printer strobe) do not latch.
  e.out(0, 0x00);
  e.out(2, 0x05);
  e.out(2, 0x04);
  REQUIRE(d->fifoLevel(e.now) == 16);

  // 7 samples per millisecond: 1/7000 s per sample.
  e.now = 1'000'000;
  REQUIRE(d->fifoLevel(e.now) == 9);
  REQUIRE((e.in(1) & 0x40) == 0);
  e.now = hw::audio::DisneySoundSource::tickNs(16);
  REQUIRE(d->fifoLevel(e.now) == 0);
  REQUIRE(e.in(1) == 0x07);

  // The samples come out at the FIFO clock and the last one played holds.
  // (The first OUT 04h already latched the data port's 00h: control was
  // 0Ch, so the FIFO got 00h, 80h, 84h, ..., B8h.)
  std::vector<float> buf(2 * 8, 0.0f);
  e.now += 1'000'000;
  e.port.generateSamples(buf.data(), 8);
  REQUIRE(near(buf[14], 0.5f * (0xB8 - 128) / 128.0f));
}

TEST_CASE("Disney Sound Source output follows the FIFO clock", "[lpt][disney][audio]") {
  LptEnv e;
  e.now = 0;
  auto dss = std::make_unique<hw::audio::DisneySoundSource>();
  auto *d = dss.get();
  d->renderer().setDcBlock(false);
  e.port.setDevice(std::move(dss));
  std::vector<float> buf(2 * 441, 0.0f);
  e.port.generateSamples(buf.data(), 1); // establish the render position
  // Two samples: +1/2 then -1/2 of full scale.
  for (uint8_t v : {uint8_t(0xC0), uint8_t(0x40)}) {
    e.out(0, v);
    e.out(2, 0x0C);
    e.out(2, 0x04);
  }
  e.now = 10'000'000; // 10 ms = 441 frames
  std::fill(buf.begin(), buf.end(), 0.0f);
  e.port.generateSamples(buf.data(), 441);
  // Frame f covers [f, f+1) * 22.68 us.
  auto frameAt = [&](double ns) {
    return static_cast<size_t>(ns * 44100.0 / 1e9);
  };
  REQUIRE(near(buf[2 * frameAt(100'000)], 0.0f));          // before tick 1
  REQUIRE(near(buf[2 * frameAt(200'000)], 0.25f));         // 0xC0 from tick 1
  REQUIRE(near(buf[2 * frameAt(350'000)], -0.25f));        // 0x40 from tick 2
  REQUIRE(near(buf[2 * 440], -0.25f));                     // held
}
