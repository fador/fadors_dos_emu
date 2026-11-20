// Gravis Ultrasound (GF1): registers, DRAM, detection, timers, voices,
// volume ramps, the voice IRQ queue, DMA uploads and resampled output.

#include "test_framework.hpp"
#include "hw/DMA8237.hpp"
#include "hw/audio/GravisUltrasound.hpp"
#include "memory/MemoryBus.hpp"
#include <cmath>
#include <memory>
#include <vector>

using namespace fador;
using hw::audio::GravisUltrasound;
using hw::audio::GusConfig;

namespace {

struct GusEnv {
  uint64_t now = 1'000'000;
  memory::MemoryBus mem;
  hw::DMA8237 dma;
  std::unique_ptr<GravisUltrasound> gus;
  std::vector<int> irqs;
  uint16_t base;

  explicit GusEnv(GusConfig config = {}) : base(config.basePort) {
    gus = std::make_unique<GravisUltrasound>(mem, dma, config);
    gus->setTimeSource([this] { return now; });
    gus->setIrqCallback([this](int line) { irqs.push_back(line); });
  }
  void out(uint16_t off, uint8_t v) { gus->write8(static_cast<uint16_t>(base + off), v); }
  uint8_t in(uint16_t off) { return gus->read8(static_cast<uint16_t>(base + off)); }
  void reg8(uint8_t r, uint8_t v) {
    out(0x103, r);
    out(0x105, v);
  }
  void reg16(uint8_t r, uint16_t v) {
    out(0x103, r);
    gus->write16(static_cast<uint16_t>(base + 0x104), v);
  }
  uint8_t rreg8(uint8_t r) {
    out(0x103, r);
    return in(0x105);
  }
  uint16_t rreg16(uint8_t r) {
    out(0x103, r);
    return gus->read16(static_cast<uint16_t>(base + 0x104));
  }
  void voice(int v) { out(0x102, static_cast<uint8_t>(v)); }
  void poke(uint32_t a, uint8_t v) {
    reg16(0x43, static_cast<uint16_t>(a & 0xFFFF));
    reg8(0x44, static_cast<uint8_t>(a >> 16));
    out(0x107, v);
  }
  uint8_t peek(uint32_t a) {
    reg16(0x43, static_cast<uint16_t>(a & 0xFFFF));
    reg8(0x44, static_cast<uint8_t>(a >> 16));
    return in(0x107);
  }
  // 20-bit address (+ 9-bit fraction) into a register pair (02h, 04h, 0Ah).
  void addr(uint8_t hiReg, uint32_t a, uint32_t frac = 0) {
    reg16(hiReg, static_cast<uint16_t>((a >> 7) & 0x1FFF));
    reg16(static_cast<uint8_t>(hiReg + 1),
          static_cast<uint16_t>(((a & 0x7F) << 9) | frac));
  }
  uint32_t position() {
    const uint32_t c = static_cast<uint32_t>(rreg16(0x8A)) << 16 | rreg16(0x8B);
    return c >> 9;
  }
  void run() { // reset, then run with DAC and IRQs enabled
    reg8(0x4C, 0x00);
    reg8(0x4C, 0x01);
    reg8(0x4C, 0x07);
  }
  void advance(uint64_t ns) {
    now += ns;
    gus->advanceTo(now);
  }
  // Ticks at the current rate in ns, from the current time.
  uint64_t ticksNs(uint64_t ticks) const {
    return static_cast<uint64_t>(std::ceil(static_cast<double>(ticks) * 1e9 /
                                           gus->gf1Rate()));
  }
  // An 8-bit looping voice at volume 4095, centre pan, 1 sample per tick.
  void startVoice(int v, uint32_t start, uint32_t end, uint8_t ctrl,
                  uint16_t fc = 1024) {
    voice(v);
    reg8(0x00, 0x03);
    reg16(0x01, fc);
    addr(0x02, start);
    addr(0x04, end);
    addr(0x0A, start);
    reg16(0x09, 0xFFF0);
    reg8(0x0C, 7);
    reg8(0x0D, 0x03);
    reg8(0x00, ctrl);
  }
};

} // namespace

TEST_CASE("GUS: option parsing and ULTRASND", "[gus]") {
  GusConfig c;
  std::string error;
  REQUIRE(hw::audio::parseGusSpec("", c, error));
  REQUIRE(hw::audio::ultrasndVariable(c) == "240,3,3,11,11");
  REQUIRE(hw::audio::parseGusSpec("220,irq=5,dma=1,mem=512", c, error));
  REQUIRE(c.basePort == 0x220);
  REQUIRE(c.irq == 5);
  REQUIRE(c.dma == 1);
  REQUIRE(c.dramKB == 512);
  REQUIRE(hw::audio::ultrasndVariable(c) == "220,1,1,5,5");
  REQUIRE(!hw::audio::parseGusSpec("300", c, error));
  REQUIRE(!hw::audio::parseGusSpec("irq=4", c, error));
  REQUIRE(!hw::audio::parseGusSpec("dma=2", c, error));
  REQUIRE(!hw::audio::parseGusSpec("mem=100", c, error));
  REQUIRE(!hw::audio::parseGusSpec("foo=1", c, error));
}

TEST_CASE("GUS: DRAM peek/poke through 43h/44h/3X7", "[gus]") {
  GusEnv e;
  e.poke(0x00000, 0xAA);
  e.poke(0x00100, 0x55);
  e.poke(0xFFFFF, 0x5A);
  e.poke(0x12345, 0x77);
  REQUIRE(e.peek(0x00000) == 0xAA);
  REQUIRE(e.peek(0x00100) == 0x55);
  REQUIRE(e.peek(0xFFFFF) == 0x5A);
  REQUIRE(e.peek(0x12345) == 0x77);
  REQUIRE(e.rreg16(0x43) == 0x2345);
  REQUIRE(e.rreg8(0x44) == 0x01);
  // 44h holds address bits 19-16 only.
  e.reg8(0x44, 0xF1);
  REQUIRE(e.rreg8(0x44) == 0x01);
}

TEST_CASE("GUS: UltraProbe and DRAM sizing", "[gus]") {
  for (uint32_t kb : {256u, 512u, 768u, 1024u}) {
    GusConfig c;
    c.dramKB = kb;
    GusEnv e(c);
    // UltraProbe: reset, release, poke 0 / 100h, peek 0, reset.
    e.reg8(0x4C, 0x00);
    e.advance(10'000);
    e.reg8(0x4C, 0x01);
    e.advance(10'000);
    e.poke(0, 0xAA);
    e.poke(0x100, 0x55);
    REQUIRE(e.peek(0) == 0xAA);
    e.reg8(0x4C, 0x00);
    // UltraSizeDram: a bank is present if 0xAA / 0x55 read back.
    auto good = [&](uint32_t a) {
      e.poke(a, 0xAA);
      e.poke(a + 1, 0x55);
      return e.peek(a) == 0xAA && e.peek(a + 1) == 0x55;
    };
    uint32_t size = 0;
    while (size < 1024 && good(size * 1024))
      size += 256;
    REQUIRE(size == kb);
  }
}

TEST_CASE("GUS: IRQ/DMA latches and register control", "[gus]") {
  GusEnv e;
  REQUIRE(e.gus->irqLine() == 11);
  REQUIRE(e.gus->dmaChannel() == 3);
  // The SDK sequence: DMA latch with mix bit 6 clear, IRQ latch with it set.
  e.out(0x000, 0x0B);
  e.out(0x00B, 0x01 | 0x80); // DMA 1
  e.out(0x000, 0x4B);
  e.out(0x00B, 0x02); // IRQ 5
  e.out(0x102, 0x00);
  e.out(0x000, 0x09);
  REQUIRE(e.gus->dmaChannel() == 1);
  REQUIRE(e.gus->irqLine() == 5);
  // With 2XF selecting another register, 2XB does not reach the latches.
  e.out(0x00F, 0x05);
  e.out(0x000, 0x49);
  e.out(0x00B, 0x00);
  e.out(0x00F, 0x00);
  REQUIRE(e.gus->irqLine() == 5);
  // IRQ 2 is delivered as IRQ 9.
  e.out(0x00B, 0x01);
  REQUIRE(e.gus->irqLine() == 2);
  e.out(0x000, 0x09);
  e.run();
  e.reg8(0x46, 0xFF);
  e.reg8(0x45, 0x04);
  e.out(0x008, 0x04);
  e.out(0x009, 0x01);
  e.advance(80'000);
  REQUIRE(!e.irqs.empty());
  REQUIRE(e.irqs.back() == 9);
}

TEST_CASE("GUS: voice register read-back and active voices", "[gus]") {
  GusEnv e;
  e.voice(5);
  REQUIRE(e.in(0x102) == 5);
  e.reg16(0x01, 0x1234);
  e.reg16(0x02, 0x1ABC);
  e.reg16(0x03, 0xFE00);
  e.reg16(0x04, 0x0123);
  e.reg16(0x05, 0x4560);
  e.reg8(0x06, 0x3F);
  e.reg8(0x07, 0x10);
  e.reg8(0x08, 0xE0);
  e.reg16(0x09, 0xABC0);
  e.reg16(0x0A, 0x0042);
  e.reg16(0x0B, 0x8000);
  e.reg8(0x0C, 0x0C);
  e.reg8(0x0D, 0x03);
  e.reg8(0x00, 0x0B);
  REQUIRE(e.rreg16(0x81) == 0x1234);
  REQUIRE(e.rreg16(0x82) == 0x1ABC);
  REQUIRE(e.rreg16(0x83) == 0xFE00);
  REQUIRE(e.rreg16(0x84) == 0x0123);
  REQUIRE(e.rreg16(0x85) == 0x4560);
  REQUIRE(e.rreg8(0x86) == 0x3F);
  REQUIRE(e.rreg8(0x87) == 0x10);
  REQUIRE(e.rreg8(0x88) == 0xE0);
  REQUIRE(e.rreg16(0x89) == 0xABC0);
  REQUIRE(e.rreg16(0x8A) == 0x0042);
  REQUIRE(e.rreg16(0x8B) == 0x8000);
  REQUIRE(e.rreg8(0x8C) == 0x0C);
  REQUIRE(e.rreg8(0x8D) == 0x03);
  REQUIRE(e.rreg8(0x80) == 0x0B);
  // Byte writes: 3X4 latches the low byte, 3X5 commits.
  e.out(0x103, 0x01);
  e.out(0x104, 0x78);
  e.out(0x105, 0x56);
  REQUIRE(e.rreg16(0x81) == 0x5678);
  REQUIRE(e.rreg8(0x8E) == 0xCD); // 14 voices after reset
  REQUIRE(e.gus->gf1Rate() == 44100.0);

  // 617400 / voices, exactly: 44100 Hz at 14, 19293.75 Hz at 32.
  e.run();
  e.reg8(0x0E, 0xC0 | 31);
  REQUIRE(e.rreg8(0x8E) == 0xDF);
  uint64_t t0 = e.gus->gf1Ticks();
  e.advance(4'000'000'000);
  e.in(0x006);
  REQUIRE(e.gus->gf1Ticks() - t0 == 77175);
  e.reg8(0x0E, 0xC0 | 2); // fewer than 14: 14
  REQUIRE(e.gus->activeVoices() == 14);
  t0 = e.gus->gf1Ticks();
  e.advance(1'000'000'000);
  e.in(0x006);
  REQUIRE(e.gus->gf1Ticks() - t0 == 44100);
}

TEST_CASE("GUS: timers 1 and 2 raise IRQs at 80/320 us steps", "[gus]") {
  GusEnv e;
  e.run();
  e.reg8(0x46, 256 - 10); // 800 us
  e.reg8(0x47, 256 - 5);  // 1600 us
  e.reg8(0x45, 0x04);     // timer 1 IRQ only
  e.out(0x008, 0x04);
  e.out(0x009, 0x03); // start both
  REQUIRE(e.gus->nextEventNs() == e.now + 800'000);
  e.advance(799'999);
  REQUIRE(e.irqs.empty());
  e.advance(1);
  REQUIRE(e.irqs.size() == 1);
  REQUIRE(e.irqs[0] == 11);
  REQUIRE((e.in(0x006) & 0x0C) == 0x04);
  REQUIRE(e.in(0x008) == (0x80 | 0x40 | 0x04));
  // Acknowledge: clear and set the enable bit in 45h.
  e.reg8(0x45, 0x00);
  e.reg8(0x45, 0x04);
  REQUIRE((e.in(0x006) & 0x0C) == 0x00);
  e.advance(800'000);
  REQUIRE(e.irqs.size() == 2);
  // Timer 2 expired too (no IRQ): its flag shows in 2X8; 80h resets them.
  REQUIRE((e.in(0x008) & 0x20) == 0x20);
  e.out(0x009, 0x80);
  REQUIRE((e.in(0x008) & 0x60) == 0x00);
  // Masked timer 1: no status flag, still an IRQ.
  e.out(0x009, 0x41);
  e.reg8(0x45, 0x00);
  e.reg8(0x45, 0x04);
  e.advance(800'000);
  REQUIRE(e.irqs.size() == 3);
  REQUIRE((e.in(0x008) & 0x40) == 0x00);
  // No IRQs with the GF1 master IRQ enable (4Ch bit 2) off.
  e.reg8(0x4C, 0x03);
  e.reg8(0x45, 0x04);
  e.out(0x009, 0x01);
  e.advance(2'000'000);
  REQUIRE(e.irqs.size() == 3);
}

TEST_CASE("GUS: looped 8-bit voice repeats with the loop length", "[gus]") {
  GusEnv e;
  e.run();
  for (uint32_t i = 0; i < 17; ++i)
    e.poke(0x1000 + i, static_cast<uint8_t>(i * 7));
  std::vector<float> cap;
  e.gus->setCapture(&cap);
  e.startVoice(0, 0x1000, 0x1010, 0x08); // loop 16 samples
  e.advance(e.ticksNs(200));
  e.in(0x006);
  const size_t n = cap.size() / 2;
  REQUIRE(n >= 199);
  const float g = GravisUltrasound::volumeGain(4095) * 0.5f / 32768.0f;
  for (size_t t = 0; t < 16; ++t) {
    const float expected = static_cast<float>((t * 7) * 256) * g;
    REQUIRE(std::fabs(cap[2 * t] - expected) < 1e-6f);
    REQUIRE(std::fabs(cap[2 * t + 1] - expected) < 1e-6f);
  }
  for (size_t t = 0; t + 16 < n; ++t)
    REQUIRE(cap[2 * t] == cap[2 * (t + 16)]);
  // Half speed (FC 512): each sample twice, the second one interpolated.
  cap.clear();
  e.voice(0);
  e.reg8(0x00, 0x03);
  e.addr(0x0A, 0x1000);
  e.reg16(0x01, 512);
  e.reg8(0x00, 0x08);
  e.advance(e.ticksNs(100));
  e.in(0x006);
  REQUIRE(cap.size() / 2 >= 64);
  for (size_t t = 0; t + 32 < cap.size() / 2; ++t)
    REQUIRE(std::fabs(cap[2 * t] - cap[2 * (t + 32)]) < 1e-7f);
  const float half = static_cast<float>(7 * 256) * 0.5f * g;
  REQUIRE(std::fabs(cap[2] - half) < 1e-6f);
}

TEST_CASE("GUS: one-shot voice stops at the end address", "[gus]") {
  GusEnv e;
  e.run();
  e.startVoice(2, 0x100, 0x140, 0x00);
  e.advance(e.ticksNs(100));
  e.voice(2);
  REQUIRE((e.rreg8(0x80) & 0x01) == 0x01);
  REQUIRE(e.position() == 0x140);
  REQUIRE(e.irqs.empty()); // no IRQ enabled
}

TEST_CASE("GUS: bidirectional loop reverses at both ends", "[gus]") {
  GusEnv e;
  e.run();
  e.startVoice(1, 0x200, 0x210, 0x08 | 0x10);
  std::vector<uint32_t> pos;
  std::vector<bool> dec;
  for (int t = 0; t < 70; ++t) {
    e.voice(1);
    pos.push_back(e.position() - 0x200);
    dec.push_back((e.rreg8(0x80) & 0x40) != 0);
    e.advance(e.ticksNs(1));
  }
  // 0..15, 16 (turn), 15..1, 0 (turn), 1, ...: a period of 32 ticks.
  size_t t0 = 0;
  while (t0 < pos.size() && pos[t0] != 0)
    ++t0;
  REQUIRE(t0 < 4);
  for (size_t t = t0; t + 32 < pos.size(); ++t) {
    REQUIRE(pos[t] <= 16);
    REQUIRE(pos[t] == pos[t + 32]);
  }
  for (uint32_t k = 0; k <= 16; ++k)
    REQUIRE(pos[t0 + k] == k);
  for (uint32_t k = 0; k < 16; ++k)
    REQUIRE(pos[t0 + 16 + k] == 16 - k);
  REQUIRE(!dec[t0 + 5]);
  REQUIRE(dec[t0 + 20]);
}

TEST_CASE("GUS: 16-bit voices use the bank address translation", "[gus]") {
  GusEnv e;
  e.run();
  // 16-bit sample k of bank 1 (256K) at register address 40000h + k lives
  // at byte 40000h + 2k.
  const int16_t values[4] = {1000, -2000, 3000, -4000};
  for (int k = 0; k < 4; ++k) {
    e.poke(0x40000 + 0x20 + 2 * k, static_cast<uint8_t>(values[k] & 0xFF));
    e.poke(0x40000 + 0x20 + 2 * k + 1, static_cast<uint8_t>(values[k] >> 8));
  }
  std::vector<float> cap;
  e.gus->setCapture(&cap);
  e.startVoice(0, 0x40010, 0x40014, 0x08 | 0x04);
  e.advance(e.ticksNs(12));
  e.in(0x006);
  REQUIRE(cap.size() / 2 >= 8);
  const float g = GravisUltrasound::volumeGain(4095) * 0.5f / 32768.0f;
  for (size_t t = 0; t < 8; ++t)
    REQUIRE(std::fabs(cap[2 * t] - values[t % 4] * g) < 1e-6f);
}

TEST_CASE("GUS: volume ramp, ramp IRQ and logarithmic volume", "[gus]") {
  REQUIRE(GravisUltrasound::volumeGain(0) == 0.0f);
  REQUIRE(std::fabs(GravisUltrasound::volumeGain(0xF00) - 0.5f) < 1e-7f);
  REQUIRE(std::fabs(GravisUltrasound::volumeGain(0xE80) - 0.375f) < 1e-7f);
  REQUIRE(std::fabs(GravisUltrasound::volumeGain(0xFFF) - 511.0f / 512.0f) < 1e-7f);

  GusEnv e;
  e.run();
  e.voice(4);
  e.reg8(0x00, 0x03);
  e.reg16(0x09, 0x1000); // 100h
  e.reg8(0x07, 0x10);    // 100h
  e.reg8(0x08, 0xF0);    // F00h
  e.reg8(0x06, 0x40 | 63); // +63 every 8 ticks
  e.reg8(0x0D, 0x20);      // up, IRQ, no loop: go
  // ceil(E00h / 63) = 57 steps, 8 ticks apart.
  const uint64_t due = e.ticksNs(57 * 8);
  REQUIRE(e.gus->nextEventNs() >= e.now + due - 1);
  REQUIRE(e.gus->nextEventNs() <= e.now + due + 1);
  const uint64_t start = e.now;
  e.now = start + due - e.ticksNs(1);
  e.gus->advanceTo(e.now);
  e.in(0x006);
  REQUIRE(e.irqs.empty());
  e.now = e.gus->nextEventNs();
  e.gus->advanceTo(e.now);
  REQUIRE(e.irqs.size() == 1);
  REQUIRE((e.in(0x006) & 0x60) == 0x40);
  e.voice(4);
  REQUIRE(e.rreg16(0x89) == 0xF000);
  REQUIRE(e.rreg8(0x8D) == (0x80 | 0x20 | 0x01));
  // 8Fh: voice 4, ramp IRQ (bit 6 low), no wavetable IRQ (bit 7 high).
  e.voice(0);
  REQUIRE(e.rreg8(0x8F) == (0x80 | 0x20 | 4));
  REQUIRE((e.in(0x006) & 0x60) == 0x00);
  REQUIRE((e.rreg8(0x8F) & 0xC0) == 0xC0);

  // Looping bidirectional ramp between 400h and 800h.
  e.voice(6);
  e.reg16(0x09, 0x4000);
  e.reg8(0x07, 0x40);
  e.reg8(0x08, 0x80);
  e.reg8(0x06, 32);         // +32 per tick
  e.reg8(0x0D, 0x08 | 0x10); // loop, bidirectional, no IRQ
  int turns = 0;
  bool lastDec = false;
  for (int t = 0; t < 200; ++t) {
    e.advance(e.ticksNs(1));
    const uint16_t v = e.rreg16(0x89) >> 4;
    REQUIRE(v >= 0x400);
    REQUIRE(v <= 0x800);
    const bool dec = (e.rreg8(0x8D) & 0x40) != 0;
    if (dec != lastDec)
      ++turns;
    lastDec = dec;
  }
  REQUIRE(turns >= 5); // 32 steps per lap -> 6 turns in 200 ticks
}

TEST_CASE("GUS: wavetable IRQs are queued in order of occurrence", "[gus]") {
  GusEnv e;
  e.run();
  e.startVoice(3, 0x000, 0x064, 0x20); // IRQ after 100 ticks
  e.startVoice(1, 0x000, 0x032, 0x20); // IRQ after 50 ticks
  e.startVoice(7, 0x000, 0x064, 0x00); // no IRQ
  e.voice(0);
  // Both voices start in the same tick: the first event is 50 ticks away.
  const uint64_t first = e.gus->nextEventNs();
  REQUIRE(first <= e.now + e.ticksNs(50) + 1);
  REQUIRE(first + 1 >= e.now + e.ticksNs(49));
  // Let both happen before anything looks (one late sync).
  e.now += e.ticksNs(120);
  e.in(0x006);
  REQUIRE(!e.irqs.empty());
  REQUIRE((e.in(0x006) & 0x20) == 0x20);
  e.voice(1);
  REQUIRE((e.rreg8(0x80) & 0x81) == 0x81); // stopped, IRQ pending
  REQUIRE(e.rreg8(0x8F) == (0x40 | 0x20 | 1));
  REQUIRE(e.rreg8(0x8F) == (0x40 | 0x20 | 3));
  REQUIRE((e.rreg8(0x8F) & 0xC0) == 0xC0);
  REQUIRE((e.in(0x006) & 0x60) == 0x00);
  e.voice(1);
  REQUIRE((e.rreg8(0x80) & 0x80) == 0x00);

  // Writing the voice control without bit 7 drops a pending IRQ.
  e.startVoice(2, 0x000, 0x010, 0x20);
  e.advance(e.ticksNs(20));
  REQUIRE((e.in(0x006) & 0x20) == 0x20);
  e.voice(2);
  e.reg8(0x00, 0x03);
  REQUIRE((e.in(0x006) & 0x20) == 0x00);
  REQUIRE((e.rreg8(0x8F) & 0xC0) == 0xC0);
  // Bits 7 + 5 written: a software IRQ.
  const size_t before = e.irqs.size();
  e.reg8(0x00, 0xA3);
  REQUIRE(e.irqs.size() == before + 1);
  REQUIRE(e.rreg8(0x8F) == (0x40 | 0x20 | 2));
}

namespace {

// 8237 single-cycle read transfer (memory -> device) on `ch`.
void programDma(hw::DMA8237 &dma, int ch, uint32_t addr, uint32_t bytes) {
  if (ch < 4) {
    static const uint8_t page[4] = {0x87, 0x83, 0x81, 0x82};
    dma.write8(0x0A, static_cast<uint8_t>(0x04 | ch));
    dma.write8(0x0B, static_cast<uint8_t>(0x48 | ch));
    dma.write8(0x0C, 0);
    dma.write8(static_cast<uint16_t>(ch * 2), addr & 0xFF);
    dma.write8(static_cast<uint16_t>(ch * 2), (addr >> 8) & 0xFF);
    dma.write8(static_cast<uint16_t>(ch * 2 + 1), (bytes - 1) & 0xFF);
    dma.write8(static_cast<uint16_t>(ch * 2 + 1), ((bytes - 1) >> 8) & 0xFF);
    dma.write8(page[ch], static_cast<uint8_t>(addr >> 16));
  } else {
    static const uint8_t page[4] = {0x8F, 0x8B, 0x89, 0x8A};
    const int c = ch - 4;
    const uint32_t words = bytes / 2, waddr = (addr >> 1) & 0xFFFF;
    dma.write8(0xD4, static_cast<uint8_t>(0x04 | c));
    dma.write8(0xD6, static_cast<uint8_t>(0x48 | c));
    dma.write8(0xD8, 0);
    dma.write8(static_cast<uint16_t>(0xC0 + c * 4), waddr & 0xFF);
    dma.write8(static_cast<uint16_t>(0xC0 + c * 4), (waddr >> 8) & 0xFF);
    dma.write8(static_cast<uint16_t>(0xC2 + c * 4), (words - 1) & 0xFF);
    dma.write8(static_cast<uint16_t>(0xC2 + c * 4), ((words - 1) >> 8) & 0xFF);
    dma.write8(page[c], static_cast<uint8_t>((addr >> 16) & 0xFE));
  }
}

void unmaskDma(hw::DMA8237 &dma, int ch) {
  if (ch < 4)
    dma.write8(0x0A, static_cast<uint8_t>(ch));
  else
    dma.write8(0xD4, static_cast<uint8_t>(ch - 4));
}

} // namespace

TEST_CASE("GUS: 8-bit DMA upload with invert MSB and TC IRQ", "[gus]") {
  GusEnv e;
  e.run();
  const uint8_t data[6] = {0x00, 0x7F, 0x80, 0xFF, 0x01, 0x81};
  for (int i = 0; i < 6; ++i)
    e.mem.write8(0x23000 + i, data[i]);
  programDma(e.dma, 3, 0x23000, 6);
  e.reg16(0x42, 0x1000);       // DRAM 10000h
  e.reg8(0x41, 0x01 | 0x20 | 0x80); // go, TC IRQ, invert, 8-bit, divisor 1
  // Masked channel: nothing moves until it is unmasked.
  e.advance(100'000);
  REQUIRE(e.peek(0x10000) == 0x00);
  unmaskDma(e.dma, 3);
  e.advance(1);
  // 6 transfers at 650 kHz take 9.2 us.
  const uint64_t done = e.gus->nextEventNs();
  REQUIRE(done > e.now + 8'000);
  REQUIRE(done < e.now + 10'000);
  REQUIRE(e.irqs.empty());
  e.now = done;
  e.gus->advanceTo(e.now);
  REQUIRE(e.irqs.size() == 1);
  for (int i = 0; i < 6; ++i)
    REQUIRE(e.peek(0x10000 + i) == static_cast<uint8_t>(data[i] ^ 0x80));
  REQUIRE((e.in(0x006) & 0x80) == 0x80);
  REQUIRE(e.rreg8(0x41) == (0x40 | 0x20 | 0x80)); // TC pending, go cleared
  REQUIRE((e.in(0x006) & 0x80) == 0x00);
  REQUIRE(e.rreg8(0x41) == (0x20 | 0x80));
  REQUIRE(e.dma.isMasked(3)); // single-cycle 8237 masks itself at TC

  // Rate divisor 4: four times slower.
  programDma(e.dma, 3, 0x23000, 6);
  e.reg8(0x41, 0x01 | 0x18);
  unmaskDma(e.dma, 3);
  e.advance(1);
  REQUIRE(e.gus->nextEventNs() > e.now + 35'000);
}

TEST_CASE("GUS: 16-bit data DMA inverts only the high bytes", "[gus]") {
  GusEnv e;
  e.run();
  const uint8_t data[4] = {0x34, 0x12, 0x78, 0x56};
  for (int i = 0; i < 4; ++i)
    e.mem.write8(0x24000 + i, data[i]);
  // 8-bit channel carrying 16-bit data.
  programDma(e.dma, 3, 0x24000, 4);
  e.reg16(0x42, 0x0010);
  e.reg8(0x41, 0x01 | 0x40 | 0x80);
  unmaskDma(e.dma, 3);
  e.advance(1); // the unmask is seen at the next advanceTo()
  e.advance(100'000);
  REQUIRE(e.peek(0x100) == 0x34);
  REQUIRE(e.peek(0x101) == 0x92);
  REQUIRE(e.peek(0x102) == 0x78);
  REQUIRE(e.peek(0x103) == 0xD6);
  REQUIRE(e.irqs.empty()); // TC IRQ not enabled

  // 16-bit channel 5: 42h is translated (bits 15-14 = 256K bank).
  GusConfig c;
  c.dma = 5;
  GusEnv f(c);
  f.run();
  for (int i = 0; i < 4; ++i)
    f.mem.write8(0x40000 + i, data[i]);
  programDma(f.dma, 5, 0x40000, 4);
  f.reg16(0x42, 0x4010); // bank 1, word offset 10h << 4 -> 40200h
  f.reg8(0x41, 0x01 | 0x04 | 0x40 | 0x20);
  unmaskDma(f.dma, 5);
  f.advance(1);
  f.advance(100'000);
  REQUIRE(f.irqs.size() == 1);
  REQUIRE(f.peek(0x40200) == 0x34);
  REQUIRE(f.peek(0x40201) == 0x12);
  REQUIRE(f.peek(0x40202) == 0x78);
  REQUIRE(f.peek(0x40203) == 0x56);
}

TEST_CASE("GUS: resampled output has the voice frequency and level", "[gus]") {
  GusEnv e;
  e.run();
  e.reg8(0x0E, 0xC0 | 31); // 32 voices: 19293.75 Hz
  // One sine period in 32 samples (+ the interpolation guard sample).
  for (uint32_t i = 0; i <= 32; ++i)
    e.poke(0x2000 + i,
           static_cast<uint8_t>(static_cast<int8_t>(std::lround(
               127.0 * std::sin(2.0 * 3.14159265358979 * i / 32.0)))));
  e.startVoice(9, 0x2000, 0x2020, 0x08);
  const double freq = 617400.0 / 32.0 / 32.0; // 602.9 Hz
  // Render 1 s at 44.1 kHz in 10 ms blocks on the output-frame grid.
  std::vector<float> out;
  uint64_t frame = 44100 * (e.now / 1'000'000'000) + 1000;
  for (int block = 0; block < 100; ++block) {
    const uint64_t next = frame + 441;
    e.now = (next * 1'000'000'000 + 44099) / 44100;
    std::vector<float> buf(441 * 2, 0.0f);
    e.gus->advanceTo(e.now);
    e.gus->generateSamples(buf.data(), 441);
    out.insert(out.end(), buf.begin(), buf.end());
    frame = next;
  }
  int crossings = 0;
  double sumSq = 0.0;
  for (size_t i = 4410; i < out.size() / 2; ++i) { // skip the start
    if (out[2 * (i - 1)] < 0.0f && out[2 * i] >= 0.0f)
      ++crossings;
    sumSq += static_cast<double>(out[2 * i]) * out[2 * i];
    REQUIRE(out[2 * i] == out[2 * i + 1]); // centred
  }
  const double seconds = (out.size() / 2 - 4410) / 44100.0;
  REQUIRE(std::fabs(crossings / seconds - freq) < 3.0);
  const double rms = std::sqrt(sumSq / (out.size() / 2 - 4410));
  const double expected = 127.0 * 256.0 * 0.5 / 32768.0 *
                          GravisUltrasound::volumeGain(4095) / std::sqrt(2.0);
  REQUIRE(std::fabs(rms / expected - 1.0) < 0.03);

  // Pan 0: left only.  DAC disabled (4Ch bit 1): silence.
  e.voice(9);
  e.reg8(0x0C, 0);
  std::vector<float> buf(441 * 2, 0.0f);
  frame += 441;
  e.now = (frame * 1'000'000'000 + 44099) / 44100;
  e.gus->generateSamples(buf.data(), 441);
  frame += 441;
  e.now = (frame * 1'000'000'000 + 44099) / 44100;
  std::fill(buf.begin(), buf.end(), 0.0f);
  e.gus->generateSamples(buf.data(), 441);
  float maxL = 0, maxR = 0;
  for (size_t i = 0; i < 441; ++i) {
    maxL = std::max(maxL, std::fabs(buf[2 * i]));
    maxR = std::max(maxR, std::fabs(buf[2 * i + 1]));
  }
  REQUIRE(maxL > 0.4f);
  REQUIRE(maxR == 0.0f);
  e.reg8(0x4C, 0x05);
  frame += 441;
  e.now = (frame * 1'000'000'000 + 44099) / 44100;
  e.gus->generateSamples(buf.data(), 441);
  frame += 441;
  e.now = (frame * 1'000'000'000 + 44099) / 44100;
  std::fill(buf.begin(), buf.end(), 0.0f);
  e.gus->generateSamples(buf.data(), 441);
  for (float s : buf)
    REQUIRE(s == 0.0f);
}

TEST_CASE("GUS: master reset stops voices and clears IRQs", "[gus]") {
  GusEnv e;
  e.run();
  e.startVoice(0, 0, 0x10, 0x20 | 0x08);
  e.advance(e.ticksNs(40));
  REQUIRE((e.in(0x006) & 0x20) == 0x20);
  e.reg8(0x0E, 0xC0 | 19);
  e.reg8(0x4C, 0x00);
  REQUIRE(e.in(0x006) == 0x00);
  e.voice(0);
  REQUIRE((e.rreg8(0x80) & 0x01) == 0x01);
  REQUIRE(e.gus->activeVoices() == 14);
  // Held in reset, the GF1 does not run.
  const uint32_t p = e.position();
  e.reg8(0x00, 0x08);
  e.advance(e.ticksNs(40));
  REQUIRE(e.position() == p);
}
