// Virtual time base, 8254 PIT modes and MC146818 RTC timing.
#include "test_framework.hpp"
#include "hw/CMOSRTC.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIT8254.hpp"
#include "hw/VirtualClock.hpp"

#include <cstdint>
#include <vector>

using namespace fador::hw;

namespace {

// A PIT driven by a hand-moved tick counter.
struct PitRig {
  PIT8254 pit;
  uint64_t tick = 0;
  PitRig() {
    pit.setTimeSource([this] { return PIT8254::nsAtTick(tick); });
  }
  void step(uint64_t n = 1) { tick += n; }
  void control(uint8_t v) { pit.write8(0x43, v); }
  void count16(int ch, uint16_t v) {
    pit.write8(static_cast<uint16_t>(0x40 + ch), v & 0xFF);
    pit.write8(static_cast<uint16_t>(0x40 + ch), v >> 8);
  }
  uint16_t read16(int ch) {
    const uint8_t lo = pit.read8(static_cast<uint16_t>(0x40 + ch));
    const uint8_t hi = pit.read8(static_cast<uint16_t>(0x40 + ch));
    return static_cast<uint16_t>(lo | (hi << 8));
  }
  uint16_t latched(int ch) {
    control(static_cast<uint8_t>(ch << 6)); // counter latch command
    return read16(ch);
  }
  uint8_t status(int ch) {
    control(static_cast<uint8_t>(0xE0 | (2 << ch))); // read-back: status
    return pit.read8(static_cast<uint16_t>(0x40 + ch));
  }
  bool out(int ch) { return pit.output(ch); }
  int irqs() {
    pit.advanceTo(PIT8254::nsAtTick(tick));
    int n = 0;
    while (pit.checkPendingIRQ0())
      ++n;
    return n;
  }
};

uint8_t rtcRead(CMOSRTC &rtc, uint8_t idx) {
  rtc.write8(0x70, idx);
  return rtc.read8(0x71);
}
void rtcWrite(CMOSRTC &rtc, uint8_t idx, uint8_t v) {
  rtc.write8(0x70, idx);
  rtc.write8(0x71, v);
}

// An RTC driven by a hand-moved virtual time.
struct RtcRig {
  CMOSRTC rtc;
  uint64_t ns = 0;
  RtcRig() {
    rtc.setTimeSource([this] { return ns; });
  }
  // Advance in steps, acknowledging every IRQ 8 like an interrupt
  // handler (reads register C).  Returns the number of IRQs.
  int run(uint64_t totalNs, uint64_t stepNs, bool acknowledge = true) {
    int irqs = 0;
    for (uint64_t t = 0; t < totalNs; t += stepNs) {
      ns += stepNs;
      rtc.advanceTo(ns);
      while (rtc.checkPendingIRQ8()) {
        ++irqs;
        if (acknowledge)
          rtc.acknowledgeInterrupt();
      }
    }
    return irqs;
  }
};

} // namespace

// ══════════════════════════════════════════════════════════════════
//  VirtualClock
// ══════════════════════════════════════════════════════════════════

TEST_CASE("VirtualClock: machine presets are MHz and cycles per instruction",
          "[Timing]") {
  auto p = findMachinePreset("486dx2-66");
  REQUIRE(p.has_value());
  REQUIRE(p->mhz == 66.0);
  REQUIRE(p->cpi == 2.0);
  REQUIRE(p->instructionsPerSecond() == 33000000);
  REQUIRE(findMachinePreset("386")->name == "386dx-33");
  REQUIRE(findMachinePreset("p90")->name == "pentium-90");
  REQUIRE(findMachinePreset("pentium-100")->instructionsPerSecond() ==
          83333333);
  REQUIRE(!findMachinePreset("z80").has_value());
  REQUIRE(machinePresetList().find("386dx-33") != std::string::npos);
  REQUIRE(defaultDeterministicMachine().name == "486dx2-66");
}

TEST_CASE("VirtualClock: deterministic time is a function of the work",
          "[Timing]") {
  int hostCalls = 0;
  auto makeClock = [&](VirtualClock &clock, const uint64_t *counter) {
    clock.setHostClock(
        [&hostCalls] {
          ++hostCalls;
          return uint64_t{123456789} * static_cast<uint64_t>(hostCalls);
        },
        [](uint64_t) {});
    clock.attachInstructionCounter(counter);
    clock.setMachine(*findMachinePreset("486dx2-66"));
    clock.setPacing(VirtualClock::Pacing::Deterministic);
    clock.start();
  };

  SECTION("33 million instructions at 486dx2-66 are one second") {
    uint64_t counter = 0;
    VirtualClock clock;
    makeClock(clock, &counter);
    for (int i = 0; i < 33000; ++i) {
      counter += 1000;
      clock.sync();
    }
    const uint64_t now = clock.nowNs();
    REQUIRE(now >= 999999999ULL);
    REQUIRE(now <= 1000000001ULL);
    REQUIRE(hostCalls == 0); // never reads the host clock
  }

  SECTION("two runs with the same work produce identical time stamps") {
    std::vector<uint64_t> a, b;
    for (auto *trace : {&a, &b}) {
      uint64_t counter = 0;
      VirtualClock clock;
      makeClock(clock, &counter);
      for (int i = 0; i < 5000; ++i) {
        counter += 1024 + static_cast<uint64_t>(i % 7);
        if (i % 13 == 0)
          clock.idle();
        if (i % 17 == 0)
          clock.advanceCycles(5);
        clock.sync();
        trace->push_back(clock.nowNs());
      }
    }
    REQUIRE(a == b);
    REQUIRE(hostCalls == 0);
  }

  SECTION("time interpolates inside a batch without sync") {
    uint64_t counter = 0;
    VirtualClock clock;
    makeClock(clock, &counter);
    counter += 33; // 33 instructions = 1 us
    REQUIRE(clock.nowNs() >= 999);
    REQUIRE(clock.nowNs() <= 1000);
  }

  SECTION("a waiting guest lets an idle quantum pass") {
    uint64_t counter = 0;
    VirtualClock clock;
    makeClock(clock, &counter);
    clock.idle();
    REQUIRE(clock.nowNs() == VirtualClock::kIdleQuantumNs);
    counter += 8;
    clock.idle();
    REQUIRE(clock.nowNs() >= 2 * VirtualClock::kIdleQuantumNs);
    counter += 100000; // real work between the polls: no extra time
    const uint64_t before = clock.nowNs();
    clock.idle();
    REQUIRE(clock.nowNs() == before);
  }

  SECTION("explicit cycles use the clock frequency") {
    uint64_t counter = 0;
    VirtualClock clock;
    makeClock(clock, &counter);
    clock.setCpuHz(1e6);
    clock.advanceCycles(1000);
    REQUIRE(clock.nowNs() == 1000000);
    clock.setCyclesPerInstruction(4.0);
    counter += 250; // 250 instructions x 4 cycles at 1 MHz = 1 ms
    REQUIRE(clock.nowNs() == 2000000);
  }
}

TEST_CASE("VirtualClock: fixed pacing sleeps to stay behind the wall clock",
          "[Timing]") {
  uint64_t host = 1000;
  uint64_t slept = 0;
  uint64_t counter = 0;
  VirtualClock clock;
  clock.setHostClock([&] { return host; },
                     [&](uint64_t ns) {
                       slept += ns;
                       host += ns;
                     });
  clock.attachInstructionCounter(&counter);
  clock.setMachine(MachineSpeed{"", 1.0, 1.0}); // 1 M instructions/s
  clock.setPacing(VirtualClock::Pacing::Fixed);
  clock.start();

  // A fast host: 10 ms of guest work in no host time -> sleeps 10 ms.
  counter += 10000;
  clock.sync();
  REQUIRE(clock.nowNs() == 10000000);
  REQUIRE(slept == 10000000);

  // Less than the slack ahead: no sleep.
  counter += 500;
  clock.sync();
  REQUIRE(slept == 10000000);

  // A slow host: virtual time lags; never more than kMaxCatchUpNs.
  host += 1000000000;
  clock.sync();
  REQUIRE(clock.droppedNs() > 0);
  counter += 1000;
  clock.sync();
  REQUIRE(slept == 10000000);
  REQUIRE(clock.nowNs() == 11500000);
}

TEST_CASE("VirtualClock: auto pacing tracks the host clock", "[Timing]") {
  uint64_t host = 5000;
  uint64_t counter = 0;
  VirtualClock clock;
  clock.setHostClock([&] { return host; }, [](uint64_t) {});
  clock.attachInstructionCounter(&counter);
  clock.start();
  REQUIRE(clock.pacing() == VirtualClock::Pacing::Auto);

  // Host retires one instruction per 10 ns.
  for (int i = 0; i < 2000; ++i) {
    counter += 1024;
    host += 10240;
    clock.sync();
  }
  const uint64_t elapsed = host - 5000;
  REQUIRE(clock.nowNs() <= elapsed + VirtualClock::kMaxLeadNs);
  REQUIRE(clock.nowNs() + 20000 >= elapsed);
  REQUIRE(clock.estimatedNsPerUnit() > 9.0);
  REQUIRE(clock.estimatedNsPerUnit() < 11.0);

  // Mid-batch reads advance with the instructions.
  counter += 512;
  const uint64_t mid = clock.nowNs();
  REQUIRE(mid > elapsed);
  REQUIRE(mid <= elapsed + 6000);

  // A host stall moves virtual time by at most kMaxJumpNs.
  host += 2000000000;
  const uint64_t before = clock.nowNs();
  clock.sync();
  REQUIRE(clock.nowNs() - before <= VirtualClock::kMaxJumpNs);
  REQUIRE(clock.droppedNs() >= 1900000000);

  // A guest far ahead of the host stops advancing.
  counter += 10000000; // interpolation is capped
  clock.sync();
  const uint64_t frozen = clock.nowNs();
  counter += 100000;
  REQUIRE(clock.nowNs() == frozen);
}

// ══════════════════════════════════════════════════════════════════
//  PIT 8254
// ══════════════════════════════════════════════════════════════════

TEST_CASE("PIT8254: tick conversion is exact", "[Timing][PIT]") {
  REQUIRE(PIT8254::ticksAt(1000000000ULL) == 1193181);
  REQUIRE(PIT8254::ticksAt(88000) == 105);
  for (uint64_t t : {1ULL, 2ULL, 104ULL, 105ULL, 106ULL, 1193181ULL}) {
    REQUIRE(PIT8254::ticksAt(PIT8254::nsAtTick(t)) == t);
    REQUIRE(PIT8254::ticksAt(PIT8254::nsAtTick(t) - 1) == t - 1);
  }
}

TEST_CASE("PIT8254: default channel 0 gives 18.2 Hz", "[Timing][PIT]") {
  PIT8254 pit;
  pit.advanceTo(1000000000ULL);
  int ticks = 0;
  while (pit.checkPendingIRQ0())
    ++ticks;
  REQUIRE(ticks == 18);
  pit.advanceTo(10000000000ULL);
  while (pit.checkPendingIRQ0())
    ++ticks;
  REQUIRE(ticks == 182); // 18.2065 Hz
  REQUIRE(pit.nextIrq0Ns() == PIT8254::nsAtTick(183 * 65536ULL));
}

TEST_CASE("PIT8254: mode 0 interrupt on terminal count", "[Timing][PIT]") {
  PitRig r;
  r.control(0x30); // ch0, LSB/MSB, mode 0, binary
  REQUIRE(!r.out(0));
  REQUIRE(r.status(0) == (0x40 | 0x30 | 0x00)); // NULL COUNT, RW=3, mode 0
  r.count16(0, 5);
  REQUIRE(r.latched(0) == 5); // loads on the next CLK
  r.step();                   // load
  REQUIRE(r.latched(0) == 5);
  REQUIRE(r.status(0) == 0x30); // count loaded: NULL COUNT clear, OUT low
  r.step(4);
  REQUIRE(r.latched(0) == 1);
  REQUIRE(!r.out(0));
  REQUIRE(r.irqs() == 0);
  r.step(); // N+1 CLKs after the write
  REQUIRE(r.out(0));
  REQUIRE(r.latched(0) == 0);
  REQUIRE(r.irqs() == 1);
  r.step();
  REQUIRE(r.latched(0) == 0xFFFF); // keeps counting, OUT stays high
  r.step(200000);
  REQUIRE(r.out(0));
  REQUIRE(r.irqs() == 0); // one-shot

  SECTION("writing the first byte stops the count and drops OUT") {
    r.pit.write8(0x40, 0x10);
    REQUIRE(!r.out(0));
    const uint16_t held = r.latched(0);
    r.step(10);
    REQUIRE(r.latched(0) == held);
    r.pit.write8(0x40, 0x00); // count 0x0010
    r.step(1 + 16);
    REQUIRE(r.out(0));
    REQUIRE(r.irqs() == 1);
  }
}

TEST_CASE("PIT8254: mode 1 hardware retriggerable one-shot", "[Timing][PIT]") {
  PitRig r;
  r.control(0xB2); // ch2, LSB/MSB, mode 1
  r.count16(2, 3);
  REQUIRE(r.out(2));
  r.step(10); // no trigger yet (gate low)
  REQUIRE(r.out(2));
  r.pit.setGate(2, true); // rising edge triggers
  r.step();
  REQUIRE(!r.out(2));
  REQUIRE(r.latched(2) == 3);
  r.step(2);
  REQUIRE(!r.out(2));
  REQUIRE(r.latched(2) == 1);
  r.step();
  REQUIRE(r.out(2));
  // Retrigger in the middle of a pulse restarts it.
  r.pit.setGate(2, false);
  r.pit.setGate(2, true);
  r.step(2);
  r.pit.setGate(2, false);
  r.pit.setGate(2, true);
  r.step(3);
  REQUIRE(!r.out(2)); // would have ended without the retrigger
  r.step();
  REQUIRE(r.out(2));
}

TEST_CASE("PIT8254: mode 2 rate generator", "[Timing][PIT]") {
  PitRig r;
  r.control(0x34); // ch0, LSB/MSB, mode 2
  REQUIRE(r.out(0));
  r.count16(0, 4);
  r.step(); // load
  REQUIRE(r.latched(0) == 4);
  r.step(2);
  REQUIRE(r.latched(0) == 2);
  REQUIRE(r.out(0));
  r.step();
  REQUIRE(r.latched(0) == 1);
  REQUIRE(!r.out(0)); // low for the one CLK at count 1
  REQUIRE(r.irqs() == 0);
  r.step();
  REQUIRE(r.out(0));
  REQUIRE(r.latched(0) == 4);
  REQUIRE(r.irqs() == 1);
  r.step(40);
  REQUIRE(r.irqs() == 10);

  SECTION("a new count takes effect at the next reload") {
    r.count16(0, 10);
    REQUIRE((r.status(0) & 0x40) != 0); // NULL COUNT until loaded
    r.step(3);
    REQUIRE(r.latched(0) == 1); // the current period runs out
    r.step();
    REQUIRE(r.latched(0) == 10);
    REQUIRE((r.status(0) & 0x40) == 0);
    r.irqs();
    r.step(100);
    REQUIRE(r.irqs() == 10);
  }
}

TEST_CASE("PIT8254: mode 3 square wave", "[Timing][PIT]") {
  PitRig r;

  SECTION("even count: CE runs down by two, OUT high and low halves") {
    r.control(0x36);
    r.count16(0, 6);
    r.step();
    REQUIRE(r.latched(0) == 6);
    REQUIRE(r.out(0));
    r.step();
    REQUIRE(r.latched(0) == 4);
    r.step();
    REQUIRE(r.latched(0) == 2);
    REQUIRE(r.out(0));
    r.step();
    REQUIRE(!r.out(0));
    REQUIRE(r.latched(0) == 6);
    r.step(2);
    REQUIRE(!r.out(0));
    REQUIRE(r.irqs() == 0);
    r.step();
    REQUIRE(r.out(0));
    REQUIRE(r.irqs() == 1);
    r.step(6000);
    REQUIRE(r.irqs() == 1000);
  }

  SECTION("odd count: high one CLK longer than low") {
    r.control(0x36);
    r.count16(0, 5);
    r.step();
    const uint16_t expect[] = {4, 2, 0, 4, 2, 4};
    const bool outs[] = {true, true, true, false, false, true};
    for (int i = 0; i < 6; ++i) {
      REQUIRE(r.latched(0) == expect[i]);
      REQUIRE(r.out(0) == outs[i]);
      r.step();
    }
  }

  SECTION("gate low forces OUT high and stops counting (channel 2)") {
    r.control(0xB6); // ch2, mode 3
    r.count16(2, 4);
    r.step(5);
    REQUIRE(r.out(2));
    REQUIRE((r.pit.port61Status() & 0x20) != 0);
    r.pit.setGate(2, true); // restart on the next CLK
    r.step();               // load
    r.step(2);
    REQUIRE(!r.out(2));
    REQUIRE((r.pit.port61Status() & 0x20) == 0); // OUT2 in port 61h bit 5
    r.pit.setGate(2, false);
    REQUIRE(r.out(2));
    const uint16_t held = r.latched(2);
    r.step(7);
    REQUIRE(r.latched(2) == held);
  }
}

TEST_CASE("PIT8254: mode 4 and 5 strobes", "[Timing][PIT]") {
  PitRig r;

  SECTION("mode 4 software triggered strobe") {
    r.control(0x38);
    r.count16(0, 3);
    r.step(); // load
    REQUIRE(r.out(0));
    r.step(3);
    REQUIRE(!r.out(0)); // strobe at count 0
    REQUIRE(r.irqs() == 0);
    r.step();
    REQUIRE(r.out(0));
    REQUIRE(r.irqs() == 1);
    r.step(200000);
    REQUIRE(r.irqs() == 0);
    // A new count restarts the strobe.
    r.count16(0, 2);
    r.step(1 + 2 + 1);
    REQUIRE(r.irqs() == 1);
  }

  SECTION("mode 5 hardware triggered strobe") {
    r.control(0xBA); // ch2
    r.count16(2, 3);
    r.step(10);
    REQUIRE(r.out(2));
    r.pit.setGate(2, true);
    r.step(1 + 3);
    REQUIRE(!r.out(2));
    r.step();
    REQUIRE(r.out(2));
  }
}

TEST_CASE("PIT8254: latch, read-back and flip-flops", "[Timing][PIT]") {
  PitRig r;
  r.control(0x34); // mode 2
  r.count16(0, 1000);
  REQUIRE(r.status(0) == 0xF4); // OUT high, NULL COUNT, RW 3, mode 2
  r.step(101);
  REQUIRE(r.status(0) == 0xB4);

  SECTION("a latched count holds until read; a second latch is ignored") {
    r.control(0x00);
    r.step(50);
    r.control(0x00);
    REQUIRE(r.read16(0) == 900);
    REQUIRE(r.read16(0) == 850); // live again
  }

  SECTION("read-back of status and count: status first") {
    r.control(0xC2); // latch count and status of counter 0
    r.step(10);
    REQUIRE(r.pit.read8(0x40) == 0xB4);
    REQUIRE(r.read16(0) == 900);
  }

  SECTION("read-back can address several counters") {
    r.control(0xDE); // count only, counters 0-2
    r.step(10);
    REQUIRE(r.read16(0) == 900);
  }

  SECTION("reads and writes have separate flip-flops") {
    r.control(0x00);
    REQUIRE(r.pit.read8(0x40) == (900 & 0xFF)); // LSB
    r.pit.write8(0x40, 0x34);                   // new LSB
    REQUIRE(r.pit.read8(0x40) == (900 >> 8));   // MSB
    r.pit.write8(0x40, 0x12);                   // new MSB
    r.step(900);                                // current period ends
    REQUIRE(r.latched(0) == 0x1234);
  }

  SECTION("LSB-only and MSB-only access") {
    r.control(0x14); // LSB only, mode 2
    r.pit.write8(0x40, 200);
    r.step(1 + 50);
    REQUIRE(r.pit.read8(0x40) == 150);
    REQUIRE(r.pit.read8(0x40) == 150);
    r.control(0x24); // MSB only
    r.pit.write8(0x40, 0x02); // count 0x0200
    r.step(1 + 0x100);
    REQUIRE(r.pit.read8(0x40) == 0x01);
  }
}

TEST_CASE("PIT8254: BCD counting", "[Timing][PIT]") {
  PitRig r;
  r.control(0x31); // mode 0, BCD
  r.count16(0, 0x0100);
  r.step(2);
  REQUIRE(r.latched(0) == 0x0099);
  r.step(99);
  REQUIRE(r.out(0));
  r.step();
  REQUIRE(r.latched(0) == 0x9999); // wraps to 9999

  SECTION("count 0 is 10000 in BCD") {
    r.control(0x35); // mode 2, BCD
    r.count16(0, 0);
    r.irqs();
    r.step(1 + 100000);
    REQUIRE(r.irqs() == 10);
  }
}

TEST_CASE("PIT8254: the IRQ 0 path only works when an edge is due",
          "[Timing][PIT]") {
  PIT8254 pit;
  pit.write8(0x43, 0x34);
  pit.write8(0x40, 0xE8); // 1000 ticks
  pit.write8(0x40, 0x03);
  const uint64_t due = pit.nextIrq0Ns();
  REQUIRE(due == PIT8254::nsAtTick(1001));
  pit.advanceTo(due - 1);
  REQUIRE(!pit.checkPendingIRQ0());
  pit.advanceTo(due);
  REQUIRE(pit.checkPendingIRQ0());
  REQUIRE(pit.nextIrq0Ns() == PIT8254::nsAtTick(2001));
}

TEST_CASE("PIT8254: port 61h refresh toggle and OUT2", "[Timing][PIT]") {
  PitRig r;
  KeyboardController kbd;
  kbd.setPort61StatusSource([&r] { return r.pit.port61Status(); });
  kbd.write8(0x61, 0x03); // gate + speaker data: read back in bits 0-1
  REQUIRE((kbd.read8(0x61) & 0x0F) == 0x03);
  REQUIRE((kbd.read8(0x61) & 0x10) == 0);
  r.step(PIT8254::kRefreshTicks);
  REQUIRE((kbd.read8(0x61) & 0x10) != 0);
  REQUIRE((kbd.read8(0x61) & 0x10) != 0); // stable without time passing
  r.step(PIT8254::kRefreshTicks);
  REQUIRE((kbd.read8(0x61) & 0x10) == 0);
  // ~15 us per toggle: 1 ms of reads sees ~66 toggles.
  int toggles = 0;
  uint8_t last = kbd.read8(0x61) & 0x10;
  for (int i = 0; i < 1193; ++i) {
    r.step();
    const uint8_t bit = kbd.read8(0x61) & 0x10;
    toggles += bit != last;
    last = bit;
  }
  REQUIRE(toggles >= 65);
  REQUIRE(toggles <= 67);
  // OUT2 with the gate opened through port 61h bit 0 (as main.cpp wires).
  r.control(0xB6);
  r.count16(2, 100);
  r.pit.setGate(2, true);
  r.step(1 + 50);
  REQUIRE((kbd.read8(0x61) & 0x20) == 0);
  r.step(50);
  REQUIRE((kbd.read8(0x61) & 0x20) != 0);
}

TEST_CASE("PIT8254: channel 2 count still reaches the PC speaker",
          "[Timing][PIT]") {
  PIT8254 pit;
  uint16_t gotCount = 0;
  uint8_t gotMode = 0xFF;
  pit.setChannel2Callback([&](uint16_t count, uint8_t mode) {
    gotCount = count;
    gotMode = mode;
  });
  pit.write8(0x43, 0xB6);
  pit.write8(0x42, 0x33);
  REQUIRE(gotMode == 0xFF); // only after the full count
  pit.write8(0x42, 0x05);
  REQUIRE(gotCount == 0x0533);
  REQUIRE(gotMode == 3);
}

// ══════════════════════════════════════════════════════════════════
//  CMOS RTC
// ══════════════════════════════════════════════════════════════════

TEST_CASE("CMOSRTC: periodic interrupt follows the register A rate",
          "[Timing][CMOS]") {
  struct Case {
    uint8_t rate;
    int hz;
  };
  for (const Case c : {Case{0x6, 1024}, Case{0xF, 2}, Case{0x3, 8192},
                       Case{0x1, 256}, Case{0xA, 64}}) {
    RtcRig r;
    rtcWrite(r.rtc, 0x0A, static_cast<uint8_t>(0x20 | c.rate));
    rtcWrite(r.rtc, 0x0B, 0x42); // PIE, 24 h
    REQUIRE(r.rtc.periodicRateHz() == c.hz);
    const int irqs = r.run(1000000000ULL, 20000);
    REQUIRE(irqs == c.hz);
  }
}

TEST_CASE("CMOSRTC: interrupt flags and IRQ 8 line", "[Timing][CMOS]") {
  RtcRig r;

  SECTION("PF is set without PIE but raises no IRQ") {
    REQUIRE(r.run(10000000, 100000) == 0);
    REQUIRE((rtcRead(r.rtc, 0x0C) & 0xC0) == 0x40);
    REQUIRE(rtcRead(r.rtc, 0x0C) == 0); // read clears
  }

  SECTION("without reading register C the line stays asserted") {
    rtcWrite(r.rtc, 0x0B, 0x42);
    REQUIRE(r.run(100000000, 100000, false) == 1);
    REQUIRE((rtcRead(r.rtc, 0x0C) & 0xC0) == 0xC0);
    REQUIRE(r.run(2000000, 100000) >= 1); // re-armed by the read
  }

  SECTION("update-ended interrupt once per second") {
    rtcWrite(r.rtc, 0x0B, 0x12); // UIE
    REQUIRE(r.run(3000000000ULL, 1000000) == 3);
  }

  SECTION("alarm interrupt") {
    rtcWrite(r.rtc, 0x0B, 0x82); // SET: stop updates while writing
    rtcWrite(r.rtc, 0x00, 0x10);
    rtcWrite(r.rtc, 0x02, 0x20);
    rtcWrite(r.rtc, 0x04, 0x08);
    rtcWrite(r.rtc, 0x01, 0x13); // alarm 08:20:13
    rtcWrite(r.rtc, 0x03, 0x20);
    rtcWrite(r.rtc, 0x05, 0x08);
    rtcWrite(r.rtc, 0x0B, 0x22); // AIE
    REQUIRE(r.run(2500000000ULL, 1000000) == 0);
    REQUIRE(r.run(1000000000ULL, 1000000) == 1);
    rtcWrite(r.rtc, 0x01, 0xC0); // "don't care" seconds: every second
    REQUIRE(r.run(3000000000ULL, 1000000) == 3);
  }
}

TEST_CASE("CMOSRTC: clock runs on virtual time", "[Timing][CMOS]") {
  RtcRig r;
  rtcWrite(r.rtc, 0x0B, 0x82); // SET
  rtcWrite(r.rtc, 0x00, 0x59);
  rtcWrite(r.rtc, 0x02, 0x59);
  rtcWrite(r.rtc, 0x04, 0x23);
  rtcWrite(r.rtc, 0x06, 0x06);
  rtcWrite(r.rtc, 0x07, 0x31);
  rtcWrite(r.rtc, 0x08, 0x12);
  rtcWrite(r.rtc, 0x09, 0x99);
  rtcWrite(r.rtc, 0x32, 0x19);
  r.ns = 5000000000ULL; // SET holds the clock
  REQUIRE(rtcRead(r.rtc, 0x00) == 0x59);
  rtcWrite(r.rtc, 0x0B, 0x02);

  SECTION("seconds carry into the calendar") {
    r.ns = 6000000000ULL - 1;
    REQUIRE(rtcRead(r.rtc, 0x00) == 0x59);
    r.ns += 1;
    REQUIRE(rtcRead(r.rtc, 0x00) == 0x00);
    REQUIRE(rtcRead(r.rtc, 0x02) == 0x00);
    REQUIRE(rtcRead(r.rtc, 0x04) == 0x00);
    REQUIRE(rtcRead(r.rtc, 0x06) == 0x07);
    REQUIRE(rtcRead(r.rtc, 0x07) == 0x01);
    REQUIRE(rtcRead(r.rtc, 0x08) == 0x01);
    REQUIRE(rtcRead(r.rtc, 0x09) == 0x00);
    REQUIRE(rtcRead(r.rtc, 0x32) == 0x20);
    r.ns += 3600000000000ULL; // one hour later
    REQUIRE(rtcRead(r.rtc, 0x04) == 0x01);
  }

  SECTION("UIP is set for 1984 us before each update") {
    r.ns = 6000000000ULL - 3000000;
    REQUIRE((rtcRead(r.rtc, 0x0A) & 0x80) == 0);
    r.ns = 6000000000ULL - 1000000;
    REQUIRE((rtcRead(r.rtc, 0x0A) & 0x80) != 0);
    REQUIRE(rtcRead(r.rtc, 0x00) == 0x59);
    r.ns = 6000000000ULL;
    REQUIRE((rtcRead(r.rtc, 0x0A) & 0x80) == 0);
    REQUIRE(rtcRead(r.rtc, 0x00) == 0x00);
  }

  SECTION("binary 12-hour mode converts and keeps counting") {
    rtcWrite(r.rtc, 0x0B, 0x04); // binary, 12 h
    REQUIRE(rtcRead(r.rtc, 0x04) == (0x80 | 11)); // 11 PM
    REQUIRE(rtcRead(r.rtc, 0x00) == 59);
    r.ns = 6000000000ULL;
    REQUIRE(rtcRead(r.rtc, 0x04) == 12); // 12 AM
    REQUIRE(rtcRead(r.rtc, 0x07) == 1);
  }

  SECTION("divider reset stops the clock; the first update is 500 ms after") {
    rtcWrite(r.rtc, 0x0A, 0x76);
    r.ns = 20000000000ULL;
    REQUIRE(rtcRead(r.rtc, 0x00) == 0x59);
    rtcWrite(r.rtc, 0x0A, 0x26);
    r.ns += 499000000;
    REQUIRE(rtcRead(r.rtc, 0x00) == 0x59);
    r.ns += 1000000;
    REQUIRE(rtcRead(r.rtc, 0x00) == 0x00);
  }
}

TEST_CASE("CMOSRTC: leap years", "[Timing][CMOS]") {
  RtcRig r;
  rtcWrite(r.rtc, 0x0B, 0x82);
  rtcWrite(r.rtc, 0x00, 0x59);
  rtcWrite(r.rtc, 0x02, 0x59);
  rtcWrite(r.rtc, 0x04, 0x23);
  rtcWrite(r.rtc, 0x07, 0x28);
  rtcWrite(r.rtc, 0x08, 0x02);
  rtcWrite(r.rtc, 0x09, 0x24);
  rtcWrite(r.rtc, 0x32, 0x20);
  rtcWrite(r.rtc, 0x0B, 0x02);
  r.ns = 1000000000ULL;
  REQUIRE(rtcRead(r.rtc, 0x07) == 0x29);
  REQUIRE(rtcRead(r.rtc, 0x08) == 0x02);
  r.ns += 86400ULL * 1000000000ULL;
  REQUIRE(rtcRead(r.rtc, 0x07) == 0x01);
  REQUIRE(rtcRead(r.rtc, 0x08) == 0x03);
}
