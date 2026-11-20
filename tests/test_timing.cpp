// Virtual time base, 8254 PIT modes and MC146818 RTC timing.
#include "test_framework.hpp"
#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "hw/BIOS.hpp"
#include "hw/CMOSRTC.hpp"
#include "hw/CivilTime.hpp"
#include "hw/DOS.hpp"
#include "hw/IOBus.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/MachineTime.hpp"
#include "hw/PIC8259.hpp"
#include "hw/PIT8254.hpp"
#include "hw/VirtualClock.hpp"
#include "memory/MemoryBus.hpp"

#include <cstdint>
#include <vector>

using namespace fador::hw;
namespace cpu = fador::cpu;

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

// ══════════════════════════════════════════════════════════════════
//  CPU model <-> emulated time (MachineTime)
// ══════════════════════════════════════════════════════════════════

TEST_CASE("MachineTime: presets select the CPU model and its clock",
          "[Timing][Model]") {
  using cpu::CpuModelId;
  SECTION("every preset maps to a CPU model") {
    for (const char *name :
         {"8088", "286-12", "386sx-16", "386dx-25", "386dx-33", "486sx-25",
          "486dx-33", "486dx2-66", "486dx4-100", "pentium-60", "pentium-90",
          "pentium-100", "pentium-133", "pentiumpro-150", "pentiumpro-200"}) {
      const auto p = findMachinePreset(name);
      REQUIRE(p.has_value());
      REQUIRE(presetCpuModel(*p).has_value());
      REQUIRE(p->cycleScale >= 1.0);
    }
    REQUIRE(*presetCpuModel(*findMachinePreset("486dx2-66")) ==
            CpuModelId::I486DX2);
    REQUIRE(*presetCpuModel(*findMachinePreset("486dx-33")) ==
            CpuModelId::I486DX);
    REQUIRE(*presetCpuModel(*findMachinePreset("386dx-25")) ==
            CpuModelId::I386DX);
    REQUIRE(*presetCpuModel(*findMachinePreset("486dx4")) ==
            CpuModelId::I486DX4);
    REQUIRE(*presetCpuModel(*findMachinePreset("pentium-133")) ==
            CpuModelId::Pentium);
    REQUIRE(*presetCpuModel(*findMachinePreset("ppro")) ==
            CpuModelId::PentiumPro);
  }

  SECTION("a preset alone selects model, clock and fixed pacing") {
    SpeedOptions o;
    o.preset = findMachinePreset("486dx2-66");
    const SpeedPlan plan = planSpeed(o);
    REQUIRE(plan.model == CpuModelId::I486DX2);
    REQUIRE(plan.pacing == VirtualClock::Pacing::Fixed);
    REQUIRE(plan.workHz == 66e6);
    REQUIRE(plan.tscHz == 66000000);
    REQUIRE(!plan.countInstructions);
    REQUIRE(plan.warnings.empty());
  }

  SECTION("pre-386 presets run the 386 table at a corrected clock") {
    SpeedOptions o;
    o.preset = findMachinePreset("xt");
    const SpeedPlan plan = planSpeed(o);
    REQUIRE(plan.model == CpuModelId::I386DX);
    REQUIRE(plan.workHz == 4.77e6 / 3.2);
    o.preset = findMachinePreset("286");
    REQUIRE(planSpeed(o).workHz == 12e6 / 2.2);
  }

  SECTION("an explicit --cpu wins; the preset only sets the MHz") {
    SpeedOptions o;
    o.cpu = CpuModelId::I386DX;
    o.preset = findMachinePreset("pentium-90");
    const SpeedPlan plan = planSpeed(o);
    REQUIRE(plan.model == CpuModelId::I386DX);
    REQUIRE(plan.workHz == 90e6);
    REQUIRE(plan.warnings.size() == 1);
    o.cpu = CpuModelId::Pentium; // matching model: no warning
    REQUIRE(planSpeed(o).warnings.empty());
  }

  SECTION("no speed option: the model's nominal clock, auto pacing") {
    SpeedOptions o;
    SpeedPlan plan = planSpeed(o);
    REQUIRE(plan.model == cpu::kDefaultCpuModel);
    REQUIRE(plan.pacing == VirtualClock::Pacing::Auto);
    REQUIRE(plan.workHz == cpu::cpuModel(plan.model).clockHz);
    o.cpu = CpuModelId::PentiumPro;
    o.deterministic = true;
    plan = planSpeed(o);
    REQUIRE(plan.pacing == VirtualClock::Pacing::Deterministic);
    REQUIRE(plan.workHz == 200e6);
    REQUIRE(plan.tscHz == 200000000);
  }

  SECTION("--throttle-ips paces by instructions") {
    SpeedOptions o;
    o.throttleIps = 1000000;
    const SpeedPlan plan = planSpeed(o);
    REQUIRE(plan.countInstructions);
    REQUIRE(plan.workHz == 1e6);
    REQUIRE(plan.pacing == VirtualClock::Pacing::Fixed);
    REQUIRE(plan.tscHz == cpu::cpuModel(plan.model).clockHz);
  }
}

TEST_CASE("VirtualClock: waiting for an event", "[Timing]") {
  uint64_t host = 0;
  std::vector<uint64_t> sleeps;
  uint64_t counter = 0;
  VirtualClock clock;
  clock.setHostClock([&] { return host; },
                     [&](uint64_t ns) {
                       sleeps.push_back(ns);
                       host += ns;
                     });
  clock.attachInstructionCounter(&counter);
  clock.setCyclesPerInstruction(1.0);
  clock.setCpuHz(100e6);

  SECTION("deterministic: time jumps to the deadline, never back") {
    clock.setPacing(VirtualClock::Pacing::Deterministic);
    clock.start();
    counter += 100; // 1 us
    clock.waitUntil(5'000'000);
    REQUIRE(clock.nowNs() == 5'000'000);
    clock.waitUntil(1'000);
    REQUIRE(clock.nowNs() == 5'000'000);
    counter += 100;
    REQUIRE(clock.nowNs() == 5'001'000);
    REQUIRE(sleeps.empty());
  }

  SECTION("fixed: time jumps and the host sleeps until the wall catches up") {
    clock.setPacing(VirtualClock::Pacing::Fixed);
    clock.start();
    clock.waitUntil(2'000'000);
    REQUIRE(clock.nowNs() == 2'000'000);
    REQUIRE(sleeps.size() == 1);
    REQUIRE(sleeps[0] == 2'000'000);
  }

  SECTION("auto: the host sleeps at most kMaxIdleSleepNs, time follows it") {
    clock.setPacing(VirtualClock::Pacing::Auto);
    clock.start();
    clock.waitUntil(5'000'000);
    REQUIRE(sleeps.size() == 1);
    REQUIRE(sleeps[0] == VirtualClock::kMaxIdleSleepNs);
    REQUIRE(clock.nowNs() == VirtualClock::kMaxIdleSleepNs);
    sleeps.clear();
    clock.waitUntil(clock.nowNs()); // already due: no sleep
    REQUIRE(sleeps.empty());
  }

  SECTION("noteIdle: only calls with little work in between are waiting") {
    clock.setIdleWorkThreshold(2048);
    clock.setPacing(VirtualClock::Pacing::Deterministic);
    counter += 5000;
    REQUIRE(!clock.noteIdle());
    counter += 300;
    REQUIRE(clock.noteIdle());
    counter += 3000;
    REQUIRE(!clock.noteIdle());
  }
}

namespace {

// CPU + clock + PIT + PIC with MachineTime hooks like main.cpp's.
struct HaltRig {
  cpu::CPU cpu{cpu::CpuModelId::Pentium};
  VirtualClock clock;
  MachineTime time{clock, cpu};
  PIT8254 pit;
  PIC8259 pic{true};
  int delivered = 0;
  bool running = true;

  explicit HaltRig(VirtualClock::Pacing pacing = VirtualClock::Pacing::Deterministic) {
    SpeedOptions o;
    o.cpu = cpu::CpuModelId::Pentium;
    o.deterministic = pacing == VirtualClock::Pacing::Deterministic;
    time.configure(planSpeed(o));
    clock.setHostClock([] { return uint64_t{0}; }, [](uint64_t) {});
    clock.start();
    pit.setTimeSource(clock.timeSource());
    pic.write8(0x21, 0x00); // unmask everything
    time.addWakeSource([this] { return pit.nextIrq0Ns(); });
    time.setHooks({[this] {
                     clock.sync();
                     pit.advanceTo(clock.nowNs());
                     while (pit.checkPendingIRQ0())
                       pic.raiseIRQ(0);
                   },
                   [this] {
                     if (!cpu.hardwareInterruptsEnabled() ||
                         pic.getPendingInterrupt() < 0)
                       return false;
                     pic.acknowledgeInterrupt();
                     pic.write8(0x20, 0x20); // the ISR's EOI
                     ++delivered;
                     return true;
                   },
                   [this] { return running; }});
    cpu.setEFLAGS(cpu.getEFLAGS() | cpu::FLAG_INTERRUPT);
  }
};

} // namespace

TEST_CASE("MachineTime: HLT sleeps until the next IRQ 0", "[Timing][HLT]") {
  SECTION("deterministic: each HLT ends exactly at the PIT edge") {
    HaltRig rig;
    rig.cpu.addGuestCycles(1234);
    for (int i = 1; i <= 100; ++i) {
      const uint64_t due = rig.pit.nextIrq0Ns();
      REQUIRE(due > rig.clock.nowNs());
      rig.cpu.halt();
      REQUIRE(rig.delivered == i);
      REQUIRE(rig.clock.nowNs() == due);
    }
    // Channel 0 at its default 65536 count: 100 periods.
    REQUIRE(PIT8254::ticksAt(rig.clock.nowNs()) / 65536 == 100);
    REQUIRE(rig.time.halts() == 100);
    REQUIRE(rig.cpu.cycles() == 1234); // waiting is not executed work
  }

  SECTION("the TSC counts the halted time at the core clock") {
    HaltRig rig;
    const uint64_t tsc0 = rig.cpu.readTSC();
    for (int i = 0; i < 18; ++i)
      rig.cpu.halt();
    const uint64_t tsc1 = rig.cpu.readTSC();
    const double seconds = static_cast<double>(rig.clock.nowNs()) * 1e-9;
    REQUIRE(std::abs(static_cast<double>(tsc1 - tsc0) / seconds - 100e6) <
            1e3);
    REQUIRE(rig.cpu.readTSC() > tsc1); // strictly increasing
  }

  SECTION("IF=0: HLT does not wait") {
    HaltRig rig;
    rig.cpu.setEFLAGS(rig.cpu.getEFLAGS() & ~cpu::FLAG_INTERRUPT);
    rig.cpu.halt();
    REQUIRE(rig.clock.nowNs() == 0);
    REQUIRE(rig.delivered == 0);
    REQUIRE(rig.time.haltsWithInterruptsDisabled() == 1);
  }

  SECTION("an STI shadow ends with the HLT") {
    HaltRig rig;
    rig.cpu.beginInterruptShadow();
    rig.cpu.halt();
    REQUIRE(rig.delivered == 1);
  }

  SECTION("the run stopping ends the wait") {
    HaltRig rig;
    rig.pic.write8(0x21, 0xFF); // everything masked: nothing can wake it
    int calls = 0;
    rig.time.setHooks({[] {}, [] { return false; },
                       [&] { return ++calls < 5; }});
    rig.cpu.halt();
    REQUIRE(calls == 5);
    REQUIRE(rig.clock.nowNs() == 4 * MachineTime::kWaitSliceNs);
  }

  SECTION("INT 15h AH=86h style delay: time passes, IRQs wait") {
    HaltRig rig;
    rig.time.delay(200'000'000); // 200 ms: several PIT periods
    REQUIRE(rig.clock.nowNs() == 200'000'000);
    REQUIRE(rig.delivered == 0);
    REQUIRE(rig.pic.getPendingInterrupt() >= 0);
  }

  SECTION("idle: only a waiting guest moves time to the next event") {
    HaltRig rig;
    rig.cpu.addGuestCycles(100000);
    rig.time.guestIdle(); // work since the last idle call: no wait
    REQUIRE(rig.clock.nowNs() == 1000000); // 100000 cycles at 100 MHz
    rig.cpu.addGuestCycles(200);
    rig.time.guestIdle();
    REQUIRE(rig.clock.nowNs() == 1002000 + MachineTime::kWaitSliceNs);
  }
}

TEST_CASE("MachineTime: the TSC is emulated time at the core clock",
          "[Timing][TSC]") {
  cpu::CPU c(cpu::CpuModelId::Pentium);
  VirtualClock clock;
  MachineTime time(clock, c);
  SpeedOptions o;
  o.preset = findMachinePreset("pentium-133");
  o.deterministic = true;
  time.configure(planSpeed(o));
  clock.start();
  const uint64_t t0 = c.readTSC();
  c.addGuestCycles(133000);
  REQUIRE(clock.nowNs() == 1000000);
  REQUIRE(c.readTSC() - t0 == 133000);
  clock.advanceNs(1000000); // time without work (an idle wait)
  REQUIRE(c.readTSC() - t0 == 266000);
  c.writeTSC(5);
  REQUIRE(c.readTSC() == 5);
  REQUIRE(c.readTSC() == 6); // same instant: still increasing
}

// ══════════════════════════════════════════════════════════════════
//  Guest wall clock: RTC, BIOS tick count, DOS date/time
// ══════════════════════════════════════════════════════════════════

TEST_CASE("CivilTime: calendar arithmetic and --start-time parsing",
          "[Timing][Clock]") {
  REQUIRE(daysFromCivil(1970, 1, 1) == 0);
  REQUIRE(weekdayFromDays(daysFromCivil(1994, 6, 1)) == 3);   // Wednesday
  REQUIRE(weekdayFromDays(daysFromCivil(1980, 1, 1)) == 2);   // Tuesday
  REQUIRE(weekdayFromDays(daysFromCivil(2000, 2, 29)) == 2);  // Tuesday
  for (int64_t d = daysFromCivil(1979, 12, 1); d < daysFromCivil(2101, 1, 1);
       d += 17) {
    int y = 0, m = 0, dd = 0;
    civilFromDays(d, y, m, dd);
    REQUIRE(daysFromCivil(y, m, dd) == d);
  }
  REQUIRE(biosTicksAt(24, 0, 0) == kBiosTicksPerDay);
  REQUIRE(biosTicksAt(12, 0, 0) == kBiosTicksPerDay / 2);
  const auto t = parseCivilDateTime("2001-02-28T08:30:05");
  REQUIRE(t.has_value());
  REQUIRE((*t == CivilDateTime{2001, 2, 28, 8, 30, 5}));
  REQUIRE((*parseCivilDateTime("1999-12-31") ==
           CivilDateTime{1999, 12, 31, 0, 0, 0}));
  REQUIRE(!parseCivilDateTime("2001-02-29T00:00:00").has_value());
  REQUIRE(!parseCivilDateTime("1979-12-31T00:00:00").has_value());
  REQUIRE(!parseCivilDateTime("2001-02-28T24:00:00").has_value());
  REQUIRE(!parseCivilDateTime("2001-2-28").has_value());
}

TEST_CASE("CMOSRTC: setDateTime/dateTime in every register format",
          "[Timing][CMOS]") {
  for (uint8_t regB : {0x02, 0x00, 0x06, 0x04}) { // 24h/12h, BCD/binary
    RtcRig rig;
    rtcWrite(rig.rtc, 0x0B, regB);
    rig.rtc.setDateTime({1999, 12, 31, 23, 59, 58});
    REQUIRE((rig.rtc.dateTime() == CivilDateTime{1999, 12, 31, 23, 59, 58}));
    rig.ns += 2'100'000'000; // the clock keeps running
    REQUIRE((rig.rtc.dateTime() == CivilDateTime{2000, 1, 1, 0, 0, 0}));
    // Day of week register: 2000-01-01 was a Saturday (7).
    const uint8_t dow = rtcRead(rig.rtc, 0x06);
    REQUIRE(dow == 7);
  }
}

namespace {

struct ClockRig {
  cpu::CPU cpu;
  fador::memory::MemoryBus mem;
  KeyboardController kbd;
  PIT8254 pit;
  PIC8259 pic{true};
  CMOSRTC rtc;
  uint64_t ns = 0;
  BIOS bios{cpu, mem, kbd, pit, pic};
  DOS dos{cpu, mem};
  ClockRig() {
    bios.initialize();
    dos.initialize();
    rtc.setTimeSource([this] { return ns; });
    bios.setCMOS(&rtc);
    dos.setRTC(&rtc);
    rtc.setDateTime(kDeterministicStartTime);
    bios.loadTickCountFromRTC();
    dos.setDate(1994, 6, 1);
  }
  bool carry() const { return (cpu.getEFLAGS() & cpu::FLAG_CARRY) != 0; }
  void int1A(uint8_t ah) {
    cpu.setReg8(cpu::AH, ah);
    bios.handleInterrupt(0x1A);
  }
  void int21(uint8_t ah) {
    cpu.setReg8(cpu::AH, ah);
    dos.handleInterrupt(0x21);
  }
};

} // namespace

TEST_CASE("BIOS: tick count, midnight flag and INT 1Ah", "[Timing][BIOS]") {
  ClockRig rig;
  SECTION("POST loads the tick count from the RTC") {
    REQUIRE(rig.mem.read32(0x46C) == kBiosTicksPerDay / 2);
    rig.int1A(0x00);
    REQUIRE((static_cast<uint32_t>(rig.cpu.getReg16(cpu::CX)) << 16 |
             rig.cpu.getReg16(cpu::DX)) == rig.mem.read32(0x46C));
    REQUIRE(rig.cpu.getReg8(cpu::AL) == 0);
  }

  SECTION("the tick count wraps at midnight and AH=00h reports it once") {
    rig.mem.write32(0x46C, kBiosTicksPerDay - 2);
    rig.bios.advanceTickCount();
    REQUIRE(rig.mem.read32(0x46C) == kBiosTicksPerDay - 1);
    rig.bios.advanceTickCount();
    REQUIRE(rig.mem.read32(0x46C) == 0);
    REQUIRE(rig.mem.read8(0x470) == 1);
    rig.int1A(0x00);
    REQUIRE(rig.cpu.getReg8(cpu::AL) == 1);
    rig.int1A(0x00);
    REQUIRE(rig.cpu.getReg8(cpu::AL) == 0);
  }

  SECTION("the ROM INT 8 handler wraps the count too") {
    IOBus iobus;
    cpu::InstructionDecoder decoder(rig.cpu, rig.mem, iobus, rig.bios,
                                    rig.dos);
    rig.mem.write32(0x46C, kBiosTicksPerDay - 1);
    decoder.loadSegment(cpu::SS, 0x1000);
    rig.cpu.setReg16(cpu::SP, 0x0100);
    decoder.loadSegment(cpu::CS, 0x2000);
    rig.cpu.setEIP(0x0010);
    rig.mem.write8(0x20010, 0x90);
    rig.cpu.push16(0x0002);
    rig.cpu.push16(0x2000);
    rig.cpu.push16(0x0010);
    decoder.loadSegment(cpu::CS, rig.mem.read16(0x08 * 4 + 2));
    rig.cpu.setEIP(rig.mem.read16(0x08 * 4));
    for (int i = 0; i < 64 && rig.cpu.getSegReg(cpu::CS) != 0x2000; ++i)
      decoder.step();
    REQUIRE(rig.cpu.getSegReg(cpu::CS) == 0x2000);
    REQUIRE(rig.mem.read32(0x46C) == 0);
    REQUIRE(rig.mem.read8(0x470) == 1);
  }

  SECTION("AH=02h/04h read the RTC; AH=03h/05h set it") {
    rig.ns = 3'500'000'000; // 3.5 s later
    rig.int1A(0x02);
    REQUIRE(!rig.carry());
    REQUIRE(rig.cpu.getReg8(cpu::CH) == 0x12);
    REQUIRE(rig.cpu.getReg8(cpu::CL) == 0x00);
    REQUIRE(rig.cpu.getReg8(cpu::DH) == 0x03);
    rig.int1A(0x04);
    REQUIRE(rig.cpu.getReg16(cpu::CX) == 0x1994);
    REQUIRE(rig.cpu.getReg16(cpu::DX) == 0x0601);
    rig.cpu.setReg16(cpu::CX, 0x2359);
    rig.cpu.setReg16(cpu::DX, 0x5901); // 23:59:59, DST on
    rig.int1A(0x03);
    REQUIRE(!rig.carry());
    rig.cpu.setReg16(cpu::CX, 0x2023);
    rig.cpu.setReg16(cpu::DX, 0x1231);
    rig.int1A(0x05);
    REQUIRE(!rig.carry());
    rig.ns += 1'000'000'000;
    REQUIRE((rig.rtc.dateTime() == CivilDateTime{2024, 1, 1, 0, 0, 0}));
    rig.int1A(0x02);
    REQUIRE(rig.cpu.getReg8(cpu::DL) == 1);
    rig.cpu.setReg16(cpu::CX, 0x2460); // invalid
    rig.int1A(0x03);
    REQUIRE(rig.carry());
  }
}

TEST_CASE("DOS: date and time come from the BIOS tick count",
          "[Timing][DOS]") {
  ClockRig rig;
  SECTION("AH=2Ch/2Ah read the emulated clock") {
    rig.mem.write32(0x46C, biosTicksAt(12, 34, 56));
    rig.int21(0x2C);
    REQUIRE(rig.cpu.getReg8(cpu::CH) == 12);
    REQUIRE(rig.cpu.getReg8(cpu::CL) == 34);
    REQUIRE(rig.cpu.getReg8(cpu::DH) == 56);
    REQUIRE(rig.cpu.getReg8(cpu::DL) < 6);
    rig.int21(0x2A);
    REQUIRE(rig.cpu.getReg16(cpu::CX) == 1994);
    REQUIRE(rig.cpu.getReg8(cpu::DH) == 6);
    REQUIRE(rig.cpu.getReg8(cpu::DL) == 1);
    REQUIRE(rig.cpu.getReg8(cpu::AL) == 3);
  }

  SECTION("AH=2Dh sets the time; reading it back gives the same second") {
    for (uint8_t hundredths : {0, 37, 94}) {
      rig.cpu.setReg16(cpu::CX, 0x0817); // 08:23
      rig.cpu.setReg16(cpu::DX, static_cast<uint16_t>(0x2D00 | hundredths));
      rig.int21(0x2D);
      REQUIRE(rig.cpu.getReg8(cpu::AL) == 0);
      rig.int1A(0x00); // INT 1Ah AH=00h sees the same count
      const uint32_t ticks = static_cast<uint32_t>(rig.cpu.getReg16(cpu::CX))
                                 << 16 |
                             rig.cpu.getReg16(cpu::DX);
      REQUIRE(ticks == rig.mem.read32(0x46C));
      rig.int21(0x2C);
      REQUIRE(rig.cpu.getReg16(cpu::CX) == 0x0817);
      const int cs = rig.cpu.getReg8(cpu::DH) * 100 + rig.cpu.getReg8(cpu::DL);
      REQUIRE(cs >= 45 * 100 + hundredths);
      REQUIRE(cs <= 45 * 100 + hundredths + 5); // one tick is 5.5/100 s
    }
    // The RTC was set too (DOS 3.3+).
    REQUIRE(rig.rtc.dateTime().hour == 8);
    REQUIRE(rig.rtc.dateTime().minute == 23);
    rig.cpu.setReg16(cpu::CX, 0x1800); // 24:00 is invalid
    rig.cpu.setReg16(cpu::DX, 0);
    rig.int21(0x2D);
    REQUIRE(rig.cpu.getReg8(cpu::AL) == 0xFF);
  }

  SECTION("AH=2Bh sets the date; midnight advances it") {
    rig.cpu.setReg16(cpu::CX, 1999);
    rig.cpu.setReg16(cpu::DX, 0x0C1F); // 12-31
    rig.int21(0x2B);
    REQUIRE(rig.cpu.getReg8(cpu::AL) == 0);
    REQUIRE(rig.rtc.dateTime().year == 1999);
    rig.mem.write32(0x46C, kBiosTicksPerDay - 1);
    rig.bios.advanceTickCount();
    rig.int21(0x2A);
    REQUIRE(rig.cpu.getReg16(cpu::CX) == 2000);
    REQUIRE(rig.cpu.getReg16(cpu::DX) == 0x0101);
    REQUIRE(rig.cpu.getReg8(cpu::AL) == 6); // Saturday
    rig.int21(0x2A);                         // only one day per midnight
    REQUIRE(rig.cpu.getReg16(cpu::DX) == 0x0101);
    rig.cpu.setReg16(cpu::CX, 1999);
    rig.cpu.setReg16(cpu::DX, 0x021D); // 1999-02-29 does not exist
    rig.int21(0x2B);
    REQUIRE(rig.cpu.getReg8(cpu::AL) == 0xFF);
  }
}
