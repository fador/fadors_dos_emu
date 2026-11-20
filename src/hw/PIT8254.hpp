#pragma once
#include "IODevice.hpp"
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>

namespace fador::hw {

/// Intel 8254 programmable interval timer (ports 40h-43h).
///
/// Clocked at 105/88 MHz (1.193182 MHz) from virtual time: the PIT keeps
/// the tick it was last brought up to and evaluates all elapsed ticks in
/// closed form when it is accessed, so idle channels cost nothing.  The
/// main loop calls advanceTo() between batches; that is a single compare
/// until channel 0's next rising OUT edge (IRQ 0) is due.
///
/// Implemented: modes 0-5 with their OUT waveforms, gate inputs (channel 2
/// gate = port 61h bit 0; channels 0/1 have GATE tied high as on the PC),
/// binary and BCD counting, LSB/MSB/word access with separate read and
/// write flip-flops, the counter-latch command, the read-back command with
/// status bytes (OUT, NULL COUNT, RW, mode, BCD) and the "count is loaded
/// on the next CLK pulse" behaviour.  Port 61h bit 4 (DRAM refresh
/// toggle) flips every 18 ticks (~15.1 us), the PC's refresh period, and
/// bit 5 shows OUT2.
class PIT8254 : public IODevice {
public:
  static constexpr uint64_t kNever = std::numeric_limits<uint64_t>::max();
  /// DRAM refresh request period in PIT ticks (port 61h bit 4 toggles).
  static constexpr uint64_t kRefreshTicks = 18;

  PIT8254();
  ~PIT8254() override = default;

  uint8_t read8(uint16_t port) override;
  void write8(uint16_t port, uint8_t value) override;

  /// Virtual-time source (ns).  Port accesses read it to bring the
  /// counters up to date.  Without one, time moves only via advanceTo()
  /// and advanceTime().
  void setTimeSource(std::function<uint64_t()> nowNs) {
    m_timeSource = std::move(nowNs);
  }

  /// Bring IRQ 0 up to virtual time nowNs (main loop, once per batch).
  void advanceTo(uint64_t nowNs) {
    if (nowNs > m_nowNs)
      m_nowNs = nowNs;
    if (nowNs >= m_nextIrqNs)
      catchUp();
  }
  /// Advance by a duration relative to the last known time (unit tests).
  void advanceTime(std::chrono::nanoseconds elapsed);
  /// Dequeue one IRQ 0 (a rising edge of channel 0's OUT).
  bool checkPendingIRQ0();
  /// Virtual time of the next IRQ 0, or kNever.
  uint64_t nextIrq0Ns() const { return m_nextIrqNs; }

  /// GATE input of a channel (only channel 2's is wired on a PC).
  void setGate(int channel, bool level);
  /// Current OUT pin level.
  bool output(int channel);
  /// Port 61h input bits: 4 = refresh toggle, 5 = OUT2.
  uint8_t port61Status();

  /// Called when channel 2 (PC speaker) gets a new count (count, mode).
  void setChannel2Callback(std::function<void(uint16_t, uint8_t)> cb) {
    m_ch2Callback = std::move(cb);
  }

  /// PIT ticks elapsed at virtual time ns (exact: ns * 105 / 88000).
  static uint64_t ticksAt(uint64_t ns) {
    return ns / 88000 * 105 + (ns % 88000) * 105 / 88000;
  }
  /// First virtual ns at which ticksAt() reaches tick.
  static uint64_t nsAtTick(uint64_t tick) {
    return tick / 105 * 88000 + ((tick % 105) * 88000 + 104) / 105;
  }

private:
  struct Channel {
    // Programming
    uint8_t mode = 3;     // 0-5 (6 and 7 are aliases of 2 and 3)
    uint8_t modeBits = 3; // M2-M0 as written (status byte)
    uint8_t rw = 3;       // 1 = LSB, 2 = MSB, 3 = LSB then MSB
    bool bcd = false;
    uint16_t cr = 0;      // count register, as written (BCD if bcd)
    bool crWritten = false;
    bool writeMsbNext = false;
    uint8_t writeLsb = 0;
    bool readMsbNext = false;
    bool countLatched = false;
    uint16_t latch = 0;
    bool statusLatched = false;
    uint8_t status = 0;
    // Counting element
    uint32_t ce = 0;      // binary value, 0 = full count
    bool out = true;
    bool gate = true;
    bool nullCount = true;
    bool loadPending = false; // CR -> CE on the next CLK pulse
    bool armed = false;       // counting (modes 1/5: triggered)
    bool done = false;        // modes 0/1: terminal count; 4/5: strobe over
    // Mode 3
    uint32_t half = 0;        // ticks left in the current half cycle
    uint32_t n3 = 0;          // count of the current cycle
  };

  void sync();
  void catchUp();
  void recomputeNextIrq();
  uint64_t advanceChannel(Channel &c, uint64_t ticks);
  static void loadCounter(Channel &c);
  void writeControl(uint8_t value);
  void writeCount(int index, uint8_t value);
  void countComplete(int index);
  static uint64_t ticksToRisingEdge(const Channel &c);
  static uint16_t visibleCount(const Channel &c);
  static uint8_t statusByte(const Channel &c);
  static uint32_t modulus(const Channel &c) { return c.bcd ? 10000 : 65536; }
  static uint32_t initialCount(const Channel &c);
  static uint32_t highHalf(uint32_t n) { return (n + 1) / 2; }
  static uint32_t lowHalf(uint32_t n) { return n >= 2 ? n / 2 : 1; }
  void noteEdge(int index, bool wasHigh);

  std::array<Channel, 3> m_channels{};
  uint64_t m_nowNs = 0;    // latest virtual time seen
  uint64_t m_lastTick = 0; // ticks the channels have been advanced to
  uint64_t m_nextIrqNs = kNever;
  uint64_t m_pendingIRQ0 = 0;
  std::function<uint64_t()> m_timeSource;
  std::function<void(uint16_t, uint8_t)> m_ch2Callback;
};

} // namespace fador::hw
