#pragma once

#include "IODevice.hpp"
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>

namespace fador::hw {

class PIC8259;

/// @brief CMOS/RTC (MC146818-compatible) real-time clock and NVRAM.
///
/// Provides I/O ports 0x70 (index/NMI mask) and 0x71 (data).  The date and
/// time are taken from the host clock once, at construction; from then on
/// the clock runs on virtual time: a 32.768 kHz divider chain drives the
/// once-per-second update cycle (UIP is high for the 1984 us before each
/// update), the periodic interrupt (register A rate select), the
/// update-ended and alarm interrupts.  IRQ 8 is raised when IRQF in
/// register C goes from 0 to 1; reading register C clears the flags.
/// Status registers, equipment bytes, memory sizing, and a valid checksum
/// are initialized to reasonable defaults.
class CMOSRTC : public IODevice {
public:
  static constexpr uint64_t kNever = std::numeric_limits<uint64_t>::max();
  static constexpr uint64_t kTicksPerSecond = 32768;
  /// UIP is set this many 32.768 kHz ticks (1984 us) before an update.
  static constexpr uint64_t kUipTicks = 65;

  CMOSRTC();

  uint8_t read8(uint16_t port) override;
  void write8(uint16_t port, uint8_t value) override;

  /// Configure base (conventional) memory size in KiB.
  void setBaseMemoryKB(uint16_t kb);
  /// Configure extended memory size (above 1 MiB) in KiB.
  /// Accepts a 32-bit value so that > 64 MiB configurations can
  /// use the overflow registers 0x30-0x31.
  void setExtendedMemoryKB(uint32_t kb);

  /// Load the RTC date/time registers from the host's local time.
  void syncTime();

  /// Set slave PIC for IRQ8 delivery
  void setPIC(PIC8259* pic) { m_pic = pic; }

  /// Virtual-time source (ns) read on every port access.
  void setTimeSource(std::function<uint64_t()> nowNs) {
    m_timeSource = std::move(nowNs);
  }
  /// Bring the clock up to virtual time nowNs (main loop, once per batch);
  /// a single compare unless an interrupt is due.
  void advanceTo(uint64_t nowNs) {
    if (nowNs > m_nowNs)
      m_nowNs = nowNs;
    if (nowNs >= m_nextIrqNs)
      catchUp();
  }
  /// Advance by a duration relative to the last known time (unit tests).
  void advanceTime(std::chrono::nanoseconds elapsed);

  /// Check if IRQ8 is pending and dequeue it
  bool checkPendingIRQ8();
  /// Virtual time at which the next interrupt flag may be set, or kNever.
  uint64_t nextIrqNs() const { return m_nextIrqNs; }

  /// Read and clear register C without changing the index register (the
  /// BIOS IRQ 8 handler does this so the next interrupt can be raised).
  uint8_t acknowledgeInterrupt();

  /// Periodic interrupt rate in Hz (0 = off), from register A.
  double periodicRateHz() const;

private:
  void initializeDefaults();
  void recalcChecksum();

  static uint8_t toBcd8(int value);
  static int fromBcd8(uint8_t bcd);

  void sync();
  void catchUp();
  void recomputeNextIrq();
  bool dividerRunning() const;
  /// Periodic interrupt period in 32.768 kHz ticks (0 = off).
  uint64_t periodTicks() const;
  int64_t nextSecondTick() const;
  void setFlags(uint8_t flags);
  void updateIrqLine();
  void tickSecond();
  bool alarmMatches() const;
  void writeStatusA(uint8_t value);
  void writeStatusB(uint8_t value);

  // Time register access in the data format selected by register B.
  static int decode(uint8_t v, uint8_t regB);
  static uint8_t encode(int v, uint8_t regB);
  static int hour24(uint8_t raw, uint8_t regB);
  static uint8_t hourRaw(int hour, uint8_t regB);

  static uint64_t ticksAt(uint64_t ns) {
    return ns / 125000000 * 4096 + (ns % 125000000) * 4096 / 125000000;
  }
  static uint64_t nsAtTick(uint64_t tick) {
    return tick / 4096 * 125000000 + ((tick % 4096) * 125000000 + 4095) / 4096;
  }

  uint8_t m_cmosIndex = 0;
  bool m_nmiEnabled = true;
  std::array<uint8_t, 256> m_cmosRAM{};
  PIC8259* m_pic = nullptr;

  std::function<uint64_t()> m_timeSource;
  uint64_t m_nowNs = 0;
  uint64_t m_lastTick = 0;   // 32.768 kHz ticks processed so far
  int64_t m_divOrigin = 0;   // tick at which the divider chain started
  uint64_t m_nextIrqNs = kNever;
  uint64_t m_pendingIRQ8 = 0;
};

} // namespace fador::hw
