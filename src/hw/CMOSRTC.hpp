#pragma once

#include "IODevice.hpp"
#include <array>
#include <chrono>
#include <cstdint>

namespace fador::hw {

class PIC8259;

/// @brief CMOS/RTC (MC146818-compatible) real-time clock and NVRAM.
///
/// Provides I/O ports 0x70 (index/NMI mask) and 0x71 (data).  The 14
/// RTC time/date bytes at indexes 00h-0Dh are kept in sync with the
/// host system clock.  Status registers, equipment bytes, memory
/// sizing, and a valid checksum are initialized to reasonable defaults.
class CMOSRTC : public IODevice {
public:
  CMOSRTC();

  uint8_t read8(uint16_t port) override;
  void write8(uint16_t port, uint8_t value) override;

  /// Configure base (conventional) memory size in KiB.
  void setBaseMemoryKB(uint16_t kb);
  /// Configure extended memory size (above 1 MiB) in KiB.
  /// Accepts a 32-bit value so that > 64 MiB configurations can
  /// use the overflow registers 0x30-0x31.
  void setExtendedMemoryKB(uint32_t kb);

  /// Reseed the RTC time registers from the host clock.
  /// Call periodically so that time-tick registers stay current.
  void syncTime();

  /// Set slave PIC for IRQ8 delivery
  void setPIC(PIC8259* pic) { m_pic = pic; }

  /// Advance RTC time and fire pending IRQ8 (periodic or alarm)
  void advanceTime();

  /// Check if IRQ8 is pending and dequeue it
  bool checkPendingIRQ8();

private:
  void initializeDefaults();
  void recalcChecksum();

  static uint8_t toBcd8(int value);
  static int fromBcd8(uint8_t bcd);

  /// Periodic interrupt rate in Hz (from Status Reg A bits 0-3)
  int periodicRateHz() const;

  /// Microseconds per periodic interrupt tick
  long long periodicIntervalUs() const;

  /// Check if alarm interrupt should fire
  bool checkAlarm();

  uint8_t m_cmosIndex = 0;
  bool m_nmiEnabled = true;
  std::array<uint8_t, 256> m_cmosRAM{};
  PIC8259* m_pic = nullptr;

  // IRQ8 tracking
  uint64_t m_pendingIRQ8 = 0;
  std::chrono::steady_clock::time_point m_lastAdvance;
  long long m_accumulatedUs = 0;
};

} // namespace fador::hw
