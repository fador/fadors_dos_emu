#pragma once

#include "IODevice.hpp"
#include <array>
#include <cstdint>

namespace fador::hw {

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

private:
  void initializeDefaults();
  void recalcChecksum();

  static uint8_t toBcd8(int value);
  static int fromBcd8(uint8_t bcd);

  uint8_t m_cmosIndex = 0;
  bool m_nmiEnabled = true;
  std::array<uint8_t, 256> m_cmosRAM{};
};

} // namespace fador::hw
