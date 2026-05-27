#include "CMOSRTC.hpp"
#include "../utils/Logger.hpp"

#include <chrono>
#include <ctime>

namespace fador::hw {

// ────────────────────────────────────────────────────────────
//  public API
// ────────────────────────────────────────────────────────────

CMOSRTC::CMOSRTC() {
  initializeDefaults();
  LOG_INFO("CMOS/RTC initialized (256 bytes NVRAM)");
}

void CMOSRTC::setBaseMemoryKB(uint16_t kb) {
  m_cmosRAM[0x15] = static_cast<uint8_t>(kb & 0xFF);
  m_cmosRAM[0x16] = static_cast<uint8_t>((kb >> 8) & 0xFF);
  recalcChecksum();
}

void CMOSRTC::setExtendedMemoryKB(uint32_t kb) {
  // Standard extended-memory registers 0x17-0x18 hold up to 65535 KiB.
  // Registers 0x30-0x31 (introduced by later BIOSes) hold the overflow.
  uint16_t lower = (kb > 0xFFFFu) ? 0xFFFFu : static_cast<uint16_t>(kb);
  uint16_t upper = (kb > 0xFFFFu) ? static_cast<uint16_t>(kb - 0x10000u) : 0u;

  m_cmosRAM[0x17] = static_cast<uint8_t>(lower & 0xFF);
  m_cmosRAM[0x18] = static_cast<uint8_t>((lower >> 8) & 0xFF);
  m_cmosRAM[0x30] = static_cast<uint8_t>(upper & 0xFF);
  m_cmosRAM[0x31] = static_cast<uint8_t>((upper >> 8) & 0xFF);
  recalcChecksum();
}

uint8_t CMOSRTC::read8(uint16_t port) {
  if (port == 0x71) {
    // Reading port 0x71 returns the byte at the currently selected index.
    uint8_t idx = m_cmosIndex & 0x7F; // index is 7-bit

    // RTC time registers – refresh from host clock on first time-register read
    if (idx <= 0x09) {
      syncTime();
    }

    // Status Register C is read-then-clear (bit 7 IRQ flag)
    if (idx == 0x0C) {
      uint8_t val = m_cmosRAM[0x0C];
      m_cmosRAM[0x0C] = 0;
      return val;
    }

    return m_cmosRAM[idx];
  }
  // Port 0x70 is write-only; reads return 0xFF
  return 0xFF;
}

void CMOSRTC::write8(uint16_t port, uint8_t value) {
  if (port == 0x70) {
    // Bit 7 = NMI disable (1 = disable, 0 = enable)
    m_nmiEnabled = (value & 0x80) == 0;
    m_cmosIndex = value & 0x7F;
  } else if (port == 0x71) {
    uint8_t idx = m_cmosIndex & 0x7F;

    // Status Register A: only bits 6-4 (divider) are writable; bit 7 is RO
    if (idx == 0x0A) {
      m_cmosRAM[0x0A] = (m_cmosRAM[0x0A] & 0x80) | (value & 0x70) | (value & 0x0F);
      return;
    }
    // Status Register B: mask read-only reserved bits
    if (idx == 0x0B) {
      m_cmosRAM[0x0B] = value & 0x7F;
      return;
    }
    // Status Register C and D are read-only
    if (idx == 0x0C || idx == 0x0D) {
      return;
    }

    // All other registers are writable
    if (idx < 0x80) {
      m_cmosRAM[idx] = value;

      // Writing anywhere below 0x2E invalidates the checksum;
      // recalculate so programs that validate the checksum don't reject us.
      // Exclude the RTC time area (0x00-0x09) because those are RO in
      // many implementations, but for simplicity we let writes through
      // and fix the checksum regardless.
      if (idx < 0x2E) {
        recalcChecksum();
      }
    }
  }
}

// ────────────────────────────────────────────────────────────
//  time synchronisation
// ────────────────────────────────────────────────────────────

void CMOSRTC::syncTime() {
  auto now = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm lt{};
#ifdef _WIN32
  localtime_s(&lt, &t);
#else
  localtime_r(&t, &lt);
#endif

  // Registers 0x00–0x09 (BCD format expected by DOS programs)
  m_cmosRAM[0x00] = toBcd8(lt.tm_sec);          // seconds
  m_cmosRAM[0x02] = toBcd8(lt.tm_min);          // minutes
  // Hours: honour 24/12h flag in Status B
  if (m_cmosRAM[0x0B] & 0x02) {
    // 24-hour mode
    m_cmosRAM[0x04] = toBcd8(lt.tm_hour);
  } else {
    // 12-hour mode
    int hour12 = lt.tm_hour % 12;
    if (hour12 == 0)
      hour12 = 12;
    m_cmosRAM[0x04] = toBcd8(hour12);
    if (lt.tm_hour >= 12)
      m_cmosRAM[0x04] |= 0x80; // PM bit
  }

  m_cmosRAM[0x06] = toBcd8(lt.tm_wday + 1);     // day of week (1=Sun)
  m_cmosRAM[0x07] = toBcd8(lt.tm_mday);          // day of month
  m_cmosRAM[0x08] = toBcd8(lt.tm_mon + 1);       // month
  m_cmosRAM[0x09] = toBcd8((lt.tm_year + 1900) % 100); // year within century
  m_cmosRAM[0x32] = toBcd8((lt.tm_year + 1900) / 100); // century
}

// ────────────────────────────────────────────────────────────
//  initialisation helpers
// ────────────────────────────────────────────────────────────

void CMOSRTC::initializeDefaults() {
  m_cmosRAM = {}; // all zeros
  m_lastAdvance = std::chrono::steady_clock::now();
  m_accumulatedUs = 0;
  m_pendingIRQ8 = 0;

  // ── Status Register A (0x0A) ──────────────────────────
  // Bits 6-4 = 010 (time base = 32768 Hz)
  // Bits 3-0 = 0110 (rate = 1.024 ms / 976 Hz interrupt)
  // Bit 7 = UIP (0 = no update in progress)
  m_cmosRAM[0x0A] = 0x26;

  // ── Status Register B (0x0B) ──────────────────────────
  // Bit 0 = DST (1 = enable)
  // Bit 1 = 24/12 hour (1 = 24-hour)
  // Bit 2 = data mode (1 = binary, 0 = BCD) → 0 = BCD
  // Bit 3 = square wave enable
  // Bit 4 = update ended interrupt
  // Bit 5 = alarm interrupt
  // Bit 6 = periodic interrupt
  // Bit 7 = SET (1 = clock updates inhibited, 0 = normal)
  m_cmosRAM[0x0B] = 0x02; // 24-hour, DST disabled, BCD

  // ── Status Register C (0x0C): read-only, starts 0 ────
  m_cmosRAM[0x0C] = 0x00;

  // ── Status Register D (0x0D): bit 7 = valid CMOS RAM ──
  m_cmosRAM[0x0D] = 0x80;

  // ── Diagnostic Status (0x0E) ─────────────────────────
  // 0x00 = no errors (successful POST)
  m_cmosRAM[0x0E] = 0x00;

  // ── Shutdown Status (0x0F) ───────────────────────────
  // 0x00 = soft reset / normal power-on (not a protected-mode shutdown)
  m_cmosRAM[0x0F] = 0x00;

  // ── Floppy Drive Types (0x10) ────────────────────────
  // Upper nibble = drive 1, lower nibble = drive 0
  // 0x04 = 1.44 MB 3.5" (both drives)
  m_cmosRAM[0x10] = 0x44;

  // ── Hard Disk Types (0x12, 0x19) ─────────────────────
  // 0x0F = user-defined / type 15 (generic), 0x00 = none
  m_cmosRAM[0x12] = 0x0F; // primary master
  m_cmosRAM[0x19] = 0x00; // secondary master (none)

  // ── Equipment Byte (0x14) ────────────────────────────
  // Bit 7-6 = number of floppy drives minus 1 (01 = 2 drives)
  // Bit 5-4 = display type (00 = EGA/VGA)
  // Bit 3-2 = reserved
  // Bit 1 = math coprocessor present (1)
  // Bit 0 = boot from floppy (0 = not specified)
  m_cmosRAM[0x14] = 0x4D; // 2 floppy drives, VGA, coprocessor

  // ── Memory Sizing ────────────────────────────────────
  // Base memory: initially 640 KiB (typical)
  setBaseMemoryKB(640);
  // Extended memory: 0 for now (caller adjusts later)
  setExtendedMemoryKB(0);

  // ── Information Flags (0x33) ─────────────────────────
  // Bit 7 = 128 KiB or more of extended memory is present (for IBM AT compat)
  m_cmosRAM[0x33] = 0x80;

  // ── Other reserved fields stay at 0x00 ────────────────

  // Sync the RTC time registers from host clock
  syncTime();
}

// ────────────────────────────────────────────────────────────
//  checksum
// ────────────────────────────────────────────────────────────

void CMOSRTC::recalcChecksum() {
  uint16_t sum = 0;
  // The checksum covers addresses 0x10 through 0x2D on most BIOSes.
  // Some also include 0x00-0x0F; the common denominator is 0x10-0x2D.
  for (int i = 0x10; i <= 0x2D; ++i) {
    sum += m_cmosRAM[i];
  }
  m_cmosRAM[0x2E] = static_cast<uint8_t>((sum >> 8) & 0xFF);   // high
  m_cmosRAM[0x2F] = static_cast<uint8_t>(sum & 0xFF);          // low
}

uint8_t CMOSRTC::toBcd8(int value) {
  return static_cast<uint8_t>(((value / 10) << 4) | (value % 10));
}

int CMOSRTC::fromBcd8(uint8_t bcd) {
  return ((bcd >> 4) & 0x0F) * 10 + (bcd & 0x0F);
}

int CMOSRTC::periodicRateHz() const {
  uint8_t rate = m_cmosRAM[0x0A] & 0x0F;
  if (rate == 0) return 0;
  static const int rates[] = {
    0, 256, 128, 8192, 4096, 2048, 1024,
    512, 256, 128, 64, 32, 16, 8, 4, 2
  };
  return (rate <= 15) ? rates[rate] : 1024;
}

long long CMOSRTC::periodicIntervalUs() const {
  int hz = periodicRateHz();
  if (hz == 0) return 0;
  return 1000000LL / hz;
}

bool CMOSRTC::checkAlarm() {
  if (!(m_cmosRAM[0x0B] & 0x20)) return false; // AIE not enabled

  uint8_t alarmSec = m_cmosRAM[0x01];
  uint8_t alarmMin = m_cmosRAM[0x03];
  uint8_t alarmHr  = m_cmosRAM[0x05];

  syncTime(); // refresh current time

  uint8_t curSec = m_cmosRAM[0x00];
  uint8_t curMin = m_cmosRAM[0x02];
  uint8_t curHr  = m_cmosRAM[0x04];

  // Don't care bits: 0xC0 = "don't care" in BCD for hours, 0x80 for others
  bool secMatch = (alarmSec & 0xC0) || (fromBcd8(alarmSec & 0x3F) == fromBcd8(curSec & 0x3F));
  bool minMatch = (alarmMin & 0xC0) || (fromBcd8(alarmMin & 0x3F) == fromBcd8(curMin & 0x3F));
  bool hrMatch  = (alarmHr & 0xC0)  || (fromBcd8(alarmHr & 0x3F) == fromBcd8(curHr & 0x3F));

  return secMatch && minMatch && hrMatch;
}

void CMOSRTC::advanceTime() {
  auto now = std::chrono::steady_clock::now();
  long long elapsedUs = std::chrono::duration_cast<std::chrono::microseconds>(
      now - m_lastAdvance).count();
  m_lastAdvance = now;

  if (elapsedUs <= 0) return;

  m_accumulatedUs += elapsedUs;

  // Periodic interrupt
  bool pie = (m_cmosRAM[0x0B] & 0x40) != 0;
  if (pie) {
    long long interval = periodicIntervalUs();
    while (interval > 0 && m_accumulatedUs >= interval) {
      m_accumulatedUs -= interval;
      ++m_pendingIRQ8;
      // Set PF (Periodic Interrupt Flag) in Status C
      m_cmosRAM[0x0C] |= 0xC0; // IRQF + PF
    }
  }

  // Alarm interrupt
  if ((m_cmosRAM[0x0B] & 0x20) && checkAlarm()) {
    ++m_pendingIRQ8;
    m_cmosRAM[0x0C] |= 0xA0; // IRQF + AF
  }
}

bool CMOSRTC::checkPendingIRQ8() {
  if (m_pendingIRQ8 == 0) return false;
  --m_pendingIRQ8;
  return true;
}

} // namespace fador::hw
