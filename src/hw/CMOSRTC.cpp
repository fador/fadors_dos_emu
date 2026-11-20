#include "CMOSRTC.hpp"
#include "../utils/Logger.hpp"

#include <algorithm>
#include <chrono>

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
  if (port != 0x71)
    return 0xFF; // port 0x70 is write-only

  const uint8_t idx = m_cmosIndex & 0x7F; // index is 7-bit
  if (idx <= 0x0D)
    sync(); // clock registers and flags follow virtual time

  if (idx == 0x0A) {
    const bool uip = dividerRunning() && !(m_cmosRAM[0x0B] & 0x80) &&
                     nextSecondTick() - static_cast<int64_t>(m_lastTick) <=
                         static_cast<int64_t>(kUipTicks);
    return static_cast<uint8_t>((m_cmosRAM[0x0A] & 0x7F) | (uip ? 0x80 : 0));
  }
  if (idx == 0x0C)
    return acknowledgeInterrupt(); // read-then-clear
  return m_cmosRAM[idx];
}

void CMOSRTC::write8(uint16_t port, uint8_t value) {
  if (port == 0x70) {
    // Bit 7 = NMI disable (1 = disable, 0 = enable)
    m_nmiEnabled = (value & 0x80) == 0;
    m_cmosIndex = value & 0x7F;
    return;
  }
  if (port != 0x71)
    return;

  const uint8_t idx = m_cmosIndex & 0x7F;
  if (idx <= 0x0D)
    sync();
  switch (idx) {
  case 0x0A:
    writeStatusA(value);
    return;
  case 0x0B:
    writeStatusB(value);
    return;
  case 0x0C: // Status Register C and D are read-only
  case 0x0D:
    return;
  default:
    break;
  }

  m_cmosRAM[idx] = value;
  // Writing anywhere below 0x2E invalidates the checksum; recalculate so
  // programs that validate the checksum don't reject us.
  if (idx < 0x2E)
    recalcChecksum();
  if (idx <= 0x09)
    recomputeNextIrq();
}

uint8_t CMOSRTC::acknowledgeInterrupt() {
  sync();
  const uint8_t val = m_cmosRAM[0x0C];
  m_cmosRAM[0x0C] = 0;
  recomputeNextIrq(); // IRQF is clear again: the next flag raises IRQ 8
  return val;
}

void CMOSRTC::writeStatusA(uint8_t value) {
  // Bit 7 (UIP) is read-only.  Divider values 11x hold the chain in
  // reset; leaving reset starts it so the first update is 500 ms later.
  const bool wasRunning = dividerRunning();
  m_cmosRAM[0x0A] = value & 0x7F;
  if (!wasRunning && dividerRunning())
    m_divOrigin = static_cast<int64_t>(m_lastTick) -
                  static_cast<int64_t>(kTicksPerSecond / 2);
  recomputeNextIrq();
}

void CMOSRTC::writeStatusB(uint8_t value) {
  const uint8_t old = m_cmosRAM[0x0B];
  if (value & 0x80)
    value &= static_cast<uint8_t>(~0x10); // SET clears UIE
  // Changing the data format (binary/BCD, 12/24 h) converts the clock and
  // alarm registers, so the clock keeps the right time.
  if ((old ^ value) & 0x06) {
    for (int reg : {0x00, 0x01, 0x02, 0x03, 0x06, 0x07, 0x08, 0x09, 0x32}) {
      if ((reg == 0x01 || reg == 0x03) && (m_cmosRAM[reg] & 0xC0) == 0xC0)
        continue; // alarm "don't care"
      m_cmosRAM[reg] = encode(decode(m_cmosRAM[reg], old), value);
    }
    for (int reg : {0x04, 0x05}) {
      if (reg == 0x05 && (m_cmosRAM[reg] & 0xC0) == 0xC0)
        continue;
      m_cmosRAM[reg] = hourRaw(hour24(m_cmosRAM[reg], old), value);
    }
  }
  m_cmosRAM[0x0B] = value;
  updateIrqLine(); // enabling an interrupt whose flag is set raises it
  recomputeNextIrq();
}

// ────────────────────────────────────────────────────────────
//  time synchronisation
// ────────────────────────────────────────────────────────────

void CMOSRTC::syncTime() { setDateTime(hostLocalTime()); }

void CMOSRTC::setDateTime(const CivilDateTime &dt) {
  sync(); // pending updates happen before the new value is stored
  const uint8_t b = m_cmosRAM[0x0B];
  const int64_t days = daysFromCivil(dt.year, dt.month, dt.day);
  m_cmosRAM[0x00] = encode(dt.second, b);
  m_cmosRAM[0x02] = encode(dt.minute, b);
  m_cmosRAM[0x04] = hourRaw(dt.hour, b);
  m_cmosRAM[0x06] = encode(weekdayFromDays(days) + 1, b); // 1 = Sunday
  m_cmosRAM[0x07] = encode(dt.day, b);
  m_cmosRAM[0x08] = encode(dt.month, b);
  m_cmosRAM[0x09] = encode(dt.year % 100, b);
  m_cmosRAM[0x32] = encode(dt.year / 100, b); // century
  recomputeNextIrq();
}

CivilDateTime CMOSRTC::dateTime() {
  sync();
  const uint8_t b = m_cmosRAM[0x0B];
  CivilDateTime dt;
  dt.year = decode(m_cmosRAM[0x32], b) * 100 + decode(m_cmosRAM[0x09], b);
  dt.month = decode(m_cmosRAM[0x08], b);
  dt.day = decode(m_cmosRAM[0x07], b);
  dt.hour = hour24(m_cmosRAM[0x04], b);
  dt.minute = decode(m_cmosRAM[0x02], b);
  dt.second = decode(m_cmosRAM[0x00], b);
  return dt;
}

void CMOSRTC::setDaylightSaving(bool enabled) {
  m_cmosRAM[0x0B] = static_cast<uint8_t>((m_cmosRAM[0x0B] & ~0x01) |
                                         (enabled ? 0x01 : 0x00));
}

void CMOSRTC::advanceTime(std::chrono::nanoseconds elapsed) {
  if (elapsed.count() > 0)
    m_nowNs += static_cast<uint64_t>(elapsed.count());
  catchUp();
}

void CMOSRTC::sync() {
  if (m_timeSource) {
    const uint64_t now = m_timeSource();
    if (now > m_nowNs)
      m_nowNs = now;
  }
  catchUp();
}

bool CMOSRTC::dividerRunning() const {
  return (m_cmosRAM[0x0A] & 0x60) != 0x60; // 110 / 111 = divider reset
}

uint64_t CMOSRTC::periodTicks() const {
  const uint8_t rate = m_cmosRAM[0x0A] & 0x0F;
  if (rate == 0)
    return 0;
  if (rate <= 2) // 1 and 2 behave like 8 and 9 with a 32.768 kHz base
    return 1ULL << (rate + 6);
  return 1ULL << (rate - 1);
}

double CMOSRTC::periodicRateHz() const {
  const uint64_t p = periodTicks();
  return p ? static_cast<double>(kTicksPerSecond) / static_cast<double>(p)
           : 0.0;
}

int64_t CMOSRTC::nextSecondTick() const {
  const int64_t rel = static_cast<int64_t>(m_lastTick) - m_divOrigin;
  const int64_t sec = static_cast<int64_t>(kTicksPerSecond);
  return m_divOrigin + (rel / sec + 1) * sec;
}

void CMOSRTC::catchUp() {
  const uint64_t tick = ticksAt(m_nowNs);
  if (tick <= m_lastTick)
    return;
  const int64_t from = static_cast<int64_t>(m_lastTick) - m_divOrigin;
  const int64_t to = static_cast<int64_t>(tick) - m_divOrigin;
  m_lastTick = tick;
  if (dividerRunning()) {
    if (const uint64_t p = periodTicks()) {
      const auto period = static_cast<int64_t>(p);
      if (to / period > from / period)
        setFlags(0x40); // PF
    }
    if (!(m_cmosRAM[0x0B] & 0x80)) { // SET inhibits updates
      const auto sec = static_cast<int64_t>(kTicksPerSecond);
      for (int64_t s = from / sec; s < to / sec; ++s) {
        tickSecond();
        uint8_t flags = 0x10; // UF
        if (alarmMatches())
          flags |= 0x20; // AF
        setFlags(flags);
      }
    }
  }
  recomputeNextIrq();
}

void CMOSRTC::setFlags(uint8_t flags) {
  m_cmosRAM[0x0C] |= flags;
  updateIrqLine();
}

// IRQF = (PF & PIE) | (AF & AIE) | (UF & UIE); IRQ 8 fires on its rising
// edge and the line stays asserted until register C is read.
void CMOSRTC::updateIrqLine() {
  const bool irqf = (m_cmosRAM[0x0C] & m_cmosRAM[0x0B] & 0x70) != 0;
  if (irqf && !(m_cmosRAM[0x0C] & 0x80)) {
    m_cmosRAM[0x0C] |= 0x80;
    ++m_pendingIRQ8;
  }
}

void CMOSRTC::recomputeNextIrq() {
  m_nextIrqNs = kNever;
  if (!dividerRunning() || (m_cmosRAM[0x0C] & 0x80))
    return; // no new edge until register C is read
  const uint8_t b = m_cmosRAM[0x0B];
  int64_t next = std::numeric_limits<int64_t>::max();
  if (b & 0x40) {
    if (const uint64_t p = periodTicks()) {
      const auto period = static_cast<int64_t>(p);
      const int64_t rel = static_cast<int64_t>(m_lastTick) - m_divOrigin;
      next = m_divOrigin + (rel / period + 1) * period;
    }
  }
  if ((b & 0x30) && !(b & 0x80))
    next = std::min(next, nextSecondTick());
  if (next != std::numeric_limits<int64_t>::max())
    m_nextIrqNs = nsAtTick(static_cast<uint64_t>(next));
}

bool CMOSRTC::checkPendingIRQ8() {
  if (m_pendingIRQ8 == 0)
    return false;
  --m_pendingIRQ8;
  return true;
}

// One update cycle: advance seconds and carry into the calendar.
void CMOSRTC::tickSecond() {
  const uint8_t b = m_cmosRAM[0x0B];
  auto &r = m_cmosRAM;
  auto bump = [&](int reg, int limit, int reset) {
    const int v = decode(r[reg], b) + 1;
    r[reg] = encode(v >= limit ? reset : v, b);
    return v >= limit;
  };
  if (!bump(0x00, 60, 0) || !bump(0x02, 60, 0))
    return;
  const int hour = hour24(r[0x04], b) + 1;
  r[0x04] = hourRaw(hour % 24, b);
  if (hour < 24)
    return;
  r[0x06] = encode(decode(r[0x06], b) % 7 + 1, b);
  const int year = decode(r[0x32], b) * 100 + decode(r[0x09], b);
  const int month = decode(r[0x08], b);
  static constexpr int kDays[12] = {31, 28, 31, 30, 31, 30,
                                    31, 31, 30, 31, 30, 31};
  const bool leap = (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
  const int days =
      (month >= 1 && month <= 12) ? kDays[month - 1] + (month == 2 && leap) : 31;
  if (!bump(0x07, days + 1, 1) || !bump(0x08, 13, 1) || !bump(0x09, 100, 0))
    return;
  r[0x32] = encode(decode(r[0x32], b) + 1, b);
}

bool CMOSRTC::alarmMatches() const {
  auto match = [&](int alarm, int time) {
    return (m_cmosRAM[alarm] & 0xC0) == 0xC0 ||
           m_cmosRAM[alarm] == m_cmosRAM[time];
  };
  return match(0x01, 0x00) && match(0x03, 0x02) && match(0x05, 0x04);
}

int CMOSRTC::decode(uint8_t v, uint8_t regB) {
  return (regB & 0x04) ? v : fromBcd8(v);
}

uint8_t CMOSRTC::encode(int v, uint8_t regB) {
  return (regB & 0x04) ? static_cast<uint8_t>(v) : toBcd8(v);
}

int CMOSRTC::hour24(uint8_t raw, uint8_t regB) {
  if (regB & 0x02)
    return decode(raw, regB);
  const int h = decode(raw & 0x7F, regB) % 12;
  return (raw & 0x80) ? h + 12 : h;
}

uint8_t CMOSRTC::hourRaw(int hour, uint8_t regB) {
  if (regB & 0x02)
    return encode(hour, regB);
  int h12 = hour % 12;
  if (h12 == 0)
    h12 = 12;
  return static_cast<uint8_t>(encode(h12, regB) | (hour >= 12 ? 0x80 : 0));
}
// ────────────────────────────────────────────────────────────
//  initialisation helpers
// ────────────────────────────────────────────────────────────

void CMOSRTC::initializeDefaults() {
  m_cmosRAM = {}; // all zeros
  m_pendingIRQ8 = 0;
  m_lastTick = 0;
  m_divOrigin = 0;

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
  m_cmosRAM[0x14] = 0x43; // 2 floppy drives, VGA, coprocessor, floppy present

  // ── Memory Sizing ────────────────────────────────────
  // Base memory: initially 640 KiB (typical)
  setBaseMemoryKB(640);
  // Extended memory: 0 for now (caller adjusts later)
  setExtendedMemoryKB(0);

  // ── Information Flags (0x33) ─────────────────────────
  // Bit 7 = 128 KiB or more of extended memory is present (for IBM AT compat)
  m_cmosRAM[0x33] = 0x80;

  // ── Other reserved fields stay at 0x00 ────────────────

  // Date and time start from the host clock; virtual time moves them on.
  syncTime();
  recomputeNextIrq();
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

} // namespace fador::hw
