#include "test_framework.hpp"
#include "hw/CMOSRTC.hpp"
#include <chrono>
#include <ctime>

using namespace fador::hw;

// ── helpers ──────────────────────────────────────────────────────
namespace {
uint8_t readReg(CMOSRTC &cmos, uint8_t idx) {
  cmos.write8(0x70, idx & 0x7F);
  return cmos.read8(0x71);
}
void writeReg(CMOSRTC &cmos, uint8_t idx, uint8_t val) {
  cmos.write8(0x70, idx & 0x7F);
  cmos.write8(0x71, val);
}
} // namespace

// ══════════════════════════════════════════════════════════════════
//  Basic I/O port dispatch
// ══════════════════════════════════════════════════════════════════

TEST_CASE("CMOSRTC: port dispatch and unmapped ports", "[HW][CMOS]") {
  CMOSRTC cmos;

  SECTION("unmapped port reads return 0xFF") {
    REQUIRE(cmos.read8(0x72) == 0xFF);
    REQUIRE(cmos.read8(0x00) == 0xFF); // DMA port, not CMOS
    REQUIRE(cmos.read8(0x6F) == 0xFF);
  }

  SECTION("unmapped port writes are ignored (no crash)") {
    cmos.write8(0x72, 0xAA);
    cmos.write8(0x6F, 0xBB);
    // No crash means pass
    REQUIRE(true);
  }

  SECTION("write index, read data, change index, read different data") {
    CMOSRTC cmos2;
    cmos2.write8(0x70, 0x0D); // Status D
    uint8_t d1 = cmos2.read8(0x71);
    REQUIRE(d1 == 0x80);

    cmos2.write8(0x70, 0x0E); // Diagnostic
    uint8_t d2 = cmos2.read8(0x71);
    REQUIRE(d2 == 0x00);
  }
}

// ══════════════════════════════════════════════════════════════════
//  Status registers
// ══════════════════════════════════════════════════════════════════

TEST_CASE("CMOSRTC: Status Register A (0x0A)", "[HW][CMOS]") {
  CMOSRTC cmos;

  SECTION("initial value has divider bits set") {
    uint8_t a = readReg(cmos, 0x0A);
    // Bits 6-4 should be 010 (time base = 32768 Hz)
    REQUIRE((a & 0x70) == 0x20);
  }

  SECTION("writable bits (6-4 and 3-0) can be changed") {
    // Set divider=111 and rate=1010
    writeReg(cmos, 0x0A, 0x7A);
    uint8_t a = readReg(cmos, 0x0A);
    REQUIRE((a & 0x70) == 0x70);
    REQUIRE((a & 0x0F) == 0x0A);
  }

  SECTION("bit 7 (UIP) is read-only") {
    uint8_t orig = readReg(cmos, 0x0A);
    writeReg(cmos, 0x0A, orig | 0x80); // Try to set UIP
    uint8_t after = readReg(cmos, 0x0A);
    // Bit 7 should be preserved (the device decides UIP)
    REQUIRE((after & 0x80) == (orig & 0x80));
  }
}

TEST_CASE("CMOSRTC: Status Register B (0x0B)", "[HW][CMOS]") {
  CMOSRTC cmos;

  SECTION("initial value: 24-hour mode, BCD") {
    uint8_t b = readReg(cmos, 0x0B);
    REQUIRE((b & 0x02) != 0); // 24-hour mode
    REQUIRE((b & 0x04) == 0); // BCD data mode
  }

  SECTION("can switch to 12-hour mode") {
    writeReg(cmos, 0x0B, 0x00); // 12-hour, BCD
    uint8_t b = readReg(cmos, 0x0B);
    REQUIRE((b & 0x02) == 0); // 12-hour mode
  }

  SECTION("bit 7 (SET) is writable") {
    writeReg(cmos, 0x0B, 0x82); // 24-hour, SET=1
    uint8_t b = readReg(cmos, 0x0B);
    REQUIRE((b & 0x80) != 0); // SET bit set
    writeReg(cmos, 0x0B, 0x02); // restore
    uint8_t b2 = readReg(cmos, 0x0B);
    REQUIRE((b2 & 0x80) == 0); // SET bit cleared
  }
}

TEST_CASE("CMOSRTC: Status Register C (0x0C) – read-then-clear", "[HW][CMOS]") {
  CMOSRTC cmos;

  SECTION("reads return 0 and are idempotent") {
    uint8_t c1 = readReg(cmos, 0x0C);
    uint8_t c2 = readReg(cmos, 0x0C);
    REQUIRE(c1 == 0x00);
    REQUIRE(c2 == 0x00);
  }

  SECTION("write to 0x0C is ignored") {
    writeReg(cmos, 0x0C, 0xFF);
    uint8_t c = readReg(cmos, 0x0C);
    REQUIRE(c == 0x00); // Still 0, write ignored
  }
}

TEST_CASE("CMOSRTC: Status Register D (0x0D) – valid CMOS RAM", "[HW][CMOS]") {
  CMOSRTC cmos;

  SECTION("bit 7 is always set") {
    uint8_t d = readReg(cmos, 0x0D);
    REQUIRE((d & 0x80) != 0);
  }

  SECTION("write to 0x0D is ignored") {
    writeReg(cmos, 0x0D, 0x00); // Try to clear valid bit
    uint8_t d = readReg(cmos, 0x0D);
    REQUIRE((d & 0x80) != 0); // Still valid
  }
}

// ══════════════════════════════════════════════════════════════════
//  Diagnostic, shutdown, floppy, equipment
// ══════════════════════════════════════════════════════════════════

TEST_CASE("CMOSRTC: diagnostic and shutdown registers", "[HW][CMOS]") {
  CMOSRTC cmos;

  SECTION("POST diagnostic (0x0E) reports no errors") {
    REQUIRE(readReg(cmos, 0x0E) == 0x00);
  }

  SECTION("shutdown status (0x0F) is soft-reset") {
    REQUIRE(readReg(cmos, 0x0F) == 0x00);
  }

  SECTION("diagnostic and shutdown are writable") {
    writeReg(cmos, 0x0E, 0x12);
    REQUIRE(readReg(cmos, 0x0E) == 0x12);
    writeReg(cmos, 0x0F, 0x09); // Shutdown code 9 (return to real mode)
    REQUIRE(readReg(cmos, 0x0F) == 0x09);
  }
}

TEST_CASE("CMOSRTC: floppy drive types (0x10)", "[HW][CMOS]") {
  CMOSRTC cmos;

  SECTION("both drives default to 1.44 MB (0x4)") {
    uint8_t floppy = readReg(cmos, 0x10);
    REQUIRE((floppy & 0x0F) == 0x04); // Drive A
    REQUIRE(((floppy >> 4) & 0x0F) == 0x04); // Drive B
  }

  SECTION("can configure different floppy types") {
    writeReg(cmos, 0x10, 0x14); // A=1.44MB, B=360KB
    uint8_t floppy = readReg(cmos, 0x10);
    REQUIRE((floppy & 0x0F) == 0x04);
    REQUIRE(((floppy >> 4) & 0x0F) == 0x01);
  }
}

TEST_CASE("CMOSRTC: equipment byte (0x14)", "[HW][CMOS]") {
  CMOSRTC cmos;

  SECTION("math coprocessor is reported present") {
    uint8_t equip = readReg(cmos, 0x14);
    REQUIRE((equip & 0x02) != 0); // Bit 1 = coprocessor
  }

  SECTION("floppy drive count is 2") {
    uint8_t equip = readReg(cmos, 0x14);
    uint8_t floppyBits = (equip >> 6) & 0x03;
    REQUIRE(floppyBits == 0x01); // 01 = 2 drives
  }
}

// ══════════════════════════════════════════════════════════════════
//  Memory sizing
// ══════════════════════════════════════════════════════════════════

TEST_CASE("CMOSRTC: base memory (0x15-0x16)", "[HW][CMOS]") {
  CMOSRTC cmos;

  SECTION("defaults to 640 KiB") {
    uint8_t lo = readReg(cmos, 0x15);
    uint8_t hi = readReg(cmos, 0x16);
    uint16_t kb = lo | (static_cast<uint16_t>(hi) << 8);
    REQUIRE(kb == 640);
  }

  SECTION("can be reconfigured") {
    cmos.setBaseMemoryKB(512);
    uint8_t lo = readReg(cmos, 0x15);
    uint8_t hi = readReg(cmos, 0x16);
    uint16_t kb = lo | (static_cast<uint16_t>(hi) << 8);
    REQUIRE(kb == 512);
  }

  SECTION("reconfiguration recalculates checksum") {
    uint8_t csumHiBefore = readReg(cmos, 0x2E);
    uint8_t csumLoBefore = readReg(cmos, 0x2F);

    cmos.setBaseMemoryKB(512);

    uint8_t csumHiAfter = readReg(cmos, 0x2E);
    uint8_t csumLoAfter = readReg(cmos, 0x2F);

    bool changed = (csumHiBefore != csumHiAfter) || (csumLoBefore != csumLoAfter);
    REQUIRE(changed);
  }
}

TEST_CASE("CMOSRTC: extended memory (0x17-0x18, 0x30-0x31)", "[HW][CMOS]") {
  CMOSRTC cmos;

  SECTION("defaults to 0 KiB extended") {
    uint8_t lo = readReg(cmos, 0x17);
    uint8_t hi = readReg(cmos, 0x18);
    REQUIRE(lo == 0x00);
    REQUIRE(hi == 0x00);
  }

  SECTION("small extended memory (e.g. 2 MiB) stored in 0x17-0x18") {
    cmos.setExtendedMemoryKB(2048); // 2 MiB
    uint8_t lo  = readReg(cmos, 0x17);
    uint8_t hi  = readReg(cmos, 0x18);
    uint16_t kb = lo | (static_cast<uint16_t>(hi) << 8);
    REQUIRE(kb == 2048);

    // Overflow registers stay 0
    REQUIRE(readReg(cmos, 0x30) == 0x00);
    REQUIRE(readReg(cmos, 0x31) == 0x00);
  }

  SECTION("exactly 65535 KiB (max in 0x17-0x18)") {
    cmos.setExtendedMemoryKB(65535);
    uint8_t lo  = readReg(cmos, 0x17);
    uint8_t hi  = readReg(cmos, 0x18);
    uint16_t kb = lo | (static_cast<uint16_t>(hi) << 8);
    REQUIRE(kb == 65535);
    REQUIRE(readReg(cmos, 0x30) == 0x00);
    REQUIRE(readReg(cmos, 0x31) == 0x00);
  }

  SECTION("large memory (>65535 KiB) uses overflow registers") {
    constexpr uint32_t k128MB_KB = 130048; // 128 MiB - 1 MiB
    cmos.setExtendedMemoryKB(k128MB_KB);

    // 0x17-0x18 should be capped at 0xFFFF
    uint8_t lo17 = readReg(cmos, 0x17);
    uint8_t hi18 = readReg(cmos, 0x18);
    REQUIRE(lo17 == 0xFF);
    REQUIRE(hi18 == 0xFF);

    // Overflow registers should hold the remainder
    uint8_t lo30 = readReg(cmos, 0x30);
    uint8_t hi31 = readReg(cmos, 0x31);
    uint16_t overflow = lo30 | (static_cast<uint16_t>(hi31) << 8);
    REQUIRE(overflow > 0);
    // 130048 - 65536 = 64512
    REQUIRE(overflow == 64512);
  }
}

// ══════════════════════════════════════════════════════════════════
//  Time registers (RTC)
// ══════════════════════════════════════════════════════════════════

TEST_CASE("CMOSRTC: RTC time registers populate on read", "[HW][CMOS]") {
  CMOSRTC cmos;

  SECTION("seconds (0x00) returns BCD value") {
    uint8_t sec = readReg(cmos, 0x00);
    // BCD: high nibble 0-5, low nibble 0-9
    REQUIRE((sec >> 4) <= 5);
    REQUIRE((sec & 0x0F) <= 9);
  }

  SECTION("minutes (0x02) returns BCD value") {
    uint8_t min = readReg(cmos, 0x02);
    REQUIRE((min >> 4) <= 5);
    REQUIRE((min & 0x0F) <= 9);
  }

  SECTION("hours (0x04) in 24-hour BCD mode") {
    // Default is 24-hour mode (Status B bit 1 = 1)
    uint8_t hour = readReg(cmos, 0x04);
    REQUIRE((hour >> 4) <= 2);
    REQUIRE((hour & 0x0F) <= 9);
    // PM bit (bit 7) should be 0 in 24-hour mode
    // Actually in 24h mode, the PM bit is not used, but values
    // 0x00-0x23 in BCD can have bit 7 set for hours >= 20
    // (e.g. 0x23 = 23:00). Just verify it's valid BCD.
  }

  SECTION("day of week (0x06) is 1-7") {
    uint8_t dow = readReg(cmos, 0x06);
    REQUIRE(dow >= 0x01);
    REQUIRE(dow <= 0x07);
  }

  SECTION("day of month (0x07) is 1-31") {
    uint8_t dom = readReg(cmos, 0x07);
    REQUIRE(dom >= 0x01);
    REQUIRE(dom <= 0x31);
  }

  SECTION("month (0x08) is 1-12") {
    uint8_t mon = readReg(cmos, 0x08);
    REQUIRE(mon >= 0x01);
    REQUIRE(mon <= 0x12);
  }

  SECTION("year (0x09) is 0-99 BCD") {
    uint8_t yr = readReg(cmos, 0x09);
    REQUIRE((yr & 0x0F) <= 9);
    // High nibble can be 0-9
  }

  SECTION("century (0x32) is valid BCD") {
    uint8_t cent = readReg(cmos, 0x32);
    // Century should be 20 for years 2000-2099
    REQUIRE((cent & 0x0F) <= 9);
  }

  SECTION("hour value in 12-hour mode") {
    // Switch to 12-hour mode
    writeReg(cmos, 0x0B, 0x00); // 12-hour, BCD, DST disabled
    uint8_t hour12 = readReg(cmos, 0x04);
    // In 12-hour mode, hours are 1-12
    uint8_t h = hour12 & 0x7F; // mask PM bit
    REQUIRE(h >= 0x01);
    REQUIRE(h <= 0x12);
    // Restore 24-hour mode
    writeReg(cmos, 0x0B, 0x02);
  }

  SECTION("all time registers consistent across a single read") {
    // Read all time registers in sequence — they should not
    // change mid-read (no wraparound glitches).
    uint8_t s  = readReg(cmos, 0x00);
    uint8_t m  = readReg(cmos, 0x02);
    uint8_t h  = readReg(cmos, 0x04);
    uint8_t dow = readReg(cmos, 0x06);
    uint8_t dom = readReg(cmos, 0x07);
    uint8_t mon = readReg(cmos, 0x08);
    uint8_t yr = readReg(cmos, 0x09);

    // All values should be valid
    REQUIRE(s <= 0x59);
    REQUIRE(m <= 0x59);
    REQUIRE(mon >= 0x01);
    REQUIRE(mon <= 0x12);
    REQUIRE(dom >= 0x01);
    REQUIRE(dom <= 0x31);
    REQUIRE(dow >= 0x01);
    REQUIRE(dow <= 0x07);
    // All BCD nibbles valid
    REQUIRE((s & 0x0F) <= 0x09);
    REQUIRE((m & 0x0F) <= 0x09);
    REQUIRE((h & 0x0F) <= 0x09);
  }
}

// ══════════════════════════════════════════════════════════════════
//  Index register and NMI
// ══════════════════════════════════════════════════════════════════

TEST_CASE("CMOSRTC: index register (port 0x70) 7-bit masking", "[HW][CMOS]") {
  CMOSRTC cmos;

  SECTION("bits 0-6 select the register, bit 7 is irrelevant for index") {
    // Select register 0x0D via index with bit 7 set
    cmos.write8(0x70, 0x8D); // 0x80 | 0x0D
    uint8_t d1 = cmos.read8(0x71);
    REQUIRE(d1 == 0x80); // Should still read reg 0x0D

    // Same index without bit 7
    cmos.write8(0x70, 0x0D);
    uint8_t d2 = cmos.read8(0x71);
    REQUIRE(d2 == 0x80);
  }

  SECTION("NMI disable bit stored but does not block reads") {
    // Set NMI disable = 1
    cmos.write8(0x70, 0x80); // NMI off, index 0
    uint8_t sec = cmos.read8(0x71); // Should still return seconds
    REQUIRE((sec & 0x0F) <= 9); // Valid BCD
  }
}

// ══════════════════════════════════════════════════════════════════
//  Hard disk types
// ══════════════════════════════════════════════════════════════════

TEST_CASE("CMOSRTC: hard disk type registers", "[HW][CMOS]") {
  CMOSRTC cmos;

  SECTION("primary master (0x12) defaults to type 0x0F") {
    REQUIRE(readReg(cmos, 0x12) == 0x0F);
  }

  SECTION("secondary master (0x19) defaults to none (0x00)") {
    REQUIRE(readReg(cmos, 0x19) == 0x00);
  }

  SECTION("can reconfigure drive types") {
    writeReg(cmos, 0x12, 0x06); // Type 6 hard disk
    writeReg(cmos, 0x19, 0x06); // Secondary also type 6
    REQUIRE(readReg(cmos, 0x12) == 0x06);
    REQUIRE(readReg(cmos, 0x19) == 0x06);
  }
}

// ══════════════════════════════════════════════════════════════════
//  Information flags
// ══════════════════════════════════════════════════════════════════

TEST_CASE("CMOSRTC: information flags (0x33)", "[HW][CMOS]") {
  CMOSRTC cmos;

  SECTION("bit 7 indicates extended memory present") {
    uint8_t flags = readReg(cmos, 0x33);
    REQUIRE((flags & 0x80) != 0);
  }
}

// ══════════════════════════════════════════════════════════════════
//  Checksum
// ══════════════════════════════════════════════════════════════════

TEST_CASE("CMOSRTC: checksum (0x2E-0x2F)", "[HW][CMOS]") {
  CMOSRTC cmos;

  SECTION("checksum is non-zero after initialization") {
    uint8_t hi = readReg(cmos, 0x2E);
    uint8_t lo = readReg(cmos, 0x2F);
    uint16_t csum = (static_cast<uint16_t>(hi) << 8) | lo;
    REQUIRE(csum != 0); // Should have calculated something
  }

  SECTION("checksum changes when configuration data changes") {
    uint8_t hiBefore = readReg(cmos, 0x2E);
    uint8_t loBefore = readReg(cmos, 0x2F);

    // Write a configuration byte (0x14 = equipment)
    writeReg(cmos, 0x14, 0x00);

    uint8_t hiAfter = readReg(cmos, 0x2E);
    uint8_t loAfter = readReg(cmos, 0x2F);

    bool changed = (hiBefore != hiAfter) || (loBefore != loAfter);
    REQUIRE(changed);
  }
}

// ══════════════════════════════════════════════════════════════════
//  Writes to RTC time area trigger checksum update
// ══════════════════════════════════════════════════════════════════

TEST_CASE("CMOSRTC: writing configuration data recalculates checksum", "[HW][CMOS]") {
  CMOSRTC cmos;

  SECTION("writing below 0x2E updates checksum") {
    uint8_t hi = readReg(cmos, 0x2E);
    uint8_t lo = readReg(cmos, 0x2F);

    // Write to register 0x14 (equipment byte, below 0x2E)
    writeReg(cmos, 0x14, 0x5D);

    uint8_t hi2 = readReg(cmos, 0x2E);
    uint8_t lo2 = readReg(cmos, 0x2F);

    // Checksum should be recalculated (may or may not change depending
    // on the values, but the function is called).
    // Actually, writing 0x14 from 0x4D to 0x5D changes one nibble,
    // so the checksum should change.
    bool changed = (hi != hi2) || (lo != lo2);
    REQUIRE(changed);
  }

  SECTION("writing below 0x10 also recalculates checksum") {
    // Write to floppy types (0x10) — this is still < 0x2E
    uint8_t hi = readReg(cmos, 0x2E);
    uint8_t lo = readReg(cmos, 0x2F);
    writeReg(cmos, 0x10, 0x24); // Different floppy types
    uint8_t hi2 = readReg(cmos, 0x2E);
    uint8_t lo2 = readReg(cmos, 0x2F);
    bool changed = (hi != hi2) || (lo != lo2);
    REQUIRE(changed);
  }
}

// ══════════════════════════════════════════════════════════════════
//  Index register bounds
// ══════════════════════════════════════════════════════════════════

TEST_CASE("CMOSRTC: index register wraps at 128", "[HW][CMOS]") {
  CMOSRTC cmos;

  SECTION("index 127 (0x7F) is the last valid register") {
    cmos.write8(0x70, 0x7F);
    uint8_t v = cmos.read8(0x71);
    // 0x7F may be uninitialized (0), but shouldn't crash
    REQUIRE(v <= 0xFF); // Sanity check
  }

  SECTION("index above 127 is masked to 7 bits") {
    // Write index 0x8D (= 141, masked to 0x0D = 13)
    cmos.write8(0x70, 0x8D);
    uint8_t v = cmos.read8(0x71);
    REQUIRE(v == 0x80); // Status D (0x0D)
  }
}

// ══════════════════════════════════════════════════════════════════
//  Consecutive reads are stable
// ══════════════════════════════════════════════════════════════════

TEST_CASE("CMOSRTC: consecutive reads of the same register", "[HW][CMOS]") {
  CMOSRTC cmos;

  SECTION("Status D is stable across reads") {
    uint8_t a = readReg(cmos, 0x0D);
    uint8_t b = readReg(cmos, 0x0D);
    REQUIRE(a == b);
    REQUIRE(a == 0x80);
  }

  SECTION("Diagnostic is stable") {
    uint8_t a = readReg(cmos, 0x0E);
    uint8_t b = readReg(cmos, 0x0E);
    REQUIRE(a == b);
  }

  SECTION("Checksum is stable without reconfiguration") {
    uint8_t hi1 = readReg(cmos, 0x2E);
    uint8_t lo1 = readReg(cmos, 0x2F);
    uint8_t hi2 = readReg(cmos, 0x2E);
    uint8_t lo2 = readReg(cmos, 0x2F);
    REQUIRE(hi1 == hi2);
    REQUIRE(lo1 == lo2);
  }
}

// ══════════════════════════════════════════════════════════════════
//  Integration: two CMOS instances are independent
// ══════════════════════════════════════════════════════════════════

TEST_CASE("CMOSRTC: independent instances", "[HW][CMOS]") {
  CMOSRTC cmos1;
  CMOSRTC cmos2;

  SECTION("modifying one does not affect the other") {
    writeReg(cmos1, 0x14, 0x00);
    uint8_t equip2 = readReg(cmos2, 0x14);
    REQUIRE(equip2 == 0x4D); // Default unchanged
  }

  SECTION("memory configuration is per-instance") {
    cmos1.setBaseMemoryKB(512);
    uint8_t lo2 = readReg(cmos2, 0x15);
    uint8_t hi2 = readReg(cmos2, 0x16);
    uint16_t kb2 = lo2 | (static_cast<uint16_t>(hi2) << 8);
    REQUIRE(kb2 == 640); // cmos2 still default
  }
}
