// INT 10h character output in every mode: text pages and mode 7, glyphs
// in CGA, EGA/VGA planar and 256-colour modes (with the XOR rule),
// AH=08h read-back by font matching, scrolling and AH=13h strings.
#include "test_framework.hpp"
#include "cpu/CPU.hpp"
#include "hw/BIOS.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIC8259.hpp"
#include "hw/PIT8254.hpp"
#include "hw/VGAController.hpp"
#include "memory/MemoryBus.hpp"
#include "ui/FrameRenderer.hpp"

using namespace fador;

namespace {

struct TextRig {
  cpu::CPU cpu;
  memory::MemoryBus mem;
  hw::VGAController vga{mem};
  hw::KeyboardController kbd;
  hw::PIT8254 pit;
  hw::PIC8259 pic{true};
  hw::BIOS bios{cpu, mem, kbd, pit, pic};
  ui::FrameRenderer frame{mem, vga};

  TextRig() {
    mem.setVGA(&vga);
    bios.setVGA(&vga);
    vga.setTimeSource([] { return uint64_t{0}; });
    bios.initialize();
    frame.setBIOS(&bios);
  }
  void int10(uint16_t ax, uint16_t bx = 0, uint16_t cx = 0, uint16_t dx = 0) {
    cpu.setReg16(cpu::AX, ax);
    cpu.setReg16(cpu::BX, bx);
    cpu.setReg16(cpu::CX, cx);
    cpu.setReg16(cpu::DX, dx);
    bios.handleInterrupt(0x10);
  }
  void gotoXY(uint8_t page, uint8_t row, uint8_t col) {
    int10(0x0200, static_cast<uint16_t>(page << 8),  0,
          static_cast<uint16_t>((row << 8) | col));
  }
  uint16_t readAt(uint8_t page, uint8_t row, uint8_t col) {
    gotoXY(page, row, col);
    int10(0x0800, static_cast<uint16_t>(page << 8));
    return cpu.getReg16(cpu::AX);
  }
  // Row r of character ch in the font INT 43h points to
  uint8_t font(uint8_t ch, int r, int height) {
    const uint32_t base = (static_cast<uint32_t>(mem.read16(0x43 * 4 + 2)) << 4) +
                          mem.read16(0x43 * 4);
    return mem.read8(base + ch * static_cast<uint32_t>(height) + r);
  }
  uint32_t cgaLine(int y) const {
    return 0xB8000 + ((y & 1) ? 0x2000u : 0u) + static_cast<uint32_t>(y >> 1) * 80u;
  }
  uint32_t px(int x, int y) const {
    return frame.pixels()[static_cast<size_t>(y) * frame.width() + x];
  }
  void writeString(uint8_t mode, uint8_t page, uint8_t attr, uint8_t row,
                   uint8_t col, const char *s, uint16_t len) {
    const uint32_t buf = 0x7000;
    const uint16_t bytes = (mode & 2) ? len * 2 : len; // char, attr pairs
    for (uint16_t i = 0; i < bytes; ++i)
      mem.write8(buf + i, static_cast<uint8_t>(s[i]));
    cpu.setSegReg(cpu::ES, 0x0700);
    cpu.setReg16(cpu::BP, 0);
    int10(static_cast<uint16_t>(0x1300 | mode),
          static_cast<uint16_t>((page << 8) | attr), len,
          static_cast<uint16_t>((row << 8) | col));
  }
};

uint16_t expand2bpp(uint8_t bits, uint8_t colour) {
  uint16_t w = 0;
  for (int b = 0; b < 8; ++b)
    if (bits & (0x80 >> b))
      w = static_cast<uint16_t>(w | ((colour & 3) << (14 - 2 * b)));
  return w;
}

} // namespace

TEST_CASE("INT 10h text: CGA mode 04h glyphs, XOR and read-back", "[VideoText]") {
  TextRig rig;
  rig.int10(0x0004);
  rig.gotoXY(0, 2, 5);
  rig.int10(0x0941, 0x0003, 1); // 'A', colour 3
  for (int r = 0; r < 8; ++r) {
    const uint32_t a = rig.cgaLine(2 * 8 + r) + 5 * 2;
    const uint16_t want = expand2bpp(rig.font('A', r, 8), 3);
    REQUIRE(rig.mem.read8(a) == (want >> 8));
    REQUIRE(rig.mem.read8(a + 1) == (want & 0xFF));
  }
  REQUIRE(rig.readAt(0, 2, 5) == 'A');
  // AH=09h does not move the cursor
  rig.int10(0x0300);
  REQUIRE(rig.cpu.getReg16(cpu::DX) == 0x0205);

  // XOR (colour bit 7) with the same glyph and colour clears it
  rig.int10(0x0941, 0x0083, 1);
  for (int r = 0; r < 8; ++r)
    REQUIRE(rig.mem.read16(rig.cgaLine(16 + r) + 10) == 0);
  REQUIRE(rig.readAt(0, 2, 5) == 0); // a blank cell matches character 0

  // CX repeats along the row
  rig.gotoXY(0, 0, 0);
  rig.int10(0x0958, 0x0002, 3);
  REQUIRE(rig.readAt(0, 0, 2) == 'X');
  REQUIRE(rig.readAt(0, 0, 3) == 0);
}

TEST_CASE("INT 10h text: CGA mode 06h teletype and the INT 1Fh upper font",
          "[VideoText]") {
  TextRig rig;
  rig.int10(0x0006);
  rig.int10(0x0E48, 0x0001);
  rig.int10(0x0E69, 0x0001);
  for (int r = 0; r < 8; ++r) {
    REQUIRE(rig.mem.read8(rig.cgaLine(r)) == rig.font('H', r, 8));
    REQUIRE(rig.mem.read8(rig.cgaLine(r) + 1) == rig.font('i', r, 8));
  }
  rig.int10(0x0300);
  REQUIRE(rig.cpu.getReg16(cpu::DX) == 0x0002);

  // Characters 80h-FFh come from the table INT 1Fh points to
  for (int r = 0; r < 8; ++r)
    rig.mem.write8(0x6000 + 8 + r, static_cast<uint8_t>(0x11 * (r + 1)));
  rig.mem.write16(0x1F * 4, 0x0000);
  rig.mem.write16(0x1F * 4 + 2, 0x0600);
  rig.gotoXY(0, 1, 0);
  rig.int10(0x0981, 0x0001, 1);
  for (int r = 0; r < 8; ++r)
    REQUIRE(rig.mem.read8(rig.cgaLine(8 + r)) == 0x11 * (r + 1));
  REQUIRE(rig.readAt(0, 1, 0) == 0x81);

  // Teletype at the bottom right scrolls the screen up (fill 0)
  rig.gotoXY(0, 24, 79);
  rig.int10(0x0E5A, 0x0001);
  rig.int10(0x0300);
  REQUIRE(rig.cpu.getReg16(cpu::DX) == 0x1800);
  REQUIRE(rig.readAt(0, 23, 79) == 'Z');
  REQUIRE(rig.readAt(0, 0, 0) == 0x81); // row 1 moved up
}

TEST_CASE("INT 10h text: planar modes 0Dh and 12h, colours, XOR and pages",
          "[VideoText]") {
  TextRig rig;
  rig.int10(0x000D);
  rig.gotoXY(0, 3, 4);
  rig.int10(0x0942, 0x000C, 1); // 'B' colour 12: planes 2 and 3
  for (int r = 0; r < 8; ++r) {
    const uint32_t off = (3 * 8 + r) * 40 + 4;
    const uint8_t g = rig.font('B', r, 8);
    REQUIRE(rig.vga.readPlane(0, off) == 0);
    REQUIRE(rig.vga.readPlane(1, off) == 0);
    REQUIRE(rig.vga.readPlane(2, off) == g);
    REQUIRE(rig.vga.readPlane(3, off) == g);
  }
  REQUIRE(rig.frame.render(false));
  // Glyph pixels show colour 12 through the attribute controller
  int lit = 0;
  for (int y = 24; y < 32; ++y)
    for (int x = 32; x < 40; ++x)
      lit += rig.px(x, y) == rig.frame.paletteToARGB(rig.vga.attrReg(12));
  REQUIRE(lit > 10);
  REQUIRE(rig.readAt(0, 3, 4) == 'B');

  // XOR colour 0Ch over it: planes 2/3 cleared; colour 03h XOR: planes 0/1
  rig.int10(0x0942, 0x008F, 1);
  for (int r = 0; r < 8; ++r) {
    const uint32_t off = (3 * 8 + r) * 40 + 4;
    const uint8_t g = rig.font('B', r, 8);
    REQUIRE(rig.vga.readPlane(0, off) == g);
    REQUIRE(rig.vga.readPlane(1, off) == g);
    REQUIRE(rig.vga.readPlane(2, off) == 0);
  }
  // Non-XOR writes clear the cell background
  rig.int10(0x092E, 0x0001, 1);
  REQUIRE(rig.vga.readPlane(1, (3 * 8 + 7) * 40 + 4) == 0);

  // Page 1 of mode 0Dh is 2000h bytes further; AH=05h shows it
  rig.gotoXY(1, 0, 0);
  rig.int10(0x0951, 0x010F, 1);
  REQUIRE(rig.vga.readPlane(0, 0x2000 + 2 * 40) == rig.font('Q', 2, 8));
  REQUIRE(rig.vga.readPlane(0, 2 * 40) == 0);
  REQUIRE(rig.readAt(1, 0, 0) == 'Q');
  REQUIRE(rig.readAt(0, 0, 0) == 0);
  rig.int10(0x0501);
  REQUIRE(rig.vga.getDisplayStart() == 0x2000);

  // Mode 12h: 8x16 cells, 80 columns
  rig.int10(0x0012);
  rig.gotoXY(0, 29, 79);
  rig.int10(0x0957, 0x0009, 1);
  for (int r = 0; r < 16; ++r) {
    const uint32_t off = (29 * 16 + r) * 80 + 79;
    REQUIRE(rig.vga.readPlane(0, off) == rig.font('W', r, 16));
    REQUIRE(rig.vga.readPlane(1, off) == 0);
    REQUIRE(rig.vga.readPlane(3, off) == rig.font('W', r, 16));
  }
  REQUIRE(rig.readAt(0, 29, 79) == 'W');
}

TEST_CASE("INT 10h text: mode 13h glyphs, read-back and scrolling", "[VideoText]") {
  TextRig rig;
  rig.int10(0x0013);
  rig.gotoXY(0, 1, 2);
  rig.int10(0x0941, 0x002A, 1);
  REQUIRE(rig.frame.render(false));
  for (int r = 0; r < 8; ++r)
    for (int x = 0; x < 8; ++x) {
      const bool on = (rig.font('A', r, 8) & (0x80 >> x)) != 0;
      REQUIRE(rig.px(16 + x, 8 + r) == rig.frame.paletteToARGB(on ? 0x2A : 0));
    }
  REQUIRE(rig.readAt(0, 1, 2) == 'A');

  // AH=06h: scroll rows 0-24 up by one, fill colour 5
  rig.int10(0x0601, 0x0500, 0x0000, 0x1827);
  REQUIRE(rig.readAt(0, 0, 2) == 'A');
  REQUIRE(rig.readAt(0, 1, 2) == 0);
  rig.frame.render(false);
  REQUIRE(rig.px(0, 199) == rig.frame.paletteToARGB(5));
  // AH=07h: scroll down by two within columns 0-9
  rig.int10(0x0702, 0x0000, 0x0000, 0x1809);
  REQUIRE(rig.readAt(0, 2, 2) == 'A');
  REQUIRE(rig.readAt(0, 0, 2) == 0);

  // AH=13h string with attributes in the string, cursor moved
  const char cells[] = {'H', 0x0C, 'I', 0x0E};
  rig.writeString(0x03, 0, 0, 10, 0, cells, 2);
  REQUIRE(rig.readAt(0, 10, 0) == 'H');
  REQUIRE(rig.readAt(0, 10, 1) == 'I');
  rig.frame.render(false);
  bool sawYellow = false;
  for (int x = 8; x < 16; ++x)
    for (int y = 80; y < 88; ++y)
      sawYellow |= rig.px(x, y) == rig.frame.paletteToARGB(0x0E);
  REQUIRE(sawYellow);
}

TEST_CASE("INT 10h text: text mode pages and the per-page cursor", "[VideoText]") {
  TextRig rig;
  rig.int10(0x0003);
  REQUIRE(rig.mem.read16(0x44C) == 0x1000);
  rig.gotoXY(2, 5, 10);
  rig.gotoXY(0, 1, 1);
  rig.int10(0x0300, 0x0200);
  REQUIRE(rig.cpu.getReg16(cpu::DX) == 0x050A);
  rig.int10(0x0300, 0x0000);
  REQUIRE(rig.cpu.getReg16(cpu::DX) == 0x0101);

  // AH=09h on page 2 writes into that page only
  rig.gotoXY(2, 5, 10);
  rig.int10(0x0941, 0x021E, 2);
  const uint32_t cell = 0xB8000 + 2 * 0x1000 + (5 * 80 + 10) * 2;
  REQUIRE(rig.mem.read16(cell) == 0x1E41);
  REQUIRE(rig.mem.read16(cell + 2) == 0x1E41);
  REQUIRE(rig.mem.read16(0xB8000 + (5 * 80 + 10) * 2) == 0x0720);
  REQUIRE(rig.readAt(2, 5, 10) == 0x1E41);
  // AH=0Ah keeps the attribute
  rig.gotoXY(2, 5, 10);
  rig.int10(0x0A42, 0x0200, 1);
  REQUIRE(rig.mem.read16(cell) == 0x1E42);

  // AH=05h: page 2 becomes visible (start address in words), the hardware
  // cursor follows that page's cursor, and teletype writes to it
  rig.int10(0x0502);
  REQUIRE(rig.mem.read8(0x462) == 2);
  REQUIRE(rig.vga.getDisplayStart() == 0x1000);
  REQUIRE(rig.vga.getCursorLocation() == 0x1000 + 5 * 80 + 10);
  rig.int10(0x0E5A);
  REQUIRE(rig.mem.read8(cell) == 'Z');
  rig.int10(0x0300, 0x0200);
  REQUIRE(rig.cpu.getReg16(cpu::DX) == 0x050B);
  rig.int10(0x0300, 0x0000);
  REQUIRE(rig.cpu.getReg16(cpu::DX) == 0x0101);
  REQUIRE(rig.frame.render(false));
  REQUIRE(rig.frame.textScreen().find("ZA") != std::string::npos);

  // AH=06h scrolls the active page; teletype scrolls at the bottom and
  // fills with the attribute at the cursor
  rig.int10(0x0601, 0x4F00, 0x0000, 0x184F);
  REQUIRE(rig.mem.read16(cell - 160) == 0x1E5A);
  REQUIRE(rig.mem.read16(0xB8000 + 2 * 0x1000 + 24 * 160) == 0x4F20);
  rig.gotoXY(2, 24, 0);
  rig.int10(0x0E0A);
  REQUIRE(rig.mem.read16(0xB8000 + 2 * 0x1000 + 23 * 160) == 0x4F20);
  REQUIRE(rig.mem.read16(0xB8000 + 2 * 0x1000 + 24 * 160) == 0x4F20);
  REQUIRE(rig.mem.read16(cell - 320) == 0x1E5A);
  // Page 0 untouched
  REQUIRE(rig.mem.read16(0xB8000) == 0x0720);

  // AH=13h mode 0 (attribute in BL, cursor not moved) on page 1
  rig.writeString(0x00, 1, 0x2F, 3, 78, "abc", 3);
  REQUIRE(rig.mem.read16(0xB8000 + 0x1000 + (3 * 80 + 78) * 2) == 0x2F61);
  REQUIRE(rig.mem.read16(0xB8000 + 0x1000 + (4 * 80) * 2) == 0x2F63);
  rig.int10(0x0300, 0x0100);
  REQUIRE(rig.cpu.getReg16(cpu::DX) == 0x0000);
  // Mode 1: cursor follows the string, CR/LF are control characters
  rig.writeString(0x01, 1, 0x1F, 0, 0, "x\r\ny", 4);
  rig.int10(0x0300, 0x0100);
  REQUIRE(rig.cpu.getReg16(cpu::DX) == 0x0101);
  REQUIRE(rig.mem.read16(0xB8000 + 0x1000 + 160) == 0x1F79);
}

TEST_CASE("INT 10h text: mode 7 uses B0000h", "[VideoText]") {
  TextRig rig;
  rig.int10(0x0007);
  rig.int10(0x0E4D);
  rig.int10(0x0E37);
  REQUIRE(rig.mem.read8(0xB0000) == 'M');
  REQUIRE(rig.mem.read8(0xB0002) == '7');
  rig.gotoXY(0, 0, 0);
  rig.int10(0x0941, 0x0070, 1);
  REQUIRE(rig.mem.read16(0xB0000) == 0x7041);
  REQUIRE(rig.readAt(0, 0, 0) == 0x7041);
  REQUIRE(rig.frame.render(false));
  REQUIRE(rig.frame.width() == 720);
  REQUIRE(rig.frame.textScreen().rfind("A7", 0) == 0);
  // The B8000h window is not used
  REQUIRE(rig.mem.read8(0xB8000) != 'A');
}

TEST_CASE("Text memory: A0000h odd/even access reaches the text buffer",
          "[VideoText]") {
  TextRig rig;
  rig.int10(0x0003);
  rig.int10(0x0E41); // 'A' at B8000h through the BIOS
  // Map the 64 KiB A0000h window (GC 06h memory map 1, odd/even kept)
  rig.vga.write8(0x3CE, 0x06);
  rig.vga.write8(0x3CF, 0x06);
  REQUIRE(rig.mem.read8(0xA0000) == 'A');
  REQUIRE(rig.mem.read8(0xA0001) == 0x07);
  rig.mem.write8(0xA0002, 'Z'); // even: plane 0 (characters)
  rig.mem.write8(0xA0003, 0x4E); // odd: plane 1 (attributes)
  REQUIRE(rig.mem.read16(0xB8002) == 0x4E5A);
  // Plane 2 (the font) is still reached by planar addressing
  rig.vga.write8(0x3C4, 0x02);
  rig.vga.write8(0x3C5, 0x04); // map mask: plane 2
  rig.vga.write8(0x3C4, 0x04);
  rig.vga.write8(0x3C5, 0x06); // sequential addressing
  rig.vga.write8(0x3CE, 0x05);
  rig.vga.write8(0x3CF, 0x00);
  rig.vga.write8(0x3CE, 0x04);
  rig.vga.write8(0x3CF, 0x02); // read map 2
  rig.mem.write8(0xA0000 + 'A' * 32, 0xFF);
  REQUIRE(rig.vga.readPlane(2, 'A' * 32) == 0xFF);
  REQUIRE(rig.mem.read8(0xA0000 + 'A' * 32) == 0xFF);
  REQUIRE(rig.mem.read16(0xB8000) == 0x0741);
  rig.frame.render(false);
  REQUIRE(rig.frame.textScreen().rfind("AZ", 0) == 0);
}
