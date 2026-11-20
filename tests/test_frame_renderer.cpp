// Register-driven rendering: standard BIOS modes and tweaked modes (Mode X,
// split screen, pel panning, 90x30 / 80x50 text, custom fonts) produce the
// expected framebuffer geometry and pixels.
#include "test_framework.hpp"
#include "cpu/CPU.hpp"
#include "hw/BIOS.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIC8259.hpp"
#include "hw/PIT8254.hpp"
#include "hw/VGAController.hpp"
#include "memory/MemoryBus.hpp"
#include "ui/FrameRenderer.hpp"

#include <cmath>

using namespace fador;

namespace {

struct RenderRig {
  cpu::CPU cpu;
  memory::MemoryBus mem;
  hw::VGAController vga{mem};
  hw::KeyboardController kbd;
  hw::PIT8254 pit;
  hw::PIC8259 pic{true};
  hw::BIOS bios{cpu, mem, kbd, pit, pic};
  ui::FrameRenderer frame{mem, vga};
  uint64_t fakeNs = 0;

  RenderRig() {
    mem.setVGA(&vga);
    bios.setVGA(&vga);
    bios.initialize();
    frame.setBIOS(&bios);
    vga.setTimeSource([this] { return fakeNs; });
  }
  void int10(uint16_t ax, uint16_t bx = 0, uint16_t cx = 0, uint16_t dx = 0) {
    cpu.setReg16(cpu::AX, ax);
    cpu.setReg16(cpu::BX, bx);
    cpu.setReg16(cpu::CX, cx);
    cpu.setReg16(cpu::DX, dx);
    bios.handleInterrupt(0x10);
  }
  void crtc(uint8_t i, uint8_t v) {
    vga.write8(0x3D4, i);
    vga.write8(0x3D5, v);
  }
  void seq(uint8_t i, uint8_t v) {
    vga.write8(0x3C4, i);
    vga.write8(0x3C5, v);
  }
  void gc(uint8_t i, uint8_t v) {
    vga.write8(0x3CE, i);
    vga.write8(0x3CF, v);
  }
  void attr(uint8_t i, uint8_t v) {
    vga.read8(0x3DA);
    vga.write8(0x3C0, static_cast<uint8_t>(i | 0x20));
    vga.write8(0x3C0, v);
  }
  uint32_t px(int x, int y) const {
    return frame.pixels()[static_cast<size_t>(y) * frame.width() + x];
  }
  uint32_t dac(uint8_t i) const { return frame.paletteToARGB(i); }

  // Mode 13h -> unchained 320x240 (Mode X)
  void modeX() {
    int10(0x0013);
    seq(4, 0x06);
    vga.write8(0x3C2, 0xE3);
    crtc(0x11, 0x0E);
    const uint8_t regs[][2] = {{0x06, 0x0D}, {0x07, 0x3E}, {0x09, 0x41},
                               {0x10, 0xEA}, {0x11, 0xAC}, {0x12, 0xDF},
                               {0x14, 0x00}, {0x15, 0xE7}, {0x16, 0x06},
                               {0x17, 0xE3}};
    for (auto &r : regs)
      crtc(r[0], r[1]);
  }
  void putModeX(int x, int y, uint8_t c, uint32_t pageBase = 0) {
    seq(2, static_cast<uint8_t>(1 << (x & 3)));
    mem.write8(0xA0000 + pageBase + static_cast<uint32_t>(y) * 80 + (x >> 2), c);
  }
};

} // namespace

TEST_CASE("Renderer: standard modes take their geometry from the CRTC", "[Renderer]") {
  RenderRig rig;
  const struct { uint16_t mode; int w, h; } modes[] = {
      {0x01, 360, 400}, {0x03, 720, 400}, {0x04, 320, 200}, {0x06, 640, 200},
      {0x07, 720, 400}, {0x0D, 320, 200}, {0x0E, 640, 200}, {0x10, 640, 350},
      {0x11, 640, 480}, {0x12, 640, 480}, {0x13, 320, 200}};
  for (const auto &m : modes) {
    rig.int10(m.mode);
    ui::FrameRenderer::CrtLayout layout;
    REQUIRE(rig.frame.decodeLayout(layout));
    REQUIRE(rig.frame.render(false));
    REQUIRE(rig.frame.width() == m.w);
    REQUIRE(rig.frame.height() == m.h);
  }
}

TEST_CASE("Renderer: mode 13h pixels, PEL mask and blanking", "[Renderer]") {
  RenderRig rig;
  rig.int10(0x0013);
  rig.mem.write8(0xA0000 + 5 * 320 + 7, 0x2A);
  rig.mem.write8(0xA0000 + 199 * 320 + 319, 0x1F);
  REQUIRE(rig.frame.render(false));
  REQUIRE(rig.px(7, 5) == rig.dac(0x2A));
  REQUIRE(rig.px(319, 199) == rig.dac(0x1F));
  REQUIRE(rig.px(8, 5) == rig.dac(0x00));

  rig.vga.write8(0x3C6, 0x0F);
  rig.frame.render(false);
  REQUIRE(rig.px(319, 199) == rig.dac(0x0F));
  rig.vga.write8(0x3C6, 0xFF);

  // Display start in doubleword units: +1 shifts by 4 pixels
  rig.crtc(0x0D, 0x01);
  rig.frame.render(false);
  REQUIRE(rig.px(3, 5) == rig.dac(0x2A));
  rig.crtc(0x0D, 0x00);

  // Sequencer screen off -> black; palette address source clear -> overscan
  rig.seq(1, 0x21);
  rig.frame.render(false);
  REQUIRE(rig.px(7, 5) == 0xFF000000u);
  rig.seq(1, 0x01);
  rig.int10(0x1001, 0x0400); // overscan colour 4
  rig.vga.read8(0x3DA);
  rig.vga.write8(0x3C0, 0x00); // PAS = 0
  rig.frame.render(false);
  REQUIRE(rig.px(7, 5) == rig.dac(0x04));
}

TEST_CASE("Renderer: Mode X 320x240 with page flipping and split screen", "[Renderer]") {
  RenderRig rig;
  rig.modeX();
  REQUIRE(rig.frame.render(false));
  REQUIRE(rig.frame.width() == 320);
  REQUIRE(rig.frame.height() == 240);

  rig.putModeX(5, 0, 0x11);
  rig.putModeX(318, 239, 0x22);
  const uint32_t page2 = 80 * 240;
  for (int y = 0; y < 240; ++y)
    rig.putModeX(1, y, static_cast<uint8_t>(0x40 + (y & 0x0F)), page2);
  rig.frame.render(false);
  REQUIRE(rig.px(5, 0) == rig.dac(0x11));
  REQUIRE(rig.px(318, 239) == rig.dac(0x22));

  // Flip to the second page
  rig.crtc(0x0C, static_cast<uint8_t>(page2 >> 8));
  rig.crtc(0x0D, static_cast<uint8_t>(page2 & 0xFF));
  rig.frame.render(false);
  REQUIRE(rig.px(1, 3) == rig.dac(0x43));
  REQUIRE(rig.px(5, 0) == rig.dac(0x00));

  // Split screen at line 120 (scanline 239 in the double-scanned 480):
  // the lower part restarts at address 0 (page 1)
  rig.crtc(0x07, 0x2E); // line compare bit 8 = 0
  rig.crtc(0x09, 0x01); // line compare bit 9 = 0, max scan line 1
  rig.crtc(0x18, 239);
  rig.frame.render(false);
  REQUIRE(rig.px(1, 119) == rig.dac(0x40 + (119 & 0x0F)));
  REQUIRE(rig.px(5, 120) == rig.dac(0x11));
  REQUIRE(rig.px(1, 120) == rig.dac(0x00));
}

TEST_CASE("Renderer: tweaked 256-colour geometries", "[Renderer]") {
  RenderRig rig;

  SECTION("320x400 unchained") {
    rig.int10(0x0013);
    rig.seq(4, 0x06);
    rig.crtc(0x09, 0x40); // max scan line 0: no line doubling
    rig.crtc(0x14, 0x00);
    rig.crtc(0x17, 0xE3);
    rig.frame.render(false);
    REQUIRE(rig.frame.width() == 320);
    REQUIRE(rig.frame.height() == 400);
    rig.seq(2, 0x02);
    rig.mem.write8(0xA0000 + 399 * 80, 0x33); // x=1, y=399
    rig.frame.render(false);
    REQUIRE(rig.px(1, 399) == rig.dac(0x33));
  }

  SECTION("360x480") {
    rig.int10(0x0013);
    rig.seq(4, 0x06);
    rig.vga.write8(0x3C2, 0xE7); // 28 MHz
    rig.crtc(0x11, 0x0E);
    const uint8_t regs[][2] = {{0x00, 0x6B}, {0x01, 0x59}, {0x02, 0x5A},
                               {0x03, 0x8E}, {0x04, 0x5E}, {0x05, 0x8A},
                               {0x06, 0x0D}, {0x07, 0x3E}, {0x09, 0x40},
                               {0x10, 0xEA}, {0x11, 0xAC}, {0x12, 0xDF},
                               {0x13, 0x2D}, {0x14, 0x00}, {0x15, 0xE7},
                               {0x16, 0x06}, {0x17, 0xE3}};
    for (auto &r : regs)
      rig.crtc(r[0], r[1]);
    rig.frame.render(false);
    REQUIRE(rig.frame.width() == 360);
    REQUIRE(rig.frame.height() == 480);
    rig.seq(2, 0x08);
    rig.mem.write8(0xA0000 + 479 * 90 + 89, 0x44); // x=359, y=479
    rig.frame.render(false);
    REQUIRE(rig.px(359, 479) == rig.dac(0x44));
  }

  SECTION("256x256") {
    rig.int10(0x0013);
    rig.crtc(0x11, 0x0E);
    rig.crtc(0x01, 0x3F);
    rig.crtc(0x13, 0x20);
    rig.crtc(0x12, 0xFF);
    rig.crtc(0x07, 0x1F); // VDE bit 8
    rig.frame.render(false);
    REQUIRE(rig.frame.width() == 256);
    REQUIRE(rig.frame.height() == 256);
    rig.mem.write8(0xA0000 + 255 * 256 + 255, 0x55);
    rig.frame.render(false);
    REQUIRE(rig.px(255, 255) == rig.dac(0x55));
  }
}

TEST_CASE("Renderer: 16-colour planar palette, colour select and panning", "[Renderer]") {
  RenderRig rig;
  rig.int10(0x0012);
  // Pixel (8..15, 0) colour 1, pixel 3 colour 6
  rig.vga.writePlane(0, 1, 0xFF);
  rig.vga.writePlane(1, 0, 0x10);
  rig.vga.writePlane(2, 0, 0x10);
  rig.frame.render(false);
  REQUIRE(rig.px(8, 0) == rig.dac(0x01));
  REQUIRE(rig.px(3, 0) == rig.dac(0x14)); // palette register 6 = 14h (brown)

  rig.int10(0x1000, 0x3F01); // palette register 1 -> 3Fh
  rig.frame.render(false);
  REQUIRE(rig.px(8, 0) == rig.dac(0x3F));

  // P5/P4 from the colour select register; P7/P6 always from it
  rig.attr(0x10, 0x81);
  rig.attr(0x14, 0x05);
  rig.frame.render(false);
  REQUIRE(rig.px(8, 0) == rig.dac(0x5F));
  rig.attr(0x10, 0x01);
  rig.attr(0x14, 0x00);

  // Pel panning: screen x shows memory pixel x+3
  rig.attr(0x13, 0x03);
  rig.frame.render(false);
  REQUIRE(rig.px(0, 0) == rig.dac(0x14));
  REQUIRE(rig.px(5, 0) == rig.dac(0x3F));
  REQUIRE(rig.px(4, 0) == rig.dac(0x00));

  // With PPM set, the split-screen area is not panned
  rig.crtc(0x18, 0x00);
  rig.crtc(0x07, rig.vga.crtcReg(0x07) & ~0x10);
  rig.crtc(0x09, rig.vga.crtcReg(0x09) & ~0x40); // line compare = 0
  rig.attr(0x10, 0x21);
  rig.frame.render(false);
  REQUIRE(rig.px(0, 0) == rig.dac(0x14)); // line 0 is above the split
  REQUIRE(rig.px(8, 1) == rig.dac(0x3F)); // split restarts at address 0, unpanned
  REQUIRE(rig.px(3, 1) == rig.dac(0x14));
}

TEST_CASE("Renderer: CGA modes read interleaved memory at B8000h", "[Renderer]") {
  RenderRig rig;
  rig.int10(0x0004);
  rig.mem.write8(0xB8000, 0xE4);          // pixels 3,2,1,0 on line 0
  rig.mem.write8(0xB8000 + 0x2000, 0x40); // pixel 1 on line 1
  rig.frame.render(false);
  REQUIRE(rig.frame.width() == 320);
  REQUIRE(rig.frame.height() == 200);
  REQUIRE(rig.px(0, 0) == rig.dac(0x17)); // palette 1 high intensity: white
  REQUIRE(rig.px(1, 0) == rig.dac(0x15));
  REQUIRE(rig.px(2, 0) == rig.dac(0x13));
  REQUIRE(rig.px(3, 0) == rig.dac(0x00));
  REQUIRE(rig.px(0, 1) == rig.dac(0x13));
}

TEST_CASE("Renderer: text mode attributes, blink, cursor and 9-dot cells", "[Renderer]") {
  RenderRig rig;
  rig.int10(0x0003);
  // Cell (0,0): full block, attr 9Fh (blink bit set); cell (1,0): ' ' attr 07h
  rig.mem.write8(0xB8000, 0xDB);
  rig.mem.write8(0xB8001, 0x9F);
  rig.mem.write8(0xB8002, 'A');
  rig.mem.write8(0xB8003, 0x1E);
  rig.frame.render(false);
  REQUIRE(rig.frame.width() == 720);
  REQUIRE(rig.px(0, 0) == rig.dac(0x3F));
  REQUIRE(rig.px(8, 0) == rig.dac(0x3F)); // line-graphics 9th column
  // 'A' row 0 is blank (background = palette 1), 9th column background
  REQUIRE(rig.px(9, 0) == rig.dac(0x01));
  REQUIRE(rig.px(17, 5) == rig.dac(0x01));

  SECTION("Blink phase hides blinking characters") {
    rig.fakeNs = 16 * rig.vga.timing().frameNs + 1000; // blink off phase
    rig.frame.render(true);
    REQUIRE(rig.px(0, 0) == rig.dac(0x01)); // fg = bg (blue)
  }

  SECTION("Intensity instead of blink") {
    rig.int10(0x1003, 0x0000);
    rig.mem.write8(0xB8000, ' ');
    rig.frame.render(false);
    REQUIRE(rig.px(0, 0) == rig.dac(0x39)); // bright blue background
  }

  SECTION("Cursor from CRTC 0Ah/0Bh/0Eh/0Fh") {
    rig.int10(0x0200, 0x0000, 0, 0x0203); // row 2, col 3
    REQUIRE(rig.vga.getCursorLocation() == 2 * 80 + 3);
    REQUIRE(rig.vga.crtcReg(0x0A) == 0x0D);
    REQUIRE(rig.vga.crtcReg(0x0B) == 0x0E);
    rig.mem.write8(0xB8000 + (2 * 80 + 3) * 2 + 1, 0x0E);
    rig.frame.render(true);
    REQUIRE(rig.px(27, 2 * 16 + 13) == rig.dac(0x3E));
    REQUIRE(rig.px(27, 2 * 16 + 12) == rig.dac(0x00));
    rig.frame.render(false); // screenshots omit the cursor
    REQUIRE(rig.px(27, 2 * 16 + 13) == rig.dac(0x00));
    // Cursor shape 0..15 via the registers directly
    rig.crtc(0x0A, 0x00);
    rig.crtc(0x0B, 0x0F);
    rig.frame.render(true);
    REQUIRE(rig.px(27, 2 * 16) == rig.dac(0x3E));
    rig.crtc(0x0A, 0x20); // cursor disable bit
    rig.frame.render(true);
    REQUIRE(rig.px(27, 2 * 16) == rig.dac(0x00));
  }
}

TEST_CASE("Renderer: fonts come from plane 2 (INT 10h 11xxh and direct writes)", "[Renderer]") {
  RenderRig rig;
  rig.int10(0x0003);
  rig.mem.write8(0xB8000, 'A');
  rig.mem.write8(0xB8001, 0x07);
  rig.mem.write8(0xB8002, 'B');
  rig.mem.write8(0xB8003, 0x07);

  SECTION("INT 10h AX=1110h user font") {
    // Glyph for 'A': 16 rows of 81h
    const uint32_t buf = 0x7000;
    for (int r = 0; r < 16; ++r)
      rig.mem.write8(buf + r, 0x81);
    rig.cpu.setSegReg(cpu::ES, 0x0700);
    rig.cpu.setReg16(cpu::BP, 0x0000);
    rig.int10(0x1110, 0x1000, 1, 'A');
    rig.frame.render(false);
    REQUIRE(rig.px(0, 0) == rig.dac(0x07));
    REQUIRE(rig.px(7, 15) == rig.dac(0x07));
    REQUIRE(rig.px(1, 0) == rig.dac(0x00));
  }

  SECTION("Direct plane-2 writes through the sequencer and GC") {
    rig.seq(2, 0x04);
    rig.seq(4, 0x07);
    rig.gc(4, 0x02);
    rig.gc(5, 0x00);
    rig.gc(6, 0x04);
    for (int r = 0; r < 16; ++r)
      rig.mem.write8(0xA0000 + 'B' * 32 + r, 0xF0);
    rig.seq(2, 0x03);
    rig.seq(4, 0x03);
    rig.gc(4, 0x00);
    rig.gc(5, 0x10);
    rig.gc(6, 0x0E);
    rig.frame.render(false);
    REQUIRE(rig.px(9, 0) == rig.dac(0x07));
    REQUIRE(rig.px(13, 15) == rig.dac(0x00));
  }

  SECTION("Second character map selected by attribute bit 3") {
    const uint32_t buf = 0x7000;
    for (int r = 0; r < 16; ++r)
      rig.mem.write8(buf + r, 0xFF);
    rig.cpu.setSegReg(cpu::ES, 0x0700);
    rig.cpu.setReg16(cpu::BP, 0x0000);
    rig.int10(0x1100, 0x1001, 1, 'A'); // block 1
    rig.int10(0x1103, 0x0004);         // map A = block 1, map B = block 0
    rig.mem.write8(0xB8001, 0x0F);     // bit 3: map A
    rig.frame.render(false);
    REQUIRE(rig.px(1, 0) == rig.dac(0x3F));
    REQUIRE(rig.px(10, 0) == rig.dac(0x00)); // 'B' from map B, row 0 blank
  }
}

TEST_CASE("Renderer: tweaked text modes (80x50, 90x30)", "[Renderer]") {
  RenderRig rig;

  SECTION("80x50 with the 8x8 font") {
    rig.int10(0x0003);
    rig.int10(0x1112, 0x0000);
    REQUIRE(rig.mem.read8(0x484) == 49);
    REQUIRE(rig.mem.read8(0x485) == 8);
    rig.mem.write8(0xB8000 + 49 * 160, 0xDB);
    rig.mem.write8(0xB8000 + 49 * 160 + 1, 0x0C);
    rig.frame.render(false);
    REQUIRE(rig.frame.width() == 720);
    REQUIRE(rig.frame.height() == 400);
    REQUIRE(rig.px(2, 49 * 8 + 3) == rig.dac(0x3C));
  }

  SECTION("90x30: 8-dot clocks, 480 lines") {
    rig.int10(0x0003);
    rig.seq(1, 0x01);            // 8-dot characters
    rig.vga.write8(0x3C2, 0xE7); // 28 MHz, 480-line sync polarity
    rig.crtc(0x11, 0x0C);
    const uint8_t regs[][2] = {{0x00, 0x6B}, {0x01, 0x59}, {0x02, 0x5A},
                               {0x03, 0x82}, {0x04, 0x60}, {0x05, 0x8D},
                               {0x06, 0x0B}, {0x07, 0x3E}, {0x10, 0xEA},
                               {0x11, 0x8C}, {0x12, 0xDF}, {0x13, 0x2D},
                               {0x15, 0xE7}, {0x16, 0x04}};
    for (auto &r : regs)
      rig.crtc(r[0], r[1]);
    rig.frame.render(false);
    REQUIRE(rig.frame.width() == 720);
    REQUIRE(rig.frame.height() == 480);
    const uint32_t cell = 0xB8000 + (29 * 90 + 89) * 2;
    rig.mem.write8(cell, 0xDB);
    rig.mem.write8(cell + 1, 0x1A);
    rig.frame.render(false);
    REQUIRE(rig.px(89 * 8 + 4, 29 * 16 + 8) == rig.dac(0x3A));
    REQUIRE(std::abs(rig.vga.timing().refreshHz() - 60.0) < 1.0);
  }
}
