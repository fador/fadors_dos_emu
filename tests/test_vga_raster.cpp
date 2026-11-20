// Scanline-accurate frame capture and the vertical retrace interrupt:
// palette, display start, pel panning and split-screen changes made while
// the emulated beam is inside the frame land on the right lines.
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

struct RasterRig {
  cpu::CPU cpu;
  memory::MemoryBus mem;
  hw::VGAController vga{mem};
  hw::KeyboardController kbd;
  hw::PIT8254 pit;
  hw::PIC8259 pic{true};
  hw::BIOS bios{cpu, mem, kbd, pit, pic};
  ui::FrameRenderer frame{mem, vga};
  uint64_t fakeNs = 0;

  RasterRig() {
    mem.setVGA(&vga);
    bios.setVGA(&vga);
    vga.setTimeSource([this] { return fakeNs; });
    bios.initialize();
    frame.setBIOS(&bios);
    frame.setCaptureCursor(false);
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
  void attr(uint8_t i, uint8_t v) {
    vga.read8(0x3DA);
    vga.write8(0x3C0, static_cast<uint8_t>(i | 0x20));
    vga.write8(0x3C0, v);
  }
  void dacWrite(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    vga.write8(0x3C8, index);
    vga.write8(0x3C9, r);
    vga.write8(0x3C9, g);
    vga.write8(0x3C9, b);
  }
  static uint32_t argb(uint8_t r6, uint8_t g6, uint8_t b6) {
    auto e = [](uint8_t v) { return static_cast<uint32_t>((v << 2) | (v >> 4)); };
    return 0xFF000000u | (e(r6) << 16) | (e(g6) << 8) | e(b6);
  }
  // Move the emulated time to the start of physical scanline `line` of
  // frame `f` (the lines before it have been scanned out).
  void atLine(uint64_t f, uint32_t line) {
    const auto &t = vga.timing();
    fakeNs = f * t.frameNs + (uint64_t{line} * t.linePs + 999) / 1000;
    vga.advanceTo(fakeNs);
  }
  void endOfFrame(uint64_t f) {
    fakeNs = (f + 1) * vga.timing().frameNs - 1;
    vga.advanceTo(fakeNs);
  }
  uint32_t px(int x, int y) const {
    const auto &f = frame.completedFrame();
    return f.pixels[static_cast<size_t>(y) * f.width + x];
  }
  // Mode 13h with pixel (x, y) = colour fn(x, y)
  template <typename Fn> void fill13(Fn fn) {
    for (int y = 0; y < 200; ++y)
      for (int x = 0; x < 320; ++x)
        mem.write8(0xA0000 + static_cast<uint32_t>(y) * 320 + x,
                   static_cast<uint8_t>(fn(x, y)));
  }
  void startCapture() {
    vga.setRasterSink(&frame);
    // Start in a clean frame: request at its very beginning
    const uint64_t frameNs = vga.timing().frameNs;
    fakeNs = (fakeNs / frameNs + 1) * frameNs;
    vga.setContinuousCapture(true);
  }
  uint64_t currentFrame() const { return fakeNs / vga.timing().frameNs; }
};

} // namespace

TEST_CASE("Raster: DAC writes at given scanlines give colour bands", "[Raster]") {
  RasterRig rig;
  rig.int10(0x0013);
  rig.fill13([](int, int) { return 1; });
  rig.dacWrite(1, 63, 0, 0);
  rig.startCapture();
  const uint64_t f = rig.currentFrame();
  REQUIRE(rig.vga.timing().displayLines == 400);

  // Mode 13h is double scanned: physical line 2y shows row y
  rig.atLine(f, 100);
  rig.dacWrite(1, 0, 63, 0);
  rig.atLine(f, 201); // mid-row: row 100 started with the previous colour
  rig.dacWrite(1, 0, 0, 63);
  rig.atLine(f, 300);
  rig.dacWrite(1, 63, 63, 63);
  rig.endOfFrame(f);

  const auto &out = rig.frame.completedFrame();
  REQUIRE(out.serial == 1);
  REQUIRE(out.number == f);
  REQUIRE(out.width == 320);
  REQUIRE(out.height == 200);
  REQUIRE(rig.px(10, 0) == RasterRig::argb(63, 0, 0));
  REQUIRE(rig.px(10, 49) == RasterRig::argb(63, 0, 0));
  REQUIRE(rig.px(10, 50) == RasterRig::argb(0, 63, 0));
  REQUIRE(rig.px(319, 100) == RasterRig::argb(0, 63, 0));
  REQUIRE(rig.px(0, 101) == RasterRig::argb(0, 0, 63));
  REQUIRE(rig.px(0, 149) == RasterRig::argb(0, 0, 63));
  REQUIRE(rig.px(0, 150) == RasterRig::argb(63, 63, 63));
  REQUIRE(rig.px(0, 199) == RasterRig::argb(63, 63, 63));

  // The next frame (no changes) is a single colour again
  rig.endOfFrame(f + 1);
  REQUIRE(rig.frame.completedFrame().serial == 2);
  REQUIRE(rig.px(10, 0) == RasterRig::argb(63, 63, 63));
  REQUIRE(rig.px(10, 199) == RasterRig::argb(63, 63, 63));
}

TEST_CASE("Raster: a change in horizontal blanking applies from the next line",
          "[Raster]") {
  RasterRig rig;
  rig.int10(0x0003);
  rig.startCapture();
  const uint64_t f = rig.currentFrame();
  const auto &t = rig.vga.timing();
  // Text mode, 400 lines.  In the blanking after line 99's display the
  // beam has finished line 99; the change shows from line 100.
  rig.fakeNs = f * t.frameNs + (99 * t.linePs + t.displayPs + 999) / 1000;
  rig.dacWrite(0, 0, 0, 40); // background (attribute 07h: DAC 0)
  rig.endOfFrame(f);
  REQUIRE(rig.px(100, 99) == 0xFF000000u);
  REQUIRE(rig.px(100, 100) == RasterRig::argb(0, 0, 40));
}

TEST_CASE("Raster: mid-frame display start and pel panning changes", "[Raster]") {
  RasterRig rig;
  rig.int10(0x0013);
  rig.fill13([](int x, int y) { return (y & 0x80) ? x : y; });
  rig.startCapture();
  const uint64_t f = rig.currentFrame();

  // A new start address written during the display (10 rows further
  // down; CRTC start counts doublewords, 80 per 320-byte row) is latched
  // at the next vertical retrace: no tearing in this frame.
  rig.atLine(f, 200);
  rig.crtc(0x0C, static_cast<uint8_t>((80 * 10) >> 8));
  rig.crtc(0x0D, static_cast<uint8_t>((80 * 10) & 0xFF));
  rig.endOfFrame(f);
  const auto dac = [&](uint8_t i) { return rig.frame.paletteToARGB(i); };
  REQUIRE(rig.px(5, 0) == dac(0));
  REQUIRE(rig.px(5, 99) == dac(99));
  REQUIRE(rig.px(5, 100) == dac(100));
  // The whole next frame uses the new start
  rig.endOfFrame(f + 1);
  REQUIRE(rig.px(5, 0) == dac(10));
  REQUIRE(rig.px(5, 117) == dac(127));

  // Written during the retrace, it misses the next frame (the latch has
  // already happened) and shows in the one after.
  const auto &t = rig.vga.timing();
  rig.fakeNs = (f + 1) * t.frameNs + (t.retraceStart + 1) * t.linePs / 1000;
  rig.crtc(0x0D, static_cast<uint8_t>((80 * 20) & 0xFF));
  rig.crtc(0x0C, static_cast<uint8_t>((80 * 20) >> 8));
  rig.endOfFrame(f + 2);
  REQUIRE(rig.px(5, 0) == dac(10));
  rig.endOfFrame(f + 3);
  REQUIRE(rig.px(5, 0) == dac(20));
  // A whole-screen render() shows the registers as they are
  REQUIRE(rig.frame.render(false));
  REQUIRE(rig.frame.pixels()[5] == dac(20));
  const uint64_t g = f + 4;

  // Pel panning (mode 13h pans in pixel pairs: 2 = one pixel) from row 60
  rig.crtc(0x0C, 0);
  rig.crtc(0x0D, 0);
  rig.fill13([](int x, int) { return x; });
  rig.endOfFrame(g); // the start address change reaches the display
  rig.atLine(g + 1, 0);
  rig.attr(0x13, 0x00);
  rig.atLine(g + 1, 120);
  rig.attr(0x13, 0x02);
  rig.endOfFrame(g + 1);
  REQUIRE(rig.px(4, 59) == dac(4));
  REQUIRE(rig.px(4, 60) == dac(5));
  REQUIRE(rig.px(4, 199) == dac(5));
}

TEST_CASE("Raster: split screen with mid-frame changes", "[Raster]") {
  RasterRig rig;
  rig.int10(0x0013);
  // Rows 0-49 colour 1 at address 0, rows 100+ colour 2 (shown from start)
  rig.fill13([](int, int y) { return y < 100 ? 1 : 2; });
  rig.dacWrite(1, 0, 0, 63);
  rig.dacWrite(2, 0, 63, 0);
  // Display starts at row 100; line compare after physical line 199
  // (row 99) restarts the lower part at address 0
  rig.crtc(0x0C, static_cast<uint8_t>((80 * 100) >> 8));
  rig.crtc(0x0D, static_cast<uint8_t>((80 * 100) & 0xFF));
  rig.crtc(0x07, 0x0F); // line compare bit 8 = 0
  rig.crtc(0x09, 0x01); // line compare bit 9 = 0
  rig.crtc(0x18, 199);
  rig.startCapture();
  const uint64_t f = rig.currentFrame();

  // A new start address (row 10) does not move the lower (address 0)
  // part; a palette change at line 300 does recolour it.
  rig.atLine(f, 250);
  rig.crtc(0x0C, static_cast<uint8_t>((80 * 10) >> 8));
  rig.crtc(0x0D, static_cast<uint8_t>((80 * 10) & 0xFF));
  rig.atLine(f, 300);
  rig.dacWrite(1, 63, 0, 0);
  rig.endOfFrame(f);
  REQUIRE(rig.px(0, 0) == RasterRig::argb(0, 63, 0));
  REQUIRE(rig.px(0, 99) == RasterRig::argb(0, 63, 0));
  REQUIRE(rig.px(0, 100) == RasterRig::argb(0, 0, 63));
  REQUIRE(rig.px(0, 149) == RasterRig::argb(0, 0, 63));
  REQUIRE(rig.px(0, 150) == RasterRig::argb(63, 0, 0));
  REQUIRE(rig.px(0, 199) == RasterRig::argb(63, 0, 0));

  // Moving the split point above the beam: no split in this frame (the
  // top part now starts at row 10), the next frame splits at line 99.
  rig.atLine(f + 1, 100);
  rig.crtc(0x18, 99);
  rig.endOfFrame(f + 1);
  REQUIRE(rig.px(0, 50) == RasterRig::argb(63, 0, 0));  // row 60
  REQUIRE(rig.px(0, 120) == RasterRig::argb(0, 63, 0)); // row 130
  rig.endOfFrame(f + 2);
  REQUIRE(rig.px(0, 30) == RasterRig::argb(63, 0, 0));  // row 40
  REQUIRE(rig.px(0, 49) == RasterRig::argb(63, 0, 0));  // row 59
  REQUIRE(rig.px(0, 120) == RasterRig::argb(63, 0, 0)); // address 0 + row 70
}

TEST_CASE("Raster: frames are only rasterised on request", "[Raster]") {
  RasterRig rig;
  rig.int10(0x0013);
  rig.vga.setRasterSink(&rig.frame);
  const uint64_t frameNs = rig.vga.timing().frameNs;
  rig.fakeNs = frameNs * 10 + frameNs / 2;
  rig.vga.advanceTo(rig.fakeNs);
  rig.dacWrite(3, 1, 2, 3);
  REQUIRE(!rig.frame.captureOpen());
  REQUIRE(rig.frame.completedFrame().serial == 0);

  // A request in the middle of frame 10 captures frame 11
  rig.vga.requestFrame();
  rig.endOfFrame(10);
  REQUIRE(rig.frame.completedFrame().serial == 0);
  rig.endOfFrame(11);
  REQUIRE(rig.frame.completedFrame().serial == 1);
  REQUIRE(rig.frame.completedFrame().number == 11);
  // ... and nothing after it
  rig.endOfFrame(12);
  rig.endOfFrame(13);
  REQUIRE(rig.frame.completedFrame().serial == 1);
  REQUIRE(!rig.frame.captureOpen());

  // Frames that passed without any advanceTo or display change (the guest
  // was halted) are captured from the current state
  rig.vga.requestFrame();
  rig.fakeNs = 20 * frameNs;
  rig.vga.advanceTo(rig.fakeNs);
  REQUIRE(rig.frame.completedFrame().serial == 2);
  REQUIRE(rig.frame.completedFrame().number == 19); // the latest whole frame
}

TEST_CASE("Raster: mode change in the middle of a captured frame", "[Raster]") {
  RasterRig rig;
  rig.int10(0x0003);
  rig.startCapture();
  const uint64_t f = rig.currentFrame();
  rig.atLine(f, 150);
  rig.int10(0x0012);
  rig.endOfFrame(f + 3);
  const auto &out = rig.frame.completedFrame();
  REQUIRE(out.serial >= 1);
  REQUIRE(out.width == 640);
  REQUIRE(out.height == 480);
  REQUIRE(out.pixels.size() == 640u * 480u);
}

TEST_CASE("Raster: vertical retrace interrupt (CRTC 11h bits 4/5, 3C2h bit 7)",
          "[Raster]") {
  RasterRig rig;
  int irqs = 0;
  rig.vga.setIrqCallback([&irqs] { ++irqs; });
  rig.int10(0x0003);
  // The BIOS mode tables hold the latch cleared (bit 4 = 0)
  REQUIRE((rig.vga.crtcReg(0x11) & 0x10) == 0);
  REQUIRE(rig.vga.nextIrqNs() == UINT64_MAX);
  REQUIRE((rig.vga.read8(0x3C2) & 0x80) == 0);

  const auto &t = rig.vga.timing();
  const uint64_t frameNs = t.frameNs;
  rig.fakeNs = 5 * frameNs + 1000;
  // Enable: bit 5 clear, bit 4 set (protect bit kept)
  rig.crtc(0x11, static_cast<uint8_t>((rig.vga.crtcReg(0x11) & 0xCF) | 0x10));
  const uint64_t due = 5 * frameNs + t.retraceStart * t.linePs / 1000;
  REQUIRE(rig.vga.nextIrqNs() == due);
  rig.vga.advanceTo(due - 1);
  REQUIRE(irqs == 0);
  rig.fakeNs = due - 1;
  REQUIRE((rig.vga.read8(0x3C2) & 0x80) == 0);
  rig.fakeNs = due;
  rig.vga.advanceTo(due);
  REQUIRE(irqs == 1);
  REQUIRE((rig.vga.read8(0x3C2) & 0x80) != 0);
  // The latch stays set: no further interrupts until it is cleared
  REQUIRE(rig.vga.nextIrqNs() == UINT64_MAX);
  rig.fakeNs = due + 3 * frameNs;
  rig.vga.advanceTo(rig.fakeNs);
  REQUIRE(irqs == 1);

  // ISR: clear (bit 4 = 0), then re-enable (bit 4 = 1)
  const uint8_t r11 = rig.vga.crtcReg(0x11);
  rig.crtc(0x11, static_cast<uint8_t>(r11 & ~0x10));
  REQUIRE((rig.vga.read8(0x3C2) & 0x80) == 0);
  rig.crtc(0x11, r11);
  REQUIRE(rig.vga.nextIrqNs() == due + 4 * frameNs);
  rig.fakeNs = due + 4 * frameNs;
  rig.vga.advanceTo(rig.fakeNs);
  REQUIRE(irqs == 2);

  // Disabled (bit 5): the status latch is still set, without an interrupt
  rig.crtc(0x11, static_cast<uint8_t>((r11 & ~0x10) | 0x20));
  rig.crtc(0x11, static_cast<uint8_t>(r11 | 0x20));
  REQUIRE(rig.vga.nextIrqNs() == UINT64_MAX);
  rig.fakeNs += frameNs;
  REQUIRE((rig.vga.read8(0x3C2) & 0x80) != 0);
  rig.vga.advanceTo(rig.fakeNs);
  REQUIRE(irqs == 2);
}

TEST_CASE("Raster: INT 71h (IRQ9) is redirected to INT 0Ah", "[Raster]") {
  RasterRig rig;
  const uint32_t vec71 = (static_cast<uint32_t>(rig.mem.read16(0x71 * 4 + 2)) << 4) +
                         rig.mem.read16(0x71 * 4);
  // PUSH AX; MOV AL,20h; OUT A0h,AL; POP AX; INT 0Ah; IRET
  const uint8_t expect[] = {0x50, 0xB0, 0x20, 0xE6, 0xA0, 0x58, 0xCD, 0x0A, 0xCF};
  for (size_t i = 0; i < sizeof(expect); ++i)
    REQUIRE(rig.mem.read8(vec71 + static_cast<uint32_t>(i)) == expect[i]);
}
