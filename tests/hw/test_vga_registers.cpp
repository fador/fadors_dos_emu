// VGA register-file semantics: attribute flip-flop, CRTC protect bit, DAC
// read/write sequences, miscellaneous output addressing, graphics
// controller read/write modes, and CRTC-derived retrace timing driven by a
// fake time source.
#include "../test_framework.hpp"
#include "cpu/CPU.hpp"
#include "hw/BIOS.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIC8259.hpp"
#include "hw/PIT8254.hpp"
#include "hw/VGAController.hpp"
#include "memory/MemoryBus.hpp"

#include <cmath>
#include <memory>

using namespace fador;

namespace {

// Minimal machine: memory + VGA + BIOS, enough for INT 10h mode sets.
struct VgaRig {
  cpu::CPU cpu;
  memory::MemoryBus mem;
  hw::VGAController vga{mem};
  hw::KeyboardController kbd;
  hw::PIT8254 pit;
  hw::PIC8259 pic{true};
  hw::BIOS bios{cpu, mem, kbd, pit, pic};
  uint64_t fakeNs = 0;

  VgaRig() {
    mem.setVGA(&vga);
    bios.setVGA(&vga);
    bios.initialize();
    vga.setTimeSource([this] { return fakeNs; });
  }
  void int10(uint16_t ax, uint16_t bx = 0, uint16_t cx = 0, uint16_t dx = 0) {
    cpu.setReg16(cpu::AX, ax);
    cpu.setReg16(cpu::BX, bx);
    cpu.setReg16(cpu::CX, cx);
    cpu.setReg16(cpu::DX, dx);
    bios.handleInterrupt(0x10);
  }
};

void gc(hw::VGAController &vga, uint8_t index, uint8_t value) {
  vga.write8(0x3CE, index);
  vga.write8(0x3CF, value);
}

void seq(hw::VGAController &vga, uint8_t index, uint8_t value) {
  vga.write8(0x3C4, index);
  vga.write8(0x3C5, value);
}

// Unchained planar access with all planes enabled
void makePlanar(hw::VGAController &vga) {
  seq(vga, 4, 0x06);
  seq(vga, 2, 0x0F);
}

} // namespace

TEST_CASE("VGA: attribute controller flip-flop and readback", "[VGA]") {
  memory::MemoryBus mem;
  hw::VGAController vga(mem);

  // Reading input status 1 resets the flip-flop to "index"
  vga.read8(0x3DA);
  vga.write8(0x3C0, 0x05);          // index 5, PAS=0 (palette writable)
  REQUIRE(vga.read8(0x3C0) == 0x05);
  vga.write8(0x3C0, 0x2A);          // data
  REQUIRE(vga.attrReg(0x05) == 0x2A);
  REQUIRE(vga.read8(0x3C1) == 0x2A);
  REQUIRE(!vga.attrPaletteSourceEnabled());

  // Next write is an index again
  vga.write8(0x3C0, 0x10);
  vga.write8(0x3C0, 0x41);
  REQUIRE(vga.attrReg(0x10) == 0x41);

  // A 3DAh read in the middle of an index/data pair restarts at index
  vga.write8(0x3C0, 0x11);          // index 11h
  vga.read8(0x3DA);                 // reset
  vga.write8(0x3C0, 0x12);          // treated as index 12h
  vga.write8(0x3C0, 0x07);          // data for 12h
  REQUIRE(vga.attrReg(0x11) == 0x00);
  REQUIRE(vga.attrReg(0x12) == 0x07);

  // With PAS=1 the internal palette is read-only, other registers are not
  vga.write8(0x3C0, 0x20 | 0x03);
  REQUIRE(vga.attrPaletteSourceEnabled());
  vga.write8(0x3C0, 0x3F);
  REQUIRE(vga.attrReg(0x03) == 0x00);
  vga.write8(0x3C0, 0x20 | 0x13);
  vga.write8(0x3C0, 0x04);
  REQUIRE(vga.attrReg(0x13) == 0x04);
  REQUIRE(vga.read8(0x3C0) == 0x33);
}

TEST_CASE("VGA: CRTC protect bit and overflow registers", "[VGA]") {
  memory::MemoryBus mem;
  hw::VGAController vga(mem);
  auto crtc = [&](uint8_t i, uint8_t v) {
    vga.write8(0x3D4, i);
    vga.write8(0x3D5, v);
  };
  crtc(0x00, 0x5F);
  crtc(0x07, 0x00);
  crtc(0x11, 0x8E); // protect on
  crtc(0x00, 0x2D); // ignored
  crtc(0x06, 0x0B); // ignored
  crtc(0x07, 0xFF); // only bit 4 (line compare bit 8) is writable
  crtc(0x12, 0xDF); // not protected
  REQUIRE(vga.crtcReg(0x00) == 0x5F);
  REQUIRE(vga.crtcReg(0x06) == 0x00);
  REQUIRE(vga.crtcReg(0x07) == 0x10);
  REQUIRE(vga.crtcReg(0x12) == 0xDF);
  vga.write8(0x3D4, 0x00);
  REQUIRE(vga.read8(0x3D5) == 0x5F);

  crtc(0x11, 0x0E); // protect off
  crtc(0x06, 0x0B);
  crtc(0x07, 0x3E);
  crtc(0x09, 0x40);
  crtc(0x18, 0x2F);
  REQUIRE(vga.verticalTotal() == 0x20B);
  REQUIRE(vga.verticalDisplayEnd() == 0x1DF);
  REQUIRE(vga.lineCompare() == 0x32F);
}

TEST_CASE("VGA: DAC read/write sequences, state and PEL mask", "[VGA]") {
  memory::MemoryBus mem;
  hw::VGAController vga(mem);

  vga.write8(0x3C8, 0xFE);
  for (uint8_t v : {1, 2, 3, 4, 5, 6, 7, 8, 9})
    vga.write8(0x3C9, v); // FEh, FFh, then wraps to 00h
  REQUIRE(vga.read8(0x3C7) == 0x00); // write mode
  REQUIRE(vga.read8(0x3C8) == 0x01);
  const uint8_t *pal = mem.dacPalette();
  REQUIRE(pal[0xFE * 3] == 1);
  REQUIRE(pal[0xFF * 3 + 2] == 6);
  REQUIRE(pal[0] == 7);

  vga.write8(0x3C9, 0xFF); // values are 6-bit
  REQUIRE(pal[1 * 3] == 0x3F);

  vga.write8(0x3C7, 0xFF);
  REQUIRE(vga.read8(0x3C7) == 0x03); // read mode
  REQUIRE(vga.read8(0x3C8) == 0x00); // write index = read index + 1
  REQUIRE(vga.read8(0x3C9) == 4);
  REQUIRE(vga.read8(0x3C9) == 5);
  REQUIRE(vga.read8(0x3C9) == 6);
  REQUIRE(vga.read8(0x3C9) == 7); // auto-increment to entry 0
  REQUIRE(vga.read8(0x3C9) == 8);

  // A new write address restarts the R/G/B cycle
  vga.write8(0x3C8, 0x10);
  vga.write8(0x3C9, 0x11);
  vga.write8(0x3C8, 0x20);
  vga.write8(0x3C9, 0x21);
  REQUIRE(pal[0x20 * 3] == 0x21);
  REQUIRE(pal[0x10 * 3] == 0x11);

  REQUIRE(vga.read8(0x3C6) == 0xFF);
  vga.write8(0x3C6, 0x0F);
  REQUIRE(vga.read8(0x3C6) == 0x0F);
  REQUIRE(vga.pelMask() == 0x0F);
}

TEST_CASE("VGA: misc output selects colour or mono CRTC ports", "[VGA]") {
  memory::MemoryBus mem;
  hw::VGAController vga(mem);
  REQUIRE(vga.read8(0x3CC) == 0x67);
  vga.write8(0x3D4, 0x0E);
  vga.write8(0x3D5, 0x12);
  REQUIRE(vga.read8(0x3B5) == 0xFF); // mono port not decoded

  vga.write8(0x3C2, 0x66); // mono addressing
  REQUIRE(vga.read8(0x3CC) == 0x66);
  REQUIRE(vga.read8(0x3D5) == 0xFF);
  REQUIRE(vga.read8(0x3DA) == 0xFF);
  vga.write8(0x3B4, 0x0E);
  REQUIRE(vga.read8(0x3B5) == 0x12);
  REQUIRE((vga.read8(0x3BA) & ~0x09) == 0);

  // Feature control: written at 3BAh/3DAh, read at 3CAh
  vga.write8(0x3BA, 0x02);
  REQUIRE(vga.read8(0x3CA) == 0x02);
  // Sequencer/GC readback including reserved-bit masking
  seq(vga, 1, 0xFF);
  REQUIRE(vga.read8(0x3C5) == 0x3D);
  gc(vga, 6, 0xFF);
  REQUIRE(vga.read8(0x3CF) == 0x0F);
}

TEST_CASE("VGA: read mode 1 colour compare and don't care", "[VGA]") {
  memory::MemoryBus mem;
  hw::VGAController vga(mem);
  makePlanar(vga);
  // Pixel colours along one byte: planes 0..3
  vga.writePlane(0, 0x10, 0b11110000);
  vga.writePlane(1, 0x10, 0b11001100);
  vga.writePlane(2, 0x10, 0b10101010);
  vga.writePlane(3, 0x10, 0b00000000);
  gc(vga, 5, 0x08);     // read mode 1
  gc(vga, 2, 0x07);     // compare colour 7
  gc(vga, 7, 0x0F);     // all planes significant
  REQUIRE(vga.planeRead8(0x10) == 0b10000000);
  gc(vga, 7, 0x03);     // only planes 0,1: colours x011
  REQUIRE(vga.planeRead8(0x10) == 0b11000000);
  gc(vga, 7, 0x00);     // nothing significant: all match
  REQUIRE(vga.planeRead8(0x10) == 0xFF);
  gc(vga, 5, 0x00);     // read mode 0 uses read map select
  gc(vga, 4, 0x01);
  REQUIRE(vga.planeRead8(0x10) == 0b11001100);
}

TEST_CASE("VGA: write modes 0-3 with latches, rotate and logical ops", "[VGA]") {
  memory::MemoryBus mem;
  hw::VGAController vga(mem);
  makePlanar(vga);
  for (int p = 0; p < 4; ++p)
    vga.writePlane(p, 0, static_cast<uint8_t>(0x0F << p));

  SECTION("Mode 0: rotate, set/reset, bit mask, XOR") {
    vga.planeRead8(0); // load latches
    gc(vga, 3, 0x18 | 0x04); // XOR, rotate right 4
    gc(vga, 1, 0x02);        // plane 1 from set/reset
    gc(vga, 0, 0x02);        // set/reset plane 1 = 1s
    gc(vga, 8, 0xF0);        // only high nibble
    vga.planeWrite8(0, 0x0F); // rotated -> F0h
    // plane 0: latch 0F ^ F0 = FF, masked high -> F0 | (0F & 0F)
    REQUIRE(vga.readPlane(0, 0) == 0xFF);
    // plane 1: latch 1E, data FFh (set/reset) ^ 1E = E1 -> E0 | 0E
    REQUIRE(vga.readPlane(1, 0) == 0xEE);
    // plane 2: latch 3C ^ F0 = CC -> C0 | 0C
    REQUIRE(vga.readPlane(2, 0) == 0xCC);
    // plane 3: latch 78 ^ F0 = 88 -> 80 | 08
    REQUIRE(vga.readPlane(3, 0) == 0x88);
  }

  SECTION("Mode 1: latch copy ignores bit mask and CPU data") {
    vga.planeRead8(0);
    gc(vga, 5, 0x01);
    gc(vga, 8, 0x00);
    vga.planeWrite8(5, 0x55);
    for (int p = 0; p < 4; ++p)
      REQUIRE(vga.readPlane(p, 5) == static_cast<uint8_t>(0x0F << p));
  }

  SECTION("Mode 2: colour fill with AND") {
    vga.planeRead8(0);
    gc(vga, 5, 0x02);
    gc(vga, 3, 0x08); // AND
    gc(vga, 8, 0xFF);
    vga.planeWrite8(0, 0x05); // colour 0101b
    REQUIRE(vga.readPlane(0, 0) == 0x0F); // FF & 0F
    REQUIRE(vga.readPlane(1, 0) == 0x00);
    REQUIRE(vga.readPlane(2, 0) == 0x3C);
    REQUIRE(vga.readPlane(3, 0) == 0x00);
  }

  SECTION("Mode 3: rotated data ANDed with bit mask selects set/reset") {
    vga.planeRead8(0);
    gc(vga, 5, 0x03);
    gc(vga, 0, 0x09);  // colour 1001b
    gc(vga, 3, 0x01);  // rotate right 1
    gc(vga, 8, 0x3C);
    vga.planeWrite8(0, 0x0F); // rotated 87h & 3Ch = 04h
    REQUIRE(vga.readPlane(0, 0) == (0x0F | 0x04));
    REQUIRE(vga.readPlane(1, 0) == (0x1E & ~0x04));
    REQUIRE(vga.readPlane(2, 0) == (0x3C & ~0x04));
    REQUIRE(vga.readPlane(3, 0) == (0x78 | 0x04));
  }

  SECTION("Map mask limits the written planes") {
    seq(vga, 2, 0x04);
    gc(vga, 5, 0x00);
    vga.planeWrite8(9, 0xA5);
    REQUIRE(vga.readPlane(2, 9) == 0xA5);
    REQUIRE(vga.readPlane(0, 9) == 0x00);
  }
}

TEST_CASE("VGA: chain-4, odd/even and memory map decoding", "[VGA]") {
  memory::MemoryBus mem;
  hw::VGAController vga(mem);

  SECTION("Chain-4 goes through set/reset when enabled") {
    gc(vga, 1, 0x0F);
    gc(vga, 0, 0x05);
    vga.planeWrite8(0x102, 0x00); // plane 2, offset 100h
    REQUIRE(vga.readPlane(2, 0x100) == 0xFF);
    REQUIRE(vga.readPlane(0, 0x100) == 0x00);
  }

  SECTION("Host odd/even") {
    seq(vga, 4, 0x02); // odd/even, no chain-4
    gc(vga, 5, 0x10);
    vga.planeWrite8(0x20, 'A');
    vga.planeWrite8(0x21, 0x1F);
    REQUIRE(vga.readPlane(0, 0x20) == 'A');
    REQUIRE(vga.readPlane(1, 0x20) == 0x1F);
    REQUIRE(vga.readPlane(2, 0x20) == 'A'); // map mask 0Fh: planes 0 and 2
    REQUIRE(vga.planeRead8(0x21) == 0x1F);
  }

  SECTION("Memory map B8000h leaves A0000h undecoded") {
    gc(vga, 6, 0x0E);
    vga.planeWrite8(0, 0x12);
    REQUIRE(vga.readPlane(0, 0) == 0x00);
    REQUIRE(vga.planeRead8(0) == 0xFF);
  }
}

TEST_CASE("VGA: retrace timing follows the programmed CRTC", "[VGA]") {
  VgaRig rig;
  auto &vga = rig.vga;

  SECTION("Mode 3: 70 Hz, 449 lines") {
    rig.int10(0x0003);
    const auto &t = vga.timing();
    REQUIRE(t.totalLines == 449);
    REQUIRE(t.displayLines == 400);
    REQUIRE(std::fabs(t.refreshHz() - 70.08) < 0.05);
    // Start of frame: display enabled
    rig.fakeNs = 0;
    REQUIRE(vga.read8(0x3DA) == 0x00);
    REQUIRE(vga.currentScanline() == 0);
    // Late in line 10: horizontal blanking only
    rig.fakeNs = (10 * t.linePs + t.linePs - 1000000) / 1000;
    REQUIRE(vga.currentScanline() == 10);
    REQUIRE(vga.read8(0x3DA) == 0x01);
    // Line 405: vertical blanking, not yet retrace
    rig.fakeNs = 405 * t.linePs / 1000 + 100;
    REQUIRE(vga.read8(0x3DA) == 0x01);
    // Retrace starts at line 412 (CRTC 10h + overflow) for 2 lines
    rig.fakeNs = 412 * t.linePs / 1000 + 100;
    REQUIRE(vga.read8(0x3DA) == 0x09);
    rig.fakeNs = 414 * t.linePs / 1000 + 100;
    REQUIRE(vga.read8(0x3DA) == 0x01);
    // Next frame wraps
    rig.fakeNs = t.frameNs + 100;
    REQUIRE(vga.read8(0x3DA) == 0x00);
    REQUIRE(vga.frameCounter() == 1);
  }

  SECTION("Mode 12h: 60 Hz, 525 lines") {
    rig.int10(0x0012);
    const auto &t = vga.timing();
    REQUIRE(t.totalLines == 525);
    REQUIRE(t.displayLines == 480);
    REQUIRE(std::fabs(t.refreshHz() - 59.94) < 0.05);
    rig.fakeNs = 490 * t.linePs / 1000 + 100;
    REQUIRE(vga.read8(0x3DA) == 0x09);
  }

  SECTION("Mode 13h: 70 Hz") {
    rig.int10(0x0013);
    REQUIRE(std::fabs(vga.timing().refreshHz() - 70.08) < 0.05);
  }

  SECTION("Reprogramming the vertical total changes the rate") {
    rig.int10(0x0013);
    vga.write8(0x3D4, 0x11);
    vga.write8(0x3D5, 0x0E); // unprotect
    vga.write8(0x3D4, 0x06);
    vga.write8(0x3D5, 0x0D);
    vga.write8(0x3D4, 0x07);
    vga.write8(0x3D5, 0x3E); // Mode X 240-line timing: 527 lines
    REQUIRE(vga.timing().totalLines == 527);
    REQUIRE(std::fabs(vga.timing().refreshHz() - 59.7) < 0.1);
  }
}
