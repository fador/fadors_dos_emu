// FrameRenderer with the host-SIMD kernels: every level must produce the
// same image as a per-pixel model of the VGA address mapping (including
// CRTC start addresses that wrap past the end of the 64 KiB planes).

#include "test_framework.hpp"
#include "hw/VGAController.hpp"
#include "memory/MemoryBus.hpp"
#include "ui/FrameRenderer.hpp"
#include "utils/simd/Simd.hpp"

#include <random>
#include <string>
#include <vector>

using namespace fador;
using utils::SimdLevel;

namespace {

struct VgaRig {
  memory::MemoryBus memory;
  hw::VGAController vga{memory};
  ui::FrameRenderer frame{memory, vga};

  VgaRig() {
    memory.setVGA(&vga);
    std::mt19937 rng(2024);
    uint8_t *dac = memory.dacPalette();
    for (uint32_t i = 0; i < memory::MemoryBus::DAC_SIZE; ++i)
      dac[i] = static_cast<uint8_t>(rng() & 0x3F);
    setChain4(false);
    for (int p = 0; p < 4; ++p) {
      seq(2, static_cast<uint8_t>(1u << p));
      for (uint32_t off = 0; off < 0x10000; ++off)
        vga.planeWrite8(off, static_cast<uint8_t>(rng()));
    }
    seq(2, 0x0F);
  }
  void seq(uint8_t index, uint8_t value) {
    vga.write8(0x3C4, index);
    vga.write8(0x3C5, value);
  }
  void crtc(uint8_t index, uint8_t value) {
    vga.write8(0x3D4, index);
    vga.write8(0x3D5, value);
  }
  void setChain4(bool on) { seq(4, on ? 0x0E : 0x06); }
  void setStart(uint16_t start) {
    crtc(0x0C, static_cast<uint8_t>(start >> 8));
    crtc(0x0D, static_cast<uint8_t>(start));
  }
};

// Per-pixel model of the renderer's address mapping.
std::vector<uint32_t> model(VgaRig &rig, uint8_t mode, int w, int h) {
  std::vector<uint32_t> img(static_cast<size_t>(w) * h);
  const auto &vga = rig.vga;
  const uint8_t crtcOff = vga.getCRTCOffset();
  const uint32_t start = vga.getDisplayStart();
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      uint8_t idx = 0;
      if (mode == 0x13 && vga.isChain4()) {
        const int rowBytes = crtcOff ? crtcOff * 8 : w;
        const uint32_t off = start * 4 + y * rowBytes + x;
        idx = vga.readPlane(off & 3, off & 0xFFFC); // IBM chain-4 layout
      } else if (mode == 0x13) {
        const int rowBytes = crtcOff ? crtcOff * 2 : w / 4;
        idx = vga.readPlane(x & 3, start + y * rowBytes + (x >> 2));
      } else {
        const int rowBytes = crtcOff ? crtcOff * 2 : w / 8;
        const uint32_t off = start + y * rowBytes + (x >> 3);
        const uint8_t bit = static_cast<uint8_t>(0x80 >> (x & 7));
        if (mode == 0x11) {
          idx = (vga.readPlane(0, off) & bit) ? 15 : 0;
        } else {
          for (int p = 0; p < 4; ++p)
            if (vga.readPlane(p, off) & bit)
              idx |= static_cast<uint8_t>(1 << p);
        }
      }
      img[static_cast<size_t>(y) * w + x] = rig.frame.paletteToARGB(idx);
    }
  return img;
}

void checkAllLevels(VgaRig &rig, uint8_t mode, const std::string &what) {
  rig.memory.write8(0x449, mode);
  const SimdLevel saved = utils::simd::activeLevel();
  for (SimdLevel level : {SimdLevel::Scalar, SimdLevel::SSE2, SimdLevel::SSE41,
                          SimdLevel::AVX2, SimdLevel::NEON}) {
    if (!utils::simdLevelSupported(level))
      continue;
    utils::simd::setActiveLevel(level);
    REQUIRE(rig.frame.render(false));
    const auto expect = model(rig, mode, rig.frame.width(), rig.frame.height());
    if (rig.frame.pixels() != expect)
      throw std::runtime_error("render mismatch: " + what + " at level " +
                               utils::simdLevelName(level));
  }
  utils::simd::setActiveLevel(saved);
}

} // namespace

TEST_CASE("HostSimd: FrameRenderer VGA modes match the per-pixel model",
          "[simd][video]") {
  VgaRig rig;
  SECTION("mode 13h chain-4") {
    rig.setChain4(true);
    checkAllLevels(rig, 0x13, "13h");
    rig.setStart(0x3FF1); // start * 4 wraps the plane offset
    rig.crtc(0x13, 41);
    checkAllLevels(rig, 0x13, "13h wrapped");
  }
  SECTION("Mode X (unchained)") {
    rig.setChain4(false);
    checkAllLevels(rig, 0x13, "mode X");
    rig.setStart(0xFF13);
    rig.crtc(0x13, 45);
    checkAllLevels(rig, 0x13, "mode X wrapped");
  }
  SECTION("16-colour planar 12h / 0Dh and monochrome 11h") {
    rig.setChain4(false);
    checkAllLevels(rig, 0x12, "12h");
    checkAllLevels(rig, 0x0D, "0Dh");
    checkAllLevels(rig, 0x11, "11h");
    rig.setStart(0xFFE7);
    rig.crtc(0x13, 43);
    checkAllLevels(rig, 0x12, "12h wrapped");
    checkAllLevels(rig, 0x11, "11h wrapped");
  }
}
