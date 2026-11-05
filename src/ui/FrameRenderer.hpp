#pragma once

#include "../hw/VGAController.hpp"
#include "../memory/MemoryBus.hpp"
#include <cstdint>
#include <string>
#include <vector>

namespace fador::hw {
class BIOS;
}

namespace fador::ui {

// Backend-independent rasteriser: converts the current emulated display
// (text modes, CGA, EGA/VGA planar, mode 13h / Mode-X and VBE linear
// modes) into a 32-bit ARGB image.  Used by the SDL frontend and by the
// headless screenshot / regression-test path, so both always agree on
// what the guest is showing.
class FrameRenderer {
public:
  FrameRenderer(memory::MemoryBus &memory, hw::VGAController &vga)
      : m_memory(memory), m_vga(vga) {}

  void setBIOS(const hw::BIOS *bios) { m_bios = bios; }

  // Rasterise the current screen.  Returns false when the video mode is
  // unknown (the previous image is kept in that case).
  bool render(bool drawCursor = true);

  int width() const { return m_width; }
  int height() const { return m_height; }
  const std::vector<uint32_t> &pixels() const { return m_pixels; }

  // Identifies the current video configuration (mode number, VBE mode,
  // resolution).  Changes whenever a frontend has to resize its surface.
  uint32_t modeKey() const { return m_modeKey; }

  // Writes the last rendered frame as a PNG (RGB, no compression needed
  // beyond zlib "stored" blocks, so there is no external dependency).
  bool savePNG(const std::string &path) const;

  // Plain-text dump of the 80x25 (or current size) text screen.
  std::string textScreen() const;

  uint32_t paletteToARGB(uint8_t index) const;

private:
  void resize(int w, int h);
  void renderText(bool drawCursor);
  bool renderGraphics(uint8_t mode);
  bool renderVbe();

  memory::MemoryBus &m_memory;
  hw::VGAController &m_vga;
  const hw::BIOS *m_bios = nullptr;
  int m_width = 0;
  int m_height = 0;
  uint32_t m_modeKey = 0xFFFFFFFFu;
  std::vector<uint32_t> m_pixels;
};

// Encode an ARGB image as a PNG file.
bool writePNG(const std::string &path, int width, int height,
              const std::vector<uint32_t> &argb);

} // namespace fador::ui
