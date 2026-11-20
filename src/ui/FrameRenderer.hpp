#pragma once

#include "../hw/VGAController.hpp"
#include "../memory/MemoryBus.hpp"
#include <array>
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
//
// Two ways to use it:
//  - render(): rasterise the whole screen from the current state now.
//  - as the VGA's raster sink (VGAController::setRasterSink): emulated
//    frames are captured scanline by scanline as the beam passes display
//    changes (palette per line, split screens, mid-frame start address or
//    panning changes) and published as completedFrame() when the frame's
//    display ends.  Frontends present the latest completed frame.
class FrameRenderer : public hw::VgaRasterSink {
public:
  FrameRenderer(memory::MemoryBus &memory, hw::VGAController &vga)
      : m_memory(memory), m_vga(vga) {}

  void setBIOS(const hw::BIOS *bios) { m_bios = bios; }

  // Rasterise the current screen.  Returns false when the video mode is
  // unknown (the previous image is kept in that case).
  bool render(bool drawCursor = true);

  // ── Captured frames (VgaRasterSink) ──────────────────────────────
  struct Frame {
    int width = 0;
    int height = 0;
    uint32_t modeKey = 0xFFFFFFFFu;
    uint64_t number = 0; // emulated frame number (VGAController::frameCounter)
    uint64_t serial = 0; // completed frames so far (0: none yet)
    std::vector<uint32_t> pixels;
  };
  void beginFrame(uint64_t frameNumber) override;
  void scanTo(uint32_t line) override;
  void endFrame() override;
  // Draw the text cursor (and blink) in captured frames
  void setCaptureCursor(bool on) { m_captureCursor = on; }
  const Frame &completedFrame() const { return m_completed; }
  bool captureOpen() const { return m_capOpen; }

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

  // Display geometry decoded from the CRTC, sequencer, graphics and
  // attribute controller registers (public for tests and diagnostics).
  struct CrtLayout {
    enum class Kind : uint8_t { Text, Planar16, Packed256, Cga } kind = Kind::Text;
    int width = 0;          // output pixels
    int height = 0;         // output lines
    int charClocks = 0;     // displayed character clocks (CRTC 01h + 1)
    int dotsPerChar = 8;    // text: 8 or 9 (sequencer 01h bit 0)
    int charHeight = 1;     // CRTC 09h maximum scan line + 1
    int scanDivider = 1;    // 2 with double scanning (CRTC 09h bit 7)
    int lineRepeat = 1;     // physical scanlines per output line
    int addrShift = 0;      // 0 byte, 1 word, 2 doubleword addressing
    int pan = 0;            // horizontal pel panning in output pixels
    int presetRowScan = 0;  // CRTC 08h bits 0-4
    bool countBy2 = false;  // CRTC 17h bit 3
    bool panInSplit = true; // attribute 10h bit 5 clear
    uint32_t start = 0;     // CRTC 0Ch/0Dh + byte panning
    uint32_t rowStride = 0; // CRTC 13h * 2 (address counter units)
    uint32_t lineCompare = 0x3FF;
    uint32_t textBase = 0xB8000; // text / CGA memory (GC 06h memory map)
  };
  // Returns false when the registers do not describe a displayable mode
  // (e.g. before the BIOS programmed them); the renderer then falls back
  // to the BIOS mode number.
  bool decodeLayout(CrtLayout &layout) const;

private:
  void resize(int w, int h);
  void buildPalette();
  void buildAttributeLut();
  uint32_t mapAddress(const CrtLayout &l, uint32_t counter, int rowScan) const;
  void renderRegisters(const CrtLayout &l, bool drawCursor);
  // Output lines [fromY, toY) of a register-described frame
  void renderRegisterLines(const CrtLayout &l, int fromY, int toY,
                           uint64_t frameNumber, bool drawCursor);
  bool vbeActive() const;
  bool renderFallback(bool drawCursor);
  void renderTextLine(const CrtLayout &l, uint32_t rowAddr, int rowScan,
                      bool cursorOn, bool blinkOn);
  void renderGraphicsLine(const CrtLayout &l, uint32_t rowAddr, int rowScan);
  // Fallbacks keyed on the BIOS mode number
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
  std::vector<uint8_t> m_line;          // one scanline of DAC indices
  std::array<uint32_t, 256> m_palette{}; // DAC index -> ARGB (PEL mask applied)
  std::array<uint8_t, 16> m_attrLut{};  // 4-bit pixel -> DAC index

  // Frame capture
  bool m_capOpen = false;
  bool m_captureCursor = true;
  uint64_t m_capNumber = 0;
  int m_capNextY = 0; // first output line not rasterised yet
  static constexpr uint32_t kNoSplit = 0xFFFFFFFEu;
  uint32_t m_capPhys = 0;           // scanlines compared with line compare
  uint32_t m_capSplitAt = kNoSplit; // last line compare match this frame
  Frame m_completed;
};

// Encode an ARGB image as a PNG file.
bool writePNG(const std::string &path, int width, int height,
              const std::vector<uint32_t> &argb);

} // namespace fador::ui
