#pragma once
#include "../memory/MemoryBus.hpp"

#include <cstdint>

namespace fador::hw {

class VGAController;

// A VBE linear-framebuffer mode the BIOS text functions draw into.
struct VbeTextSurface {
  uint32_t base = 0;  // physical address of the displayed page
  uint32_t pitch = 0; // bytes per scan line
  uint16_t width = 0;
  uint16_t height = 0;
  uint8_t bpp = 0;       // 8, 15, 16, 24 or 32
  uint32_t fontAddr = 0; // 8x16 glyph table (physical)
};

// Character output of the video BIOS (INT 10h AH=02h/03h, 06h-0Ah, 0Eh,
// 13h) in every mode, as the IBM BIOS does it:
//  - text modes keep char/attribute pairs at B8000h (B0000h in mode 7),
//    one page per BDA regen size (44Ch), cursor per page (450h);
//  - graphics modes draw glyphs from the graphics font (INT 43h, the
//    upper 128 CGA characters from INT 1Fh) with the colour in the
//    attribute: CGA 4/5 (2 bpp) and 6 (1 bpp) at B8000h, EGA/VGA planar
//    0Dh-12h (with pages), mode 13h and VBE modes.  Bit 7 of the colour
//    XORs the glyph into the screen (CGA and planar modes); otherwise the
//    cell's background is cleared to colour 0;
//  - AH=08h reads graphics characters back by matching the cell against
//    the font.
// The BDA (mode 449h, columns 44Ah, rows 484h, character height 485h,
// pages 44Ch/44Eh/462h) describes the current mode.  Used by the BIOS
// (INT 10h) and the DOS console.
class VideoText {
public:
  explicit VideoText(memory::MemoryBus &memory,
                     const VbeTextSurface *vbe = nullptr);

  enum class Kind : uint8_t { Text, Cga4, Cga2, Planar, Linear256, Vbe };
  Kind kind() const { return m_kind; }
  bool graphics() const { return m_kind != Kind::Text; }
  int cols() const { return m_cols; }
  int rows() const { return m_rows; }
  uint8_t activePage() const;

  // Cursor of a page (BDA 450h + 2 * page); setCursor also moves the
  // hardware cursor when the page is displayed.
  void cursor(uint8_t page, uint8_t &row, uint8_t &col) const;
  void setCursor(uint8_t page, uint8_t row, uint8_t col);

  // AH=09h (setAttr) / 0Ah: `count` copies at (row, col) on `page`.  In
  // graphics modes `attr` is the colour (both functions).
  void writeChars(uint8_t page, uint8_t row, uint8_t col, uint8_t ch,
                  uint8_t attr, bool setAttr, uint16_t count);
  // AH=08h: AL = character, AH = attribute (0 in graphics modes)
  uint16_t readChar(uint8_t page, uint8_t row, uint8_t col) const;
  // AH=06h/07h: scroll a window of `page` by `lines` (0 = clear it),
  // filling with `fill` (attribute in text modes, colour / byte otherwise)
  void scroll(bool up, uint8_t lines, uint8_t top, uint8_t left,
              uint8_t bottom, uint8_t right, uint8_t fill, uint8_t page);
  // AH=0Eh and the AH=13h character path: BEL, BS, LF and CR act on the
  // cursor; other characters are written at the cursor (with `attr` if
  // setAttr, else keeping the cell's attribute in text modes) and the
  // cursor advances, wrapping and scrolling the page.
  void teletype(uint8_t page, uint8_t ch, uint8_t attr, bool setAttr);

private:
  uint32_t textAddress(uint8_t page, int row, int col) const;
  uint32_t pageBase(uint8_t page) const;
  const uint8_t *glyph(uint8_t ch) const;
  void drawGlyph(uint8_t page, int row, int col, uint8_t ch, uint8_t colour);
  // Cell bit pattern (1 = pixel not background), charHeight bytes
  void readCell(uint8_t page, int row, int col, uint8_t *bits) const;
  void copyCellRow(uint8_t page, int dstRow, int srcRow, int left, int right);
  void fillCellRow(uint8_t page, int row, int left, int right, uint8_t fill);
  // VBE helpers
  uint32_t vbePixelAddress(int x, int y) const;
  void vbePut(int x, int y, uint8_t colour);
  bool vbeGet(int x, int y) const;

  memory::MemoryBus &m_memory;
  VGAController *m_vga;
  const VbeTextSurface *m_vbe;
  Kind m_kind = Kind::Text;
  uint8_t m_mode = 3;
  int m_cols = 80;
  int m_rows = 25;
  int m_charHeight = 8;
  uint32_t m_textBase = 0xB8000;
};

} // namespace fador::hw
