#include "VideoText.hpp"
#include "VGAController.hpp"
#include "VideoMode.hpp"

#include <algorithm>
#include <array>

namespace fador::hw {

namespace {

constexpr uint32_t kCgaBase = 0xB8000;

// CGA graphics: scanline y starts in the even (y & 1 = 0) or odd bank
uint32_t cgaLine(int y) {
  return kCgaBase + ((y & 1) ? 0x2000u : 0u) + static_cast<uint32_t>(y >> 1) * 80u;
}

} // namespace

VideoText::VideoText(memory::MemoryBus &memory, const VbeTextSurface *vbe)
    : m_memory(memory), m_vga(memory.getVGA()), m_vbe(vbe) {
  if (vbe && vbe->bpp != 0 && vbe->width >= 8 && vbe->height >= 16) {
    m_kind = Kind::Vbe;
    m_cols = vbe->width / 8;
    m_rows = vbe->height / 16;
    m_charHeight = 16;
    return;
  }
  m_mode = static_cast<uint8_t>(memory.read8(0x449) & 0x7F);
  const VideoModeInfo *mi = findVideoMode(m_mode);
  m_cols = memory.read16(0x44A);
  if (m_cols == 0 || m_cols > 255)
    m_cols = mi ? mi->textCols : 80;
  m_rows = memory.read8(0x484) + 1;
  if (m_rows < 2)
    m_rows = mi ? mi->textRows : 25;
  m_charHeight = std::clamp<int>(memory.read8(0x485), 1, 32);
  if (!mi || !mi->isGraphics) {
    m_kind = Kind::Text;
    m_textBase = m_mode == 0x07 ? 0xB0000 : 0xB8000;
    return;
  }
  switch (mi->layout) {
  case VMemLayout::CGA4: m_kind = Kind::Cga4; m_charHeight = 8; break;
  case VMemLayout::CGA2: m_kind = Kind::Cga2; m_charHeight = 8; break;
  case VMemLayout::Planar: m_kind = Kind::Planar; break;
  case VMemLayout::Linear256: m_kind = Kind::Linear256; break;
  default: m_kind = Kind::Text; break;
  }
  if ((m_kind == Kind::Planar || m_kind == Kind::Linear256) && !m_vga)
    m_kind = Kind::Text;
}

uint8_t VideoText::activePage() const { return m_memory.read8(0x462) & 7; }

void VideoText::cursor(uint8_t page, uint8_t &row, uint8_t &col) const {
  col = m_memory.read8(0x450 + (page & 7) * 2);
  row = m_memory.read8(0x451 + (page & 7) * 2);
}

void VideoText::setCursor(uint8_t page, uint8_t row, uint8_t col) {
  page &= 7;
  m_memory.write8(0x450 + page * 2, col);
  m_memory.write8(0x451 + page * 2, row);
  if (page == activePage())
    syncCursorFromBDA(m_memory);
}

uint32_t VideoText::pageBase(uint8_t page) const {
  if (m_kind != Kind::Text && m_kind != Kind::Planar)
    return 0; // CGA graphics, mode 13h and VBE have a single page
  return static_cast<uint32_t>(page & 7) * m_memory.read16(0x44C);
}

uint32_t VideoText::textAddress(uint8_t page, int row, int col) const {
  return m_textBase +
         ((pageBase(page) + static_cast<uint32_t>(row * m_cols + col) * 2u) & 0x7FFFu);
}

const uint8_t *VideoText::glyph(uint8_t ch) const {
  uint32_t addr;
  if (m_kind == Kind::Vbe) {
    addr = m_vbe->fontAddr + ch * 16u;
  } else {
    // CGA modes: characters 80h-FFh come from the INT 1Fh table
    const bool upper = (m_kind == Kind::Cga4 || m_kind == Kind::Cga2) && ch >= 0x80;
    const uint8_t vec = upper ? 0x1F : 0x43;
    addr = (static_cast<uint32_t>(m_memory.read16(vec * 4 + 2)) << 4) +
           m_memory.read16(vec * 4) +
           static_cast<uint32_t>(upper ? ch - 0x80 : ch) * static_cast<uint32_t>(m_charHeight);
  }
  return m_memory.contiguousAccess(addr, static_cast<uint32_t>(m_charHeight));
}

uint32_t VideoText::vbePixelAddress(int x, int y) const {
  const uint32_t bytes = (m_vbe->bpp + 7u) / 8u;
  return m_vbe->base + static_cast<uint32_t>(y) * m_vbe->pitch +
         static_cast<uint32_t>(x) * bytes;
}

void VideoText::vbePut(int x, int y, uint8_t colour) {
  const uint32_t a = vbePixelAddress(x, y);
  if (m_vbe->bpp == 8) {
    m_memory.write8(a, colour);
    return;
  }
  // Direct-colour modes: the colour indexes the DAC
  const uint8_t *dac = m_memory.dacPalette() + colour * 3u;
  const uint32_t r = (dac[0] << 2) | (dac[0] >> 4);
  const uint32_t g = (dac[1] << 2) | (dac[1] >> 4);
  const uint32_t b = (dac[2] << 2) | (dac[2] >> 4);
  switch (m_vbe->bpp) {
  case 15:
    m_memory.write16(a, static_cast<uint16_t>(((r >> 3) << 10) | ((g >> 3) << 5) | (b >> 3)));
    break;
  case 16:
    m_memory.write16(a, static_cast<uint16_t>(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)));
    break;
  case 24:
    m_memory.write8(a, static_cast<uint8_t>(b));
    m_memory.write8(a + 1, static_cast<uint8_t>(g));
    m_memory.write8(a + 2, static_cast<uint8_t>(r));
    break;
  default:
    m_memory.write32(a, (r << 16) | (g << 8) | b);
    break;
  }
}

bool VideoText::vbeGet(int x, int y) const {
  const uint32_t a = vbePixelAddress(x, y);
  switch (m_vbe->bpp) {
  case 8: return m_memory.read8(a) != 0;
  case 15:
  case 16: return m_memory.read16(a) != 0;
  case 24: return (m_memory.read16(a) | m_memory.read8(a + 2)) != 0;
  default: return (m_memory.read32(a) & 0xFFFFFFu) != 0;
  }
}

void VideoText::drawGlyph(uint8_t page, int row, int col, uint8_t ch,
                          uint8_t colour) {
  std::array<uint8_t, 32> g{};
  if (const uint8_t *src = glyph(ch))
    std::copy_n(src, m_charHeight, g.begin());
  const bool xorMode = (colour & 0x80) != 0;
  const int h = m_charHeight;
  switch (m_kind) {
  case Kind::Cga4: {
    const uint32_t c = colour & 3u;
    for (int r = 0; r < 8; ++r) {
      uint32_t pix = 0;
      for (int b = 0; b < 8; ++b)
        if (g[r] & (0x80 >> b))
          pix |= c << (14 - 2 * b);
      const uint32_t a = cgaLine(row * 8 + r) + static_cast<uint32_t>(col) * 2u;
      uint8_t hi = static_cast<uint8_t>(pix >> 8), lo = static_cast<uint8_t>(pix);
      if (xorMode) {
        hi ^= m_memory.read8(a);
        lo ^= m_memory.read8(a + 1);
      }
      m_memory.write8(a, hi);
      m_memory.write8(a + 1, lo);
    }
    break;
  }
  case Kind::Cga2:
    for (int r = 0; r < 8; ++r) {
      const uint32_t a = cgaLine(row * 8 + r) + static_cast<uint32_t>(col);
      uint8_t v = (colour & 1) ? g[r] : 0;
      if (xorMode)
        v ^= m_memory.read8(a);
      m_memory.write8(a, v);
    }
    break;
  case Kind::Planar: {
    const uint32_t base = pageBase(page);
    for (int r = 0; r < h; ++r) {
      const uint32_t off = base + static_cast<uint32_t>((row * h + r) * m_cols + col);
      for (int p = 0; p < 4; ++p) {
        uint8_t v = (colour & (1u << p)) ? g[r] : 0;
        if (xorMode)
          v ^= m_vga->readPlane(p, off);
        m_vga->writePlane(p, off, v);
      }
    }
    break;
  }
  case Kind::Linear256: {
    // Chain-4 layout: byte address a is in plane a & 3 at offset a & ~3
    const uint32_t width = static_cast<uint32_t>(m_cols) * 8u;
    for (int r = 0; r < h; ++r)
      for (int x = 0; x < 8; ++x) {
        const uint32_t a = static_cast<uint32_t>(row * h + r) * width +
                           static_cast<uint32_t>(col * 8 + x);
        m_vga->writePlane(static_cast<int>(a & 3), a & ~3u,
                          (g[r] & (0x80 >> x)) ? colour : 0);
      }
    break;
  }
  case Kind::Vbe:
    for (int r = 0; r < h; ++r)
      for (int x = 0; x < 8; ++x)
        vbePut(col * 8 + x, row * h + r, (g[r] & (0x80 >> x)) ? colour : 0);
    break;
  case Kind::Text:
    break;
  }
}

void VideoText::readCell(uint8_t page, int row, int col, uint8_t *bits) const {
  const int h = m_charHeight;
  for (int r = 0; r < h; ++r) {
    uint8_t v = 0;
    switch (m_kind) {
    case Kind::Cga4: {
      const uint32_t a = cgaLine(row * 8 + r) + static_cast<uint32_t>(col) * 2u;
      const uint32_t w = (uint32_t(m_memory.read8(a)) << 8) | m_memory.read8(a + 1);
      for (int b = 0; b < 8; ++b)
        if ((w >> (14 - 2 * b)) & 3u)
          v |= static_cast<uint8_t>(0x80 >> b);
      break;
    }
    case Kind::Cga2:
      v = m_memory.read8(cgaLine(row * 8 + r) + static_cast<uint32_t>(col));
      break;
    case Kind::Planar: {
      const uint32_t off = pageBase(page) + static_cast<uint32_t>((row * h + r) * m_cols + col);
      for (int p = 0; p < 4; ++p)
        v |= m_vga->readPlane(p, off);
      break;
    }
    case Kind::Linear256: {
      const uint32_t width = static_cast<uint32_t>(m_cols) * 8u;
      for (int x = 0; x < 8; ++x) {
        const uint32_t a = static_cast<uint32_t>(row * h + r) * width +
                           static_cast<uint32_t>(col * 8 + x);
        if (m_vga->readPlane(static_cast<int>(a & 3), a & ~3u))
          v |= static_cast<uint8_t>(0x80 >> x);
      }
      break;
    }
    case Kind::Vbe:
      for (int x = 0; x < 8; ++x)
        if (vbeGet(col * 8 + x, row * h + r))
          v |= static_cast<uint8_t>(0x80 >> x);
      break;
    case Kind::Text:
      break;
    }
    bits[r] = v;
  }
}

void VideoText::writeChars(uint8_t page, uint8_t row, uint8_t col, uint8_t ch,
                           uint8_t attr, bool setAttr, uint16_t count) {
  if (m_kind == Kind::Text) {
    // Consecutive cells in memory (continuing on the following rows),
    // up to the end of the 32 KiB text window
    uint32_t off = (pageBase(page) + static_cast<uint32_t>(row * m_cols + col) * 2u);
    for (uint32_t i = 0; i < count && off < 0x8000u; ++i, off += 2) {
      m_memory.write8(m_textBase + off, ch);
      if (setAttr)
        m_memory.write8(m_textBase + off + 1, attr);
    }
    return;
  }
  int r = row, c = col;
  for (uint32_t i = 0; i < count && r < m_rows; ++i) {
    drawGlyph(page, r, c, ch, attr);
    if (++c >= m_cols) {
      c = 0;
      ++r;
    }
  }
}

uint16_t VideoText::readChar(uint8_t page, uint8_t row, uint8_t col) const {
  if (m_kind == Kind::Text) {
    const uint32_t a = textAddress(page, row, col);
    return static_cast<uint16_t>(m_memory.read8(a) | (m_memory.read8(a + 1) << 8));
  }
  if (row >= m_rows || col >= m_cols)
    return 0;
  std::array<uint8_t, 32> bits{};
  readCell(page, row, col, bits.data());
  // Match the cell against the font, as the IBM BIOS does
  for (int ch = 0; ch < 256; ++ch) {
    const uint8_t *g = glyph(static_cast<uint8_t>(ch));
    if (g && std::equal(bits.begin(), bits.begin() + m_charHeight, g))
      return static_cast<uint16_t>(ch);
  }
  return 0;
}

void VideoText::copyCellRow(uint8_t page, int dst, int src, int left, int right) {
  const int h = m_charHeight;
  const int n = right - left + 1;
  switch (m_kind) {
  case Kind::Text:
    for (int c = left; c <= right; ++c)
      m_memory.write16(textAddress(page, dst, c),
                       m_memory.read16(textAddress(page, src, c)));
    break;
  case Kind::Cga4:
  case Kind::Cga2: {
    const int bpc = m_kind == Kind::Cga4 ? 2 : 1;
    for (int r = 0; r < 8; ++r) {
      const uint32_t d = cgaLine(dst * 8 + r) + static_cast<uint32_t>(left * bpc);
      const uint32_t s = cgaLine(src * 8 + r) + static_cast<uint32_t>(left * bpc);
      for (int i = 0; i < n * bpc; ++i)
        m_memory.write8(d + i, m_memory.read8(s + i));
    }
    break;
  }
  case Kind::Planar: {
    const uint32_t base = pageBase(page);
    for (int r = 0; r < h; ++r) {
      const uint32_t d = base + static_cast<uint32_t>((dst * h + r) * m_cols + left);
      const uint32_t s = base + static_cast<uint32_t>((src * h + r) * m_cols + left);
      for (int p = 0; p < 4; ++p)
        for (int i = 0; i < n; ++i)
          m_vga->writePlane(p, d + i, m_vga->readPlane(p, s + i));
    }
    break;
  }
  case Kind::Linear256: {
    const uint32_t width = static_cast<uint32_t>(m_cols) * 8u;
    for (int r = 0; r < h; ++r)
      for (int x = left * 8; x < (right + 1) * 8; ++x) {
        const uint32_t d = static_cast<uint32_t>(dst * h + r) * width + x;
        const uint32_t s = static_cast<uint32_t>(src * h + r) * width + x;
        m_vga->writePlane(static_cast<int>(d & 3), d & ~3u,
                          m_vga->readPlane(static_cast<int>(s & 3), s & ~3u));
      }
    break;
  }
  case Kind::Vbe: {
    const uint32_t bytes = static_cast<uint32_t>(n) * 8u * ((m_vbe->bpp + 7u) / 8u);
    for (int r = 0; r < h; ++r) {
      const uint32_t d = vbePixelAddress(left * 8, dst * h + r);
      const uint32_t s = vbePixelAddress(left * 8, src * h + r);
      uint8_t *dp = m_memory.contiguousAccess(d, bytes);
      const uint8_t *sp = m_memory.contiguousAccess(s, bytes);
      if (dp && sp)
        std::copy_n(sp, bytes, dp);
    }
    break;
  }
  }
}

void VideoText::fillCellRow(uint8_t page, int row, int left, int right,
                            uint8_t fill) {
  const int h = m_charHeight;
  const int n = right - left + 1;
  switch (m_kind) {
  case Kind::Text:
    for (int c = left; c <= right; ++c)
      m_memory.write16(textAddress(page, row, c),
                       static_cast<uint16_t>(0x20 | (fill << 8)));
    break;
  case Kind::Cga4:
  case Kind::Cga2: {
    // The fill byte is used as is (as the IBM BIOS does)
    const int bpc = m_kind == Kind::Cga4 ? 2 : 1;
    for (int r = 0; r < 8; ++r) {
      const uint32_t d = cgaLine(row * 8 + r) + static_cast<uint32_t>(left * bpc);
      for (int i = 0; i < n * bpc; ++i)
        m_memory.write8(d + i, fill);
    }
    break;
  }
  case Kind::Planar: {
    const uint32_t base = pageBase(page);
    for (int r = 0; r < h; ++r) {
      const uint32_t d = base + static_cast<uint32_t>((row * h + r) * m_cols + left);
      for (int p = 0; p < 4; ++p) {
        const uint8_t v = (fill & (1u << p)) ? 0xFF : 0x00;
        for (int i = 0; i < n; ++i)
          m_vga->writePlane(p, d + i, v);
      }
    }
    break;
  }
  case Kind::Linear256: {
    const uint32_t width = static_cast<uint32_t>(m_cols) * 8u;
    for (int r = 0; r < h; ++r)
      for (int x = left * 8; x < (right + 1) * 8; ++x) {
        const uint32_t d = static_cast<uint32_t>(row * h + r) * width + x;
        m_vga->writePlane(static_cast<int>(d & 3), d & ~3u, fill);
      }
    break;
  }
  case Kind::Vbe:
    for (int r = 0; r < h; ++r)
      for (int x = left * 8; x < (right + 1) * 8; ++x)
        vbePut(x, row * h + r, fill);
    break;
  }
}

void VideoText::scroll(bool up, uint8_t lines, uint8_t top, uint8_t left,
                       uint8_t bottom, uint8_t right, uint8_t fill,
                       uint8_t page) {
  const int b = std::min<int>(bottom, m_rows - 1);
  const int r = std::min<int>(right, m_cols - 1);
  if (top > b || left > r)
    return;
  const int height = b - top + 1;
  const int n = (lines == 0 || lines >= height) ? height : lines;
  if (up) {
    for (int row = top; row + n <= b; ++row)
      copyCellRow(page, row, row + n, left, r);
    for (int row = b - n + 1; row <= b; ++row)
      fillCellRow(page, row, left, r, fill);
  } else {
    for (int row = b; row - n >= top; --row)
      copyCellRow(page, row, row - n, left, r);
    for (int row = top; row < top + n; ++row)
      fillCellRow(page, row, left, r, fill);
  }
}

void VideoText::teletype(uint8_t page, uint8_t ch, uint8_t attr, bool setAttr) {
  uint8_t row, col;
  cursor(page, row, col);
  switch (ch) {
  case 0x07: // BEL
    return;
  case 0x08: // backspace
    if (col > 0)
      --col;
    break;
  case 0x0A: // line feed
    ++row;
    break;
  case 0x0D: // carriage return
    col = 0;
    break;
  default:
    if (m_kind == Kind::Text) {
      const uint32_t a = textAddress(page, row, col);
      m_memory.write8(a, ch);
      if (setAttr)
        m_memory.write8(a + 1, attr);
      else if (m_memory.read8(a + 1) == 0)
        m_memory.write8(a + 1, 0x07); // never print invisible text
    } else if (row < m_rows && col < m_cols) {
      drawGlyph(page, row, col, ch, attr);
    }
    if (++col >= m_cols) {
      col = 0;
      ++row;
    }
    break;
  }
  if (row >= m_rows) {
    // Scroll the page; text modes fill with the attribute at the cursor
    const int last = m_rows - 1;
    const uint8_t fill = m_kind == Kind::Text
                             ? m_memory.read8(textAddress(page, last, std::min<int>(col, m_cols - 1)) + 1)
                             : 0;
    scroll(true, 1, 0, 0, static_cast<uint8_t>(last),
           static_cast<uint8_t>(m_cols - 1), fill, page);
    row = static_cast<uint8_t>(last);
  }
  setCursor(page, row, col);
}

} // namespace fador::hw
