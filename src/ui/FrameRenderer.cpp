#include "FrameRenderer.hpp"
#include "../hw/BIOS.hpp"
#include "../hw/VideoMode.hpp"
#include "VGAFont.hpp"
#include "../utils/simd/Simd.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>

namespace fador::ui {

namespace {

// ── Minimal PNG encoder (zlib "stored" deflate blocks) ────────────────
uint32_t crc32Update(uint32_t crc, const uint8_t *data, size_t len) {
  static const auto table = [] {
    std::array<uint32_t, 256> t{};
    for (uint32_t n = 0; n < 256; ++n) {
      uint32_t c = n;
      for (int k = 0; k < 8; ++k)
        c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : (c >> 1);
      t[n] = c;
    }
    return t;
  }();
  crc = ~crc;
  for (size_t i = 0; i < len; ++i)
    crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
  return ~crc;
}

void put32be(std::vector<uint8_t> &out, uint32_t v) {
  out.push_back(static_cast<uint8_t>(v >> 24));
  out.push_back(static_cast<uint8_t>(v >> 16));
  out.push_back(static_cast<uint8_t>(v >> 8));
  out.push_back(static_cast<uint8_t>(v));
}

void writeChunk(std::FILE *f, const char type[4],
                const std::vector<uint8_t> &data) {
  std::vector<uint8_t> buf;
  put32be(buf, static_cast<uint32_t>(data.size()));
  size_t typePos = buf.size();
  buf.insert(buf.end(), type, type + 4);
  buf.insert(buf.end(), data.begin(), data.end());
  uint32_t crc = crc32Update(0, buf.data() + typePos, buf.size() - typePos);
  put32be(buf, crc);
  std::fwrite(buf.data(), 1, buf.size(), f);
}

// Text-mode / CGA attribute colours are looked up through the DAC so
// palette effects work; CGA modes use the BIOS default palettes.
constexpr uint8_t kCga4Palette[4] = {0, 11, 13, 15};

// Final step of every palettised path: one scanline of 8-bit DAC indices
// to ARGB through the 256-entry palette (PEL mask already applied).
void convertIndexedScanline(const uint8_t *src, uint32_t *dst, int count,
                            const uint32_t *palette) {
  utils::simd::kernels().paletteToArgb(dst, src, count, palette);
}

constexpr uint32_t kBlack = 0xFF000000u;

} // namespace

bool writePNG(const std::string &path, int width, int height,
              const std::vector<uint32_t> &argb) {
  if (width <= 0 || height <= 0 ||
      argb.size() < static_cast<size_t>(width) * height)
    return false;
  std::unique_ptr<std::FILE, int (*)(std::FILE *)> f(
      std::fopen(path.c_str(), "wb"), &std::fclose);
  if (!f)
    return false;

  static const uint8_t sig[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
  std::fwrite(sig, 1, sizeof(sig), f.get());

  std::vector<uint8_t> ihdr;
  put32be(ihdr, static_cast<uint32_t>(width));
  put32be(ihdr, static_cast<uint32_t>(height));
  ihdr.insert(ihdr.end(), {8, 2, 0, 0, 0}); // 8-bit RGB
  writeChunk(f.get(), "IHDR", ihdr);

  // Raw scanlines: filter byte 0 + RGB triplets
  std::vector<uint8_t> raw;
  raw.reserve(static_cast<size_t>(height) * (1 + width * 3));
  for (int y = 0; y < height; ++y) {
    raw.push_back(0);
    for (int x = 0; x < width; ++x) {
      uint32_t p = argb[static_cast<size_t>(y) * width + x];
      raw.push_back(static_cast<uint8_t>(p >> 16));
      raw.push_back(static_cast<uint8_t>(p >> 8));
      raw.push_back(static_cast<uint8_t>(p));
    }
  }

  std::vector<uint8_t> z;
  z.push_back(0x78);
  z.push_back(0x01);
  uint32_t a = 1, b = 0;
  for (uint8_t v : raw) {
    a = (a + v) % 65521;
    b = (b + a) % 65521;
  }
  size_t pos = 0;
  do {
    size_t len = std::min<size_t>(65535, raw.size() - pos);
    bool last = pos + len == raw.size();
    z.push_back(last ? 1 : 0);
    z.push_back(static_cast<uint8_t>(len));
    z.push_back(static_cast<uint8_t>(len >> 8));
    z.push_back(static_cast<uint8_t>(~len));
    z.push_back(static_cast<uint8_t>((~len) >> 8));
    z.insert(z.end(), raw.begin() + static_cast<std::ptrdiff_t>(pos),
             raw.begin() + static_cast<std::ptrdiff_t>(pos + len));
    pos += len;
  } while (pos < raw.size());
  put32be(z, (b << 16) | a);
  writeChunk(f.get(), "IDAT", z);
  writeChunk(f.get(), "IEND", {});
  return true;
}

uint32_t FrameRenderer::paletteToARGB(uint8_t index) const {
  const uint8_t *pal = m_memory.dacPalette();
  uint8_t r6 = pal[index * 3 + 0];
  uint8_t g6 = pal[index * 3 + 1];
  uint8_t b6 = pal[index * 3 + 2];
  uint8_t r = static_cast<uint8_t>((r6 << 2) | (r6 >> 4));
  uint8_t g = static_cast<uint8_t>((g6 << 2) | (g6 >> 4));
  uint8_t b = static_cast<uint8_t>((b6 << 2) | (b6 >> 4));
  return 0xFF000000u | (uint32_t(r) << 16) | (uint32_t(g) << 8) | b;
}

void FrameRenderer::resize(int w, int h) {
  if (w != m_width || h != m_height ||
      m_pixels.size() != static_cast<size_t>(w) * h) {
    m_width = w;
    m_height = h;
    m_pixels.assign(static_cast<size_t>(w) * h, 0xFF000000u);
  }
}

bool FrameRenderer::render(bool drawCursor) {
  if (m_bios && m_bios->vbeState().mode != 0) {
    const auto &v = m_bios->vbeState();
    m_modeKey = 0x10000u | v.mode;
    return renderVbe();
  }
  const uint8_t mode = m_memory.read8(0x449);
  CrtLayout layout;
  if (decodeLayout(layout)) {
    m_modeKey = mode;
    renderRegisters(layout, drawCursor);
    return true;
  }
  const auto *mi = hw::findVideoMode(mode);
  if (!mi)
    return false;
  m_modeKey = mode;
  if (mi->isGraphics)
    return renderGraphics(mode);
  renderText(drawCursor);
  return true;
}

// ── Register-driven rendering ─────────────────────────────────────────

void FrameRenderer::buildPalette() {
  const uint8_t mask = m_vga.pelMask();
  for (int i = 0; i < 256; ++i)
    m_palette[i] = paletteToARGB(static_cast<uint8_t>(i & mask));
}

void FrameRenderer::buildAttributeLut() {
  const uint8_t mc = m_vga.attrReg(0x10);
  const uint8_t planeEnable = m_vga.attrReg(0x12) & 0x0F;
  const uint8_t colorSelect = m_vga.attrReg(0x14);
  for (int v = 0; v < 16; ++v) {
    uint8_t p = m_vga.attrReg(static_cast<uint8_t>(v & planeEnable)) & 0x3F;
    if (mc & 0x80) // P5/P4 from the colour select register
      p = static_cast<uint8_t>((p & 0x0F) | ((colorSelect & 0x03) << 4));
    m_attrLut[v] = static_cast<uint8_t>(p | ((colorSelect & 0x0C) << 4));
  }
}

bool FrameRenderer::decodeLayout(CrtLayout &l) const {
  const hw::VGAController &v = m_vga;
  if (v.crtcReg(0x01) == 0 || v.crtcReg(0x13) == 0)
    return false;
  const int scanLines = static_cast<int>(v.verticalDisplayEnd()) + 1;
  if (scanLines < 2)
    return false;

  l.charClocks = v.crtcReg(0x01) + 1;
  l.charHeight = v.maxScanLine() + 1;
  l.scanDivider = v.doubleScan() ? 2 : 1;
  const uint8_t mode17 = v.crtcReg(0x17);
  l.addrShift = (v.crtcReg(0x14) & 0x40) ? 2 : (mode17 & 0x40) ? 0 : 1;
  l.countBy2 = (mode17 & 0x08) != 0;
  l.start = v.getDisplayStart() + ((v.crtcReg(0x08) >> 5) & 3u);
  l.rowStride = v.getCRTCOffset() * 2u;
  l.lineCompare = v.lineCompare();
  l.presetRowScan = v.crtcReg(0x08) & 0x1F;
  const uint8_t mc = v.attrReg(0x10);
  l.panInSplit = (mc & 0x20) == 0;
  const uint8_t map = (v.gcReg(0x06) >> 2) & 3;
  l.textBase = map == 2 ? 0xB0000u : 0xB8000u;
  const uint8_t pel = v.attrReg(0x13) & 0x0F;

  if (!(mc & 0x01)) {
    l.kind = CrtLayout::Kind::Text;
    l.dotsPerChar = (v.seqReg(1) & 0x01) ? 8 : 9;
    l.width = l.charClocks * l.dotsPerChar;
    l.lineRepeat = l.scanDivider;
    if (l.dotsPerChar == 9)
      l.pan = pel >= 8 ? 0 : pel + 1;
    else
      l.pan = pel & 7;
  } else {
    const bool rowScanAddressing = (mode17 & 0x03) != 0x03;
    // CGA/Hercules compatible addressing (CRTC 17h bits 0/1 clear) maps
    // each row-scan line to its own memory bank, so rows differ.
    l.lineRepeat = l.scanDivider * (rowScanAddressing ? 1 : l.charHeight);
    if (mc & 0x40) {
      l.kind = CrtLayout::Kind::Packed256;
      l.width = l.charClocks * 4;
      l.pan = (pel & 7) >> 1;
    } else if (map >= 2) {
      l.kind = CrtLayout::Kind::Cga;
      l.width = l.charClocks * 8;
      l.pan = 0;
    } else {
      l.kind = CrtLayout::Kind::Planar16;
      l.width = l.charClocks * 8;
      l.pan = pel & 7;
    }
  }
  l.height = scanLines / l.lineRepeat;
  return l.width >= 8 && l.width <= 2048 && l.height >= 1 && l.height <= 2048;
}

uint32_t FrameRenderer::mapAddress(const CrtLayout &l, uint32_t c,
                                   int rowScan) const {
  uint32_t a;
  switch (l.addrShift) {
  case 2: a = (c << 2) | ((c >> 14) & 3u); break;
  case 1: {
    const unsigned wrapBit = (m_vga.crtcReg(0x17) & 0x20) ? 15 : 13;
    a = (c << 1) | ((c >> wrapBit) & 1u);
    break;
  }
  default: a = c; break;
  }
  const uint8_t mode17 = m_vga.crtcReg(0x17);
  if (!(mode17 & 0x01))
    a = (a & ~0x2000u) | ((static_cast<uint32_t>(rowScan) & 1u) << 13);
  if (!(mode17 & 0x02))
    a = (a & ~0x4000u) | ((static_cast<uint32_t>(rowScan) & 2u) << 13);
  return a & 0xFFFFu;
}

void FrameRenderer::renderRegisters(const CrtLayout &l, bool drawCursor) {
  resize(l.width, l.height);
  // Sequencer screen-off blanks to black; a cleared palette address
  // source bit shows the overscan colour.
  if (m_vga.screenOff() || !m_vga.attrPaletteSourceEnabled()) {
    uint32_t fill = kBlack;
    if (!m_vga.screenOff()) {
      buildPalette();
      fill = m_palette[m_vga.attrReg(0x11)];
    }
    std::fill(m_pixels.begin(), m_pixels.end(), fill);
    return;
  }
  buildPalette();
  buildAttributeLut();
  m_line.assign(static_cast<size_t>(l.charClocks + 2) * 9 + 16, 0);

  const uint64_t frame = drawCursor ? m_vga.frameCounter() : 0;
  const bool cursorPhase = ((frame >> 3) & 1) == 0; // 16-frame cursor blink
  const bool blinkPhase = ((frame >> 4) & 1) == 0;  // 32-frame text blink
  const bool text = l.kind == CrtLayout::Kind::Text;

  for (int y = 0; y < l.height; ++y) {
    const uint32_t phys = static_cast<uint32_t>(y * l.lineRepeat);
    // Line compare: the lines after it restart at address 0
    const bool split = phys > l.lineCompare;
    const uint32_t local = split ? phys - (l.lineCompare + 1) : phys;
    uint32_t scanRow = local / static_cast<uint32_t>(l.scanDivider);
    if (text && !split)
      scanRow += static_cast<uint32_t>(l.presetRowScan);
    const uint32_t charRow = scanRow / static_cast<uint32_t>(l.charHeight);
    const int rowScan = static_cast<int>(scanRow % static_cast<uint32_t>(l.charHeight));
    const uint32_t rowAddr = (split ? 0u : l.start) + charRow * l.rowStride;
    const int pan = (split && !l.panInSplit) ? 0 : l.pan;

    if (text)
      renderTextLine(l, rowAddr, rowScan, drawCursor && cursorPhase, blinkPhase);
    else
      renderGraphicsLine(l, rowAddr, rowScan);
    convertIndexedScanline(m_line.data() + pan,
                           &m_pixels[static_cast<size_t>(y) * l.width], l.width,
                           m_palette.data());
  }
}

void FrameRenderer::renderTextLine(const CrtLayout &l, uint32_t rowAddr,
                                   int rowScan, bool cursorOn, bool blinkOn) {
  const uint8_t *ram = m_memory.directAccess(l.textBase);
  const uint8_t *font = m_vga.planeData(2);
  const uint8_t mc = m_vga.attrReg(0x10);
  const bool blinkEnabled = (mc & 0x08) != 0;
  const bool lineGraphics = (mc & 0x04) != 0;
  const uint8_t charMap = m_vga.seqReg(3);
  // Character map select: map A (attribute bit 3 set) and map B offsets
  auto mapOffset = [](unsigned sel) { return (sel & 3u) * 0x4000u + ((sel >> 2) & 1u) * 0x2000u; };
  const uint32_t fontA = mapOffset(((charMap >> 2) & 3u) | ((charMap >> 3) & 4u));
  const uint32_t fontB = mapOffset((charMap & 3u) | ((charMap >> 2) & 4u));
  const uint8_t curStartReg = m_vga.crtcReg(0x0A);
  const int curStart = curStartReg & 0x1F;
  const int curEnd = m_vga.crtcReg(0x0B) & 0x1F;
  const bool cursorRow = cursorOn && !(curStartReg & 0x20) && rowScan >= curStart &&
                         rowScan <= curEnd;
  const uint32_t cursorLoc = m_vga.getCursorLocation();
  const int underline = m_vga.crtcReg(0x14) & 0x1F;
  const int dots = l.dotsPerChar;

  uint8_t *out = m_line.data();
  for (int col = 0; col < l.charClocks + 1; ++col) {
    const uint32_t c = rowAddr + static_cast<uint32_t>(l.countBy2 ? col / 2 : col);
    const uint32_t a = mapAddress(l, c, rowScan) & 0x7FFEu;
    const uint8_t ch = ram ? ram[a] : 0;
    const uint8_t attr = ram ? ram[a + 1] : 0;
    uint8_t fg = attr & 0x0F;
    uint8_t bg = (attr >> 4) & 0x0F;
    if (blinkEnabled) {
      bg &= 0x07;
      if ((attr & 0x80) && !blinkOn)
        fg = bg;
    }
    uint8_t bits = rowScan < 32
                       ? font[((attr & 0x08) ? fontA : fontB) + ch * 32u +
                              static_cast<uint32_t>(rowScan)]
                       : 0;
    if (rowScan == underline && (attr & 0x77) == 0x01)
      bits = 0xFF;
    if (cursorRow && (c & 0xFFFF) == cursorLoc)
      bits = 0xFF;
    const uint8_t fgIdx = m_attrLut[fg];
    const uint8_t bgIdx = m_attrLut[bg];
    for (int x = 0; x < 8; ++x)
      out[x] = (bits & (0x80 >> x)) ? fgIdx : bgIdx;
    if (dots == 9)
      out[8] = (lineGraphics && ch >= 0xC0 && ch <= 0xDF && (bits & 1)) ? fgIdx : bgIdx;
    out += dots;
  }
}

void FrameRenderer::renderGraphicsLine(const CrtLayout &l, uint32_t rowAddr,
                                       int rowScan) {
  uint8_t *out = m_line.data();
  const int clocks = l.charClocks + 1; // one extra for pel panning
  switch (l.kind) {
  case CrtLayout::Kind::Packed256:
    for (int k = 0; k < clocks; ++k) {
      const uint32_t c = rowAddr + static_cast<uint32_t>(l.countBy2 ? k / 2 : k);
      const uint32_t a = mapAddress(l, c, rowScan);
      for (int p = 0; p < 4; ++p)
        *out++ = m_vga.readPlane(p, a);
    }
    break;
  case CrtLayout::Kind::Planar16:
    for (int k = 0; k < clocks; ++k) {
      const uint32_t c = rowAddr + static_cast<uint32_t>(l.countBy2 ? k / 2 : k);
      const uint32_t a = mapAddress(l, c, rowScan);
      const uint8_t p0 = m_vga.readPlane(0, a), p1 = m_vga.readPlane(1, a);
      const uint8_t p2 = m_vga.readPlane(2, a), p3 = m_vga.readPlane(3, a);
      for (int b = 7; b >= 0; --b)
        *out++ = m_attrLut[((p0 >> b) & 1) | (((p1 >> b) & 1) << 1) |
                           (((p2 >> b) & 1) << 2) | (((p3 >> b) & 1) << 3)];
    }
    break;
  case CrtLayout::Kind::Cga: {
    // CGA-compatible modes keep their bytes in conventional RAM at B8000h
    const uint8_t *ram = m_memory.directAccess(l.textBase);
    const bool twoBpp = (m_vga.gcReg(0x05) & 0x20) != 0;
    const int bytesPerClock = l.addrShift == 1 ? 2 : 1;
    for (int k = 0; k < clocks; ++k) {
      const uint32_t c = rowAddr + static_cast<uint32_t>(l.countBy2 ? k / 2 : k);
      const uint32_t a = mapAddress(l, c, rowScan);
      for (int i = 0; i < bytesPerClock; ++i) {
        const uint8_t byte = ram ? ram[(a + static_cast<uint32_t>(i)) & 0x7FFFu] : 0;
        if (twoBpp) {
          for (int s = 6; s >= 0; s -= 2)
            *out++ = m_attrLut[(byte >> s) & 3];
        } else {
          for (int b = 7; b >= 0; --b)
            *out++ = m_attrLut[(byte >> b) & 1];
        }
      }
      // Byte-mode 4-colour data would produce only 4 pixels per clock;
      // pad so the line always holds 8 pixels per character clock.
      if (twoBpp && bytesPerClock == 1)
        for (int i = 0; i < 4; ++i, ++out)
          *out = out[-4];
    }
    break;
  }
  case CrtLayout::Kind::Text:
    break;
  }
}

void FrameRenderer::renderText(bool drawCursor) {
  uint16_t cols = m_memory.read16(0x44A);
  int rows = m_memory.read8(0x484) + 1;
  if (cols == 0 || cols > 132)
    cols = 80;
  if (rows <= 1 || rows > 60)
    rows = 25;
  resize(cols * 8, rows * 16);

  const uint32_t vramBase = (m_memory.read8(0x449) == 0x07) ? 0xB0000 : 0xB8000;
  const uint32_t pageOffset = m_memory.read16(0x44E);
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      uint32_t cell = vramBase + pageOffset + (row * cols + col) * 2u;
      uint8_t ch = m_memory.read8(cell);
      uint8_t attr = m_memory.read8(cell + 1);
      uint32_t fg = paletteToARGB(attr & 0x0F);
      uint32_t bg = paletteToARGB((attr >> 4) & 0x07);
      const uint8_t *glyph = kVGAFont8x16[ch];
      for (int gy = 0; gy < 16; ++gy) {
        uint8_t bits = glyph[gy];
        uint32_t *dst = &m_pixels[static_cast<size_t>(row * 16 + gy) * m_width +
                                  col * 8];
        for (int gx = 0; gx < 8; ++gx)
          dst[gx] = (bits & (0x80 >> gx)) ? fg : bg;
      }
    }
  }

  if (drawCursor) {
    uint8_t page = m_memory.read8(0x462) & 7;
    uint8_t curCol = m_memory.read8(0x450 + page * 2);
    uint8_t curRow = m_memory.read8(0x451 + page * 2);
    uint8_t cursorStart = m_memory.read8(0x461);
    bool hidden = (cursorStart & 0x20) != 0;
    if (!hidden && curRow < rows && curCol < cols) {
      uint8_t attr = m_memory.read8(vramBase + pageOffset +
                                    (curRow * cols + curCol) * 2u + 1);
      uint32_t color = paletteToARGB(attr & 0x0F);
      for (int gy = 14; gy < 16; ++gy) {
        uint32_t *dst = &m_pixels[static_cast<size_t>(curRow * 16 + gy) * m_width +
                                  curCol * 8];
        for (int gx = 0; gx < 8; ++gx)
          dst[gx] = color;
      }
    }
  }
}

bool FrameRenderer::renderGraphics(uint8_t mode) {
  const auto *mi = hw::findVideoMode(mode);
  const int w = mi->width;
  const int h = mi->height;
  resize(w, h);
  const uint8_t crtcOff = m_vga.getCRTCOffset();
  const uint32_t displayStart = m_vga.getDisplayStart();

  // Cache the 256 palette entries once per frame
  std::array<uint32_t, 256> pal{};
  for (int i = 0; i < 256; ++i)
    pal[i] = paletteToARGB(static_cast<uint8_t>(i));

  // Scanlines are converted to colour indices (plane interleave / bit
  // planes → bytes) and then through the palette by the host-SIMD kernels.
  const auto &simdK = utils::simd::kernels();
  const uint8_t *planes[4] = {m_vga.planeData(0), m_vga.planeData(1),
                              m_vga.planeData(2), m_vga.planeData(3)};
  std::vector<uint8_t> line(static_cast<size_t>(w));

  switch (mi->layout) {
  case hw::VMemLayout::Linear256:
    if (!m_vga.isChain4()) {
      // Mode-X: unchained planar 256-colour.  CRTC offset counts
      // character clocks = 2 bytes per plane.
      const int rowBytes = crtcOff ? crtcOff * 2 : w / 4;
      for (int y = 0; y < h; ++y) {
        utils::simd::vgaUnchainedRow(line.data(), planes,
                                     displayStart + y * rowBytes, 0, w);
        simdK.paletteToArgb(&m_pixels[static_cast<size_t>(y) * w],
                            line.data(), w, pal.data());
      }
    } else {
      // Mode 13h (chain-4): CRTC offset counts 8 bytes per unit.  Byte
      // address a lives in plane a & 3 at offset a >> 2.
      const int rowBytes = crtcOff ? crtcOff * 8 : w;
      for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
          uint32_t off = displayStart * 4 + y * rowBytes + x;
          m_pixels[static_cast<size_t>(y) * w + x] =
              pal[m_vga.readPlane(off & 3, off & 0xFFFC)];
        }
    }
    return true;
  case hw::VMemLayout::Planar: {
    // EGA/VGA 16-colour planar modes (and the 2/4-colour variants)
    const int rowBytes = crtcOff ? crtcOff * 2 : w / 8;
    const bool mono = (mode == 0x11 || mode == 0x0F);
    // Monochrome modes only use plane 0: reading it as all four planes
    // yields colour 15 for set bits and 0 otherwise.
    const uint8_t *monoPlanes[4] = {planes[0], planes[0], planes[0], planes[0]};
    for (int y = 0; y < h; ++y) {
      utils::simd::vgaPlanarRow(line.data(), mono ? monoPlanes : planes,
                                displayStart + y * rowBytes, w);
      simdK.paletteToArgb(&m_pixels[static_cast<size_t>(y) * w], line.data(),
                          w, pal.data());
    }
    return true;
  }
  case hw::VMemLayout::CGA4:
  case hw::VMemLayout::CGA2: {
    const bool twoColor = mi->layout == hw::VMemLayout::CGA2;
    for (int y = 0; y < h; ++y) {
      uint32_t rowBase = 0xB8000 + ((y & 1) ? 0x2000u : 0u) + (y >> 1) * 80u;
      for (int x = 0; x < w; ++x) {
        uint8_t idx;
        if (twoColor) {
          uint8_t byte = m_memory.read8(rowBase + (x >> 3));
          idx = (byte & (0x80 >> (x & 7))) ? 15 : 0;
        } else {
          uint8_t byte = m_memory.read8(rowBase + (x >> 2));
          idx = kCga4Palette[(byte >> ((3 - (x & 3)) * 2)) & 3];
        }
        m_pixels[static_cast<size_t>(y) * w + x] = pal[idx];
      }
    }
    return true;
  }
  default:
    return false;
  }
}

bool FrameRenderer::renderVbe() {
  const auto &v = m_bios->vbeState();
  resize(v.width, v.height);
  const uint32_t bytesPP = (v.bpp + 7u) / 8u;
  const uint32_t start = memory::MemoryBus::VBE_LFB_BASE + v.startY * v.pitch +
                         v.startX * bytesPP;
  const uint64_t needed = static_cast<uint64_t>(v.pitch) * v.height;
  const uint8_t *fb = m_memory.contiguousAccess(start, static_cast<uint32_t>(
                                                           std::min<uint64_t>(needed, 0xFFFFFFFFu)));
  if (!fb)
    return false;

  if (v.bpp == 8)
    buildPalette();

  const auto &simdK = utils::simd::kernels();
  for (int y = 0; y < v.height; ++y) {
    const uint8_t *src = fb + static_cast<size_t>(y) * v.pitch;
    uint32_t *dst = &m_pixels[static_cast<size_t>(y) * v.width];
    switch (v.bpp) {
    case 8:
      convertIndexedScanline(src, dst, v.width, m_palette.data());
      break;
    case 15:
      simdK.rgb555ToArgb(dst, src, v.width);
      break;
    case 16:
      simdK.rgb565ToArgb(dst, src, v.width);
      break;
    case 24:
      simdK.rgb24ToArgb(dst, src, v.width);
      break;
    default: // 32
      simdK.rgb32ToArgb(dst, src, v.width);
      break;
    }
  }
  return true;
}

bool FrameRenderer::savePNG(const std::string &path) const {
  return writePNG(path, m_width, m_height, m_pixels);
}

std::string FrameRenderer::textScreen() const {
  std::string out;
  if (m_bios && m_bios->vbeState().mode != 0)
    return out;
  uint8_t mode = m_memory.read8(0x449);
  const auto *mi = hw::findVideoMode(mode);
  if (mi && mi->isGraphics)
    return out;
  uint16_t cols = m_memory.read16(0x44A);
  int rows = m_memory.read8(0x484) + 1;
  if (cols == 0 || cols > 132)
    cols = 80;
  if (rows <= 1 || rows > 60)
    rows = 25;
  const uint32_t vramBase = (mode == 0x07) ? 0xB0000 : 0xB8000;
  const uint32_t pageOffset = m_memory.read16(0x44E);
  for (int row = 0; row < rows; ++row) {
    std::string line;
    for (int col = 0; col < cols; ++col) {
      uint8_t ch = m_memory.read8(vramBase + pageOffset + (row * cols + col) * 2u);
      line += (ch >= 0x20 && ch < 0x7F) ? static_cast<char>(ch) : ' ';
    }
    while (!line.empty() && line.back() == ' ')
      line.pop_back();
    out += line;
    out += '\n';
  }
  while (out.size() >= 2 && out[out.size() - 1] == '\n' &&
         out[out.size() - 2] == '\n')
    out.pop_back();
  return out;
}

} // namespace fador::ui
