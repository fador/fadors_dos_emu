#include "FrameRenderer.hpp"
#include "../hw/BIOS.hpp"
#include "../hw/VideoMode.hpp"
#include "VGAFont.hpp"

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
  uint8_t mode = m_memory.read8(0x449);
  const auto *mi = hw::findVideoMode(mode);
  if (!mi)
    return false;
  m_modeKey = mode;
  if (mi->isGraphics)
    return renderGraphics(mode);
  renderText(drawCursor);
  return true;
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

  switch (mi->layout) {
  case hw::VMemLayout::Linear256:
    if (!m_vga.isChain4()) {
      // Mode-X: unchained planar 256-colour.  CRTC offset counts
      // character clocks = 2 bytes per plane.
      const int rowBytes = crtcOff ? crtcOff * 2 : w / 4;
      for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
          m_pixels[static_cast<size_t>(y) * w + x] = pal[m_vga.readPlane(
              x & 3, displayStart + y * rowBytes + (x >> 2))];
    } else {
      // Mode 13h (chain-4): CRTC offset counts 8 bytes per unit.
      const int rowBytes = crtcOff ? crtcOff * 8 : w;
      for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
          uint32_t off = displayStart * 4 + y * rowBytes + x;
          m_pixels[static_cast<size_t>(y) * w + x] =
              pal[m_vga.readPlane(off & 3, (off >> 2) & 0xFFFF)];
        }
    }
    return true;
  case hw::VMemLayout::Planar: {
    // EGA/VGA 16-colour planar modes (and the 2/4-colour variants)
    const int rowBytes = crtcOff ? crtcOff * 2 : w / 8;
    const bool mono = (mode == 0x11 || mode == 0x0F);
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        uint32_t off = displayStart + y * rowBytes + (x >> 3);
        uint8_t bit = static_cast<uint8_t>(0x80 >> (x & 7));
        uint8_t idx = 0;
        if (mono) {
          idx = (m_vga.readPlane(0, off) & bit) ? 15 : 0;
        } else {
          for (int p = 0; p < 4; ++p)
            if (m_vga.readPlane(p, off) & bit)
              idx |= static_cast<uint8_t>(1 << p);
        }
        m_pixels[static_cast<size_t>(y) * w + x] = pal[idx];
      }
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

  std::array<uint32_t, 256> pal{};
  if (v.bpp == 8)
    for (int i = 0; i < 256; ++i)
      pal[i] = paletteToARGB(static_cast<uint8_t>(i));

  for (int y = 0; y < v.height; ++y) {
    const uint8_t *src = fb + static_cast<size_t>(y) * v.pitch;
    uint32_t *dst = &m_pixels[static_cast<size_t>(y) * v.width];
    switch (v.bpp) {
    case 8:
      for (int x = 0; x < v.width; ++x)
        dst[x] = pal[src[x]];
      break;
    case 15:
      for (int x = 0; x < v.width; ++x) {
        uint16_t p = static_cast<uint16_t>(src[x * 2] | (src[x * 2 + 1] << 8));
        uint32_t r = (p >> 10) & 0x1F, g = (p >> 5) & 0x1F, b = p & 0x1F;
        dst[x] = 0xFF000000u | (((r << 3) | (r >> 2)) << 16) |
                 (((g << 3) | (g >> 2)) << 8) | ((b << 3) | (b >> 2));
      }
      break;
    case 16:
      for (int x = 0; x < v.width; ++x) {
        uint16_t p = static_cast<uint16_t>(src[x * 2] | (src[x * 2 + 1] << 8));
        uint32_t r = (p >> 11) & 0x1F, g = (p >> 5) & 0x3F, b = p & 0x1F;
        dst[x] = 0xFF000000u | (((r << 3) | (r >> 2)) << 16) |
                 (((g << 2) | (g >> 4)) << 8) | ((b << 3) | (b >> 2));
      }
      break;
    case 24:
      for (int x = 0; x < v.width; ++x)
        dst[x] = 0xFF000000u | (uint32_t(src[x * 3 + 2]) << 16) |
                 (uint32_t(src[x * 3 + 1]) << 8) | src[x * 3];
      break;
    default: // 32
      for (int x = 0; x < v.width; ++x)
        dst[x] = 0xFF000000u | (uint32_t(src[x * 4 + 2]) << 16) |
                 (uint32_t(src[x * 4 + 1]) << 8) | src[x * 4];
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
