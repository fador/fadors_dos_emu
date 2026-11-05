#ifdef HAVE_SDL2
#include "SDLRenderer.hpp"
#include "../hw/BIOS.hpp"
#include "../hw/VideoMode.hpp"
#include "VGAFont.hpp"
#include "../utils/Logger.hpp"
#include <algorithm>
#include <cstring>
#include <unordered_map>

namespace fador::ui {


// ── Standard 16-color CGA/EGA palette (ARGB8888) ──────────────────────
static constexpr uint32_t kCGAPalette[16] = {
    0xFF000000, // 0  Black
    0xFF0000AA, // 1  Blue
    0xFF00AA00, // 2  Green
    0xFF00AAAA, // 3  Cyan
    0xFFAA0000, // 4  Red
    0xFFAA00AA, // 5  Magenta
    0xFFAA5500, // 6  Brown
    0xFFAAAAAA, // 7  Light Gray
    0xFF555555, // 8  Dark Gray
    0xFF5555FF, // 9  Light Blue
    0xFF55FF55, // 10 Light Green
    0xFF55FFFF, // 11 Light Cyan
    0xFFFF5555, // 12 Light Red
    0xFFFF55FF, // 13 Light Magenta
    0xFFFFFF55, // 14 Yellow
    0xFFFFFFFF, // 15 White
};

// ── Construction / destruction ─────────────────────────────────────────

SDLRenderer::SDLRenderer(memory::MemoryBus &memory, hw::KeyboardController &kbd,
                         hw::VGAController &vga)
    : m_memory(memory), m_kbd(kbd), m_vga(vga) {
  SDL_SetMainReady();
  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    LOG_ERROR("SDL_Init failed: ", SDL_GetError());
    return;
  }

  // Default to mode 03h text size: 720×400, scaled up
  m_texWidth = 720;
  m_texHeight = 400;

  m_window = SDL_CreateWindow("Fador's DOS Emulator", SDL_WINDOWPOS_CENTERED,
                              SDL_WINDOWPOS_CENTERED, m_texWidth * SCALE,
                              m_texHeight * SCALE,
                              SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

  if (!m_window) {
    LOG_ERROR("SDL_CreateWindow failed: ", SDL_GetError());
    return;
  }

  m_renderer = SDL_CreateRenderer(m_window, -1, SDL_RENDERER_ACCELERATED);
  if (!m_renderer) {
    // Fall back to software renderer
    m_renderer = SDL_CreateRenderer(m_window, -1, SDL_RENDERER_SOFTWARE);
  }
  if (!m_renderer) {
    LOG_ERROR("SDL_CreateRenderer failed: ", SDL_GetError());
    return;
  }

  SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0"); // nearest-neighbor

  m_texture =
      SDL_CreateTexture(m_renderer, SDL_PIXELFORMAT_ARGB8888,
                        SDL_TEXTUREACCESS_STREAMING, m_texWidth, m_texHeight);

  m_framebuffer.resize(m_texWidth * m_texHeight, 0xFF000000);
}

SDLRenderer::~SDLRenderer() {
  if (m_texture)
    SDL_DestroyTexture(m_texture);
  if (m_renderer)
    SDL_DestroyRenderer(m_renderer);
  if (m_window)
    SDL_DestroyWindow(m_window);
  SDL_Quit();
}

// ── Pixel helpers ──────────────────────────────────────────────────────

uint32_t SDLRenderer::paletteToARGB(uint8_t index) const {
  const uint8_t *pal = m_memory.dacPalette();
  uint8_t r6 = pal[index * 3 + 0];
  uint8_t g6 = pal[index * 3 + 1];
  uint8_t b6 = pal[index * 3 + 2];
  uint8_t r = (r6 << 2) | (r6 >> 4);
  uint8_t g = (g6 << 2) | (g6 >> 4);
  uint8_t b = (b6 << 2) | (b6 >> 4);
  return 0xFF000000u | (uint32_t(r) << 16) | (uint32_t(g) << 8) | b;
}

uint8_t SDLRenderer::readPixel(int x, int y, uint8_t mode) const {
  const auto *mi = hw::findVideoMode(mode);
  if (!mi)
    return 0;

  switch (mi->layout) {
  case hw::VMemLayout::Linear256:
    return m_memory.read8(mi->vramBase + y * mi->width + x);
  case hw::VMemLayout::CGA4: {
    uint32_t bank = (y & 1) ? 0x2000 : 0;
    uint32_t off = bank + (y >> 1) * 80 + (x >> 2);
    uint8_t shift = (3 - (x & 3)) * 2;
    return (m_memory.read8(mi->vramBase + off) >> shift) & 0x03;
  }
  case hw::VMemLayout::CGA2: {
    uint32_t bank = (y & 1) ? 0x2000 : 0;
    uint32_t off = bank + (y >> 1) * 80 + (x >> 3);
    uint8_t bit = 7 - (x & 7);
    return (m_memory.read8(mi->vramBase + off) >> bit) & 1;
  }
  case hw::VMemLayout::Planar: {
    uint32_t byteOff = (y * mi->width + x) / 8;
    uint8_t bit = 7 - (x & 7);
    uint32_t planeSize = (static_cast<uint32_t>(mi->width) * mi->height) / 8;
    uint8_t pixel = 0;
    for (int p = 0; p < 4; ++p) {
      if (m_memory.read8(mi->vramBase + p * planeSize + byteOff) & (1 << bit))
        pixel |= (1 << p);
    }
    return pixel;
  }
  default:
    return 0;
  }
}

// ── Rendering ──────────────────────────────────────────────────────────

void SDLRenderer::render(bool /*force*/) {
  if (!m_window || !m_renderer || !m_texture)
    return;

  uint8_t videoMode = m_memory.read8(0x449);
  const auto *mi = hw::findVideoMode(videoMode);
  if(!mi) {
    LOG_WARN("Unknown video mode: ", std::hex, int(videoMode), std::dec);
    return;
  }

  // Determine required texture dimensions
  int needW, needH;
  if (mi && mi->isGraphics) {
    needW = mi->width;
    needH = mi->height;
  } else {
    // Text mode: render at 8×16 font size × text dimensions
    uint16_t cols = m_memory.read16(0x44A);
    uint8_t rows = m_memory.read8(0x484) + 1;
    if (cols == 0)
      cols = 80;
    if (rows == 0)
      rows = 25;
    needW = cols * 8;
    needH = rows * 16;
  }

  // Recreate texture if dimensions changed
  if (needW != m_texWidth || needH != m_texHeight ||
      videoMode != m_lastVideoMode) {
        LOG_INFO("Video mode ", std::hex, int(videoMode), std::dec,
                 " requires texture size ", needW, "x", needH);
    m_texWidth = needW;
    m_texHeight = needH;
    if (m_texture)
      SDL_DestroyTexture(m_texture);
    m_texture =
        SDL_CreateTexture(m_renderer, SDL_PIXELFORMAT_ARGB8888,
                          SDL_TEXTUREACCESS_STREAMING, m_texWidth, m_texHeight);
    m_framebuffer.resize(m_texWidth * m_texHeight, 0xFF000000);
    // Resize window to fit (with scale)
    int scale = SCALE;
    // For higher-res modes, reduce scale so window isn't enormous
    if (m_texWidth > 400 || m_texHeight > 400)
      scale = 2;
    if (m_texWidth > 640 || m_texHeight > 480)
      scale = 1;
    int targetW = m_texWidth * scale;
    int targetH = m_texHeight * scale;

    // Keep auto-resize within the display's usable bounds so mode changes
    // can't grow the window beyond the desktop.
    SDL_Rect usable{};
    if (SDL_GetDisplayUsableBounds(0, &usable) == 0 && usable.w > 0 &&
        usable.h > 0) {
      int maxW = std::max(320, (usable.w * 95) / 100);
      int maxH = std::max(200, (usable.h * 95) / 100);
      targetW = std::min(targetW, maxW);
      targetH = std::min(targetH, maxH);
    }

    SDL_SetWindowSize(m_window, targetW, targetH);
    m_lastVideoMode = videoMode;
  }

  if (mi && mi->isGraphics) {
    renderGraphicsMode();
  } else {
    renderTextMode();
  }

  // Upload framebuffer to texture and present
  SDL_UpdateTexture(m_texture, nullptr, m_framebuffer.data(),
                    m_texWidth * sizeof(uint32_t));
  SDL_RenderClear(m_renderer);

  // Preserve texture aspect ratio regardless of window size (letterbox).
  int winW = 1, winH = 1;
  SDL_GetWindowSize(m_window, &winW, &winH);
  float sx = static_cast<float>(winW) / static_cast<float>(m_texWidth);
  float sy = static_cast<float>(winH) / static_cast<float>(m_texHeight);
  float s = std::min(sx, sy);
  if (s <= 0.0f)
    s = 1.0f;
  int dstW = std::max(1, static_cast<int>(m_texWidth * s));
  int dstH = std::max(1, static_cast<int>(m_texHeight * s));
  SDL_Rect dst{(winW - dstW) / 2, (winH - dstH) / 2, dstW, dstH};
  SDL_RenderCopy(m_renderer, m_texture, nullptr, &dst);
  SDL_RenderPresent(m_renderer);
}

void SDLRenderer::renderTextMode() {
  uint16_t cols = m_memory.read16(0x44A);
  uint8_t rows = m_memory.read8(0x484) + 1;
  if (cols == 0)
    cols = 80;
  if (rows == 0)
    rows = 25;

  uint32_t vramBase = 0xB8000;

  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      uint8_t ch = m_memory.read8(vramBase + (row * cols + col) * 2);
      uint8_t attr = m_memory.read8(vramBase + (row * cols + col) * 2 + 1);

      uint32_t fgColor = kCGAPalette[attr & 0x0F];
      uint32_t bgColor = kCGAPalette[(attr >> 4) & 0x07];

      const uint8_t *glyph = kVGAFont8x16[ch];

      for (int gy = 0; gy < 16; ++gy) {
        uint8_t bits = glyph[gy];
        int py = row * 16 + gy;
        if (py >= m_texHeight)
          break;
        for (int gx = 0; gx < 8; ++gx) {
          int px = col * 8 + gx;
          if (px >= m_texWidth)
            break;
          m_framebuffer[py * m_texWidth + px] =
              (bits & (0x80 >> gx)) ? fgColor : bgColor;
        }
      }
    }
  }

  // Draw cursor
  uint8_t curRow = m_memory.read8(0x451);
  uint8_t curCol = m_memory.read8(0x450);
  if (curRow < rows && curCol < cols) {
    uint8_t attr = m_memory.read8(vramBase + (curRow * cols + curCol) * 2 + 1);
    uint32_t curColor = kCGAPalette[attr & 0x0F];
    for (int gy = 14; gy < 16; ++gy) {
      int py = curRow * 16 + gy;
      if (py >= m_texHeight)
        break;
      for (int gx = 0; gx < 8; ++gx) {
        int px = curCol * 8 + gx;
        if (px >= m_texWidth)
          break;
        m_framebuffer[py * m_texWidth + px] = curColor;
      }
    }
  }
}

void SDLRenderer::renderGraphicsMode() {
  uint8_t videoMode = m_memory.read8(0x449);
  const auto *mi = hw::findVideoMode(videoMode);
  if (!mi)
    return;

  int w = mi->width;
  int h = mi->height;

  // CRTC Offset register (index 0x13) controls the actual row stride
  // in VRAM. Games may set a virtual width wider than the visible width
  // for smooth scrolling.  Use it when available, fall back to standard.
  uint8_t crtcOff = m_vga.getCRTCOffset();

  if (mi->layout == hw::VMemLayout::Linear256 && !m_vga.isChain4()) {
    // Mode-X: Chain-4 disabled, unchained 256-color planar mode.
    // CRTC offset is in character-clocks; one char = 8 pixels = 2 bytes
    // per plane in planar 8bpp mode.
    uint32_t displayStart = m_vga.getDisplayStart();
    int rowBytes = (crtcOff > 0) ? crtcOff * 2 : w / 4;
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        int plane = x & 3;
        uint32_t offset = displayStart + y * rowBytes + (x >> 2);
        m_framebuffer[y * m_texWidth + x] =
            paletteToARGB(m_vga.readPlane(plane, offset));
      }
    }
  } else if (mi->layout == hw::VMemLayout::Linear256) {
    // Standard mode 13h (Chain-4): CRTC offset in character-clocks;
    // one char = 8 pixels = 8 bytes at 8bpp.
    uint32_t displayStart = m_vga.getDisplayStart();
    int rowBytes = (crtcOff > 0) ? crtcOff * 8 : w;
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        uint32_t vramOff = displayStart + y * rowBytes + x;
        m_framebuffer[y * m_texWidth + x] =
            paletteToARGB(m_vga.readPlane(vramOff & 3, vramOff >> 2));
      }
    }
  } else {
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        m_framebuffer[y * m_texWidth + x] =
            paletteToARGB(readPixel(x, y, videoMode));
      }
    }
  }
}

// ── Input handling ─────────────────────────────────────────────────────

bool SDLRenderer::pollInput() {
  SDL_Event ev;
  bool processed = false;

  while (SDL_PollEvent(&ev)) {
    processed = true;
    switch (ev.type) {
    case SDL_QUIT:
      m_quit = true;
      break;
    case SDL_KEYDOWN:
    case SDL_KEYUP:
      handleSDLKey(ev.key);
      break;
    case SDL_MOUSEMOTION:
    case SDL_MOUSEBUTTONDOWN:
    case SDL_MOUSEBUTTONUP:
      handleSDLMouse(ev);
      break;
    }
  }
  return processed;
}

namespace {
struct KeyMapping {
  uint8_t scancode;
  uint8_t asciiUnshifted;
  uint8_t asciiShifted;
  bool extended;
};

static const std::unordered_map<SDL_Scancode, KeyMapping> kKeyMap = {
    {SDL_SCANCODE_ESCAPE, {0x01, 27, 27, false}},
    {SDL_SCANCODE_1, {0x02, '1', '!', false}},
    {SDL_SCANCODE_2, {0x03, '2', '@', false}},
    {SDL_SCANCODE_3, {0x04, '3', '#', false}},
    {SDL_SCANCODE_4, {0x05, '4', '$', false}},
    {SDL_SCANCODE_5, {0x06, '5', '%', false}},
    {SDL_SCANCODE_6, {0x07, '6', '^', false}},
    {SDL_SCANCODE_7, {0x08, '7', '&', false}},
    {SDL_SCANCODE_8, {0x09, '8', '*', false}},
    {SDL_SCANCODE_9, {0x0A, '9', '(', false}},
    {SDL_SCANCODE_0, {0x0B, '0', ')', false}},
    {SDL_SCANCODE_MINUS, {0x0C, '-', '_', false}},
    {SDL_SCANCODE_EQUALS, {0x0D, '=', '+', false}},
    {SDL_SCANCODE_BACKSPACE, {0x0E, 8, 8, false}},
    {SDL_SCANCODE_TAB, {0x0F, 9, 9, false}},
    {SDL_SCANCODE_Q, {0x10, 'q', 'Q', false}},
    {SDL_SCANCODE_W, {0x11, 'w', 'W', false}},
    {SDL_SCANCODE_E, {0x12, 'e', 'E', false}},
    {SDL_SCANCODE_R, {0x13, 'r', 'R', false}},
    {SDL_SCANCODE_T, {0x14, 't', 'T', false}},
    {SDL_SCANCODE_Y, {0x15, 'y', 'Y', false}},
    {SDL_SCANCODE_U, {0x16, 'u', 'U', false}},
    {SDL_SCANCODE_I, {0x17, 'i', 'I', false}},
    {SDL_SCANCODE_O, {0x18, 'o', 'O', false}},
    {SDL_SCANCODE_P, {0x19, 'p', 'P', false}},
    {SDL_SCANCODE_LEFTBRACKET, {0x1A, '[', '{', false}},
    {SDL_SCANCODE_RIGHTBRACKET, {0x1B, ']', '}', false}},
    {SDL_SCANCODE_RETURN, {0x1C, 13, 13, false}},
    {SDL_SCANCODE_A, {0x1E, 'a', 'A', false}},
    {SDL_SCANCODE_S, {0x1F, 's', 'S', false}},
    {SDL_SCANCODE_D, {0x20, 'd', 'D', false}},
    {SDL_SCANCODE_F, {0x21, 'f', 'F', false}},
    {SDL_SCANCODE_G, {0x22, 'g', 'G', false}},
    {SDL_SCANCODE_H, {0x23, 'h', 'H', false}},
    {SDL_SCANCODE_J, {0x24, 'j', 'J', false}},
    {SDL_SCANCODE_K, {0x25, 'k', 'K', false}},
    {SDL_SCANCODE_L, {0x26, 'l', 'L', false}},
    {SDL_SCANCODE_SEMICOLON, {0x27, ';', ':', false}},
    {SDL_SCANCODE_APOSTROPHE, {0x28, '\'', '"', false}},
    {SDL_SCANCODE_GRAVE, {0x29, '`', '~', false}},
    {SDL_SCANCODE_LSHIFT, {0x2A, 0, 0, false}},
    {SDL_SCANCODE_BACKSLASH, {0x2B, '\\', '|', false}},
    {SDL_SCANCODE_Z, {0x2C, 'z', 'Z', false}},
    {SDL_SCANCODE_X, {0x2D, 'x', 'X', false}},
    {SDL_SCANCODE_C, {0x2E, 'c', 'C', false}},
    {SDL_SCANCODE_V, {0x2F, 'v', 'V', false}},
    {SDL_SCANCODE_B, {0x30, 'b', 'B', false}},
    {SDL_SCANCODE_N, {0x31, 'n', 'N', false}},
    {SDL_SCANCODE_M, {0x32, 'm', 'M', false}},
    {SDL_SCANCODE_COMMA, {0x33, ',', '<', false}},
    {SDL_SCANCODE_PERIOD, {0x34, '.', '>', false}},
    {SDL_SCANCODE_SLASH, {0x35, '/', '?', false}},
    {SDL_SCANCODE_RSHIFT, {0x36, 0, 0, false}},
    {SDL_SCANCODE_SPACE, {0x39, ' ', ' ', false}},
    {SDL_SCANCODE_CAPSLOCK, {0x3A, 0, 0, false}},
    {SDL_SCANCODE_F1, {0x3B, 0, 0, false}},
    {SDL_SCANCODE_F2, {0x3C, 0, 0, false}},
    {SDL_SCANCODE_F3, {0x3D, 0, 0, false}},
    {SDL_SCANCODE_F4, {0x3E, 0, 0, false}},
    {SDL_SCANCODE_F5, {0x3F, 0, 0, false}},
    {SDL_SCANCODE_F6, {0x40, 0, 0, false}},
    {SDL_SCANCODE_F7, {0x41, 0, 0, false}},
    {SDL_SCANCODE_F8, {0x42, 0, 0, false}},
    {SDL_SCANCODE_F9, {0x43, 0, 0, false}},
    {SDL_SCANCODE_F10, {0x44, 0, 0, false}},
    {SDL_SCANCODE_F11, {0x57, 0, 0, true}},
    {SDL_SCANCODE_F12, {0x58, 0, 0, true}},
    {SDL_SCANCODE_UP, {0x48, 0, 0, true}},
    {SDL_SCANCODE_DOWN, {0x50, 0, 0, true}},
    {SDL_SCANCODE_LEFT, {0x4B, 0, 0, true}},
    {SDL_SCANCODE_RIGHT, {0x4D, 0, 0, true}},
    {SDL_SCANCODE_HOME, {0x47, 0, 0, true}},
    {SDL_SCANCODE_END, {0x4F, 0, 0, true}},
    {SDL_SCANCODE_PAGEUP, {0x49, 0, 0, true}},
    {SDL_SCANCODE_PAGEDOWN, {0x51, 0, 0, true}},
    {SDL_SCANCODE_INSERT, {0x52, 0, 0, true}},
    {SDL_SCANCODE_DELETE, {0x53, 0, 0, true}},
    {SDL_SCANCODE_LCTRL, {0x1D, 0, 0, false}},
    {SDL_SCANCODE_RCTRL, {0x1D, 0, 0, true}},
    {SDL_SCANCODE_LALT, {0x38, 0, 0, false}},
    {SDL_SCANCODE_RALT, {0x38, 0, 0, true}},
    {SDL_SCANCODE_NUMLOCKCLEAR, {0x45, 0, 0, false}},
    {SDL_SCANCODE_SCROLLLOCK, {0x46, 0, 0, false}},
    {SDL_SCANCODE_PRINTSCREEN, {0x37, 0, 0, false}},
};
} // namespace

void SDLRenderer::handleSDLKey(const SDL_KeyboardEvent &ev) {
  bool pressed = (ev.type == SDL_KEYDOWN);
  // Ignore repeats for make/break simulation
  if (ev.repeat && pressed)
    return;

  uint8_t scancode = 0;
  uint8_t ascii = 0;
  bool extended = false; // true for keys that need 0xE0 prefix

  // Handle Ctrl+F10 for mouse capture toggle immediately
  if (ev.keysym.scancode == SDL_SCANCODE_F10 && pressed &&
      (ev.keysym.mod & KMOD_CTRL)) {
    m_mouseCaptured = !m_mouseCaptured;
    SDL_SetRelativeMouseMode(m_mouseCaptured ? SDL_TRUE : SDL_FALSE);
    LOG_INFO("Mouse capture ", m_mouseCaptured ? "enabled" : "disabled");
    return; // Eat the shortcut
  }

  // Lookup scancode mapping
  auto it = kKeyMap.find(ev.keysym.scancode);
  if (it != kKeyMap.end()) {
    scancode = it->second.scancode;
    extended = it->second.extended;
    ascii = (ev.keysym.mod & KMOD_SHIFT) ? it->second.asciiShifted
                                         : it->second.asciiUnshifted;
  }

  // Ctrl+letter: ascii = 1..26
  if (scancode != 0 && (ev.keysym.mod & KMOD_CTRL) && ascii >= 'a' &&
      ascii <= 'z') {
    ascii = ascii - 'a' + 1;
  } else if (scancode != 0 && (ev.keysym.mod & KMOD_CTRL) && ascii >= 'A' &&
             ascii <= 'Z') {
    ascii = ascii - 'A' + 1;
  }

  // Alt+letter: ascii = 0, with alt-specific scancode
  if (scancode != 0 && (ev.keysym.mod & KMOD_ALT) &&
      !(ev.keysym.mod & KMOD_CTRL)) {
    ascii = 0; // Alt combinations have zero ASCII
  }

  // Update BDA shift flags at 0x417
  updateShiftFlags(ev);

  if (scancode != 0) {
    if (extended) {
      if (pressed) {
        m_kbd.pushMakeKeyExtended(ascii, scancode);
      } else {
        m_kbd.pushBreakKeyExtended(scancode);
      }
    } else {
      if (pressed) {
        m_kbd.pushMakeKey(ascii, scancode);
      } else {
        m_kbd.pushBreakKey(scancode);
      }
    }
  }
}

void SDLRenderer::updateShiftFlags(const SDL_KeyboardEvent &ev) {
  // BDA 0x417 shift flag bits:
  //   bit 0 = Right Shift pressed
  //   bit 1 = Left Shift pressed
  //   bit 2 = Ctrl pressed
  //   bit 3 = Alt pressed
  //   bit 4 = Scroll Lock active
  //   bit 5 = Num Lock active
  //   bit 6 = Caps Lock active
  //   bit 7 = Insert active
  uint8_t flags = m_memory.read8(0x417);

  bool pressed = (ev.type == SDL_KEYDOWN);

  switch (ev.keysym.scancode) {
  case SDL_SCANCODE_RSHIFT:
    flags = pressed ? (flags | 0x01) : (flags & ~0x01);
    break;
  case SDL_SCANCODE_LSHIFT:
    flags = pressed ? (flags | 0x02) : (flags & ~0x02);
    break;
  case SDL_SCANCODE_LCTRL:
  case SDL_SCANCODE_RCTRL:
    flags = pressed ? (flags | 0x04) : (flags & ~0x04);
    break;
  case SDL_SCANCODE_LALT:
  case SDL_SCANCODE_RALT:
    flags = pressed ? (flags | 0x08) : (flags & ~0x08);
    break;
  case SDL_SCANCODE_NUMLOCKCLEAR:
    if (pressed) flags ^= 0x20;
    break;
  case SDL_SCANCODE_SCROLLLOCK:
    if (pressed) flags ^= 0x10;
    break;
  case SDL_SCANCODE_CAPSLOCK:
    if (pressed) flags ^= 0x40;
    break;
  default:
    break;
  }

  m_memory.write8(0x417, flags);
}

void SDLRenderer::handleSDLMouse(const SDL_Event &ev) {
  if (!m_bios)
    return;
  auto &ms = m_bios->mouseState();

  // Convert SDL pixel coordinates to DOS virtual mouse coordinates
  // Get the actual window size to compute scale
  int winW = 1, winH = 1;
  SDL_GetWindowSize(m_window, &winW, &winH);

  // Scale SDL coordinates to the game's mouse coordinate range
  int rangeX = ms.maxX - ms.minX + 1;
  int rangeY = ms.maxY - ms.minY + 1;

  if (ev.type == SDL_MOUSEMOTION) {
    // Use SDL relative motion for mickey counters, but scale them correctly
    // to the virtual resolution, preserving sub-pixel fractional remainders.
    float rawXRel =
        static_cast<float>(ev.motion.xrel) * static_cast<float>(rangeX) / winW;
    float rawYRel =
        static_cast<float>(ev.motion.yrel) * static_cast<float>(rangeY) / winH;

    rawXRel += m_mouseXFraction;
    rawYRel += m_mouseYFraction;

    int16_t scaledXRel = static_cast<int16_t>(rawXRel);
    int16_t scaledYRel = static_cast<int16_t>(rawYRel);

    m_mouseXFraction = rawXRel - static_cast<float>(scaledXRel);
    m_mouseYFraction = rawYRel - static_cast<float>(scaledYRel);

    ms.mickeysX += scaledXRel;
    ms.mickeysY += scaledYRel;

    if (m_mouseCaptured) {
      // Apply bounds-checked relative deltas directly to absolute position to
      // avoid snapping to host OS coords
      ms.x += scaledXRel;
      ms.y += scaledYRel;
    } else {
      // Absolute position scaled to the game's coordinate bounds
      ms.x = static_cast<int16_t>(ms.minX + ev.motion.x * rangeX / winW);
      ms.y = static_cast<int16_t>(ms.minY + ev.motion.y * rangeY / winH);
    }
  }

  if (ev.type == SDL_MOUSEBUTTONDOWN || ev.type == SDL_MOUSEBUTTONUP) {
    if (ev.type == SDL_MOUSEBUTTONDOWN && !m_mouseCaptured) {
      // Auto-capture mouse on first click
      m_mouseCaptured = true;
      SDL_SetRelativeMouseMode(SDL_TRUE);
      LOG_INFO("Mouse captured (press Ctrl+F10 to release)");
    }

    if (!m_mouseCaptured) {
      ms.x = static_cast<int16_t>(ms.minX + ev.button.x * rangeX / winW);
      ms.y = static_cast<int16_t>(ms.minY + ev.button.y * rangeY / winH);
    }

    bool btnPressed = (ev.type == SDL_MOUSEBUTTONDOWN);
    int dosBtn = -1;
    switch (ev.button.button) {
    case SDL_BUTTON_LEFT:
      dosBtn = 0;
      break;
    case SDL_BUTTON_RIGHT:
      dosBtn = 1;
      break;
    case SDL_BUTTON_MIDDLE:
      dosBtn = 2;
      break;
    }
    if (dosBtn >= 0 && dosBtn <= 2) {
      if (btnPressed) {
        ms.buttons |= (1 << dosBtn);
        ms.pressCount[dosBtn]++;
        ms.lastPressX[dosBtn] = ms.x;
        ms.lastPressY[dosBtn] = ms.y;
      } else {
        ms.buttons &= ~(1 << dosBtn);
        ms.releaseCount[dosBtn]++;
        ms.lastReleaseX[dosBtn] = ms.x;
        ms.lastReleaseY[dosBtn] = ms.y;
      }
    }
  }

  // Clamp to bounds
  if (ms.x < ms.minX)
    ms.x = ms.minX;
  if (ms.x > ms.maxX)
    ms.x = ms.maxX;
  if (ms.y < ms.minY)
    ms.y = ms.minY;
  if (ms.y > ms.maxY)
    ms.y = ms.maxY;
}

} // namespace fador::ui

#endif // HAVE_SDL2
