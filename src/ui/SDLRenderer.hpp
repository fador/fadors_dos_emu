#pragma once
#ifdef HAVE_SDL2

#include "../hw/KeyboardController.hpp"
#include "../hw/VGAController.hpp"
#include "../memory/MemoryBus.hpp"
#include <SDL.h>
#include <cstdint>
#include <vector>

namespace fador::hw {
class BIOS;
}

namespace fador::ui {

// SDL2 graphical window renderer for the DOS emulator.
// Renders both text and graphics video modes into a native window.
// Also handles keyboard and mouse input via SDL events.
class SDLRenderer {
public:
  SDLRenderer(memory::MemoryBus &memory, hw::KeyboardController &kbd,
              hw::VGAController &vga);
  ~SDLRenderer();

  void setBIOS(hw::BIOS &bios) { m_bios = &bios; }

  // Renders the current VRAM state to the SDL window
  void render(bool force = false);

  // Polls SDL events and pushes keys/mouse to the keyboard controller
  // Returns true if input was processed
  bool pollInput();

  // Returns true if the user closed the window
  bool shouldQuit() const { return m_quit; }

private:
  memory::MemoryBus &m_memory;
  hw::KeyboardController &m_kbd;
  hw::VGAController &m_vga;
  hw::BIOS *m_bios = nullptr;

  SDL_Window *m_window = nullptr;
  SDL_Renderer *m_renderer = nullptr;
  SDL_Texture *m_texture = nullptr;

  int m_texWidth = 0;
  int m_texHeight = 0;
  uint8_t m_lastVideoMode = 0xFF;
  bool m_quit = false;

  // Sub-pixel fractional trackers for precision mouse delta scaling
  float m_mouseXFraction = 0.0f;
  float m_mouseYFraction = 0.0f;
  bool m_mouseCaptured = false;

  // Window scale factor for small resolutions
  static constexpr int SCALE = 3;

  // Pixel buffer (ARGB8888)
  std::vector<uint32_t> m_framebuffer;

  static constexpr uint32_t PALETTE_BASE = 0xE0000;

  void renderTextMode();
  void renderGraphicsMode();

  // Read a single pixel from VRAM given the current video mode
  uint8_t readPixel(int x, int y, uint8_t mode) const;

  // Convert 6-bit VGA palette entry to ARGB8888
  uint32_t paletteToARGB(uint8_t index) const;

  // CP437 bitmap font (8x16)
  static const uint8_t kFont8x16[256][16];

  // Handle an SDL key event
  void handleSDLKey(const SDL_KeyboardEvent &ev);

  // Update BDA shift flags (0x417/0x418) based on modifier key state
  void updateShiftFlags(const SDL_KeyboardEvent &ev);

  // Handle an SDL mouse event
  void handleSDLMouse(const SDL_Event &ev);
};

} // namespace fador::ui

#endif // HAVE_SDL2
