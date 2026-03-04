#pragma once
#include <cstdint>
#include <vector>
#include "../memory/MemoryBus.hpp"

namespace fador::ui {

class TerminalRenderer {
public:
    TerminalRenderer(memory::MemoryBus& memory);
    ~TerminalRenderer() = default;

    // Renders the current state of VRAM to the console
    // force - if true, redraw the entire screen
    void render(bool force = false);

    // Clears the host terminal screen
    void clearScreen();

private:
    memory::MemoryBus& m_memory;
    
    struct CharCell {
        uint8_t c;
        uint8_t attr;
        bool operator!=(const CharCell& other) const {
            return c != other.c || attr != other.attr;
        }
    };

    std::vector<CharCell> m_lastFrame;
    int m_lastTextCols = 0;
    int m_lastTextRows = 0;

    // Graphics mode state
    static constexpr int TARGET_COLS = 80; // terminal width target
    static constexpr uint32_t PALETTE_BASE = 0xE0000;

    std::vector<uint32_t> m_lastGraphicsFrame; // packed RGB per cell
    uint8_t m_lastVideoMode = 0xFF;

    void renderTextMode(bool force);
    void renderGraphicsMode(bool force);

    // Decode a pixel's palette index from VRAM depending on mode layout
    uint8_t readPixel(int x, int y, uint8_t mode) const;

    // Maps BIOS color attributes to ANSI escape codes
    const char* getAnsiColor(uint8_t attr, bool background);

    // Appends the UTF-8 encoding of a CP437 character to the buffer
    static void appendCP437(std::string& buf, uint8_t ch);
};

} // namespace fador::ui
