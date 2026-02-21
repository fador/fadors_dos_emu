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
    
    // Maps BIOS color attributes to ANSI escape codes
    const char* getAnsiColor(uint8_t attr, bool background);
};

} // namespace fador::ui
