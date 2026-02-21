#include "TerminalRenderer.hpp"
#include <iostream>
#include <iomanip>

namespace fador::ui {

TerminalRenderer::TerminalRenderer(memory::MemoryBus& memory)
    : m_memory(memory) {
    m_lastFrame.resize(80 * 25, {0, 0});
}

void TerminalRenderer::clearScreen() {
    std::cout << "\033[2J\033[H" << std::flush;
}

void TerminalRenderer::render(bool force) {
    bool changed = false;
    uint32_t vramBase = 0xB8000;

    // Check for changes
    for (int i = 0; i < 80 * 25; ++i) {
        uint8_t c = m_memory.read8(vramBase + i * 2);
        uint8_t attr = m_memory.read8(vramBase + i * 2 + 1);
        if (force || c != m_lastFrame[i].c || attr != m_lastFrame[i].attr) {
            changed = true;
            m_lastFrame[i] = {c, attr};
        }
    }

    if (!changed) return;

    // Move cursor to top-left
    std::cout << "\033[H";

    uint8_t currentAttr = 0xFF;

    for (int row = 0; row < 25; ++row) {
        for (int col = 0; col < 80; ++col) {
            const auto& cell = m_lastFrame[row * 80 + col];
            
            // Optimization: only send color codes if they change
            if (cell.attr != currentAttr) {
                // ANSI: Esc[<bg>;<fg>m
                // Foreground bits 0-3, Background bits 4-6 (bit 7 is blink/intensity)
                std::cout << "\033[0;" << getAnsiColor(cell.attr & 0x0F, false)
                          << ";" << getAnsiColor((cell.attr & 0x70) >> 4, true) << "m";
                currentAttr = cell.attr;
            }

            if (cell.c == 0) std::cout << ' ';
            else std::cout << (char)cell.c;
        }
        std::cout << "\n";
    }
    std::cout << "\033[0m" << std::flush; // Reset attributes
}

const char* TerminalRenderer::getAnsiColor(uint8_t color, bool background) {
    int base = background ? 40 : 30;
    switch (color & 0x07) {
        case 0: return background ? "40" : "30"; // Black
        case 1: return background ? "44" : "34"; // Blue
        case 2: return background ? "42" : "32"; // Green
        case 3: return background ? "46" : "36"; // Cyan
        case 4: return background ? "41" : "31"; // Red
        case 5: return background ? "45" : "35"; // Magenta
        case 6: return background ? "43" : "33"; // Brown/Yellow
        case 7: return background ? "47" : "37"; // Light Gray
        default: return background ? "40" : "37";
    }
}

} // namespace fador::ui
