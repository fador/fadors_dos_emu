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
    if (background) {
        // Background uses 3 bits (0-7)
        switch (color & 0x07) {
            case 0: return "40"; // Black
            case 1: return "44"; // Blue
            case 2: return "42"; // Green
            case 3: return "46"; // Cyan
            case 4: return "41"; // Red
            case 5: return "45"; // Magenta
            case 6: return "43"; // Brown/Yellow
            case 7: return "47"; // Light Gray
            default: return "40";
        }
    }
    // Foreground uses 4 bits (0-15), with 8-15 being bright/bold variants
    switch (color & 0x0F) {
        case 0:  return "30";  // Black
        case 1:  return "34";  // Blue
        case 2:  return "32";  // Green
        case 3:  return "36";  // Cyan
        case 4:  return "31";  // Red
        case 5:  return "35";  // Magenta
        case 6:  return "33";  // Brown/Yellow
        case 7:  return "37";  // Light Gray
        case 8:  return "90";  // Dark Gray (bright black)
        case 9:  return "94";  // Bright Blue
        case 10: return "92";  // Bright Green
        case 11: return "96";  // Bright Cyan
        case 12: return "91";  // Bright Red
        case 13: return "95";  // Bright Magenta
        case 14: return "93";  // Bright Yellow
        case 15: return "97";  // Bright White
        default: return "37";
    }
}

} // namespace fador::ui
