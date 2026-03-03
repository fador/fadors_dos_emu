#include "TerminalRenderer.hpp"
#include "../hw/VideoMode.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>

namespace fador::ui {

TerminalRenderer::TerminalRenderer(memory::MemoryBus& memory)
    : m_memory(memory) {
    m_lastFrame.resize(80 * 25, {0, 0});
    m_lastTextCols = 80;
    m_lastTextRows = 25;
}

void TerminalRenderer::clearScreen() {
    std::cout << "\033[2J\033[H" << std::flush;
}

void TerminalRenderer::render(bool force) {
    uint8_t videoMode = m_memory.read8(0x449);
    if (videoMode != m_lastVideoMode) {
        // Mode changed \u2014 force full redraw and clear
        m_lastVideoMode = videoMode;
        clearScreen();
        force = true;
    }

    const auto* mi = hw::findVideoMode(videoMode);
    if (mi && mi->isGraphics) {
        renderGraphicsMode(force);
    } else {
        renderTextMode(force);
    }
}

void TerminalRenderer::renderTextMode(bool force) {
    uint16_t cols = m_memory.read16(0x44A);
    uint8_t  rows = m_memory.read8(0x484) + 1;
    if (cols == 0) cols = 80;
    if (rows == 0) rows = 25;

    int totalCells = cols * rows;
    if (cols != m_lastTextCols || rows != m_lastTextRows) {
        m_lastFrame.resize(totalCells, {0, 0});
        m_lastTextCols = cols;
        m_lastTextRows = rows;
        force = true;
    }

    bool changed = false;
    uint32_t vramBase = 0xB8000;

    for (int i = 0; i < totalCells; ++i) {
        uint8_t c = m_memory.read8(vramBase + i * 2);
        uint8_t attr = m_memory.read8(vramBase + i * 2 + 1);
        if (force || c != m_lastFrame[i].c || attr != m_lastFrame[i].attr) {
            changed = true;
            m_lastFrame[i] = {c, attr};
        }
    }

    if (!changed) return;

    std::string buf;
    buf.reserve(cols * rows * 4);
    buf += "\033[?25l"; // Hide cursor during repaint
    uint8_t currentAttr = 0xFF;

    for (int row = 0; row < rows; ++row) {
        buf += "\033[";
        buf += std::to_string(row + 1);
        buf += ";1H";
        for (int col = 0; col < cols; ++col) {
            const auto& cell = m_lastFrame[row * cols + col];
            if (cell.attr != currentAttr) {
                buf += "\033[0;";
                buf += getAnsiColor(cell.attr & 0x0F, false);
                buf += ";";
                buf += getAnsiColor((cell.attr & 0x70) >> 4, true);
                buf += "m";
                currentAttr = cell.attr;
            }
            buf += (cell.c == 0) ? ' ' : static_cast<char>(cell.c);
        }
    }
    buf += "\033[0m";
    // Position terminal cursor at the emulated cursor position
    uint8_t curRow = m_memory.read8(0x451);
    uint8_t curCol = m_memory.read8(0x450);
    buf += "\033[";
    buf += std::to_string(curRow + 1);
    buf += ";";
    buf += std::to_string(curCol + 1);
    buf += "H";
    buf += "\033[?25h"; // Show cursor
    std::cout.write(buf.data(), buf.size());
    std::cout.flush();
}

uint8_t TerminalRenderer::readPixel(int x, int y, uint8_t mode) const {
    const auto* mi = hw::findVideoMode(mode);
    if (!mi) return 0;

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

void TerminalRenderer::renderGraphicsMode(bool force) {
    static const char ramp[] = " .,:;+*?#%@";
    constexpr int rampLen = 11;

    uint8_t videoMode = m_memory.read8(0x449);
    const auto* mi = hw::findVideoMode(videoMode);
    if (!mi) return;

    int gfxW = mi->width;
    int gfxH = mi->height;

    // Compute block size to fit ~TARGET_COLS wide, maintaining aspect ratio
    int blockW = std::max(1, gfxW / TARGET_COLS);
    int blockH = blockW; // square blocks for aspect
    // In text terminals chars are ~2x tall, so double vertical sampling
    blockH = std::max(1, blockW * 2);

    int outCols = gfxW / blockW;
    int outRows = gfxH / blockH;
    if (outCols < 1) outCols = 1;
    if (outRows < 1) outRows = 1;

    int totalCells = outCols * outRows;
    if (static_cast<int>(m_lastGraphicsFrame.size()) != totalCells) {
        m_lastGraphicsFrame.resize(totalCells, 0xFFFFFFFF);
        force = true;
    }

    struct CellColor { uint8_t r, g, b; };
    std::vector<CellColor> cells(totalCells);
    bool changed = false;

    const uint8_t* pal = m_memory.directAccess(PALETTE_BASE);

    for (int row = 0; row < outRows; ++row) {
        for (int col = 0; col < outCols; ++col) {
            int totalR = 0, totalG = 0, totalB = 0;
            int count = 0;
            for (int py = 0; py < blockH; ++py) {
                int y = row * blockH + py;
                if (y >= gfxH) break;
                for (int px = 0; px < blockW; ++px) {
                    int x = col * blockW + px;
                    if (x >= gfxW) break;
                    uint8_t idx = readPixel(x, y, videoMode);

                    // Use palette to get RGB (6-bit to 8-bit)
                    uint8_t r6 = pal[idx * 3 + 0];
                    uint8_t g6 = pal[idx * 3 + 1];
                    uint8_t b6 = pal[idx * 3 + 2];
                    totalR += (r6 << 2) | (r6 >> 4);
                    totalG += (g6 << 2) | (g6 >> 4);
                    totalB += (b6 << 2) | (b6 >> 4);
                    count++;
                }
            }
            if (count == 0) count = 1;
            uint8_t avgR = static_cast<uint8_t>(totalR / count);
            uint8_t avgG = static_cast<uint8_t>(totalG / count);
            uint8_t avgB = static_cast<uint8_t>(totalB / count);

            cells[row * outCols + col] = {avgR, avgG, avgB};

            uint32_t packed = (uint32_t(avgR) << 16) | (uint32_t(avgG) << 8) | avgB;
            if (packed != m_lastGraphicsFrame[row * outCols + col]) {
                changed = true;
                m_lastGraphicsFrame[row * outCols + col] = packed;
            }
        }
    }

    if (!changed && !force) return;

    std::cout << "\033[H";
    for (int row = 0; row < outRows; ++row) {
        for (int col = 0; col < outCols; ++col) {
            const auto& c = cells[row * outCols + col];
            int bright = (77 * c.r + 150 * c.g + 29 * c.b) >> 8;
            int ci = bright * (rampLen - 1) / 255;
            if (ci >= rampLen) ci = rampLen - 1;

            std::cout << "\033[38;2;"
                      << (int)c.r << ';' << (int)c.g << ';' << (int)c.b
                      << 'm' << ramp[ci];
        }
        std::cout << '\n';
    }
    std::cout << "\033[0m" << std::flush;
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
