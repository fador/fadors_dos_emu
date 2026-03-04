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
            appendCP437(buf, cell.c);
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

// Full CP437 to Unicode mapping table (256 entries).
// Characters 0x00-0x1F map to special display glyphs (not control codes).
// Characters 0x20-0x7E map to standard ASCII.
// Characters 0x80-0xFF map to extended CP437 (accented, box-drawing, math).
static constexpr char16_t cp437ToUnicode[256] = {
    // 0x00-0x0F
    u' ',      u'\u263A', u'\u263B', u'\u2665', u'\u2666', u'\u2663', u'\u2660', u'\u2022',
    u'\u25D8', u'\u25CB', u'\u25D9', u'\u2642', u'\u2640', u'\u266A', u'\u266B', u'\u263C',
    // 0x10-0x1F
    u'\u25BA', u'\u25C4', u'\u2195', u'\u203C', u'\u00B6', u'\u00A7', u'\u25AC', u'\u21A8',
    u'\u2191', u'\u2193', u'\u2192', u'\u2190', u'\u221F', u'\u2194', u'\u25B2', u'\u25BC',
    // 0x20-0x7E: standard ASCII
    u' ', u'!', u'"', u'#', u'$', u'%', u'&', u'\'', u'(', u')', u'*', u'+', u',', u'-', u'.', u'/',
    u'0', u'1', u'2', u'3', u'4', u'5', u'6', u'7', u'8', u'9', u':', u';', u'<', u'=', u'>', u'?',
    u'@', u'A', u'B', u'C', u'D', u'E', u'F', u'G', u'H', u'I', u'J', u'K', u'L', u'M', u'N', u'O',
    u'P', u'Q', u'R', u'S', u'T', u'U', u'V', u'W', u'X', u'Y', u'Z', u'[', u'\\', u']', u'^', u'_',
    u'`', u'a', u'b', u'c', u'd', u'e', u'f', u'g', u'h', u'i', u'j', u'k', u'l', u'm', u'n', u'o',
    u'p', u'q', u'r', u's', u't', u'u', u'v', u'w', u'x', u'y', u'z', u'{', u'|', u'}', u'~',
    // 0x7F
    u'\u2302',
    // 0x80-0x8F
    u'\u00C7', u'\u00FC', u'\u00E9', u'\u00E2', u'\u00E4', u'\u00E0', u'\u00E5', u'\u00E7',
    u'\u00EA', u'\u00EB', u'\u00E8', u'\u00EF', u'\u00EE', u'\u00EC', u'\u00C4', u'\u00C5',
    // 0x90-0x9F
    u'\u00C9', u'\u00E6', u'\u00C6', u'\u00F4', u'\u00F6', u'\u00F2', u'\u00FB', u'\u00F9',
    u'\u00FF', u'\u00D6', u'\u00DC', u'\u00A2', u'\u00A3', u'\u00A5', u'\u20A7', u'\u0192',
    // 0xA0-0xAF
    u'\u00E1', u'\u00ED', u'\u00F3', u'\u00FA', u'\u00F1', u'\u00D1', u'\u00AA', u'\u00BA',
    u'\u00BF', u'\u2310', u'\u00AC', u'\u00BD', u'\u00BC', u'\u00A1', u'\u00AB', u'\u00BB',
    // 0xB0-0xBF: shade + box drawing
    u'\u2591', u'\u2592', u'\u2593', u'\u2502', u'\u2524', u'\u2561', u'\u2562', u'\u2556',
    u'\u2555', u'\u2563', u'\u2551', u'\u2557', u'\u255D', u'\u255C', u'\u255B', u'\u2510',
    // 0xC0-0xCF
    u'\u2514', u'\u2534', u'\u252C', u'\u251C', u'\u2500', u'\u253C', u'\u255E', u'\u255F',
    u'\u255A', u'\u2554', u'\u2569', u'\u2566', u'\u2560', u'\u2550', u'\u256C', u'\u2567',
    // 0xD0-0xDF
    u'\u2568', u'\u2564', u'\u2565', u'\u2559', u'\u2558', u'\u2552', u'\u2553', u'\u256B',
    u'\u256A', u'\u2518', u'\u250C', u'\u2588', u'\u2584', u'\u258C', u'\u2590', u'\u2580',
    // 0xE0-0xEF: Greek/math
    u'\u03B1', u'\u00DF', u'\u0393', u'\u03C0', u'\u03A3', u'\u03C3', u'\u00B5', u'\u03C4',
    u'\u03A6', u'\u0398', u'\u03A9', u'\u03B4', u'\u221E', u'\u03C6', u'\u03B5', u'\u2229',
    // 0xF0-0xFF
    u'\u2261', u'\u00B1', u'\u2265', u'\u2264', u'\u2320', u'\u2321', u'\u00F7', u'\u2248',
    u'\u00B0', u'\u2219', u'\u00B7', u'\u221A', u'\u207F', u'\u00B2', u'\u25A0', u'\u00A0',
};

void TerminalRenderer::appendCP437(std::string& buf, uint8_t ch) {
    char16_t u = cp437ToUnicode[ch];
    if (u < 0x80) {
        buf += static_cast<char>(u);
    } else if (u < 0x800) {
        buf += static_cast<char>(0xC0 | (u >> 6));
        buf += static_cast<char>(0x80 | (u & 0x3F));
    } else {
        buf += static_cast<char>(0xE0 | (u >> 12));
        buf += static_cast<char>(0x80 | ((u >> 6) & 0x3F));
        buf += static_cast<char>(0x80 | (u & 0x3F));
    }
}

} // namespace fador::ui
