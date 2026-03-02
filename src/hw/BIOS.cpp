#include "BIOS.hpp"
#include "KeyboardController.hpp"
#include "PIT8254.hpp"
#include "../utils/Logger.hpp"
#include <fstream>
#include <algorithm>
#include <thread>

namespace fador::hw {

BIOS::BIOS(cpu::CPU& cpu, memory::MemoryBus& memory, KeyboardController& kbd, PIT8254& pit)
    : m_cpu(cpu)
    , m_memory(memory)
    , m_kbd(kbd)
    , m_pit(pit) {
}

bool BIOS::loadDiskImage(const std::string& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        LOG_ERROR("BIOS: Failed to open disk image: ", path);
        return false;
    }
    
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    m_floppyData.resize(static_cast<size_t>(size));
    if (file.read(reinterpret_cast<char*>(m_floppyData.data()), size)) {
        m_floppyLoaded = true;
        LOG_INFO("BIOS: Loaded floppy image ", path, " (", size, " bytes)");
        return true;
    }
    return false;
}

bool BIOS::handleInterrupt(uint8_t vector) {
    switch (vector) {
        case 0x10:
            handleVideoService();
            return true;
        case 0x13:
            handleDiskService();
            return true;
        case 0x16:
            handleKeyboardService();
            return true;
        case 0x1A:
            handleTimeService();
            return true;
        // HLE stubs for common BIOS/system interrupts
        case 0x11: // Equipment List
            m_cpu.setReg16(cpu::AX, 0x002F); // Typical: FDD, video, RAM, etc.
            LOG_DEBUG("BIOS INT 11h: Equipment List (stubbed)");
            return true;
        case 0x12: // Memory Size
            m_cpu.setReg16(cpu::AX, 640); // 640KB conventional
            LOG_DEBUG("BIOS INT 12h: Memory Size (stubbed)");
            return true;
        case 0x14: // Serial Port
            m_cpu.setReg8(cpu::AH, 0); // No error
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            LOG_DEBUG("BIOS INT 14h: Serial Port (stubbed)");
            return true;
        case 0x15: // System Services
            m_cpu.setReg8(cpu::AH, 0x86); // Function not supported (default)
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            LOG_DEBUG("BIOS INT 15h: System Services (stubbed)");
            return true;
        case 0x17: // Printer
            m_cpu.setReg8(cpu::AH, 0); // No error
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            LOG_DEBUG("BIOS INT 17h: Printer (stubbed)");
            return true;
        case 0x2F: { // Multiplex
            uint16_t ax = m_cpu.getReg16(cpu::AX);
            LOG_DEBUG("BIOS: INT 2Fh Multiplex (stubbed), AX=0x", std::hex, ax);
            m_cpu.setReg8(cpu::AL, 0x00); // Not supported
            return true;
        }
        case 0x1B: // Ctrl-Break
        case 0x1C: // Timer Tick
        case 0x1D: // Video Parameters
        case 0x1E: // Diskette Params
        case 0x1F: // Graphics Char Table
            LOG_DEBUG("BIOS INT ", std::hex, (int)vector, ": System stub (no-op)");
            return true;
    }
    return false;
}

void BIOS::handleVideoService() {
    uint8_t ah = m_cpu.getReg8(cpu::AH);
    switch (ah) {
        case 0x00: { // Set Video Mode
            uint8_t al = m_cpu.getReg8(cpu::AL) & 0x7F; // bit 7 = don't clear if set
            bool clearScreen = !(m_cpu.getReg8(cpu::AL) & 0x80);
            m_memory.write8(0x449, al); // BDA: current mode
            m_memory.write16(0x44A, 80); // BDA: columns per row
            m_memory.write8(0x484, 24);  // BDA: rows - 1
            m_memory.write16(0x44C, 80 * 25 * 2); // BDA: page size in bytes
            m_memory.write8(0x462, 0);   // BDA: active page = 0
            m_memory.write16(0x44E, 0);  // BDA: page offset = 0
            // Reset cursor for all 8 pages
            for (int p = 0; p < 8; ++p) {
                m_memory.write8(0x450 + p * 2, 0);     // col
                m_memory.write8(0x450 + p * 2 + 1, 0); // row
            }
            if (clearScreen) {
                for (uint32_t i = 0; i < 80 * 25 * 2; i += 2) {
                    m_memory.write8(0xB8000 + i, ' ');
                    m_memory.write8(0xB8000 + i + 1, 0x07);
                }
            }
            LOG_VIDEO("BIOS INT 10h: Set Video Mode ", (int)al);
            break;
        }
        case 0x0F: { // Get Video Mode
            m_cpu.setReg8(cpu::AL, m_memory.read8(0x449)); // Current mode
            m_cpu.setReg8(cpu::AH, static_cast<uint8_t>(m_memory.read16(0x44A))); // Columns
            m_cpu.setReg8(cpu::BH, m_memory.read8(0x462)); // Active page
            break;
        }
        case 0x01: { // Set Cursor Type
            m_memory.write16(0x460, m_cpu.getReg16(cpu::CX));
            LOG_DEBUG("BIOS INT 10h: Set Cursor Type CX=", m_cpu.getReg16(cpu::CX));
            break;
        }
        case 0x02: { // Set Cursor Position
            uint8_t page = m_cpu.getReg8(cpu::BH);
            uint8_t dh = m_cpu.getReg8(cpu::DH); // Row
            uint8_t dl = m_cpu.getReg8(cpu::DL); // Col
            if (page > 7) page = 0;
            m_memory.write8(0x450 + page * 2, dl);
            m_memory.write8(0x450 + page * 2 + 1, dh);
            LOG_DEBUG("BIOS INT 10h: Set Cursor Position page=", (int)page, " to ", (int)dh, ":", (int)dl);
            break;
        }
        case 0x03: { // Get Cursor Position/Type
            uint8_t page = m_cpu.getReg8(cpu::BH);
            if (page > 7) page = 0;
            m_cpu.setReg8(cpu::DL, m_memory.read8(0x450 + page * 2));
            m_cpu.setReg8(cpu::DH, m_memory.read8(0x450 + page * 2 + 1));
            m_cpu.setReg16(cpu::CX, m_memory.read16(0x460));
            break;
        }
        case 0x04: { // Read Light Pen Position
            m_cpu.setReg8(cpu::AH, 0); // Not triggered
            LOG_DEBUG("BIOS INT 10h: Read Light Pen (stubbed)");
            break;
        }
        case 0x05: { // Set Active Page
            uint8_t page = m_cpu.getReg8(cpu::AL);
            if (page > 7) page = 0;
            m_memory.write8(0x462, page);
            m_memory.write16(0x44E, static_cast<uint16_t>(page * 80 * 25 * 2));
            LOG_DEBUG("BIOS INT 10h: Set Active Page ", (int)page);
            break;
        }
        case 0x06: // Scroll Up
        case 0x07: { // Scroll Down
            uint8_t al = m_cpu.getReg8(cpu::AL);
            uint8_t row1 = m_cpu.getReg8(cpu::CH);
            uint8_t row2 = m_cpu.getReg8(cpu::DH);
            uint8_t col1 = m_cpu.getReg8(cpu::CL);
            uint8_t col2 = m_cpu.getReg8(cpu::DL);
            uint8_t attr = m_cpu.getReg8(cpu::BH);
            if (row2 > 24) row2 = 24;
            if (col2 > 79) col2 = 79;
            uint8_t lines = al;
            if (lines == 0 || lines > (row2 - row1 + 1)) {
                for (uint8_t r = row1; r <= row2; ++r) {
                    for (uint8_t c = col1; c <= col2; ++c) {
                        uint32_t off = (r * 80 + c) * 2;
                        m_memory.write8(0xB8000 + off, ' ');
                        m_memory.write8(0xB8000 + off + 1, attr);
                    }
                }
            } else if (ah == 0x06) {
                for (uint8_t r = row1; r <= row2 - lines; ++r) {
                    for (uint8_t c = col1; c <= col2; ++c) {
                        uint32_t dst = (r * 80 + c) * 2;
                        uint32_t src = ((r + lines) * 80 + c) * 2;
                        m_memory.write8(0xB8000 + dst, m_memory.read8(0xB8000 + src));
                        m_memory.write8(0xB8000 + dst + 1, m_memory.read8(0xB8000 + src + 1));
                    }
                }
                for (uint8_t r = row2 - lines + 1; r <= row2; ++r) {
                    for (uint8_t c = col1; c <= col2; ++c) {
                        uint32_t off = (r * 80 + c) * 2;
                        m_memory.write8(0xB8000 + off, ' ');
                        m_memory.write8(0xB8000 + off + 1, attr);
                    }
                }
            } else {
                for (uint8_t r = row2; r >= row1 + lines; --r) {
                    for (uint8_t c = col1; c <= col2; ++c) {
                        uint32_t dst = (r * 80 + c) * 2;
                        uint32_t src = ((r - lines) * 80 + c) * 2;
                        m_memory.write8(0xB8000 + dst, m_memory.read8(0xB8000 + src));
                        m_memory.write8(0xB8000 + dst + 1, m_memory.read8(0xB8000 + src + 1));
                    }
                }
                for (uint8_t r = row1; r < row1 + lines; ++r) {
                    for (uint8_t c = col1; c <= col2; ++c) {
                        uint32_t off = (r * 80 + c) * 2;
                        m_memory.write8(0xB8000 + off, ' ');
                        m_memory.write8(0xB8000 + off + 1, attr);
                    }
                }
            }
            LOG_VIDEO("BIOS INT 10h: Scroll ", (ah == 0x06 ? "Up" : "Down"), " AL=", (int)al, " Window=", (int)row1, ",", (int)col1, "-", (int)row2, ",", (int)col2);
            break;
        }
        case 0x08: { // Read Character and Attribute at Cursor
            uint8_t page = m_cpu.getReg8(cpu::BH);
            if (page > 7) page = 0;
            uint8_t col = m_memory.read8(0x450 + page * 2);
            uint8_t row = m_memory.read8(0x450 + page * 2 + 1);
            uint32_t off = (row * 80 + col) * 2;
            m_cpu.setReg8(cpu::AL, m_memory.read8(0xB8000 + off));
            m_cpu.setReg8(cpu::AH, m_memory.read8(0xB8000 + off + 1));
            break;
        }
        case 0x09: { // Write Character and Attribute at Cursor
            uint8_t c = m_cpu.getReg8(cpu::AL);
            uint8_t attr = m_cpu.getReg8(cpu::BL);
            uint16_t count = m_cpu.getReg16(cpu::CX);
            uint8_t cursorCol = m_memory.read8(0x450);
            uint8_t cursorRow = m_memory.read8(0x451);
            for (uint16_t i = 0; i < count; ++i) {
                uint32_t off = (cursorRow * 80 + cursorCol) * 2;
                m_memory.write8(0xB8000 + off, c);
                m_memory.write8(0xB8000 + off + 1, attr);
                cursorCol++;
                if (cursorCol >= 80) { cursorCol = 0; cursorRow++; }
                if (cursorRow >= 25) { cursorRow = 24; }
            }
            // AH=09h does NOT advance the cursor in the BDA
            LOG_VIDEO("BIOS INT 10h AH=09h: Write Char '", (char)c, "' attr=", std::hex, (int)attr, " count=", count);
            break;
        }
        case 0x0A: { // Write Character Only at Cursor (no attribute change)
            uint8_t c = m_cpu.getReg8(cpu::AL);
            uint16_t count = m_cpu.getReg16(cpu::CX);
            uint8_t cursorCol = m_memory.read8(0x450);
            uint8_t cursorRow = m_memory.read8(0x451);
            for (uint16_t i = 0; i < count; ++i) {
                uint32_t off = (cursorRow * 80 + cursorCol) * 2;
                m_memory.write8(0xB8000 + off, c);
                // attribute byte left unchanged
                cursorCol++;
                if (cursorCol >= 80) { cursorCol = 0; cursorRow++; }
                if (cursorRow >= 25) { cursorRow = 24; }
            }
            LOG_VIDEO("BIOS INT 10h AH=0Ah: Write Char Only '", (char)c, "' count=", count);
            break;
        }
        case 0x0B: { // Set Color Palette / Background/Border
            uint8_t bh = m_cpu.getReg8(cpu::BH);
            uint8_t bl = m_cpu.getReg8(cpu::BL);
            LOG_DEBUG("BIOS INT 10h AH=0Bh: Set Palette BH=", (int)bh, " BL=", (int)bl, " (stubbed)");
            break;
        }
        case 0x0E: { // Teletype Output
            uint8_t al = m_cpu.getReg8(cpu::AL);
            uint8_t page = m_cpu.getReg8(cpu::BH);
            if (page > 7) page = 0;
            uint8_t cursorCol = m_memory.read8(0x450 + page * 2);
            uint8_t cursorRow = m_memory.read8(0x450 + page * 2 + 1);

            switch (al) {
                case 0x07: // BEL
                    break;
                case 0x08: // Backspace
                    if (cursorCol > 0) cursorCol--;
                    break;
                case 0x0A: // Line Feed
                    cursorRow++;
                    break;
                case 0x0D: // Carriage Return
                    cursorCol = 0;
                    break;
                default: { // Printable character
                    uint32_t off = (cursorRow * 80 + cursorCol) * 2;
                    m_memory.write8(0xB8000 + off, al);
                    // Keep existing attribute
                    if (m_memory.read8(0xB8000 + off + 1) == 0)
                        m_memory.write8(0xB8000 + off + 1, 0x07);
                    cursorCol++;
                    if (cursorCol >= 80) { cursorCol = 0; cursorRow++; }
                    break;
                }
            }
            // Scroll up if past last row
            if (cursorRow >= 25) {
                // Scroll entire screen up one line
                for (uint8_t r = 0; r < 24; ++r) {
                    for (uint8_t c = 0; c < 80; ++c) {
                        uint32_t dst = (r * 80 + c) * 2;
                        uint32_t src = ((r + 1) * 80 + c) * 2;
                        m_memory.write8(0xB8000 + dst, m_memory.read8(0xB8000 + src));
                        m_memory.write8(0xB8000 + dst + 1, m_memory.read8(0xB8000 + src + 1));
                    }
                }
                // Clear last line
                for (uint8_t c = 0; c < 80; ++c) {
                    uint32_t off = (24 * 80 + c) * 2;
                    m_memory.write8(0xB8000 + off, ' ');
                    m_memory.write8(0xB8000 + off + 1, 0x07);
                }
                cursorRow = 24;
            }
            m_memory.write8(0x450 + page * 2, cursorCol);
            m_memory.write8(0x450 + page * 2 + 1, cursorRow);
            std::cerr << (char)al << std::flush;
            break;
        }
        case 0x10: { // DAC/Palette Functions
            uint8_t al = m_cpu.getReg8(cpu::AL);
            switch (al) {
                case 0x00: // Set Individual Palette Register
                    LOG_DEBUG("BIOS INT 10h AH=10h AL=00h: Set Palette Reg BL=", (int)m_cpu.getReg8(cpu::BL), " (stubbed)");
                    break;
                case 0x01: // Set Overscan (Border) Color
                    LOG_DEBUG("BIOS INT 10h AH=10h AL=01h: Set Border Color (stubbed)");
                    break;
                case 0x02: // Set All Palette Registers
                    LOG_DEBUG("BIOS INT 10h AH=10h AL=02h: Set All Palette (stubbed)");
                    break;
                case 0x03: // Toggle Intensity/Blinking
                    LOG_DEBUG("BIOS INT 10h AH=10h AL=03h: Toggle Blink BL=", (int)m_cpu.getReg8(cpu::BL), " (stubbed)");
                    break;
                case 0x07: // Read Individual Palette Register
                    m_cpu.setReg8(cpu::BH, m_cpu.getReg8(cpu::BL)); // Echo back
                    break;
                case 0x08: // Read Overscan Register
                    m_cpu.setReg8(cpu::BH, 0x00); // Black border
                    break;
                case 0x10: // Set Individual DAC Register
                    LOG_DEBUG("BIOS INT 10h AH=10h AL=10h: Set DAC Reg (stubbed)");
                    break;
                case 0x12: // Set Block of DAC Registers
                    LOG_DEBUG("BIOS INT 10h AH=10h AL=12h: Set DAC Block (stubbed)");
                    break;
                case 0x15: // Read Individual DAC Register
                    m_cpu.setReg8(cpu::DH, 0); m_cpu.setReg8(cpu::CH, 0); m_cpu.setReg8(cpu::CL, 0);
                    break;
                case 0x17: // Read Block of DAC Registers
                    LOG_DEBUG("BIOS INT 10h AH=10h AL=17h: Read DAC Block (stubbed)");
                    break;
                case 0x1A: // Get Color Page State
                    m_cpu.setReg8(cpu::BL, 0); // Paging mode 0
                    m_cpu.setReg8(cpu::BH, 0); // Current page 0
                    break;
                default:
                    LOG_DEBUG("BIOS INT 10h AH=10h: Palette subfn AL=", std::hex, (int)al, " (stubbed)");
                    break;
            }
            break;
        }
        case 0x11: { // Character Generator / Font Services
            uint8_t al = m_cpu.getReg8(cpu::AL);
            switch (al) {
                case 0x30: { // Get Font Information
                    uint8_t bh = m_cpu.getReg8(cpu::BH);
                    // Return standard 8x16 font info
                    m_cpu.setReg16(cpu::CX, 16);  // Bytes per character (height)
                    m_cpu.setReg8(cpu::DL, 24);   // Rows - 1
                    // ES:BP -> font table (point to a dummy area)
                    m_cpu.setSegReg(cpu::ES, 0xC000);
                    m_cpu.setReg16(cpu::BP, 0x0000);
                    LOG_DEBUG("BIOS INT 10h AH=11h AL=30h: Get Font Info pointer=", (int)bh);
                    break;
                }
                default:
                    LOG_DEBUG("BIOS INT 10h AH=11h: Font subfn AL=", std::hex, (int)al, " (stubbed)");
                    break;
            }
            break;
        }
        case 0x12: { // Alternate Function Select (Video Subsystem Configuration)
            uint8_t bl = m_cpu.getReg8(cpu::BL);
            switch (bl) {
                case 0x10: { // Get EGA Info
                    m_cpu.setReg8(cpu::BH, 0x00); // Color mode
                    m_cpu.setReg8(cpu::BL, 0x03); // 256K EGA memory
                    m_cpu.setReg8(cpu::CH, 0x00); // Feature bits
                    m_cpu.setReg8(cpu::CL, 0x09); // Switch settings
                    LOG_DEBUG("BIOS INT 10h AH=12h BL=10h: Get EGA Info");
                    break;
                }
                case 0x30: { // Set Vertical Resolution
                    uint8_t al = m_cpu.getReg8(cpu::AL);
                    m_cpu.setReg8(cpu::AL, 0x12); // Function supported
                    LOG_DEBUG("BIOS INT 10h AH=12h BL=30h: Set VertRes AL=", (int)al, " (stubbed)");
                    break;
                }
                case 0x34: { // Cursor Emulation
                    m_cpu.setReg8(cpu::AL, 0x12);
                    LOG_DEBUG("BIOS INT 10h AH=12h BL=34h: Cursor Emulation (stubbed)");
                    break;
                }
                case 0x36: { // Video Refresh Control
                    m_cpu.setReg8(cpu::AL, 0x12);
                    LOG_DEBUG("BIOS INT 10h AH=12h BL=36h: Video Refresh (stubbed)");
                    break;
                }
                default:
                    LOG_DEBUG("BIOS INT 10h AH=12h: Alt Select BL=", std::hex, (int)bl, " (stubbed)");
                    m_cpu.setReg8(cpu::AL, 0x12);
                    break;
            }
            break;
        }
        case 0x13: { // Write String
            uint8_t al = m_cpu.getReg8(cpu::AL); // Write mode
            uint8_t page = m_cpu.getReg8(cpu::BH);
            uint8_t attr = m_cpu.getReg8(cpu::BL);
            uint16_t count = m_cpu.getReg16(cpu::CX);
            uint8_t row = m_cpu.getReg8(cpu::DH);
            uint8_t col = m_cpu.getReg8(cpu::DL);
            uint16_t es = m_cpu.getSegReg(cpu::ES);
            uint16_t bp = m_cpu.getReg16(cpu::BP);
            uint32_t strAddr = (es << 4) + bp;

            if (page > 7) page = 0;

            for (uint16_t i = 0; i < count; ++i) {
                uint8_t c;
                uint8_t charAttr = attr;
                if (al & 0x02) {
                    // Mode 2/3: string contains char, attr pairs
                    c = m_memory.read8(strAddr++);
                    charAttr = m_memory.read8(strAddr++);
                } else {
                    // Mode 0/1: string is chars only, use BL for attribute
                    c = m_memory.read8(strAddr++);
                }

                // Handle control characters
                if (c == 0x0D) { col = 0; continue; }
                if (c == 0x0A) { row++; continue; }
                if (c == 0x08) { if (col > 0) col--; continue; }
                if (c == 0x07) { continue; } // BEL

                uint32_t off = (row * 80 + col) * 2;
                m_memory.write8(0xB8000 + off, c);
                m_memory.write8(0xB8000 + off + 1, charAttr);
                col++;
                if (col >= 80) { col = 0; row++; }
                if (row >= 25) { row = 24; }
            }

            // Modes 1 and 3 update the cursor position
            if (al & 0x01) {
                m_memory.write8(0x450 + page * 2, col);
                m_memory.write8(0x450 + page * 2 + 1, row);
            }
            LOG_VIDEO("BIOS INT 10h AH=13h: Write String mode=", (int)al, " count=", count);
            break;
        }
        case 0x1A: { // Get/Set Display Combination Code
            uint8_t al = m_cpu.getReg8(cpu::AL);
            if (al == 0x00) {
                // Get: report VGA color
                m_cpu.setReg8(cpu::AL, 0x1A); // Function supported
                m_cpu.setReg8(cpu::BL, 0x08); // Active: VGA w/ color analog
                m_cpu.setReg8(cpu::BH, 0x00); // Alternate: none
            } else {
                m_cpu.setReg8(cpu::AL, 0x1A);
            }
            LOG_DEBUG("BIOS INT 10h AH=1Ah: Display Combo Code AL=", (int)al);
            break;
        }
        case 0x1B: { // Functionality/State Information
            // ES:DI -> 64-byte buffer to fill
            uint16_t es = m_cpu.getSegReg(cpu::ES);
            uint16_t di = m_cpu.getReg16(cpu::DI);
            uint32_t addr = (es << 4) + di;
            // Zero the buffer
            for (int i = 0; i < 64; ++i) m_memory.write8(addr + i, 0);
            // Fill key fields
            m_memory.write8(addr + 0x00, m_memory.read8(0x449)); // Current mode
            m_memory.write16(addr + 0x01, 80);  // Columns
            m_memory.write16(addr + 0x22, 80 * 25 * 2); // Page size
            m_memory.write8(addr + 0x27, 24);   // Rows - 1
            m_memory.write16(addr + 0x28, 16);  // Char height
            m_memory.write8(addr + 0x2A, m_memory.read8(0x462)); // Active page
            m_cpu.setReg8(cpu::AL, 0x1B); // Function supported
            LOG_DEBUG("BIOS INT 10h AH=1Bh: State Info (filled)");
            break;
        }
        case 0xFE: { // Get Video Buffer Segment (Borland/TopView API)
            // Programs pass a guessed video segment in ES:DI and expect
            // ES to be updated with the actual segment to write to.
            // For text mode on CGA/EGA/VGA the answer is always B800h.
            m_cpu.setSegReg(cpu::ES, 0xB800);
            LOG_DEBUG("BIOS INT 10h AH=FEh: Get Video Buffer -> ES=B800h");
            break;
        }
        default:
            LOG_WARN("BIOS INT 10h: Unknown function AH=0x", std::hex, (int)ah);
            break;
    }
}

void BIOS::handleKeyboardService() {
    uint8_t ah = m_cpu.getReg8(cpu::AH);
    switch (ah) {
        case 0x10: // Enhanced Read (fall through to 00h)
        case 0x00: { // Read Character (Blocking)
            if (!m_kbd.hasKey()) {
                // No key available — rewind EIP so INT 16h re-executes
                // next step(), giving the main loop a chance to poll input.
                m_cpu.setEIP(m_cpu.getEIP() - 2);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                return;
            }
            auto [ascii, scancode] = m_kbd.popKey();
            m_cpu.setReg16(cpu::AX,
                (static_cast<uint16_t>(scancode) << 8) | ascii);
            LOG_TRACE("BIOS INT 16h AH=00h: key scan=", std::hex, (int)scancode,
                      " ascii=", (int)ascii);
            break;
        }
        case 0x11: // Enhanced Status (fall through to 01h)
        case 0x01: { // Get Keyboard Status (peek, don't consume)
            if (m_kbd.hasKey()) {
                auto [ascii, scancode] = m_kbd.peekKey();
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_ZERO); // ZF=0 → key available
                m_cpu.setReg16(cpu::AX,
                    (static_cast<uint16_t>(scancode) << 8) | ascii);
            } else {
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_ZERO);  // ZF=1 → no key
            }
            break;
        }
        case 0x02: { // Get Shift Flags
            m_cpu.setReg8(cpu::AL, 0); // No modifier keys pressed
            break;
        }
        default:
            LOG_WARN("BIOS INT 16h: Unknown function AH=0x", std::hex, (int)ah);
            break;
    }
}

void BIOS::initialize() {
    // 1. Setup BDA (BIOS Data Area) at 0x40:0x00
    m_memory.write8(0x449, 0x03);    // Current video mode: 80x25 color text
    m_memory.write16(0x44A, 80);     // Number of columns
    m_memory.write16(0x44C, 80*25*2);// Page size in bytes
    m_memory.write16(0x44E, 0);      // Current page offset
    // Cursor positions for all 8 pages
    for (int p = 0; p < 8; ++p) {
        m_memory.write8(0x450 + p * 2, 0); // Col
        m_memory.write8(0x450 + p * 2 + 1, 0); // Row
    }
    m_memory.write16(0x460, 0x0607); // Cursor type: start=6 end=7
    m_memory.write8(0x462, 0);       // Active video page
    m_memory.write16(0x463, 0x3D4);  // CRTC base I/O port (color)
    m_memory.write8(0x484, 24);      // Number of rows - 1
    m_memory.write16(0x485, 16);     // Character height in pixels

    // Memory size at 0x40:0x13
    m_memory.write16(0x413, 640); // 640KB conventional memory

    // Initialize VRAM to spaces with default attribute
    for (uint32_t i = 0; i < 80 * 25 * 2; i += 2) {
        m_memory.write8(0xB8000 + i, ' ');
        m_memory.write8(0xB8000 + i + 1, 0x07);
    }

    // 2. Setup IVT (Interrupt Vector Table) vectors
    // Even though we HLE, we point them to a dummy IRET at 0xFFFF:0x000E
    uint16_t dummyCS = 0xFFFF;
    uint16_t dummyIP = 0x000E;
    m_memory.write8(0xFFFFE, 0xCF); // IRET instruction at dummy location

    // Low exception vectors (0x00–0x07: divide, debug, NMI, breakpoint, overflow,
    // bounds, #UD, device-not-avail) – also point at the dummy IRET so that a
    // triggerInterrupt(6) for an unrecognised opcode returns cleanly instead of
    // executing the IVT table as code (CS:0 crash).
    for (uint8_t v = 0x00; v <= 0x07; ++v) {
        m_memory.write16(v * 4, dummyIP);
        m_memory.write16(v * 4 + 2, dummyCS);
    }
    uint8_t handledVectors[] = { 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F };
    for (uint8_t v : handledVectors) {
        m_memory.write16(v * 4, dummyIP);
        m_memory.write16(v * 4 + 2, dummyCS);
    }
    
    LOG_INFO("BIOS: BDA and IVT initialized.");
}

void BIOS::handleTimeService() {
    uint8_t ah = m_cpu.getReg8(cpu::AH);
    switch (ah) {
        case 0x00: { // Read system timer
            LOG_TRACE("BIOS INT 1Ah: Get System Time");
            // BIOS timer tick count at 0x40:0x6C
            uint32_t ticks = m_memory.read32(0x46C);
            m_cpu.setReg16(cpu::CX, static_cast<uint16_t>(ticks >> 16));
            m_cpu.setReg16(cpu::DX, static_cast<uint16_t>(ticks & 0xFFFF));
            m_cpu.setReg8(cpu::AL, 0); // Midnight flag
            break;
        }
        default:
            LOG_WARN("BIOS INT 1Ah: Unknown function AH=0x", std::hex, (int)ah);
            break;
    }
}

void BIOS::handleDiskService() {
    uint8_t ah = m_cpu.getReg8(cpu::AH);
    uint8_t dl = m_cpu.getReg8(cpu::DL); // Drive

    switch (ah) {
        case 0x00: { // Reset Disk System
            LOG_DEBUG("BIOS INT 13h: Reset Disk System");
            m_cpu.setReg8(cpu::AH, 0); // Success
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            break;
        }
        case 0x02: { // Read Sectors
            uint8_t al = m_cpu.getReg8(cpu::AL); // Num sectors
            uint8_t ch = m_cpu.getReg8(cpu::CH); // Cylinder (low 8)
            uint8_t cl = m_cpu.getReg8(cpu::CL); // Sector (1-63) + Cy (high 2)
            uint8_t dh = m_cpu.getReg8(cpu::DH); // Head
            uint16_t es = m_cpu.getSegReg(cpu::ES);
            uint16_t bx = m_cpu.getReg16(cpu::BX);

            uint16_t cylinder = ch | ((cl & 0xC0) << 2);
            uint8_t sector = cl & 0x3F;

            LOG_DEBUG("BIOS INT 13h: Read ", (int)al, " sectors from Drive ", (int)dl, 
                      " (C:", cylinder, " H:", (int)dh, " S:", (int)sector, ") to ", 
                      std::hex, es, ":", bx);

            if (dl == 0x00 && m_floppyLoaded) {
                // CHS Mapping for 1.44MB floppy: 80/2/18
                uint32_t lba = ((cylinder * 2 + dh) * 18) + (sector - 1);
                uint32_t dest = (es << 4) + bx;
                
                for (int i = 0; i < al; ++i) {
                    uint32_t srcStart = (lba + i) * 512;
                    if (srcStart + 512 > m_floppyData.size()) {
                        m_cpu.setReg8(cpu::AH, 0x01); // Invalid parameter
                        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
                        return;
                    }
                    for (int j = 0; j < 512; ++j) {
                        m_memory.write8(dest + i * 512 + j, m_floppyData[srcStart + j]);
                    }
                }
                m_cpu.setReg8(cpu::AH, 0); // Success
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
                m_cpu.setReg8(cpu::AL, al); // AL = sectors read
            } else {
                LOG_WARN("BIOS INT 13h: Drive not found or not loaded: ", (int)dl);
                m_cpu.setReg8(cpu::AH, 0x01); // Invalid function/parameter
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            }
            break;
        }
        case 0x08: { // Get Drive Parameters
            if (dl == 0x00) { // Floppy 0
                m_cpu.setReg8(cpu::AH, 0);
                m_cpu.setReg8(cpu::BL, 0x04); // 1.44MB
                m_cpu.setReg8(cpu::CH, 79);   // Max cylinder index (low 8 bits)
                // CL: bits 6-7 = high 2 bits of cyl, bits 0-5 = max sector number
                m_cpu.setReg8(cpu::CL, 18);   // 18 sectors/track
                m_cpu.setReg8(cpu::DH, 1);    // Max head index (0-based, so 1 means 2 heads)
                m_cpu.setReg8(cpu::DL, 1);    // Num floppy drives
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            } else {
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
                m_cpu.setReg8(cpu::AH, 0x07); // Drive parameter activity failed
            }
            break;
        }
        case 0x15: { // Get Disk Type
            if (dl == 0x00) { // Floppy
                m_cpu.setReg8(cpu::AH, 0x01); // Floppy, no change line
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            } else {
                m_cpu.setReg8(cpu::AH, 0x00); // Drive not present
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            }
            break;
        }
        default:
            LOG_WARN("BIOS INT 13h: Unknown function AH=0x", std::hex, (int)ah);
            m_cpu.setReg8(cpu::AH, 0x01);
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            break;
    }
}

} // namespace fador::hw
