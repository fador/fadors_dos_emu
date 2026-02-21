#include "BIOS.hpp"
#include "KeyboardController.hpp"
#include "PIT8254.hpp"
#include "../utils/Logger.hpp"
#include <fstream>
#include <algorithm>

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
            uint8_t al = m_cpu.getReg8(cpu::AL);
            // Only text mode 03h supported for now
            m_memory.write8(0x449, al); // BIOS Data Area: current mode
            // Clear VRAM
            for (uint32_t i = 0; i < 80 * 25 * 2; ++i) m_memory.write8(0xB8000 + i, 0);
            LOG_VIDEO("BIOS INT 10h: Set Video Mode ", (int)al);
            break;
        }
        case 0x0F: { // Get Video Mode
            m_cpu.setReg8(cpu::AL, m_memory.read8(0x449)); // Current mode
            m_cpu.setReg8(cpu::AH, 80); // Columns
            m_cpu.setReg8(cpu::BH, 0); // Active page
            break;
        }
        case 0x01: { // Set Cursor Type
            m_memory.write16(0x460, m_cpu.getReg16(cpu::CX));
            LOG_DEBUG("BIOS INT 10h: Set Cursor Type CX=", m_cpu.getReg16(cpu::CX));
            break;
        }
        case 0x02: { // Set Cursor Position
            uint8_t dh = m_cpu.getReg8(cpu::DH); // Row
            uint8_t dl = m_cpu.getReg8(cpu::DL); // Col
            m_memory.write8(0x450, dl);
            m_memory.write8(0x451, dh);
            LOG_DEBUG("BIOS INT 10h: Set Cursor Position to ", (int)dh, ":", (int)dl);
            break;
        }
        case 0x03: { // Get Cursor Position/Type
            m_cpu.setReg8(cpu::DH, m_memory.read8(0x451));
            m_cpu.setReg8(cpu::DL, m_memory.read8(0x450));
            m_cpu.setReg16(cpu::CX, m_memory.read16(0x460));
            break;
        }
        case 0x05: { // Set Active Page
            m_memory.write8(0x462, m_cpu.getReg8(cpu::AL));
            LOG_DEBUG("BIOS INT 10h: Set Active Page ", (int)m_cpu.getReg8(cpu::AL));
            break;
        }
        case 0x06: // Scroll Up
        case 0x07: { // Scroll Down
            // For now, just clear the window if whole screen
            uint8_t al = m_cpu.getReg8(cpu::AL);
            uint8_t row1 = m_cpu.getReg8(cpu::CH);
            uint8_t row2 = m_cpu.getReg8(cpu::DH);
            uint8_t col1 = m_cpu.getReg8(cpu::CL);
            uint8_t col2 = m_cpu.getReg8(cpu::DL);
            uint8_t attr = m_cpu.getReg8(cpu::BH);
            if (row1 == 0 && col1 == 0 && row2 == 24 && col2 == 79) {
                for (uint32_t i = 0; i < 80 * 25 * 2; i += 2) {
                    m_memory.write8(0xB8000 + i, ' ');
                    m_memory.write8(0xB8000 + i + 1, attr);
                }
            }
            LOG_VIDEO("BIOS INT 10h: Scroll ", (ah == 0x06 ? "Up" : "Down"), " AL=", (int)al, " Window=", (int)row1, ",", (int)col1, "-", (int)row2, ",", (int)col2);
            break;
        }
        case 0x08: { // Read Character and Attribute
            uint8_t al = m_memory.read8(0xB8000 + ((m_memory.read8(0x451) * 80 + m_memory.read8(0x450)) * 2));
            uint8_t attr = m_memory.read8(0xB8000 + ((m_memory.read8(0x451) * 80 + m_memory.read8(0x450)) * 2) + 1);
            m_cpu.setReg8(cpu::AL, al);
            m_cpu.setReg8(cpu::AH, attr);
            break;
        }
        case 0x09: { // Write Character and Attribute
            uint8_t c = m_cpu.getReg8(cpu::AL);
            uint8_t attr = m_cpu.getReg8(cpu::BL);
            uint16_t count = m_cpu.getReg16(cpu::CX);
            LOG_VIDEO("BIOS INT 10h AH=09h: Write Char '", (char)c, "' attr=", std::hex, (int)attr, " count=", count);
            break;
        }
        case 0x0E: { // Teletype Output
            uint8_t al = m_cpu.getReg8(cpu::AL);
            LOG_VIDEO("BIOS INT 10h AH=0Eh: Teletype Output '", (char)al, "' (0x", std::hex, (int)al, ")");
            uint16_t cursorCol = m_memory.read8(0x450);
            uint16_t cursorRow = m_memory.read8(0x451);
            uint32_t offset = (cursorRow * 80 + cursorCol) * 2;
            m_memory.write8(0xB8000 + offset, al);
            m_memory.write8(0xB8000 + offset + 1, 0x07);
            cursorCol++;
            if (cursorCol >= 80) { cursorCol = 0; cursorRow++; }
            if (cursorRow >= 25) { cursorRow = 0; }
            m_memory.write8(0x450, static_cast<uint8_t>(cursorCol));
            m_memory.write8(0x451, static_cast<uint8_t>(cursorRow));
            std::cerr << (char)al << std::flush;
            LOG_VIDEO("BIOS INT 10h: Teletype Output '", (char)al, "' at ", (int)cursorRow, ":", (int)cursorCol);
            break;
        }
        case 0x11: { // Read Light Pen Position - Not implemented
            m_cpu.setReg8(cpu::AL, 0); // No light pen
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_ZERO);
            LOG_DEBUG("BIOS INT 10h: Read Light Pen Position (stubbed)");
            break;
        }
        case 0xFE: { // Write VBE Protected Mode Info - Not implemented
            LOG_DEBUG("BIOS INT 10h: Write VBE Protected Mode Info (stubbed)");
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
        case 0x00: { // Read Character (Blocking)
            LOG_TRACE("BIOS INT 16h: Read Character (Blocking)");
            // In a better emulator, we'd yield or loop until a key is pushed
            // For now, we just check and return whatever is in the buffer or 0
            uint8_t scancode = m_kbd.read8(0x60);
            m_cpu.setReg16(cpu::AX, (static_cast<uint16_t>(scancode) << 8)); // Scancode in AH, ASCII in AL (simplified)
            break;
        }
        case 0x01: { // Get Keyboard Status
            uint8_t status = m_kbd.read8(0x64);
            if (status & 0x01) {
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_ZERO);
                // Return character in AX without removing it (peek)
                // This is slightly tricky as our KBD buffer pops on read60.
                // For HLE, we might need a peek function.
                m_cpu.setReg16(cpu::AX, 0); // Mock for now
            } else {
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_ZERO);
            }
            break;
        }
        default:
            LOG_WARN("BIOS INT 16h: Unknown function AH=0x", std::hex, (int)ah);
            break;
    }
}

void BIOS::initialize() {
    // 1. Setup BDA (BIOS Data Area) at 0x40:0x00
    // Cursor position already handled at 0x450
    m_memory.write8(0x450, 0); // Col
    m_memory.write8(0x451, 0); // Row
    
    // Memory size at 0x40:0x13
    m_memory.write16(0x413, 640); // 640KB conventional memory

    // 2. Setup IVT (Interrupt Vector Table) vectors
    // Even though we HLE, we point them to a dummy IRET at 0xFFFF:0x000E
    uint16_t dummyCS = 0xFFFF;
    uint16_t dummyIP = 0x000E;
    m_memory.write8(0xFFFFE, 0xCF); // IRET instruction at dummy location

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
