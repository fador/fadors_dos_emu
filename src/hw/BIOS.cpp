#include "BIOS.hpp"
#include "KeyboardController.hpp"
#include "PIT8254.hpp"
#include "../utils/Logger.hpp"

namespace fador::hw {

BIOS::BIOS(cpu::CPU& cpu, memory::MemoryBus& memory, KeyboardController& kbd, PIT8254& pit)
    : m_cpu(cpu)
    , m_memory(memory)
    , m_kbd(kbd)
    , m_pit(pit) {
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
    }
    return false;
}

void BIOS::handleVideoService() {
    uint8_t ah = m_cpu.getReg8(cpu::AH);
    switch (ah) {
        case 0x0E: { // Teletype Output
            uint8_t al = m_cpu.getReg8(cpu::AL);
            // Get cursor position from BDA (0x40:0x50 - Page 0 cursor pos)
            uint16_t cursorCol = m_memory.read8(0x450);
            uint16_t cursorRow = m_memory.read8(0x451);

            // Write to VRAM (0xB800:0000)
            // 80x25 text mode: 2 bytes per char (char, attribute)
            uint32_t offset = (cursorRow * 80 + cursorCol) * 2;
            m_memory.write8(0xB8000 + offset, al);
            m_memory.write8(0xB8000 + offset + 1, 0x07); // Light gray on black

            // Advance cursor
            cursorCol++;
            if (cursorCol >= 80) {
                cursorCol = 0;
                cursorRow++;
            }
            if (cursorRow >= 25) {
                // Should scroll, but for now just wrap
                cursorRow = 0;
            }

            m_memory.write8(0x450, static_cast<uint8_t>(cursorCol));
            m_memory.write8(0x451, static_cast<uint8_t>(cursorRow));
            
            LOG_TRACE("BIOS INT 10h: Teletype Output '", (char)al, "' at ", (int)cursorRow, ":", (int)cursorCol);
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
        case 0x03: { // Get Cursor Position
            m_cpu.setReg8(cpu::DH, m_memory.read8(0x451));
            m_cpu.setReg8(cpu::DL, m_memory.read8(0x450));
            m_cpu.setReg16(cpu::CX, 0x0607); // Standard cursor shape
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
    if (ah == 0x00) {
        LOG_DEBUG("BIOS INT 13h: Reset Disk System");
        m_cpu.setReg8(cpu::AH, 0); // Success
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    }
}

} // namespace fador::hw
