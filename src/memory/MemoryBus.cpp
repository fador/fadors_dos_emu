#include "MemoryBus.hpp"
#include <iostream>
#include <iomanip>

namespace fador::memory {

MemoryBus::MemoryBus() {
    m_ram.resize(MEMORY_SIZE, 0);
    LOG_INFO("MemoryBus initialized with ", MEMORY_SIZE, " bytes");
}

uint8_t MemoryBus::read8(uint32_t address) const {
    if (address < MEMORY_SIZE) {
        return m_ram[address];
    }
    LOG_WARN("Memory out of bounds READ 8-bit at: 0x", std::hex, address);
    return 0xFF;
}

uint16_t MemoryBus::read16(uint32_t address) const {
    if (address + 1 < MEMORY_SIZE) {
        return m_ram[address] | (m_ram[address + 1] << 8);
    }
    LOG_WARN("Memory out of bounds READ 16-bit at: 0x", std::hex, address);
    return 0xFFFF;
}

uint32_t MemoryBus::read32(uint32_t address) const {
    if (address + 3 < MEMORY_SIZE) {
        return m_ram[address] |
               (m_ram[address + 1] << 8) |
               (m_ram[address + 2] << 16) |
               (m_ram[address + 3] << 24);
    }
    LOG_WARN("Memory out of bounds READ 32-bit at: 0x", std::hex, address);
    return 0xFFFFFFFF;
}

void MemoryBus::write8(uint32_t address, uint8_t value) {
    if (address < MEMORY_SIZE) {
        m_ram[address] = value;
        // Simple hook for CGA/VGA text mode (0xB8000 - 0xBFFFF)
        if (address >= 0xB8000 && address < 0xC0000) {
            // Emulation graphical backend will pick this up
        }
    } else {
        LOG_WARN("Memory out of bounds WRITE 8-bit at: 0x", std::hex, address, " val: 0x", static_cast<int>(value));
    }
}

void MemoryBus::write16(uint32_t address, uint16_t value) {
    if (address + 1 < MEMORY_SIZE) {
        m_ram[address] = value & 0xFF;
        m_ram[address + 1] = (value >> 8) & 0xFF;
    } else {
        LOG_WARN("Memory out of bounds WRITE 16-bit at: 0x", std::hex, address, " val: 0x", value);
    }
}

void MemoryBus::write32(uint32_t address, uint32_t value) {
    if (address + 3 < MEMORY_SIZE) {
        m_ram[address] = value & 0xFF;
        m_ram[address + 1] = (value >> 8) & 0xFF;
        m_ram[address + 2] = (value >> 16) & 0xFF;
        m_ram[address + 3] = (value >> 24) & 0xFF;
    } else {
        LOG_WARN("Memory out of bounds WRITE 32-bit at: 0x", std::hex, address, " val: 0x", value);
    }
}

uint8_t* MemoryBus::directAccess(uint32_t address) {
    if (address < MEMORY_SIZE) {
        return &m_ram[address];
    }
    return nullptr;
}

} // namespace fador::memory
