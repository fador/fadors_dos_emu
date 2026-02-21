#include "MemoryBus.hpp"
#include <iostream>
#include <iomanip>

namespace fador::memory {

MemoryBus::MemoryBus() {
    m_ram.resize(MEMORY_SIZE, 0);
    LOG_INFO("MemoryBus initialized with ", MEMORY_SIZE, " bytes");
}

uint8_t MemoryBus::read8(uint32_t address) const {
    uint32_t effectiveAddress = m_a20Enabled ? address : (address & 0xFFFFF);
    if (effectiveAddress < MEMORY_SIZE) {
        return m_ram[effectiveAddress];
    }
    LOG_WARN("Memory out of bounds READ 8-bit at: 0x", std::hex, address, " (Effective: 0x", effectiveAddress, ")");
    return 0xFF;
}

uint16_t MemoryBus::read16(uint32_t address) const {
    uint32_t effectiveAddress = m_a20Enabled ? address : (address & 0xFFFFF);
    if (effectiveAddress + 1 < MEMORY_SIZE) {
        return m_ram[effectiveAddress] | (m_ram[effectiveAddress + 1] << 8);
    }
    LOG_WARN("Memory out of bounds READ 16-bit at: 0x", std::hex, address, " (Effective: 0x", effectiveAddress, ")");
    return 0xFFFF;
}

uint32_t MemoryBus::read32(uint32_t address) const {
    uint32_t effectiveAddress = m_a20Enabled ? address : (address & 0xFFFFF);
    if (effectiveAddress + 3 < MEMORY_SIZE) {
        return m_ram[effectiveAddress] |
               (m_ram[effectiveAddress + 1] << 8) |
               (m_ram[effectiveAddress + 2] << 16) |
               (m_ram[effectiveAddress + 3] << 24);
    }
    LOG_WARN("Memory out of bounds READ 32-bit at: 0x", std::hex, address, " (Effective: 0x", effectiveAddress, ")");
    return 0xFFFFFFFF;
}

void MemoryBus::write8(uint32_t address, uint8_t value) {
    uint32_t effectiveAddress = m_a20Enabled ? address : (address & 0xFFFFF);
    if (effectiveAddress < MEMORY_SIZE) {
        m_ram[effectiveAddress] = value;
        // Simple hook for CGA/VGA text mode (0xB8000 - 0xBFFFF)
        if (effectiveAddress >= 0xB8000 && effectiveAddress < 0xC0000) {
            // Emulation graphical backend will pick this up
        }
    } else {
        LOG_WARN("Memory out of bounds WRITE 8-bit at: 0x", std::hex, address, " (Effective: 0x", effectiveAddress, ") val: 0x", static_cast<int>(value));
    }
}

void MemoryBus::write16(uint32_t address, uint16_t value) {
    uint32_t effectiveAddress = m_a20Enabled ? address : (address & 0xFFFFF);
    if (effectiveAddress + 1 < MEMORY_SIZE) {
        m_ram[effectiveAddress] = value & 0xFF;
        m_ram[effectiveAddress + 1] = (value >> 8) & 0xFF;
    } else {
        LOG_WARN("Memory out of bounds WRITE 16-bit at: 0x", std::hex, address, " (Effective: 0x", effectiveAddress, ") val: 0x", value);
    }
}

void MemoryBus::write32(uint32_t address, uint32_t value) {
    uint32_t effectiveAddress = m_a20Enabled ? address : (address & 0xFFFFF);
    if (effectiveAddress + 3 < MEMORY_SIZE) {
        m_ram[effectiveAddress] = value & 0xFF;
        m_ram[effectiveAddress + 1] = (value >> 8) & 0xFF;
        m_ram[effectiveAddress + 2] = (value >> 16) & 0xFF;
        m_ram[effectiveAddress + 3] = (value >> 24) & 0xFF;
    } else {
        LOG_WARN("Memory out of bounds WRITE 32-bit at: 0x", std::hex, address, " (Effective: 0x", effectiveAddress, ") val: 0x", value);
    }
}

uint8_t* MemoryBus::directAccess(uint32_t address) {
    uint32_t effectiveAddress = m_a20Enabled ? address : (address & 0xFFFFF);
    if (effectiveAddress < MEMORY_SIZE) {
        return &m_ram[effectiveAddress];
    }
    return nullptr;
}

} // namespace fador::memory
