#include "CPU.hpp"
#include "../utils/Logger.hpp"
#include "../memory/MemoryBus.hpp"

namespace fador::cpu {

CPU::CPU() {
    reset();
}

void CPU::reset() {
    m_regs.fill(0);
    // Real mode startup values
    m_segRegs.fill(0);
    m_segRegs[CS] = 0xF000;
    
    m_eip = 0xFFF0;
    m_eflags = 0x00000002;
    
    m_cr.fill(0);
    m_dr.fill(0);

    LOG_INFO("CPU Reset: CS:EIP = 0x", std::hex, m_segRegs[CS], ":0x", m_eip);
}

uint32_t CPU::getReg32(uint8_t index) const {
    return m_regs[index & 7];
}

void CPU::setReg32(uint8_t index, uint32_t value) {
    m_regs[index & 7] = value;
}

uint16_t CPU::getReg16(uint8_t index) const {
    return static_cast<uint16_t>(m_regs[index & 7] & 0xFFFF);
}

void CPU::setReg16(uint8_t index, uint16_t value) {
    uint32_t& reg = m_regs[index & 7];
    reg = (reg & 0xFFFF0000) | value;
}

uint8_t CPU::getReg8(uint8_t index) const {
    uint8_t regIndex = index & 3; // 0=AL/AH...3=BL/BH
    bool highByte = (index & 4) != 0;
    
    if (highByte) {
        return static_cast<uint8_t>((m_regs[regIndex] >> 8) & 0xFF);
    } else {
        return static_cast<uint8_t>(m_regs[regIndex] & 0xFF);
    }
}

void CPU::setReg8(uint8_t index, uint8_t value) {
    uint8_t regIndex = index & 3;
    bool highByte = (index & 4) != 0;
    
    uint32_t& reg = m_regs[regIndex];
    if (highByte) {
        reg = (reg & 0xFFFF00FF) | (static_cast<uint32_t>(value) << 8);
    } else {
        reg = (reg & 0xFFFFFF00) | value;
    }
}

void CPU::push16(uint16_t value) {
    uint16_t sp = getReg16(SP) - 2;
    setReg16(SP, sp);
    if (m_memory) {
        m_memory->write16((static_cast<uint32_t>(m_segRegs[SS]) << 4) + sp, value);
    }
}

void CPU::push32(uint32_t value) {
    uint32_t esp = getReg32(ESP) - 4;
    setReg32(ESP, esp);
    if (m_memory) {
        m_memory->write32((static_cast<uint32_t>(m_segRegs[SS]) << 4) + esp, value);
    }
}

uint16_t CPU::pop16() {
    uint16_t sp = getReg16(SP);
    uint16_t value = 0;
    if (m_memory) {
        value = m_memory->read16((static_cast<uint32_t>(m_segRegs[SS]) << 4) + sp);
    }
    setReg16(SP, sp + 2);
    return value;
}

uint32_t CPU::pop32() {
    uint32_t esp = getReg32(ESP);
    uint32_t value = 0;
    if (m_memory) {
        value = m_memory->read32((static_cast<uint32_t>(m_segRegs[SS]) << 4) + esp);
    }
    setReg32(ESP, esp + 4);
    return value;
}

} // namespace fador::cpu
