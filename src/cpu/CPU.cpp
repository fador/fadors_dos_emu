#include "CPU.hpp"
#include "../memory/MemoryBus.hpp"
#include "../utils/Logger.hpp"

namespace fador::cpu {

CPU::CPU() { reset(); }

void CPU::reset() {
  m_regs.fill(0);
  // Real mode startup values
  m_segRegs.fill(0);
  m_segRegs[CS] = 0xF000;
  m_is32BitCode = false;
  m_is32BitStack = false;

  m_eip = 0xFFF0;
  m_eflags = 0x00000002;

  m_cr.fill(0);
  m_dr.fill(0);

  // Initialise segment bases for Real Mode
  for (int i = 0; i < 6; i++) {
    m_segBase[i] = static_cast<uint32_t>(m_segRegs[i]) << 4;
  }

  LOG_INFO("CPU Reset: CS:EIP = 0x", std::hex, m_segRegs[CS], ":0x", m_eip);
}

uint32_t CPU::getReg32(uint8_t index) const { return m_regs[index & 7]; }

void CPU::setReg32(uint8_t index, uint32_t value) { m_regs[index & 7] = value; }

uint16_t CPU::getReg16(uint8_t index) const {
  return static_cast<uint16_t>(m_regs[index & 7] & 0xFFFF);
}

void CPU::setReg16(uint8_t index, uint16_t value) {
  uint32_t &reg = m_regs[index & 7];
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

  uint32_t &reg = m_regs[regIndex];
  if (highByte) {
    reg = (reg & 0xFFFF00FF) | (static_cast<uint32_t>(value) << 8);
  } else {
    reg = (reg & 0xFFFFFF00) | value;
  }
}

void CPU::push16(uint16_t value) {
  bool use32Stack = m_is32BitStack;
  if ((m_cr[0] & 1) && !(m_eflags & 0x00020000) && m_memory) {
    uint16_t ss = m_segRegs[SS];
    if (ss != 0) {
      const auto &table = (ss & 0x04) ? m_ldtr : m_gdtr;
      uint32_t index = static_cast<uint32_t>(ss & ~7);
      if (table.base != 0 && index + 7 <= table.limit) {
        uint32_t entry = table.base + index;
        uint32_t high = m_memory->read32(entry + 4);
        bool present = (high & 0x00008000u) != 0;
        bool codeOrData = (high & 0x00001000u) != 0;
        if (present && codeOrData)
          use32Stack = (high & 0x00400000u) != 0;
        else
          use32Stack = false;
      } else {
        use32Stack = false;
      }
    } else {
      use32Stack = false;
    }
  }
  if (use32Stack) {
    uint32_t esp = getReg32(ESP) - 2;
    setReg32(ESP, esp);
    if (m_memory)
      m_memory->write16(m_segBase[SS] + esp, value);
  } else {
    uint16_t sp = getReg16(SP) - 2;
    setReg16(SP, sp);
    if (m_memory)
      m_memory->write16(m_segBase[SS] + sp, value);
  }
}

void CPU::push32(uint32_t value) {
  bool use32Stack = m_is32BitStack;
  if ((m_cr[0] & 1) && !(m_eflags & 0x00020000) && m_memory) {
    uint16_t ss = m_segRegs[SS];
    if (ss != 0) {
      const auto &table = (ss & 0x04) ? m_ldtr : m_gdtr;
      uint32_t index = static_cast<uint32_t>(ss & ~7);
      if (table.base != 0 && index + 7 <= table.limit) {
        uint32_t entry = table.base + index;
        uint32_t high = m_memory->read32(entry + 4);
        bool present = (high & 0x00008000u) != 0;
        bool codeOrData = (high & 0x00001000u) != 0;
        if (present && codeOrData)
          use32Stack = (high & 0x00400000u) != 0;
        else
          use32Stack = false;
      } else {
        use32Stack = false;
      }
    } else {
      use32Stack = false;
    }
  }
  if (use32Stack) {
    uint32_t esp = getReg32(ESP) - 4;
    setReg32(ESP, esp);
    if (m_memory) {
      m_memory->write32(m_segBase[SS] + esp, value);
    }
  } else {
    uint16_t sp = getReg16(SP) - 4;
    setReg16(SP, sp);
    if (m_memory) {
      m_memory->write32(m_segBase[SS] + sp, value);
    }
  }
}

uint16_t CPU::pop16() {
  uint16_t value = 0;
  bool use32Stack = m_is32BitStack;
  if ((m_cr[0] & 1) && !(m_eflags & 0x00020000) && m_memory) {
    uint16_t ss = m_segRegs[SS];
    if (ss != 0) {
      const auto &table = (ss & 0x04) ? m_ldtr : m_gdtr;
      uint32_t index = static_cast<uint32_t>(ss & ~7);
      if (table.base != 0 && index + 7 <= table.limit) {
        uint32_t entry = table.base + index;
        uint32_t high = m_memory->read32(entry + 4);
        bool present = (high & 0x00008000u) != 0;
        bool codeOrData = (high & 0x00001000u) != 0;
        if (present && codeOrData)
          use32Stack = (high & 0x00400000u) != 0;
        else
          use32Stack = false;
      } else {
        use32Stack = false;
      }
    } else {
      use32Stack = false;
    }
  }
  if (use32Stack) {
    uint32_t esp = getReg32(ESP);
    if (m_memory)
      value = m_memory->read16(m_segBase[SS] + esp);
    setReg32(ESP, esp + 2);
  } else {
    uint16_t sp = getReg16(SP);
    if (m_memory)
      value = m_memory->read16(m_segBase[SS] + sp);
    setReg16(SP, sp + 2);
  }
  return value;
}

uint32_t CPU::pop32() {
  uint32_t value = 0;
  bool use32Stack = m_is32BitStack;
  if ((m_cr[0] & 1) && !(m_eflags & 0x00020000) && m_memory) {
    uint16_t ss = m_segRegs[SS];
    if (ss != 0) {
      const auto &table = (ss & 0x04) ? m_ldtr : m_gdtr;
      uint32_t index = static_cast<uint32_t>(ss & ~7);
      if (table.base != 0 && index + 7 <= table.limit) {
        uint32_t entry = table.base + index;
        uint32_t high = m_memory->read32(entry + 4);
        bool present = (high & 0x00008000u) != 0;
        bool codeOrData = (high & 0x00001000u) != 0;
        if (present && codeOrData)
          use32Stack = (high & 0x00400000u) != 0;
        else
          use32Stack = false;
      } else {
        use32Stack = false;
      }
    } else {
      use32Stack = false;
    }
  }
  if (use32Stack) {
    uint32_t esp = getReg32(ESP);
    if (m_memory)
      value = m_memory->read32(m_segBase[SS] + esp);
    setReg32(ESP, esp + 4);
  } else {
    uint16_t sp = getReg16(SP);
    if (m_memory)
      value = m_memory->read32(m_segBase[SS] + sp);
    setReg16(SP, sp + 4);
  }
  return value;
}

} // namespace fador::cpu
