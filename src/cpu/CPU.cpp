#include "CPU.hpp"
#include "../memory/MemoryBus.hpp"
#include "../utils/Logger.hpp"

#include <cmath>

namespace fador::cpu {

namespace {

uint32_t decodeSegmentBase(uint32_t low, uint32_t high) {
  return ((low >> 16) & 0xFFFF) | ((high & 0x000000FF) << 16) |
         (high & 0xFF000000);
}

bool decodeSegmentIs32Bit(uint32_t high) {
  return (high & 0x00400000u) != 0;
}

} // namespace

CPU::CPU() { reset(); }

void CPU::reset() {
  m_regs.fill(0);
  // Real mode startup values
  m_segRegs.fill(0);
  m_segRegs[CS] = 0xF000;
  m_is32BitCode = false;
  m_is32BitStack = false;

  m_eip = 0xFFF0;
  m_instructionStartEIP = 0xFFF0;
  m_instructionStartCS = 0xF000;
  m_prevInstructionCS = 0xF000;
  m_eflags = 0x00000002;

  m_cr.fill(0);
  m_nativeProtection = false;
  if (m_memory)
    m_memory->setPaging(false, 0, false);
  m_dr.fill(0);
  resetFPU();

  // Initialise segment bases for Real Mode
  for (int i = 0; i < 6; i++) {
    m_segBase[i] = static_cast<uint32_t>(m_segRegs[i]) << 4;
  }
  m_segmentStateVersion = 1;
  m_dirtySegmentMask = 0;
  m_interruptShadow = 0;

  LOG_INFO("CPU Reset: CS:EIP = 0x", std::hex, m_segRegs[CS], ":0x", m_eip);
}

void CPU::loadSegment(SegRegIndex segIndex, uint16_t value) {
  uint8_t seg = static_cast<uint8_t>(segIndex);
  uint8_t mask = static_cast<uint8_t>(1u << seg);
  m_segRegs[seg] = value;

  bool isRealMode = !(m_cr[0] & 1) || (m_eflags & 0x00020000);
  if (isRealMode) {
    m_segBase[seg] = static_cast<uint32_t>(value) << 4;
    if (seg == CS) {
      m_is32BitCode = false;
    }
    if (seg == SS) {
      m_is32BitStack = false;
    }
  } else if (value == 0 && seg != CS && seg != SS) {
    m_segBase[seg] = 0;
  } else if (!m_memory) {
    m_segBase[seg] = static_cast<uint32_t>(value) << 4;
    if (seg == CS) {
      m_is32BitCode = false;
    }
    if (seg == SS) {
      m_is32BitStack = false;
    }
  } else {
    const auto &table = (value & 0x04) ? m_ldtr : m_gdtr;
    uint32_t entryOffset = static_cast<uint32_t>(value & ~7u);
    if (table.base == 0 || entryOffset + 7 > table.limit) {
      m_segBase[seg] = static_cast<uint32_t>(value) << 4;
      if (seg == CS) {
        m_is32BitCode = false;
      }
      if (seg == SS) {
        m_is32BitStack = false;
      }
    } else {
      uint32_t entryAddr = table.base + entryOffset;
      uint32_t low = m_memory->lread32(entryAddr);
      uint32_t high = m_memory->lread32(entryAddr + 4);
      m_segBase[seg] = decodeSegmentBase(low, high);
      if (seg == CS) {
        m_is32BitCode = decodeSegmentIs32Bit(high);
      }
      if (seg == SS) {
        m_is32BitStack = decodeSegmentIs32Bit(high);
      }
    }
  }

  m_dirtySegmentMask = static_cast<uint8_t>(m_dirtySegmentMask & ~mask);
  ++m_segmentStateVersion;

  if (segIndex == CS && !m_is32BitCode) {
    m_eip &= 0xFFFF;
  }
}

} // namespace fador::cpu
