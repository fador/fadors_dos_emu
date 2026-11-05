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

uint8_t classifyFPUTag(double value) {
  if (std::isnan(value) || std::isinf(value)) {
    return FPU_TAG_SPECIAL;
  }
  if (value == 0.0) {
    return FPU_TAG_ZERO;
  }
  return FPU_TAG_VALID;
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

void CPU::resetFPU() {
  m_fpuRegs.fill(0.0);
  m_fpuTags.fill(FPU_TAG_EMPTY);
  m_fpuControlWord = FPU_CONTROL_DEFAULT;
  m_fpuStatusWord = 0;
  m_fpuInstructionPointer = 0;
  m_fpuInstructionSelector = 0;
  m_fpuLastOpcode = 0;
  m_fpuDataPointer = 0;
  m_fpuDataSelector = 0;
}

uint16_t CPU::getFPUTagWord() const {
  uint16_t tagWord = 0;
  for (int index = 0; index < 8; ++index) {
    tagWord |= static_cast<uint16_t>((m_fpuTags[index] & 0x3) << (index * 2));
  }
  return tagWord;
}

void CPU::setFPUTagWord(uint16_t val) {
  for (int index = 0; index < 8; ++index) {
    m_fpuTags[index] = static_cast<uint8_t>((val >> (index * 2)) & 0x3);
  }
}

bool CPU::isFPURegisterEmpty(uint8_t logicalIndex) const {
  return m_fpuTags[fpuPhysicalIndex(logicalIndex)] == FPU_TAG_EMPTY;
}

double CPU::getFPURegister(uint8_t logicalIndex) const {
  return m_fpuRegs[fpuPhysicalIndex(logicalIndex)];
}

void CPU::setFPURegister(uint8_t logicalIndex, double value) {
  const uint8_t physicalIndex = fpuPhysicalIndex(logicalIndex);
  m_fpuRegs[physicalIndex] = value;
  m_fpuTags[physicalIndex] = classifyFPUTag(value);
}

void CPU::freeFPURegister(uint8_t logicalIndex) {
  const uint8_t physicalIndex = fpuPhysicalIndex(logicalIndex);
  m_fpuRegs[physicalIndex] = 0.0;
  m_fpuTags[physicalIndex] = FPU_TAG_EMPTY;
}

bool CPU::pushFPU(double value) {
  const uint8_t nextTop = static_cast<uint8_t>((getFPUTop() - 1) & 0x7);
  if (m_fpuTags[nextTop] != FPU_TAG_EMPTY) {
    m_fpuStatusWord = static_cast<uint16_t>(m_fpuStatusWord |
                                            FPU_STATUS_IE |
                                            FPU_STATUS_SF |
                                            FPU_STATUS_C1);
    return false;
  }

  setFPUTop(nextTop);
  m_fpuRegs[nextTop] = value;
  m_fpuTags[nextTop] = classifyFPUTag(value);
  return true;
}

bool CPU::popFPU() {
  const uint8_t top = getFPUTop();
  if (m_fpuTags[top] == FPU_TAG_EMPTY) {
    m_fpuStatusWord = static_cast<uint16_t>((m_fpuStatusWord |
                                             FPU_STATUS_IE |
                                             FPU_STATUS_SF) &
                                            ~FPU_STATUS_C1);
    return false;
  }

  m_fpuRegs[top] = 0.0;
  m_fpuTags[top] = FPU_TAG_EMPTY;
  setFPUTop(static_cast<uint8_t>((top + 1) & 0x7));
  return true;
}

} // namespace fador::cpu
