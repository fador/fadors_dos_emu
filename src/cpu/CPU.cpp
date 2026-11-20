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

CPU::CPU(CpuModelId model) {
  setModel(model);
  reset();
}

void CPU::setModel(CpuModelId id) {
  m_model = &cpuModel(id);
  m_features = m_model->features;
  buildCycleTable(*m_model, m_cycleTable);
  const auto cost = [&](CycleClass c) {
    return static_cast<uint32_t>(m_model->cycles[static_cast<size_t>(c)]);
  };
  m_memOperandCycles = m_model->memOperandCycles;
  // MUL/DIV share opcodes F6/F7 with TEST/NOT/NEG, which the table charges
  // as ALU operations; the handlers add the difference.
  m_mulExtraCycles = cost(CycleClass::Mul) - cost(CycleClass::Alu);
  m_divExtraCycles = cost(CycleClass::Div) - cost(CycleClass::Alu);
  ++m_segmentStateVersion; // decoders re-read the cycle table
  m_eflags = (m_eflags & m_model->eflagsMask) | 0x2u;
  setCR(0, (m_cr[0] & (m_model->cr0Mask | m_model->cr0Fixed)) |
               m_model->cr0Fixed);
  setCR(4, m_cr[4] & m_model->cr4Mask);
}

uint64_t CPU::readTSC() {
  uint64_t tsc = (m_tscSource ? m_tscSource() : m_guestCycles) + m_tscOffset;
  // Two reads within one tick of the source still differ (software divides
  // by TSC deltas); the TSC never runs backwards.
  if (m_tscRead && tsc <= m_lastTsc)
    tsc = m_lastTsc + 1;
  m_lastTsc = tsc;
  m_tscRead = true;
  return tsc;
}

void CPU::writeTSC(uint64_t value) {
  m_tscOffset = value - (m_tscSource ? m_tscSource() : m_guestCycles);
  m_tscRead = false;
}

void CPU::setTimeStampSource(std::function<uint64_t()> source) {
  const uint64_t current =
      (m_tscSource ? m_tscSource() : m_guestCycles) + m_tscOffset;
  m_tscSource = std::move(source);
  writeTSC(current);
}

void CPU::reset() {
  m_regs.fill(0);
  // EDX holds the component/revision signature after reset.
  m_regs[EDX] = m_model->signature;
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
  // ET: a 387-compatible FPU is present (hardwired on the 486 and later).
  m_cr[0] = 0x00000010u;
  m_nativeProtection = false;
  // The time-stamp counter restarts at 0.
  m_tscOffset = 0 - (m_tscSource ? m_tscSource() : m_guestCycles);
  m_tscRead = false;
  if (m_memory)
    m_memory->setPaging(false, 0, false);
  m_dr.fill(0);
  resetFPU();

  // Initialise segment bases for Real Mode
  for (int i = 0; i < 6; i++) {
    m_segBase[i] = static_cast<uint32_t>(m_segRegs[i]) << 4;
  }
  ++m_segmentStateVersion;
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
