#include "MemoryBus.hpp"
#include <algorithm>
#include <iostream>

namespace fador::memory {

MemoryBus::MemoryBus() {
  m_ram.resize(MEMORY_SIZE, 0);

  // Fill conventional memory above BDA (0x500–0x9FFFF) with a non-zero
  // pattern that includes periodic zero bytes.  On real hardware this
  // region contains residue from COMMAND.COM, device drivers, prior
  // programs, etc. — a mix of non-zero data with natural zero bytes
  // interspersed.
  //
  // Pure zero fill is wrong: programs that read uninitialised far-heap
  // allocations (e.g. Turbo C 2.01's Turbo Vision) misinterpret zeroes
  // as end-of-table sentinels before any table entries exist.
  //
  // Pure 0xCC fill is also wrong: it contains no null terminators, so
  // string copies from uninitialised heap run off the end of buffers,
  // causing stack corruption (buffer overflow into saved-BP).
  //
  // A deterministic pseudo-random fill (full-period LCG mod 256) gives
  // a realistic mix: ~255 non-zero bytes between each naturally
  // occurring zero, long enough to avoid false end-of-table matches
  // within any single table entry, but short enough to terminate
  // runaway string scans within a reasonable distance.
  {
    uint8_t val = 0xCC; // seed
    for (size_t i = 0x500; i < 0xA0000; ++i) {
      m_ram[i] = val;
      val = static_cast<uint8_t>(val * 141 + 3);
    }
  }

  LOG_INFO("MemoryBus initialized with ", MEMORY_SIZE, " bytes");
}

uint8_t MemoryBus::read8(uint32_t address) const {
  uint32_t effectiveAddress = m_a20Enabled ? address : (address & 0xFFFFF);
  if (effectiveAddress < MEMORY_SIZE) {
    return m_ram[effectiveAddress];
  }
  LOG_WARN("Memory out of bounds READ 8-bit at: 0x", std::hex, address,
           " (Effective: 0x", effectiveAddress, ")");
  return 0xFF;
}

uint16_t MemoryBus::read16(uint32_t address) const {
  uint32_t effectiveAddress = m_a20Enabled ? address : (address & 0xFFFFF);
  if (effectiveAddress + 1 < MEMORY_SIZE) {
    return m_ram[effectiveAddress] | (m_ram[effectiveAddress + 1] << 8);
  }
  LOG_WARN("Memory out of bounds READ 16-bit at: 0x", std::hex, address,
           " (Effective: 0x", effectiveAddress, ")");
  return 0xFFFF;
}

uint32_t MemoryBus::read32(uint32_t address) const {
  uint32_t effectiveAddress = m_a20Enabled ? address : (address & 0xFFFFF);
  if (effectiveAddress + 3 < MEMORY_SIZE) {
    return m_ram[effectiveAddress] | (m_ram[effectiveAddress + 1] << 8) |
           (m_ram[effectiveAddress + 2] << 16) |
           (m_ram[effectiveAddress + 3] << 24);
  }
  LOG_WARN("Memory out of bounds READ 32-bit at: 0x", std::hex, address,
           " (Effective: 0x", effectiveAddress, ")");
  return 0xFFFFFFFF;
}

void MemoryBus::write8(uint32_t address, uint8_t value) {
  uint32_t effectiveAddress = m_a20Enabled ? address : (address & 0xFFFFF);
  if (effectiveAddress < MEMORY_SIZE) {
    if (effectiveAddress < 0x400) {
      LOG_DEBUG("IVT WRITE8 at 0x", std::hex, effectiveAddress, " val: 0x",
                static_cast<int>(value));
    }
    m_ram[effectiveAddress] = value;
    // Simple hook for CGA/VGA text mode (0xB8000 - 0xBFFFF)
    if (effectiveAddress >= 0xB8000 && effectiveAddress < 0xBFFFF) {
      // Temporarily escalate VRAM write logs to track down rendering bug
      LOG_VIDEO("VRAM WRITE at 0x", std::hex, effectiveAddress, " val: 0x",
                static_cast<int>(value), " ('",
                (char)(value >= 32 && value < 127 ? value : '.'), "')");
    }
  } else {
    LOG_WARN("Memory out of bounds WRITE 8-bit at: 0x", std::hex, address,
             " (Effective: 0x", effectiveAddress, ") val: 0x",
             static_cast<int>(value));
  }
}

void MemoryBus::write16(uint32_t address, uint16_t value) {
  uint32_t effectiveAddress = m_a20Enabled ? address : (address & 0xFFFFF);
  if (effectiveAddress + 1 < MEMORY_SIZE) {
    if (effectiveAddress < 0x400) {
      LOG_DEBUG("IVT WRITE16 at 0x", std::hex, effectiveAddress, " val: 0x",
                static_cast<int>(value));
    }
    m_ram[effectiveAddress] = value & 0xFF;
    m_ram[effectiveAddress + 1] = (value >> 8) & 0xFF;
    if (effectiveAddress >= 0xB8000 && effectiveAddress < 0xBFFFF) {
      uint8_t c1 = value & 0xFF;
      LOG_VIDEO("VRAM WRITE16 at 0x", std::hex, effectiveAddress, " val: 0x",
                value, " ('", (char)(c1 >= 32 && c1 < 127 ? c1 : '.'), "')");
    }
  } else {
    LOG_WARN("Memory out of bounds WRITE 16-bit at: 0x", std::hex, address,
             " (Effective: 0x", effectiveAddress, ") val: 0x", value);
  }
}

void MemoryBus::write32(uint32_t address, uint32_t value) {
  uint32_t effectiveAddress = m_a20Enabled ? address : (address & 0xFFFFF);
  if (effectiveAddress + 3 < MEMORY_SIZE) {
    m_ram[effectiveAddress] = value & 0xFF;
    m_ram[effectiveAddress + 1] = (value >> 8) & 0xFF;
    m_ram[effectiveAddress + 2] = (value >> 16) & 0xFF;
    m_ram[effectiveAddress + 3] = (value >> 24) & 0xFF;
    if (effectiveAddress >= 0xB8000 && effectiveAddress < 0xBFFFF) {
      uint8_t c1 = value & 0xFF;
      uint8_t c2 = (value >> 16) & 0xFF;
      LOG_VIDEO("VRAM WRITE32 at 0x", std::hex, effectiveAddress, " val: 0x",
                value, " ('", (char)(c1 >= 32 && c1 < 127 ? c1 : '.'), "' '",
                (char)(c2 >= 32 && c2 < 127 ? c2 : '.'), "')");
    }
  } else {
    LOG_WARN("Memory out of bounds WRITE 32-bit at: 0x", std::hex, address,
             " (Effective: 0x", effectiveAddress, ") val: 0x", value);
  }
}

uint8_t *MemoryBus::directAccess(uint32_t address) {
  if (address < MEMORY_SIZE) {
    return &m_ram[address];
  }
  return nullptr;
}

} // namespace fador::memory
