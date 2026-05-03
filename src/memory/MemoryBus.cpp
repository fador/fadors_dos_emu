#include "MemoryBus.hpp"
#include "../hw/VGAController.hpp"
#include <algorithm>
#include <iostream>

namespace fador::memory {

namespace {

bool intersectsRange(uint32_t start, uint32_t size, uint32_t rangeStart,
                     uint32_t rangeEnd) {
  if (size == 0) {
    return false;
  }

  return start < rangeEnd &&
         static_cast<uint64_t>(start) + static_cast<uint64_t>(size) >
             rangeStart;
}

} // namespace

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

uint8_t MemoryBus::read8Slow(uint32_t address, uint32_t effectiveAddress) const {
  if (effectiveAddress < MEMORY_SIZE) {
    if (needsPlaneRead8(effectiveAddress)) {
      return m_vga->planeRead8(effectiveAddress - VGA_PLANAR_START);
    }
    return m_ram[effectiveAddress];
  }
  LOG_WARN("Memory out of bounds READ 8-bit at: 0x", std::hex, address,
           " (Effective: 0x", effectiveAddress, ")");
  return 0xFF;
}

uint16_t MemoryBus::read16Slow(uint32_t address,
                               uint32_t effectiveAddress) const {
  if (effectiveAddress + 1 < MEMORY_SIZE) {
    return m_ram[effectiveAddress] | (m_ram[effectiveAddress + 1] << 8);
  }
  LOG_WARN("Memory out of bounds READ 16-bit at: 0x", std::hex, address,
           " (Effective: 0x", effectiveAddress, ")");
  return 0xFFFF;
}

uint32_t MemoryBus::read32Slow(uint32_t address,
                               uint32_t effectiveAddress) const {
  if (effectiveAddress + 3 < MEMORY_SIZE) {
    return m_ram[effectiveAddress] | (m_ram[effectiveAddress + 1] << 8) |
           (m_ram[effectiveAddress + 2] << 16) |
           (m_ram[effectiveAddress + 3] << 24);
  }
  LOG_WARN("Memory out of bounds READ 32-bit at: 0x", std::hex, address,
           " (Effective: 0x", effectiveAddress, ")");
  return 0xFFFFFFFF;
}

void MemoryBus::write8Slow(uint32_t address, uint32_t effectiveAddress,
                           uint8_t value) {
  if (effectiveAddress < MEMORY_SIZE) {
    if (effectiveAddress < 0x400) {
      LOG_DEBUG("IVT WRITE8 at 0x", std::hex, effectiveAddress, " val: 0x",
                static_cast<int>(value));
    }
    m_ram[effectiveAddress] = value;
    if (needsPlaneWrite8(effectiveAddress)) {
      m_vga->planeWrite8(effectiveAddress - VGA_PLANAR_START, value);
    }
    if (needsTextVramLog(effectiveAddress)) {
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

void MemoryBus::write16Slow(uint32_t address, uint32_t effectiveAddress,
                            uint16_t value) {
  if (effectiveAddress + 1 < MEMORY_SIZE) {
    if (effectiveAddress < 0x400) {
      LOG_DEBUG("IVT WRITE16 at 0x", std::hex, effectiveAddress, " val: 0x",
                static_cast<int>(value));
    }
    m_ram[effectiveAddress] = value & 0xFF;
    m_ram[effectiveAddress + 1] = (value >> 8) & 0xFF;
    if (needsPlaneWrite16(effectiveAddress)) {
      m_vga->planeWrite8(effectiveAddress - VGA_PLANAR_START, value & 0xFF);
      m_vga->planeWrite8(effectiveAddress - VGA_PLANAR_START + 1,
                         (value >> 8) & 0xFF);
    }
    if (needsTextVramLog(effectiveAddress)) {
      uint8_t c1 = value & 0xFF;
      LOG_VIDEO("VRAM WRITE16 at 0x", std::hex, effectiveAddress, " val: 0x",
                value, " ('", (char)(c1 >= 32 && c1 < 127 ? c1 : '.'), "')");
    }
  } else {
    LOG_WARN("Memory out of bounds WRITE 16-bit at: 0x", std::hex, address,
             " (Effective: 0x", effectiveAddress, ") val: 0x", value);
  }
}

void MemoryBus::write32Slow(uint32_t address, uint32_t effectiveAddress,
                            uint32_t value) {
  if (effectiveAddress + 3 < MEMORY_SIZE) {
    m_ram[effectiveAddress] = value & 0xFF;
    m_ram[effectiveAddress + 1] = (value >> 8) & 0xFF;
    m_ram[effectiveAddress + 2] = (value >> 16) & 0xFF;
    m_ram[effectiveAddress + 3] = (value >> 24) & 0xFF;
    if (needsPlaneWrite32(effectiveAddress)) {
      m_vga->planeWrite8(effectiveAddress - VGA_PLANAR_START, value & 0xFF);
      m_vga->planeWrite8(effectiveAddress - VGA_PLANAR_START + 1,
                         (value >> 8) & 0xFF);
      m_vga->planeWrite8(effectiveAddress - VGA_PLANAR_START + 2,
                         (value >> 16) & 0xFF);
      m_vga->planeWrite8(effectiveAddress - VGA_PLANAR_START + 3,
                         (value >> 24) & 0xFF);
    }
    if (needsTextVramLog(effectiveAddress)) {
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

uint8_t *MemoryBus::contiguousAccess(uint32_t address, uint32_t size) {
  uint32_t effectiveAddress = m_a20Enabled ? address : (address & A20_MASK);

  if (size == 0) {
    return effectiveAddress < MEMORY_SIZE ? &m_ram[effectiveAddress] : nullptr;
  }

  if (!m_a20Enabled && size - 1 > A20_MASK - effectiveAddress) {
    return nullptr;
  }

  if (effectiveAddress >= MEMORY_SIZE || size > MEMORY_SIZE - effectiveAddress) {
    return nullptr;
  }

  if (m_vga &&
      intersectsRange(effectiveAddress, size, VGA_PLANAR_START, VGA_PLANAR_END)) {
    return nullptr;
  }

  return &m_ram[effectiveAddress];
}

const uint8_t *MemoryBus::contiguousAccess(uint32_t address,
                                           uint32_t size) const {
  return const_cast<MemoryBus *>(this)->contiguousAccess(address, size);
}

} // namespace fador::memory
