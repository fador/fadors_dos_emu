#include "MemoryBus.hpp"
#include "../hw/VGAController.hpp"
#include <algorithm>
#include <iostream>
#include <new>

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
  m_ram.reset(static_cast<uint8_t *>(std::calloc(MEMORY_SIZE, 1)));
  if (!m_ram)
    throw std::bad_alloc();

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

// Slow paths: handled byte by byte so accesses that straddle the VGA
// window or the A20 wrap point are routed correctly for every byte.
uint8_t MemoryBus::read8Slow(uint32_t address, uint32_t effectiveAddress) const {
  if (effectiveAddress < MEMORY_SIZE) {
    if (needsPlaneRead8(effectiveAddress))
      return m_vga->planeRead8(effectiveAddress - VGA_PLANAR_START);
    return m_ram[effectiveAddress];
  }
  reportOutOfBounds(address, false);
  LOG_WARN("Memory out of bounds READ 8-bit at: 0x", std::hex, address,
           " (Effective: 0x", effectiveAddress, ")");
  return 0xFF;
}

uint16_t MemoryBus::read16Slow(uint32_t address,
                               uint32_t effectiveAddress) const {
  (void)effectiveAddress;
  return static_cast<uint16_t>(read8(address) | (read8(address + 1) << 8));
}

uint32_t MemoryBus::read32Slow(uint32_t address,
                               uint32_t effectiveAddress) const {
  (void)effectiveAddress;
  return static_cast<uint32_t>(read16(address)) |
         (static_cast<uint32_t>(read16(address + 2)) << 16);
}

void MemoryBus::write8Slow(uint32_t address, uint32_t effectiveAddress,
                           uint8_t value) {
  if (effectiveAddress < MEMORY_SIZE) {
    if (effectiveAddress < IVT_BDA_LIMIT) {
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
    reportOutOfBounds(address, true);
    LOG_WARN("Memory out of bounds WRITE 8-bit at: 0x", std::hex, address,
             " (Effective: 0x", effectiveAddress, ") val: 0x",
             static_cast<int>(value));
  }
}

void MemoryBus::write16Slow(uint32_t address, uint32_t effectiveAddress,
                            uint16_t value) {
  (void)effectiveAddress;
  const bool tracing = m_traceWrites;
  m_traceWrites = false; // log once per access, not per byte
  write8Slow(address, (address) & m_a20Mask, static_cast<uint8_t>(value));
  write8Slow(address + 1, (address + 1) & m_a20Mask,
             static_cast<uint8_t>(value >> 8));
  m_traceWrites = tracing;
}

void MemoryBus::write32Slow(uint32_t address, uint32_t effectiveAddress,
                            uint32_t value) {
  (void)effectiveAddress;
  for (uint32_t i = 0; i < 4; ++i)
    write8Slow(address + i, (address + i) & m_a20Mask,
               static_cast<uint8_t>(value >> (8 * i)));
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

  // With A20 masked, a range that crosses a 1 MiB boundary wraps around
  // (bit 20 is forced to zero), so it is not contiguous in host memory.
  if (!m_a20Enabled &&
      (static_cast<uint64_t>(effectiveAddress) >> 20) !=
          ((static_cast<uint64_t>(effectiveAddress) + size - 1) >> 20)) {
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

void MemoryBus::clearVGA() {
  if (m_vga)
    m_vga->clearPlanes();
}

} // namespace fador::memory
