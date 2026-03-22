#include "MemoryBus.hpp"
#include <algorithm>
#include <execinfo.h>
#include <iostream>
#include <memory>

namespace fador::memory {

namespace {
constexpr uint32_t kWatchAddrStart = 0x75E;
constexpr uint32_t kWatchAddrEnd = 0x761;

bool overlapsWatchAddr(uint32_t base, uint32_t width) {
  if (width == 0)
    return false;
  uint32_t end = base + width - 1;
  return base <= kWatchAddrEnd && end >= kWatchAddrStart;
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

uint8_t MemoryBus::read8(uint32_t address) const {
  uint32_t effectiveAddress = m_a20Enabled ? address : (address & 0xFFFFF);
  if (effectiveAddress < MEMORY_SIZE) {
    if (overlapsWatchAddr(effectiveAddress, 1)) {
      static int watchLogs = 0;
      if (watchLogs < 2000) {
        ++watchLogs;
        void *caller = __builtin_return_address(0);
        LOG_ERROR("MEM ABS WATCH read8 at 0x", std::hex, effectiveAddress,
                  " val: 0x", static_cast<uint32_t>(m_ram[effectiveAddress]),
                  " caller=0x", reinterpret_cast<uintptr_t>(caller));
      }
    }
    return m_ram[effectiveAddress];
  }
  LOG_WARN("Memory out of bounds READ 8-bit at: 0x", std::hex, address,
           " (Effective: 0x", effectiveAddress, ")");
  return 0xFF;
}

uint16_t MemoryBus::read16(uint32_t address) const {
  uint32_t effectiveAddress = m_a20Enabled ? address : (address & 0xFFFFF);
  if (effectiveAddress + 1 < MEMORY_SIZE) {
    if (overlapsWatchAddr(effectiveAddress, 2)) {
      static int watchLogs = 0;
      if (watchLogs < 2000) {
        ++watchLogs;
        uint16_t value = m_ram[effectiveAddress] | (m_ram[effectiveAddress + 1] << 8);
        void *caller = __builtin_return_address(0);
        LOG_ERROR("MEM ABS WATCH read16 at 0x", std::hex, effectiveAddress,
                  " val: 0x", static_cast<uint32_t>(value),
                  " caller=0x", reinterpret_cast<uintptr_t>(caller));
      }
    }
    return m_ram[effectiveAddress] | (m_ram[effectiveAddress + 1] << 8);
  }
  LOG_WARN("Memory out of bounds READ 16-bit at: 0x", std::hex, address,
           " (Effective: 0x", effectiveAddress, ")");
  return 0xFFFF;
}

uint32_t MemoryBus::read32(uint32_t address) const {
  uint32_t effectiveAddress = m_a20Enabled ? address : (address & 0xFFFFF);
  if (effectiveAddress + 3 < MEMORY_SIZE) {
    if (overlapsWatchAddr(effectiveAddress, 4)) {
      static int watchLogs = 0;
      if (watchLogs < 2000) {
        ++watchLogs;
        uint32_t value = m_ram[effectiveAddress] | (m_ram[effectiveAddress + 1] << 8) |
                         (m_ram[effectiveAddress + 2] << 16) |
                         (m_ram[effectiveAddress + 3] << 24);
        void *caller = __builtin_return_address(0);
        LOG_ERROR("MEM ABS WATCH read32 at 0x", std::hex, effectiveAddress,
                  " val: 0x", value,
                  " caller=0x", reinterpret_cast<uintptr_t>(caller));
      }
    }
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
    if (overlapsWatchAddr(effectiveAddress, 1)) {
      void *caller = __builtin_return_address(0);
      LOG_ERROR("MEM ABS WATCH write8 at 0x", std::hex, effectiveAddress,
                " val: 0x", static_cast<uint32_t>(value),
                " caller=0x", reinterpret_cast<uintptr_t>(caller));
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
    if (overlapsWatchAddr(effectiveAddress, 2)) {
      void *caller = __builtin_return_address(0);
      LOG_ERROR("MEM ABS WATCH write16 at 0x", std::hex, effectiveAddress,
                " val: 0x", static_cast<uint32_t>(value),
                " caller=0x", reinterpret_cast<uintptr_t>(caller));
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
    if (overlapsWatchAddr(effectiveAddress, 4)) {
      void *caller = __builtin_return_address(0);
      LOG_ERROR("MEM ABS WATCH write32 at 0x", std::hex, effectiveAddress,
                " val: 0x", value,
                " caller=0x", reinterpret_cast<uintptr_t>(caller));

      // One-shot symbolic backtrace for the repeating IDT-like clobber pattern.
      if (effectiveAddress == 0x75E &&
          (value == 0x016F0024 || value == 0x70700000)) {
        static int btCount = 0;
        if (btCount < 6) {
          ++btCount;
          void *frames[12] = {};
          int n = ::backtrace(frames, 12);
          std::unique_ptr<char *, decltype(&std::free)> symbols(
              ::backtrace_symbols(frames, n), &std::free);
          if (symbols) {
            for (int i = 0; i < n; ++i) {
              LOG_ERROR("MEM ABS WATCH bt[", i, "] ", symbols.get()[i]);
            }
          }
        }
      }
    }
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
