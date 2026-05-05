#pragma once
#include "../utils/Logger.hpp"
#include <cstdint>
#include <vector>

namespace fador::hw { class VGAController; }

namespace fador::memory {

class MemoryBus {
public:
  static constexpr uint32_t MEMORY_SIZE = 128 * 1024 * 1024; // 128MB


  MemoryBus();
  ~MemoryBus() = default;

  // Disallow copy/move
  MemoryBus(const MemoryBus &) = delete;
  MemoryBus &operator=(const MemoryBus &) = delete;

  uint8_t read8(uint32_t address) const {
    uint32_t effective = effectiveAddress(address);
    if (effective < MEMORY_SIZE && !needsPlaneRead8(effective)) {
      return m_ram[effective];
    }
    return read8Slow(address, effective);
  }

  uint16_t read16(uint32_t address) const {
    uint32_t effective = effectiveAddress(address);
    if (effective < MEMORY_SIZE - 1) {
      return static_cast<uint16_t>(m_ram[effective]) |
             (static_cast<uint16_t>(m_ram[effective + 1]) << 8);
    }
    return read16Slow(address, effective);
  }

  uint32_t read32(uint32_t address) const {
    uint32_t effective = effectiveAddress(address);
    if (effective < MEMORY_SIZE - 3) {
      return static_cast<uint32_t>(m_ram[effective]) |
             (static_cast<uint32_t>(m_ram[effective + 1]) << 8) |
             (static_cast<uint32_t>(m_ram[effective + 2]) << 16) |
             (static_cast<uint32_t>(m_ram[effective + 3]) << 24);
    }
    return read32Slow(address, effective);
  }

  void write8(uint32_t address, uint8_t value) {
    uint32_t effective = effectiveAddress(address);
    if (effective < MEMORY_SIZE && effective >= IVT_BDA_LIMIT &&
        !needsPlaneWrite8(effective) && !needsTextVramLog(effective)) {
      m_ram[effective] = value;
      return;
    }
    write8Slow(address, effective, value);
  }

  void write16(uint32_t address, uint16_t value) {
    uint32_t effective = effectiveAddress(address);
    if (effective < MEMORY_SIZE - 1 && effective >= IVT_BDA_LIMIT &&
        !needsPlaneWrite16(effective) && !needsTextVramLog(effective)) {
      m_ram[effective] = static_cast<uint8_t>(value & 0xFF);
      m_ram[effective + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
      return;
    }
    write16Slow(address, effective, value);
  }

  void write32(uint32_t address, uint32_t value) {
    uint32_t effective = effectiveAddress(address);
    if (effective < MEMORY_SIZE - 3 && effective >= IVT_BDA_LIMIT &&
        !needsPlaneWrite32(effective) && !needsTextVramLog(effective)) {
      m_ram[effective] = static_cast<uint8_t>(value & 0xFF);
      m_ram[effective + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
      m_ram[effective + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
      m_ram[effective + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
      return;
    }
    write32Slow(address, effective, value);
  }

  // Direct access to a memory chunk, e.g., for loading ROM/RAM to a specific
  // address or video
  uint8_t *directAccess(uint32_t address);

  // Returns a direct pointer only when the effective range is contiguous RAM
  // with no VGA plane side effects.
  uint8_t *contiguousAccess(uint32_t address, uint32_t size);
  const uint8_t *contiguousAccess(uint32_t address, uint32_t size) const;

  void setA20(bool enabled) {
    m_a20Enabled = enabled;
    LOG_DEBUG("Memory: A20 Gate ", enabled ? "Enabled" : "Disabled");
  }
  bool isA20Enabled() const { return m_a20Enabled; }

  // Connect VGA controller for plane-aware VRAM access
  void setVGA(hw::VGAController *vga) { m_vga = vga; }
  const hw::VGAController *getVGA() const { return m_vga; }

private:
  static constexpr uint32_t A20_MASK = 0xFFEFFFFF;
  static constexpr uint32_t IVT_BDA_LIMIT = 0x400;
  static constexpr uint32_t VGA_PLANAR_START = 0xA0000;
  static constexpr uint32_t VGA_PLANAR_END = 0xB0000;
  static constexpr uint32_t TEXT_VRAM_START = 0xB8000;
  static constexpr uint32_t TEXT_VRAM_END = 0xBFFFF;

  uint32_t effectiveAddress(uint32_t address) const {
    return m_a20Enabled ? address : (address & A20_MASK);
  }

  bool needsPlaneRead8(uint32_t effective) const {
    return m_vga != nullptr && effective >= VGA_PLANAR_START &&
           effective < VGA_PLANAR_END;
  }

  bool needsPlaneWrite8(uint32_t effective) const {
    return needsPlaneRead8(effective);
  }

  bool needsPlaneWrite16(uint32_t effective) const {
    return m_vga != nullptr && effective >= VGA_PLANAR_START &&
           effective < (VGA_PLANAR_END - 1);
  }

  bool needsPlaneWrite32(uint32_t effective) const {
    return m_vga != nullptr && effective >= VGA_PLANAR_START &&
           effective < (VGA_PLANAR_END - 3);
  }

  bool needsTextVramLog(uint32_t effective) const {
    return effective >= TEXT_VRAM_START && effective < TEXT_VRAM_END;
  }

  uint8_t read8Slow(uint32_t address, uint32_t effectiveAddress) const;
  uint16_t read16Slow(uint32_t address, uint32_t effectiveAddress) const;
  uint32_t read32Slow(uint32_t address, uint32_t effectiveAddress) const;
  void write8Slow(uint32_t address, uint32_t effectiveAddress, uint8_t value);
  void write16Slow(uint32_t address, uint32_t effectiveAddress,
                   uint16_t value);
  void write32Slow(uint32_t address, uint32_t effectiveAddress,
                   uint32_t value);

  std::vector<uint8_t> m_ram;
  bool m_a20Enabled{false};
  hw::VGAController *m_vga = nullptr;
};

} // namespace fador::memory
