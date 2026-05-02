#pragma once
#include "../utils/Logger.hpp"
#include <cstdint>
#include <vector>

namespace fador::hw { class VGAController; }

namespace fador::memory {

class MemoryBus {
public:
  MemoryBus();
  ~MemoryBus() = default;

  // Disallow copy/move
  MemoryBus(const MemoryBus &) = delete;
  MemoryBus &operator=(const MemoryBus &) = delete;

  uint8_t read8(uint32_t address) const;
  uint16_t read16(uint32_t address) const;
  uint32_t read32(uint32_t address) const;

  void write8(uint32_t address, uint8_t value);
  void write16(uint32_t address, uint16_t value);
  void write32(uint32_t address, uint32_t value);

  // Direct access to a memory chunk, e.g., for loading ROM/RAM to a specific
  // address or video
  uint8_t *directAccess(uint32_t address);

  void setA20(bool enabled) {
    m_a20Enabled = enabled;
    LOG_DEBUG("Memory: A20 Gate ", enabled ? "Enabled" : "Disabled");
  }
  bool isA20Enabled() const { return m_a20Enabled; }

  // Connect VGA controller for plane-aware VRAM access
  void setVGA(hw::VGAController *vga) { m_vga = vga; }
  const hw::VGAController *getVGA() const { return m_vga; }

  static constexpr uint32_t MEMORY_SIZE = 64 * 1024 * 1024; // 64MB

private:
  std::vector<uint8_t> m_ram;
  bool m_a20Enabled{false};
  hw::VGAController *m_vga = nullptr;
};

} // namespace fador::memory
