#pragma once
#include "../utils/Logger.hpp"
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <vector>

namespace fador::hw { class VGAController; }

namespace fador::memory {

class MemoryBus {
public:
  static constexpr uint32_t MEMORY_SIZE = 128 * 1024 * 1024; // 128MB

  // Physical memory map above 1 MiB:
  //   0x00100000 .. XMS_LIMIT-1        XMS / DPMI memory pool
  //   VBE_LFB_BASE .. +VBE_VRAM_SIZE   SVGA video memory (VBE linear FB)
  //   SYSTEM_RESERVED_BASE ..          host-owned tables (DPMI LDT, ...)
  // Nothing a program can allocate overlaps video memory or the tables.
  static constexpr uint32_t VBE_LFB_BASE = 0x07000000;
  static constexpr uint32_t VBE_VRAM_SIZE = 8 * 1024 * 1024;
  static constexpr uint32_t XMS_LIMIT = VBE_LFB_BASE;
  static constexpr uint32_t SYSTEM_RESERVED_BASE = VBE_LFB_BASE + VBE_VRAM_SIZE;


  MemoryBus();
  ~MemoryBus() = default;

  // Disallow copy/move
  MemoryBus(const MemoryBus &) = delete;
  MemoryBus &operator=(const MemoryBus &) = delete;

  // Fast paths: a single range check decides whether an access touches
  // plain RAM.  Everything else (VGA plane window A0000h-AFFFFh, accesses
  // past the end of RAM, write tracing) goes through the *Slow functions.
  uint8_t read8(uint32_t address) const {
    const uint32_t e = address & m_a20Mask;
    if (isPlain(e, 1))
      return m_ram[e];
    return read8Slow(address, e);
  }

  uint16_t read16(uint32_t address) const {
    const uint32_t e = address & m_a20Mask;
    if (isPlain(e, 2)) {
      uint16_t v;
      std::memcpy(&v, &m_ram[e], sizeof(v));
      return v;
    }
    return read16Slow(address, e);
  }

  uint32_t read32(uint32_t address) const {
    const uint32_t e = address & m_a20Mask;
    if (isPlain(e, 4)) {
      uint32_t v;
      std::memcpy(&v, &m_ram[e], sizeof(v));
      return v;
    }
    return read32Slow(address, e);
  }

  void write8(uint32_t address, uint8_t value) {
    const uint32_t e = address & m_a20Mask;
    if (isPlainWrite(e, 1)) {
      m_ram[e] = value;
      return;
    }
    write8Slow(address, e, value);
  }

  void write16(uint32_t address, uint16_t value) {
    const uint32_t e = address & m_a20Mask;
    if (isPlainWrite(e, 2)) {
      std::memcpy(&m_ram[e], &value, sizeof(value));
      return;
    }
    write16Slow(address, e, value);
  }

  void write32(uint32_t address, uint32_t value) {
    const uint32_t e = address & m_a20Mask;
    if (isPlainWrite(e, 4)) {
      std::memcpy(&m_ram[e], &value, sizeof(value));
      return;
    }
    write32Slow(address, e, value);
  }

  // Route every write through the slow path so IVT/BDA and text-VRAM
  // writes are logged (enabled for --trace/--debug=video sessions).
  void setWriteTracing(bool enabled) { m_traceWrites = enabled; }

  // Direct access to a memory chunk, e.g., for loading ROM/RAM to a specific
  // address or video
  uint8_t *directAccess(uint32_t address);

  // Returns a direct pointer only when the effective range is contiguous RAM
  // with no VGA plane side effects.
  uint8_t *contiguousAccess(uint32_t address, uint32_t size);
  const uint8_t *contiguousAccess(uint32_t address, uint32_t size) const;

  void setA20(bool enabled) {
    m_a20Enabled = enabled;
    m_a20Mask = enabled ? 0xFFFFFFFFu : A20_MASK;
    LOG_DEBUG("Memory: A20 Gate ", enabled ? "Enabled" : "Disabled");
  }
  bool isA20Enabled() const { return m_a20Enabled; }

  // Connect VGA controller for plane-aware VRAM access
  void setVGA(hw::VGAController *vga) { m_vga = vga; }
  const hw::VGAController *getVGA() const { return m_vga; }

  // Clear all VGA plane memory (used on video mode change)
  void clearVGA();

  // Optional diagnostic hook invoked (in the slow path only) for accesses
  // beyond emulated physical memory, e.g. to log the faulting CS:EIP.
  void setOutOfBoundsHandler(std::function<void(uint32_t address, bool write)> h) {
    m_oobHandler = std::move(h);
  }

  // VGA DAC palette RAM: 256 entries x (R,G,B), 6 bits each.  It lives
  // outside the guest-visible address space (it used to be stored at
  // E000:0000, where EMS page frames, UMBs and ROM scanners could clobber
  // or read it).
  static constexpr uint32_t DAC_SIZE = 256 * 3;
  uint8_t *dacPalette() { return m_dac.data(); }
  const uint8_t *dacPalette() const { return m_dac.data(); }

private:
  static constexpr uint32_t A20_MASK = 0xFFEFFFFF;
  static constexpr uint32_t IVT_BDA_LIMIT = 0x400;
  static constexpr uint32_t VGA_PLANAR_START = 0xA0000;
  static constexpr uint32_t VGA_PLANAR_END = 0xB0000;
  static constexpr uint32_t TEXT_VRAM_START = 0xB8000;
  static constexpr uint32_t TEXT_VRAM_END = 0xBFFFF;

  uint32_t effectiveAddress(uint32_t address) const {
    return address & m_a20Mask;
  }

  // [e, e+n) lies inside RAM and does not touch the VGA plane window
  bool isPlain(uint32_t e, uint32_t n) const {
    if (e > MEMORY_SIZE - n)
      return false;
    return m_vga == nullptr || e >= VGA_PLANAR_END || e + n <= VGA_PLANAR_START;
  }
  bool isPlainWrite(uint32_t e, uint32_t n) const {
    return !m_traceWrites && isPlain(e, n);
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

  struct FreeDeleter {
    void operator()(uint8_t *p) const { std::free(p); }
  };
  // calloc()-backed so the 128 MiB of guest RAM is zero pages supplied
  // lazily by the OS: constructing a MemoryBus no longer touches (and
  // commits) every page up front.
  std::unique_ptr<uint8_t[], FreeDeleter> m_ram;
  std::array<uint8_t, DAC_SIZE> m_dac{};
  std::function<void(uint32_t, bool)> m_oobHandler;
  void reportOutOfBounds(uint32_t address, bool write) const {
    if (m_oobHandler)
      m_oobHandler(address, write);
  }
  bool m_a20Enabled{false};
  uint32_t m_a20Mask{A20_MASK};
  bool m_traceWrites{false};
  hw::VGAController *m_vga = nullptr;
};

} // namespace fador::memory
