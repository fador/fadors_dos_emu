#pragma once

#include "../../cpu/CPU.hpp"
#include "../../memory/MemoryBus.hpp"
#include "../DriveManager.hpp"
#include <cstdint>

namespace fador::hw::cdrom {

// HLE implementation of MSCDEX (INT 2Fh, AH=15h) backed by the CD-ROM ISO
// images mounted in DriveManager.
class Mscdex {
public:
  static constexpr uint16_t kDeviceHeaderSeg = 0xF000;
  static constexpr uint16_t kDeviceHeaderOff = 0x0800;
  static constexpr uint16_t kDriverEntryOff = 0x0820;

  Mscdex(cpu::CPU &cpu, memory::MemoryBus &memory, DriveManager &driveManager);

  // Writes the dummy MSCD001 device driver header into BIOS ROM memory so
  // INT 2Fh AX=1501h and IOCTL 00h return a valid far pointer.
  void initializeDeviceHeader();

  // Handles INT 2Fh when AH == 0x15.  Always returns true.
  bool handleInt2F();

private:
  void handleDeviceRequest(uint8_t driveIdx, uint8_t subunit, uint32_t reqAddr);
  void handleIoctlInput(uint8_t driveIdx, const IsoImage &iso, uint32_t reqAddr);
  void handleIoctlOutput(uint32_t reqAddr);
  void handleReadLong(const IsoImage &iso, uint32_t reqAddr);

  uint32_t linearFromFar(uint16_t seg, uint16_t off) const;

  cpu::CPU &m_cpu;
  memory::MemoryBus &m_memory;
  DriveManager &m_driveManager;
  uint32_t m_lastHeadLba = 0;
};

} // namespace fador::hw::cdrom
