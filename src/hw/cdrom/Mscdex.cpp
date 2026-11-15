#include "Mscdex.hpp"
#include "../../utils/Logger.hpp"

#include <algorithm>
#include <array>
#include <cstring>

namespace fador::hw::cdrom {

namespace {

constexpr uint16_t kStatusDone = 0x0100;
constexpr uint16_t kStatusError = 0x8000;

constexpr uint8_t kErrUnknownUnit = 0x01;
constexpr uint8_t kErrNotReady = 0x02;
constexpr uint8_t kErrUnknownCommand = 0x03;
constexpr uint8_t kErrSectorNotFound = 0x08;
constexpr uint8_t kErrGeneralFailure = 0x0C;

uint32_t lbaToRedBook(uint32_t lba) {
  const uint32_t totalFrames = lba + 150;
  const uint8_t frame = static_cast<uint8_t>(totalFrames % 75);
  const uint8_t sec = static_cast<uint8_t>((totalFrames / 75) % 60);
  const uint8_t min = static_cast<uint8_t>((totalFrames / 75) / 60);
  return static_cast<uint32_t>(frame) | (static_cast<uint32_t>(sec) << 8) |
         (static_cast<uint32_t>(min) << 16);
}

uint32_t redBookToLba(uint32_t msf) {
  const uint32_t frame = msf & 0xFF;
  const uint32_t sec = (msf >> 8) & 0xFF;
  const uint32_t min = (msf >> 16) & 0xFF;
  const uint32_t total = (min * 60u + sec) * 75u + frame;
  return total >= 150 ? (total - 150) : 0;
}

void writeLE32BE32(memory::MemoryBus &mem, uint32_t addr, uint32_t val) {
  mem.write32(addr, val);
  mem.write8(addr + 4, static_cast<uint8_t>((val >> 24) & 0xFF));
  mem.write8(addr + 5, static_cast<uint8_t>((val >> 16) & 0xFF));
  mem.write8(addr + 6, static_cast<uint8_t>((val >> 8) & 0xFF));
  mem.write8(addr + 7, static_cast<uint8_t>(val & 0xFF));
}

void writeLE16BE16(memory::MemoryBus &mem, uint32_t addr, uint16_t val) {
  mem.write16(addr, val);
  mem.write8(addr + 2, static_cast<uint8_t>((val >> 8) & 0xFF));
  mem.write8(addr + 3, static_cast<uint8_t>(val & 0xFF));
}

} // namespace

Mscdex::Mscdex(cpu::CPU &cpu, memory::MemoryBus &memory,
               DriveManager &driveManager)
    : m_cpu(cpu), m_memory(memory), m_driveManager(driveManager) {}

void Mscdex::initializeDeviceHeader() {
  const uint32_t hdr =
      (static_cast<uint32_t>(kDeviceHeaderSeg) << 4) + kDeviceHeaderOff;
  m_memory.write32(hdr + 0x00, 0xFFFFFFFF);    // Next driver
  m_memory.write16(hdr + 0x04, 0xC800);        // Char/CD-ROM + IOCTL + open/close
  m_memory.write16(hdr + 0x06, kDriverEntryOff); // Strategy
  m_memory.write16(hdr + 0x08, kDriverEntryOff); // Interrupt
  static constexpr char kDevName[8] = {'M', 'S', 'C', 'D', '0', '0', '1', ' '};
  for (uint32_t i = 0; i < 8; ++i)
    m_memory.write8(hdr + 0x0A + i, static_cast<uint8_t>(kDevName[i]));
  m_memory.write16(hdr + 0x12, 0x0000); // Reserved

  const auto cdDrives = m_driveManager.getCdromDriveIndices();
  m_memory.write8(hdr + 0x14, cdDrives.empty() ? 0 : cdDrives.front());
  m_memory.write8(hdr + 0x15, static_cast<uint8_t>(cdDrives.size()));

  const uint32_t entry =
      (static_cast<uint32_t>(kDeviceHeaderSeg) << 4) + kDriverEntryOff;
  m_memory.write8(entry, 0xCB); // RETF
}

uint32_t Mscdex::linearFromFar(uint16_t seg, uint16_t off) const {
  return (static_cast<uint32_t>(seg) << 4) + off;
}

bool Mscdex::handleInt2F() {
  const uint16_t ax = m_cpu.getReg16(cpu::AX);
  const auto cdDrives = m_driveManager.getCdromDriveIndices();

  if (cdDrives.empty()) {
    LOG_DEBUG("DOS: INT 2Fh MSCDEX AX=0x", std::hex, ax, " -> Not installed");
    m_cpu.setReg8(cpu::AL, 0x00);
    if (ax == 0x1500) {
      m_cpu.setReg16(cpu::BX, 0x0000);
      m_cpu.setReg16(cpu::CX, 0x0000);
    } else if (ax == 0x150B) {
      m_cpu.setReg16(cpu::AX, 0x0000);
    }
    return true;
  }

  // Keep the ROM device header's first-drive and subunit-count fields in sync
  // with DriveManager mounts.
  const uint32_t hdr =
      (static_cast<uint32_t>(kDeviceHeaderSeg) << 4) + kDeviceHeaderOff;
  m_memory.write8(hdr + 0x14, cdDrives.front());
  m_memory.write8(hdr + 0x15, static_cast<uint8_t>(cdDrives.size()));

  auto clearCarry = [&]() {
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
  };
  auto setError = [&](uint16_t errCode) {
    m_cpu.setReg16(cpu::AX, errCode);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
  };

  auto writeDriveString = [&](uint16_t driveIdx, const std::string &s) {
    const uint32_t dst =
        m_cpu.getSegBase(cpu::ES) + m_cpu.getReg16(cpu::BX);
    if (driveIdx >= 26 || !m_driveManager.isCdrom(static_cast<uint8_t>(driveIdx))) {
      setError(0x000F); // Invalid drive
      return;
    }
    const size_t len = std::min<size_t>(s.size(), 37);
    for (size_t i = 0; i < len; ++i)
      m_memory.write8(dst + static_cast<uint32_t>(i), static_cast<uint8_t>(s[i]));
    m_memory.write8(dst + static_cast<uint32_t>(len), 0);
    clearCarry();
  };

  switch (ax) {
  case 0x1500: { // Installation Check
    m_cpu.setReg16(cpu::BX, static_cast<uint16_t>(cdDrives.size()));
    m_cpu.setReg16(cpu::CX, static_cast<uint16_t>(cdDrives.front()));
    clearCarry();
    LOG_DOS("MSCDEX: 1500h count=", cdDrives.size(),
            " first=", static_cast<int>(cdDrives.front()));
    break;
  }
  case 0x1501: { // Get Drive Device List
    uint32_t dst = m_cpu.getSegBase(cpu::ES) + m_cpu.getReg16(cpu::BX);
    const uint32_t hdrFar =
        (static_cast<uint32_t>(kDeviceHeaderSeg) << 16) | kDeviceHeaderOff;
    for (size_t i = 0; i < cdDrives.size(); ++i) {
      m_memory.write8(dst, static_cast<uint8_t>(i));
      m_memory.write32(dst + 1, hdrFar);
      dst += 5;
    }
    clearCarry();
    break;
  }
  case 0x1502: { // Get Copyright File Name
    const uint16_t cx = m_cpu.getReg16(cpu::CX);
    auto iso = cx < 26 ? m_driveManager.getCdromImage(static_cast<uint8_t>(cx)) : nullptr;
    writeDriveString(cx, iso ? iso->copyrightFile() : std::string());
    break;
  }
  case 0x1503: { // Get Abstract File Name
    const uint16_t cx = m_cpu.getReg16(cpu::CX);
    auto iso = cx < 26 ? m_driveManager.getCdromImage(static_cast<uint8_t>(cx)) : nullptr;
    writeDriveString(cx, iso ? iso->abstractFile() : std::string());
    break;
  }
  case 0x1504: { // Get Bibliographic Doc File Name
    const uint16_t cx = m_cpu.getReg16(cpu::CX);
    auto iso = cx < 26 ? m_driveManager.getCdromImage(static_cast<uint8_t>(cx)) : nullptr;
    writeDriveString(cx, iso ? iso->bibliographicFile() : std::string());
    break;
  }
  case 0x1505: { // Read VTOC
    const uint16_t cx = m_cpu.getReg16(cpu::CX);
    const uint16_t dx = m_cpu.getReg16(cpu::DX);
    auto iso = cx < 26 ? m_driveManager.getCdromImage(static_cast<uint8_t>(cx)) : nullptr;
    if (!iso) {
      setError(15); // Invalid drive
      break;
    }
    std::array<uint8_t, IsoImage::kSectorSize> sec{};
    if (!iso->readSector(16u + dx, sec)) {
      setError(21); // Not ready
      break;
    }
    const uint32_t dst = m_cpu.getSegBase(cpu::ES) + m_cpu.getReg16(cpu::BX);
    for (uint32_t i = 0; i < IsoImage::kSectorSize; ++i)
      m_memory.write8(dst + i, sec[i]);
    uint16_t descType = 0;
    if (sec[0] == 1)
      descType = 1;
    else if (sec[0] == 0xFF)
      descType = 0xFF;
    m_cpu.setReg16(cpu::AX, descType);
    clearCarry();
    break;
  }
  case 0x1506: // Turn Debugging On
  case 0x1507: // Turn Debugging Off
    clearCarry();
    break;
  case 0x1508: { // Absolute Disk Read
    const uint16_t cx = m_cpu.getReg16(cpu::CX);
    const uint16_t dx = m_cpu.getReg16(cpu::DX);
    const uint32_t startLba =
        (static_cast<uint32_t>(m_cpu.getReg16(cpu::SI)) << 16) |
        m_cpu.getReg16(cpu::DI);
    auto iso = cx < 26 ? m_driveManager.getCdromImage(static_cast<uint8_t>(cx)) : nullptr;
    if (!iso) {
      setError(0x000F);
      break;
    }
    std::vector<uint8_t> buf(static_cast<size_t>(dx) * IsoImage::kSectorSize);
    if (dx > 0 && !iso->readSectors(startLba, dx, buf)) {
      setError(0x0015); // Not ready
      break;
    }
    const uint32_t dst = m_cpu.getSegBase(cpu::ES) + m_cpu.getReg16(cpu::BX);
    for (size_t i = 0; i < buf.size(); ++i)
      m_memory.write8(dst + static_cast<uint32_t>(i), buf[i]);
    m_lastHeadLba = startLba + dx;
    clearCarry();
    break;
  }
  case 0x1509: // Absolute Disk Write (unsupported / write-protected)
    setError(0x0013);
    break;
  case 0x150B: { // Drive Check
    const uint16_t cx = m_cpu.getReg16(cpu::CX);
    const bool isCd = cx < 26 && m_driveManager.isCdrom(static_cast<uint8_t>(cx));
    m_cpu.setReg16(cpu::BX, 0xADAD);
    m_cpu.setReg16(cpu::AX, isCd ? 0x0001 : 0x0000);
    clearCarry();
    break;
  }
  case 0x150C: { // Get MSCDEX Version
    m_cpu.setReg16(cpu::BX, 0x0217); // MSCDEX v2.23
    clearCarry();
    break;
  }
  case 0x150D: { // Get CD-ROM Drive Letters
    const uint32_t dst = m_cpu.getSegBase(cpu::ES) + m_cpu.getReg16(cpu::BX);
    for (size_t i = 0; i < cdDrives.size(); ++i)
      m_memory.write8(dst + static_cast<uint32_t>(i), cdDrives[i]);
    clearCarry();
    break;
  }
  case 0x150E: { // Get/Set Volume Descriptor Preference
    const uint16_t cx = m_cpu.getReg16(cpu::CX);
    const uint16_t bx = m_cpu.getReg16(cpu::BX);
    if (cx >= 26 || !m_driveManager.isCdrom(static_cast<uint8_t>(cx))) {
      setError(15);
      break;
    }
    if (bx == 0x0000) {
      m_cpu.setReg16(cpu::DX, 0x0100); // Primary volume descriptor
      clearCarry();
    } else if (bx == 0x0001) {
      clearCarry();
    } else {
      setError(1);
    }
    break;
  }
  case 0x150F: { // Get Directory Entry
    const uint8_t cl = m_cpu.getReg8(cpu::CL);
    const uint8_t ch = m_cpu.getReg8(cpu::CH);
    auto iso = cl < 26 ? m_driveManager.getCdromImage(cl) : nullptr;
    if (!iso) {
      setError(15);
      break;
    }
    const uint32_t pathAddr = m_cpu.getSegBase(cpu::ES) + m_cpu.getReg16(cpu::BX);
    std::string path;
    for (uint32_t i = 0; i < 256; ++i) {
      uint8_t c = m_memory.read8(pathAddr + i);
      if (c == 0)
        break;
      path += static_cast<char>(c);
    }
    // Strip optional "D:" prefix if present.
    if (path.size() >= 2 && std::isalpha(static_cast<unsigned char>(path[0])) &&
        path[1] == ':') {
      path = path.substr(2);
    }
    auto entry = iso->lookup(path);
    if (!entry) {
      setError(2); // File not found
      break;
    }
    const uint32_t outAddr = linearFromFar(m_cpu.getReg16(cpu::SI),
                                           m_cpu.getReg16(cpu::DI));
    if (ch & 0x01) {
      // Canonicalized directory entry (285 bytes).
      for (uint32_t i = 0; i < 285; ++i)
        m_memory.write8(outAddr + i, 0);
      m_memory.write8(outAddr + 0x00, 0);
      m_memory.write32(outAddr + 0x01, entry->extentLba);
      m_memory.write16(outAddr + 0x05, IsoImage::kSectorSize);
      m_memory.write32(outAddr + 0x07, entry->size);
      for (uint32_t i = 0; i < 7; ++i)
        m_memory.write8(outAddr + 0x0B + i, entry->recordingTime[i]);
      m_memory.write8(outAddr + 0x12, entry->flags);
      m_memory.write16(outAddr + 0x15, 1); // Volume set seq #
      const uint8_t nlen = static_cast<uint8_t>(std::min<size_t>(entry->name.size(), 37));
      m_memory.write8(outAddr + 0x17, nlen);
      for (uint8_t i = 0; i < nlen; ++i)
        m_memory.write8(outAddr + 0x18 + i, static_cast<uint8_t>(entry->name[i]));
      m_memory.write16(outAddr + 0x3E, 1); // Version 1
    } else {
      // Direct ISO 9660 directory entry.
      const uint8_t nlen = static_cast<uint8_t>(std::min<size_t>(entry->name.size(), 220));
      uint8_t recLen = static_cast<uint8_t>(33 + nlen);
      if (recLen % 2 != 0)
        ++recLen;
      for (uint8_t i = 0; i < recLen; ++i)
        m_memory.write8(outAddr + i, 0);
      m_memory.write8(outAddr + 0x00, recLen);
      m_memory.write8(outAddr + 0x01, 0); // XAR length
      writeLE32BE32(m_memory, outAddr + 0x02, entry->extentLba);
      writeLE32BE32(m_memory, outAddr + 0x0A, entry->size);
      for (uint32_t i = 0; i < 7; ++i)
        m_memory.write8(outAddr + 0x12 + i, entry->recordingTime[i]);
      m_memory.write8(outAddr + 0x19, entry->flags);
      writeLE16BE16(m_memory, outAddr + 0x1C, 1);
      m_memory.write8(outAddr + 0x20, nlen);
      for (uint8_t i = 0; i < nlen; ++i)
        m_memory.write8(outAddr + 0x21 + i, static_cast<uint8_t>(entry->name[i]));
    }
    m_cpu.setReg16(cpu::AX, 1); // ISO 9660
    clearCarry();
    break;
  }
  case 0x1510: { // Send Device Driver Request
    const uint16_t cx = m_cpu.getReg16(cpu::CX);
    auto it = std::find(cdDrives.begin(), cdDrives.end(), static_cast<uint8_t>(cx));
    if (cx >= 26 || it == cdDrives.end()) {
      setError(0x000F); // Invalid drive
      break;
    }
    const uint8_t subunit = static_cast<uint8_t>(it - cdDrives.begin());
    const uint32_t reqAddr = m_cpu.getSegBase(cpu::ES) + m_cpu.getReg16(cpu::BX);
    handleDeviceRequest(static_cast<uint8_t>(cx), subunit, reqAddr);
    clearCarry();
    break;
  }
  default:
    LOG_DOS("MSCDEX: Unhandled INT 2Fh AX=0x", std::hex, ax);
    setError(0x0001); // Invalid function
    break;
  }
  return true;
}

void Mscdex::handleDeviceRequest(uint8_t driveIdx, uint8_t subunit,
                                 uint32_t reqAddr) {
  m_memory.write8(reqAddr + 1, subunit);
  const uint8_t cmd = m_memory.read8(reqAddr + 2);
  auto iso = m_driveManager.getCdromImage(driveIdx);
  if (!iso) {
    m_memory.write16(reqAddr + 3, kStatusDone | kStatusError | kErrUnknownUnit);
    return;
  }

  switch (cmd) {
  case 0x03: // IOCTL Input
    handleIoctlInput(driveIdx, *iso, reqAddr);
    break;
  case 0x0C: // IOCTL Output
    handleIoctlOutput(reqAddr);
    break;
  case 0x0D: // Device Open
  case 0x0E: // Device Close
    m_memory.write16(reqAddr + 3, kStatusDone);
    break;
  case 0x80: // Read Long
    handleReadLong(*iso, reqAddr);
    break;
  case 0x82: // Read Long Prefetch
    m_memory.write16(reqAddr + 3, kStatusDone);
    break;
  case 0x83: { // Seek
    const uint8_t addrMode = m_memory.read8(reqAddr + 0x0D);
    const uint32_t rawLoc = m_memory.read32(reqAddr + 0x14);
    const uint32_t lba = (addrMode == 1) ? redBookToLba(rawLoc) : rawLoc;
    if (lba >= iso->discSectors()) {
      m_memory.write16(reqAddr + 3,
                       kStatusDone | kStatusError | kErrSectorNotFound);
    } else {
      m_lastHeadLba = lba;
      m_memory.write16(reqAddr + 3, kStatusDone);
    }
    break;
  }
  case 0x84: // Play Audio
  case 0x85: // Stop Audio
  case 0x88: // Resume Audio
    m_memory.write16(reqAddr + 3, kStatusDone);
    break;
  default:
    LOG_DOS("MSCDEX: Unhandled device request cmd=0x", std::hex,
            static_cast<int>(cmd));
    m_memory.write16(reqAddr + 3,
                     kStatusDone | kStatusError | kErrUnknownCommand);
    break;
  }
}

void Mscdex::handleIoctlInput(uint8_t /*driveIdx*/, const IsoImage &iso,
                              uint32_t reqAddr) {
  const uint16_t bufOff = m_memory.read16(reqAddr + 0x0E);
  const uint16_t bufSeg = m_memory.read16(reqAddr + 0x10);
  const uint32_t cb = linearFromFar(bufSeg, bufOff);
  const uint8_t fn = m_memory.read8(cb);

  switch (fn) {
  case 0x00: { // Return Device Header Address
    m_memory.write32(cb + 1,
                     (static_cast<uint32_t>(kDeviceHeaderSeg) << 16) |
                         kDeviceHeaderOff);
    m_memory.write16(reqAddr + 3, kStatusDone);
    break;
  }
  case 0x01: { // Location of Head
    const uint8_t mode = m_memory.read8(cb + 1);
    const uint32_t val =
        (mode == 1) ? lbaToRedBook(m_lastHeadLba) : m_lastHeadLba;
    m_memory.write32(cb + 2, val);
    m_memory.write16(reqAddr + 3, kStatusDone);
    break;
  }
  case 0x04: { // Audio Channel Info
    m_memory.write8(cb + 1, 0);
    m_memory.write8(cb + 2, 0xFF);
    m_memory.write8(cb + 3, 1);
    m_memory.write8(cb + 4, 0xFF);
    m_memory.write8(cb + 5, 2);
    m_memory.write8(cb + 6, 0x00);
    m_memory.write8(cb + 7, 3);
    m_memory.write8(cb + 8, 0x00);
    m_memory.write16(reqAddr + 3, kStatusDone);
    break;
  }
  case 0x06: { // Device Status
    // Bit 2: cooked + raw, Bit 3: read-only, Bit 4: data + audio,
    // Bit 9: HSG + RedBook addressing supported.
    m_memory.write32(cb + 1, 0x0000021C);
    m_memory.write16(reqAddr + 3, kStatusDone);
    break;
  }
  case 0x07: { // Return Sector Size
    const uint8_t mode = m_memory.read8(cb + 1);
    const uint16_t size = (mode == 1) ? 2352 : IsoImage::kSectorSize;
    m_memory.write16(cb + 2, size);
    m_memory.write16(reqAddr + 3, kStatusDone);
    break;
  }
  case 0x08: { // Return Volume Size
    m_memory.write32(cb + 1, iso.volumeSectors());
    m_memory.write16(reqAddr + 3, kStatusDone);
    break;
  }
  case 0x09: { // Media Changed
    m_memory.write8(cb + 1, 1); // 1 = Media not changed
    m_memory.write16(reqAddr + 3, kStatusDone);
    break;
  }
  case 0x0A: { // Audio Disk Info
    const auto &tracks = iso.tracks();
    const uint8_t lo = tracks.empty() ? 1 : tracks.front().number;
    const uint8_t hi = tracks.empty() ? 1 : tracks.back().number;
    m_memory.write8(cb + 1, lo);
    m_memory.write8(cb + 2, hi);
    m_memory.write32(cb + 3, lbaToRedBook(iso.discSectors()));
    m_memory.write16(reqAddr + 3, kStatusDone);
    break;
  }
  case 0x0B: { // Audio Track Info
    const uint8_t trackNum = m_memory.read8(cb + 1);
    const auto &tracks = iso.tracks();
    auto it = std::find_if(tracks.begin(), tracks.end(),
                           [&](const IsoTrack &t) { return t.number == trackNum; });
    if (it == tracks.end()) {
      m_memory.write16(reqAddr + 3,
                       kStatusDone | kStatusError | kErrGeneralFailure);
      break;
    }
    m_memory.write32(cb + 2, lbaToRedBook(it->startLba));
    m_memory.write8(cb + 6, it->audio ? 0x00 : 0x40); // bit 6: data track
    m_memory.write16(reqAddr + 3, kStatusDone);
    break;
  }
  case 0x0C: { // Audio Q-Channel Info
    for (uint32_t i = 1; i <= 10; ++i)
      m_memory.write8(cb + i, 0);
    m_memory.write16(reqAddr + 3, kStatusDone);
    break;
  }
  case 0x0F: { // Audio Status Info
    m_memory.write16(cb + 1, 0); // Not paused
    m_memory.write32(cb + 3, 0);
    m_memory.write32(cb + 7, 0);
    m_memory.write16(reqAddr + 3, kStatusDone);
    break;
  }
  default:
    LOG_DOS("MSCDEX: Unhandled IOCTL Input fn=0x", std::hex,
            static_cast<int>(fn));
    m_memory.write16(reqAddr + 3,
                     kStatusDone | kStatusError | kErrUnknownCommand);
    break;
  }
}

void Mscdex::handleIoctlOutput(uint32_t reqAddr) {
  const uint16_t bufOff = m_memory.read16(reqAddr + 0x0E);
  const uint16_t bufSeg = m_memory.read16(reqAddr + 0x10);
  const uint32_t cb = linearFromFar(bufSeg, bufOff);
  const uint8_t fn = m_memory.read8(cb);
  if (fn <= 0x05) {
    m_memory.write16(reqAddr + 3, kStatusDone);
  } else {
    m_memory.write16(reqAddr + 3,
                     kStatusDone | kStatusError | kErrUnknownCommand);
  }
}

void Mscdex::handleReadLong(const IsoImage &iso, uint32_t reqAddr) {
  const uint8_t addrMode = m_memory.read8(reqAddr + 0x0D);
  const uint16_t bufOff = m_memory.read16(reqAddr + 0x0E);
  const uint16_t bufSeg = m_memory.read16(reqAddr + 0x10);
  const uint16_t numSectors = m_memory.read16(reqAddr + 0x12);
  const uint32_t startRaw = m_memory.read32(reqAddr + 0x14);
  const uint8_t readMode = m_memory.read8(reqAddr + 0x18);

  if (readMode != 0) {
    m_memory.write16(reqAddr + 3,
                     kStatusDone | kStatusError | kErrUnknownCommand);
    return;
  }

  const uint32_t lba = (addrMode == 1) ? redBookToLba(startRaw) : startRaw;
  std::vector<uint8_t> buf(static_cast<size_t>(numSectors) * IsoImage::kSectorSize);
  if (numSectors > 0 && !iso.readSectors(lba, numSectors, buf)) {
    m_memory.write16(reqAddr + 3, kStatusDone | kStatusError | kErrNotReady);
    return;
  }

  const uint32_t dst = linearFromFar(bufSeg, bufOff);
  for (size_t i = 0; i < buf.size(); ++i)
    m_memory.write8(dst + static_cast<uint32_t>(i), buf[i]);
  m_lastHeadLba = lba + numSectors;
  m_memory.write16(reqAddr + 3, kStatusDone);
}

} // namespace fador::hw::cdrom
