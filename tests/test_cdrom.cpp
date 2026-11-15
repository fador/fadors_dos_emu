#include "test_framework.hpp"
#include "cpu/CPU.hpp"
#include "hw/DOS.hpp"
#include "hw/DriveManager.hpp"
#include "hw/cdrom/IsoImage.hpp"
#include "memory/MemoryBus.hpp"

#include <array>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace fador::hw;
using namespace fador::hw::cdrom;
using namespace fador::cpu;
using namespace fador::memory;

namespace {

namespace fs = std::filesystem;

void putLE16(uint8_t *p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

void putBE16(uint8_t *p, uint16_t v) {
  p[0] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[1] = static_cast<uint8_t>(v & 0xFF);
}

void putBoth16(uint8_t *p, uint16_t v) {
  putLE16(p, v);
  putBE16(p + 2, v);
}

void putLE32(uint8_t *p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

void putBE32(uint8_t *p, uint32_t v) {
  p[0] = static_cast<uint8_t>((v >> 24) & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[3] = static_cast<uint8_t>(v & 0xFF);
}

void putBoth32(uint8_t *p, uint32_t v) {
  putLE32(p, v);
  putBE32(p + 4, v);
}

uint8_t appendDirRecord(uint8_t *buf, size_t offset, const std::string &name,
                        uint32_t lba, uint32_t size, uint8_t flags) {
  uint8_t nameLen = static_cast<uint8_t>(name.size());
  uint8_t recLen = static_cast<uint8_t>(33 + nameLen);
  if (recLen % 2 != 0)
    ++recLen;

  uint8_t *r = buf + offset;
  std::memset(r, 0, recLen);
  r[0] = recLen;
  r[1] = 0; // XAR length
  putBoth32(r + 2, lba);
  putBoth32(r + 10, size);
  r[18] = 96; // 1996
  r[19] = 7;  // July
  r[20] = 15; // 15th
  r[21] = 14; // 14:30:20
  r[22] = 30;
  r[23] = 20;
  r[24] = 0;
  r[25] = flags;
  putBoth16(r + 28, 1);
  r[32] = nameLen;
  std::memcpy(r + 33, name.data(), nameLen);
  return recLen;
}

// Builds a minimal valid ISO 9660 image with:
//   - Sector 16: Primary Volume Descriptor (volume "TESTCD", copyright "COPY.TXT")
//   - Sector 17: Volume Descriptor Set Terminator
//   - Sector 18: Root directory (HELLO.TXT;1, DATA/ subdir)
//   - Sector 19: DATA directory (NESTED.BIN;1)
//   - Sector 20: HELLO.TXT payload
//   - Sector 21: NESTED.BIN payload
std::vector<uint8_t> buildTestIsoBytes() {
  constexpr uint32_t kTotalSectors = 22;
  std::vector<uint8_t> img(kTotalSectors * IsoImage::kSectorSize, 0);

  const std::string helloText = "Hello from ISO 9660!\r\n";
  const std::string nestedData = "ABCDEF0123456789";

  // Sector 16: Primary Volume Descriptor
  uint8_t *pvd = img.data() + 16 * IsoImage::kSectorSize;
  pvd[0] = 0x01;
  std::memcpy(pvd + 1, "CD001", 5);
  pvd[6] = 0x01;
  std::memset(pvd + 40, ' ', 32);
  std::memcpy(pvd + 40, "TESTCD", 6);
  putBoth32(pvd + 80, kTotalSectors);
  putBoth16(pvd + 120, 1);
  putBoth16(pvd + 124, 1);
  putBoth16(pvd + 128, IsoImage::kSectorSize);
  std::memset(pvd + 702, ' ', 37 * 3);
  std::memcpy(pvd + 702, "COPY.TXT", 8);
  std::memcpy(pvd + 739, "ABS.TXT", 7);
  std::memcpy(pvd + 776, "BIB.TXT", 7);
  appendDirRecord(pvd, 156, std::string(1, '\0'), 18, IsoImage::kSectorSize, 0x02);

  // Sector 17: Terminator
  uint8_t *term = img.data() + 17 * IsoImage::kSectorSize;
  term[0] = 0xFF;
  std::memcpy(term + 1, "CD001", 5);
  term[6] = 0x01;

  // Sector 18: Root Directory
  uint8_t *root = img.data() + 18 * IsoImage::kSectorSize;
  size_t off = 0;
  off += appendDirRecord(root, off, std::string(1, '\0'), 18, IsoImage::kSectorSize, 0x02);
  off += appendDirRecord(root, off, std::string(1, '\1'), 18, IsoImage::kSectorSize, 0x02);
  off += appendDirRecord(root, off, "HELLO.TXT;1", 20, static_cast<uint32_t>(helloText.size()), 0x00);
  off += appendDirRecord(root, off, "DATA", 19, IsoImage::kSectorSize, 0x02);

  // Sector 19: DATA Directory
  uint8_t *dataDir = img.data() + 19 * IsoImage::kSectorSize;
  off = 0;
  off += appendDirRecord(dataDir, off, std::string(1, '\0'), 19, IsoImage::kSectorSize, 0x02);
  off += appendDirRecord(dataDir, off, std::string(1, '\1'), 18, IsoImage::kSectorSize, 0x02);
  off += appendDirRecord(dataDir, off, "NESTED.BIN;1", 21, static_cast<uint32_t>(nestedData.size()), 0x00);

  // Sector 20 & 21: Payloads
  std::memcpy(img.data() + 20 * IsoImage::kSectorSize, helloText.data(), helloText.size());
  std::memcpy(img.data() + 21 * IsoImage::kSectorSize, nestedData.data(), nestedData.size());

  return img;
}

struct TempIsoFile {
  fs::path path;
  explicit TempIsoFile(const std::string &name = "fador_test_cdrom.iso") {
    path = fs::temp_directory_path() / name;
    auto bytes = buildTestIsoBytes();
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char *>(bytes.data()),
              static_cast<std::streamsize>(bytes.size()));
  }
  ~TempIsoFile() {
    std::error_code ec;
    fs::remove(path, ec);
  }
};

void writeGuestString(MemoryBus &mem, uint32_t addr, const std::string &s) {
  for (size_t i = 0; i < s.size(); ++i)
    mem.write8(addr + static_cast<uint32_t>(i), static_cast<uint8_t>(s[i]));
  mem.write8(addr + static_cast<uint32_t>(s.size()), 0);
}

std::string readGuestString(MemoryBus &mem, uint32_t addr, size_t maxLen = 64) {
  std::string out;
  for (size_t i = 0; i < maxLen; ++i) {
    uint8_t c = mem.read8(addr + static_cast<uint32_t>(i));
    if (c == 0)
      break;
    out += static_cast<char>(c);
  }
  return out;
}

} // namespace

TEST_CASE("CDROM: IsoImage parsing and file reading", "[CDROM][ISO]") {
  TempIsoFile tmp("fador_test_iso_parse.iso");
  std::string err;
  auto iso = IsoImage::open(tmp.path.string(), &err);
  REQUIRE(iso != nullptr);
  REQUIRE(iso->volumeLabel() == "TESTCD");
  REQUIRE(iso->volumeSectors() == 22);
  REQUIRE(iso->copyrightFile() == "COPY.TXT");
  REQUIRE(iso->abstractFile() == "ABS.TXT");
  REQUIRE(iso->bibliographicFile() == "BIB.TXT");

  auto rootList = iso->listDirectory("");
  REQUIRE(rootList.has_value());
  REQUIRE(rootList->size() == 2);

  auto hello = iso->lookup("hello.txt");
  REQUIRE(hello.has_value());
  REQUIRE(!hello->isDirectory());
  REQUIRE(hello->size == 22);

  std::array<uint8_t, 32> buf{};
  size_t n = iso->readFile(*hello, 0, std::span<uint8_t>(buf.data(), buf.size()));
  REQUIRE(n == 22);
  REQUIRE(std::string(reinterpret_cast<char *>(buf.data()), n) ==
          "Hello from ISO 9660!\r\n");

  auto nested = iso->lookup("\\DATA\\NESTED.BIN");
  REQUIRE(nested.has_value());
  REQUIRE(nested->size == 16);
  std::array<uint8_t, 16> nbuf{};
  REQUIRE(iso->readFile(*nested, 6, std::span<uint8_t>(nbuf.data(), 10)) == 10);
  REQUIRE(std::string(reinterpret_cast<char *>(nbuf.data()), 10) == "0123456789");
}

TEST_CASE("CDROM: DOS INT 21h file and directory operations", "[CDROM][DOS]") {
  TempIsoFile tmp("fador_test_iso_dos.iso");
  CPU cpu;
  MemoryBus memory;
  DriveManager dm;
  DOS dos(cpu, memory);
  dos.setDriveManager(&dm);
  dos.initialize();

  REQUIRE(dm.mountCdrom('D', tmp.path.string()));
  REQUIRE(dm.isCdrom('D'));

  const uint16_t dataSeg = 0x2000;
  const uint32_t dataBase = static_cast<uint32_t>(dataSeg) << 4;
  cpu.setSegReg(DS, dataSeg);
  cpu.setSegBase(DS, dataBase);

  SECTION("Open, Seek, Read, Close on D:") {
    writeGuestString(memory, dataBase + 0x0000, "D:\\HELLO.TXT");
    cpu.setReg8(AH, 0x3D);
    cpu.setReg8(AL, 0x00); // Read-only
    cpu.setReg16(DX, 0x0000);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
    uint16_t handle = cpu.getReg16(AX);
    REQUIRE(handle >= 5);

    // Seek to offset 6 from start
    cpu.setReg8(AH, 0x42);
    cpu.setReg8(AL, 0x00);
    cpu.setReg16(BX, handle);
    cpu.setReg16(CX, 0);
    cpu.setReg16(DX, 6);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
    REQUIRE(cpu.getReg16(AX) == 6);

    // Read 4 bytes ("from")
    cpu.setReg8(AH, 0x3F);
    cpu.setReg16(BX, handle);
    cpu.setReg16(CX, 4);
    cpu.setReg16(DX, 0x0100);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
    REQUIRE(cpu.getReg16(AX) == 4);
    REQUIRE(readGuestString(memory, dataBase + 0x0100, 4) == "from");

    // Close
    cpu.setReg8(AH, 0x3E);
    cpu.setReg16(BX, handle);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
  }

  SECTION("FindFirst / FindNext and CHDIR / GETCWD") {
    // Set DTA
    cpu.setReg8(AH, 0x1A);
    cpu.setReg16(DX, 0x0200);
    dos.handleInterrupt(0x21);

    writeGuestString(memory, dataBase + 0x0000, "D:\\*.*");
    cpu.setReg8(AH, 0x4E);
    cpu.setReg8(CL, 0x10); // Include directories
    cpu.setReg16(DX, 0x0000);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
    REQUIRE(readGuestString(memory, dataBase + 0x021E) == "HELLO.TXT");

    cpu.setReg8(AH, 0x4F);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
    REQUIRE(readGuestString(memory, dataBase + 0x021E) == "DATA");
    REQUIRE((memory.read8(dataBase + 0x0215) & 0x10) != 0);

    // CHDIR into D:\DATA and check GETCWD
    writeGuestString(memory, dataBase + 0x0000, "D:\\DATA");
    cpu.setReg8(AH, 0x3B);
    cpu.setReg16(DX, 0x0000);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));

    cpu.setReg8(AH, 0x47);
    cpu.setReg8(DL, 4); // D:
    cpu.setReg16(SI, 0x0300);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
    REQUIRE(readGuestString(memory, dataBase + 0x0300) == "DATA");
  }

  SECTION("Read-only enforcement rejects write/create/delete") {
    writeGuestString(memory, dataBase + 0x0000, "D:\\NEW.TXT");
    cpu.setReg8(AH, 0x3C);
    cpu.setReg16(CX, 0);
    cpu.setReg16(DX, 0x0000);
    dos.handleInterrupt(0x21);
    REQUIRE((cpu.getEFLAGS() & FLAG_CARRY) != 0);
    REQUIRE(cpu.getReg16(AX) == 0x05);

    writeGuestString(memory, dataBase + 0x0000, "D:\\HELLO.TXT");
    cpu.setReg8(AH, 0x41);
    cpu.setReg16(DX, 0x0000);
    dos.handleInterrupt(0x21);
    REQUIRE((cpu.getEFLAGS() & FLAG_CARRY) != 0);
    REQUIRE(cpu.getReg16(AX) == 0x05);
  }
}

TEST_CASE("CDROM: MSCDEX INT 2Fh AX=15xx services", "[CDROM][MSCDEX]") {
  TempIsoFile tmp("fador_test_iso_mscdex.iso");
  CPU cpu;
  MemoryBus memory;
  DriveManager dm;
  DOS dos(cpu, memory);
  dos.setDriveManager(&dm);
  dos.initialize();

  // Before mounting any CD-ROM, AX=1500h reports BX=0
  cpu.setReg16(AX, 0x1500);
  cpu.setReg16(BX, 0x0000);
  dos.handleInterrupt(0x2F);
  REQUIRE(cpu.getReg16(BX) == 0);

  REQUIRE(dm.mountCdrom('D', tmp.path.string()));

  const uint16_t esSeg = 0x2200;
  const uint32_t esBase = static_cast<uint32_t>(esSeg) << 4;
  cpu.setSegReg(ES, esSeg);
  cpu.setSegBase(ES, esBase);

  // AX=1500h: Installation check
  cpu.setReg16(AX, 0x1500);
  cpu.setReg16(BX, 0x0000);
  dos.handleInterrupt(0x2F);
  REQUIRE(cpu.getReg16(BX) == 1); // 1 CD-ROM drive
  REQUIRE(cpu.getReg16(CX) == 3); // D: (0=A, 3=D)

  // AX=150Ch: Version
  cpu.setReg16(AX, 0x150C);
  cpu.setReg16(BX, 0x0000);
  dos.handleInterrupt(0x2F);
  REQUIRE(cpu.getReg16(BX) >= 0x0200);

  // AX=150Bh: Drive check (D: yes, C: no)
  cpu.setReg16(AX, 0x150B);
  cpu.setReg16(CX, 3);
  dos.handleInterrupt(0x2F);
  REQUIRE(cpu.getReg16(BX) == 0xADAD);
  REQUIRE(cpu.getReg16(AX) != 0);

  cpu.setReg16(AX, 0x150B);
  cpu.setReg16(CX, 2);
  dos.handleInterrupt(0x2F);
  REQUIRE(cpu.getReg16(BX) == 0xADAD);
  REQUIRE(cpu.getReg16(AX) == 0);

  // AX=150Dh: Drive letters
  cpu.setReg16(AX, 0x150D);
  cpu.setReg16(BX, 0x0000);
  dos.handleInterrupt(0x2F);
  REQUIRE(memory.read8(esBase) == 3);

  // AX=1505h: Read VTOC (sector index 0 -> PVD)
  cpu.setReg16(AX, 0x1505);
  cpu.setReg16(CX, 3);
  cpu.setReg16(DX, 0);
  cpu.setReg16(BX, 0x0100);
  dos.handleInterrupt(0x2F);
  REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
  REQUIRE(cpu.getReg16(AX) == 1);
  REQUIRE(memory.read8(esBase + 0x0100) == 1);
  REQUIRE(memory.read8(esBase + 0x0101) == 'C');

  // AX=1508h: Absolute Disk Read (sector 20 -> HELLO.TXT)
  cpu.setReg16(AX, 0x1508);
  cpu.setReg16(CX, 3);
  cpu.setReg16(SI, 0);
  cpu.setReg16(DI, 20);
  cpu.setReg16(DX, 1);
  cpu.setReg16(BX, 0x1000);
  dos.handleInterrupt(0x2F);
  REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
  REQUIRE(readGuestString(memory, esBase + 0x1000, 22) ==
          "Hello from ISO 9660!\r\n");

  // AX=1510h: Device Driver Request - IOCTL Input 08h (Volume Size)
  const uint16_t reqOff = 0x2000;
  const uint16_t cbOff = 0x2100;
  memory.write8(esBase + reqOff + 0, 26);   // Header length
  memory.write8(esBase + reqOff + 2, 0x03); // IOCTL Input
  memory.write16(esBase + reqOff + 3, 0);   // Status
  memory.write16(esBase + reqOff + 0x0E, cbOff);
  memory.write16(esBase + reqOff + 0x10, esSeg);
  memory.write8(esBase + cbOff + 0, 0x08);  // Volume size
  cpu.setReg16(AX, 0x1510);
  cpu.setReg16(CX, 3);
  cpu.setReg16(BX, reqOff);
  dos.handleInterrupt(0x2F);
  REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
  REQUIRE(memory.read16(esBase + reqOff + 3) == 0x0100);
  REQUIRE(memory.read32(esBase + cbOff + 1) == 22);

  // AX=1510h: Device Driver Request - Read Long (0x80) sector 21
  memory.write8(esBase + reqOff + 0, 28);
  memory.write8(esBase + reqOff + 2, 0x80); // Read Long
  memory.write16(esBase + reqOff + 3, 0);
  memory.write8(esBase + reqOff + 0x0D, 0); // HSG (LBA)
  memory.write16(esBase + reqOff + 0x0E, 0x3000);
  memory.write16(esBase + reqOff + 0x10, esSeg);
  memory.write16(esBase + reqOff + 0x12, 1);  // 1 sector
  memory.write32(esBase + reqOff + 0x14, 21); // LBA 21
  memory.write8(esBase + reqOff + 0x18, 0);   // Cooked mode
  cpu.setReg16(AX, 0x1510);
  cpu.setReg16(CX, 3);
  cpu.setReg16(BX, reqOff);
  dos.handleInterrupt(0x2F);
  REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
  REQUIRE(memory.read16(esBase + reqOff + 3) == 0x0100);
  REQUIRE(readGuestString(memory, esBase + 0x3000, 16) == "ABCDEF0123456789");
}
