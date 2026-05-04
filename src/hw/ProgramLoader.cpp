#include "ProgramLoader.hpp"
#include "../memory/himem/HIMEM.hpp"
#include "../utils/Logger.hpp"
#include "DOS.hpp"
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <vector>

namespace fador::hw { extern memory::HIMEM *g_himem; }

namespace fador::hw {

namespace {

constexpr std::array<uint8_t, 4> kFBOVSignature{{'F', 'B', 'O', 'V'}};

uint16_t readLE16(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t readLE32(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

struct FBOVHeader {
  uint16_t blockSize;
  uint16_t overlayId;
  uint32_t auxOffset;
  uint32_t auxCount;
};

std::vector<DOS::FBOVOverlay>
parseFBOVOverlays(std::ifstream &file, uint32_t startOffset, uint32_t fileSize) {
  std::vector<DOS::FBOVOverlay> overlays;
  if (fileSize <= startOffset || fileSize - startOffset < 16u)
    return overlays;

  std::vector<uint8_t> tail(fileSize - startOffset);
  file.clear();
  file.seekg(static_cast<std::streamoff>(startOffset), std::ios::beg);
  if (!file.read(reinterpret_cast<char *>(tail.data()),
                 static_cast<std::streamsize>(tail.size()))) {
    file.clear();
    return overlays;
  }

  auto it = std::search(tail.begin(), tail.end(), kFBOVSignature.begin(),
                        kFBOVSignature.end());
  while (it != tail.end()) {
    const size_t relOffset =
        static_cast<size_t>(std::distance(tail.begin(), it));
    if (tail.size() - relOffset < 16u)
      break;

    const uint8_t *headerData = tail.data() + relOffset + kFBOVSignature.size();
    const FBOVHeader header{readLE16(headerData + 0), readLE16(headerData + 2),
                            readLE32(headerData + 4), readLE32(headerData + 8)};
    const uint32_t absOffset = startOffset + static_cast<uint32_t>(relOffset);

    const bool valid =
        header.blockSize >= 16u && header.overlayId != 0 &&
        absOffset + header.blockSize <= fileSize;
    if (valid) {
      overlays.push_back({header.overlayId, absOffset, header.blockSize,
                          header.auxOffset, header.auxCount, 0});
    }

    auto next = it + 1;
    if (valid) {
      next = tail.begin() + static_cast<std::ptrdiff_t>(relOffset + header.blockSize);
    }
    it = std::search(next, tail.end(), kFBOVSignature.begin(),
                     kFBOVSignature.end());
  }

  file.clear();
  return overlays;
}

} // namespace

ProgramLoader::ProgramLoader(cpu::CPU &cpu, memory::MemoryBus &memory,
                             memory::HIMEM *himem)
    : m_cpu(cpu), m_memory(memory), m_himem(himem) {}

bool ProgramLoader::loadCOM(const std::string &path, uint16_t segment,
                            const std::string &args) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    LOG_ERROR("ProgramLoader: Failed to open .COM file: ", path);
    return false;
  }

  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);

  if (size > 0xFF00) { // Max size for COM file (64KB - 256 bytes)
    LOG_ERROR("ProgramLoader: .COM file too large: ", size);
    return false;
  }

  // 1. Create PSP at segment:0000
  createPSP(segment, args, path);

  // 2. Load file data at segment:0100
  uint32_t loadAddr = (segment << 4) + 0x100;
  std::vector<uint8_t> buffer(static_cast<size_t>(size));
  if (file.read(reinterpret_cast<char *>(buffer.data()), size)) {
    for (uint32_t i = 0; i < size; ++i) {
      m_memory.write8(loadAddr + i, buffer[i]);
    }
  } else {
    LOG_ERROR("ProgramLoader: Failed to read .COM file data");
    return false;
  }

  // 3. Set CPU state for .COM execution
  m_cpu.setSegReg(cpu::CS, segment);
  m_cpu.setSegReg(cpu::DS, segment);
  m_cpu.setSegReg(cpu::ES, segment);
  m_cpu.setSegReg(cpu::SS, segment);
  m_cpu.setEIP(0x100);
  m_cpu.setReg16(cpu::SP, 0xFFFE); // Stack at top of segment

  LOG_INFO("ProgramLoader: Successfully loaded .COM file: ", path, " at ",
           std::hex, segment, ":0100");
  return true;
}

bool ProgramLoader::loadEXE(const std::string &path, uint16_t segment, DOS &dos,
                            const std::string &args, bool useHimem) {
  // Basic MZ Parsing
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    LOG_ERROR("ProgramLoader: Failed to open .EXE file: ", path);
    return false;
  }

  file.seekg(0, std::ios::end);
  const uint32_t fileSize = static_cast<uint32_t>(file.tellg());
  file.seekg(0, std::ios::beg);

  dos.clearOverlayInfo();

  struct MZHeader {
    uint16_t signature; // 'MZ'
    uint16_t lastPageSize;
    uint16_t numPages;
    uint16_t numReloc;
    uint16_t headerSize; // In paragraphs (16 bytes)
    uint16_t minAlloc;
    uint16_t maxAlloc;
    uint16_t ss; // Initial SS (relative to load)
    uint16_t sp; // Initial SP
    uint16_t checksum;
    uint16_t ip; // Initial IP
    uint16_t cs; // Initial CS (relative to load)
    uint16_t relocOffset;
    uint16_t overlayNum;
  } header;

  if (!file.read(reinterpret_cast<char *>(&header), sizeof(header))) {
    LOG_ERROR("ProgramLoader: Failed to read EXE header");
    return false;
  }

  if (header.signature != 0x5A4D) { // 'ZM' or 'MZ' (LE)
    LOG_ERROR("ProgramLoader: Invalid EXE signature: 0x", std::hex,
              header.signature);
    return false;
  }

  LOG_INFO("ProgramLoader: MZ Stack initial SS:SP = ", std::hex, header.ss, ":",
           header.sp);
  LOG_INFO("ProgramLoader: MZ Entry CS:IP = ", std::hex, header.cs, ":",
           header.ip);

  // Check for NE header offset at 0x3C
  file.seekg(0x3C, std::ios::beg);
  uint16_t neOffset = 0;
  file.read(reinterpret_cast<char *>(&neOffset), sizeof(neOffset));

  if (neOffset > 0) {
    file.seekg(neOffset, std::ios::beg);
    uint16_t neSig = 0;
    file.read(reinterpret_cast<char *>(&neSig), sizeof(neSig));
    if (neSig == 0x454E) { // 'NE'
      LOG_INFO("ProgramLoader: NE Header found at 0x", std::hex, neOffset);

      file.seekg(neOffset + 0x0E, std::ios::beg);
      uint16_t appFlags = 0;
      file.read(reinterpret_cast<char *>(&appFlags), sizeof(appFlags));

      file.seekg(neOffset + 0x36, std::ios::beg);
      uint8_t targetOS = 0;
      file.read(reinterpret_cast<char *>(&targetOS), sizeof(targetOS));
      LOG_INFO("ProgramLoader: NE Header AppFlags=0x", std::hex, appFlags,
               " TargetOS=0x", (int)targetOS);

      file.seekg(neOffset + 0x14, std::ios::beg);
      uint16_t entryIP = 0;
      file.read(reinterpret_cast<char *>(&entryIP), sizeof(entryIP));
      uint16_t entrySegIdx = 0;
      file.read(reinterpret_cast<char *>(&entrySegIdx), sizeof(entrySegIdx));
      LOG_INFO("ProgramLoader: NE Entry Point: SegIdx=", std::dec, entrySegIdx,
               " IP=0x", std::hex, entryIP);

      // Read NE Header fields for VROOMM
      file.seekg(neOffset + 0x1C, std::ios::beg);
      uint16_t numSegments = 0;
      file.read(reinterpret_cast<char *>(&numSegments), sizeof(numSegments));

      file.seekg(neOffset + 0x22, std::ios::beg);
      uint16_t segTableOffset = 0;
      file.read(reinterpret_cast<char *>(&segTableOffset),
                sizeof(segTableOffset));

      file.seekg(neOffset + 0x32, std::ios::beg);
      uint16_t alignShift = 0;
      file.read(reinterpret_cast<char *>(&alignShift), sizeof(alignShift));

      // Read more tables for debugging relocations
      file.seekg(neOffset + 0x1E, std::ios::beg);
      uint16_t numModRefs = 0;
      file.read(reinterpret_cast<char *>(&numModRefs), 2);

      file.seekg(neOffset + 0x26, std::ios::beg);
      uint16_t modRefTableOff = 0;
      file.read(reinterpret_cast<char *>(&modRefTableOff), 2);

      file.seekg(neOffset + 0x28, std::ios::beg);
      uint16_t importTableOff = 0;
      file.read(reinterpret_cast<char *>(&importTableOff), 2);

      LOG_INFO("ProgramLoader: NE Modules: ", numModRefs, " ModRefOff: 0x",
               std::hex, modRefTableOff, " ImportOff: 0x", importTableOff);

      for (int i = 0; i < numModRefs; ++i) {
        file.seekg(neOffset + modRefTableOff + (i * 2), std::ios::beg);
        uint16_t nameOff = 0;
        file.read(reinterpret_cast<char *>(&nameOff), 2);
        file.seekg(neOffset + importTableOff + nameOff, std::ios::beg);
        uint8_t len = 0;
        file.read(reinterpret_cast<char *>(&len), 1);
        std::string name(len, ' ');
        file.read(&name[0], len);
        LOG_INFO("ProgramLoader: Module ", i + 1, ": ", name);
      }

      // Read Segment Table
      std::vector<DOS::NESegment> segments;

      file.seekg(neOffset + segTableOffset, std::ios::beg);
      LOG_INFO("ProgramLoader: NE alignShift=", std::dec, alignShift);

      // Get MZ size to determine which segments are resident
      file.seekg(2, std::ios::beg);
      uint16_t lastPageSize = 0;
      file.read(reinterpret_cast<char *>(&lastPageSize), 2);
      uint16_t numPages = 0;
      file.read(reinterpret_cast<char *>(&numPages), 2);
      uint32_t mzSize =
          (numPages - 1) * 512 + (lastPageSize == 0 ? 512 : lastPageSize);
      LOG_INFO("ProgramLoader: MZ Header reports image size 0x", std::hex,
               mzSize, " bytes");

      file.seekg(10, std::ios::beg);
      uint16_t minAlloc = 0;
      file.read(reinterpret_cast<char *>(&minAlloc), 2);
      uint16_t maxAlloc = 0;
      file.read(reinterpret_cast<char *>(&maxAlloc), 2);
      LOG_INFO("ProgramLoader: MZ Header minAlloc=0x", std::hex, minAlloc,
               " maxAlloc=0x", maxAlloc);

      file.seekg(neOffset + segTableOffset, std::ios::beg);
      for (int i = 0; i < numSegments; ++i) {
        DOS::NESegment seg;
        file.read(reinterpret_cast<char *>(&seg.fileOffsetSector), 2);
        file.read(reinterpret_cast<char *>(&seg.length), 2);
        file.read(reinterpret_cast<char *>(&seg.flags), 2);
        file.read(reinterpret_cast<char *>(&seg.minAlloc), 2);

        uint32_t fileOff = (uint32_t)seg.fileOffsetSector << alignShift;

        if (fileOff > 0 && fileOff < mzSize) {
          uint32_t mzBaseIdx = (uint32_t)header.headerSize * 16;
          if (fileOff >= mzBaseIdx) {
            seg.loadedSegment =
                (segment + 0x10) + (uint16_t)((fileOff - mzBaseIdx) / 16);
            LOG_INFO("ProgramLoader: Segment ", i + 1,
                     " marked as resident at 0x", std::hex, seg.loadedSegment);
          } else {
            seg.loadedSegment = 0;
          }
        } else {
          seg.loadedSegment = 0;
        }
        segments.push_back(seg);
        LOG_INFO("ProgramLoader: Segment ", std::dec, i + 1, ": sector=0x",
                 std::hex, seg.fileOffsetSector, " len=0x", seg.length,
                 " flags=0x", seg.flags, " minAlloc=0x", seg.minAlloc);
      }

      dos.setNEInfo(path, alignShift, segments, segment + 0x10);
      LOG_INFO("ProgramLoader: VROOMM Info set. Segments: ", std::dec,
               numSegments, " AlignShift: ", alignShift);
    }
  }

  // Load only the MZ image portion of the file. MS-DOS executables often have
  // extra data appended (overlays, LE/LX headers, debug info) which must NOT
  // be loaded into conventional memory, as it would overflow into VRAM/BIOS.
  uint32_t imageOffset = static_cast<uint32_t>(header.headerSize) * 16;
  uint32_t mzImageBytes =
      (static_cast<uint32_t>(header.numPages) - 1) * 512 +
      (header.lastPageSize == 0 ? 512 : header.lastPageSize);

  if (mzImageBytes < imageOffset) {
    LOG_ERROR("ProgramLoader: Invalid MZ header (image size < header size)");
    return false;
  }
  uint32_t imageSize = mzImageBytes - imageOffset;

  bool isNEExe = false;
  std::vector<DOS::FBOVOverlay> fbovOverlays;
  if (neOffset > 0) {
    file.seekg(neOffset, std::ios::beg);
    uint16_t neSig2 = 0;
    file.read(reinterpret_cast<char *>(&neSig2), sizeof(neSig2));
    if (neSig2 == 0x454E) {
      isNEExe = true;
      LOG_INFO("ProgramLoader: NE exe: loading only MZ stub (0x", std::hex,
               imageSize, " bytes); overlay segments loaded on demand");
    }
  }

  if (!isNEExe) {
    fbovOverlays = parseFBOVOverlays(file, mzImageBytes, fileSize);
    if (!fbovOverlays.empty()) {
      dos.setFBOVInfo(path, fbovOverlays);
      LOG_INFO("ProgramLoader: FBOV overlay info set. Blocks: ", std::dec,
               fbovOverlays.size());
    }
  }

  // For now, let's just use a simple load to segment:0000 (after PSP)
  // Actually, EXE load usually puts PSP at segment, and image at segment + 10h
  // (256 bytes)
  createPSP(segment, args, path);

  uint16_t loadSegment = segment + 0x10; // Image starts after 256-byte PSP
  uint32_t loadAddr = (loadSegment << 4);

  std::vector<uint8_t> buffer(imageSize);
  file.clear();
  file.seekg(imageOffset, std::ios::beg);
  if (!file.read(reinterpret_cast<char *>(buffer.data()), imageSize)) {
    LOG_ERROR("ProgramLoader: Failed to read EXE image");
    return false;
  }

  if (loadAddr + imageSize > memory::MemoryBus::MEMORY_SIZE) {
    if (useHimem) {
      // Try to use XMS (HIMEM)
      if (m_himem && m_himem->available() >= imageSize) {
        uint16_t handle = m_himem->allocate(imageSize);
        uint8_t *xmsPtr = m_himem->getBlock(handle);
        if (xmsPtr) {
          std::copy(buffer.begin(), buffer.end(), xmsPtr);
          LOG_WARN("ProgramLoader: Loaded EXE into XMS (HIMEM) at handle ",
                   handle, ". This is not true DOS behavior.");
          // Set CPU state to a fake segment (e.g., 0xF000) for test/demo only
          m_cpu.setSegReg(cpu::CS, 0xF000);
          m_cpu.setEIP(0);
          m_cpu.setSegReg(cpu::SS, 0xF000);
          m_cpu.setReg16(cpu::SP, 0xFFFE);
          m_cpu.setSegReg(cpu::DS, 0xF000);
          m_cpu.setSegReg(cpu::ES, 0xF000);
          return true;
        } else {
          LOG_ERROR("ProgramLoader: Failed to get XMS block for EXE");
          return false;
        }
      } else {
        LOG_ERROR("ProgramLoader: Not enough XMS (HIMEM) for EXE image");
        return false;
      }
    } else {
      LOG_ERROR(
          "ProgramLoader: EXE image too large for DOS memory: loadAddr=0x",
          std::hex, loadAddr, " size=0x", imageSize);
      return false;
    }
  } else {
    for (uint32_t i = 0; i < imageSize; ++i) {
      m_memory.write8(loadAddr + i, buffer[i]);
    }

    // The memory beyond the loaded image retains the 0xCC fill written
    // by MemoryBus::MemoryBus().  This simulates dirty DOS RAM — real DOS
    // leaves leftover data from the boot process / previous programs.
    // Borland mixed-model programs (e.g. TC 2.01) scan their far-heap
    // allocation and rely on non-zero bytes to avoid false end-of-table
    // sentinel matches; zeroing this area causes premature table
    // termination and broken initialisation paths.

    // For executables with demand-loaded overlays, the PSP MCB was sized to
    // consume all conventional memory in DOS::initialize(). Shrink the loaded
    // block to the actual resident image so the remainder is available for
    // overlay allocations.
    if (isNEExe || !fbovOverlays.empty()) {
      // PSP (16 para) + MZ stub (rounded up to paragraph boundary)
      uint16_t stubParas = static_cast<uint16_t>((imageSize + 15u) / 16u);
      uint16_t loadedParas = 0x10u + stubParas;                // PSP + stub
      uint16_t pspMcbSeg = static_cast<uint16_t>(segment - 1); // 0x0FFF
      const uint32_t pspMcbAddr = static_cast<uint32_t>(pspMcbSeg) << 4u;
      const uint16_t currentProgramParas = m_memory.read16(pspMcbAddr + 3u);
      const uint8_t nextBlockType = m_memory.read8(pspMcbAddr + 0u);

      // Update PSP MCB size (byte offset 3 within MCB header)
      m_memory.write16(pspMcbAddr + 3u, loadedParas);
      // PSP MCB type must be 'M' when another block follows.
      m_memory.write8(pspMcbAddr + 0u, 'M');

      if (currentProgramParas > loadedParas + 1u) {
        // Write a new free MCB between the loaded image and the next block.
        uint16_t freeMcbSeg = static_cast<uint16_t>(pspMcbSeg + loadedParas + 1u);
        uint32_t freeMcbAddr = static_cast<uint32_t>(freeMcbSeg) << 4u;
        uint16_t freeSize =
            static_cast<uint16_t>(currentProgramParas - loadedParas - 1u);
        m_memory.write8(freeMcbAddr + 0u, nextBlockType);
        m_memory.write16(freeMcbAddr + 1u, 0u); // owner = 0 (free)
        m_memory.write16(freeMcbAddr + 3u, freeSize);
        for (uint32_t offset = 0x05; offset < 0x10; ++offset) {
          m_memory.write8(freeMcbAddr + offset, 0u);
        }

        LOG_INFO("ProgramLoader: Overlay-capable MCB updated – PSP block 0x",
                 std::hex, loadedParas, " paras; free block at seg 0x",
                 freeMcbSeg, " size 0x", freeSize, " paras");
      }

      m_memory.write16((static_cast<uint32_t>(segment) << 4u) + 0x02,
                       static_cast<uint16_t>(segment + loadedParas));
    }
  }

  // Relocations
  if (header.numReloc > 0) {
    file.seekg(header.relocOffset, std::ios::beg);
    for (int i = 0; i < header.numReloc; ++i) {
      uint16_t offset, relSegment;
      file.read(reinterpret_cast<char *>(&offset), 2);
      file.read(reinterpret_cast<char *>(&relSegment), 2);
      uint32_t relocAddr = ((loadSegment + relSegment) << 4) + offset;
      if (relocAddr + 1 >= memory::MemoryBus::MEMORY_SIZE) {
        LOG_ERROR("ProgramLoader: Relocation address out of DOS memory: 0x",
                  std::hex, relocAddr);
        return false;
      }
      uint16_t val = m_memory.read16(relocAddr);
      m_memory.write16(relocAddr, val + loadSegment);
    }
  }

  // Set CPU state
  m_cpu.setSegReg(cpu::CS, loadSegment + header.cs);
  m_cpu.setEIP(header.ip);
  m_cpu.setSegReg(cpu::SS, loadSegment + header.ss);
  m_cpu.setReg16(cpu::SP, header.sp);
  m_cpu.setSegReg(cpu::DS, segment); // DS and ES point to PSP
  m_cpu.setSegReg(cpu::ES, segment);

  LOG_INFO("ProgramLoader: Successfully loaded .EXE file: ", path,
           " at CS:EIP=", std::hex, m_cpu.getSegReg(cpu::CS), ":",
           m_cpu.getEIP());
  return true;
}

void ProgramLoader::createPSP(uint16_t segment, const std::string &args,
                              const std::string &programPath) {
  constexpr uint16_t kEnvBlockParas = 0x20;
  uint32_t pspAddr = (segment << 4);

  // Zero the entire 256-byte PSP first so no fields contain 0xCC garbage
  for (int i = 0; i < 256; ++i) {
    m_memory.write8(pspAddr + i, 0x00);
  }

  // INT 20h instruction (CD 20) at offset 0
  m_memory.write8(pspAddr + 0x00, 0xCD);
  m_memory.write8(pspAddr + 0x01, 0x20);

  const uint32_t mcbAddr = ((segment - 1u) << 4);
  const uint8_t mcbType = m_memory.read8(mcbAddr + 0x00);
  const uint16_t mcbOwner = m_memory.read16(mcbAddr + 0x01);
  const bool hasOwningMCB = (mcbType == 'M' || mcbType == 'Z') &&
                            mcbOwner == segment;
  uint16_t programBlockParas = hasOwningMCB
                                   ? m_memory.read16(mcbAddr + 0x03)
                                   : static_cast<uint16_t>(0xA000u - segment);
  uint16_t envSegment = static_cast<uint16_t>(segment - kEnvBlockParas);

  if (hasOwningMCB && programBlockParas > kEnvBlockParas + 1u) {
    const uint8_t nextBlockType = mcbType;
    const uint16_t envMcbSeg =
        static_cast<uint16_t>(segment + programBlockParas - kEnvBlockParas - 1u);
    const uint32_t envMcbAddr = static_cast<uint32_t>(envMcbSeg) << 4u;

    programBlockParas =
        static_cast<uint16_t>(programBlockParas - kEnvBlockParas - 1u);
    envSegment = static_cast<uint16_t>(envMcbSeg + 1u);

    m_memory.write8(mcbAddr + 0x00, 'M');
    m_memory.write16(mcbAddr + 0x03, programBlockParas);

    m_memory.write8(envMcbAddr + 0x00, nextBlockType);
    m_memory.write16(envMcbAddr + 0x01, segment);
    m_memory.write16(envMcbAddr + 0x03, kEnvBlockParas);
    for (uint32_t offset = 0x05; offset < 0x10; ++offset) {
      m_memory.write8(envMcbAddr + offset, 0x00);
    }
  }

  // Segment of the first paragraph beyond this program block.
  m_memory.write16(pspAddr + 0x02,
                   static_cast<uint16_t>(segment + programBlockParas));

  // Saved interrupt vectors (from IVT) at offsets 0x0A-0x15
  // INT 22h (Terminate Address)
  m_memory.write16(pspAddr + 0x0A, m_memory.read16(0x22 * 4));     // IP
  m_memory.write16(pspAddr + 0x0C, m_memory.read16(0x22 * 4 + 2)); // CS
  // INT 23h (Ctrl-Break Address)
  m_memory.write16(pspAddr + 0x0E, m_memory.read16(0x23 * 4));
  m_memory.write16(pspAddr + 0x10, m_memory.read16(0x23 * 4 + 2));
  // INT 24h (Critical Error Address)
  m_memory.write16(pspAddr + 0x12, m_memory.read16(0x24 * 4));
  m_memory.write16(pspAddr + 0x14, m_memory.read16(0x24 * 4 + 2));

  // Parent PSP segment at offset 0x16 (no parent — use own segment)
  m_memory.write16(pspAddr + 0x16, segment);

  // Job File Table (JFT) at offset 0x18: 20 bytes
  // Handles 0-4 map to SFT indices 0-4 (stdin/stdout/stderr/stdaux/stdprn)
  for (int i = 0; i < 5; ++i) {
    m_memory.write8(pspAddr + 0x18 + i, static_cast<uint8_t>(i));
  }
  // Handles 5-19 are unused (0xFF = closed)
  for (int i = 5; i < 20; ++i) {
    m_memory.write8(pspAddr + 0x18 + i, 0xFF);
  }

  // JFT size at offset 0x32 (default 20 handles)
  m_memory.write16(pspAddr + 0x32, 0x0014);

  // JFT pointer at offset 0x34 (far pointer to PSP:0018)
  m_memory.write16(pspAddr + 0x34, 0x0018);  // Offset
  m_memory.write16(pspAddr + 0x36, segment); // Segment

  // DOS function dispatcher at offset 0x50: INT 21h / RETF
  m_memory.write8(pspAddr + 0x50, 0xCD); // INT
  m_memory.write8(pspAddr + 0x51, 0x21); // 21h
  m_memory.write8(pspAddr + 0x52, 0xCB); // RETF

  // Environment block segment at offset 0x2C.
  // Prefer a real MCB-backed block owned by the PSP when one is available.
  uint32_t envAddr = (envSegment << 4);
  for (uint32_t i = 0; i < static_cast<uint32_t>(kEnvBlockParas) * 16u; ++i) {
    m_memory.write8(envAddr + i, 0x00);
  }

  // Derive directory containing the program for PATH/LIB/INCLUDE
  std::filesystem::path progFsPath = std::filesystem::absolute(programPath);
  std::string progDir = progFsPath.parent_path().string();

  // Convert host paths to DOS-style paths for the environment block.
  // DOS programs expect backslash separators and a drive letter prefix.
  // Use the last directory component as the DOS directory name.
  std::string dirName = progFsPath.parent_path().filename().string();
  std::transform(
      dirName.begin(), dirName.end(), dirName.begin(),
      [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  std::string fileName = progFsPath.filename().string();
  std::transform(
      fileName.begin(), fileName.end(), fileName.begin(),
      [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  std::string dosDir = "C:\\" + dirName;
  std::string dosFullPath = dosDir + "\\" + fileName;

  // Format: VAR=VAL\0\0\0\x01\x00PROGRAM_PATH\0
  std::string envStr;
  envStr += "PATH=";
  envStr += dosDir;
  envStr += '\0';
  envStr += "LIB=";
  envStr += dosDir;
  envStr += '\0';
  envStr += "INCLUDE=";
  envStr += dosDir;
  envStr += '\0';
  envStr += "BLASTER=A220 I5 D1 T3";
  envStr += '\0';
  envStr += '\0';               // End of variables
  envStr.append("\x01\x00", 2); // Signature for program name follows
  envStr += dosFullPath;
  envStr += '\0';

  for (size_t i = 0; i < envStr.length(); ++i) {
    m_memory.write8(envAddr + i, static_cast<uint8_t>(envStr[i]));
  }
  m_memory.write16(pspAddr + 0x2C, envSegment);
  LOG_INFO("ProgramLoader: Environment block created at segment 0x", std::hex,
           envSegment, ", program path: ", dosFullPath);

  // Command tail size at offset 0x80
  // In DOS, the command tail starts with a space before args.
  // If no args, length = 0 and offset 0x81 = CR.
  if (args.empty()) {
    m_memory.write8(pspAddr + 0x80, 0);    // Length = 0
    m_memory.write8(pspAddr + 0x81, 0x0D); // CR terminator
    LOG_INFO("ProgramLoader: PSP Command Tail: (empty)");
  } else {
    std::string tail = " " + args; // Leading space is required
    uint8_t len = static_cast<uint8_t>(std::min<size_t>(tail.length(), 126));
    m_memory.write8(pspAddr + 0x80, len);
    LOG_INFO("ProgramLoader: PSP Command Tail: '", tail, "' (len=", (int)len,
             ")");
    for (uint8_t i = 0; i < len; ++i) {
      m_memory.write8(pspAddr + 0x81 + i, static_cast<uint8_t>(tail[i]));
    }
    m_memory.write8(pspAddr + 0x81 + len, 0x0D); // CR terminator
  }
}

} // namespace fador::hw
