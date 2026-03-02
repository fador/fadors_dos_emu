#include "ProgramLoader.hpp"
#include "../utils/Logger.hpp"
#include <fstream>
#include <vector>
#include "../memory/himem/HIMEM.hpp"
#include "DOS.hpp"

namespace fador::hw {

ProgramLoader::ProgramLoader(cpu::CPU& cpu, memory::MemoryBus& memory)
    : m_cpu(cpu), m_memory(memory) {
}

bool ProgramLoader::loadCOM(const std::string& path, uint16_t segment, const std::string& args) {
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
    createPSP(segment, args);

    // 2. Load file data at segment:0100
    uint32_t loadAddr = (segment << 4) + 0x100;
    std::vector<uint8_t> buffer(static_cast<size_t>(size));
    if (file.read(reinterpret_cast<char*>(buffer.data()), size)) {
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

    LOG_INFO("ProgramLoader: Successfully loaded .COM file: ", path, " at ", std::hex, segment, ":0100");
    return true;
}

bool ProgramLoader::loadEXE(const std::string& path, uint16_t segment, DOS& dos, const std::string& args, bool useHimem) {
    // Basic MZ Parsing
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        LOG_ERROR("ProgramLoader: Failed to open .EXE file: ", path);
        return false;
    }

    struct MZHeader {
        uint16_t signature;      // 'MZ'
        uint16_t lastPageSize;
        uint16_t numPages;
        uint16_t numReloc;
        uint16_t headerSize;    // In paragraphs (16 bytes)
        uint16_t minAlloc;
        uint16_t maxAlloc;
        uint16_t ss;            // Initial SS (relative to load)
        uint16_t sp;            // Initial SP
        uint16_t checksum;
        uint16_t ip;            // Initial IP
        uint16_t cs;            // Initial CS (relative to load)
        uint16_t relocOffset;
        uint16_t overlayNum;
    } header;

    if (!file.read(reinterpret_cast<char*>(&header), sizeof(header))) {
        LOG_ERROR("ProgramLoader: Failed to read EXE header");
        return false;
    }

    if (header.signature != 0x5A4D) { // 'ZM' or 'MZ' (LE)
        LOG_ERROR("ProgramLoader: Invalid EXE signature: 0x", std::hex, header.signature);
        return false;
    }

    LOG_INFO("ProgramLoader: MZ Stack initial SS:SP = ", std::hex, header.ss, ":", header.sp);
    LOG_INFO("ProgramLoader: MZ Entry CS:IP = ", std::hex, header.cs, ":", header.ip);

    // Check for NE header offset at 0x3C
    file.seekg(0x3C, std::ios::beg);
    uint16_t neOffset = 0;
    file.read(reinterpret_cast<char*>(&neOffset), sizeof(neOffset));

    if (neOffset > 0) {
        file.seekg(neOffset, std::ios::beg);
        uint16_t neSig = 0;
        file.read(reinterpret_cast<char*>(&neSig), sizeof(neSig));
        if (neSig == 0x454E) { // 'NE'
            LOG_INFO("ProgramLoader: NE Header found at 0x", std::hex, neOffset);
            
            file.seekg(neOffset + 0x0E, std::ios::beg);
            uint16_t appFlags = 0;
            file.read(reinterpret_cast<char*>(&appFlags), sizeof(appFlags));
            
            file.seekg(neOffset + 0x36, std::ios::beg);
            uint8_t targetOS = 0;
            file.read(reinterpret_cast<char*>(&targetOS), sizeof(targetOS));
            LOG_INFO("ProgramLoader: NE Header AppFlags=0x", std::hex, appFlags, " TargetOS=0x", (int)targetOS);

            file.seekg(neOffset + 0x14, std::ios::beg);
            uint16_t entryIP = 0;
            file.read(reinterpret_cast<char*>(&entryIP), sizeof(entryIP));
            uint16_t entrySegIdx = 0;
            file.read(reinterpret_cast<char*>(&entrySegIdx), sizeof(entrySegIdx));
            LOG_INFO("ProgramLoader: NE Entry Point: SegIdx=", std::dec, entrySegIdx, " IP=0x", std::hex, entryIP);
            
            // Read NE Header fields for VROOMM
            file.seekg(neOffset + 0x1C, std::ios::beg);
            uint16_t numSegments = 0;
            file.read(reinterpret_cast<char*>(&numSegments), sizeof(numSegments));
            
            file.seekg(neOffset + 0x22, std::ios::beg);
            uint16_t segTableOffset = 0;
            file.read(reinterpret_cast<char*>(&segTableOffset), sizeof(segTableOffset));
            
            file.seekg(neOffset + 0x32, std::ios::beg);
            uint16_t alignShift = 0;
            file.read(reinterpret_cast<char*>(&alignShift), sizeof(alignShift));
            
            // Read more tables for debugging relocations
            file.seekg(neOffset + 0x1E, std::ios::beg);
            uint16_t numModRefs = 0;
            file.read(reinterpret_cast<char*>(&numModRefs), 2);
            
            file.seekg(neOffset + 0x26, std::ios::beg);
            uint16_t modRefTableOff = 0;
            file.read(reinterpret_cast<char*>(&modRefTableOff), 2);
            
            file.seekg(neOffset + 0x28, std::ios::beg);
            uint16_t importTableOff = 0;
            file.read(reinterpret_cast<char*>(&importTableOff), 2);
            
            LOG_INFO("ProgramLoader: NE Modules: ", numModRefs, " ModRefOff: 0x", std::hex, modRefTableOff, " ImportOff: 0x", importTableOff);
            
            for (int i = 0; i < numModRefs; ++i) {
                file.seekg(neOffset + modRefTableOff + (i * 2), std::ios::beg);
                uint16_t nameOff = 0;
                file.read(reinterpret_cast<char*>(&nameOff), 2);
                file.seekg(neOffset + importTableOff + nameOff, std::ios::beg);
                uint8_t len = 0;
                file.read(reinterpret_cast<char*>(&len), 1);
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
            file.read(reinterpret_cast<char*>(&lastPageSize), 2);
            uint16_t numPages = 0;
            file.read(reinterpret_cast<char*>(&numPages), 2);
            uint32_t mzSize = (numPages - 1) * 512 + (lastPageSize == 0 ? 512 : lastPageSize);
            LOG_INFO("ProgramLoader: MZ Header reports image size 0x", std::hex, mzSize, " bytes");

            file.seekg(10, std::ios::beg);
            uint16_t minAlloc = 0;
            file.read(reinterpret_cast<char*>(&minAlloc), 2);
            uint16_t maxAlloc = 0;
            file.read(reinterpret_cast<char*>(&maxAlloc), 2);
            LOG_INFO("ProgramLoader: MZ Header minAlloc=0x", std::hex, minAlloc, " maxAlloc=0x", maxAlloc);

            file.seekg(neOffset + segTableOffset, std::ios::beg);
            for (int i = 0; i < numSegments; ++i) {
                DOS::NESegment seg;
                file.read(reinterpret_cast<char*>(&seg.fileOffsetSector), 2);
                file.read(reinterpret_cast<char*>(&seg.length), 2);
                file.read(reinterpret_cast<char*>(&seg.flags), 2);
                file.read(reinterpret_cast<char*>(&seg.minAlloc), 2);
                
                uint32_t fileOff = (uint32_t)seg.fileOffsetSector << alignShift;

                if (fileOff > 0 && fileOff < mzSize) {
                    uint32_t mzBaseIdx = (uint32_t)header.headerSize * 16;
                    if (fileOff >= mzBaseIdx) {
                        seg.loadedSegment = (segment + 0x10) + (uint16_t)((fileOff - mzBaseIdx) / 16);
                        LOG_INFO("ProgramLoader: Segment ", i + 1, " marked as resident at 0x", std::hex, seg.loadedSegment);
                    } else {
                        seg.loadedSegment = 0;
                    }
                } else {
                    seg.loadedSegment = 0;
                }
               segments.push_back(seg);
                LOG_INFO("ProgramLoader: Segment ", std::dec, i + 1, ": sector=0x", std::hex, seg.fileOffsetSector, 
                         " len=0x", seg.length, " flags=0x", seg.flags, " minAlloc=0x", seg.minAlloc);
            }
            
            dos.setNEInfo(path, alignShift, segments, segment + 0x10);
            LOG_INFO("ProgramLoader: VROOMM Info set. Segments: ", std::dec, numSegments, " AlignShift: ", alignShift);
        }
    }

    // Load actual program image
    uint32_t imageOffset = header.headerSize * 16;
    file.seekg(0, std::ios::end);
    uint32_t fileSize = file.tellg();
    file.seekg(imageOffset, std::ios::beg);
    uint32_t imageSize = fileSize - imageOffset;
    
    // For now, let's just use a simple load to segment:0000 (after PSP)
    // Actually, EXE load usually puts PSP at segment, and image at segment + 10h (256 bytes)
    createPSP(segment, args);

    uint16_t loadSegment = segment + 0x10; // Image starts after 256-byte PSP
    uint32_t loadAddr = (loadSegment << 4);

    std::vector<uint8_t> buffer(imageSize);
    if (!file.read(reinterpret_cast<char*>(buffer.data()), imageSize)) {
        LOG_ERROR("ProgramLoader: Failed to read EXE image");
        return false;
    }

    if (loadAddr + imageSize > memory::MemoryBus::MEMORY_SIZE) {
        if (useHimem) {
            // Try to use XMS (HIMEM)
            // Find global HIMEM instance (hack: static pointer, or pass in constructor in future)
            extern fador::memory::HIMEM* g_himem;
            if (g_himem && g_himem->available() >= imageSize) {
                uint16_t handle = g_himem->allocate(imageSize);
                uint8_t* xmsPtr = g_himem->getBlock(handle);
                if (xmsPtr) {
                    std::copy(buffer.begin(), buffer.end(), xmsPtr);
                    LOG_WARN("ProgramLoader: Loaded EXE into XMS (HIMEM) at handle ", handle, ". This is not true DOS behavior.");
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
            LOG_ERROR("ProgramLoader: EXE image too large for DOS memory: loadAddr=0x", std::hex, loadAddr, " size=0x", imageSize);
            return false;
        }
    } else {
        for (uint32_t i = 0; i < imageSize; ++i) {
            m_memory.write8(loadAddr + i, buffer[i]);
        }
    }

    // Relocations
    if (header.numReloc > 0) {
        file.seekg(header.relocOffset, std::ios::beg);
        for (int i = 0; i < header.numReloc; ++i) {
            uint16_t offset, relSegment;
            file.read(reinterpret_cast<char*>(&offset), 2);
            file.read(reinterpret_cast<char*>(&relSegment), 2);
            uint32_t relocAddr = ((loadSegment + relSegment) << 4) + offset;
            if (relocAddr + 1 >= memory::MemoryBus::MEMORY_SIZE) {
                LOG_ERROR("ProgramLoader: Relocation address out of DOS memory: 0x", std::hex, relocAddr);
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

    LOG_INFO("ProgramLoader: Successfully loaded .EXE file: ", path, " at CS:EIP=", std::hex, m_cpu.getSegReg(cpu::CS), ":", m_cpu.getEIP());
    return true;
}

void ProgramLoader::createPSP(uint16_t segment, const std::string& args) {
    uint32_t pspAddr = (segment << 4);
    // INT 20h instruction (CD 20) at offset 0
    m_memory.write8(pspAddr + 0x00, 0xCD);
    m_memory.write8(pspAddr + 0x01, 0x20);

    // Segment of top of memory at offset 2 (stubbed to 640KB)
    m_memory.write16(pspAddr + 0x02, 0xA000); 

    // Environment block segment at offset 0x2C
    // Let's place it at segment + 0x08 (dummy small block before code)
    uint16_t envSegment = segment + 0x08;
    uint32_t envAddr = (envSegment << 4);
    // Format: VAR=VAL\0\0\0\x01\x00PROGRAM_PATH\0
    std::string envStr = "PATH=C:\\TCC\0";
    envStr += "LIB=C:\\TCC\\LIB\0";
    envStr += "INCLUDE=C:\\TCC\\INCLUDE\0";
    envStr += "\0"; // End of variables
    envStr += "\x01\x00"; // Signature for program name follows
    envStr += "C:\\TCC\\TCC.EXE\0";
    
    for (size_t i = 0; i < envStr.length(); ++i) {
        m_memory.write8(envAddr + i, static_cast<uint8_t>(envStr[i]));
    }
    m_memory.write16(pspAddr + 0x2C, envSegment);
    LOG_INFO("ProgramLoader: Environment block created at segment 0x", std::hex, envSegment);

    // Command tail size at offset 0x80
    std::string tail = " " + args; // Leading space is required
    uint8_t len = static_cast<uint8_t>(std::min<size_t>(tail.length(), 126));
    m_memory.write8(pspAddr + 0x80, len);
    LOG_INFO("ProgramLoader: PSP Command Tail: '", tail, "' (len=", (int)len, ")");
    
    // Command tail at offset 0x81
    for (uint8_t i = 0; i < len; ++i) {
        m_memory.write8(pspAddr + 0x81 + i, static_cast<uint8_t>(tail[i]));
    }
    m_memory.write8(pspAddr + 0x81 + len, 0x0D); // CR terminator
}

} // namespace fador::hw
