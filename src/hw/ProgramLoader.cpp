#include "ProgramLoader.hpp"
#include "../utils/Logger.hpp"
#include <fstream>
#include <vector>

namespace fador::hw {

ProgramLoader::ProgramLoader(cpu::CPU& cpu, memory::MemoryBus& memory)
    : m_cpu(cpu), m_memory(memory) {
}

bool ProgramLoader::loadCOM(const std::string& path, uint16_t segment) {
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
    createPSP(segment);

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

bool ProgramLoader::loadEXE(const std::string& path, uint16_t segment) {
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

    // Load actual program image
    uint32_t imageOffset = header.headerSize * 16;
    file.seekg(imageOffset, std::ios::beg);

    uint32_t imageSize = (header.numPages * 512) - (header.lastPageSize ? (512 - header.lastPageSize) : 0) - imageOffset;
    
    // For now, let's just use a simple load to segment:0000 (after PSP)
    // Actually, EXE load usually puts PSP at segment, and image at segment + 10h (256 bytes)
    createPSP(segment);
    uint16_t loadSegment = segment + 0x10; // Image starts after 256-byte PSP
    uint32_t loadAddr = (loadSegment << 4);

    std::vector<uint8_t> buffer(imageSize);
    if (!file.read(reinterpret_cast<char*>(buffer.data()), imageSize)) {
        LOG_ERROR("ProgramLoader: Failed to read EXE image");
        return false;
    }

    for (uint32_t i = 0; i < imageSize; ++i) {
        m_memory.write8(loadAddr + i, buffer[i]);
    }

    // Relocations
    if (header.numReloc > 0) {
        file.seekg(header.relocOffset, std::ios::beg);
        for (int i = 0; i < header.numReloc; ++i) {
            uint16_t offset, relSegment;
            file.read(reinterpret_cast<char*>(&offset), 2);
            file.read(reinterpret_cast<char*>(&relSegment), 2);
            
            uint32_t relocAddr = ((loadSegment + relSegment) << 4) + offset;
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

void ProgramLoader::createPSP(uint16_t segment) {
    uint32_t pspAddr = (segment << 4);
    // INT 20h instruction (CD 20) at offset 0
    m_memory.write8(pspAddr + 0x00, 0xCD);
    m_memory.write8(pspAddr + 0x01, 0x20);

    // Segment of top of memory at offset 2 (stubbed to 640KB)
    m_memory.write16(pspAddr + 0x02, 0xA000); 

    // Command tail size (stubbed)
    m_memory.write8(pspAddr + 0x80, 0);
}

} // namespace fador::hw
