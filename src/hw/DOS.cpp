#include "DOS.hpp"
#include "../utils/Logger.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <filesystem>

namespace fs = std::filesystem;

namespace fador::hw {

DOS::DOS(cpu::CPU& cpu, memory::MemoryBus& memory)
    : m_cpu(cpu)
    , m_memory(memory) {
}

void DOS::initialize() {
    LOG_INFO("DOS: Kernel initialized.");

    // Setup initial MCB chain
    // First MCB at 0x0700: 'Z' type (last), owned by 0x0000 (free), 
    // size = TOTAL - 0x0700 - 1 (for MCB itself)
    MCB initial;
    initial.type = 'Z';
    initial.owner = 0x0000;
    initial.size = LAST_PARA - FIRST_MCB_SEGMENT - 1;
    for (int i = 0; i < 8; ++i) initial.name[i] = 0;
    
    writeMCB(FIRST_MCB_SEGMENT, initial);
    LOG_INFO("DOS: Initial MCB chain setup at segment 0x", std::hex, FIRST_MCB_SEGMENT, 
             " size 0x", initial.size, " paras");
}

bool DOS::handleInterrupt(uint8_t vector) {
    switch (vector) {
        case 0x20: // Terminate
            terminateProcess(0);
            return true;
        case 0x21: // DOS API
            handleDOSService();
            return true;
    }
    return false;
}

void DOS::handleDOSService() {
    uint8_t ah = m_cpu.getReg8(cpu::AH);
    switch (ah) {
        case 0x09: { // Print String
            uint16_t ds = m_cpu.getSegReg(cpu::DS);
            uint16_t dx = m_cpu.getReg16(cpu::DX);
            uint32_t addr = (ds << 4) + dx;
            
            std::string str = readDOSString(addr);
            LOG_INFO("DOS: PrintString: ", str);
            // Print to real console
            std::cout << str;
            break;
        }
        case 0x30: { // Get DOS Version
            LOG_DEBUG("DOS: Get DOS Version");
            m_cpu.setReg8(cpu::AL, 5); // Major 5
            m_cpu.setReg8(cpu::AH, 0); // Minor 0
            break;
        }
        case 0x3D: { // Open File
            uint16_t ds = m_cpu.getSegReg(cpu::DS);
            uint16_t dx = m_cpu.getReg16(cpu::DX);
            std::string filename = readFilename((ds << 4) + dx);
            uint8_t mode = m_cpu.getReg8(cpu::AL);

            LOG_DEBUG("DOS: Open file '", filename, "' mode ", (int)mode);

            std::ios_base::openmode openmode = std::ios::binary | std::ios::in;
            if ((mode & 0x03) == 1) openmode = std::ios::binary | std::ios::out;
            if ((mode & 0x03) == 2) openmode = std::ios::binary | std::ios::in | std::ios::out;

            auto handle = std::make_unique<FileHandle>();
            handle->path = filename;
            handle->stream.open(filename, openmode);

            if (handle->stream.is_open()) {
                m_fileHandles.push_back(std::move(handle));
                m_cpu.setReg16(cpu::AX, static_cast<uint16_t>(m_fileHandles.size() + 4)); // Start handles at 5
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            } else {
                m_cpu.setReg16(cpu::AX, 0x02); // File not found
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            }
            break;
        }
        case 0x3E: { // Close File
            uint16_t h = m_cpu.getReg16(cpu::BX);
            LOG_DEBUG("DOS: Close handle ", h);
            if (h >= 5 && (h - 5) < m_fileHandles.size()) {
                m_fileHandles[h - 5]->stream.close();
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            } else {
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            }
            break;
        }
        case 0x3F: { // Read from File
            uint16_t h = m_cpu.getReg16(cpu::BX);
            uint16_t cx = m_cpu.getReg16(cpu::CX); // Count
            uint16_t ds = m_cpu.getSegReg(cpu::DS);
            uint16_t dx = m_cpu.getReg16(cpu::DX);
            uint32_t addr = (ds << 4) + dx;

            if (h >= 5 && (h - 5) < m_fileHandles.size()) {
                std::vector<char> buffer(cx);
                m_fileHandles[h - 5]->stream.read(buffer.data(), cx);
                std::streamsize read = m_fileHandles[h - 5]->stream.gcount();
                for (int i = 0; i < read; ++i) {
                    m_memory.write8(addr + i, static_cast<uint8_t>(buffer[i]));
                }
                m_cpu.setReg16(cpu::AX, static_cast<uint16_t>(read));
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            } else {
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            }
            break;
        }
        case 0x40: { // Write to File
            uint16_t h = m_cpu.getReg16(cpu::BX);
            uint16_t cx = m_cpu.getReg16(cpu::CX);
            uint16_t ds = m_cpu.getSegReg(cpu::DS);
            uint16_t dx = m_cpu.getReg16(cpu::DX);
            uint32_t addr = (ds << 4) + dx;

            if (h == 1 || h == 2) { // Stdout/Stderr
                std::string s;
                for (int i = 0; i < cx; ++i) s += (char)m_memory.read8(addr + i);
                std::cout << s;
                m_cpu.setReg16(cpu::AX, cx);
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            } else if (h >= 5 && (h - 5) < m_fileHandles.size()) {
                std::vector<char> buffer(cx);
                for (int i = 0; i < cx; ++i) buffer[i] = m_memory.read8(addr + i);
                m_fileHandles[h - 5]->stream.write(buffer.data(), cx);
                m_cpu.setReg16(cpu::AX, cx);
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            } else {
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            }
            break;
        }
        case 0x42: { // Lseek
            uint16_t h = m_cpu.getReg16(cpu::BX);
            uint8_t origin = m_cpu.getReg8(cpu::AL);
            int32_t offset = (static_cast<int32_t>(m_cpu.getReg16(cpu::CX)) << 16) | m_cpu.getReg16(cpu::DX);

            if (h >= 5 && (h - 5) < m_fileHandles.size()) {
                std::ios_base::seekdir dir;
                if (origin == 0) dir = std::ios::beg;
                else if (origin == 1) dir = std::ios::cur;
                else dir = std::ios::end;

                m_fileHandles[h - 5]->stream.clear();
                m_fileHandles[h - 5]->stream.seekg(offset, dir);
                m_fileHandles[h - 5]->stream.seekp(offset, dir);

                uint32_t pos = static_cast<uint32_t>(m_fileHandles[h - 5]->stream.tellg());
                m_cpu.setReg16(cpu::DX, static_cast<uint16_t>(pos >> 16));
                m_cpu.setReg16(cpu::AX, static_cast<uint16_t>(pos & 0xFFFF));
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            } else {
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            }
            break;
        }
        case 0x39: // MKDIR
        case 0x3A: // RMDIR
        case 0x3B: // CHDIR
        case 0x47: // GETCWD
            handleDirectoryService();
            break;

        case 0x48: // Allocate Memory
        case 0x49: // Free Memory
        case 0x4A: // Resize Block
            handleMemoryManagement();
            break;

        case 0x4C: { // Terminate with return code
            uint8_t al = m_cpu.getReg8(cpu::AL);
            terminateProcess(al);
            break;
        }
        default:
            LOG_WARN("DOS: Unknown INT 21h function AH=0x", std::hex, (int)ah);
            break;
    }
}

void DOS::terminateProcess(uint8_t exitCode) {
    LOG_INFO("DOS: Process terminated with exit code ", (int)exitCode);
    // In a real emulator, we might halt or jump back to a shell loop
    // For now, we'll just log it.
}

std::string DOS::readDOSString(uint32_t address) {
    std::string result;
    while (true) {
        uint8_t c = m_memory.read8(address++);
        if (c == '$' || result.length() > 255) break; // DOS strings end with '$'
        result += (char)c;
    }
    return result;
}

std::string DOS::readFilename(uint32_t address) {
    std::string result;
    while (true) {
        uint8_t c = m_memory.read8(address++);
        if (c == 0 || result.length() > 255) break; 
        result += (char)c;
    }
    return result;
}


DOS::MCB DOS::readMCB(uint16_t segment) {
    uint32_t addr = (segment << 4);
    MCB mcb;
    mcb.type = m_memory.read8(addr + 0);
    mcb.owner = m_memory.read16(addr + 1);
    mcb.size = m_memory.read16(addr + 3);
    for (int i = 0; i < 8; ++i) mcb.name[i] = (char)m_memory.read8(addr + 8 + i);
    return mcb;
}

void DOS::writeMCB(uint16_t segment, const MCB& mcb) {
    uint32_t addr = (segment << 4);
    m_memory.write8(addr + 0, mcb.type);
    m_memory.write16(addr + 1, mcb.owner);
    m_memory.write16(addr + 3, mcb.size);
    for (int i = 0; i < 8; ++i) m_memory.write8(addr + 8 + i, (uint8_t)mcb.name[i]);
}

void DOS::handleMemoryManagement() {
    uint8_t ah = m_cpu.getReg8(cpu::AH);
    if (ah == 0x48) { // Allocate
        uint16_t requested = m_cpu.getReg16(cpu::BX);
        uint16_t current = FIRST_MCB_SEGMENT;
        uint16_t bestFit = 0;
        uint16_t bestFitSize = 0xFFFF;

        while (true) {
            MCB mcb = readMCB(current);
            if (mcb.owner == 0 && mcb.size >= requested) {
                if (mcb.size < bestFitSize) {
                    bestFit = current;
                    bestFitSize = mcb.size;
                }
            }
            if (mcb.type == 'Z') break;
            current = static_cast<uint16_t>(current + mcb.size + 1);
        }

        if (bestFit != 0) {
            MCB mcb = readMCB(bestFit);
            if (mcb.size > requested + 1) { // Split
                MCB next;
                next.type = mcb.type;
                next.owner = 0;
                next.size = mcb.size - requested - 1;
                
                mcb.type = 'M';
                mcb.size = requested;
                mcb.owner = 0xFFFF; // Temporary owner (will be fixed by whoever calls this if they set ES)
                
                writeMCB(bestFit, mcb);
                writeMCB(static_cast<uint16_t>(bestFit + requested + 1), next);
            } else {
                mcb.owner = 0xFFFF;
                writeMCB(bestFit, mcb);
            }
            m_cpu.setReg16(cpu::AX, bestFit + 1);
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            LOG_DEBUG("DOS: Allocated ", requested, " paras at segment ", std::hex, bestFit + 1);
        } else {
            m_cpu.setReg16(cpu::AX, 0x08); // Insufficient memory
            m_cpu.setReg16(cpu::BX, bestFitSize == 0xFFFF ? 0 : bestFitSize); // Max available
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            LOG_WARN("DOS: Failed to allocate ", requested, " paras");
        }
    } else if (ah == 0x49) { // Free
        uint16_t segment = m_cpu.getSegReg(cpu::ES);
        uint16_t mcbSeg = segment - 1;
        MCB mcb = readMCB(mcbSeg);
        if (mcb.type == 'M' || mcb.type == 'Z') {
            mcb.owner = 0;
            writeMCB(mcbSeg, mcb);
            
            // Simplified merge: only merge with next if free
            if (mcb.type == 'M') {
                uint16_t nextSeg = mcbSeg + mcb.size + 1;
                MCB next = readMCB(nextSeg);
                if (next.owner == 0) {
                    mcb.type = next.type;
                    mcb.size = static_cast<uint16_t>(mcb.size + next.size + 1);
                    writeMCB(mcbSeg, mcb);
                }
            }
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            LOG_DEBUG("DOS: Freed segment ", std::hex, segment);
        } else {
            m_cpu.setReg16(cpu::AX, 0x09); // Memory block address invalid
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            LOG_ERROR("DOS: Failed to free invalid segment ", std::hex, segment);
        }
    } else if (ah == 0x4A) { // Resize
        uint16_t segment = m_cpu.getSegReg(cpu::ES);
        uint16_t requested = m_cpu.getReg16(cpu::BX);
        uint16_t mcbSeg = segment - 1;
        MCB mcb = readMCB(mcbSeg);

        if (mcb.size >= requested) { // Shrink
            if (mcb.size > requested + 1) {
                MCB next;
                next.type = mcb.type;
                next.owner = 0;
                next.size = mcb.size - requested - 1;

                mcb.type = 'M';
                mcb.size = requested;
                
                writeMCB(mcbSeg, mcb);
                writeMCB(static_cast<uint16_t>(mcbSeg + requested + 1), next);
            }
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            LOG_DEBUG("DOS: Shrunk segment ", std::hex, segment, " to ", requested, " paras");
        } else {
            // Expand (only if next is free)
            if (mcb.type == 'M') {
                uint16_t nextSeg = mcbSeg + mcb.size + 1;
                MCB next = readMCB(nextSeg);
                if (next.owner == 0 && (mcb.size + 1 + next.size) >= requested) {
                    uint16_t combinedSize = mcb.size + 1 + next.size;
                    if (combinedSize > requested + 1) {
                        MCB newNext;
                        newNext.type = next.type;
                        newNext.owner = 0;
                        newNext.size = combinedSize - requested - 1;
                        
                        mcb.type = 'M';
                        mcb.size = requested;
                        writeMCB(mcbSeg, mcb);
                        writeMCB(static_cast<uint16_t>(mcbSeg + requested + 1), newNext);
                    } else {
                        mcb.type = next.type;
                        mcb.size = static_cast<uint16_t>(combinedSize);
                        writeMCB(mcbSeg, mcb);
                    }
                    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
                    LOG_DEBUG("DOS: Expanded segment ", std::hex, segment, " to ", requested, " paras");
                    return;
                }
            }
            m_cpu.setReg16(cpu::AX, 0x08); // Insufficient memory
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            LOG_WARN("DOS: Failed to expand segment ", std::hex, segment, " to ", requested, " paras");
        }
    }
}

void DOS::handleDirectoryService() {
    uint8_t ah = m_cpu.getReg8(cpu::AH);
    uint16_t ds = m_cpu.getSegReg(cpu::DS);
    uint16_t dx = m_cpu.getReg16(cpu::DX);
    uint32_t addr = (ds << 4) + dx;

    try {
        if (ah == 0x39) { // MKDIR
            std::string path = readFilename(addr);
            LOG_DEBUG("DOS: MKDIR '", path, "'");
            if (fs::create_directory(path)) {
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
                m_cpu.setReg16(cpu::AX, 0);
            } else {
                m_cpu.setReg16(cpu::AX, 0x03); // Path not found or already exists
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            }
        } else if (ah == 0x3A) { // RMDIR
            std::string path = readFilename(addr);
            LOG_DEBUG("DOS: RMDIR '", path, "'");
            if (fs::remove(path)) {
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
                m_cpu.setReg16(cpu::AX, 0);
            } else {
                m_cpu.setReg16(cpu::AX, 0x05); // Access denied (or not empty/doesn't exist)
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            }
        } else if (ah == 0x3B) { // CHDIR
            std::string path = readFilename(addr);
            LOG_DEBUG("DOS: CHDIR '", path, "'");
            if (fs::exists(path) && fs::is_directory(path)) {
                m_currentDir = path;
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
                m_cpu.setReg16(cpu::AX, 0);
            } else {
                m_cpu.setReg16(cpu::AX, 0x03); // Path not found
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            }
        } else if (ah == 0x47) { // GETCWD
            uint16_t ds_si = m_cpu.getSegReg(cpu::DS);
            uint16_t si = m_cpu.getReg16(cpu::SI);
            uint32_t outAddr = (ds_si << 4) + si;
            
            // DOS returns path relative to drive (no leading slash, no drive letter)
            // For now we just return our tracked currentDir
            std::string path = m_currentDir;
            if (path == ".") path = "";
            
            LOG_DEBUG("DOS: GETCWD returning '", path, "'");
            for (size_t i = 0; i < path.length(); ++i) {
                m_memory.write8(outAddr + i, static_cast<uint8_t>(path[i]));
            }
            m_memory.write8(outAddr + path.length(), 0); // Null terminator
            
            m_cpu.setReg16(cpu::AX, 0x0100); // Success (AX=0 is also common but AH=0 is enough)
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
        }
    } catch (const std::exception& e) {
        LOG_ERROR("DOS Directory Error: ", e.what());
        m_cpu.setReg16(cpu::AX, 0x05); // General failure
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    }
}

void DOS::dumpMCBChain() {
    uint16_t current = FIRST_MCB_SEGMENT;
    LOG_INFO("--- DOS MCB Chain Dump ---");
    while (true) {
        MCB mcb = readMCB(current);
        LOG_INFO("MCB [", (char)mcb.type, "] Seg: 0x", std::hex, current, 
                 " Owner: 0x", mcb.owner, " Size: ", std::dec, mcb.size, " paras");
        if (mcb.type == 'Z') break;
        current = static_cast<uint16_t>(current + mcb.size + 1);
        if (current >= LAST_PARA) break;
    }
}


} // namespace fador::hw
