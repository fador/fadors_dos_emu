#include "../memory/himem/HIMEM.hpp"

namespace fador { namespace hw { memory::HIMEM* g_himem = nullptr; } }
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
    m_himem = std::make_unique<memory::HIMEM>();
    fador::hw::g_himem = m_himem.get();
}

void DOS::initialize() {
    LOG_INFO("DOS: Kernel initialized.");

    // Setup initial MCB chain
    // Block 1: 0x0700 to 0x0FFF (reserved/free)
    MCB first;
    first.type = 'M';
    first.owner = 0x0000;
    first.size = (0x0FFF - 0x0700);
    for (int i = 0; i < 8; ++i) first.name[i] = 0;
    writeMCB(FIRST_MCB_SEGMENT, first);

    // Block 2: 0x0FFF (for block at 0x1000)
    MCB psp;
    psp.type = 'Z';
    psp.owner = 0x1000; // Owned by the program
    psp.size = static_cast<uint16_t>(LAST_PARA - 0x1000 - 1);
    for (int i = 0; i < 8; ++i) psp.name[i] = 0;
    writeMCB(0x1000 - 1, psp);

    LOG_INFO("DOS: Initial MCB chain setup. PSP block at 0x1000, size 0x", std::hex, psp.size, " paras");

    // Default DTA is at offset 0x80 of the initial PSP
    m_dtaPtr = (0x1000 << 16) | 0x0080;
}

bool DOS::handleInterrupt(uint8_t vector) {
    switch (vector) {
        case 0x20: // Terminate
            terminateProcess(0);
            return true;
        case 0x21: // DOS API
            handleDOSService();
            return true;
        case 0x3F: { // Overlay Manager / Generic Proxy
             uint16_t ax = m_cpu.getReg16(cpu::AX);
             uint16_t dx = m_cpu.getReg16(cpu::DX);
             uint32_t eip = m_cpu.getEIP();
             uint16_t cs = m_cpu.getSegReg(cpu::CS);
             uint32_t addr = (cs << 4) + eip;
             
             uint32_t metadata = m_memory.read32(addr);
             LOG_DOS("DOS: INT 3Fh called. AX=0x", std::hex, ax, " DX=0x", dx, 
                     " DS=0x", m_cpu.getSegReg(cpu::DS), " ES=0x", m_cpu.getSegReg(cpu::ES),
                     " at ", cs, ":", eip, " metadata: 0x", metadata);
             
             // Advance EIP to skip metadata (assume 4 bytes for thunk table)
             m_cpu.setEIP(eip + 4);
             return true;
        }
        case 0x2F: { // Multiplex
            uint16_t ax = m_cpu.getReg16(cpu::AX);
            LOG_DOS("DOS: INT 2Fh Multiplex (stubbed), AX=0x", std::hex, ax);
            m_cpu.setReg8(cpu::AL, 0x00); // Not supported
            return true;
        }
    }
    return false;
}

void DOS::handleDOSService() {
    uint8_t ah = m_cpu.getReg8(cpu::AH);
    switch (ah) {
        case 0x02: { // Print Character
            uint8_t c = m_cpu.getReg8(cpu::DL);
            std::cerr << (char)c << std::flush;
            break;
        }
        case 0x06: { // Direct Console I/O
            uint8_t dl = m_cpu.getReg8(cpu::DL);
            if (dl != 0xFF) {
                std::cerr << (char)dl << std::flush;
            } else {
                // Input not implemented, set ZF if no char
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_ZERO);
            }
            break;
        }
        case 0x09: { // Print String
            uint16_t ds = m_cpu.getSegReg(cpu::DS);
            uint16_t dx = m_cpu.getReg16(cpu::DX);
            uint32_t addr = (ds << 4) + dx;
            
            std::string str = readDOSString(addr);
            // Print to real console
            std::cerr << str << std::flush;
            break;
        }
        case 0x1A: { // Set DTA
            uint16_t ds = m_cpu.getSegReg(cpu::DS);
            uint16_t dx = m_cpu.getReg16(cpu::DX);
            m_dtaPtr = (static_cast<uint32_t>(ds) << 16) | dx;
            LOG_DOS("DOS: Set DTA to ", std::hex, ds, ":", dx);
            break;
        }
        case 0x2F: { // Get DTA Address
            uint16_t ds = static_cast<uint16_t>(m_dtaPtr >> 16);
            uint16_t bx = static_cast<uint16_t>(m_dtaPtr & 0xFFFF);
            m_cpu.setSegReg(cpu::ES, ds);
            m_cpu.setReg16(cpu::BX, bx);
            LOG_DOS("DOS: Get DTA -> ", std::hex, ds, ":", bx);
            break;
        }
        case 0x25: { // Set Interrupt Vector
            uint8_t intNum = m_cpu.getReg8(cpu::AL);
            uint16_t ds = m_cpu.getSegReg(cpu::DS);
            uint16_t dx = m_cpu.getReg16(cpu::DX);
            // Write vector
            m_memory.write16(intNum * 4, dx);
            m_memory.write16(intNum * 4 + 2, ds);
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            LOG_DOS("DOS: Set INT vector 0x", std::hex, (int)intNum, " to ", ds, ":", dx);
            break;
        }
        case 0x35: { // Get Interrupt Vector
            uint8_t intNum = m_cpu.getReg8(cpu::AL);
            uint16_t seg = m_memory.read16(intNum * 4 + 2);
            uint16_t off = m_memory.read16(intNum * 4);
            
            m_cpu.setSegReg(cpu::ES, seg);
            m_cpu.setReg16(cpu::BX, off);
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            LOG_DOS("DOS: Get INT vector 0x", std::hex, (int)intNum, " -> ", seg, ":", off);
            break;
        }
        
 
        case 0x2E: { // Set/Reset Verify Flag (Not implemented)
            LOG_DOS("DOS: Set/Reset Verify Flag (AH=2Eh) - Not implemented");
            break;
        }
        case 0x0E: // Select Default Drive
        case 0x19: // Get Current Default Drive
        case 0x36: // Get Free Disk Space
            handleDriveService();
            break;
        case 0x30: { // Get DOS Version
            LOG_DOS("DOS: Get DOS Version (Reported: 5.0)");
            m_cpu.setReg8(cpu::AL, 5); // Major 5
            m_cpu.setReg8(cpu::AH, 0); // Minor 0
            m_cpu.setReg16(cpu::BX, 0xFF00); // OEM
            m_cpu.setReg16(cpu::CX, 0x0000);
            break;
        }
        case 0x3D: { // Open File
            uint16_t ds = m_cpu.getSegReg(cpu::DS);
            uint16_t dx = m_cpu.getReg16(cpu::DX);
            std::string filename = readFilename((ds << 4) + dx);
            uint8_t mode = m_cpu.getReg8(cpu::AL);

            LOG_DOS("DOS: Open file '", filename, "' mode ", (int)mode);

            std::ios_base::openmode openmode = std::ios::binary | std::ios::in;
            if ((mode & 0x03) == 1) openmode = std::ios::binary | std::ios::out;
            if ((mode & 0x03) == 2) openmode = std::ios::binary | std::ios::in | std::ios::out;

            auto handle = std::make_unique<FileHandle>();
            handle->path = filename;
            handle->stream.open(filename, openmode);

            if (handle->stream.is_open()) {
                m_fileHandles.push_back(std::move(handle));
                uint16_t h = static_cast<uint16_t>(m_fileHandles.size() + 4);
                m_cpu.setReg16(cpu::AX, h); // Start handles at 5
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
                LOG_DOS("DOS: Opened '", filename, "' as handle ", h);
            } else {
                m_cpu.setReg16(cpu::AX, 0x02); // File not found
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
                LOG_WARN("DOS: Failed to open '", filename, "'");
            }
            break;
        }
        case 0x3E: { // Close File
            uint16_t h = m_cpu.getReg16(cpu::BX);
            LOG_DOS("DOS: Close handle ", h);
            if (h >= 5 && (h - 5) < static_cast<int>(m_fileHandles.size())) {
                LOG_DOS("DOS: Closing file '", m_fileHandles[h-5]->path, "'");
                m_fileHandles[h - 5]->stream.close();
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            } else {
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            }
            break;
        }
        case 0x3F: { // Read from File or Device
            uint16_t h = m_cpu.getReg16(cpu::BX);
            uint16_t cx = m_cpu.getReg16(cpu::CX); // Count
            uint16_t ds = m_cpu.getSegReg(cpu::DS);
            uint16_t dx = m_cpu.getReg16(cpu::DX);

            // 1. Handle STDIN (Handle 0)
            if (h == 0) {
                // TODO: Wire this to your emulator's keyboard/input buffer.
                // For now, returning 0 bytes read (EOF) or blocking.
                m_cpu.setReg16(cpu::AX, 0);
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
                break;
            }
            
            // Fail on other reserved handles (STDOUT, STDERR) since they are write-only, 
            // or unhandled ones.
            if (h > 0 && h < 5) {
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
                m_cpu.setReg16(cpu::AX, 0x05); // Error 5: Access Denied
                break;
            }

            // 2. Handle User Files (>= 5)
            int fd_index = h - 5;
            if (fd_index >= 0 && fd_index < static_cast<int>(m_fileHandles.size()) && m_fileHandles[fd_index]) {
                if (cx > 0) {
                    std::vector<char> buffer(cx); // Consider a chunked read to avoid large allocs
                    m_fileHandles[fd_index]->stream.read(buffer.data(), cx);
                    std::streamsize read = m_fileHandles[fd_index]->stream.gcount();

                    // 3. Emulate proper 16-bit segment offset wrap-around
                    for (int i = 0; i < read; ++i) {
                        uint16_t offset = dx + i; 
                        uint32_t addr = (ds << 4) + offset;
                        m_memory.write8(addr, static_cast<uint8_t>(buffer[i]));
                    }

                    m_cpu.setReg16(cpu::AX, static_cast<uint16_t>(read));
                } else {
                    m_cpu.setReg16(cpu::AX, 0); // Requested 0 bytes
                }
                
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY); // Success
            } else {
                // 4. Proper error reporting for invalid handles
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
                m_cpu.setReg16(cpu::AX, 0x06); // Error 6: Invalid Handle
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
                LOG_DOS("DOS: Write to ", (h==1?"stdout":"stderr"), ": '", s, "'");
                std::cerr << s << std::flush;
                m_cpu.setReg16(cpu::AX, cx);
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            } else if (h >= 5 && (h - 5) < static_cast<int>(m_fileHandles.size())) {
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

            if (h >= 5 && (h - 5) < static_cast<int>(m_fileHandles.size())) {
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

        case 0x4E: // Find First
        case 0x4F: // Find Next
            handleDirectorySearch();
            break;

        case 0x4B: { // Exec
            uint8_t mode = m_cpu.getReg8(cpu::AL);
            uint16_t ds = m_cpu.getSegReg(cpu::DS);
            uint16_t dx = m_cpu.getReg16(cpu::DX);
            std::string filename = readFilename((ds << 4) + dx);
            LOG_DOS("DOS: Exec '", filename, "' mode ", (int)mode);
            m_cpu.setReg16(cpu::AX, 0x02); // File not found (simplified stub)
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            break;
        }
        case 0x2C: { // Get System Time
            LOG_DOS("DOS: Get System Time (stub)");
            m_cpu.setReg8(cpu::CH, 12); // Hour
            m_cpu.setReg8(cpu::CL, 0);  // Minute
            m_cpu.setReg8(cpu::DH, 0);  // Second
            m_cpu.setReg8(cpu::DL, 0);  // 1/100 second
            break;
        }

        case 0x4C: { // Terminate with return code
            uint8_t al = m_cpu.getReg8(cpu::AL);
            terminateProcess(al);
            break;
        }
        case 0x57: {
            LOG_DOS("DOS: Set/Reset Verify Flag (AH=0x57) - Not implemented");
            break;
        }
        default:
            LOG_WARN("DOS: Unknown INT 21h function AH=0x", std::hex, (int)ah);
            break;
    }
}
void DOS::terminateProcess(uint8_t exitCode) {
    LOG_DOS("DOS: Process terminated with exit code ", (int)exitCode);
    m_terminated = true;
    m_exitCode = exitCode;
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

void DOS::handleDriveService() {
    uint8_t ah = m_cpu.getReg8(cpu::AH);
    if (ah == 0x0E) { // Select Default Drive
        uint8_t drive = m_cpu.getReg8(cpu::DL);
        m_currentDrive = drive;
        LOG_DEBUG("DOS: Selected drive ", (int)m_currentDrive);
        m_cpu.setReg8(cpu::AL, 5); // DOS often returns total logical drives (e.g., 5: A-E)
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else if (ah == 0x19) { // Get Current Default Drive
        m_cpu.setReg8(cpu::AL, m_currentDrive);
        LOG_DEBUG("DOS: Current drive is ", (int)m_currentDrive);
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else if (ah == 0x36) { // Get Free Disk Space
        uint8_t drive = m_cpu.getReg8(cpu::DL);
        if (drive == 0) drive = m_currentDrive + 1;
        
        LOG_DEBUG("DOS: Get Free Disk Space for drive ", (int)drive);
        
        // Mock 1GB total, 500MB free
        // AX = Sectors per cluster
        // BX = Number of free clusters
        // CX = Bytes per sector
        // DX = Total clusters
        m_cpu.setReg16(cpu::AX, 32);     // 32 sectors/cluster (16KB clusters)
        m_cpu.setReg16(cpu::BX, 32768);  // 32768 free clusters (512MB)
        m_cpu.setReg16(cpu::CX, 512);    // 512 bytes/sector
        m_cpu.setReg16(cpu::DX, 65535);  // 65535 total clusters (~1GB)
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    }
}

void DOS::handleDirectorySearch() {
    uint8_t ah = m_cpu.getReg8(cpu::AH);
    uint32_t dtaAddr = ((m_dtaPtr >> 16) << 4) + (m_dtaPtr & 0xFFFF);

    if (ah == 0x4E) { // Find First
        uint16_t ds = m_cpu.getSegReg(cpu::DS);
        uint16_t dx = m_cpu.getReg16(cpu::DX);
        std::string pattern = readFilename((ds << 4) + dx);
        uint8_t attr = m_cpu.getReg8(cpu::CL);

        LOG_DEBUG("DOS: FindFirst '", pattern, "' attr 0x", std::hex, (int)attr);

        // Store pattern in DTA (first 13 bytes)
        for (int i = 0; i < 13; ++i) {
            uint8_t c = (i < static_cast<int>(pattern.length())) ? static_cast<uint8_t>(pattern[i]) : 0;
            m_memory.write8(dtaAddr + i, c);
        }
        // Store index 0 in DTA (bytes 14-15)
        m_memory.write16(dtaAddr + 14, 0);

        // Perform search
        // For now, very simplified: just list the current directory
        std::vector<fs::path> matches;
        try {
            for (const auto& entry : fs::directory_iterator(m_currentDir)) {
                // TODO: Actual glob matching. For now, just match anything or fixed names.
                matches.push_back(entry.path().filename());
            }
        } catch (...) {
            m_cpu.setReg16(cpu::AX, 0x02); // File not found
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            return;
        }

        if (matches.empty()) {
            m_cpu.setReg16(cpu::AX, 0x12); // No more files
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            return;
        }

        // Fill DTA with first match
        auto first = matches[0].string();
        for (int i = 0; i < 13; ++i) {
            uint8_t c = (i < static_cast<int>(first.length())) ? static_cast<uint8_t>(first[i]) : 0;
            m_memory.write8(dtaAddr + 0x1E + i, c);
        }
        m_memory.write32(dtaAddr + 0x1A, 0); // Size (mock)
        
        m_cpu.setReg16(cpu::AX, 0);
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);

    } else if (ah == 0x4F) { // Find Next
        LOG_DEBUG("DOS: FindNext");
        // Read index from DTA
        uint16_t index = m_memory.read16(dtaAddr + 14);
        index++;
        
        // Re-scan and find the indexth match (not efficient but correct for HLE)
        std::vector<fs::path> matches;
        try {
            for (const auto& entry : fs::directory_iterator(m_currentDir)) {
                matches.push_back(entry.path().filename());
            }
        } catch (...) {}

        if (index >= matches.size()) {
            m_cpu.setReg16(cpu::AX, 0x12); // No more files
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            return;
        }

        // Store new index
        m_memory.write16(dtaAddr + 14, index);

        // Fill DTA
        auto next = matches[index].string();
        for (int i = 0; i < 13; ++i) {
            uint8_t c = (static_cast<size_t>(i) < next.length()) ? static_cast<uint8_t>(next[i]) : 0;
            m_memory.write8(dtaAddr + 0x1E + i, c);
        }
        
        m_cpu.setReg16(cpu::AX, 0);
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
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
