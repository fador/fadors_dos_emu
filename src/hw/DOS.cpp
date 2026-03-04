#include "../memory/himem/HIMEM.hpp"

namespace fador { namespace hw { memory::HIMEM* g_himem = nullptr; } }
#include "DOS.hpp"
#include "KeyboardController.hpp"
#include "../utils/Logger.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <thread>

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
    // Block 1: 0x0700 to 0x0FFE (reserved/free)
    MCB first;
    first.type = 'M';
    first.owner = 0x0000;
    first.size = (0x0FFF - 0x0700 - 1); // 0x8FE paras
    for (int i = 0; i < 8; ++i) first.name[i] = 0;
    writeMCB(FIRST_MCB_SEGMENT, first);
    m_pspSegment = 0x1000;

    // Block 2: 0x0FFF (for block at 0x1000)
    MCB psp;
    psp.type = 'M'; // Not the last block anymore
    psp.owner = m_pspSegment; 
    psp.size = 0x7000; // 448KB (+ PSP = 512KB total)
    for (int i = 0; i < 8; ++i) psp.name[i] = 0;
    writeMCB(0x1000 - 1, psp);

    // Block 3: The rest of memory as free
    MCB freeRest;
    freeRest.type = 'Z'; // Last block
    freeRest.owner = 0;
    freeRest.size = static_cast<uint16_t>(LAST_PARA - (0x1000 + 0x7000) - 1);
    for (int i = 0; i < 8; ++i) freeRest.name[i] = 0;
    writeMCB(0x1000 + 0x7000, freeRest);

    LOG_INFO("DOS: Initial MCB chain setup. PSP block at 0x1000 size 0x7000, Free block at 0x", std::hex, 0x1000 + 0x7000 + 1, " size 0x", freeRest.size);

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
        case 0x3F: { // Overlay Manager / Generic Proxy (VROOMM)
             uint16_t cs = m_cpu.getSegReg(cpu::CS);
             uint32_t eip = m_cpu.getEIP();
             uint32_t addr = (cs << 4) + eip;
             
             // Metadata follows INT 3F: [uint16_t offset, uint16_t segmentIndex]
             uint16_t targetOffset = m_memory.read16(addr);
             uint16_t segIndex = m_memory.read16(addr + 2);
             LOG_DOS("DOS: VROOMM thunk raw: ", std::hex, m_memory.read8(addr), " ", m_memory.read8(addr+1), " ", 
                     m_memory.read8(addr+2), " ", m_memory.read8(addr+3));
             LOG_DOS("DOS: VROOMM interpreted: segIndex=", std::dec, segIndex, " (0x", std::hex, segIndex, ") offset=0x", targetOffset);
             
             // Borland thunks natively use 1-based segment indices (same as NE specs)
             // Segment 1 is often resident, thunks usually refer to segments 2+
             uint16_t internalSegIdx = (segIndex > 0) ? segIndex - 1 : 0;
            if (internalSegIdx >= m_neSegments.size()) {
                 LOG_ERROR("DOS: VROOMM invalid segment index ", std::dec, segIndex, " (max=", m_neSegments.size(), ")");
                 return true;
             }

             uint16_t loadedSeg = loadOverlaySegment(internalSegIdx);
             if (loadedSeg == 0) {
                 LOG_ERROR("DOS: VROOMM failed to load segment index ", std::dec, segIndex);
                 return true;
             }

             // Patch the thunk at [CS:EIP-2] with a JMP FAR (EA OO OO SS SS)
             uint32_t thunkStart = (cs << 4) + (eip - 2);
             m_memory.write8(thunkStart, 0xEA); // JMP far
             m_memory.write16(thunkStart + 1, targetOffset);
             m_memory.write16(thunkStart + 3, loadedSeg);
             
             LOG_DOS("DOS: Thunk at ", std::hex, cs, ":", (eip-2), " patched to JMP ", loadedSeg, ":", targetOffset);

             // Rewind EIP to re-execute the patched instruction
             m_cpu.setEIP(eip - 2);
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
    LOG_DEBUG("[DOS] INT 21h AH=0x", std::hex, (int)ah, " AL=0x", (int)m_cpu.getReg8(cpu::AL));
    switch (ah) {
        case 0x01: { // Read Character with Echo (blocking)
            if (!m_kbd || !m_kbd->hasKey()) {
                if (m_pollInput) m_pollInput();
                if (!m_kbd || !m_kbd->hasKey()) {
                    m_cpu.setEIP(m_cpu.getEIP() - 2);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    return;
                }
            }
            {
                auto [ascii, scancode] = m_kbd->popKey();
                m_cpu.setReg8(cpu::AL, ascii);
                if (ascii >= 0x20) writeCharToVRAM(ascii);
            }
            break;
        }
        case 0x02: { // Print Character
            uint8_t c = m_cpu.getReg8(cpu::DL);
            writeCharToVRAM(c);
            break;
        }
        case 0x06: { // Direct Console I/O
            uint8_t dl = m_cpu.getReg8(cpu::DL);
            if (dl != 0xFF) {
                writeCharToVRAM(dl);
            } else {
                if (m_kbd && !m_kbd->hasKey() && m_pollInput) m_pollInput();
                if (m_kbd && m_kbd->hasKey()) {
                    auto [ascii, scancode] = m_kbd->popKey();
                    m_cpu.setReg8(cpu::AL, ascii);
                    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_ZERO);
                } else {
                    m_cpu.setReg8(cpu::AL, 0);
                    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_ZERO);
                }
            }
            break;
        }
        case 0x07: // Direct Character Input (no echo, no Ctrl-C check)
        case 0x08: { // Character Input (no echo, Ctrl-C check)
            if (!m_kbd || !m_kbd->hasKey()) {
                if (m_pollInput) m_pollInput();
                if (!m_kbd || !m_kbd->hasKey()) {
                    m_cpu.setEIP(m_cpu.getEIP() - 2);
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    return;
                }
            }
            {
                auto [ascii, scancode] = m_kbd->popKey();
                m_cpu.setReg8(cpu::AL, ascii);
            }
            break;
        }
        case 0x09: { // Print String
            uint16_t ds = m_cpu.getSegReg(cpu::DS);
            uint16_t dx = m_cpu.getReg16(cpu::DX);
            uint32_t addr = (ds << 4) + dx;
            
            std::string str = readDOSString(addr);
            for (uint8_t c : str) writeCharToVRAM(c);
            break;
        }
        case 0x0A: { // Buffered Keyboard Input
            uint16_t ds = m_cpu.getSegReg(cpu::DS);
            uint16_t dx = m_cpu.getReg16(cpu::DX);
            uint32_t bufAddr = (ds << 4) + dx;
            uint8_t maxLen = m_memory.read8(bufAddr); // First byte = max chars
            // For now, store empty input (0 chars read, CR terminated)
            m_memory.write8(bufAddr + 1, 0);    // Actual chars read = 0
            m_memory.write8(bufAddr + 2, 0x0D); // CR terminator
            LOG_DOS("DOS: Buffered Input (stubbed, max=", (int)maxLen, ")");
            break;
        }
        case 0x0B: { // Get Stdin Status
            if (m_kbd && !m_kbd->hasKey() && m_pollInput) m_pollInput();
            m_cpu.setReg8(cpu::AL, (m_kbd && m_kbd->hasKey()) ? 0xFF : 0x00);
            break;
        }
        case 0x0C: { // Flush Buffer and Read
            if (m_kbd) {
                while (m_kbd->hasKey()) m_kbd->popKey();
            }
            uint8_t subFunc = m_cpu.getReg8(cpu::AL);
            if (subFunc == 0x01 || subFunc == 0x06 || subFunc == 0x07
                || subFunc == 0x08 || subFunc == 0x0A) {
                m_cpu.setReg8(cpu::AH, subFunc);
                handleDOSService();
            }
            break;
        }
        case 0x1A: { // Set DTA
            uint16_t ds = m_cpu.getSegReg(cpu::DS);
            uint16_t dx = m_cpu.getReg16(cpu::DX);
            m_dtaPtr = (static_cast<uint32_t>(ds) << 16) | dx;
            LOG_DOS("DOS: Set DTA to ", std::hex, ds, ":", dx);
            break;
        }
        case 0x2A: { // Get System Date
            // Return current date
            std::time_t t = std::time(nullptr);
            std::tm* now = std::localtime(&t);
            m_cpu.setReg16(cpu::CX, now->tm_year + 1900); // Year
            m_cpu.setReg8(cpu::DH, now->tm_mon + 1);     // Month
            m_cpu.setReg8(cpu::DL, now->tm_mday);        // Day
            m_cpu.setReg8(cpu::AL, now->tm_wday);        // Day of week (0=Sun, 1=Mon, ..., 6=Sat)
            break;
        }
        case 0x2B: { // Set System Date
            // Accept but ignore
            m_cpu.setReg8(cpu::AL, 0); // Success
            LOG_DOS("DOS: Set Date (ignored)");
            break;
        }
        case 0x51:  // Get Current PSP (undocumented)
        case 0x62: { // Get Current PSP
            m_cpu.setReg16(cpu::BX, m_pspSegment);
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            LOG_DOS("DOS: Get Current PSP -> 0x", std::hex, m_pspSegment);
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
        case 0x37: { // Get/Set Switch Character
            uint8_t al = m_cpu.getReg8(cpu::AL);
            if (al == 0x00) {
                // Get switch character
                m_cpu.setReg8(cpu::DL, '/'); // Standard DOS switch char
                m_cpu.setReg8(cpu::AL, 0x00); // Success
                LOG_DOS("DOS: Get Switch Character -> '/'");
            } else if (al == 0x01) {
                // Set switch character — accept but ignore
                m_cpu.setReg8(cpu::AL, 0x00);
                LOG_DOS("DOS: Set Switch Character (ignored)");
            } else {
                m_cpu.setReg8(cpu::AL, 0xFF); // Invalid subfunction
            }
            break;
        }
        case 0x33: { // Get/Set Ctrl-Break Check State
            uint8_t al = m_cpu.getReg8(cpu::AL);
            if (al == 0x00) {
                // Get current state
                m_cpu.setReg8(cpu::DL, m_ctrlBreakCheck ? 1 : 0);
                LOG_DOS("DOS: Get Ctrl-Break Check -> ", (int)m_ctrlBreakCheck);
            } else if (al == 0x01) {
                // Set state
                m_ctrlBreakCheck = (m_cpu.getReg8(cpu::DL) != 0);
                LOG_DOS("DOS: Set Ctrl-Break Check -> ", (int)m_ctrlBreakCheck);
            } else if (al == 0x05) {
                // Get boot drive
                m_cpu.setReg8(cpu::DL, 3); // C: drive
                LOG_DOS("DOS: Get Boot Drive -> C:");
            }
            break;
        }
        case 0x30: { // Get DOS Version
            LOG_DOS("DOS: Get DOS Version (Reported: 5.0)");
            m_cpu.setReg8(cpu::AL, 5); // Major 5
            m_cpu.setReg8(cpu::AH, 0); // Minor 0
            m_cpu.setReg16(cpu::BX, 0xFF00); // OEM
            m_cpu.setReg16(cpu::CX, 0x0000);
            break;
        }
        case 0x3C: { // Create File
            uint16_t ds = m_cpu.getSegReg(cpu::DS);
            uint16_t dx = m_cpu.getReg16(cpu::DX);
            uint32_t nameAddr = (ds << 4) + dx;
            std::string filename = resolvePath(readFilename(nameAddr));

            LOG_DOS("DOS: Create file '", filename, "'");

            auto handle = std::make_unique<FileHandle>();
            handle->path = filename;
            // Create/truncate the file for read+write
            handle->stream.open(filename, std::ios::binary | std::ios::in | std::ios::out | std::ios::trunc);
            if (!handle->stream.is_open()) {
                // File may not exist yet; create it then reopen for r/w
                { std::ofstream touch(filename, std::ios::binary); }
                handle->stream.open(filename, std::ios::binary | std::ios::in | std::ios::out | std::ios::trunc);
            }

            if (handle->stream.is_open()) {
                m_fileHandles.push_back(std::move(handle));
                uint16_t h = static_cast<uint16_t>(m_fileHandles.size() + 4);
                m_cpu.setReg16(cpu::AX, h);
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
                LOG_DOS("DOS: Created '", filename, "' as handle ", h);
            } else {
                m_cpu.setReg16(cpu::AX, 0x03); // Path not found
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
                LOG_WARN("DOS: Failed to create '", filename, "'");
            }
            break;
        }
        case 0x3D: { // Open File
            uint16_t ds = m_cpu.getSegReg(cpu::DS);
            uint16_t dx = m_cpu.getReg16(cpu::DX);
            uint32_t nameAddr = (ds << 4) + dx;
            std::string filename = resolvePath(readFilename(nameAddr));
            uint8_t mode = m_cpu.getReg8(cpu::AL);

            LOG_DOS("DOS: Open file '", filename, "' at 0x", std::hex, nameAddr, " mode ", (int)mode);

            // Reject directories — DOS cannot open directories as files
            if (fs::is_directory(filename)) {
                m_cpu.setReg16(cpu::AX, 0x05); // Access Denied
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
                LOG_DOS("DOS: Rejected directory open '", filename, "'");
                break;
            }

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

                    LOG_DEBUG("[DOS] AH=3Fh Read handle=", h, " cx=", cx, " read=", (int)read, " dest=", std::hex, ds, ":", dx, " (flat 0x", ((ds << 4) + dx), ")");

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
                for (int i = 0; i < cx; ++i) {
                    uint8_t c = m_memory.read8(addr + i);
                    writeCharToVRAM(c);
                }
                LOG_DOS("DOS: Write to ", (h==1?"stdout":"stderr"), " ", cx, " bytes");
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

            LOG_DEBUG("[DOS] Seek handle=", h, " origin=", (int)origin, " offset=", offset, " CX=", std::hex, m_cpu.getReg16(cpu::CX), " DX=", m_cpu.getReg16(cpu::DX));

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

        case 0x43: { // Get/Set File Attributes (CHMOD)
            uint8_t  al = m_cpu.getReg8(cpu::AL);
            uint16_t ds = m_cpu.getSegReg(cpu::DS);
            uint16_t dx = m_cpu.getReg16(cpu::DX);
            std::string filename = resolvePath(readFilename((static_cast<uint32_t>(ds) << 4) + dx));
            LOG_DOS("DOS: Get/Set Attributes AL=", (int)al, " file='", filename, "'");

            if (al == 0x00) { // Get attributes
                std::error_code ec;
                bool exists = fs::exists(filename, ec);
                if (!exists || ec) {
                    m_cpu.setReg16(cpu::AX, 0x02); // File not found
                    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
                } else {
                    uint16_t attr = 0x20; // Archive bit
                    if (fs::is_directory(filename, ec)) attr = 0x10;
                    auto perms = fs::status(filename, ec).permissions();
                    using p = fs::perms;
                    if ((perms & p::owner_write) == p::none) attr |= 0x01; // Read-only
                    m_cpu.setReg16(cpu::CX, attr);
                    m_cpu.setReg16(cpu::AX, attr); // Some programs check AX too
                    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
                    LOG_DOS("DOS: Attributes for '", filename, "' = 0x", std::hex, attr);
                }
            } else if (al == 0x01) { // Set attributes
                uint16_t cx = m_cpu.getReg16(cpu::CX);
                std::error_code ec;
                if (!fs::exists(filename, ec)) {
                    m_cpu.setReg16(cpu::AX, 0x02);
                    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
                } else {
                    // Only honour read-only bit mapping
                    auto perms = fs::status(filename, ec).permissions();
                    using p = fs::perms;
                    if (cx & 0x01) { // Read-only: remove write permission
                        fs::permissions(filename,
                            perms & ~(p::owner_write | p::group_write | p::others_write), ec);
                    } else {          // Writable: restore owner write
                        fs::permissions(filename, perms | p::owner_write, ec);
                    }
                    m_cpu.setReg16(cpu::AX, 0);
                    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
                }
            } else {
                m_cpu.setReg16(cpu::AX, 0x01); // Invalid function
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            }
            break;
        }
        case 0x44: { // IOCTL
            uint8_t al = m_cpu.getReg8(cpu::AL);
            uint16_t bx = m_cpu.getReg16(cpu::BX);
            if (al == 0x00) { // Get Device Info
                // Return device info word: bit 7=1 means character device (stdin/stdout/stderr)
                uint16_t info = 0;
                if (bx == 0) info = 0x80 | 0x01; // STDIN  - char device, EOF on input
                else if (bx == 1) info = 0x80 | 0x02; // STDOUT - char device
                else if (bx == 2) info = 0x80 | 0x02; // STDERR - char device
                else info = 0x0002;  // Disk file (bit 1=non-removable)
                m_cpu.setReg16(cpu::DX, info);
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
                LOG_DOS("DOS: IOCTL Get Device Info handle=", bx, " info=0x", std::hex, info);
            } else {
                // Other IOCTL subfunction – stub as unsupported
                m_cpu.setReg16(cpu::AX, 0x01); // Function not supported
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
                LOG_DOS("DOS: IOCTL AL=", (int)al, " not implemented");
            }
            break;
        }
        case 0x4B: { // Exec
            uint8_t mode = m_cpu.getReg8(cpu::AL);
            uint16_t ds = m_cpu.getSegReg(cpu::DS);
            uint16_t dx = m_cpu.getReg16(cpu::DX);
            std::string filename = resolvePath(readFilename((ds << 4) + dx));
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
        case 0x48: // Allocate Memory
        case 0x49: // Free Memory
        case 0x4A: // Resize Block
            handleMemoryManagement();
            break;

        case 0x4E: // Find First
        case 0x4F: // Find Next
            handleDirectorySearch();
            break;
        case 0x57: { // Get/Set File Date and Time
            uint8_t  al  = m_cpu.getReg8(cpu::AL);
            uint16_t bx  = m_cpu.getReg16(cpu::BX);
            if (al == 0x00) { // Get
                // Return a plausible DOS timestamp (2026-03-02 12:00:00)
                // Date: (year-1980)<<9 | month<<5 | day
                // Time: hour<<11 | min<<5 | (sec/2)
                uint16_t date = ((2026 - 1980) << 9) | (3 << 5) | 2;
                uint16_t time = (12 << 11) | (0 << 5) | 0;
                m_cpu.setReg16(cpu::CX, time);
                m_cpu.setReg16(cpu::DX, date);
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
                LOG_DOS("DOS: Get File Date/Time handle=", bx);
            } else if (al == 0x01) { // Set (ignore, just confirm)
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
                LOG_DOS("DOS: Set File Date/Time handle=", bx, " (ignored)");
            } else {
                m_cpu.setReg16(cpu::AX, 0x01);
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            }
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

void DOS::setProgramDir(const std::string& programPath) {
    fs::path p = fs::absolute(programPath);
    m_currentDir = p.parent_path().string();
    LOG_INFO("DOS: Working directory set to '", m_currentDir, "'");
}

std::string DOS::resolvePath(const std::string& path) {
    // If path is already absolute, use as-is
    fs::path p(path);
    if (p.is_absolute()) return path;
    // Resolve relative to m_currentDir
    return (fs::path(m_currentDir) / p).string();
}

void DOS::writeCharToVRAM(uint8_t c) {
    uint16_t cols = m_memory.read16(0x44A);
    uint8_t maxRow = m_memory.read8(0x484);
    if (cols == 0) cols = 80;
    if (maxRow == 0) maxRow = 24;

    uint8_t col = m_memory.read8(0x450);
    uint8_t row = m_memory.read8(0x451);

    switch (c) {
        case 0x07: break; // BEL
        case 0x08: // Backspace
            if (col > 0) col--;
            break;
        case 0x0A: // Line Feed
            row++;
            break;
        case 0x0D: // Carriage Return
            col = 0;
            break;
        default: {
            uint32_t off = (row * cols + col) * 2;
            m_memory.write8(0xB8000 + off, c);
            uint8_t attr = m_memory.read8(0xB8000 + off + 1);
            if (attr == 0) m_memory.write8(0xB8000 + off + 1, 0x07);
            col++;
            if (col >= cols) { col = 0; row++; }
            break;
        }
    }

    // Scroll up if past last row
    if (row > maxRow) {
        for (uint8_t r = 0; r < maxRow; ++r) {
            for (uint16_t cc = 0; cc < cols; ++cc) {
                uint32_t dst = (r * cols + cc) * 2;
                uint32_t src = ((r + 1) * cols + cc) * 2;
                m_memory.write8(0xB8000 + dst, m_memory.read8(0xB8000 + src));
                m_memory.write8(0xB8000 + dst + 1, m_memory.read8(0xB8000 + src + 1));
            }
        }
        for (uint16_t cc = 0; cc < cols; ++cc) {
            uint32_t off = (maxRow * cols + cc) * 2;
            m_memory.write8(0xB8000 + off, ' ');
            m_memory.write8(0xB8000 + off + 1, 0x07);
        }
        row = maxRow;
    }

    m_memory.write8(0x450, col);
    m_memory.write8(0x451, row);
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
        uint16_t largestFree = 0;
        int mcbCount = 0;

        while (true) {
            if (mcbCount++ > 1000) {
                LOG_ERROR("DOS: MCB chain corrupted or too long (infinite loop?)");
                break;
            }
            MCB mcb = readMCB(current);
            LOG_DEBUG("DOS: Checking MCB at 0x", std::hex, current, " type=", (char)mcb.type, " owner=0x", mcb.owner, " size=0x", mcb.size);
            
            if (mcb.owner == 0) {
                if (mcb.size > largestFree) largestFree = mcb.size;
                if (mcb.size >= requested && mcb.size < bestFitSize) {
                    bestFit = current;
                    bestFitSize = mcb.size;
                }
            }
            if (mcb.type == 'Z') break;
            if (mcb.size == 0 && mcb.type != 'Z') {
                LOG_ERROR("DOS: Corrupted MCB with size 0 at 0x", std::hex, current);
                break;
            }
            uint32_t next = (uint32_t)current + mcb.size + 1;
            if (next > 0xFFFF || next > LAST_PARA) {
                LOG_DEBUG("DOS: MCB chain ends/wraps at 0x", std::hex, current, " size 0x", mcb.size);
                break;
            }
            current = static_cast<uint16_t>(next);
        }

        if (bestFit != 0) {
            MCB mcb = readMCB(bestFit);
            LOG_DEBUG("DOS: Splitting MCB at 0x", std::hex, bestFit, " requested 0x", requested, " total 0x", mcb.size);
            if (mcb.size > requested + 1) { // Split
                MCB next;
                next.type = mcb.type;
                next.owner = 0;
                next.size = mcb.size - requested - 1;
                
                mcb.type = 'M';
                mcb.size = requested;
                mcb.owner = m_pspSegment; 
                
                writeMCB(bestFit, mcb);
                uint16_t nextMcbSeg = static_cast<uint16_t>(bestFit + requested + 1);
                writeMCB(nextMcbSeg, next);
                LOG_DEBUG("DOS: Created new free MCB at 0x", std::hex, nextMcbSeg, " size 0x", next.size);
            } else {
                mcb.owner = m_pspSegment;
                writeMCB(bestFit, mcb);
            }
            m_cpu.setReg16(cpu::AX, bestFit + 1);
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
            LOG_DEBUG("DOS: Allocated 0x", std::hex, requested, " paras at 0x", bestFit + 1);
        } else {
            m_cpu.setReg16(cpu::AX, 0x0008); // Insufficient memory
            m_cpu.setReg16(cpu::BX, largestFree); // Largest available block
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            LOG_WARN("DOS: Failed to allocate ", std::hex, requested, " paras (largest free: 0x", largestFree, ")");
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
            std::string path = resolvePath(readFilename(addr));
            LOG_DEBUG("DOS: MKDIR '", path, "'");
            if (fs::create_directory(path)) {
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
                m_cpu.setReg16(cpu::AX, 0);
            } else {
                m_cpu.setReg16(cpu::AX, 0x03); // Path not found or already exists
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            }
        } else if (ah == 0x3A) { // RMDIR
            std::string path = resolvePath(readFilename(addr));
            LOG_DEBUG("DOS: RMDIR '", path, "'");
            if (fs::remove(path)) {
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
                m_cpu.setReg16(cpu::AX, 0);
            } else {
                m_cpu.setReg16(cpu::AX, 0x05); // Access denied (or not empty/doesn't exist)
                m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            }
        } else if (ah == 0x3B) { // CHDIR
            std::string path = resolvePath(readFilename(addr));
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

// Case-insensitive DOS wildcard match: '?' matches any single char, '*' matches any sequence
static bool dosWildcardMatch(const std::string& pattern, const std::string& name) {
    size_t pi = 0, ni = 0;
    size_t starP = std::string::npos, starN = 0;
    while (ni < name.size()) {
        if (pi < pattern.size() && (pattern[pi] == '?' || std::tolower(static_cast<unsigned char>(pattern[pi])) == std::tolower(static_cast<unsigned char>(name[ni])))) {
            ++pi; ++ni;
        } else if (pi < pattern.size() && pattern[pi] == '*') {
            starP = pi++; starN = ni;
        } else if (starP != std::string::npos) {
            pi = starP + 1; ni = ++starN;
        } else {
            return false;
        }
    }
    while (pi < pattern.size() && pattern[pi] == '*') ++pi;
    return pi == pattern.size();
}

void DOS::handleDirectorySearch() {
    uint8_t ah = m_cpu.getReg8(cpu::AH);
    uint32_t dtaAddr = ((m_dtaPtr >> 16) << 4) + (m_dtaPtr & 0xFFFF);

    auto fillDTA = [&](const fs::path& fullPath, const std::string& dosName) {
        // Byte 0x15: file attributes
        std::error_code ec;
        uint8_t dosAttr = 0x20; // Archive
        if (fs::is_directory(fullPath, ec)) dosAttr = 0x10;
        auto perms = fs::status(fullPath, ec).permissions();
        if ((perms & fs::perms::owner_write) == fs::perms::none) dosAttr |= 0x01;
        m_memory.write8(dtaAddr + 0x15, dosAttr);

        // Bytes 0x16-0x17: file time, 0x18-0x19: file date
        auto ftime = fs::last_write_time(fullPath, ec);
        // Use a plausible default timestamp
        uint16_t dosTime = (12 << 11) | (0 << 5) | 0; // 12:00:00
        uint16_t dosDate = ((2026 - 1980) << 9) | (3 << 5) | 2; // 2026-03-02
        m_memory.write16(dtaAddr + 0x16, dosTime);
        m_memory.write16(dtaAddr + 0x18, dosDate);

        // Bytes 0x1A-0x1D: file size (32-bit)
        uint32_t fileSize = 0;
        if (fs::is_regular_file(fullPath, ec))
            fileSize = static_cast<uint32_t>(fs::file_size(fullPath, ec));
        m_memory.write32(dtaAddr + 0x1A, fileSize);

        // Bytes 0x1E-0x2A: filename (null-terminated, uppercase 8.3)
        std::string upper = dosName;
        for (auto& ch : upper) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
        for (int i = 0; i < 13; ++i) {
            uint8_t c = (static_cast<size_t>(i) < upper.size()) ? static_cast<uint8_t>(upper[i]) : 0;
            m_memory.write8(dtaAddr + 0x1E + i, c);
        }
    };

    if (ah == 0x4E) { // Find First
        uint16_t ds = m_cpu.getSegReg(cpu::DS);
        uint16_t dx = m_cpu.getReg16(cpu::DX);
        uint32_t nameAddr = (ds << 4) + dx;
        std::string rawPattern = readFilename(nameAddr);
        uint8_t attr = m_cpu.getReg8(cpu::CL);

        // Separate directory and filename pattern
        fs::path resolved = fs::path(resolvePath(rawPattern));
        std::string searchDir;
        std::string filePattern;
        if (fs::is_directory(resolved)) {
            searchDir = resolved.string();
            filePattern = "*.*";
        } else {
            searchDir = resolved.parent_path().string();
            filePattern = resolved.filename().string();
        }
        if (searchDir.empty()) searchDir = m_currentDir;

        LOG_DEBUG("DOS: FindFirst raw='", rawPattern, "' dir='", searchDir, "' pattern='", filePattern, "' attr 0x", std::hex, (int)attr);

        // Collect matching files
        m_searchResults.clear();
        m_searchIndex = 0;
        m_searchPattern = filePattern;
        m_searchDir = searchDir;

        try {
            for (const auto& entry : fs::directory_iterator(searchDir)) {
                std::string fname = entry.path().filename().string();
                // Skip . and ..
                if (fname == "." || fname == "..") continue;
                // Attribute filtering: skip dirs unless attr includes 0x10
                if (fs::is_directory(entry.path()) && !(attr & 0x10)) continue;

                if (dosWildcardMatch(filePattern, fname)) {
                    m_searchResults.push_back(entry.path());
                }
            }
        } catch (...) {
            m_cpu.setReg16(cpu::AX, 0x03); // Path not found
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            return;
        }

        if (m_searchResults.empty()) {
            m_cpu.setReg16(cpu::AX, 0x12); // No more files
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            LOG_DEBUG("DOS: FindFirst no matches for '", filePattern, "' in '", searchDir, "'");
            return;
        }

        // Store search state in DTA reserved area
        m_memory.write16(dtaAddr + 0x00, 0); // match index = 0

        fillDTA(m_searchResults[0], m_searchResults[0].filename().string());
        LOG_DOS("DOS: FindFirst matched '", m_searchResults[0].filename().string(), "' (", m_searchResults.size(), " total)");

        m_cpu.setReg16(cpu::AX, 0);
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);

    } else if (ah == 0x4F) { // Find Next
        uint16_t index = m_memory.read16(dtaAddr + 0x00);
        index++;

        if (index >= m_searchResults.size()) {
            m_cpu.setReg16(cpu::AX, 0x12); // No more files
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
            LOG_DEBUG("DOS: FindNext end of results");
            return;
        }

        m_memory.write16(dtaAddr + 0x00, index);

        fillDTA(m_searchResults[index], m_searchResults[index].filename().string());
        LOG_DOS("DOS: FindNext matched '", m_searchResults[index].filename().string(), "'");

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


void DOS::setNEInfo(const std::string& path, uint16_t alignShift, const std::vector<NESegment>& segments, uint16_t initialLoadSegment) {
    m_programPath = path;
    m_neAlignShift = alignShift;
    m_neSegments = segments;
    m_neInitialLoadSegment = initialLoadSegment;
    
    // Mark the first code segment as loaded if it's already in memory
    // (Actually, ProgramLoader loads the first segment of code to m_neInitialLoadSegment)
    if (!m_neSegments.empty()) {
        m_neSegments[0].loadedSegment = m_neInitialLoadSegment;
    }

    LOG_INFO("DOS: VROOMM Registered ", segments.size(), " segments. Initial: 0x", std::hex, initialLoadSegment, " AlignShift: ", alignShift);
}

uint16_t DOS::loadOverlaySegment(uint16_t segIndex) {
    LOG_DEBUG("DOS: loadOverlaySegment(", std::dec, segIndex, ")");
    if (segIndex >= m_neSegments.size()) {
        LOG_ERROR("DOS: loadOverlaySegment out of bounds: ", segIndex);
        return 0;
    }
    if (m_neSegments[segIndex].loadedSegment != 0) return m_neSegments[segIndex].loadedSegment;

    auto& seg = m_neSegments[segIndex];
    LOG_DOS("DOS: Loading overlay segment ", std::dec, segIndex + 1, " from ", m_programPath);
    
    std::ifstream file(m_programPath, std::ios::binary);
    if (!file) {
        LOG_ERROR("DOS: Failed to open program file for overlay loading: ", m_programPath);
        return 0;
    }

    uint32_t fileOffset = static_cast<uint32_t>(seg.fileOffsetSector) << m_neAlignShift;
    uint32_t sizeInFile = seg.length == 0 ? 0x10000 : seg.length;
    uint32_t memSize = seg.minAlloc == 0 ? 0x10000 : seg.minAlloc;
    if (sizeInFile > memSize) memSize = sizeInFile;

    LOG_DOS("DOS: Overlay seg ", std::dec, segIndex + 1, ": sector=0x", std::hex, seg.fileOffsetSector, 
            " (fileOff=0x", fileOffset, ") len=0x", seg.length, " flags=0x", seg.flags, " minAlloc=0x", seg.minAlloc);

    uint16_t paragraphs = (uint16_t)((memSize + 15) / 16);

    // Allocate memory using AH=48h logic
    m_cpu.setReg8(cpu::AH, 0x48);
    m_cpu.setReg16(cpu::BX, paragraphs);
    handleMemoryManagement();
    
    if (m_cpu.getEFLAGS() & cpu::FLAG_CARRY) {
        LOG_ERROR("DOS: Failed to allocate memory (", std::dec, paragraphs, " paras) for overlay segment ", segIndex + 1);
        return 0;
    }
    
    uint16_t targetSegment = (uint16_t)m_cpu.getReg16(cpu::AX);
    seg.loadedSegment = targetSegment;

    file.seekg(fileOffset, std::ios::beg);
    std::vector<uint8_t> buffer(paragraphs * 16, 0);
    file.read(reinterpret_cast<char*>(buffer.data()), sizeInFile);
    
    // Check for relocation information (flag 0x0100)
    if (seg.flags & 0x0100) {
        LOG_DOS("DOS: Segment ", std::dec, segIndex + 1, " has relocation information.");
        file.seekg(fileOffset + sizeInFile, std::ios::beg);
        uint16_t numRelocs = 0;
        file.read(reinterpret_cast<char*>(&numRelocs), 2);
        LOG_DOS("DOS: Applying ", std::dec, numRelocs, " fixups...");
        
        for (int i = 0; i < numRelocs; ++i) {
            uint8_t r[8];
            file.read(reinterpret_cast<char*>(r), 8);
            
            uint8_t sourceType = r[0]; // Standard NE: 0=LoByte, 2=Segment, 3=Far Addr, 5=Offset
            uint8_t flags = r[1];
            uint16_t nextOffset = *reinterpret_cast<uint16_t*>(&r[2]);
            uint16_t targetSegIdx = *reinterpret_cast<uint16_t*>(&r[4]); // 1-based index
            uint16_t targetOffset = *reinterpret_cast<uint16_t*>(&r[6]);
            
            uint8_t relocType = flags & 0x03;
            bool additive = (flags & 0x04) != 0;

            if (relocType == 0) { // Internal Reference
                uint16_t actualTargetSeg = 0;
                LOG_DOS("DOS: Reloc Internal index ", std::dec, targetSegIdx, " offset 0x", std::hex, targetOffset, 
                        " at relOffset 0x", nextOffset);
                
                if (targetSegIdx > 0 && targetSegIdx <= m_neSegments.size()) {
                    actualTargetSeg = m_neSegments[targetSegIdx - 1].loadedSegment;
                    if (actualTargetSeg == 0) actualTargetSeg = loadOverlaySegment(targetSegIdx - 1);
                } else if (targetSegIdx == 255) {
                    actualTargetSeg = m_neSegments[segIndex].loadedSegment;
                    LOG_DOS("DOS: Reloc mapping segment 255 -> self (0x", std::hex, actualTargetSeg, ")");
                } else {
                    LOG_ERROR("DOS: Reloc Internal Ref invalid segment index ", std::dec, targetSegIdx);
                    continue;
                }
                
                if (actualTargetSeg == 0) continue;
                
                int chainCount = 0;
                while (nextOffset != 0xFFFF) {
                    LOG_DEBUG("DOS: Reloc chain site: targetSeg=0x", std::hex, actualTargetSeg, " at relOffset=0x", nextOffset);
                    if (chainCount++ > 100) {
                        LOG_ERROR("DOS: Relocation chain too long or cyclic at relOffset 0x", std::hex, nextOffset);
                        break;
                    }
                    if ((size_t)nextOffset >= buffer.size()) break;
                    
                    uint16_t currentSiteValue = 0;
                    if ((size_t)nextOffset + 1 < buffer.size()) {
                        currentSiteValue = *reinterpret_cast<uint16_t*>(&buffer[nextOffset]);
                    }

                    // For additive relocations, apply the target offset over the existing inline offset.
                    uint16_t finalOffset = targetOffset;
                    if (additive) {
                        finalOffset += currentSiteValue;
                    }

                    if (sourceType == 2) { // 16-bit Segment (2 bytes)
                        if ((size_t)nextOffset + 1 < buffer.size()) {
                            buffer[nextOffset] = actualTargetSeg & 0xFF;
                            buffer[nextOffset+1] = (actualTargetSeg >> 8) & 0xFF;
                        }
                    } else if (sourceType == 3) { // 32-bit Far Address (16-bit offset + 16-bit segment)
                        if ((size_t)nextOffset + 3 < buffer.size()) {
                            buffer[nextOffset] = finalOffset & 0xFF;
                            buffer[nextOffset+1] = (finalOffset >> 8) & 0xFF;
                            buffer[nextOffset+2] = actualTargetSeg & 0xFF;
                            buffer[nextOffset+3] = (actualTargetSeg >> 8) & 0xFF;
                        }
                    } else if (sourceType == 5) { // 16-bit Offset (2 bytes)
                        if ((size_t)nextOffset + 1 < buffer.size()) {
                            buffer[nextOffset] = finalOffset & 0xFF;
                            buffer[nextOffset+1] = (finalOffset >> 8) & 0xFF;
                        }
                    } else if (sourceType == 0) { // 8-bit LoByte (1 byte)
                        buffer[nextOffset] = finalOffset & 0xFF;
                    }
                    
                    if (additive) break; // Additive relocations do not chain
                    nextOffset = currentSiteValue; // Move to next item in chain
                }
            } else if (relocType == 1 || relocType == 2) {
                // Imported Ordinal / Name - Not implemented yet
            }
        }
    }

    uint32_t targetAddr = (targetSegment << 4);
    for (uint32_t i = 0; i < buffer.size(); ++i) {
        m_memory.write8(targetAddr + i, buffer[i]);
    }
    
    LOG_DOS("DOS: Loaded overlay segment ", std::dec, segIndex + 1, " to 0x", std::hex, targetSegment);
    return targetSegment;
}

} // namespace fador::hw
