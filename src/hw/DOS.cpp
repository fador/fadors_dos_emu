#include "DOS.hpp"
#include "../utils/Logger.hpp"
#include <iostream>
#include <iomanip>

namespace fador::hw {

DOS::DOS(cpu::CPU& cpu, memory::MemoryBus& memory)
    : m_cpu(cpu)
    , m_memory(memory) {
}

void DOS::initialize() {
    LOG_INFO("DOS: Kernel initialized.");
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

} // namespace fador::hw
