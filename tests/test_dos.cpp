#include "test_framework.hpp"
#include <algorithm>
#include <fstream>
#include <cstdio>
#include <vector>
#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "memory/MemoryBus.hpp"
#include "hw/IOBus.hpp"
#include "hw/PIC8259.hpp"
#include "hw/BIOS.hpp"
#include "hw/DOS.hpp"
#include "hw/ProgramLoader.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIT8254.hpp"

using namespace fador;

namespace {

void setSegment(cpu::CPU &cpu, cpu::SegRegIndex reg, uint16_t seg) {
    cpu.setSegReg(reg, seg);
    cpu.setSegBase(reg, static_cast<uint32_t>(seg) << 4);
}

void writeCString(memory::MemoryBus &mem, uint32_t physAddr,
                  const std::string &text) {
    for (size_t i = 0; i < text.size(); ++i)
        mem.write8(physAddr + static_cast<uint32_t>(i),
                   static_cast<uint8_t>(text[i]));
    mem.write8(physAddr + static_cast<uint32_t>(text.size()), 0);
}

uint8_t emsPattern(uint16_t logicalPage, size_t offset) {
    return static_cast<uint8_t>((logicalPage * 37u + offset * 13u +
                                 static_cast<uint16_t>(offset >> 8)) & 0xFFu);
}

std::string readBlankPaddedName(memory::MemoryBus &mem, uint32_t physAddr,
                                size_t length) {
    std::string value;
    for (size_t i = 0; i < length; ++i)
        value.push_back(static_cast<char>(mem.read8(physAddr + i)));
    while (!value.empty() && value.back() == ' ')
        value.pop_back();
    return value;
}

} // namespace

TEST_CASE("DOS Emulation and Program Loading", "[DOS]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::IOBus iobus;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    
    bios.initialize();
    dos.initialize();
    cpu::InstructionDecoder decoder(cpu, mem, iobus, bios, dos);
    hw::ProgramLoader loader(cpu, mem, dos.getHIMEM());

    SECTION("COM File Loading and PSP Initialization") {
        // Create a dummy .COM file (B4 09 BA 00 00 CD 21 CD 20) -> AH=09, DX=0, INT 21, INT 20
        {
            std::ofstream ofs("test.com", std::ios::binary);
            uint8_t code[] = { 0xB4, 0x09, 0xBA, 0x07, 0x01, 0xCD, 0x21, 0xCD, 0x20, 'H', 'e', 'l', 'l', 'o', '$' };
            ofs.write(reinterpret_cast<const char*>(code), sizeof(code));
        }

        REQUIRE(loader.loadCOM("test.com", 0x1000));

        // Verify PSP
        REQUIRE(mem.read8(0x10000) == 0xCD); // INT 20h
        REQUIRE(mem.read8(0x10001) == 0x20);

        // Verify Code at 100h
        REQUIRE(mem.read8(0x10100) == 0xB4); // MOV AH, 09
        REQUIRE(mem.read8(0x10105) == 0xCD); // INT 21h

        // Verify CPU state
        REQUIRE(cpu.getSegReg(cpu::CS) == 0x1000);
        REQUIRE(cpu.getEIP() == 0x100);

        // Step through instructions
        decoder.step(); // MOV AH, 09
        REQUIRE(cpu.getReg8(cpu::AH) == 0x09);
        
        decoder.step(); // MOV DX, 0107h
        REQUIRE(cpu.getReg16(cpu::DX) == 0x0107);

        decoder.step(); // INT 21h
        // (Logging check is manual, but ensure it doesn't crash)

        decoder.step(); // INT 20h
        // (Log check for termination)

        std::remove("test.com");
    }

        SECTION("Get DOS Version") {
            cpu.setReg8(cpu::AH, 0x30);
            dos.handleInterrupt(0x21);
            REQUIRE(cpu.getReg8(cpu::AL) == 3);
            REQUIRE(cpu.getReg8(cpu::AH) == 30);
        }

    SECTION("DOS File I/O: Open, Read, Close") {
        // Create a test file
        {
            std::ofstream ofs("hello.txt");
            ofs << "DOS File I/O Test Content";
        }

        // AH=3Dh: Open file
        cpu.setSegReg(cpu::DS, 0x2000);
        cpu.setReg16(cpu::DX, 0x0000);
        std::string fname = "hello.txt";
        for (int i = 0; i < fname.length(); ++i) mem.write8(0x20000 + i, fname[i]);
        mem.write8(0x20000 + fname.length(), 0); // Null terminator

        cpu.setReg8(cpu::AH, 0x3D);
        cpu.setReg8(cpu::AL, 0); // Read-only
        dos.handleInterrupt(0x21);

        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
        uint16_t handle = cpu.getReg16(cpu::AX);
        REQUIRE(handle >= 5);

        // AH=3Fh: Read from file
        cpu.setReg8(cpu::AH, 0x3F);
        cpu.setReg16(cpu::BX, handle);
        cpu.setReg16(cpu::CX, 10); // Read 10 bytes
        cpu.setSegReg(cpu::DS, 0x3000);
        cpu.setReg16(cpu::DX, 0x0000);
        dos.handleInterrupt(0x21);

        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
        REQUIRE(cpu.getReg16(cpu::AX) == 10);
        
        std::string readStr;
        for (int i = 0; i < 10; ++i) readStr += (char)mem.read8(0x30000 + i);
        REQUIRE(readStr == "DOS File I");

        // AH=3Eh: Close file
        cpu.setReg8(cpu::AH, 0x3E);
        cpu.setReg16(cpu::BX, handle);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));

        std::remove("hello.txt");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// File I/O Edge Cases
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DOS: File open non-existent file returns error", "[DOS][File]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();

    std::string fname = "NONEXIST.TXT";
    uint32_t fnameAddr = 0x70000;
    for (size_t i = 0; i < fname.size(); i++)
        mem.write8(fnameAddr + i, fname[i]);
    mem.write8(fnameAddr + fname.size(), 0);

    cpu.setSegReg(cpu::DS, 0x7000);
    cpu.setSegBase(cpu::DS, 0x70000);
    cpu.setReg16(cpu::DX, 0x0000);
    cpu.setReg8(cpu::AH, 0x3D);
    cpu.setReg8(cpu::AL, 0x00);
    dos.handleInterrupt(0x21);

    REQUIRE(cpu.getEFLAGS() & cpu::FLAG_CARRY);
}

TEST_CASE("DOS: File read with zero bytes", "[DOS][File]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();

    {
        std::ofstream ofs("zeroread.txt", std::ios::binary);
        ofs.write("DATA", 4);
    }

    std::string fname = "ZEROREAD.TXT";
    uint32_t fnameAddr = 0x70000;
    for (size_t i = 0; i < fname.size(); i++)
        mem.write8(fnameAddr + i, fname[i]);
    mem.write8(fnameAddr + fname.size(), 0);

    cpu.setSegReg(cpu::DS, 0x7000);
    cpu.setSegBase(cpu::DS, 0x70000);
    cpu.setReg16(cpu::DX, 0x0000);
    cpu.setReg8(cpu::AH, 0x3D);
    cpu.setReg8(cpu::AL, 0x00);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    uint16_t handle = cpu.getReg16(cpu::AX);

    cpu.setReg8(cpu::AH, 0x3F);
    cpu.setReg16(cpu::BX, handle);
    cpu.setReg16(cpu::CX, 0);
    cpu.setSegReg(cpu::DS, 0x8000);
    cpu.setSegBase(cpu::DS, 0x80000);
    cpu.setReg16(cpu::DX, 0x0000);
    dos.handleInterrupt(0x21);
    uint16_t bytesRead = cpu.getReg16(cpu::AX);
    REQUIRE(bytesRead == 0);

    cpu.setReg8(cpu::AH, 0x3E);
    cpu.setReg16(cpu::BX, handle);
    dos.handleInterrupt(0x21);

    std::remove("zeroread.txt");
}

TEST_CASE("DOS: File SEEK to end", "[DOS][File]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();

    {
        std::ofstream ofs("seektest.txt", std::ios::binary);
        ofs.write("ABCDEFGH", 8);
    }

    std::string fname = "SEEKTEST.TXT";
    uint32_t fnameAddr = 0x70000;
    for (size_t i = 0; i < fname.size(); i++)
        mem.write8(fnameAddr + i, fname[i]);
    mem.write8(fnameAddr + fname.size(), 0);

    cpu.setSegReg(cpu::DS, 0x7000);
    cpu.setSegBase(cpu::DS, 0x70000);
    cpu.setReg16(cpu::DX, 0x0000);
    cpu.setReg8(cpu::AH, 0x3D);
    cpu.setReg8(cpu::AL, 0x00);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    uint16_t handle = cpu.getReg16(cpu::AX);

    // Seek to end (AL=2)
    cpu.setReg8(cpu::AH, 0x42);
    cpu.setReg16(cpu::BX, handle);
    cpu.setReg8(cpu::AL, 0x02);
    cpu.setReg16(cpu::CX, 0);
    cpu.setReg16(cpu::DX, 0);
    dos.handleInterrupt(0x21);

    cpu.setReg8(cpu::AH, 0x3E);
    cpu.setReg16(cpu::BX, handle);
    dos.handleInterrupt(0x21);

    std::remove("seektest.txt");
}

TEST_CASE("ProgramLoader parses FBOV overlays and INT 3F loads them", "[DOS][Overlay][FBOV]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();
    hw::ProgramLoader loader(cpu, mem, dos.getHIMEM());

    const char* fname = "fbov_test.exe";
    {
        std::vector<uint8_t> exe(48 + 24, 0);
        auto write16 = [&exe](size_t offset, uint16_t value) {
            exe[offset + 0] = static_cast<uint8_t>(value & 0xFF);
            exe[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
        };
        auto write32 = [&exe](size_t offset, uint32_t value) {
            exe[offset + 0] = static_cast<uint8_t>(value & 0xFF);
            exe[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
            exe[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFF);
            exe[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFF);
        };

        exe[0] = 'M';
        exe[1] = 'Z';
        write16(2, 48);      // Last page size
        write16(4, 1);       // Number of pages
        write16(6, 0);       // Relocations
        write16(8, 2);       // Header size paragraphs (32 bytes)
        write16(10, 0);      // minAlloc
        write16(12, 0);      // maxAlloc
        write16(14, 0);      // SS
        write16(16, 0xFFFE); // SP
        write16(18, 0);      // checksum
        write16(20, 0);      // IP
        write16(22, 0);      // CS
        write16(24, 0x001C); // relocation table offset
        write16(26, 0);      // overlay number

        exe[32] = 0x90;
        exe[33] = 0x90;
        exe[34] = 0xCD;
        exe[35] = 0x20;

        exe[48] = 'F';
        exe[49] = 'B';
        exe[50] = 'O';
        exe[51] = 'V';
        write16(52, 24);     // FBOV block size
        write16(54, 2);      // Overlay id
        write32(56, 0);      // aux offset
        write32(60, 0);      // aux count
        exe[64] = 'O';
        exe[65] = 'V';
        exe[66] = 'R';
        exe[67] = 'D';
        exe[68] = 'A';
        exe[69] = 'T';
        exe[70] = 'A';
        exe[71] = '!';

        std::ofstream ofs(fname, std::ios::binary);
        ofs.write(reinterpret_cast<const char*>(exe.data()), static_cast<std::streamsize>(exe.size()));
    }

    REQUIRE(loader.loadEXE(fname, dos.getPSPSegment(), dos));

    cpu.setSegReg(cpu::CS, 0x2000);
    cpu.setSegBase(cpu::CS, 0x20000);
    cpu.setEIP(0x0102);
    cpu.setInstructionStartEIP(0x0100);

    mem.write8(0x20100, 0xCD);
    mem.write8(0x20101, 0x3F);
    mem.write16(0x20102, 0x1234);
    mem.write16(0x20104, 0x0002);

    REQUIRE(dos.handleInterrupt(0x3F));
    REQUIRE(mem.read8(0x20100) == 0xEA);
    REQUIRE(mem.read16(0x20101) == 0x1234);

    const uint16_t overlaySeg = mem.read16(0x20103);
    REQUIRE(overlaySeg != 0);

    const uint32_t overlayAddr = static_cast<uint32_t>(overlaySeg) << 4;
    REQUIRE(mem.read8(overlayAddr + 0) == 'F');
    REQUIRE(mem.read8(overlayAddr + 1) == 'B');
    REQUIRE(mem.read8(overlayAddr + 2) == 'O');
    REQUIRE(mem.read8(overlayAddr + 3) == 'V');

    std::remove(fname);
}

TEST_CASE("DOS: Close invalid file handle returns error", "[DOS][File]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();

    // Try to close an invalid handle
    cpu.setReg8(cpu::AH, 0x3E);
    cpu.setReg16(cpu::BX, 0xFFFF);
    dos.handleInterrupt(0x21);
    REQUIRE(cpu.getEFLAGS() & cpu::FLAG_CARRY);
}

TEST_CASE("DOS: Duplicate file handle survives closing original", "[DOS][File]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();

    {
        std::ofstream ofs("duphandle.txt", std::ios::binary);
        ofs << "duplicate";
    }

    const std::string fname = "DUPHANDLE.TXT";
    const uint32_t fnameAddr = 0x70000;
    for (size_t i = 0; i < fname.size(); ++i)
        mem.write8(fnameAddr + i, fname[i]);
    mem.write8(fnameAddr + fname.size(), 0);

    cpu.setSegReg(cpu::DS, 0x7000);
    cpu.setSegBase(cpu::DS, 0x70000);
    cpu.setReg16(cpu::DX, 0x0000);
    cpu.setReg8(cpu::AH, 0x3D);
    cpu.setReg8(cpu::AL, 0x00);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    const uint16_t handle = cpu.getReg16(cpu::AX);

    cpu.setReg8(cpu::AH, 0x45);
    cpu.setReg16(cpu::BX, handle);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    const uint16_t dupHandle = cpu.getReg16(cpu::AX);
    REQUIRE(dupHandle >= 5);
    REQUIRE(dupHandle != handle);

    cpu.setReg8(cpu::AH, 0x3E);
    cpu.setReg16(cpu::BX, handle);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));

    cpu.setSegReg(cpu::DS, 0x8000);
    cpu.setSegBase(cpu::DS, 0x80000);
    cpu.setReg16(cpu::DX, 0x0000);
    cpu.setReg8(cpu::AH, 0x3F);
    cpu.setReg16(cpu::BX, dupHandle);
    cpu.setReg16(cpu::CX, 9);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(cpu.getReg16(cpu::AX) == 9);

    std::string readStr;
    for (int i = 0; i < 9; ++i)
        readStr += static_cast<char>(mem.read8(0x80000 + i));
    REQUIRE(readStr == "duplicate");

    cpu.setReg8(cpu::AH, 0x3E);
    cpu.setReg16(cpu::BX, dupHandle);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));

    std::remove("duphandle.txt");
}

TEST_CASE("DOS: EMMXXXX0 opens as EMS device and reports EMM386 records", "[DOS][EMS]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();

    const std::string deviceName = "EMMXXXX0";
    const uint32_t nameAddr = 0x70000;
    for (size_t i = 0; i < deviceName.size(); ++i)
        mem.write8(nameAddr + i, deviceName[i]);
    mem.write8(nameAddr + deviceName.size(), 0);

    cpu.setSegReg(cpu::DS, 0x7000);
    cpu.setSegBase(cpu::DS, 0x70000);
    cpu.setReg16(cpu::DX, 0x0000);
    cpu.setReg8(cpu::AH, 0x3D);
    cpu.setReg8(cpu::AL, 0x00);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    const uint16_t handle = cpu.getReg16(cpu::AX);
    REQUIRE(handle >= 5);

    cpu.setReg8(cpu::AH, 0x44);
    cpu.setReg8(cpu::AL, 0x00);
    cpu.setReg16(cpu::BX, handle);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(cpu.getReg16(cpu::DX) & 0x0080);

    cpu.setSegReg(cpu::DS, 0x7100);
    cpu.setSegBase(cpu::DS, 0x71000);
    mem.write8(0x71000, 0x00);
    cpu.setReg8(cpu::AH, 0x44);
    cpu.setReg8(cpu::AL, 0x02);
    cpu.setReg16(cpu::BX, handle);
    cpu.setReg16(cpu::CX, 6);
    cpu.setReg16(cpu::DX, 0x0000);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(cpu.getReg16(cpu::AX) == 6);
    REQUIRE(mem.read16(0x71000) == 0x0025);
    REQUIRE(mem.read16(0x71002) == 0x0070);
    REQUIRE(mem.read16(0x71004) == 0xF000);

    cpu.setSegReg(cpu::DS, 0x7200);
    cpu.setSegBase(cpu::DS, 0x72000);
    mem.write8(0x72000, 0x01);
    cpu.setReg8(cpu::AH, 0x44);
    cpu.setReg8(cpu::AL, 0x02);
    cpu.setReg16(cpu::BX, handle);
    cpu.setReg16(cpu::CX, 6);
    cpu.setReg16(cpu::DX, 0x0000);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(cpu.getReg16(cpu::AX) == 6);
    REQUIRE(mem.read32(0x72000) == 0x000F1100);
    REQUIRE(mem.read8(0x72004) == 0x01);
    REQUIRE(mem.read8(0x72005) == 0x10);

    cpu.setReg8(cpu::AH, 0x3E);
    cpu.setReg16(cpu::BX, handle);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
}

TEST_CASE("DOS: List of Lists exposes NUL and EMS device chain", "[DOS][EMS]") {
    constexpr uint16_t kFirstMCBSegment = 0x0800;

    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();

    cpu.setReg8(cpu::AH, 0x30);
    dos.handleInterrupt(0x21);
    REQUIRE(cpu.getReg8(cpu::AL) == 5);

    cpu.setReg8(cpu::AH, 0x52);
    dos.handleInterrupt(0x21);

    const uint16_t listSeg = cpu.getSegReg(cpu::ES);
    const uint16_t listOff = cpu.getReg16(cpu::BX);
    const uint32_t listPhys = (static_cast<uint32_t>(listSeg) << 4) + listOff;

    REQUIRE(mem.read16(listPhys - 2) == kFirstMCBSegment);

    const uint32_t nulHeader = listPhys + 0x22;
    REQUIRE(readBlankPaddedName(mem, nulHeader + 0x0A, 8) == "NUL");
    REQUIRE(mem.read16(nulHeader + 0x04) & 0x8000);
    REQUIRE(mem.read16(nulHeader + 0x04) & 0x0004);

    REQUIRE(mem.read32(listPhys + 0x08) != 0x00000000u);
    REQUIRE(mem.read32(listPhys + 0x08) != 0xFFFFFFFFu);
    REQUIRE(mem.read32(listPhys + 0x0C) != 0x00000000u);
    REQUIRE(mem.read32(listPhys + 0x0C) != 0xFFFFFFFFu);

    uint32_t currentHeader = mem.read32(nulHeader + 0x00);
    bool foundEMS = false;
    for (int depth = 0; depth < 8 && currentHeader != 0xFFFFFFFFu; ++depth) {
        const uint16_t headerOff = static_cast<uint16_t>(currentHeader & 0xFFFFu);
        const uint16_t headerSeg = static_cast<uint16_t>(currentHeader >> 16);
        const uint32_t headerPhys = (static_cast<uint32_t>(headerSeg) << 4) + headerOff;
        if (readBlankPaddedName(mem, headerPhys + 0x0A, 8) == "EMMXXXX0") {
            foundEMS = true;
            break;
        }
        currentHeader = mem.read32(headerPhys + 0x00);
    }

    REQUIRE(foundEMS);
}

TEST_CASE("EMS compatibility: EMSTEST main flow", "[DOS][EMS][BIOS]") {
    constexpr uint16_t kEmsPagesUnderTest = 128;
    constexpr uint16_t kIoctlSeg = 0x7100;
    constexpr uint16_t kFilenameSeg = 0x7200;
    constexpr uint16_t kScratchSeg = 0x7300;
    constexpr size_t kChunkSize = 128;
    constexpr size_t kDiskSize = kEmsPagesUnderTest * kChunkSize;

    const std::string tempFile = "EMSTEST.$$$";
    struct TempFileGuard {
        std::string path;
        ~TempFileGuard() { std::remove(path.c_str()); }
    } tempFileGuard{tempFile};
    std::remove(tempFile.c_str());

    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();

    const uint32_t pageFrameBase =
        static_cast<uint32_t>(hw::BIOS::EMS_PAGE_FRAME_SEGMENT) << 4;

    auto setDS = [&](uint16_t seg) { setSegment(cpu, cpu::DS, seg); };
    auto writeFilename = [&](const std::string &name) {
        setDS(kFilenameSeg);
        writeCString(mem, static_cast<uint32_t>(kFilenameSeg) << 4, name);
        cpu.setReg16(cpu::DX, 0x0000);
    };

    auto dosOpen = [&](const std::string &name, uint8_t accessMode) {
        writeFilename(name);
        cpu.setReg8(cpu::AH, 0x3D);
        cpu.setReg8(cpu::AL, accessMode);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
        return cpu.getReg16(cpu::AX);
    };

    auto dosCreate = [&](const std::string &name) {
        writeFilename(name);
        cpu.setReg8(cpu::AH, 0x3C);
        cpu.setReg16(cpu::CX, 0x0000);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
        return cpu.getReg16(cpu::AX);
    };

    auto dosClose = [&](uint16_t handle) {
        cpu.setReg8(cpu::AH, 0x3E);
        cpu.setReg16(cpu::BX, handle);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    };

    auto dosDelete = [&](const std::string &name) {
        writeFilename(name);
        cpu.setReg8(cpu::AH, 0x41);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    };

    auto dosSeek = [&](uint16_t handle, uint32_t offset) {
        cpu.setReg8(cpu::AH, 0x42);
        cpu.setReg8(cpu::AL, 0x00);
        cpu.setReg16(cpu::BX, handle);
        cpu.setReg16(cpu::CX, static_cast<uint16_t>(offset >> 16));
        cpu.setReg16(cpu::DX, static_cast<uint16_t>(offset & 0xFFFF));
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
        const uint32_t newPos =
            (static_cast<uint32_t>(cpu.getReg16(cpu::DX)) << 16) |
            cpu.getReg16(cpu::AX);
        REQUIRE(newPos == offset);
    };

    auto dosWrite = [&](uint16_t handle, uint16_t segment, uint16_t offset,
                        uint16_t count) {
        setDS(segment);
        cpu.setReg8(cpu::AH, 0x40);
        cpu.setReg16(cpu::BX, handle);
        cpu.setReg16(cpu::CX, count);
        cpu.setReg16(cpu::DX, offset);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
        REQUIRE(cpu.getReg16(cpu::AX) == count);
    };

    auto dosRead = [&](uint16_t handle, uint16_t segment, uint16_t offset,
                       uint16_t count) {
        setDS(segment);
        cpu.setReg8(cpu::AH, 0x3F);
        cpu.setReg16(cpu::BX, handle);
        cpu.setReg16(cpu::CX, count);
        cpu.setReg16(cpu::DX, offset);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
        REQUIRE(cpu.getReg16(cpu::AX) == count);
    };

    auto dosIoctlReadCharacterDevice = [&](uint16_t handle,
                                           uint8_t subFunction) {
        const uint32_t bufAddr = static_cast<uint32_t>(kIoctlSeg) << 4;
        setDS(kIoctlSeg);
        mem.write8(bufAddr, subFunction);
        cpu.setReg8(cpu::AH, 0x44);
        cpu.setReg8(cpu::AL, 0x02);
        cpu.setReg16(cpu::BX, handle);
        cpu.setReg16(cpu::CX, 6);
        cpu.setReg16(cpu::DX, 0x0000);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
        REQUIRE(cpu.getReg16(cpu::AX) == 6);
        return bufAddr;
    };

    auto emsAllocate = [&](uint16_t pageCount) {
        cpu.setReg8(cpu::AH, 0x43);
        cpu.setReg16(cpu::BX, pageCount);
        bios.handleInterrupt(0x67);
        REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
        return cpu.getReg16(cpu::DX);
    };

    auto emsMap = [&](uint8_t physicalPage, uint16_t logicalPage,
                      uint16_t handle) {
        cpu.setReg8(cpu::AH, 0x44);
        cpu.setReg8(cpu::AL, physicalPage);
        cpu.setReg16(cpu::BX, logicalPage);
        cpu.setReg16(cpu::DX, handle);
        bios.handleInterrupt(0x67);
        REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    };

    auto emsSave = [&](uint16_t handle) {
        cpu.setReg8(cpu::AH, 0x47);
        cpu.setReg16(cpu::DX, handle);
        bios.handleInterrupt(0x67);
        REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    };

    auto emsRestore = [&](uint16_t handle) {
        cpu.setReg8(cpu::AH, 0x48);
        cpu.setReg16(cpu::DX, handle);
        bios.handleInterrupt(0x67);
        REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    };

    auto emsDeallocate = [&](uint16_t handle) {
        cpu.setReg8(cpu::AH, 0x45);
        cpu.setReg16(cpu::DX, handle);
        bios.handleInterrupt(0x67);
        REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    };

    auto framePtr = [&](uint8_t physicalPage) {
        return mem.directAccess(pageFrameBase +
                                static_cast<uint32_t>(physicalPage) *
                                    hw::BIOS::EMS_PAGE_SIZE);
    };

    auto initializePages = [&](uint16_t handle,
                               std::vector<std::vector<uint8_t>> &expected) {
        expected.assign(kEmsPagesUnderTest,
                        std::vector<uint8_t>(hw::BIOS::EMS_PAGE_SIZE, 0));
        for (uint16_t logicalPage = 0; logicalPage < kEmsPagesUnderTest;
             ++logicalPage) {
            const uint8_t physicalPage =
                static_cast<uint8_t>(logicalPage % hw::BIOS::EMS_PHYSICAL_PAGE_COUNT);
            emsMap(physicalPage, logicalPage, handle);
            uint8_t *frame = framePtr(physicalPage);
            REQUIRE(frame != nullptr);
            for (size_t offset = 0; offset < expected[logicalPage].size();
                 ++offset) {
                const uint8_t value = emsPattern(logicalPage, offset);
                frame[offset] = value;
                expected[logicalPage][offset] = value;
            }
        }
    };

    auto verifyPages = [&](uint16_t handle,
                           const std::vector<std::vector<uint8_t>> &expected) {
        for (uint16_t logicalPage = 0; logicalPage < kEmsPagesUnderTest;
             ++logicalPage) {
            const uint8_t physicalPage =
                static_cast<uint8_t>(logicalPage % hw::BIOS::EMS_PHYSICAL_PAGE_COUNT);
            emsMap(physicalPage, logicalPage, handle);
            uint8_t *frame = framePtr(physicalPage);
            REQUIRE(frame != nullptr);
            REQUIRE(std::equal(frame, frame + expected[logicalPage].size(),
                               expected[logicalPage].begin()));
        }
    };

    const uint16_t emsDeviceHandle = dosOpen("EMMXXXX0", 0x00);

    cpu.setReg8(cpu::AH, 0x44);
    cpu.setReg8(cpu::AL, 0x00);
    cpu.setReg16(cpu::BX, emsDeviceHandle);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(cpu.getReg16(cpu::DX) & 0x0080);

    const uint32_t apiRecordAddr =
        dosIoctlReadCharacterDevice(emsDeviceHandle, 0x00);
    REQUIRE(mem.read16(apiRecordAddr + 0) == 0x0025);
    REQUIRE(mem.read16(apiRecordAddr + 2) == hw::BIOS::EMS_PRIVATE_API_OFFSET);
    REQUIRE(mem.read16(apiRecordAddr + 4) == 0xF000);

    const uint32_t importRecordAddr =
        dosIoctlReadCharacterDevice(emsDeviceHandle, 0x01);
    REQUIRE(mem.read32(importRecordAddr + 0) == hw::BIOS::EMS_IMPORT_RECORD_PHYS);
    REQUIRE(mem.read8(importRecordAddr + 4) == 0x01);
    REQUIRE(mem.read8(importRecordAddr + 5) == 0x10);

    dosClose(emsDeviceHandle);

    cpu.setReg8(cpu::AH, 0x46);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    REQUIRE(cpu.getReg8(cpu::AL) == 0x40);

    cpu.setReg8(cpu::AH, 0x41);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    REQUIRE(cpu.getReg16(cpu::BX) == hw::BIOS::EMS_PAGE_FRAME_SEGMENT);

    cpu.setReg8(cpu::AH, 0x42);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    REQUIRE(cpu.getReg16(cpu::BX) == hw::BIOS::EMS_TOTAL_PAGES);
    REQUIRE(cpu.getReg16(cpu::DX) == hw::BIOS::EMS_TOTAL_PAGES);

    std::vector<std::vector<uint8_t>> expectedPages;
    const uint16_t basicHandle = emsAllocate(kEmsPagesUnderTest);
    emsSave(basicHandle);
    initializePages(basicHandle, expectedPages);
    verifyPages(basicHandle, expectedPages);
    emsRestore(basicHandle);
    emsDeallocate(basicHandle);

    cpu.setReg8(cpu::AH, 0x42);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    REQUIRE(cpu.getReg16(cpu::BX) == hw::BIOS::EMS_TOTAL_PAGES);
    REQUIRE(cpu.getReg16(cpu::DX) == hw::BIOS::EMS_TOTAL_PAGES);

    std::vector<std::vector<uint8_t>> expectedIoPages;
    std::vector<uint8_t> expectedDisk(kDiskSize, 0);
    const uint16_t ioHandle = emsAllocate(kEmsPagesUnderTest);
    emsSave(ioHandle);
    initializePages(ioHandle, expectedIoPages);

    uint8_t *scratch = mem.directAccess(static_cast<uint32_t>(kScratchSeg) << 4);
    REQUIRE(scratch != nullptr);
    std::fill_n(scratch, kChunkSize, 0);

    const uint16_t fileHandle = dosCreate(tempFile);
    for (size_t diskOffset = 0; diskOffset < kDiskSize; diskOffset += kChunkSize) {
        dosSeek(fileHandle, static_cast<uint32_t>(diskOffset));
        dosWrite(fileHandle, kScratchSeg, 0x0000,
                 static_cast<uint16_t>(kChunkSize));
    }

    uint32_t rngState = 0x454D5354u;
    auto nextRand = [&]() {
        rngState = rngState * 1664525u + 1013904223u;
        return rngState;
    };

    for (size_t iteration = 0; iteration < 250; ++iteration) {
        const uint16_t logicalPage =
            static_cast<uint16_t>(nextRand() % kEmsPagesUnderTest);
        const uint8_t physicalPage =
            static_cast<uint8_t>(nextRand() % hw::BIOS::EMS_PHYSICAL_PAGE_COUNT);
        const size_t pageOffset =
            static_cast<size_t>(nextRand() %
                                (hw::BIOS::EMS_PAGE_SIZE - kChunkSize + 1));
        const size_t diskOffset =
            static_cast<size_t>((nextRand() % (kDiskSize / kChunkSize)) *
                                kChunkSize);
        const uint16_t frameOffset = static_cast<uint16_t>(
            static_cast<uint32_t>(physicalPage) * hw::BIOS::EMS_PAGE_SIZE +
            pageOffset);

        emsMap(physicalPage, logicalPage, ioHandle);
        uint8_t *frame = framePtr(physicalPage);
        REQUIRE(frame != nullptr);
        REQUIRE(std::equal(frame, frame + expectedIoPages[logicalPage].size(),
                           expectedIoPages[logicalPage].begin()));

        switch (nextRand() % 3u) {
        case 0: {
            for (size_t i = 0; i < kChunkSize; ++i) {
                const uint8_t value = static_cast<uint8_t>(nextRand() >> 24);
                frame[pageOffset + i] = value;
                expectedIoPages[logicalPage][pageOffset + i] = value;
            }

            const uint8_t aliasPhysicalPage = static_cast<uint8_t>(
                (physicalPage + 1) % hw::BIOS::EMS_PHYSICAL_PAGE_COUNT);
            emsMap(aliasPhysicalPage, logicalPage, ioHandle);
            uint8_t *aliasFrame = framePtr(aliasPhysicalPage);
            REQUIRE(aliasFrame != nullptr);
            REQUIRE(std::equal(aliasFrame,
                               aliasFrame + expectedIoPages[logicalPage].size(),
                               expectedIoPages[logicalPage].begin()));
            break;
        }
        case 1:
            dosSeek(fileHandle, static_cast<uint32_t>(diskOffset));
            dosWrite(fileHandle, hw::BIOS::EMS_PAGE_FRAME_SEGMENT, frameOffset,
                     static_cast<uint16_t>(kChunkSize));
            std::copy_n(expectedIoPages[logicalPage].begin() +
                            static_cast<std::ptrdiff_t>(pageOffset),
                        kChunkSize,
                        expectedDisk.begin() +
                            static_cast<std::ptrdiff_t>(diskOffset));
            break;
        case 2:
            dosSeek(fileHandle, static_cast<uint32_t>(diskOffset));
            dosRead(fileHandle, hw::BIOS::EMS_PAGE_FRAME_SEGMENT, frameOffset,
                    static_cast<uint16_t>(kChunkSize));
            std::copy_n(expectedDisk.begin() +
                            static_cast<std::ptrdiff_t>(diskOffset),
                        kChunkSize,
                        expectedIoPages[logicalPage].begin() +
                            static_cast<std::ptrdiff_t>(pageOffset));
            REQUIRE(std::equal(frame, frame + expectedIoPages[logicalPage].size(),
                               expectedIoPages[logicalPage].begin()));
            break;
        }

        if ((iteration % 17u) == 0) {
            dosSeek(fileHandle, static_cast<uint32_t>(diskOffset));
            dosRead(fileHandle, kScratchSeg, 0x0000,
                    static_cast<uint16_t>(kChunkSize));
            REQUIRE(std::equal(scratch, scratch + kChunkSize,
                               expectedDisk.begin() +
                                   static_cast<std::ptrdiff_t>(diskOffset)));
        }
    }

    dosClose(fileHandle);

    std::ifstream hostDiskFile(tempFile, std::ios::binary);
    REQUIRE(hostDiskFile.is_open());
    std::vector<uint8_t> hostDisk(expectedDisk.size(), 0);
    hostDiskFile.read(reinterpret_cast<char *>(hostDisk.data()),
                      static_cast<std::streamsize>(hostDisk.size()));
    REQUIRE(static_cast<size_t>(hostDiskFile.gcount()) == hostDisk.size());
    REQUIRE(hostDisk == expectedDisk);
    hostDiskFile.close();

    verifyPages(ioHandle, expectedIoPages);
    emsRestore(ioHandle);
    emsDeallocate(ioHandle);

    cpu.setReg8(cpu::AH, 0x42);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    REQUIRE(cpu.getReg16(cpu::BX) == hw::BIOS::EMS_TOTAL_PAGES);
    REQUIRE(cpu.getReg16(cpu::DX) == hw::BIOS::EMS_TOTAL_PAGES);

    dosDelete(tempFile);
    std::ifstream deletedFile(tempFile, std::ios::binary);
    REQUIRE(!deletedFile.is_open());
}

// ═══════════════════════════════════════════════════════════════════════════
// DOS Date/Time & Misc Services Edge Cases
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DOS: Get/set date edge cases", "[DOS][DateTime]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();

    // Get current date (AH=2Ah)
    cpu.setReg8(cpu::AH, 0x2A);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));

    uint16_t year = cpu.getReg16(cpu::CX);
    uint8_t month = cpu.getReg8(cpu::DH);
    uint8_t day = cpu.getReg8(cpu::DL);
    uint8_t dayOfWeek = cpu.getReg8(cpu::AL);

    REQUIRE(year >= 1980);
    REQUIRE(month >= 1 && month <= 12);
    REQUIRE(day >= 1 && day <= 31);
    REQUIRE(dayOfWeek <= 6); // 0=Sunday, 6=Saturday
}

TEST_CASE("DOS: Get time edge cases", "[DOS][DateTime]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();

    // Get current time (AH=2Ch)
    cpu.setReg8(cpu::AH, 0x2C);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));

    uint8_t hours = cpu.getReg8(cpu::CH);
    uint8_t minutes = cpu.getReg8(cpu::CL);
    uint8_t seconds = cpu.getReg8(cpu::DH);
    uint8_t hundredths = cpu.getReg8(cpu::DL);

    REQUIRE(hours <= 23);
    REQUIRE(minutes <= 59);
    REQUIRE(seconds <= 59);
    REQUIRE(hundredths <= 99);
}

TEST_CASE("DOS: Get DOS version", "[DOS][Version]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();

    cpu.setReg8(cpu::AH, 0x30);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));

    uint8_t major = cpu.getReg8(cpu::AL);
    uint8_t minor = cpu.getReg8(cpu::AH);
    REQUIRE(major >= 3);
    REQUIRE(minor <= 99);
}

TEST_CASE("DOS: Set interrupt vector and get it back", "[DOS][IVT]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();

    // Set INT 0x60 vector to F000:1234
    cpu.setReg8(cpu::AH, 0x25);
    cpu.setReg8(cpu::AL, 0x60);
    cpu.setSegReg(cpu::DS, 0xF000);
    cpu.setSegBase(cpu::DS, 0xF0000);
    cpu.setReg16(cpu::DX, 0x1234);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));

    // Get it back
    cpu.setReg8(cpu::AH, 0x35);
    cpu.setReg8(cpu::AL, 0x60);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(cpu.getSegReg(cpu::ES) == 0xF000);
    REQUIRE(cpu.getReg16(cpu::BX) == 0x1234);
}

TEST_CASE("DOS: Get/set interrupt vector round-trip", "[DOS][IVT]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();

    // First get existing vector for INT 0x10
    cpu.setReg8(cpu::AH, 0x35);
    cpu.setReg8(cpu::AL, 0x10);
    dos.handleInterrupt(0x21);
    uint16_t origES = cpu.getSegReg(cpu::ES);
    uint16_t origBX = cpu.getReg16(cpu::BX);

    // Set a new one
    cpu.setReg8(cpu::AH, 0x25);
    cpu.setReg8(cpu::AL, 0x10);
    cpu.setSegReg(cpu::DS, 0x5000);
    cpu.setReg16(cpu::DX, 0xABCD);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));

    // Get it back — should be our new value
    cpu.setReg8(cpu::AH, 0x35);
    cpu.setReg8(cpu::AL, 0x10);
    dos.handleInterrupt(0x21);
    REQUIRE(cpu.getSegReg(cpu::ES) == 0x5000);
    REQUIRE(cpu.getReg16(cpu::BX) == 0xABCD);

    // Restore original
    cpu.setReg8(cpu::AH, 0x25);
    cpu.setReg8(cpu::AL, 0x10);
    cpu.setSegReg(cpu::DS, origES);
    cpu.setReg16(cpu::DX, origBX);
    dos.handleInterrupt(0x21);
}
