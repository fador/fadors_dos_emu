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
#include "hw/DPMI.hpp"
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

        REQUIRE(loader.loadCOM("test.com", 0x1000, dos));

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

TEST_CASE("DOS: AH=58h allocation strategy is dispatched", "[DOS][Memory]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();

    cpu.setReg16(cpu::AX, 0x5800);
    cpu.setEFLAGS(cpu.getEFLAGS() | cpu::FLAG_CARRY);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(cpu.getReg16(cpu::AX) == 0x0000);

    cpu.setReg16(cpu::AX, 0x5801);
    cpu.setReg16(cpu::BX, 0x0002);
    cpu.setEFLAGS(cpu.getEFLAGS() | cpu::FLAG_CARRY);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));

    cpu.setReg16(cpu::AX, 0x5800);
    cpu.setEFLAGS(cpu.getEFLAGS() | cpu::FLAG_CARRY);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(cpu.getReg16(cpu::AX) == 0x0002);
}

TEST_CASE("DOS: AH=71h returns unsupported LFN error", "[DOS][LFN]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();

    cpu.setReg16(cpu::AX, 0x71A0);
    cpu.setEFLAGS(cpu.getEFLAGS() | cpu::FLAG_CARRY);
    dos.handleInterrupt(0x21);

    REQUIRE(cpu.getEFLAGS() & cpu::FLAG_CARRY);
    REQUIRE(cpu.getReg16(cpu::AX) == 0x7100);
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
        write32(52, 8);      // ovrSize (8 bytes of data)
        write32(56, 0);      // exeInfo
        write32(60, 2);      // segNum (overlay ID = 2)
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

TEST_CASE("ProgramLoader zeroes EXE minimum allocation area", "[DOS][Loader]") {
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

    const char* fname = "minalloc_zero.exe";
    {
        std::vector<uint8_t> exe(48, 0);
        auto write16 = [&exe](size_t offset, uint16_t value) {
            exe[offset + 0] = static_cast<uint8_t>(value & 0xFF);
            exe[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
        };

        exe[0] = 'M';
        exe[1] = 'Z';
        write16(2, 48);      // Last page size
        write16(4, 1);       // Number of pages
        write16(6, 0);       // Relocations
        write16(8, 2);       // Header size paragraphs (32 bytes)
        write16(10, 1);      // minAlloc paragraph
        write16(12, 1);      // maxAlloc paragraph
        write16(14, 0);      // SS
        write16(16, 0x0200); // SP
        write16(18, 0);      // checksum
        write16(20, 0);      // IP
        write16(22, 0);      // CS
        write16(24, 0x001C); // relocation table offset
        write16(26, 0);      // overlay number

        exe[32] = 0x90;
        exe[33] = 0x90;
        exe[34] = 0xCD;
        exe[35] = 0x20;

        std::ofstream ofs(fname, std::ios::binary);
        ofs.write(reinterpret_cast<const char*>(exe.data()), static_cast<std::streamsize>(exe.size()));
    }

    const uint16_t pspSegment = dos.getPSPSegment();
    const uint32_t loadAddr = static_cast<uint32_t>(pspSegment + 0x10) << 4;
    const uint32_t imageSize = 16;
    const uint32_t minAllocAddr = loadAddr + imageSize;
    const uint32_t farBssAddr = loadAddr + 0x2000;

    mem.write8(minAllocAddr + 0, 0xCC);
    mem.write8(minAllocAddr + 15, 0xCC);
    mem.write8(farBssAddr, 0xCC);

    REQUIRE(loader.loadEXE(fname, pspSegment, dos));
    REQUIRE(mem.read8(minAllocAddr + 0) == 0x00);
    REQUIRE(mem.read8(minAllocAddr + 15) == 0x00);
    REQUIRE(mem.read8(farBssAddr) == 0x00);

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

TEST_CASE("DOS: Nested Exec (AH=4Bh) and Get Return Code (AH=4Dh)", "[DOS][Exec]") {
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

    // 1. Create child.com
    {
        std::ofstream ofs("child.com", std::ios::binary);
        uint8_t childCode[] = { 0xB4, 0x4C, 0xB0, 0x2A, 0xCD, 0x21 }; // MOV AH, 4Ch, MOV AL, 2Ah (42), INT 21h
        ofs.write(reinterpret_cast<const char*>(childCode), sizeof(childCode));
    }

    // 2. Create parent.com
    {
        std::ofstream ofs("parent.com", std::ios::binary);
        std::vector<uint8_t> parentImg(0x180, 0);

        // Code at 100h (offset 0 in COM file, which starts at 100h in memory)
        uint8_t code[] = {
            0xBB, 0x40, 0x00,                   // mov bx, 0x40 (shrink memory to 1024 bytes)
            0xB4, 0x4A,                         // mov ah, 0x4A
            0xCD, 0x21,                         // int 0x21 (Resize block)
            0x8C, 0xC8,                         // mov ax, cs
            0xA3, 0x62, 0x01,                   // mov [0x162], ax
            0xA3, 0x66, 0x01,                   // mov [0x166], ax
            0xA3, 0x6A, 0x01,                   // mov [0x16A], ax
            0xBA, 0x50, 0x01,                   // mov dx, 0x150
            0xBB, 0x60, 0x01,                   // mov bx, 0x160
            0xB8, 0x00, 0x4B,                   // mov ax, 0x4B00
            0xCD, 0x21,                         // int 0x21 (Exec child)
            0xB4, 0x4D,                         // mov ah, 0x4D (Get Exit Code)
            0xCD, 0x21,                         // int 0x21
            0xB4, 0x4C,                         // mov ah, 0x4C
            0xCD, 0x21                          // int 0x21
        };
        std::copy(std::begin(code), std::end(code), parentImg.begin());

        // Filename at 150h (offset 0x50 in parentImg)
        std::string filename = "child.com";
        std::copy(filename.begin(), filename.end(), parentImg.begin() + 0x50);
        parentImg[0x50 + filename.size()] = 0; // null terminator

        // Parameter block at 160h (offset 0x60 in parentImg)
        parentImg[0x60] = 0; parentImg[0x61] = 0; // envSeg = 0
        parentImg[0x62] = 0x70; parentImg[0x63] = 0x01; // cmdTail offset = 0x170

        // Command tail at 170h (offset 0x70 in parentImg)
        parentImg[0x70] = 0; // length
        parentImg[0x71] = 0x0D; // CR

        ofs.write(reinterpret_cast<const char*>(parentImg.data()), parentImg.size());
    }

    // Load parent.com at the default PSP segment (0x0812)
    uint16_t parentSeg = dos.getPSPSegment();
    REQUIRE(loader.loadCOM("parent.com", parentSeg, dos));

    // Run execution loop until terminated
    int instructions = 0;
    while (!dos.isTerminated() && instructions < 1000) {
        decoder.step();
        instructions++;
    }

    // Verify exit code is 42
    REQUIRE(dos.isTerminated());
    REQUIRE(dos.getExitCode() == 42);

    std::remove("child.com");
    std::remove("parent.com");
}

TEST_CASE("DOS: Process State Save and Restore (CR0 and Descriptors)", "[DOS][Exec]") {
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

    // 1. Create child.com that exits immediately
    {
        std::ofstream ofs("child.com", std::ios::binary);
        uint8_t childCode[] = { 0xB4, 0x4C, 0xB0, 0x00, 0xCD, 0x21 }; // MOV AH, 4Ch, MOV AL, 00h, INT 21h
        ofs.write(reinterpret_cast<const char*>(childCode), sizeof(childCode));
    }

    // 2. Create parent.com
    {
        std::ofstream ofs("parent.com", std::ios::binary);
        std::vector<uint8_t> parentImg(0x180, 0);

        // Code at 100h: shrink block, then exec child, then exit
        uint8_t code[] = {
            0xBB, 0x40, 0x00,                   // mov bx, 0x40
            0xB4, 0x4A,                         // mov ah, 0x4A
            0xCD, 0x21,                         // int 0x21 (Resize block)
            0x8C, 0xC8,                         // mov ax, cs
            0xA3, 0x62, 0x01,                   // mov [0x162], ax
            0xA3, 0x66, 0x01,                   // mov [0x166], ax
            0xA3, 0x6A, 0x01,                   // mov [0x16A], ax
            0xBA, 0x50, 0x01,                   // mov dx, 0x150
            0xBB, 0x60, 0x01,                   // mov bx, 0x160
            0xB8, 0x00, 0x4B,                   // mov ax, 0x4B00
            0xCD, 0x21,                         // int 0x21 (Exec child)
            0xB4, 0x4C,                         // mov ah, 0x4C
            0xCD, 0x21                          // int 0x21
        };
        std::copy(std::begin(code), std::end(code), parentImg.begin());

        // Filename at 150h
        std::string filename = "child.com";
        std::copy(filename.begin(), filename.end(), parentImg.begin() + 0x50);
        parentImg[0x50 + filename.size()] = 0;

        // Parameter block at 160h
        parentImg[0x60] = 0; parentImg[0x61] = 0;
        parentImg[0x62] = 0x70; parentImg[0x63] = 0x01;

        // Command tail at 170h
        parentImg[0x70] = 0;
        parentImg[0x71] = 0x0D;

        ofs.write(reinterpret_cast<const char*>(parentImg.data()), parentImg.size());
    }

    // Load parent.com at the default PSP segment (0x0812)
    uint16_t parentSeg = dos.getPSPSegment();
    REQUIRE(loader.loadCOM("parent.com", parentSeg, dos));

    // Set custom registers in parent before execution
    cpu.setCR(0, 0x00000010); // Real mode CR0 with some bits
    cpu.setGDTR({0x1234, 0x56789ABC});
    cpu.setIDTR({0x4321, 0x98765432});
    cpu.setLDTR({0x1111, 0x22222222});
    cpu.setTR({0x3333, 0x44444444});
    cpu.setLDTRSelector(0x5555);
    cpu.setTRSelector(0x6666);

    // Step instructions. When parent does INT 21h (Exec child),
    // the emulator will save these parent registers.
    // In child process, we can check that we run (and optionally child sets CR0=1, i.e., Protected Mode).
    int instructions = 0;
    bool childLoaded = false;
    while (!dos.isTerminated() && instructions < 1000) {
        decoder.step();
        instructions++;

        // Once the child runs, it's at CS != parentSeg
        if (cpu.getSegReg(cpu::CS) != parentSeg) {
            childLoaded = true;
            // Simulate child changing these registers
            cpu.setCR(0, 0x60000000); // Real mode (PE=0) with CD and NW flags set
            cpu.setGDTR({0xFFFF, 0xFFFFFFFF});
            cpu.setIDTR({0xAAAA, 0xBBBBBBBB});
            cpu.setLDTR({0xCCCC, 0xDDDDDDDD});
            cpu.setTR({0xEEEE, 0x00000000});
            cpu.setLDTRSelector(0x9999);
            cpu.setTRSelector(0x8888);
        }
    }

    // After termination, parent registers should be restored back to the custom values
    REQUIRE(childLoaded);
    REQUIRE(dos.isTerminated());
    REQUIRE(cpu.getCR(0) == 0x00000010);
    REQUIRE(cpu.getGDTR().limit == 0x1234);
    REQUIRE(cpu.getGDTR().base == 0x56789ABC);
    REQUIRE(cpu.getIDTR().limit == 0x4321);
    REQUIRE(cpu.getIDTR().base == 0x98765432);
    REQUIRE(cpu.getLDTR().limit == 0x1111);
    REQUIRE(cpu.getLDTR().base == 0x22222222);
    REQUIRE(cpu.getTR().limit == 0x3333);
    REQUIRE(cpu.getTR().base == 0x44444444);
    REQUIRE(cpu.getLDTRSelector() == 0x5555);
    REQUIRE(cpu.getTRSelector() == 0x6666);

    std::remove("child.com");
    std::remove("parent.com");
}

TEST_CASE("DOS: ProgramLoader respects envSeg and Multiplex AX=FB42h check", "[DOS][Loader]") {
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
    
    hw::ProgramLoader loader(cpu, mem, dos.getHIMEM());

    // 1. Create a dummy child
    {
        std::ofstream ofs("dummy_child.com", std::ios::binary);
        uint8_t code[] = { 0xCB }; // RETF
        ofs.write(reinterpret_cast<const char*>(code), sizeof(code));
    }

    // Load with a specific envSeg
    uint16_t customEnvSeg = 0x1234;
    uint16_t childSeg = 0x2000;
    REQUIRE(loader.loadCOM("dummy_child.com", childSeg, dos, "", customEnvSeg));

    // Verify PSP offset 0x2C is set to customEnvSeg
    uint32_t pspPhys = (static_cast<uint32_t>(childSeg) << 4);
    uint16_t loadedEnvSeg = mem.read16(pspPhys + 0x2C);
    REQUIRE(loadedEnvSeg == customEnvSeg);

    std::remove("dummy_child.com");
}

TEST_CASE("DOS: INT 21h AH=5Dh Swappable Data Area and fallback", "[DOS]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::DOS dos(cpu, mem);
    dos.initialize();

    SECTION("AH=5Dh AL=06h (Get SDA address) in Real Mode") {
        cpu.setReg8(cpu::AH, 0x5D);
        cpu.setReg8(cpu::AL, 0x06);
        cpu.setReg16(cpu::SI, 0x0000);
        cpu.setSegReg(cpu::DS, 0x0000);
        cpu.setEFLAGS(cpu.getEFLAGS() | cpu::FLAG_CARRY); // Start with CF set

        REQUIRE(dos.handleInterrupt(0x21));

        REQUIRE(cpu.getSegReg(cpu::DS) == 0x0070);
        REQUIRE(cpu.getReg16(cpu::SI) == 0x000F);
        REQUIRE(cpu.getReg16(cpu::CX) == 0x80);
        REQUIRE(cpu.getReg16(cpu::DX) == 0x18);
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY)); // CF must be clear

        // Verify current PSP segment is written to physical 0x071F
        REQUIRE(mem.read16(0x071F) == dos.getPSPSegment());
    }

    SECTION("AH=5Dh AL=0Ah (Set Extended Error Info)") {
        cpu.setReg8(cpu::AH, 0x5D);
        cpu.setReg8(cpu::AL, 0x0A);
        cpu.setEFLAGS(cpu.getEFLAGS() | cpu::FLAG_CARRY); // Start with CF set

        REQUIRE(dos.handleInterrupt(0x21));
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY)); // CF must be clear
    }

    SECTION("AH=5Dh AL=0xFF (Unknown subfunction)") {
        cpu.setReg8(cpu::AH, 0x5D);
        cpu.setReg8(cpu::AL, 0xFF);
        cpu.setEFLAGS(cpu.getEFLAGS() & ~cpu::FLAG_CARRY); // Start with CF clear

        REQUIRE(dos.handleInterrupt(0x21));
        REQUIRE(cpu.getReg16(cpu::AX) == 0x0001);
        REQUIRE((cpu.getEFLAGS() & cpu::FLAG_CARRY)); // CF must be set
    }

    SECTION("Unknown INT 21h AH=0xFA (Robust error return)") {
        cpu.setReg8(cpu::AH, 0xFA);
        cpu.setEFLAGS(cpu.getEFLAGS() & ~cpu::FLAG_CARRY); // Start with CF clear

        REQUIRE(dos.handleInterrupt(0x21));
        REQUIRE(cpu.getReg16(cpu::AX) == 0x0001);
        REQUIRE((cpu.getEFLAGS() & cpu::FLAG_CARRY)); // CF must be set
    }
}

TEST_CASE("DOS: Date, Time and Forward Slash Path Resolution", "[DOS]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::DOS dos(cpu, mem);
    dos.initialize();

    SECTION("AH=2Ah Get System Date") {
        cpu.setReg8(cpu::AH, 0x2A);
        REQUIRE(dos.handleInterrupt(0x21));

        uint16_t year = cpu.getReg16(cpu::CX);
        uint8_t month = cpu.getReg8(cpu::DH);
        uint8_t day = cpu.getReg8(cpu::DL);
        uint8_t dayOfWeek = cpu.getReg8(cpu::AL);

        REQUIRE(year >= 2026);
        REQUIRE(month >= 1);
        REQUIRE(month <= 12);
        REQUIRE(day >= 1);
        REQUIRE(day <= 31);
        REQUIRE(dayOfWeek <= 6);
    }

    SECTION("AH=2Ch Get System Time") {
        cpu.setReg8(cpu::AH, 0x2C);
        REQUIRE(dos.handleInterrupt(0x21));

        uint8_t hour = cpu.getReg8(cpu::CH);
        uint8_t minute = cpu.getReg8(cpu::CL);
        uint8_t second = cpu.getReg8(cpu::DH);
        uint8_t centisecond = cpu.getReg8(cpu::DL);

        REQUIRE(hour <= 23);
        REQUIRE(minute <= 59);
        REQUIRE(second <= 59);
        REQUIRE(centisecond <= 99);
    }

    SECTION("Forward Slash Path Resolution") {
        // Create a temporary file in the current host directory
        const std::string fname = "fslash.txt";
        {
            std::ofstream ofs(fname);
            ofs << "forward slash test";
        }

        // Try to open it using /fslash.txt (starts with forward slash)
        // Set DS:DX to point to "/fslash.txt"
        const std::string dosPath = "/fslash.txt";
        const uint32_t pathAddr = 0x70000;
        for (size_t i = 0; i < dosPath.size(); ++i) {
            mem.write8(pathAddr + i, dosPath[i]);
        }
        mem.write8(pathAddr + dosPath.size(), 0); // Null terminator

        cpu.setSegReg(cpu::DS, 0x7000);
        cpu.setSegBase(cpu::DS, 0x70000);
        cpu.setReg16(cpu::DX, 0x0000);
        cpu.setReg8(cpu::AH, 0x3D); // Open File
        cpu.setReg8(cpu::AL, 0x00); // Read-only
        
        REQUIRE(dos.handleInterrupt(0x21));
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));

        uint16_t handle = cpu.getReg16(cpu::AX);

        // Close the file
        cpu.setReg8(cpu::AH, 0x3E);
        cpu.setReg16(cpu::BX, handle);
        REQUIRE(dos.handleInterrupt(0x21));
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));

        std::remove(fname.c_str());
    }
}

TEST_CASE("DOS: Process State Segment Cache Sync on Termination", "[DOS][Exec]") {
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

    // 1. Create child.com that exits immediately with code 55
    {
        std::ofstream ofs("child.com", std::ios::binary);
        uint8_t childCode[] = { 0xB4, 0x4C, 0xB0, 0x37, 0xCD, 0x21 }; // MOV AH, 4Ch, MOV AL, 55 (0x37), INT 21h
        ofs.write(reinterpret_cast<const char*>(childCode), sizeof(childCode));
    }

    // 2. Create parent.com
    {
        std::ofstream ofs("parent.com", std::ios::binary);
        std::vector<uint8_t> parentImg(0x180, 0);

        // Code at 100h: shrink block, then exec child, then exit with its exit code
        uint8_t code[] = {
            0xBB, 0x40, 0x00,                   // mov bx, 0x40
            0xB4, 0x4A,                         // mov ah, 0x4A
            0xCD, 0x21,                         // int 0x21 (Resize block)
            0x8C, 0xC8,                         // mov ax, cs
            0xA3, 0x62, 0x01,                   // mov [0x162], ax
            0xA3, 0x66, 0x01,                   // mov [0x166], ax
            0xA3, 0x6A, 0x01,                   // mov [0x16A], ax
            0xBA, 0x50, 0x01,                   // mov dx, 0x150
            0xBB, 0x60, 0x01,                   // mov bx, 0x160
            0xB8, 0x00, 0x4B,                   // mov ax, 0x4B00
            0xCD, 0x21,                         // int 0x21 (Exec child)
            0xB4, 0x4C,                         // mov ah, 0x4C
            0xB0, 0x2A,                         // mov al, 42
            0xCD, 0x21                          // int 0x21
        };
        std::copy(std::begin(code), std::end(code), parentImg.begin());

        // Filename at 150h
        std::string filename = "child.com";
        std::copy(filename.begin(), filename.end(), parentImg.begin() + 0x50);
        parentImg[0x50 + filename.size()] = 0;

        // Parameter block at 160h
        parentImg[0x60] = 0; parentImg[0x61] = 0;
        parentImg[0x62] = 0x70; parentImg[0x63] = 0x01;

        // Command tail at 170h
        parentImg[0x70] = 0;
        parentImg[0x71] = 0x0D;

        ofs.write(reinterpret_cast<const char*>(parentImg.data()), parentImg.size());
    }

    // Load parent.com at the default PSP segment (0x0812)
    uint16_t parentSeg = dos.getPSPSegment();
    REQUIRE(loader.loadCOM("parent.com", parentSeg, dos));

    // Run execution loop
    int instructions = 0;
    bool childLoaded = false;
    while (!dos.isTerminated() && instructions < 1000) {
        decoder.step();
        instructions++;

        if (cpu.getSegReg(cpu::CS) != parentSeg) {
            childLoaded = true;
        }
    }

    // After termination, the exit code should be 42
    REQUIRE(childLoaded);
    REQUIRE(dos.isTerminated());
    REQUIRE(dos.getExitCode() == 42);

    std::remove("child.com");
    std::remove("parent.com");
}

TEST_CASE("DOS: Set/Get Current PSP (AH=50h/51h/62h)", "[DOS][PSP]") {
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
    hw::DPMI dpmi(cpu, mem);
    memory::HIMEM himem;
    dpmi.setDOS(&dos);
    dos.setDPMI(&dpmi);
    dpmi.setBIOS(&bios);
    dpmi.setHIMEM(&himem);
    himem.setMemoryBus(&mem);

    // Initial PSP
    uint16_t initialPSP = dos.getPSPSegment();
    REQUIRE(initialPSP == 0x0812);

    // 1. Get current PSP in Real Mode
    cpu.setReg8(cpu::AH, 0x51);
    dos.handleInterrupt(0x21);
    REQUIRE(cpu.getReg16(cpu::BX) == initialPSP);

    cpu.setReg8(cpu::AH, 0x62);
    dos.handleInterrupt(0x21);
    REQUIRE(cpu.getReg16(cpu::BX) == initialPSP);

    // 2. Set current PSP in Real Mode
    cpu.setReg8(cpu::AH, 0x50);
    cpu.setReg16(cpu::BX, 0x0820);
    dos.handleInterrupt(0x21);
    REQUIRE(dos.getPSPSegment() == 0x0820);

    cpu.setReg8(cpu::AH, 0x51);
    dos.handleInterrupt(0x21);
    REQUIRE(cpu.getReg16(cpu::BX) == 0x0820);

    // 3. Set/Get PSP in Protected Mode
    cpu.setCR(0, cpu.getCR(0) | 1); // Enter PM
    
    // Set up stack for handleEntry
    cpu.setSegReg(cpu::CS, 0x1000);
    cpu.setSegReg(cpu::SS, 0x0000);
    cpu.setReg16(cpu::SP, 0x100);
    cpu.setReg16(cpu::AX, 0x0001); // 32-bit client
    
    // Push return frame
    uint16_t sp = cpu.getReg16(cpu::SP);
    sp -= 2; mem.write16(sp, 0x2000); // CS
    sp -= 2; mem.write16(sp, 0x0100); // IP
    cpu.setReg16(cpu::SP, sp);

    REQUIRE(dpmi.handleEntry());
    REQUIRE(dpmi.isActive());

    uint16_t pspSel = dpmi.getPSPSelector();
    
    // Get PSP in PM
    cpu.setReg8(cpu::AH, 0x51);
    dos.handleInterrupt(0x21);
    REQUIRE(cpu.getReg16(cpu::BX) == pspSel);

    // Set PSP in PM to a new selector
    // We allocate a new selector
    cpu.setReg16(cpu::AX, 0x0000); // Alloc descriptor
    cpu.setReg16(cpu::CX, 1);
    REQUIRE(dpmi.handleInt31());
    uint16_t newPSPSel = cpu.getReg16(cpu::AX);

    // Set base address to 0x8300 (segment 0x830)
    cpu.setReg16(cpu::AX, 0x0007);
    cpu.setReg16(cpu::BX, newPSPSel);
    cpu.setReg16(cpu::CX, 0x0000);
    cpu.setReg16(cpu::DX, 0x8300);
    REQUIRE(dpmi.handleInt31());

    // Call Set Current PSP with new selector
    cpu.setReg8(cpu::AH, 0x50);
    cpu.setReg16(cpu::BX, newPSPSel);
    dos.handleInterrupt(0x21);
    REQUIRE(dos.getPSPSegment() == 0x0830);

    // Get current PSP and make sure it returns the selector
    cpu.setReg8(cpu::AH, 0x62);
    dos.handleInterrupt(0x21);
    REQUIRE(cpu.getReg16(cpu::BX) == newPSPSel);
}

TEST_CASE("DOS: Child Environment Duplication", "[DOS][Exec]") {
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

    // Write child.com (exits immediately)
    {
        std::ofstream ofs("child.com", std::ios::binary);
        uint8_t childCode[] = { 0xB4, 0x4C, 0xB0, 0x2A, 0xCD, 0x21 }; // MOV AH, 4Ch, MOV AL, 2Ah, INT 21h (exit 42)
        ofs.write(reinterpret_cast<const char*>(childCode), sizeof(childCode));
    }

    // Write parent.com
    {
        std::ofstream ofs("parent.com", std::ios::binary);
        std::vector<uint8_t> parentImg(0x180, 0);

        // Code at 100h: shrink block, then exec child, then exit
        uint8_t parentCode[] = {
            0xBB, 0x40, 0x00,       // mov bx, 0x40 (shrink to 64 paragraphs = 1024 bytes)
            0xB4, 0x4A,             // mov ah, 0x4A
            0xCD, 0x21,             // int 0x21 (Resize block)
            0x8C, 0xC8,             // mov ax, cs
            0xA3, 0x62, 0x01,       // mov [0x162], ax (patch cmdTail segment in param block)
            0xBA, 0x50, 0x01,       // mov dx, 0x150 (filename at 150h)
            0xBB, 0x60, 0x01,       // mov bx, 0x160 (param block at 160h)
            0xB8, 0x00, 0x4B,       // mov ax, 0x4B00 (Exec child)
            0xCD, 0x21,             // int 0x21
            0xB4, 0x4D,             // mov ah, 0x4D
            0xCD, 0x21,             // int 0x21
            0xB4, 0x4C,             // mov ah, 0x4C
            0xCD, 0x21              // int 0x21
        };
        std::copy(parentCode, parentCode + sizeof(parentCode), parentImg.begin());

        // Filename at 150h (offset 0x50)
        std::string fn = "child.com";
        std::copy(fn.begin(), fn.end(), parentImg.begin() + 0x50);
        parentImg[0x50 + fn.length()] = 0; // null terminator

        // Param block at 160h (offset 0x60)
        parentImg[0x60] = 0x00; parentImg[0x61] = 0x00; // envSeg = 0 (inherit)
        parentImg[0x62] = 0x70; parentImg[0x63] = 0x01; // cmdTail offset 170h (offset 0x70)

        // cmdTail at 170h (offset 0x70)
        parentImg[0x70] = 0x00; // length
        parentImg[0x71] = 0x0D; // terminator

        ofs.write(reinterpret_cast<const char*>(parentImg.data()), parentImg.size());
    }

    uint16_t parentSeg = dos.getPSPSegment();
    REQUIRE(loader.loadCOM("parent.com", parentSeg, dos));

    // Patch cmdTailSeg (at 0x164, file offset 0x64) in parent memory to parentSeg
    uint32_t parentPhys = (static_cast<uint32_t>(parentSeg) << 4);
    mem.write16(parentPhys + 0x164, parentSeg);

    // Create a mock environment for parent PSP first
    // Parent env block is at parentSeg - 0x20
    uint16_t parentEnvSeg = parentSeg - 0x20;
    
    // We already wrote a mock MCB at parentEnvSeg - 1 in createPSP!
    // Let's verify it
    hw::DOS::MCB envMcb = dos.readMCB(parentEnvSeg - 1);
    REQUIRE(envMcb.type == 'M');
    REQUIRE(envMcb.owner == parentSeg);

    // Let's write some dummy env variables to parent env block
    uint32_t parentEnvAddr = (parentEnvSeg << 4);
    mem.write8(parentEnvAddr, 'A');
    mem.write8(parentEnvAddr + 1, '=');
    mem.write8(parentEnvAddr + 2, 'B');
    mem.write8(parentEnvAddr + 3, '\0');
    mem.write8(parentEnvAddr + 4, '\0');

    // Run execution loop
    int instructions = 0;
    uint16_t childSeg = 0;
    while (!dos.isTerminated() && instructions < 1000) {
        decoder.step();
        instructions++;

        if (cpu.getSegReg(cpu::CS) != parentSeg && cpu.getSegReg(cpu::CS) != 0xF000) {
            childSeg = cpu.getSegReg(cpu::CS);
        }
    }

    REQUIRE(dos.isTerminated());
    REQUIRE(dos.getExitCode() == 42); // child exited with 2Ah = 42

    // Verify child segment was allocated
    REQUIRE(childSeg != 0);

    // Verify child environment was allocated and copied
    uint16_t childEnvSeg = mem.read16((childSeg << 4) + 0x2C);
    REQUIRE(childEnvSeg != 0);
    REQUIRE(childEnvSeg != parentEnvSeg);

    // Verify child env block contains parent variables
    uint32_t childEnvAddr = (childEnvSeg << 4);
    REQUIRE(mem.read8(childEnvAddr) == 'A');
    REQUIRE(mem.read8(childEnvAddr + 1) == '=');
    REQUIRE(mem.read8(childEnvAddr + 2) == 'B');

    // Verify that the child's environment block was freed after termination
    // The MCB at childEnvSeg - 1 should have owner == 0 now
    hw::DOS::MCB childEnvMcb = dos.readMCB(childEnvSeg - 1);
    REQUIRE(childEnvMcb.owner == 0);

    std::remove("child.com");
    std::remove("parent.com");
}

// ═══════════════════════════════════════════════════════════════════════════
// VROOMM / NE Overlay Segment Loading via INT 3Fh
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("VROOMM: NE overlay segments load with relocation fixups", "[DOS][VROOMM][NE]") {
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

    const char* fname = "ne_overlay2.exe";
    {
        auto write16 = [](std::vector<uint8_t>& v, size_t off, uint16_t val) {
            v[off] = val & 0xFF; v[off+1] = (val >> 8) & 0xFF;
        };
        std::vector<uint8_t> exe(1536, 0);
        exe[0]='M'; exe[1]='Z';
        write16(exe, 2, 0); write16(exe, 4, 2); // 2 pages = 1024 bytes
        write16(exe, 8, 2); write16(exe, 10, 0); write16(exe, 12, 0);
        write16(exe, 14, 0); write16(exe, 16, 0x100);
        write16(exe, 20, 0); write16(exe, 22, 0);
        write16(exe, 0x3C, 0x40);

        exe[32]=0x90; exe[33]=0x90; // NOPs as stub

        size_t ne = 0x40;
        exe[ne]='N'; exe[ne+1]='E';
        write16(exe, ne+0x0E, 0x03); write16(exe, ne+0x18, 1);
        write16(exe, ne+0x1C, 2);    // numSegments = 2
        write16(exe, ne+0x22, 0x40); // segTableOffset = 0x40 (at 0x40+0x40=0x80)
        write16(exe, ne+0x32, 9);    // alignShift = 9
        exe[ne+0x36] = 1;

        // Segment table at file offset 0x80 (0x40 + 0x40)
        size_t segTab = 0x80;
        // Segment 1: resident data at sector 0
        write16(exe, segTab+0, 0); write16(exe, segTab+2, 0x10);
        write16(exe, segTab+4, 0x0010); write16(exe, segTab+6, 0x10);
        // Segment 2: overlay at sector 1 (file offset 512)
        write16(exe, segTab+8, 1); write16(exe, segTab+10, 0x10);
        write16(exe, segTab+12, 0x0010); write16(exe, segTab+14, 0x10);

        // Overlay data at sector 1 (file offset 512)
        for (int i=0;i<16;i++) exe[512+i]=static_cast<uint8_t>(0xCC+i);

        std::ofstream ofs(fname, std::ios::binary);
        ofs.write(reinterpret_cast<const char*>(exe.data()), 1536);
    }

    REQUIRE(loader.loadEXE(fname, dos.getPSPSegment(), dos));

    cpu.setInstructionStartEIP(0x100);
    cpu.setSegReg(cpu::CS, 0x2000);
    cpu.setSegBase(cpu::CS, 0x20000);
    cpu.setEIP(0x102);

    mem.write8(0x20100, 0xCD); mem.write8(0x20101, 0x3F);
    mem.write16(0x20102, 0x0008);
    mem.write16(0x20104, 0x0002); // segment index 2

    REQUIRE(dos.handleInterrupt(0x3F));
    REQUIRE(mem.read8(0x20100) == 0xEA); // JMP FAR patched
    uint16_t loadedSeg = mem.read16(0x20103);
    REQUIRE(loadedSeg != 0);

    // Verify at least the data was loaded
    uint32_t loadAddr = static_cast<uint32_t>(loadedSeg) << 4;
    REQUIRE(mem.read8(loadAddr) == 0xCC);

    std::remove(fname);
}

TEST_CASE("VROOMM: reloading already-loaded segment returns cached segment", "[DOS][VROOMM]") {
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

    const char* fname = "ne_cached.exe";
    {
        auto write16 = [](std::vector<uint8_t>& v, size_t off, uint16_t val) {
            v[off] = val & 0xFF; v[off+1] = (val >> 8) & 0xFF;
        };
        std::vector<uint8_t> exe(1024, 0);
        exe[0]='M'; exe[1]='Z';
        write16(exe, 2, 0); write16(exe, 4, 1);
        write16(exe, 8, 2); write16(exe, 20, 0); write16(exe, 22, 0);
        write16(exe, 0x3C, 0x40);

        exe[32]=0xB8; exe[33]=0x00; exe[34]=0x4C; exe[35]=0xCD; exe[36]=0x21;

        size_t ne = 0x40;
        exe[ne]='N'; exe[ne+1]='E';
        write16(exe, ne+0x0E, 0x03); write16(exe, ne+0x18, 1);
        write16(exe, ne+0x1C, 1); write16(exe, ne+0x22, 0x80);
        write16(exe, ne+0x32, 9); exe[ne+0x36] = 1;

        size_t segTab = 0x80;
        write16(exe, segTab+0, 1); write16(exe, segTab+2, 8);
        write16(exe, segTab+4, 0x0010); write16(exe, segTab+6, 0x10);

        for (int i=0;i<8;i++) exe[512+i]=0xCC;

        std::ofstream ofs(fname, std::ios::binary);
        ofs.write(reinterpret_cast<const char*>(exe.data()), 1024);
    }

    REQUIRE(loader.loadEXE(fname, dos.getPSPSegment(), dos));

    cpu.setInstructionStartEIP(0x100);
    cpu.setSegReg(cpu::CS, 0x2000);
    cpu.setSegBase(cpu::CS, 0x20000);

    // First load
    cpu.setEIP(0x102);
    mem.write8(0x20100, 0xCD); mem.write8(0x20101, 0x3F);
    mem.write16(0x20102, 0x0000);
    mem.write16(0x20104, 0x0001);
    REQUIRE(dos.handleInterrupt(0x3F));
    uint16_t firstSeg = mem.read16(0x20103);
    REQUIRE(firstSeg != 0);

    // Second load → cached
    mem.write8(0x20100, 0xCD); mem.write8(0x20101, 0x3F);
    mem.write16(0x20102, 0x0000);
    mem.write16(0x20104, 0x0001);
    cpu.setEIP(0x102);
    REQUIRE(dos.handleInterrupt(0x3F));
    REQUIRE(mem.read16(0x20103) == firstSeg);

    std::remove(fname);
}

TEST_CASE("VROOMM: out-of-bounds segment index returns zero", "[DOS][VROOMM]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();

    cpu.setInstructionStartEIP(0x100);
    cpu.setSegReg(cpu::CS, 0x2000);
    cpu.setSegBase(cpu::CS, 0x20000);
    cpu.setEIP(0x102);

    mem.write8(0x20100, 0xCD); mem.write8(0x20101, 0x3F);
    mem.write16(0x20102, 0x0000);
    mem.write16(0x20104, 0x0099);

    REQUIRE(dos.handleInterrupt(0x3F));
    REQUIRE(mem.read8(0x20100) == 0xCD);
    REQUIRE(mem.read8(0x20101) == 0x3F);
}

TEST_CASE("VROOMM: FBOV multiple overlays load independently", "[DOS][VROOMM][FBOV]") {
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

    const char* fname = "fbov_multi2.exe";
    {
        auto write16 = [](std::vector<uint8_t>& v, size_t off, uint16_t val) {
            v[off] = val & 0xFF; v[off+1] = (val >> 8) & 0xFF;
        };
        auto write32 = [](std::vector<uint8_t>& v, size_t off, uint32_t val) {
            v[off]=val&0xFF; v[off+1]=(val>>8)&0xFF; v[off+2]=(val>>16)&0xFF; v[off+3]=(val>>24)&0xFF;
        };

        // 2-page EXE: MZ image fills 1024 bytes, then FBOV blocks follow
        std::vector<uint8_t> exe(1536, 0);
        exe[0]='M'; exe[1]='Z';
        write16(exe, 2, 0);     // lastPageSize = 0 (full 512 for last page)
        write16(exe, 4, 2);     // numPages = 2 (1024 bytes)
        write16(exe, 8, 2);     // headerSize = 2 paragraphs
        write16(exe, 10, 0);    // minAlloc = 0
        write16(exe, 12, 0);    // maxAlloc = 0
        write16(exe, 14, 0); write16(exe, 16, 0x100);
        write16(exe, 20, 0); write16(exe, 22, 0);
        exe[32]=0x90; exe[33]=0x90;

        // FBOV block 1 at offset 1024 (right after MZ image)
        size_t fb = 1024;
        exe[fb]='F'; exe[fb+1]='B'; exe[fb+2]='O'; exe[fb+3]='V';
        write32(exe, fb+4, 8); write32(exe, fb+8, 0); write32(exe, fb+12, 1);
        for (int i=0;i<8;i++) exe[fb+16+i]=static_cast<uint8_t>(0xA0+i);

        // FBOV block 2 immediately after block 1
        fb = 1024 + 28;
        exe[fb]='F'; exe[fb+1]='B'; exe[fb+2]='O'; exe[fb+3]='V';
        write32(exe, fb+4, 8); write32(exe, fb+8, 0); write32(exe, fb+12, 2);
        for (int i=0;i<8;i++) exe[fb+16+i]=static_cast<uint8_t>(0xB0+i);

        std::ofstream ofs(fname, std::ios::binary);
        ofs.write(reinterpret_cast<const char*>(exe.data()), static_cast<std::streamsize>(exe.size()));
    }

    REQUIRE(loader.loadEXE(fname, dos.getPSPSegment(), dos));

    cpu.setInstructionStartEIP(0x100);
    cpu.setSegReg(cpu::CS, 0x2000);
    cpu.setSegBase(cpu::CS, 0x20000);

    // Load overlay 1
    cpu.setEIP(0x102);
    mem.write8(0x20100, 0xCD); mem.write8(0x20101, 0x3F);
    mem.write16(0x20102, 0x0000); mem.write16(0x20104, 0x0001);
    REQUIRE(dos.handleInterrupt(0x3F));
    REQUIRE(mem.read8(0x20100) == 0xEA);
    uint16_t seg1 = mem.read16(0x20103);
    REQUIRE(seg1 != 0);

    // Load overlay 2
    mem.write8(0x20100, 0xCD); mem.write8(0x20101, 0x3F);
    mem.write16(0x20102, 0x0000); mem.write16(0x20104, 0x0002);
    cpu.setEIP(0x102);
    REQUIRE(dos.handleInterrupt(0x3F));
    REQUIRE(mem.read8(0x20100) == 0xEA);
    uint16_t seg2 = mem.read16(0x20103);
    REQUIRE(seg2 != 0);
    REQUIRE(seg2 != seg1);

    std::remove(fname);
}

// ═══════════════════════════════════════════════════════════════════════════
// INT 21h AH=0x46 — Force Duplicate / Redirect Handle
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DOS: Redirect Handle (AH=0x46) duplicates file access", "[DOS][File]") {
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
        std::ofstream ofs("redirect.txt", std::ios::binary);
        ofs << "redirect-test-data";
    }

    const std::string fname = "REDIRECT.TXT";
    const uint32_t fnameAddr = 0x70000;
    for (size_t i = 0; i < fname.size(); ++i)
        mem.write8(fnameAddr + i, fname[i]);
    mem.write8(fnameAddr + fname.size(), 0);

    // Open file -> handle A
    cpu.setSegReg(cpu::DS, 0x7000);
    cpu.setSegBase(cpu::DS, 0x70000);
    cpu.setReg16(cpu::DX, 0x0000);
    cpu.setReg8(cpu::AH, 0x3D);
    cpu.setReg8(cpu::AL, 0x00);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    const uint16_t hA = cpu.getReg16(cpu::AX);
    REQUIRE(hA >= 5);

    // Redirect: make a higher handle point to the same file
    const uint16_t hB = 20; // higher than any existing handle
    cpu.setReg8(cpu::AH, 0x46);
    cpu.setReg16(cpu::BX, hA);
    cpu.setReg16(cpu::CX, hB);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));

    // Read from hA: first 7 bytes
    cpu.setSegReg(cpu::DS, 0x8000);
    cpu.setSegBase(cpu::DS, 0x80000);
    cpu.setReg16(cpu::DX, 0x0000);
    cpu.setReg8(cpu::AH, 0x3F);
    cpu.setReg16(cpu::BX, hA);
    cpu.setReg16(cpu::CX, 7);
    dos.handleInterrupt(0x21);
    REQUIRE(cpu.getReg16(cpu::AX) == 7);
    std::string partA;
    for (int i = 0; i < 7; ++i)
        partA += static_cast<char>(mem.read8(0x80000 + i));
    REQUIRE(partA == "redirec");

    // Read from hB: should continue from same file position
    cpu.setSegReg(cpu::DS, 0x9000);
    cpu.setSegBase(cpu::DS, 0x90000);
    cpu.setReg16(cpu::DX, 0x0000);
    cpu.setReg8(cpu::AH, 0x3F);
    cpu.setReg16(cpu::BX, hB);
    cpu.setReg16(cpu::CX, 15);
    dos.handleInterrupt(0x21);
    const uint16_t bytesB = cpu.getReg16(cpu::AX);
    REQUIRE(bytesB > 0);
    std::string partB;
    for (uint16_t i = 0; i < bytesB; ++i)
        partB += static_cast<char>(mem.read8(0x90000 + i));
    REQUIRE(partB == "t-test-data");

    // Invalid source handle fails
    cpu.setReg8(cpu::AH, 0x46);
    cpu.setReg16(cpu::BX, 0xFFFF);
    cpu.setReg16(cpu::CX, hA);
    dos.handleInterrupt(0x21);
    REQUIRE(cpu.getEFLAGS() & cpu::FLAG_CARRY);
    REQUIRE(cpu.getReg16(cpu::AX) == 0x06);

    // Cleanup
    for (auto h : {hA, hB}) {
        cpu.setReg8(cpu::AH, 0x3E);
        cpu.setReg16(cpu::BX, h);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    }
    std::remove("redirect.txt");
}

TEST_CASE("DOS: Redirect handle closes and replaces target", "[DOS][File]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();

    { std::ofstream("f1.txt", std::ios::binary) << "FileOne"; }
    { std::ofstream("f2.txt", std::ios::binary) << "FileTwo"; }

    auto openFile = [&](const std::string& name) -> uint16_t {
        const uint32_t addr = 0x70000;
        for (size_t i = 0; i < name.size(); ++i)
            mem.write8(addr + i, name[i]);
        mem.write8(addr + name.size(), 0);
        cpu.setSegReg(cpu::DS, 0x7000);
        cpu.setSegBase(cpu::DS, 0x70000);
        cpu.setReg16(cpu::DX, 0x0000);
        cpu.setReg8(cpu::AH, 0x3D);
        cpu.setReg8(cpu::AL, 0x00);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
        return cpu.getReg16(cpu::AX);
    };

    auto readAll = [&](uint16_t h) -> std::string {
        cpu.setSegReg(cpu::DS, 0x8000);
        cpu.setSegBase(cpu::DS, 0x80000);
        cpu.setReg16(cpu::DX, 0x0000);
        cpu.setReg8(cpu::AH, 0x3F);
        cpu.setReg16(cpu::BX, h);
        cpu.setReg16(cpu::CX, 20);
        dos.handleInterrupt(0x21);
        std::string s;
        for (uint16_t i = 0; i < cpu.getReg16(cpu::AX); ++i)
            s += static_cast<char>(mem.read8(0x80000 + i));
        return s;
    };

    uint16_t h1 = openFile("F1.TXT");
    uint16_t h2 = openFile("F2.TXT");

    // Before redirect: h1 reads "FileOne", h2 reads "FileTwo"
    REQUIRE(readAll(h1) == "FileOne");
    REQUIRE(readAll(h2) == "FileTwo");

    // Reopen f1 fresh
    uint16_t h1fresh = openFile("F1.TXT");

    // Redirect h2 -> h1fresh: h2 now points to f1
    cpu.setReg8(cpu::AH, 0x46);
    cpu.setReg16(cpu::BX, h1fresh);
    cpu.setReg16(cpu::CX, h2);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));

    // h2 now reads "FileOne" instead of "FileTwo"
    REQUIRE(readAll(h2) == "FileOne");

    for (auto h : {h1, h2, h1fresh}) {
        cpu.setReg8(cpu::AH, 0x3E);
        cpu.setReg16(cpu::BX, h);
        dos.handleInterrupt(0x21);
    }
    std::remove("f1.txt");
    std::remove("f2.txt");
}

// ═══════════════════════════════════════════════════════════════════════════
// INT 21h AH=0x29 — Parse Filename into FCB
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DOS: Parse Filename (AH=0x29) into FCB", "[DOS][FCB]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    bios.initialize();
    dos.initialize();

    const uint32_t srcAddr = 0x70000;
    const uint32_t fcbAddr = 0x80000;

    auto testParse = [&](const std::string& src, uint8_t flags,
                         uint8_t expectedDrive, const std::string& expectedName,
                         const std::string& expectedExt, bool expectWild) {
        for (size_t i = 0; i < src.size(); ++i)
            mem.write8(srcAddr + i, static_cast<uint8_t>(src[i]));
        mem.write8(srcAddr + src.size(), 0);
        for (int i = 0; i < 36; ++i)
            mem.write8(fcbAddr + i, 0xFF);

        cpu.setSegReg(cpu::DS, 0x7000);
        cpu.setSegBase(cpu::DS, 0x70000);
        cpu.setReg16(cpu::SI, 0x0000);
        cpu.setSegReg(cpu::ES, 0x8000);
        cpu.setSegBase(cpu::ES, 0x80000);
        cpu.setReg16(cpu::DI, 0x0000);
        cpu.setReg8(cpu::AH, 0x29);
        cpu.setReg8(cpu::AL, flags);
        dos.handleInterrupt(0x21);

        REQUIRE(cpu.getReg8(cpu::AL) == (expectWild ? 0xFF : 0x00));
        REQUIRE(mem.read8(fcbAddr + 0x00) == expectedDrive);
        for (size_t i = 0; i < 8; ++i)
            REQUIRE(mem.read8(fcbAddr + 1 + i) ==
                    static_cast<uint8_t>(expectedName[i]));
        for (size_t i = 0; i < 3; ++i)
            REQUIRE(mem.read8(fcbAddr + 9 + i) ==
                    static_cast<uint8_t>(expectedExt[i]));
        REQUIRE(mem.read16(fcbAddr + 0x0C) == 0x0000);
        REQUIRE(mem.read8(fcbAddr + 0x20) == 0x00);
    };

    SECTION("drive + name + extension") {
        testParse("C:MYFILE .TXT", 0x01, 0x03, "MYFILE  ", "TXT", false);
    }

    SECTION("no drive letter") {
        testParse("NOEXT.TXT", 0x01, 0x00, "NOEXT   ", "TXT", false);
    }

    SECTION("'?' wildcard in name") {
        testParse("A:TEST??.DAT", 0x01, 0x01, "TEST??  ", "DAT", true);
    }

    SECTION("'*' wildcard fills name with '?'") {
        testParse("B:STAR*.DAT", 0x01, 0x02, "STAR????", "DAT", true);
    }

    SECTION("'*' wildcard in extension") {
        testParse("C:NOEXT.*", 0x01, 0x03, "NOEXT   ", "???", true);
    }

    SECTION("flag bit 1: don't set drive if absent") {
        testParse("TEST.TXT", 0x03, 0xFF, "TEST    ", "TXT", false);
    }

    SECTION("flag bit 2: don't modify filename") {
        testParse("DATA.TXT", 0x05, 0x00, "\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF", "TXT", false);
    }

    SECTION("flag bit 3: don't modify extension") {
        testParse("DATA.TXT", 0x09, 0x00, "DATA    ", "\xFF\xFF\xFF", false);
    }
}

