#include "test_framework.hpp"
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
