#include "test_framework.hpp"
#include <fstream>
#include <cstdio>
#include <vector>
#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "memory/MemoryBus.hpp"
#include "hw/IOBus.hpp"
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
    hw::BIOS bios(cpu, mem, kbd, pit);
    hw::DOS dos(cpu, mem);
    
    bios.initialize();
    dos.initialize();
    cpu::InstructionDecoder decoder(cpu, mem, iobus, bios, dos);
    hw::ProgramLoader loader(cpu, mem);

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
        dos.handleDOSService();
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
