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
#include "hw/KeyboardController.hpp"
#include "hw/PIT8254.hpp"

using namespace fador;

TEST_CASE("BIOS Emulation Services", "[BIOS]") {
    utils::currentLevel = utils::LogLevel::Trace;
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::IOBus iobus;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::DOS dos(cpu, mem);
    hw::BIOS bios(cpu, mem, kbd, pit);
    bios.initialize();
    dos.initialize();
    cpu::InstructionDecoder decoder(cpu, mem, iobus, bios, dos);

    SECTION("INT 10h: AH=0Eh (Teletype)") {
        cpu.setSegReg(cpu::SegRegIndex::CS, 0x0000);
        cpu.setReg8(cpu::AH, 0x0E);
        cpu.setReg8(cpu::AL, 'X');
        
        // Ensure cursor is at 0,0
        mem.write8(0x450, 0);
        mem.write8(0x451, 0);

        // INT 10h
        mem.write8(0x100, 0xCD);
        mem.write8(0x101, 0x10);
        cpu.setEIP(0x100);
        
        decoder.step();
        
        // Verify VRAM write at 0xB800:0000
        REQUIRE(mem.read8(0xB8000) == 'X');
        REQUIRE(mem.read8(0xB8001) == 0x07); // Default attribute

        // Verify BDA cursor advance
        REQUIRE(mem.read8(0x450) == 1); // Col advanced
        REQUIRE(mem.read8(0x451) == 0); // Row same
    }

    SECTION("INT 16h: AH=01h (Keyboard Status)") {
        cpu.setSegReg(cpu::SegRegIndex::CS, 0x0000);
        cpu.setReg8(cpu::AH, 0x01);
        
        // No keys in buffer
        mem.write8(0x100, 0xCD);
        mem.write8(0x101, 0x16);
        cpu.setEIP(0x100);
        
        decoder.step();
        REQUIRE(cpu.getEFLAGS() & cpu::FLAG_ZERO); // ZF=1 means no key

        // Push a key
        kbd.pushScancode(0x1E); // 'A'
        cpu.setEIP(0x100);
        decoder.step();
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_ZERO)); // ZF=0 means key present
    }

    SECTION("INT 13h: AH=08h (Drive Parameters)") {
        cpu.setSegReg(cpu::SegRegIndex::CS, 0x1000); // Testing different CS
        cpu.setReg8(cpu::AH, 0x08);
        cpu.setReg8(cpu::DL, 0x00); // Floppy 0
        
        mem.write8(0x10100, 0xCD);
        mem.write8(0x10101, 0x13);
        cpu.setEIP(0x100);
        
        decoder.step();
        
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
        REQUIRE(cpu.getReg8(cpu::BL) == 0x04); // 1.44MB
        REQUIRE(cpu.getReg8(cpu::CL) == 18);   // 18 sectors
    }

    SECTION("INT 13h: AH=02h (Read Sectors)") {
        // Create a dummy 512 byte file
        {
            std::ofstream ofs("test_floppy.img", std::ios::binary);
            std::vector<uint8_t> dummy(512, 0xAA);
            ofs.write(reinterpret_cast<const char*>(dummy.data()), 512);
        }

        REQUIRE(bios.loadDiskImage("test_floppy.img"));

        cpu.setSegReg(cpu::SegRegIndex::CS, 0x0000);
        cpu.setSegReg(cpu::SegRegIndex::ES, 0x2000);
        cpu.setReg16(cpu::BX, 0x0000);
        cpu.setReg8(cpu::AH, 0x02); // Read
        cpu.setReg8(cpu::AL, 1);    // 1 sector
        cpu.setReg8(cpu::CH, 0);    // Cyl 0
        cpu.setReg8(cpu::CL, 1);    // Sector 1
        cpu.setReg8(cpu::DH, 0);    // Head 0
        cpu.setReg8(cpu::DL, 0);    // Drive 0

        mem.write8(0x100, 0xCD);
        mem.write8(0x101, 0x13);
        cpu.setEIP(0x100);

        decoder.step();

        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
        REQUIRE(cpu.getReg8(cpu::AH) == 0);
        REQUIRE(mem.read8(0x20000) == 0xAA);
        
        std::remove("test_floppy.img");
    }
}
