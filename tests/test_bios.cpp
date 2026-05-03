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
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
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

    SECTION("INT 10h: AH=FA (Unknown/Stub)") {
        cpu.setSegReg(cpu::SegRegIndex::CS, 0x0000);
        cpu.setReg8(cpu::AH, 0xFA);

        mem.write8(0x100, 0xCD);
        mem.write8(0x101, 0x10);
        cpu.setEIP(0x100);

        decoder.step();

        // EIP should advance past INT instruction to show service executed
        REQUIRE(cpu.getEIP() == 0x102);
    }

    SECTION("INT 10h: AH=EF (Unknown/Stub)") {
        cpu.setSegReg(cpu::SegRegIndex::CS, 0x0000);
        cpu.setReg8(cpu::AH, 0xEF);

        mem.write8(0x100, 0xCD);
        mem.write8(0x101, 0x10);
        cpu.setEIP(0x100);

        decoder.step();
        REQUIRE(cpu.getReg8(cpu::AL) == 0x00);
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    }

    SECTION("DOS INT 21h: AH=47 (Get Current Directory)") {
        cpu.setSegReg(cpu::SegRegIndex::CS, 0x0000);
        cpu.setSegReg(cpu::DS, 0x2000);
        cpu.setReg8(cpu::AH, 0x47);
        cpu.setReg8(cpu::DL, 2); // C: drive
        cpu.setReg16(cpu::SI, 0x0100);

        // Set known current directory
        dos.setProgramDir("./");

        mem.write8(0x100, 0xCD);
        mem.write8(0x101, 0x21);
        cpu.setEIP(0x100);

        decoder.step();
        REQUIRE(mem.read8((0x2000 << 4) + 0x0100) != 0);
        REQUIRE(cpu.getReg16(cpu::AX) == 0x0100);
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
    }

    SECTION("DOS INT 21h: AH=45 (Unknown function stub)") {
        cpu.setSegReg(cpu::SegRegIndex::CS, 0x0000);
        cpu.setReg8(cpu::AH, 0x45);

        mem.write8(0x100, 0xCD);
        mem.write8(0x101, 0x21);
        cpu.setEIP(0x100);

        decoder.step();
        REQUIRE(cpu.getReg16(cpu::AX) == 0x0000);
        REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));
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

TEST_CASE("BIOS EMS services allocate, map, remap, and release", "[BIOS][EMS]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::DOS dos(cpu, mem);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    bios.initialize();
    dos.initialize();

    const uint32_t pageFrameBase =
        static_cast<uint32_t>(hw::BIOS::EMS_PAGE_FRAME_SEGMENT) << 4;

    REQUIRE(mem.read32(0xF0000) == 0xFFFFFFFFu);
    REQUIRE(mem.read16(0xF0004) == 0xC000);
    REQUIRE(mem.read16(0xF0006) == hw::BIOS::EMS_PRIVATE_API_OFFSET);
    REQUIRE(mem.read16(0xF0008) == hw::BIOS::EMS_PRIVATE_API_OFFSET);
    std::string emsDeviceName;
    for (uint32_t index = 0; index < 8; ++index)
        emsDeviceName.push_back(static_cast<char>(mem.read8(0xF000A + index)));
    REQUIRE(emsDeviceName == "EMMXXXX0");
    REQUIRE(mem.read16(0xF0012) == hw::BIOS::EMS_PRIVATE_API_OFFSET);
    REQUIRE(mem.read8(0xF0014) == 'C');

    cpu.setReg8(cpu::AH, 0x46);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    REQUIRE(cpu.getReg8(cpu::AL) == 0x40);

    cpu.setReg8(cpu::AH, 0x41);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    REQUIRE(cpu.getReg16(cpu::BX) == hw::BIOS::EMS_PAGE_FRAME_SEGMENT);

    cpu.setSegReg(cpu::ES, 0x7000);
    cpu.setSegBase(cpu::ES, 0x70000);
    cpu.setReg16(cpu::DI, 0x0000);
    cpu.setReg8(cpu::AH, 0x58);
    cpu.setReg8(cpu::AL, 0x01);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    REQUIRE(cpu.getReg16(cpu::CX) == hw::BIOS::EMS_PHYSICAL_PAGE_COUNT);

    cpu.setReg8(cpu::AH, 0x58);
    cpu.setReg8(cpu::AL, 0x00);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    REQUIRE(cpu.getReg16(cpu::CX) == hw::BIOS::EMS_PHYSICAL_PAGE_COUNT);
    REQUIRE(mem.read16(0x70000) == 0xD000);
    REQUIRE(mem.read16(0x70002) == 0x0000);
    REQUIRE(mem.read16(0x70004) == 0xD400);
    REQUIRE(mem.read16(0x70006) == 0x0001);
    REQUIRE(mem.read16(0x70008) == 0xD800);
    REQUIRE(mem.read16(0x7000A) == 0x0002);
    REQUIRE(mem.read16(0x7000C) == 0xDC00);
    REQUIRE(mem.read16(0x7000E) == 0x0003);

    cpu.setReg8(cpu::AH, 0x43);
    cpu.setReg16(cpu::BX, 2);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    const uint16_t emsHandle = cpu.getReg16(cpu::DX);
    REQUIRE(emsHandle != 0);

    cpu.setReg8(cpu::AH, 0x44);
    cpu.setReg8(cpu::AL, 0);
    cpu.setReg16(cpu::BX, 0);
    cpu.setReg16(cpu::DX, emsHandle);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);

    mem.write8(pageFrameBase + 0, 0x11);
    mem.write8(pageFrameBase + 1, 0x22);

    cpu.setReg8(cpu::AH, 0x44);
    cpu.setReg8(cpu::AL, 0);
    cpu.setReg16(cpu::BX, 1);
    cpu.setReg16(cpu::DX, emsHandle);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    REQUIRE(mem.read8(pageFrameBase + 0) == 0x00);
    REQUIRE(mem.read8(pageFrameBase + 1) == 0x00);

    mem.write8(pageFrameBase + 0, 0x33);
    mem.write8(pageFrameBase + 1, 0x44);

    cpu.setReg8(cpu::AH, 0x44);
    cpu.setReg8(cpu::AL, 0);
    cpu.setReg16(cpu::BX, 0);
    cpu.setReg16(cpu::DX, emsHandle);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    REQUIRE(mem.read8(pageFrameBase + 0) == 0x11);
    REQUIRE(mem.read8(pageFrameBase + 1) == 0x22);

    cpu.setReg8(cpu::AH, 0x44);
    cpu.setReg8(cpu::AL, 0);
    cpu.setReg16(cpu::BX, 1);
    cpu.setReg16(cpu::DX, emsHandle);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    REQUIRE(mem.read8(pageFrameBase + 0) == 0x33);
    REQUIRE(mem.read8(pageFrameBase + 1) == 0x44);

    cpu.setSegReg(cpu::ES, 0x7100);
    cpu.setSegBase(cpu::ES, 0x71000);
    cpu.setReg16(cpu::DI, 0x0000);
    cpu.setReg8(cpu::AH, 0x4D);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    REQUIRE(cpu.getReg16(cpu::BX) == 1);
    REQUIRE(mem.read16(0x71000) == emsHandle);
    REQUIRE(mem.read16(0x71002) == 2);

    cpu.setReg8(cpu::AH, 0x47);
    cpu.setReg16(cpu::DX, emsHandle);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);

    cpu.setReg8(cpu::AH, 0x44);
    cpu.setReg8(cpu::AL, 0);
    cpu.setReg16(cpu::BX, 0);
    cpu.setReg16(cpu::DX, emsHandle);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    REQUIRE(mem.read8(pageFrameBase + 0) == 0x11);
    REQUIRE(mem.read8(pageFrameBase + 1) == 0x22);

    cpu.setReg8(cpu::AH, 0x48);
    cpu.setReg16(cpu::DX, emsHandle);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    REQUIRE(mem.read8(pageFrameBase + 0) == 0x33);
    REQUIRE(mem.read8(pageFrameBase + 1) == 0x44);

    mem.write8(pageFrameBase + 0, 0x55);
    mem.write8(pageFrameBase + 1, 0x66);

    cpu.setReg8(cpu::AH, 0x44);
    cpu.setReg8(cpu::AL, 1);
    cpu.setReg16(cpu::BX, 1);
    cpu.setReg16(cpu::DX, emsHandle);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    REQUIRE(mem.read8(pageFrameBase + hw::BIOS::EMS_PAGE_SIZE + 0) == 0x55);
    REQUIRE(mem.read8(pageFrameBase + hw::BIOS::EMS_PAGE_SIZE + 1) == 0x66);

    cpu.setReg8(cpu::AH, 0x45);
    cpu.setReg16(cpu::DX, emsHandle);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);

    cpu.setReg8(cpu::AH, 0x42);
    bios.handleInterrupt(0x67);
    REQUIRE(cpu.getReg8(cpu::AH) == 0x00);
    REQUIRE(cpu.getReg16(cpu::BX) == hw::BIOS::EMS_TOTAL_PAGES);
    REQUIRE(cpu.getReg16(cpu::DX) == hw::BIOS::EMS_TOTAL_PAGES);
}
