#include <catch2/catch_test_macros.hpp>
#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "memory/MemoryBus.hpp"
#include "hw/IOBus.hpp"
#include "hw/BIOS.hpp"
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
    hw::BIOS bios(cpu, mem, kbd, pit);
    bios.initialize();
    cpu::InstructionDecoder decoder(cpu, mem, iobus, bios);

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
}
