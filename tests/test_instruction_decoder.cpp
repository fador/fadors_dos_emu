#include "test_framework.hpp"
#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "memory/MemoryBus.hpp"
#include "hw/IOBus.hpp"
#include "hw/BIOS.hpp"
#include "hw/DOS.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIT8254.hpp"

using namespace fador;

TEST_CASE("CPU Instruction Execution", "[Decoder]") {
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

    // Setup base execution environment
    cpu.setSegReg(cpu::SegRegIndex::CS, 0x0000);
    cpu.setSegReg(cpu::SegRegIndex::DS, 0x0000);
    cpu.setSegReg(cpu::SegRegIndex::SS, 0x0000);
    cpu.setEIP(0x100);

    SECTION("NOP Instruction") {
        mem.write8(0x100, 0x90);
        decoder.step();
        REQUIRE(cpu.getEIP() == 0x101);
    }

    SECTION("MOV imm8 to r8") {
        // MOV AL, 0x42 (0xB0 0x42)
        mem.write8(0x100, 0xB0);
        mem.write8(0x101, 0x42);
        decoder.step();
        REQUIRE(cpu.getReg8(cpu::Reg8Index::AL) == 0x42);
        REQUIRE(cpu.getEIP() == 0x102);
    }

    SECTION("MOV imm16 to r16") {
        // MOV BX, 0x1234 (0xBB 0x34 0x12)
        mem.write8(0x100, 0xBB);
        mem.write16(0x101, 0x1234);
        decoder.step();
        REQUIRE(cpu.getReg16(cpu::Reg16Index::BX) == 0x1234);
    }

    SECTION("MOV r8 to r/m8 (Register to Register)") {
        // Setup AL = 0xAA
        cpu.setReg8(cpu::Reg8Index::AL, 0xAA);
        // MOV CL, AL 
        // Opcode: 0x88 /r (ModRM: mod=11 (3), reg=000 (AL), rm=001 (CL)) => 11000001 (0xC1)
        mem.write8(0x100, 0x88);
        mem.write8(0x101, 0xC1);
        decoder.step();
        REQUIRE(cpu.getReg8(cpu::Reg8Index::CL) == 0xAA);
    }

    SECTION("PUSH / POP r16") {
        cpu.setReg16(cpu::Reg16Index::SP, 0x200);
        cpu.setReg16(cpu::Reg16Index::AX, 0xBEEF);
        
        // PUSH AX (0x50)
        mem.write8(0x100, 0x50);
        decoder.step();
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SP) == 0x1FE); // SP decremented
        REQUIRE(mem.read16(0x1FE) == 0xBEEF);                // Memory updated
        
        // Clear AX
        cpu.setReg16(cpu::Reg16Index::AX, 0x0000);
        
        // POP AX (0x58)
        mem.write8(0x101, 0x58);
        decoder.step();
        
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SP) == 0x200); // SP restored
        REQUIRE(cpu.getReg16(cpu::Reg16Index::AX) == 0xBEEF); // AX restored
    }

    SECTION("Control Flow: Near JMP rel8") {
        // JMP +0x05 (0xEB 0x05)
        mem.write8(0x100, 0xEB);
        mem.write8(0x101, 0x05);
        decoder.step();
        REQUIRE(cpu.getEIP() == 0x107); // 0x102 (EIP after read) + 5
    }

    SECTION("Control Flow: CALL near and RET") {
        cpu.setReg16(cpu::Reg16Index::SP, 0x200);
        
        // CALL +0x04 (0xE8 0x04 0x00)
        mem.write8(0x100, 0xE8);
        mem.write16(0x101, 0x0004);
        decoder.step();
        
        REQUIRE(cpu.getEIP() == 0x107); // 0x103 + 4
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SP) == 0x1FE); // Return pushed
        REQUIRE(mem.read16(0x1FE) == 0x103);
        
        // RET (0xC3) at 0x107
        mem.write8(0x107, 0xC3);
        decoder.step();
        
        REQUIRE(cpu.getEIP() == 0x103); // Retro to after CALL
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SP) == 0x200); // SP restored
    }
    SECTION("ALU: SUB r8, r/m8") {
        cpu.setReg8(cpu::Reg8Index::AL, 0x50);
        cpu.setReg8(cpu::Reg8Index::BL, 0x20);
        // SUB AL, BL (0x2A)
        // ModRM: mod=11 (3), reg=000(AL), rm=011(BL) -> 11000011 = 0xC3
        mem.write8(0x100, 0x2A);
        mem.write8(0x101, 0xC3);
        decoder.step();
        REQUIRE(cpu.getReg8(cpu::Reg8Index::AL) == 0x30);
    }

    SECTION("ALU: AND r8, r/m8") {
        cpu.setReg8(cpu::Reg8Index::AL, 0xF0);
        cpu.setReg8(cpu::Reg8Index::BL, 0x88);
        // AND AL, BL (0x22)
        // ModRM: mod=11(3), reg=000(AL), rm=011(BL) -> 0xC3
        mem.write8(0x100, 0x22);
        mem.write8(0x101, 0xC3);
        decoder.step();
        REQUIRE(cpu.getReg8(cpu::Reg8Index::AL) == 0x80);
    }

    SECTION("ALU: OR r8, r/m8") {
        cpu.setReg8(cpu::Reg8Index::AL, 0x04);
        cpu.setReg8(cpu::Reg8Index::BL, 0x02);
        // OR AL, BL (0x0A)
        mem.write8(0x100, 0x0A);
        mem.write8(0x101, 0xC3);
        decoder.step();
        REQUIRE(cpu.getReg8(cpu::Reg8Index::AL) == 0x06);
    }

    SECTION("ALU: XOR r8, r/m8") {
        cpu.setReg8(cpu::Reg8Index::AL, 0xFF);
        cpu.setReg8(cpu::Reg8Index::BL, 0x0F);
        // XOR AL, BL (0x32)
        mem.write8(0x100, 0x32);
        mem.write8(0x101, 0xC3);
        decoder.step();
        REQUIRE(cpu.getReg8(cpu::Reg8Index::AL) == 0xF0);
    }
    
    SECTION("String Operations: MOVSB") {
        cpu.setReg16(cpu::Reg16Index::SI, 0x1000);
        cpu.setReg16(cpu::Reg16Index::DI, 0x2000);
        cpu.setSegReg(cpu::SegRegIndex::DS, 0x0000);
        cpu.setSegReg(cpu::SegRegIndex::ES, 0x0000);
        mem.write8(0x1000, 0x55);
        
        // MOVSB (0xA4)
        mem.write8(0x100, 0xA4);
        decoder.step();
        
        REQUIRE(mem.read8(0x2000) == 0x55);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SI) == 0x1001);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::DI) == 0x2001);
    }

    SECTION("String Operations: REP STOSW") {
        cpu.setReg16(cpu::Reg16Index::DI, 0x3000);
        cpu.setReg16(cpu::Reg16Index::CX, 3);
        cpu.setReg16(cpu::Reg16Index::AX, 0x1234);
        cpu.setSegReg(cpu::SegRegIndex::ES, 0x0000);
        
        // REP (0xF3) STOSW (0xAB)
        mem.write8(0x100, 0xF3);
        mem.write8(0x101, 0xAB);
        decoder.step();
        
        REQUIRE(mem.read16(0x3000) == 0x1234);
        REQUIRE(mem.read16(0x3002) == 0x1234);
        REQUIRE(mem.read16(0x3004) == 0x1234);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::CX) == 0);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::DI) == 0x3006);
    }

    SECTION("String Operations: MOVSB with Direction Flag") {
        cpu.setReg16(cpu::Reg16Index::SI, 0x1000);
        cpu.setReg16(cpu::Reg16Index::DI, 0x2000);
        cpu.setEFLAGS(cpu.getEFLAGS() | cpu::FLAG_DIRECTION);
        mem.write8(0x1000, 0xAA);
        
        // MOVSB (0xA4)
        mem.write8(0x100, 0xA4);
        decoder.step();
        
        REQUIRE(mem.read8(0x2000) == 0xAA);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SI) == 0x0FFF);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::DI) == 0x1FFF);
    }
}
