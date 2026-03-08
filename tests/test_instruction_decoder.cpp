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

    SECTION("SHRD r/m32, r32, imm8 sets ZF correctly for 32-bit result") {
        // SHRD ECX, EBX, 16 with ECX=0, EBX=1 → ECX=0x00010000 (non-zero)
        // This is the exact case that caused Runtime Error 200 in LIERO:
        // ZF must NOT be set because 32-bit ECX = 0x00010000, even though CX = 0
        cpu.setReg32(cpu::Reg16Index::CX, 0x00000000);
        cpu.setReg32(cpu::Reg16Index::BX, 0x00000001);
        cpu.setEFLAGS(cpu.getEFLAGS() | cpu::FLAG_ZERO); // start with ZF=1

        // 66 0F AC D9 10 = SHRD ECX, EBX, 16
        mem.write8(0x100, 0x66); // operand size prefix
        mem.write8(0x101, 0x0F);
        mem.write8(0x102, 0xAC);
        mem.write8(0x103, 0xD9); // ModRM: mod=11 reg=011(EBX) rm=001(ECX)
        mem.write8(0x104, 0x10); // imm8 = 16
        decoder.step();

        REQUIRE(cpu.getReg32(cpu::Reg16Index::CX) == 0x00010000);
        REQUIRE((cpu.getEFLAGS() & cpu::FLAG_ZERO) == 0); // ZF must be clear
    }

    SECTION("SHRD r/m16, r16, imm8 sets ZF when result is zero") {
        // SHRD CX, BX, 8 with CX=0x00FF, BX=0x0000 → CX=0x0000
        cpu.setReg16(cpu::Reg16Index::CX, 0x00FF);
        cpu.setReg16(cpu::Reg16Index::BX, 0x0000);

        // 0F AC D9 08 = SHRD CX, BX, 8
        mem.write8(0x100, 0x0F);
        mem.write8(0x101, 0xAC);
        mem.write8(0x102, 0xD9);
        mem.write8(0x103, 0x08); // imm8 = 8
        decoder.step();

        REQUIRE(cpu.getReg16(cpu::Reg16Index::CX) == 0x0000);
        REQUIRE((cpu.getEFLAGS() & cpu::FLAG_ZERO) != 0); // ZF must be set
    }

    SECTION("SHLD r/m32, r32, imm8 sets flags correctly") {
        // SHLD EAX, EDX, 16 with EAX=0x0000ABCD, EDX=0x12340000
        // Result: ((ABCD << 32) | 12340000) << 16 → bits [63:32] = ABCD1234
        cpu.setReg32(cpu::Reg16Index::AX, 0x0000ABCD);
        cpu.setReg32(cpu::Reg16Index::DX, 0x12340000);

        // 66 0F A4 D0 10 = SHLD EAX, EDX, 16
        mem.write8(0x100, 0x66);
        mem.write8(0x101, 0x0F);
        mem.write8(0x102, 0xA4);
        mem.write8(0x103, 0xD0); // ModRM: mod=11 reg=010(EDX) rm=000(EAX)
        mem.write8(0x104, 0x10); // imm8 = 16
        decoder.step();

        REQUIRE(cpu.getReg32(cpu::Reg16Index::AX) == 0xABCD1234);
        REQUIRE((cpu.getEFLAGS() & cpu::FLAG_ZERO) == 0);      // non-zero
        REQUIRE((cpu.getEFLAGS() & cpu::FLAG_SIGN) != 0);      // MSB set
    }

    SECTION("SHRD sets CF to last bit shifted out") {
        // SHRD CX, BX, 1 with CX=0x0003, BX=0x0000 → CX=0x0001, CF=1
        cpu.setReg16(cpu::Reg16Index::CX, 0x0003);
        cpu.setReg16(cpu::Reg16Index::BX, 0x0000);

        // 0F AC D9 01 = SHRD CX, BX, 1
        mem.write8(0x100, 0x0F);
        mem.write8(0x101, 0xAC);
        mem.write8(0x102, 0xD9);
        mem.write8(0x103, 0x01); // imm8 = 1
        decoder.step();

        REQUIRE(cpu.getReg16(cpu::Reg16Index::CX) == 0x0001);
        REQUIRE((cpu.getEFLAGS() & cpu::FLAG_CARRY) != 0); // CF=1 (bit 0 of 0x0003)
    }
}
