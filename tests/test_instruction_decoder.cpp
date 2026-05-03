#include "test_framework.hpp"
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

TEST_CASE("CPU Instruction Execution", "[Decoder]") {
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

    // Setup base execution environment
    cpu.setSegReg(cpu::SegRegIndex::CS, 0x0000);
    cpu.setSegReg(cpu::SegRegIndex::DS, 0x0000);
    cpu.setSegReg(cpu::SegRegIndex::SS, 0x0000);
    cpu.setEIP(0x100);

    SECTION("decodeModRM boundary conditions") {
        auto check_modrm = [&](uint8_t byte, uint8_t exp_mod, uint8_t exp_reg, uint8_t exp_rm) {
            cpu::ModRM modrm = decoder.decodeModRM(byte);
            REQUIRE(modrm.mod == exp_mod);
            REQUIRE(modrm.reg == exp_reg);
            REQUIRE(modrm.rm == exp_rm);
        };

        check_modrm(0x00, 0, 0, 0);
        check_modrm(0xFF, 3, 7, 7);
        check_modrm(0xC0, 3, 0, 0); // 11000000 -> mod=3, reg=0, rm=0
        check_modrm(0x38, 0, 7, 0); // 00111000 -> mod=0, reg=7, rm=0
        check_modrm(0x07, 0, 0, 7); // 00000111 -> mod=0, reg=0, rm=7
        check_modrm(0x75, 1, 6, 5); // 01110101 -> mod=1, reg=6, rm=5
        check_modrm(0xAA, 2, 5, 2); // 10101010 -> mod=2, reg=5, rm=2
    }

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

    SECTION("String Operations: REP MOVSB") {
        cpu.setReg16(cpu::Reg16Index::SI, 0x1000);
        cpu.setReg16(cpu::Reg16Index::DI, 0x2000);
        cpu.setReg16(cpu::Reg16Index::CX, 3);
        cpu.setSegReg(cpu::SegRegIndex::DS, 0x0000);
        cpu.setSegReg(cpu::SegRegIndex::ES, 0x0000);
        mem.write8(0x1000, 0x11);
        mem.write8(0x1001, 0x22);
        mem.write8(0x1002, 0x33);

        // REP (0xF3) MOVSB (0xA4)
        mem.write8(0x100, 0xF3);
        mem.write8(0x101, 0xA4);
        decoder.step();

        REQUIRE(mem.read8(0x2000) == 0x11);
        REQUIRE(mem.read8(0x2001) == 0x22);
        REQUIRE(mem.read8(0x2002) == 0x33);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::CX) == 0);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SI) == 0x1003);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::DI) == 0x2003);
    }

    SECTION("String Operations: REP MOVSW") {
        cpu.setReg16(cpu::Reg16Index::SI, 0x1000);
        cpu.setReg16(cpu::Reg16Index::DI, 0x2000);
        cpu.setReg16(cpu::Reg16Index::CX, 3);
        cpu.setSegReg(cpu::SegRegIndex::DS, 0x0000);
        cpu.setSegReg(cpu::SegRegIndex::ES, 0x0000);
        mem.write16(0x1000, 0x1122);
        mem.write16(0x1002, 0x3344);
        mem.write16(0x1004, 0x5566);

        // REP (0xF3) MOVSW (0xA5)
        mem.write8(0x100, 0xF3);
        mem.write8(0x101, 0xA5);
        decoder.step();

        REQUIRE(mem.read16(0x2000) == 0x1122);
        REQUIRE(mem.read16(0x2002) == 0x3344);
        REQUIRE(mem.read16(0x2004) == 0x5566);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::CX) == 0);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SI) == 0x1006);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::DI) == 0x2006);
    }

    SECTION("String Operations: REP MOVSD") {
        cpu.setReg32(cpu::Reg16Index::SI, 0x00001000);
        cpu.setReg32(cpu::Reg16Index::DI, 0x00002000);
        cpu.setReg16(cpu::Reg16Index::CX, 2);
        cpu.setSegReg(cpu::SegRegIndex::DS, 0x0000);
        cpu.setSegReg(cpu::SegRegIndex::ES, 0x0000);
        mem.write32(0x1000, 0x11223344);
        mem.write32(0x1004, 0x55667788);

        // REP (0xF3) operand-size override (0x66) MOVSD (0xA5)
        mem.write8(0x100, 0xF3);
        mem.write8(0x101, 0x66);
        mem.write8(0x102, 0xA5);
        decoder.step();

        REQUIRE(mem.read32(0x2000) == 0x11223344);
        REQUIRE(mem.read32(0x2004) == 0x55667788);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::CX) == 0);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SI) == 0x1008);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::DI) == 0x2008);
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

    SECTION("String Operations: REP MOVSB with Direction Flag") {
        cpu.setReg16(cpu::Reg16Index::SI, 0x1002);
        cpu.setReg16(cpu::Reg16Index::DI, 0x2002);
        cpu.setReg16(cpu::Reg16Index::CX, 3);
        cpu.setSegReg(cpu::SegRegIndex::DS, 0x0000);
        cpu.setSegReg(cpu::SegRegIndex::ES, 0x0000);
        cpu.setEFLAGS(cpu.getEFLAGS() | cpu::FLAG_DIRECTION);
        mem.write8(0x1000, 0x11);
        mem.write8(0x1001, 0x22);
        mem.write8(0x1002, 0x33);

        // REP (0xF3) MOVSB (0xA4)
        mem.write8(0x100, 0xF3);
        mem.write8(0x101, 0xA4);
        decoder.step();

        REQUIRE(mem.read8(0x2000) == 0x11);
        REQUIRE(mem.read8(0x2001) == 0x22);
        REQUIRE(mem.read8(0x2002) == 0x33);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::CX) == 0);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SI) == 0x0FFF);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::DI) == 0x1FFF);
    }

    SECTION("String Operations: REP MOVSW with Direction Flag") {
        cpu.setReg16(cpu::Reg16Index::SI, 0x1004);
        cpu.setReg16(cpu::Reg16Index::DI, 0x2004);
        cpu.setReg16(cpu::Reg16Index::CX, 3);
        cpu.setSegReg(cpu::SegRegIndex::DS, 0x0000);
        cpu.setSegReg(cpu::SegRegIndex::ES, 0x0000);
        cpu.setEFLAGS(cpu.getEFLAGS() | cpu::FLAG_DIRECTION);
        mem.write16(0x1000, 0x1122);
        mem.write16(0x1002, 0x3344);
        mem.write16(0x1004, 0x5566);

        // REP (0xF3) MOVSW (0xA5)
        mem.write8(0x100, 0xF3);
        mem.write8(0x101, 0xA5);
        decoder.step();

        REQUIRE(mem.read16(0x2000) == 0x1122);
        REQUIRE(mem.read16(0x2002) == 0x3344);
        REQUIRE(mem.read16(0x2004) == 0x5566);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::CX) == 0);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SI) == 0x0FFE);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::DI) == 0x1FFE);
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

    SECTION("SHR word [BP+disp8], imm8 uses correct displacement and count (0xC1)") {
        // Regression test: opcodes 0xC0/0xC1 with memory operands must
        // resolve the effective address (consuming the displacement byte)
        // before fetching the immediate count byte.
        //
        // Instruction: SHR word [BP+0x0A], 3
        // Encoding:    C1 6E 0A 03
        //   C1    = shift/rotate r/m16, imm8
        //   6E    = ModRM: mod=01 (disp8), reg=5 (SHR), rm=6 (BP)
        //   0A    = displacement (+10)
        //   03    = immediate count (3)
        //
        // SS:BP+10 should contain the value to shift.
        // Without the fix, displacement 0x0A was consumed as count and
        // 0x03 as displacement, corrupting a wrong memory location.

        cpu.setSegReg(cpu::SegRegIndex::SS, 0x0800);
        cpu.setReg16(cpu::Reg16Index::BP, 0x0100);
        cpu.setReg16(cpu::Reg16Index::SP, 0x0200);

        // Target: SS:BP+0x0A = 0x8000 + 0x0100 + 0x0A = 0x810A
        uint32_t targetAddr = 0x8000 + 0x0100 + 0x0A; // 0x810A
        mem.write16(targetAddr, 0x0080); // value to shift: 0x0080 >> 3 = 0x0010

        // Also write a sentinel at the wrong address (SS:BP+0x03 = 0x8103)
        uint32_t wrongAddr = 0x8000 + 0x0100 + 0x03; // 0x8103
        mem.write16(wrongAddr, 0xBEEF);

        // Write instruction: C1 6E 0A 03
        mem.write8(0x100, 0xC1);
        mem.write8(0x101, 0x6E); // mod=01, reg=5(SHR), rm=6(BP)
        mem.write8(0x102, 0x0A); // disp8 = +10
        mem.write8(0x103, 0x03); // imm8 = 3
        decoder.step();

        REQUIRE(cpu.getEIP() == 0x104);
        REQUIRE(mem.read16(targetAddr) == 0x0010); // 0x0080 >> 3 = 0x0010
        REQUIRE(mem.read16(wrongAddr) == 0xBEEF);  // sentinel untouched
    }

    SECTION("SHR byte [BP+disp8], imm8 uses correct displacement and count (0xC0)") {
        // Same regression test for 8-bit variant (opcode 0xC0).
        // Instruction: SHR byte [BP+0x06], 2
        // Encoding:    C0 6E 06 02

        cpu.setSegReg(cpu::SegRegIndex::SS, 0x0800);
        cpu.setReg16(cpu::Reg16Index::BP, 0x0100);
        cpu.setReg16(cpu::Reg16Index::SP, 0x0200);

        uint32_t targetAddr = 0x8000 + 0x0100 + 0x06; // 0x8106
        mem.write8(targetAddr, 0x40); // 0x40 >> 2 = 0x10

        uint32_t wrongAddr = 0x8000 + 0x0100 + 0x02; // 0x8102
        mem.write8(wrongAddr, 0xAA);

        mem.write8(0x100, 0xC0);
        mem.write8(0x101, 0x6E); // mod=01, reg=5(SHR), rm=6(BP)
        mem.write8(0x102, 0x06); // disp8 = +6
        mem.write8(0x103, 0x02); // imm8 = 2
        decoder.step();
        REQUIRE(cpu.getEIP() == 0x104);
        REQUIRE(mem.read8(targetAddr) == 0x10); // 0x40 >> 2 = 0x10
        REQUIRE(mem.read8(wrongAddr) == 0xAA);  // sentinel untouched
    }

    SECTION("fetch32: MOV EAX, imm32") {
        // MOV EAX, 0x12345678 (0x66 0xB8 0x78 0x56 0x34 0x12)
        mem.write8(0x100, 0x66); // Operand size override
        mem.write8(0x101, 0xB8); // MOV EAX, imm32
        mem.write32(0x102, 0x12345678);
        decoder.step();
        REQUIRE(cpu.getReg32(cpu::Reg16Index::AX) == 0x12345678);
        REQUIRE(cpu.getEIP() == 0x106);
    }

    SECTION("getEffectiveAddress32 & decodeModRM: MOV [EAX], EBX") {
        cpu.setReg32(cpu::Reg16Index::AX, 0x1000); // Address
        cpu.setReg32(cpu::Reg16Index::BX, 0xCAFEBABE); // Data
        // MOV [EAX], EBX (0x67 0x66 0x89 0x18)
        // 0x67: Address size override (32-bit addresses)
        // 0x66: Operand size override (32-bit operands)
        // 0x89: MOV r/m32, r32
        // ModRM: mod=00 (register indirect), reg=011 (EBX), rm=000 (EAX) -> 0x18
        mem.write8(0x100, 0x67);
        mem.write8(0x101, 0x66);
        mem.write8(0x102, 0x89);
        mem.write8(0x103, 0x18);
        decoder.step();
        REQUIRE(mem.read32(0x1000) == 0xCAFEBABE);
    }

    SECTION("decodeSIB & getEffectiveAddress32: MOV [EBX + ECX*4], EAX") {
        cpu.setReg32(cpu::Reg16Index::BX, 0x2000); // Base
        cpu.setReg32(cpu::Reg16Index::CX, 0x0002); // Index
        cpu.setReg32(cpu::Reg16Index::AX, 0xDEADBEEF); // Data
        // Target: 0x2000 + 2*4 = 0x2008
        // MOV [EBX + ECX*4], EAX (0x67 0x66 0x89 0x04 0x8B)
        // ModRM: mod=00 (register indirect), reg=000 (EAX), rm=100 (SIB) -> 0x04
        // SIB: scale=10 (*4), index=001 (ECX), base=011 (EBX) -> 10001011 (0x8B)
        mem.write8(0x100, 0x67);
        mem.write8(0x101, 0x66);
        mem.write8(0x102, 0x89);
        mem.write8(0x103, 0x04);
        mem.write8(0x104, 0x8B);
        decoder.step();
        REQUIRE(mem.read32(0x2008) == 0xDEADBEEF);
    }

    SECTION("getEffectiveAddress16: MOV [BP+SI+0x10], AX") {
        cpu.setReg16(cpu::Reg16Index::BP, 0x1000);
        cpu.setReg16(cpu::Reg16Index::SI, 0x0020);
        cpu.setReg16(cpu::Reg16Index::AX, 0x1234);
        // By default BP uses SS segment, SS=0 -> base=0
        // Target: 0x1000 + 0x0020 + 0x10 = 0x1030
        // MOV [BP+SI+disp8], AX (0x89 0x42 0x10)
        // ModRM: mod=01 (disp8), reg=000 (AX), rm=010 (BP+SI) -> 01000010 (0x42)
        mem.write8(0x100, 0x89);
        mem.write8(0x101, 0x42);
        mem.write8(0x102, 0x10);
        decoder.step();
        REQUIRE(mem.read16(0x1030) == 0x1234);
    }
}
