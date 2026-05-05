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

#include <cmath>
#include <cstring>

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

    auto readF64 = [&](uint32_t address) {
        uint64_t bits = static_cast<uint64_t>(mem.read32(address)) |
                        (static_cast<uint64_t>(mem.read32(address + 4)) << 32);
        double value = 0.0;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    };

    auto writeF64 = [&](uint32_t address, double value) {
        uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        mem.write32(address, static_cast<uint32_t>(bits & 0xFFFFFFFFull));
        mem.write32(address + 4, static_cast<uint32_t>(bits >> 32));
    };

    auto writeDescriptor = [&](uint32_t address,
                               uint32_t base,
                               uint32_t limit,
                               uint8_t access,
                               uint8_t flags) {
        uint32_t low = (limit & 0xFFFFu) | ((base & 0xFFFFu) << 16);
        uint32_t high = ((base >> 16) & 0xFFu) |
                        (static_cast<uint32_t>(access) << 8) |
                        (((limit >> 16) & 0x0Fu) << 16) |
                        ((static_cast<uint32_t>(flags) & 0x0Fu) << 20) |
                        (base & 0xFF000000u);
        mem.write32(address, low);
        mem.write32(address + 4, high);
    };

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

    SECTION("MOV EAX, CR0") {
        cpu.setCR(0, 0x12345679);

        mem.write8(0x100, 0x0F);
        mem.write8(0x101, 0x20);
        mem.write8(0x102, 0xC0);

        decoder.step();

        REQUIRE(cpu.getReg32(cpu::RegIndex::EAX) == 0x12345679);
        REQUIRE(cpu.getEIP() == 0x103);
    }

    SECTION("x87: FWAIT dispatches INT 75 on unmasked pending exception") {
        cpu.setReg16(cpu::Reg16Index::SP, 0x0200);
        cpu.setSegReg(cpu::SegRegIndex::CS, 0x0000);
        cpu.setSegReg(cpu::SegRegIndex::SS, 0x0000);
        cpu.setEIP(0x0100);

        mem.write16(0x75 * 4, 0x3456);
        mem.write16(0x75 * 4 + 2, 0x2222);

        cpu.setFPUControlWord(static_cast<uint16_t>(cpu::FPU_CONTROL_DEFAULT &
                                                    ~cpu::FPU_STATUS_ZE));
        cpu.setFPUStatusBits(cpu::FPU_STATUS_ZE);

        mem.write8(0x0100, 0x9B);
        decoder.step();

        REQUIRE((cpu.getFPUStatusWord() & cpu::FPU_STATUS_ES) != 0);
        REQUIRE(cpu.getSegReg(cpu::SegRegIndex::CS) == 0x2222);
        REQUIRE(cpu.getEIP() == 0x3456);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SP) == 0x01FA);
        REQUIRE(mem.read16(0x01FA) == 0x0101);
        REQUIRE(mem.read16(0x01FC) == 0x0000);
    }

    SECTION("x87: FWAIT ignores masked pending exceptions") {
        cpu.setReg16(cpu::Reg16Index::SP, 0x0200);
        cpu.setSegReg(cpu::SegRegIndex::CS, 0x0000);
        cpu.setSegReg(cpu::SegRegIndex::SS, 0x0000);
        cpu.setEIP(0x0100);

        mem.write16(0x75 * 4, 0x3456);
        mem.write16(0x75 * 4 + 2, 0x2222);

        cpu.setFPUControlWord(cpu::FPU_CONTROL_DEFAULT);
        cpu.setFPUStatusBits(cpu::FPU_STATUS_ZE);

        mem.write8(0x0100, 0x9B);
        decoder.step();

        REQUIRE((cpu.getFPUStatusWord() & cpu::FPU_STATUS_ES) == 0);
        REQUIRE(cpu.getSegReg(cpu::SegRegIndex::CS) == 0x0000);
        REQUIRE(cpu.getEIP() == 0x0101);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SP) == 0x0200);
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

    SECTION("String Operations: REP MOVSB with forward overlap") {
        cpu.setReg16(cpu::Reg16Index::SI, 0x1000);
        cpu.setReg16(cpu::Reg16Index::DI, 0x1001);
        cpu.setReg16(cpu::Reg16Index::CX, 4);
        cpu.setSegReg(cpu::SegRegIndex::DS, 0x0000);
        cpu.setSegReg(cpu::SegRegIndex::ES, 0x0000);
        mem.write8(0x1000, 0x11);
        mem.write8(0x1001, 0x22);
        mem.write8(0x1002, 0x33);
        mem.write8(0x1003, 0x44);
        mem.write8(0x1004, 0x55);

        // REP (0xF3) MOVSB (0xA4)
        mem.write8(0x100, 0xF3);
        mem.write8(0x101, 0xA4);
        decoder.step();

        REQUIRE(mem.read8(0x1000) == 0x11);
        REQUIRE(mem.read8(0x1001) == 0x11);
        REQUIRE(mem.read8(0x1002) == 0x11);
        REQUIRE(mem.read8(0x1003) == 0x11);
        REQUIRE(mem.read8(0x1004) == 0x11);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::CX) == 0);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SI) == 0x1004);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::DI) == 0x1005);
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

    SECTION("0F-prefix: BSWAP EAX") {
        cpu.setReg32(cpu::Reg32Index::EAX, 0x11223344);
        mem.write8(0x100, 0x0F);
        mem.write8(0x101, 0xC8);

        decoder.step();

        REQUIRE(cpu.getReg32(cpu::Reg32Index::EAX) == 0x44332211);
        REQUIRE(cpu.getEIP() == 0x102);
    }

    SECTION("x87: FLD1, FADDP, FSTP m64") {
        auto readDouble = [&](uint32_t address) {
            uint64_t bits = static_cast<uint64_t>(mem.read32(address)) |
                            (static_cast<uint64_t>(mem.read32(address + 4)) << 32);
            double value = 0.0;
            std::memcpy(&value, &bits, sizeof(value));
            return value;
        };

        mem.write8(0x100, 0xD9); mem.write8(0x101, 0xE8); // FLD1
        mem.write8(0x102, 0xD9); mem.write8(0x103, 0xE8); // FLD1
        mem.write8(0x104, 0xDE); mem.write8(0x105, 0xC1); // FADDP ST1, ST0
        mem.write8(0x106, 0xDD); mem.write8(0x107, 0x1E); // FSTP qword ptr [2000h]
        mem.write16(0x108, 0x2000);

        decoder.step();
        decoder.step();
        decoder.step();
        decoder.step();

        REQUIRE(std::fabs(readDouble(0x2000) - 2.0) < 1e-12);
        REQUIRE(cpu.isFPURegisterEmpty(0));
    }

    SECTION("x87: FLDCW, FLD m32, FRNDINT, FSTP m32") {
        auto writeFloat = [&](uint32_t address, float value) {
            uint32_t bits = 0;
            std::memcpy(&bits, &value, sizeof(bits));
            mem.write32(address, bits);
        };
        auto readFloat = [&](uint32_t address) {
            uint32_t bits = mem.read32(address);
            float value = 0.0f;
            std::memcpy(&value, &bits, sizeof(value));
            return value;
        };

        writeFloat(0x2300, 1.75f);
        mem.write16(0x2310, 0x0F7F);

        mem.write8(0x100, 0xD9); mem.write8(0x101, 0x2E); // FLDCW [2310h]
        mem.write16(0x102, 0x2310);
        mem.write8(0x104, 0xD9); mem.write8(0x105, 0x06); // FLD dword ptr [2300h]
        mem.write16(0x106, 0x2300);
        mem.write8(0x108, 0xD9); mem.write8(0x109, 0xFC); // FRNDINT
        mem.write8(0x10A, 0xD9); mem.write8(0x10B, 0x1E); // FSTP dword ptr [2304h]
        mem.write16(0x10C, 0x2304);

        decoder.step();
        decoder.step();
        decoder.step();
        decoder.step();

        REQUIRE(std::fabs(readFloat(0x2304) - 1.0f) < 1e-6f);
    }

    SECTION("x87: FILD and FISTP m16") {
        mem.write16(0x2400, 7);

        mem.write8(0x100, 0xDF); mem.write8(0x101, 0x06); // FILD word ptr [2400h]
        mem.write16(0x102, 0x2400);
        mem.write8(0x104, 0xDF); mem.write8(0x105, 0x1E); // FISTP word ptr [2402h]
        mem.write16(0x106, 0x2402);

        decoder.step();
        decoder.step();

        REQUIRE(mem.read16(0x2402) == 7);
        REQUIRE(cpu.isFPURegisterEmpty(0));
    }

    SECTION("x87: FXAM classifies negative zero") {
        cpu.pushFPU(-0.0);

        mem.write8(0x100, 0xD9); mem.write8(0x101, 0xE5); // FXAM

        decoder.step();

        uint16_t status = cpu.getFPUStatusWord();
        REQUIRE((status & cpu::FPU_STATUS_C1) != 0);
        REQUIRE((status & cpu::FPU_STATUS_C3) != 0);
        REQUIRE((status & cpu::FPU_STATUS_C2) == 0);
        REQUIRE((status & cpu::FPU_STATUS_C0) == 0);
    }

    SECTION("x87: FBSTP stores packed BCD and pops") {
        cpu.pushFPU(123456789012.0);

        mem.write8(0x100, 0xDF); mem.write8(0x101, 0x36); // FBSTP tbyte ptr [2500h]
        mem.write16(0x102, 0x2500);

        decoder.step();

        REQUIRE(mem.read8(0x2500) == 0x12);
        REQUIRE(mem.read8(0x2501) == 0x90);
        REQUIRE(mem.read8(0x2502) == 0x78);
        REQUIRE(mem.read8(0x2503) == 0x56);
        REQUIRE(mem.read8(0x2504) == 0x34);
        REQUIRE(mem.read8(0x2505) == 0x12);
        REQUIRE(mem.read8(0x2506) == 0x00);
        REQUIRE(mem.read8(0x2507) == 0x00);
        REQUIRE(mem.read8(0x2508) == 0x00);
        REQUIRE(mem.read8(0x2509) == 0x00);
        REQUIRE(cpu.isFPURegisterEmpty(0));
    }

    SECTION("x87: FNINIT resets stack and control word") {
        mem.write8(0x100, 0xD9); mem.write8(0x101, 0xE8); // FLD1
        mem.write8(0x102, 0xDB); mem.write8(0x103, 0xE3); // FNINIT

        decoder.step();
        REQUIRE(!cpu.isFPURegisterEmpty(0));

        decoder.step();
        REQUIRE(cpu.isFPURegisterEmpty(0));
        REQUIRE(cpu.getFPUControlWord() == cpu::FPU_CONTROL_DEFAULT);
        REQUIRE(cpu.getFPUStatusWord() == 0);
    }

    SECTION("x87: real FNSTENV stores provenance from prior memory op") {
        float source = 1.25f;
        uint32_t bits = 0;
        std::memcpy(&bits, &source, sizeof(bits));
        mem.write32(0x2300, bits);

        mem.write8(0x100, 0xD9); mem.write8(0x101, 0x06); // FLD dword ptr [2300h]
        mem.write16(0x102, 0x2300);
        mem.write8(0x104, 0x66);
        mem.write8(0x105, 0xD9); mem.write8(0x106, 0x36); // FNSTENV [3000h]
        mem.write16(0x107, 0x3000);

        decoder.step();

        REQUIRE(cpu.getFPUInstructionPointer() == 0x100);
        REQUIRE(cpu.getFPUInstructionSelector() == 0x0000);
        REQUIRE(cpu.getFPULastOpcode() == 0x0106);
        REQUIRE(cpu.getFPUDataPointer() == 0x2300);
        REQUIRE(cpu.getFPUDataSelector() == 0x0000);

        decoder.step();

        REQUIRE(mem.read32(0x300C) == 0x00000100);
        REQUIRE(mem.read32(0x3010) == 0x01060000u);
        REQUIRE(mem.read32(0x3014) == 0x00002300);
        REQUIRE(mem.read32(0x3018) == 0x00000000);
    }

    SECTION("x87: real 32-bit FNSTENV stores offsets with real-mode selectors") {
        cpu.loadSegment(cpu::SegRegIndex::CS, 0x1234);
        cpu.loadSegment(cpu::SegRegIndex::DS, 0x4321);
        cpu.setEIP(0x100);

        constexpr uint32_t codeAddress = 0x12340 + 0x100;
        constexpr uint32_t sourceAddress = 0x43210 + 0x2300;
        constexpr uint32_t saveAddress = 0x43210 + 0x3000;

        float source = 1.25f;
        uint32_t bits = 0;
        std::memcpy(&bits, &source, sizeof(bits));
        mem.write32(sourceAddress, bits);

        mem.write8(codeAddress + 0, 0xD9); mem.write8(codeAddress + 1, 0x06); // FLD dword ptr [2300h]
        mem.write16(codeAddress + 2, 0x2300);
        mem.write8(codeAddress + 4, 0x66);
        mem.write8(codeAddress + 5, 0xD9); mem.write8(codeAddress + 6, 0x36); // FNSTENV [3000h]
        mem.write16(codeAddress + 7, 0x3000);

        decoder.step();

        REQUIRE(cpu.getFPUInstructionPointer() == 0x100);
        REQUIRE(cpu.getFPUInstructionSelector() == 0x1234);
        REQUIRE(cpu.getFPULastOpcode() == 0x0106);
        REQUIRE(cpu.getFPUDataPointer() == 0x2300);
        REQUIRE(cpu.getFPUDataSelector() == 0x4321);

        decoder.step();

        REQUIRE(mem.read32(saveAddress + 12) == 0x00000100);
        REQUIRE(mem.read32(saveAddress + 16) == 0x01061234u);
        REQUIRE(mem.read32(saveAddress + 20) == 0x00002300);
        REQUIRE(mem.read32(saveAddress + 24) == 0x00004321);
    }

    SECTION("x87: real 32-bit FNSAVE stores offsets with real-mode selectors") {
        cpu.loadSegment(cpu::SegRegIndex::CS, 0x1357);
        cpu.loadSegment(cpu::SegRegIndex::DS, 0x2468);
        cpu.setEIP(0x100);

        constexpr uint32_t codeAddress = 0x13570 + 0x100;
        constexpr uint32_t sourceAddress = 0x24680 + 0x2300;
        constexpr uint32_t saveAddress = 0x24680 + 0x3100;

        float source = 1.25f;
        uint32_t bits = 0;
        std::memcpy(&bits, &source, sizeof(bits));
        mem.write32(sourceAddress, bits);

        mem.write8(codeAddress + 0, 0xD9); mem.write8(codeAddress + 1, 0x06); // FLD dword ptr [2300h]
        mem.write16(codeAddress + 2, 0x2300);
        mem.write8(codeAddress + 4, 0x66);
        mem.write8(codeAddress + 5, 0xDD); mem.write8(codeAddress + 6, 0x36); // FNSAVE [3100h]
        mem.write16(codeAddress + 7, 0x3100);

        decoder.step();
        decoder.step();

        REQUIRE(mem.read32(saveAddress + 12) == 0x00000100);
        REQUIRE(mem.read32(saveAddress + 16) == 0x01061357u);
        REQUIRE(mem.read32(saveAddress + 20) == 0x00002300);
        REQUIRE(mem.read32(saveAddress + 24) == 0x00002468);
        REQUIRE(cpu.getFPUControlWord() == cpu::FPU_CONTROL_DEFAULT);
        REQUIRE(cpu.getFPUStatusWord() == 0);
        REQUIRE(cpu.isFPURegisterEmpty(0));
    }

    SECTION("x87: 16-bit protected-mode FNSTENV and FLDENV use selector words without opcode") {
        constexpr uint32_t gdtBase = 0x1800;
        constexpr uint32_t codeBase = 0x22000;
        constexpr uint32_t dataBase = 0x34000;
        constexpr uint16_t codeSel = 0x0008;
        constexpr uint16_t dataSel = 0x0010;

        writeDescriptor(gdtBase + 0x08, codeBase, 0xFFFF, 0x9A, 0x0);
        writeDescriptor(gdtBase + 0x10, dataBase, 0xFFFF, 0x92, 0x0);

        cpu.setGDTR({0x17, gdtBase});
        cpu.setCR(0, cpu.getCR(0) | 0x1);
        cpu.setEFLAGS(cpu.getEFLAGS() & ~0x00020000u);
        cpu.loadSegment(cpu::SegRegIndex::CS, codeSel);
        cpu.loadSegment(cpu::SegRegIndex::DS, dataSel);
        cpu.setEIP(0x100);

        float source = 1.25f;
        uint32_t bits = 0;
        std::memcpy(&bits, &source, sizeof(bits));
        mem.write32(dataBase + 0x2300, bits);

        mem.write8(codeBase + 0x100, 0xD9); mem.write8(codeBase + 0x101, 0x06); // FLD dword ptr [2300h]
        mem.write16(codeBase + 0x102, 0x2300);
        mem.write8(codeBase + 0x104, 0xD9); mem.write8(codeBase + 0x105, 0x36); // FNSTENV [3000h]
        mem.write16(codeBase + 0x106, 0x3000);
        mem.write8(codeBase + 0x108, 0xD9); mem.write8(codeBase + 0x109, 0x26); // FLDENV [3000h]
        mem.write16(codeBase + 0x10A, 0x3000);

        decoder.step();

        REQUIRE(cpu.getFPUInstructionPointer() == 0x100);
        REQUIRE(cpu.getFPUInstructionSelector() == codeSel);
        REQUIRE(cpu.getFPULastOpcode() == 0x0106);
        REQUIRE(cpu.getFPUDataPointer() == 0x2300);
        REQUIRE(cpu.getFPUDataSelector() == dataSel);

        decoder.step();

        REQUIRE(mem.read16(dataBase + 0x3000) == cpu.getFPUControlWord());
        REQUIRE(mem.read16(dataBase + 0x3006) == 0x0100);
        REQUIRE(mem.read16(dataBase + 0x3008) == codeSel);
        REQUIRE(mem.read16(dataBase + 0x300A) == 0x2300);
        REQUIRE(mem.read16(dataBase + 0x300C) == dataSel);

        cpu.setFPUControlWord(cpu::FPU_CONTROL_DEFAULT);
        cpu.setFPUStatusWord(0);
        cpu.setFPUTagWord(0xFFFF);
        cpu.setFPUInstructionPointer(0);
        cpu.setFPUInstructionSelector(0);
        cpu.setFPULastOpcode(0x07FF);
        cpu.setFPUDataPointer(0);
        cpu.setFPUDataSelector(0);

        decoder.step();

        REQUIRE(cpu.getFPUInstructionPointer() == 0x0100);
        REQUIRE(cpu.getFPUInstructionSelector() == codeSel);
        REQUIRE(cpu.getFPULastOpcode() == 0x0000);
        REQUIRE(cpu.getFPUDataPointer() == 0x2300);
        REQUIRE(cpu.getFPUDataSelector() == dataSel);
    }

    SECTION("x87: 16-bit protected-mode FNSAVE and FRSTOR clear opcode on restore") {
        constexpr uint32_t gdtBase = 0x1A00;
        constexpr uint32_t codeBase = 0x26000;
        constexpr uint32_t dataBase = 0x38000;
        constexpr uint16_t codeSel = 0x0008;
        constexpr uint16_t dataSel = 0x0010;

        writeDescriptor(gdtBase + 0x08, codeBase, 0xFFFF, 0x9A, 0x0);
        writeDescriptor(gdtBase + 0x10, dataBase, 0xFFFF, 0x92, 0x0);

        cpu.setGDTR({0x17, gdtBase});
        cpu.setCR(0, cpu.getCR(0) | 0x1);
        cpu.setEFLAGS(cpu.getEFLAGS() & ~0x00020000u);
        cpu.loadSegment(cpu::SegRegIndex::CS, codeSel);
        cpu.loadSegment(cpu::SegRegIndex::DS, dataSel);
        cpu.setEIP(0x100);

        float source = 1.25f;
        uint32_t bits = 0;
        std::memcpy(&bits, &source, sizeof(bits));
        mem.write32(dataBase + 0x2300, bits);

        mem.write8(codeBase + 0x100, 0xD9); mem.write8(codeBase + 0x101, 0x06); // FLD dword ptr [2300h]
        mem.write16(codeBase + 0x102, 0x2300);
        mem.write8(codeBase + 0x104, 0xDD); mem.write8(codeBase + 0x105, 0x36); // FNSAVE [3100h]
        mem.write16(codeBase + 0x106, 0x3100);
        mem.write8(codeBase + 0x108, 0xDD); mem.write8(codeBase + 0x109, 0x26); // FRSTOR [3100h]
        mem.write16(codeBase + 0x10A, 0x3100);

        decoder.step();
        decoder.step();

        REQUIRE(mem.read16(dataBase + 0x3106) == 0x0100);
        REQUIRE(mem.read16(dataBase + 0x3108) == codeSel);
        REQUIRE(mem.read16(dataBase + 0x310A) == 0x2300);
        REQUIRE(mem.read16(dataBase + 0x310C) == dataSel);
        REQUIRE(cpu.getFPUControlWord() == cpu::FPU_CONTROL_DEFAULT);
        REQUIRE(cpu.getFPUStatusWord() == 0);
        REQUIRE(cpu.isFPURegisterEmpty(0));

        cpu.setFPULastOpcode(0x07FF);

        decoder.step();

        REQUIRE(cpu.getFPUInstructionPointer() == 0x0100);
        REQUIRE(cpu.getFPUInstructionSelector() == codeSel);
        REQUIRE(cpu.getFPULastOpcode() == 0x0000);
        REQUIRE(cpu.getFPUDataPointer() == 0x2300);
        REQUIRE(cpu.getFPUDataSelector() == dataSel);
    }

    SECTION("x87: 32-bit protected-mode FNSTENV and FLDENV preserve selector and opcode") {
        constexpr uint32_t gdtBase = 0x1C00;
        constexpr uint32_t codeBase = 0x2A000;
        constexpr uint32_t dataBase = 0x3C000;
        constexpr uint16_t codeSel = 0x0008;
        constexpr uint16_t dataSel = 0x0010;

        writeDescriptor(gdtBase + 0x08, codeBase, 0xFFFF, 0x9A, 0x4);
        writeDescriptor(gdtBase + 0x10, dataBase, 0xFFFF, 0x92, 0x4);

        cpu.setGDTR({0x17, gdtBase});
        cpu.setCR(0, cpu.getCR(0) | 0x1);
        cpu.setEFLAGS(cpu.getEFLAGS() & ~0x00020000u);
        cpu.loadSegment(cpu::SegRegIndex::CS, codeSel);
        cpu.loadSegment(cpu::SegRegIndex::DS, dataSel);
        cpu.setEIP(0x100);

        float source = 1.25f;
        uint32_t bits = 0;
        std::memcpy(&bits, &source, sizeof(bits));
        mem.write32(dataBase + 0x2300, bits);

        mem.write8(codeBase + 0x100, 0xD9); mem.write8(codeBase + 0x101, 0x05); // FLD dword ptr [2300h]
        mem.write32(codeBase + 0x102, 0x00002300);
        mem.write8(codeBase + 0x106, 0xD9); mem.write8(codeBase + 0x107, 0x35); // FNSTENV [3000h]
        mem.write32(codeBase + 0x108, 0x00003000);
        mem.write8(codeBase + 0x10C, 0xD9); mem.write8(codeBase + 0x10D, 0x25); // FLDENV [3000h]
        mem.write32(codeBase + 0x10E, 0x00003000);

        decoder.step();

        REQUIRE(cpu.is32BitCode());
        REQUIRE(cpu.getFPUInstructionPointer() == 0x100);
        REQUIRE(cpu.getFPUInstructionSelector() == codeSel);
        REQUIRE(cpu.getFPULastOpcode() == 0x0105);
        REQUIRE(cpu.getFPUDataPointer() == 0x2300);
        REQUIRE(cpu.getFPUDataSelector() == dataSel);

        decoder.step();

        REQUIRE(mem.read32(dataBase + 0x3000) == cpu.getFPUControlWord());
        REQUIRE(mem.read32(dataBase + 0x300C) == 0x00000100);
        REQUIRE(mem.read32(dataBase + 0x3010) == 0x01050008u);
        REQUIRE(mem.read32(dataBase + 0x3014) == 0x00002300);
        REQUIRE(mem.read32(dataBase + 0x3018) == 0x00000010u);

        cpu.setFPUControlWord(cpu::FPU_CONTROL_DEFAULT);
        cpu.setFPUStatusWord(0);
        cpu.setFPUTagWord(0xFFFF);
        cpu.setFPUInstructionPointer(0);
        cpu.setFPUInstructionSelector(0);
        cpu.setFPULastOpcode(0);
        cpu.setFPUDataPointer(0);
        cpu.setFPUDataSelector(0);

        decoder.step();

        REQUIRE(cpu.getFPUInstructionPointer() == 0x0100);
        REQUIRE(cpu.getFPUInstructionSelector() == codeSel);
        REQUIRE(cpu.getFPULastOpcode() == 0x0105);
        REQUIRE(cpu.getFPUDataPointer() == 0x2300);
        REQUIRE(cpu.getFPUDataSelector() == dataSel);
    }

    SECTION("x87: 32-bit protected-mode FNSAVE and FRSTOR preserve selector and opcode") {
        constexpr uint32_t gdtBase = 0x1E00;
        constexpr uint32_t codeBase = 0x2E000;
        constexpr uint32_t dataBase = 0x40000;
        constexpr uint16_t codeSel = 0x0008;
        constexpr uint16_t dataSel = 0x0010;

        writeDescriptor(gdtBase + 0x08, codeBase, 0xFFFF, 0x9A, 0x4);
        writeDescriptor(gdtBase + 0x10, dataBase, 0xFFFF, 0x92, 0x4);

        cpu.setGDTR({0x17, gdtBase});
        cpu.setCR(0, cpu.getCR(0) | 0x1);
        cpu.setEFLAGS(cpu.getEFLAGS() & ~0x00020000u);
        cpu.loadSegment(cpu::SegRegIndex::CS, codeSel);
        cpu.loadSegment(cpu::SegRegIndex::DS, dataSel);
        cpu.setEIP(0x100);

        float source = 1.25f;
        uint32_t bits = 0;
        std::memcpy(&bits, &source, sizeof(bits));
        mem.write32(dataBase + 0x2300, bits);

        mem.write8(codeBase + 0x100, 0xD9); mem.write8(codeBase + 0x101, 0x05); // FLD dword ptr [2300h]
        mem.write32(codeBase + 0x102, 0x00002300);
        mem.write8(codeBase + 0x106, 0xDD); mem.write8(codeBase + 0x107, 0x35); // FNSAVE [3100h]
        mem.write32(codeBase + 0x108, 0x00003100);
        mem.write8(codeBase + 0x10C, 0xDD); mem.write8(codeBase + 0x10D, 0x25); // FRSTOR [3100h]
        mem.write32(codeBase + 0x10E, 0x00003100);
        mem.write8(codeBase + 0x112, 0xDD); mem.write8(codeBase + 0x113, 0x1D); // FSTP qword ptr [3300h]
        mem.write32(codeBase + 0x114, 0x00003300);

        decoder.step();
        decoder.step();

        REQUIRE(mem.read32(dataBase + 0x310C) == 0x00000100);
        REQUIRE(mem.read32(dataBase + 0x3110) == 0x01050008u);
        REQUIRE(mem.read32(dataBase + 0x3114) == 0x00002300);
        REQUIRE(mem.read32(dataBase + 0x3118) == 0x00000010u);
        REQUIRE(cpu.getFPUControlWord() == cpu::FPU_CONTROL_DEFAULT);
        REQUIRE(cpu.getFPUStatusWord() == 0);
        REQUIRE(cpu.isFPURegisterEmpty(0));

        cpu.setFPULastOpcode(0);

        decoder.step();

        REQUIRE(cpu.getFPUInstructionPointer() == 0x0100);
        REQUIRE(cpu.getFPUInstructionSelector() == codeSel);
        REQUIRE(cpu.getFPULastOpcode() == 0x0105);
        REQUIRE(cpu.getFPUDataPointer() == 0x2300);
        REQUIRE(cpu.getFPUDataSelector() == dataSel);

        decoder.step();

        REQUIRE(std::fabs(readF64(dataBase + 0x3300) - 1.25) < 1e-12);
        REQUIRE(cpu.isFPURegisterEmpty(0));
    }

    SECTION("x87: FNSTENV and FLDENV roundtrip 16-bit environment") {
        cpu.setFPUControlWord(0x0240);
        cpu.setFPUStatusWord(static_cast<uint16_t>(cpu::FPU_STATUS_C0 |
                                                   cpu::FPU_STATUS_C3 |
                                                   (5u << 11)));
        cpu.setFPUTagWord(0xA55A);
        cpu.setFPUInstructionPointer(0x1234);
        cpu.setFPUInstructionSelector(0x5678);
        cpu.setFPULastOpcode(0x0321);
        cpu.setFPUDataPointer(0x9ABC);
        cpu.setFPUDataSelector(0xDEF0);

        mem.write8(0x100, 0xD9); mem.write8(0x101, 0x36); // FNSTENV [3000h]
        mem.write16(0x102, 0x3000);
        mem.write8(0x104, 0xD9); mem.write8(0x105, 0x26); // FLDENV [3000h]
        mem.write16(0x106, 0x3000);

        decoder.step();

        REQUIRE(mem.read16(0x3000) == 0x0240);
        REQUIRE(mem.read16(0x3002) == static_cast<uint16_t>(cpu::FPU_STATUS_C0 |
                                                            cpu::FPU_STATUS_C3 |
                                                            (5u << 11)));
        REQUIRE(mem.read16(0x3004) == 0xA55A);
        REQUIRE(mem.read16(0x3006) == 0x79B4);
        REQUIRE(mem.read16(0x3008) == 0x5321);
        REQUIRE(mem.read16(0x300A) == 0x89BC);
        REQUIRE(mem.read16(0x300C) == 0xE000);
        REQUIRE((cpu.getFPUControlWord() & 0x003F) == 0x003F);

        cpu.setFPUControlWord(cpu::FPU_CONTROL_DEFAULT);
        cpu.setFPUStatusWord(0);
        cpu.setFPUTagWord(0xFFFF);
        cpu.setFPUInstructionPointer(0);
        cpu.setFPUInstructionSelector(0);
        cpu.setFPULastOpcode(0);
        cpu.setFPUDataPointer(0);
        cpu.setFPUDataSelector(0);

        decoder.step();

        REQUIRE(cpu.getFPUControlWord() == 0x0240);
        REQUIRE(cpu.getFPUStatusWord() == static_cast<uint16_t>(cpu::FPU_STATUS_C0 |
                                                                cpu::FPU_STATUS_C3 |
                                                                (5u << 11)));
        REQUIRE(cpu.getFPUTagWord() == 0xA55A);
        REQUIRE(cpu.getFPUInstructionPointer() == 0x579B4);
        REQUIRE(cpu.getFPUInstructionSelector() == 0);
        REQUIRE(cpu.getFPULastOpcode() == 0x0321);
        REQUIRE(cpu.getFPUDataPointer() == 0xE89BC);
        REQUIRE(cpu.getFPUDataSelector() == 0);
    }

    SECTION("x87: FNSAVE and FRSTOR roundtrip 16-bit real-mode state") {
        cpu.setFPUControlWord(0x027F);
        cpu.setFPUStatusWord(static_cast<uint16_t>(cpu::FPU_STATUS_C2 |
                                                   (6u << 11)));
        cpu.setFPUTagWord(0x00F0);
        cpu.setFPUInstructionPointer(0x1234);
        cpu.setFPUInstructionSelector(0x5678);
        cpu.setFPULastOpcode(0x0321);
        cpu.setFPUDataPointer(0x9ABC);
        cpu.setFPUDataSelector(0xDEF0);
        cpu.pushFPU(3.5);
        cpu.pushFPU(-2.25);

        mem.write8(0x100, 0xDD); mem.write8(0x101, 0x36); // FNSAVE [3100h]
        mem.write16(0x102, 0x3100);
        mem.write8(0x104, 0xDD); mem.write8(0x105, 0x26); // FRSTOR [3100h]
        mem.write16(0x106, 0x3100);
        mem.write8(0x108, 0xDD); mem.write8(0x109, 0x1E); // FSTP qword ptr [3200h]
        mem.write16(0x10A, 0x3200);
        mem.write8(0x10C, 0xDD); mem.write8(0x10D, 0x1E); // FSTP qword ptr [3208h]
        mem.write16(0x10E, 0x3208);

        decoder.step();

        REQUIRE(mem.read16(0x3100) == 0x027F);
        REQUIRE(mem.read16(0x3102) == static_cast<uint16_t>(cpu::FPU_STATUS_C2 |
                                                            (6u << 11)));
        REQUIRE(mem.read16(0x3104) == 0x00F0);
        REQUIRE(mem.read16(0x3106) == 0x79B4);
        REQUIRE(mem.read16(0x3108) == 0x5321);
        REQUIRE(mem.read16(0x310A) == 0x89BC);
        REQUIRE(mem.read16(0x310C) == 0xE000);
        REQUIRE(cpu.getFPUControlWord() == cpu::FPU_CONTROL_DEFAULT);
        REQUIRE(cpu.getFPUStatusWord() == 0);
        REQUIRE(cpu.isFPURegisterEmpty(0));

        decoder.step();
        decoder.step();
        decoder.step();

        REQUIRE(cpu.getFPUControlWord() == 0x027F);
        REQUIRE(cpu.getFPUStatusWord() == static_cast<uint16_t>(cpu::FPU_STATUS_C2 |
                                                                (6u << 11)));
        REQUIRE(cpu.getFPUTagWord() == 0x00F0);
        REQUIRE(cpu.getFPUInstructionPointer() == 0x579B4);
        REQUIRE(cpu.getFPUInstructionSelector() == 0);
        REQUIRE(cpu.getFPULastOpcode() == 0x0321);
        REQUIRE(cpu.getFPUDataPointer() == 0xE89BC);
        REQUIRE(cpu.getFPUDataSelector() == 0);
        REQUIRE(std::fabs(readF64(0x3200) - (-2.25)) < 1e-12);
        REQUIRE(std::fabs(readF64(0x3208) - 3.5) < 1e-12);
        REQUIRE(cpu.isFPURegisterEmpty(0));
    }

    SECTION("x87: FNSAVE and FRSTOR roundtrip 32-bit state") {
        cpu.setFPUControlWord(0x027F);
        cpu.setFPUInstructionPointer(0x12345678);
        cpu.setFPUInstructionSelector(0x1357);
        cpu.setFPULastOpcode(0x0321);
        cpu.setFPUDataPointer(0x87654321);
        cpu.setFPUDataSelector(0x2468);
        cpu.pushFPU(3.5);
        cpu.pushFPU(-2.25);
        cpu.setFPUStatusBits(cpu::FPU_STATUS_C2);

        mem.write8(0x100, 0x66);
        mem.write8(0x101, 0xDD); mem.write8(0x102, 0x36); // FNSAVE [3100h]
        mem.write16(0x103, 0x3100);
        mem.write8(0x105, 0x66);
        mem.write8(0x106, 0xDD); mem.write8(0x107, 0x26); // FRSTOR [3100h]
        mem.write16(0x108, 0x3100);
        mem.write8(0x10A, 0xDD); mem.write8(0x10B, 0x1E); // FSTP qword ptr [3200h]
        mem.write16(0x10C, 0x3200);
        mem.write8(0x10E, 0xDD); mem.write8(0x10F, 0x1E); // FSTP qword ptr [3208h]
        mem.write16(0x110, 0x3208);

        decoder.step();

        REQUIRE(mem.read32(0x3100) == 0x027F);
        REQUIRE(mem.read32(0x3104) == (cpu::FPU_STATUS_C2 | (6u << 11)));
        REQUIRE(mem.read32(0x3108) == 0x00F0);
        REQUIRE(mem.read32(0x310C) == 0x12345678);
        REQUIRE(mem.read32(0x3110) == ((static_cast<uint32_t>(0x0321) << 16) | 0x1357));
        REQUIRE(mem.read32(0x3114) == 0x87654321);
        REQUIRE(mem.read32(0x3118) == 0x2468);
        REQUIRE(cpu.getFPUControlWord() == cpu::FPU_CONTROL_DEFAULT);
        REQUIRE(cpu.getFPUStatusWord() == 0);
        REQUIRE(cpu.isFPURegisterEmpty(0));

        decoder.step();
        decoder.step();
        decoder.step();

        REQUIRE(cpu.getFPUControlWord() == 0x027F);
        REQUIRE(cpu.getFPUStatusWord() == static_cast<uint16_t>(cpu::FPU_STATUS_C2 |
                                                                (6u << 11)));
        REQUIRE(cpu.getFPUTagWord() == 0x00F0);
        REQUIRE(cpu.getFPUInstructionPointer() == 0x12345678);
        REQUIRE(cpu.getFPUInstructionSelector() == 0x1357);
        REQUIRE(cpu.getFPULastOpcode() == 0x0321);
        REQUIRE(cpu.getFPUDataPointer() == 0x87654321);
        REQUIRE(cpu.getFPUDataSelector() == 0x2468);
        REQUIRE(std::fabs(readF64(0x3200) - (-2.25)) < 1e-12);
        REQUIRE(std::fabs(readF64(0x3208) - 3.5) < 1e-12);
        REQUIRE(cpu.isFPURegisterEmpty(0));
    }

    SECTION("x87: FPTAN on zero pushes 1 then tan") {
        cpu.setFPUStatusWord(cpu::FPU_STATUS_C2);

        mem.write8(0x100, 0xD9); mem.write8(0x101, 0xEE); // FLDZ
        mem.write8(0x102, 0xD9); mem.write8(0x103, 0xF2); // FPTAN
        mem.write8(0x104, 0xDD); mem.write8(0x105, 0x1E); // FSTP qword ptr [2500h]
        mem.write16(0x106, 0x2500);
        mem.write8(0x108, 0xDD); mem.write8(0x109, 0x1E); // FSTP qword ptr [2508h]
        mem.write16(0x10A, 0x2508);

        decoder.step();
        decoder.step();
        decoder.step();
        decoder.step();

        REQUIRE(std::fabs(readF64(0x2500) - 1.0) < 1e-12);
        REQUIRE(std::fabs(readF64(0x2508) - 0.0) < 1e-12);
        REQUIRE((cpu.getFPUStatusWord() & cpu::FPU_STATUS_C2) == 0);
        REQUIRE(cpu.isFPURegisterEmpty(0));
    }

    SECTION("x87: FSINCOS on zero pushes cosine above sine") {
        cpu.setFPUStatusWord(cpu::FPU_STATUS_C2);

        mem.write8(0x100, 0xD9); mem.write8(0x101, 0xEE); // FLDZ
        mem.write8(0x102, 0xD9); mem.write8(0x103, 0xFB); // FSINCOS
        mem.write8(0x104, 0xDD); mem.write8(0x105, 0x1E); // FSTP qword ptr [2600h]
        mem.write16(0x106, 0x2600);
        mem.write8(0x108, 0xDD); mem.write8(0x109, 0x1E); // FSTP qword ptr [2608h]
        mem.write16(0x10A, 0x2608);

        decoder.step();
        decoder.step();
        decoder.step();
        decoder.step();

        REQUIRE(std::fabs(readF64(0x2600) - 1.0) < 1e-12);
        REQUIRE(std::fabs(readF64(0x2608) - 0.0) < 1e-12);
        REQUIRE((cpu.getFPUStatusWord() & cpu::FPU_STATUS_C2) == 0);
        REQUIRE(cpu.isFPURegisterEmpty(0));
    }

    SECTION("x87: FPTAN out of range sets C2 and leaves stack unchanged") {
        writeF64(0x2550, std::ldexp(1.0, 70));

        mem.write8(0x100, 0xDD); mem.write8(0x101, 0x06); // FLD qword ptr [2550h]
        mem.write16(0x102, 0x2550);
        mem.write8(0x104, 0xD9); mem.write8(0x105, 0xF2); // FPTAN
        mem.write8(0x106, 0xDD); mem.write8(0x107, 0x1E); // FSTP qword ptr [2558h]
        mem.write16(0x108, 0x2558);

        decoder.step();
        decoder.step();
        decoder.step();

        REQUIRE((cpu.getFPUStatusWord() & cpu::FPU_STATUS_C2) != 0);
        REQUIRE(std::fabs(readF64(0x2558) - std::ldexp(1.0, 70)) < 1.0);
        REQUIRE(cpu.isFPURegisterEmpty(0));
    }

    SECTION("x87: FPATAN returns atan2 and pops") {
        mem.write8(0x100, 0xD9); mem.write8(0x101, 0xE8); // FLD1
        mem.write8(0x102, 0xD9); mem.write8(0x103, 0xE8); // FLD1
        mem.write8(0x104, 0xD9); mem.write8(0x105, 0xF3); // FPATAN
        mem.write8(0x106, 0xDD); mem.write8(0x107, 0x1E); // FSTP qword ptr [2700h]
        mem.write16(0x108, 0x2700);

        decoder.step();
        decoder.step();
        decoder.step();
        decoder.step();

        REQUIRE(std::fabs(readF64(0x2700) - (std::acos(-1.0) / 4.0)) < 1e-12);
        REQUIRE(cpu.isFPURegisterEmpty(0));
    }

    SECTION("x87: FXTRACT separates finite exponent and significand") {
        writeF64(0x2800, 8.0);

        mem.write8(0x100, 0xDD); mem.write8(0x101, 0x06); // FLD qword ptr [2800h]
        mem.write16(0x102, 0x2800);
        mem.write8(0x104, 0xD9); mem.write8(0x105, 0xF4); // FXTRACT
        mem.write8(0x106, 0xDD); mem.write8(0x107, 0x1E); // FSTP qword ptr [2810h]
        mem.write16(0x108, 0x2810);
        mem.write8(0x10A, 0xDD); mem.write8(0x10B, 0x1E); // FSTP qword ptr [2818h]
        mem.write16(0x10C, 0x2818);

        decoder.step();
        decoder.step();
        decoder.step();
        decoder.step();

        REQUIRE(std::fabs(readF64(0x2810) - 1.0) < 1e-12);
        REQUIRE(std::fabs(readF64(0x2818) - 3.0) < 1e-12);
        REQUIRE(cpu.isFPURegisterEmpty(0));
    }

    SECTION("x87: FTST updates status word and FNSTSW AX reads it") {
        mem.write8(0x100, 0xD9); mem.write8(0x101, 0xEE); // FLDZ
        mem.write8(0x102, 0xD9); mem.write8(0x103, 0xE4); // FTST
        mem.write8(0x104, 0xDF); mem.write8(0x105, 0xE0); // FNSTSW AX

        decoder.step();
        decoder.step();
        decoder.step();

        REQUIRE((cpu.getReg16(cpu::Reg16Index::AX) & cpu::FPU_STATUS_C3) != 0);
        REQUIRE((cpu.getReg16(cpu::Reg16Index::AX) & cpu::FPU_STATUS_C0) == 0);
        REQUIRE((cpu.getReg16(cpu::Reg16Index::AX) & cpu::FPU_STATUS_C2) == 0);
    }

    SECTION("x87: tbyte roundtrip preserves finite value") {
        writeF64(0x2900, 3.5);

        mem.write8(0x100, 0xDD); mem.write8(0x101, 0x06); // FLD qword ptr [2900h]
        mem.write16(0x102, 0x2900);
        mem.write8(0x104, 0xDB); mem.write8(0x105, 0x3E); // FSTP tbyte ptr [2910h]
        mem.write16(0x106, 0x2910);
        mem.write8(0x108, 0xDB); mem.write8(0x109, 0x2E); // FLD tbyte ptr [2910h]
        mem.write16(0x10A, 0x2910);
        mem.write8(0x10C, 0xDD); mem.write8(0x10D, 0x1E); // FSTP qword ptr [2920h]
        mem.write16(0x10E, 0x2920);

        decoder.step();
        decoder.step();
        decoder.step();
        decoder.step();

        REQUIRE(std::fabs(readF64(0x2920) - 3.5) < 1e-12);
        REQUIRE(cpu.isFPURegisterEmpty(0));
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

TEST_CASE("x87: Borland coprocessor probe takes the 80387 branch", "[Decoder][x87]") {
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

    cpu.setSegReg(cpu::SegRegIndex::CS, 0x0000);
    cpu.setSegReg(cpu::SegRegIndex::DS, 0x0000);
    cpu.setSegReg(cpu::SegRegIndex::SS, 0x0000);
    cpu.setReg16(cpu::Reg16Index::SP, 0x0200);
    cpu.setEIP(0x0100);

    mem.write8(0x100, 0xDB); mem.write8(0x101, 0xE3); // FNINIT
    mem.write8(0x102, 0x9B);                           // FWAIT
    mem.write8(0x103, 0xD9); mem.write8(0x104, 0xE8); // FLD1
    mem.write8(0x105, 0x9B);                           // FWAIT
    mem.write8(0x106, 0xD9); mem.write8(0x107, 0xEE); // FLDZ
    mem.write8(0x108, 0x9B);                           // FWAIT
    mem.write8(0x109, 0xDE); mem.write8(0x10A, 0xF9); // FDIVP ST1, ST0
    mem.write8(0x10B, 0x9B);                           // FWAIT
    mem.write8(0x10C, 0xD9); mem.write8(0x10D, 0xC0); // FLD ST0
    mem.write8(0x10E, 0x9B);                           // FWAIT
    mem.write8(0x10F, 0xD9); mem.write8(0x110, 0xE0); // FCHS
    mem.write8(0x111, 0x9B);                           // FWAIT
    mem.write8(0x112, 0xDE); mem.write8(0x113, 0xD9); // FCOMPP
    mem.write8(0x114, 0x9B);                           // FWAIT
    mem.write8(0x115, 0xDD); mem.write8(0x116, 0x3E); // FNSTSW [2000h]
    mem.write16(0x117, 0x2000);
    mem.write8(0x119, 0xA1); mem.write16(0x11A, 0x2000); // MOV AX, [2000h]
    mem.write8(0x11C, 0x9E);                              // SAHF
    mem.write8(0x11D, 0x75); mem.write8(0x11E, 0x05);     // JNZ +5
    mem.write8(0x11F, 0xB8); mem.write16(0x120, 0x0002);  // MOV AX, 2
    mem.write8(0x122, 0xEB); mem.write8(0x123, 0x03);     // JMP +3
    mem.write8(0x124, 0xB8); mem.write16(0x125, 0x0003);  // MOV AX, 3

    for (int i = 0; i < 19; ++i) {
        decoder.step();
    }

    REQUIRE(cpu.getReg16(cpu::Reg16Index::AX) == 0x0003);
    REQUIRE((cpu.getEFLAGS() & cpu::FLAG_ZERO) == 0);
    REQUIRE((cpu.getEFLAGS() & cpu::FLAG_CARRY) != 0);
    REQUIRE((cpu.getEFLAGS() & cpu::FLAG_PARITY) == 0);
}
