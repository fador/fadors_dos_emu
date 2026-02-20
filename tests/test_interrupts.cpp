#include <catch2/catch_test_macros.hpp>
#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "memory/MemoryBus.hpp"

using namespace fador;

TEST_CASE("CPU Interrupt Pipeline", "[Interrupts]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    cpu::InstructionDecoder decoder(cpu, mem);

    // Setup base execution environment
    cpu.setSegReg(cpu::SegRegIndex::CS, 0x1000);
    cpu.setSegReg(cpu::SegRegIndex::DS, 0x0000);
    cpu.setSegReg(cpu::SegRegIndex::SS, 0x0000);
    cpu.setReg16(cpu::Reg16Index::SP, 0x0100);
    cpu.setEIP(0x100);

    // Setup IVT vector 0x21 (DOS Services)
    mem.write16(0x21 * 4, 0x1234);     // IP = 0x1234
    mem.write16((0x21 * 4) + 2, 0xF000); // CS = 0xF000

    SECTION("Software Interrupt INT imm8 (0xCD 0x21)") {
        mem.write8((0x1000 << 4) + 0x100, 0xCD); // INT
        mem.write8((0x1000 << 4) + 0x101, 0x21); // 0x21

        uint32_t prevEflags = cpu.getEFLAGS();

        decoder.step();

        // Ensure Jumped
        REQUIRE(cpu.getSegReg(cpu::SegRegIndex::CS) == 0xF000);
        REQUIRE(cpu.getEIP() == 0x1234);

        // Check Stack
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SP) == 0x00FA); // 0x100 - 6 bytes pushed
        
        // Stack contains IP, CS, FLAGS
        REQUIRE(mem.read16((0x0000 << 4) + 0x00FA) == 0x0102); // Return IP
        REQUIRE(mem.read16((0x0000 << 4) + 0x00FC) == 0x1000); // Return CS
        REQUIRE(mem.read16((0x0000 << 4) + 0x00FE) == (prevEflags & 0xFFFF)); // Pushed FLAGS
    }

    SECTION("Interrupt Return IRET (0xCF)") {
        // Mock a stack frame: IP=0xABCD, CS=0x1111, FLAGS=0x0002
        cpu.setReg16(cpu::Reg16Index::SP, 0x00FA);
        mem.write16((0x0000 << 4) + 0x00FA, 0xABCD); // return IP
        mem.write16((0x0000 << 4) + 0x00FC, 0x1111); // return CS
        mem.write16((0x0000 << 4) + 0x00FE, 0x0002); // return FLAGS

        mem.write8((0x1000 << 4) + 0x100, 0xCF); // IRET
        decoder.step();

        REQUIRE(cpu.getSegReg(cpu::SegRegIndex::CS) == 0x1111);
        REQUIRE(cpu.getEIP() == 0xABCD);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SP) == 0x0100);
    }
}
