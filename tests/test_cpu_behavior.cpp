#include <catch2/catch_test_macros.hpp>
#include "cpu/CPU.hpp"
#include "memory/MemoryBus.hpp"

using namespace fador;

TEST_CASE("MemoryBus Basic Operations", "[Memory]") {
    memory::MemoryBus mem;

    SECTION("Init zero state") {
        REQUIRE(mem.read8(0x00) == 0);
        REQUIRE(mem.read16(0x00) == 0);
        REQUIRE(mem.read32(0x00) == 0);
    }

    SECTION("8-bit Read/Write") {
        mem.write8(0x100, 0xAB);
        REQUIRE(mem.read8(0x100) == 0xAB);
    }

    SECTION("16-bit Read/Write Little Endian") {
        mem.write16(0x200, 0x1234);
        REQUIRE(mem.read16(0x200) == 0x1234);
        REQUIRE(mem.read8(0x200) == 0x34); // Low byte
        REQUIRE(mem.read8(0x201) == 0x12); // High byte
    }

    SECTION("32-bit Read/Write Little Endian") {
        mem.write32(0x300, 0xDEADBEEF);
        REQUIRE(mem.read32(0x300) == 0xDEADBEEF);
        REQUIRE(mem.read16(0x300) == 0xBEEF);
        REQUIRE(mem.read16(0x302) == 0xDEAD);
    }
}

TEST_CASE("CPU State and Registers", "[CPU]") {
    cpu::CPU cpu;

    SECTION("Reset State") {
        REQUIRE(cpu.getSegReg(cpu::SegRegIndex::CS) == 0xF000);
        REQUIRE(cpu.getEIP() == 0xFFF0);
        REQUIRE(cpu.getEFLAGS() == 0x00000002);
    }

    SECTION("General Purpose 32-bit") {
        cpu.setReg32(cpu::Reg32Index::EAX, 0x11223344);
        REQUIRE(cpu.getReg32(cpu::Reg32Index::EAX) == 0x11223344);
        
        cpu.setReg32(cpu::Reg32Index::EDX, 0xFFFFFFFF);
        REQUIRE(cpu.getReg32(cpu::Reg32Index::EDX) == 0xFFFFFFFF);
    }

    SECTION("Partial Regisrter Overlap (16-bit / 8-bit)") {
        cpu.setReg32(cpu::Reg32Index::EBX, 0x00000000);

        cpu.setReg16(cpu::Reg16Index::BX, 0xABCD);
        REQUIRE(cpu.getReg32(cpu::Reg32Index::EBX) == 0x0000ABCD);

        cpu.setReg8(cpu::Reg8Index::BL, 0xEF); // Low
        REQUIRE(cpu.getReg32(cpu::Reg32Index::EBX) == 0x0000ABEF);

        cpu.setReg8(cpu::Reg8Index::BH, 0x12); // High
        REQUIRE(cpu.getReg32(cpu::Reg32Index::EBX) == 0x000012EF);
    }
}
