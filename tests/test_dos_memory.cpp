#include <catch2/catch_test_macros.hpp>
#include "hw/DOS.hpp"
#include "cpu/CPU.hpp"
#include "memory/MemoryBus.hpp"

using namespace fador::hw;
using namespace fador::cpu;
using namespace fador::memory;

TEST_CASE("DOS: Memory Management (MCB)", "[DOS][Memory]") {
    CPU cpu;
    MemoryBus memory;
    DOS dos(cpu, memory);

    dos.initialize();

    SECTION("Initial state") {
        // ES:BX = 0x1000 paragraphs (64KB)
        cpu.setReg8(AH, 0x48);
        cpu.setReg16(BX, 0x1000);
        dos.handleInterrupt(0x21);

        REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
        uint16_t segment = cpu.getReg16(AX);
        REQUIRE(segment > 0x0700);
        
        // Verify MCB owner
        uint32_t mcbAddr = (segment - 1) << 4;
        REQUIRE(memory.read8(mcbAddr) == 'M');
        REQUIRE(memory.read16(mcbAddr + 3) == 0x1000);
    }

    SECTION("Allocate and Free") {
        // Allocate 0x10
        cpu.setReg8(AH, 0x48);
        cpu.setReg16(BX, 0x10);
        dos.handleInterrupt(0x21);
        uint16_t seg1 = cpu.getReg16(AX);

        // Allocate 0x20
        cpu.setReg8(AH, 0x48);
        cpu.setReg16(BX, 0x20);
        dos.handleInterrupt(0x21);
        uint16_t seg2 = cpu.getReg16(AX);

        REQUIRE(seg2 == seg1 + 0x10 + 1);

        // Free first block
        cpu.setReg8(AH, 0x49);
        cpu.setSegReg(ES, seg1);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));

        // MCB should be free (Owner 0)
        uint32_t mcb1Addr = (seg1 - 1) << 4;
        REQUIRE(memory.read16(mcb1Addr + 1) == 0);
    }

    SECTION("Resize Block") {
        // Allocate 0x100
        cpu.setReg8(AH, 0x48);
        cpu.setReg16(BX, 0x100);
        dos.handleInterrupt(0x21);
        uint16_t seg = cpu.getReg16(AX);

        // Shrink to 0x50
        cpu.setReg8(AH, 0x4A);
        cpu.setSegReg(ES, seg);
        cpu.setReg16(BX, 0x50);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));

        uint32_t mcbAddr = (seg - 1) << 4;
        REQUIRE(memory.read16(mcbAddr + 3) == 0x50);

        // Next MCB should be free
        uint32_t nextMcbAddr = (seg + 0x50) << 4;
        REQUIRE(memory.read8(nextMcbAddr) == 'M');
        REQUIRE(memory.read16(nextMcbAddr + 1) == 0);
    }
}
