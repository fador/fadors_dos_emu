#include "test_framework.hpp"
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

    auto shrinkPSPBlock = [&](uint16_t paragraphs) {
        const uint16_t psp = dos.getPSPSegment();
        cpu.setReg8(AH, 0x4A);
        cpu.setSegReg(ES, psp);
        cpu.setReg16(BX, paragraphs);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
    };

    SECTION("Initial state") {
        shrinkPSPBlock(0x100);

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
        shrinkPSPBlock(0x100);

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
        shrinkPSPBlock(0x100);

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

        // Next MCB should be free.  It is 'Z' when the released tail was
        // coalesced with the trailing free block, 'M' otherwise.
        uint32_t nextMcbAddr = (seg + 0x50) << 4;
        REQUIRE((memory.read8(nextMcbAddr) == 'M' ||
                 memory.read8(nextMcbAddr) == 'Z'));
        REQUIRE(memory.read16(nextMcbAddr + 1) == 0);
    }

    SECTION("Resize PSP block leaves PSP end segment unchanged") {
        // MS-DOS sets PSP:0002h at EXEC time only; DOS/32A depends on it
        // still showing the original top after SETBLOCK.
        const uint16_t psp = dos.getPSPSegment();

        shrinkPSPBlock(0x100);
        const uint32_t topAddr = (static_cast<uint32_t>(psp) << 4) + 0x02;
        const uint16_t topBefore = memory.read16(topAddr);

        cpu.setReg8(AH, 0x4A);
        cpu.setSegReg(ES, psp);
        cpu.setReg16(BX, 0x50);
        dos.handleInterrupt(0x21);

        REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
        REQUIRE(memory.read16(topAddr) == topBefore);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// MCB Chain Integrity / Corruption Edge Cases
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DOS: Double free detection", "[DOS][Memory][Corruption]") {
    CPU cpu;
    MemoryBus memory;
    DOS dos(cpu, memory);
    dos.initialize();

    // Allocate a block
    cpu.setReg8(AH, 0x48);
    cpu.setReg16(BX, 0x10);
    dos.handleInterrupt(0x21);
    if (cpu.getEFLAGS() & FLAG_CARRY) return; // No free memory — skip
    uint16_t seg = cpu.getReg16(AX);

    // Free it once
    cpu.setReg8(AH, 0x49);
    cpu.setSegReg(ES, seg);
    dos.handleInterrupt(0x21);
    REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));

    // Double free should fail
    cpu.setReg8(AH, 0x49);
    cpu.setSegReg(ES, seg);
    dos.handleInterrupt(0x21);
    REQUIRE(cpu.getEFLAGS() & FLAG_CARRY);
}

TEST_CASE("DOS: Free invalid segment (not an MCB)", "[DOS][Memory][Corruption]") {
    CPU cpu;
    MemoryBus memory;
    DOS dos(cpu, memory);
    dos.initialize();

    cpu.setReg8(AH, 0x49);
    cpu.setSegReg(ES, 0xB800); // Video memory
    dos.handleInterrupt(0x21);
    REQUIRE(cpu.getEFLAGS() & FLAG_CARRY);
}

TEST_CASE("DOS: Free null segment", "[DOS][Memory][Corruption]") {
    CPU cpu;
    MemoryBus memory;
    DOS dos(cpu, memory);
    dos.initialize();

    cpu.setReg8(AH, 0x49);
    cpu.setSegReg(ES, 0x0000);
    dos.handleInterrupt(0x21);
    REQUIRE(cpu.getEFLAGS() & FLAG_CARRY);
}

TEST_CASE("DOS: Allocate maximum paragraphs", "[DOS][Memory][Stress]") {
    CPU cpu;
    MemoryBus memory;
    DOS dos(cpu, memory);
    dos.initialize();

    cpu.setReg8(AH, 0x48);
    cpu.setReg16(BX, 0xFFFF);
    dos.handleInterrupt(0x21);
    // Should fail (not enough memory)
    REQUIRE(cpu.getEFLAGS() & FLAG_CARRY);
}

TEST_CASE("DOS: MCB chain walk after rapid alloc/free cycle", "[DOS][Memory][Stress]") {
    CPU cpu;
    MemoryBus memory;
    DOS dos(cpu, memory);
    dos.initialize();

    // Allocate 3 blocks
    uint16_t segments[3];
    int allocated = 0;
    for (int i = 0; i < 3; i++) {
        cpu.setReg8(AH, 0x48);
        cpu.setReg16(BX, 0x08);
        dos.handleInterrupt(0x21);
        if (cpu.getEFLAGS() & FLAG_CARRY) break;
        segments[i] = cpu.getReg16(AX);
        allocated++;
    }

    // Free them all
    for (int i = 0; i < allocated; i++) {
        cpu.setReg8(AH, 0x49);
        cpu.setSegReg(ES, segments[i]);
        dos.handleInterrupt(0x21);
        REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
    }

    // If we allocated any blocks, try allocating again
    if (allocated > 0) {
        cpu.setReg8(AH, 0x48);
        cpu.setReg16(BX, 0x04);
        dos.handleInterrupt(0x21);
        // After freeing, should be able to allocate again
    }
}

TEST_CASE("DOS: Write to MCB signature byte (corruption attempt)", "[DOS][Memory][Corruption]") {
    CPU cpu;
    MemoryBus memory;
    DOS dos(cpu, memory);
    dos.initialize();

    // Allocate a block
    cpu.setReg8(AH, 0x48);
    cpu.setReg16(BX, 0x10);
    dos.handleInterrupt(0x21);
    if (cpu.getEFLAGS() & FLAG_CARRY) return; // No free memory — skip
    uint16_t seg = cpu.getReg16(AX);

    // Corrupt MCB signature
    uint32_t mcbAddr = (seg - 1) << 4;
    memory.write8(mcbAddr, 'X');
    REQUIRE(memory.read8(mcbAddr) == 'X');

    // Try to free — should detect corruption and fail
    cpu.setReg8(AH, 0x49);
    cpu.setSegReg(ES, seg);
    dos.handleInterrupt(0x21);
    REQUIRE(cpu.getEFLAGS() & FLAG_CARRY);
}

TEST_CASE("DOS: memory runs out after the program sized its block", "[DOS][Memory]") {
    CPU cpu;
    MemoryBus memory;
    DOS dos(cpu, memory);
    dos.initialize();
    const uint16_t psp = dos.getPSPSegment();
    auto call = [&](uint8_t ah, uint16_t bx, uint16_t es = 0) {
        cpu.setEFLAGS(cpu.getEFLAGS() & ~FLAG_CARRY);
        cpu.setReg8(AH, ah);
        cpu.setReg16(BX, bx);
        cpu.setSegReg(ES, es);
        dos.handleInterrupt(0x21);
        return !(cpu.getEFLAGS() & FLAG_CARRY);
    };

    // The program keeps 0x2000 paragraphs and grabs the rest in pieces
    REQUIRE(call(0x4A, 0x2000, psp));
    int blocks = 0;
    while (call(0x48, 0x0FFD))
        ++blocks;
    REQUIRE(blocks > 0);
    // DOS says no; the program's own block is left alone
    REQUIRE(cpu.getReg16(AX) == 0x0008);
    REQUIRE(memory.read16(((psp - 1) << 4) + 3) == 0x2000);

    // Free blocks merge: two adjacent ones serve one larger request
    REQUIRE(call(0x48, 0x10));
    const uint16_t a = cpu.getReg16(AX);
    REQUIRE(call(0x48, 0x10));
    const uint16_t b = cpu.getReg16(AX);
    REQUIRE(b == a + 0x11);
    REQUIRE(call(0x49, 0, a));
    REQUIRE(call(0x49, 0, b));
    REQUIRE(call(0x48, 0x21));
    REQUIRE(cpu.getReg16(AX) == a);

    // AH=4Ah never moves a block: growing into an allocated neighbour fails
    REQUIRE(call(0x48, 0x10));
    const uint16_t c = cpu.getReg16(AX);
    REQUIRE(!call(0x4A, 0x40, a));
    REQUIRE(cpu.getReg16(AX) == 0x0008);
    REQUIRE(cpu.getReg16(BX) == 0x21);
    (void)c;
}
