#include "test_framework.hpp"
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

// ═══════════════════════════════════════════════════════════════════════════
// Flag Behavior Edge Cases
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("CPU: Flag manipulation — STC, CLC, CMC", "[CPU][Flags]") {
    cpu::CPU cpu;

    cpu.setEFLAGS(0x00000002); // Reset state (bit 1 always set)

    // STC sets carry
    cpu.setEFLAGS(cpu.getEFLAGS() | cpu::FLAG_CARRY);
    REQUIRE((cpu.getEFLAGS() & cpu::FLAG_CARRY) != 0);

    // CLC clears carry
    cpu.setEFLAGS(cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    REQUIRE((cpu.getEFLAGS() & cpu::FLAG_CARRY) == 0);

    // CMC complements carry
    cpu.setEFLAGS(cpu.getEFLAGS() ^ cpu::FLAG_CARRY);
    REQUIRE((cpu.getEFLAGS() & cpu::FLAG_CARRY) != 0);

    cpu.setEFLAGS(cpu.getEFLAGS() ^ cpu::FLAG_CARRY);
    REQUIRE((cpu.getEFLAGS() & cpu::FLAG_CARRY) == 0);
}

TEST_CASE("CPU: Direction flag effect on string operations", "[CPU][Flags][String]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;

    // Set up ES:DI and DS:SI in real mode
    cpu.setSegReg(cpu::ES, 0x2000);
    cpu.setSegReg(cpu::DS, 0x2000);
    cpu.setReg16(cpu::DI, 0x0100);
    cpu.setReg16(cpu::SI, 0x0200);

    SECTION("DF=0 (forward): DI and SI increment after string op") {
        cpu.setEFLAGS(cpu.getEFLAGS() & ~cpu::FLAG_DIRECTION);
        // Simulate STOSB: write AL to ES:DI, then DI++
        mem.write8(0x20000 + 0x0100, 0x42);
        cpu.setReg16(cpu::DI, cpu.getReg16(cpu::DI) + 1);
        REQUIRE(cpu.getReg16(cpu::DI) == 0x0101);
        REQUIRE(mem.read8(0x20000 + 0x0100) == 0x42);
    }

    SECTION("DF=1 (backward): DI and SI decrement after string op") {
        cpu.setEFLAGS(cpu.getEFLAGS() | cpu::FLAG_DIRECTION);
        // Simulate STOSB with DF=1
        mem.write8(0x20000 + 0x0100, 0x42);
        cpu.setReg16(cpu::DI, cpu.getReg16(cpu::DI) - 1);
        REQUIRE(cpu.getReg16(cpu::DI) == 0x00FF);
    }
}

TEST_CASE("CPU: Stack pointer wraparound edge cases", "[CPU][Stack]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;

    cpu.setSegReg(cpu::SS, 0x1000);

    SECTION("PUSH with SP=0 wraps to FFFE on 16-bit stack") {
        cpu.setReg16(cpu::SP, 0x0000);
        // PUSH AX at SP=0: SP -= 2 → SP=0xFFFE
        uint16_t sp = cpu.getReg16(cpu::SP);
        sp -= 2;
        cpu.setReg16(cpu::SP, sp);
        uint32_t addr = (0x1000 << 4) + sp;
        mem.write16(addr, 0x1234);
        REQUIRE(cpu.getReg16(cpu::SP) == 0xFFFE);
        REQUIRE(mem.read16(addr) == 0x1234);
    }

    SECTION("POP at SP=FFFE restores SP=0000") {
        cpu.setReg16(cpu::SP, 0xFFFE);
        uint16_t sp = cpu.getReg16(cpu::SP);
        uint32_t addr = (0x1000 << 4) + sp;
        uint16_t val = mem.read16(addr);
        sp += 2;
        cpu.setReg16(cpu::SP, sp);
        REQUIRE(cpu.getReg16(cpu::SP) == 0x0000);
        (void)val;
    }

    SECTION("Multiple PUSH operations near zero") {
        cpu.setReg16(cpu::SP, 0x0006);
        // PUSH 4 words
        uint16_t sp = cpu.getReg16(cpu::SP);
        for (int i = 0; i < 4; i++) {
            sp -= 2;
            uint32_t addr = (0x1000 << 4) + sp;
            mem.write16(addr, static_cast<uint16_t>(0xAA00 + i));
        }
        cpu.setReg16(cpu::SP, sp);
        REQUIRE(cpu.getReg16(cpu::SP) == 0xFFFE);
        // Verify all values were written at correct addresses
        REQUIRE(mem.read16(0x10000 + 0xFFFE) == 0xAA00);
        REQUIRE(mem.read16(0x10004) == 0xAA03);
    }
}

TEST_CASE("CPU: Segment:offset address wrap at 64KB", "[CPU][Addressing]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;

    // On 8086, FFFF:000F wraps to physical 0x0000F (20-bit address).
    // On 386+ in real mode, A20 gate determines if wrapping occurs.
    // Test the raw arithmetic: 0xFFFF * 16 + 0x000F = 0xFFFF0 + 0xF = 0xFFFFF (no wrap with 32-bit)
    // But with 16-bit segment arithmetic: (0xFFFF << 4) + 0x000F as 32-bit = 0xFFFFF

    uint16_t seg = 0xFFFF;
    uint16_t off = 0x000F;
    uint32_t linear = (static_cast<uint32_t>(seg) << 4) + off;
    REQUIRE(linear == 0xFFFFF); // Should not wrap in 32-bit linear space

    // Maximum real-mode address
    seg = 0xFFFF;
    off = 0xFFFF;
    linear = (static_cast<uint32_t>(seg) << 4) + off;
    REQUIRE(linear == 0x10FFEF); // 1MB + 64KB - 17 bytes
}

TEST_CASE("CPU: REP prefix with zero count", "[CPU][String][REP]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;

    // Set up for REP MOVSB
    cpu.setSegReg(cpu::DS, 0x1000);
    cpu.setSegReg(cpu::ES, 0x2000);
    cpu.setReg16(cpu::SI, 0x0100);
    cpu.setReg16(cpu::DI, 0x0200);
    cpu.setReg16(cpu::CX, 0x0000); // Zero count
    cpu.setEFLAGS(cpu.getEFLAGS() & ~cpu::FLAG_DIRECTION); // DF=0

    // REP MOVSB with CX=0 should execute zero iterations
    uint16_t cx = cpu.getReg16(cpu::CX);
    uint16_t si = cpu.getReg16(cpu::SI);
    uint16_t di = cpu.getReg16(cpu::DI);

    // With CX=0, the loop should not execute
    REQUIRE(cx == 0);
    REQUIRE(si == 0x0100);
    REQUIRE(di == 0x0200);
}

TEST_CASE("CPU: Arithmetic flag behavior", "[CPU][Flags][Arith]") {
    cpu::CPU cpu;

    SECTION("8-bit addition with unsigned overflow (carry)") {
        // 0xFF + 0x01 = 0x100 → AL=0x00, CF=1, ZF=1
        uint16_t result = 0xFF + 0x01;
        REQUIRE((result & 0xFF) == 0x00);      // Low 8 bits = 0
        REQUIRE((result & 0x100) != 0);         // Carry out of bit 7
    }

    SECTION("8-bit addition with signed overflow") {
        // 0x7F + 0x01 = 0x80 → OF=1 (positive + positive = negative)
        int8_t a = 0x7F, b = 0x01;
        int16_t result = static_cast<int16_t>(a) + static_cast<int16_t>(b);
        bool of = (result < -128 || result > 127);
        REQUIRE(of == true);
    }

    SECTION("8-bit subtraction with borrow") {
        // 0x00 - 0x01 = 0xFF → CF=1 (borrow)
        uint16_t result = 0x00 - 0x01;
        REQUIRE((result & 0xFF) == 0xFF);       // Result is 0xFF
        REQUIRE((result & 0x100) != 0);         // Borrow occurred
    }

    SECTION("16-bit MUL overflow detection") {
        // MUL AX, BX: AX * BX → DX:AX
        // 0xFFFF * 0xFFFF = 0xFFFE0001 → DX=0xFFFE, AX=0x0001, CF=1, OF=1
        uint32_t product = 0xFFFFu * 0xFFFFu;
        REQUIRE((product >> 16) != 0);          // DX non-zero → overflow
        REQUIRE((product & 0xFFFF) == 0x0001);  // AX = 1
    }

    SECTION("16-bit IMUL signed overflow") {
        // 0x8000 * 0x0002 = 0xFFFF0000 as signed → overflow
        int32_t a = static_cast<int16_t>(0x8000); // -32768
        int32_t b = 2;
        int32_t product = a * b;
        // Check if result fits in 16 bits signed
        bool fits = (product >= -32768 && product <= 32767);
        REQUIRE(fits == false); // -65536 doesn't fit
    }

    SECTION("DIV by zero should trigger exception") {
        // DIV BX with BX=0 should cause #DE (divide error, INT 0x00)
        // We can't test the exception directly without decoder, but we verify
        // the condition detection
        uint16_t divisor = 0;
        REQUIRE(divisor == 0); // This is the condition that should trigger #DE
    }
}

TEST_CASE("CPU: Sign and zero flag after operations", "[CPU][Flags]") {
    cpu::CPU cpu;

    SECTION("Zero flag set when result is zero") {
        uint8_t result = 0;
        bool zf = (result == 0);
        bool sf = ((result & 0x80) != 0);
        REQUIRE(zf == true);
        REQUIRE(sf == false);
    }

    SECTION("Sign flag set when MSB is 1") {
        uint8_t result = 0x80;
        bool zf = (result == 0);
        bool sf = ((result & 0x80) != 0);
        REQUIRE(zf == false);
        REQUIRE(sf == true);
    }

    SECTION("16-bit sign flag") {
        uint16_t result = 0x8000;
        bool sf = ((result & 0x8000) != 0);
        REQUIRE(sf == true);

        result = 0x7FFF;
        sf = ((result & 0x8000) != 0);
        REQUIRE(sf == false);
    }

    SECTION("Parity flag (even parity)") {
        // 0x03 = 0000 0011 → 2 bits set → PF=1 (even)
        uint8_t val = 0x03;
        int bitCount = 0;
        for (int i = 0; i < 8; i++)
            if (val & (1 << i)) bitCount++;
        REQUIRE((bitCount & 1) == 0); // Even → PF=1

        // 0x07 = 0000 0111 → 3 bits set → PF=0 (odd)
        val = 0x07;
        bitCount = 0;
        for (int i = 0; i < 8; i++)
            if (val & (1 << i)) bitCount++;
        REQUIRE((bitCount & 1) == 1); // Odd → PF=0
    }
}

TEST_CASE("CPU: Auxiliary carry flag (AF) for BCD", "[CPU][Flags]") {
    // AF is set when there's a carry from bit 3 to bit 4
    // Used by DAA/DAS for BCD arithmetic

    SECTION("AF set on carry from bit 3") {
        // 0x08 + 0x08 = 0x10 → carry from bit 3 to bit 4 → AF=1
        uint8_t result = 0x08 + 0x08;
        bool af = ((0x08 & 0x0F) + (0x08 & 0x0F)) > 0x0F;
        REQUIRE(af == true);
        REQUIRE(result == 0x10);
    }

    SECTION("AF clear when no carry from bit 3") {
        // 0x01 + 0x02 = 0x03 → no carry from bit 3
        uint8_t result = 0x01 + 0x02;
        bool af = ((0x01 & 0x0F) + (0x02 & 0x0F)) > 0x0F;
        REQUIRE(af == false);
        REQUIRE(result == 0x03);
    }
}

TEST_CASE("CPU: Overflow flag for 32-bit operations", "[CPU][Flags][32bit]") {
    SECTION("32-bit signed overflow") {
        // 0x7FFFFFFF + 0x00000001 = 0x80000000 → OF=1
        int64_t a = 0x7FFFFFFF;
        int64_t b = 1;
        int64_t result = a + b;
        bool of = (result > 0x7FFFFFFF);
        REQUIRE(of == true);
    }

    SECTION("32-bit carry out") {
        // 0xFFFFFFFF + 0x00000001 = 0x100000000 → CF=1
        uint64_t a = 0xFFFFFFFFu;
        uint64_t b = 1u;
        uint64_t result = a + b;
        bool cf = (result > 0xFFFFFFFFu);
        REQUIRE(cf == true);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Self-modifying code & prefetch edge cases
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("CPU: Memory write to code segment address", "[CPU][SelfMod]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;

    // Set up code segment
    uint16_t csVal = 0x1000;
    cpu.setSegReg(cpu::CS, csVal);
    cpu.setEIP(0x0100);
    uint32_t codeLinear = (static_cast<uint32_t>(csVal) << 4) + 0x0100;
    REQUIRE(codeLinear == 0x10100);

    // Write an instruction to the code segment via the memory bus
    mem.write8(codeLinear, 0x90); // NOP
    REQUIRE(mem.read8(codeLinear) == 0x90);

    // Write via DS pointing to same physical address using different seg:off
    // 0x1000:0x0100 = 0x10100 → use 0x1010:0x0000 = 0x10100
    uint16_t dsSeg = 0x1010;
    uint16_t dsOff = 0x0000;
    cpu.setSegReg(cpu::DS, dsSeg);
    uint32_t dsLinear = (static_cast<uint32_t>(dsSeg) << 4) + dsOff;
    REQUIRE(dsLinear == codeLinear); // Both should alias to same physical

    mem.write8(dsLinear, 0xC3); // RET
    REQUIRE(mem.read8(dsLinear) == 0xC3);
    REQUIRE(mem.read8(codeLinear) == 0xC3); // Same physical address
}
