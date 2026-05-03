#include "test_framework.hpp"
#include "cpu/Disassembler.hpp"
#include "memory/MemoryBus.hpp"

using namespace fador;

TEST_CASE("Disassembler - Single Instructions", "[Disassembler]") {
    memory::MemoryBus mem;
    cpu::Disassembler disasm(mem);

    SECTION("disassembleAt: ADD AL, 5") {
        // ADD AL, 5 -> 04 05
        mem.write8(0x100, 0x04);
        mem.write8(0x101, 0x05);

        auto instr = disasm.disassembleAt(0x100);

        REQUIRE(instr.address == 0x100);
        REQUIRE(instr.length == 2);
        REQUIRE(instr.mnemonic == "ADD AL, 05h");
        REQUIRE(instr.hexBytes == "04 05");
    }

    SECTION("disassembleAt: SUB AX, DX") {
        // SUB AX, DX -> 29 D0
        mem.write8(0x100, 0x29);
        mem.write8(0x101, 0xD0);

        auto instr = disasm.disassembleAt(0x100);

        REQUIRE(instr.address == 0x100);
        REQUIRE(instr.length == 2);
        REQUIRE(instr.mnemonic == "SUB AX, DX");
        REQUIRE(instr.hexBytes == "29 D0");
    }

    SECTION("disassembleAt: MOV AX, 1234h") {
        // MOV AX, 1234h -> B8 34 12
        mem.write8(0x100, 0xB8);
        mem.write8(0x101, 0x34);
        mem.write8(0x102, 0x12);

        auto instr = disasm.disassembleAt(0x100);

        REQUIRE(instr.address == 0x100);
        REQUIRE(instr.length == 3);
        REQUIRE(instr.mnemonic == "MOV AX, 1234h");
        REQUIRE(instr.hexBytes == "B8 34 12");
    }

    SECTION("disassembleAt: BSWAP EAX") {
        mem.write8(0x100, 0x0F);
        mem.write8(0x101, 0xC8);

        auto instr = disasm.disassembleAt(0x100);

        REQUIRE(instr.address == 0x100);
        REQUIRE(instr.length == 2);
        REQUIRE(instr.mnemonic == "BSWAP EAX");
        REQUIRE(instr.hexBytes == "0F C8");
    }

    SECTION("disassembleAt: FNSTENV [3000h]") {
        mem.write8(0x100, 0xD9);
        mem.write8(0x101, 0x36);
        mem.write8(0x102, 0x00);
        mem.write8(0x103, 0x30);

        auto instr = disasm.disassembleAt(0x100);

        REQUIRE(instr.address == 0x100);
        REQUIRE(instr.length == 4);
        REQUIRE(instr.mnemonic == "FNSTENV [3000h]");
        REQUIRE(instr.hexBytes == "D9 36 00 30");
    }

    SECTION("disassembleAt: FLDENV [3000h]") {
        mem.write8(0x100, 0xD9);
        mem.write8(0x101, 0x26);
        mem.write8(0x102, 0x00);
        mem.write8(0x103, 0x30);

        auto instr = disasm.disassembleAt(0x100);

        REQUIRE(instr.address == 0x100);
        REQUIRE(instr.length == 4);
        REQUIRE(instr.mnemonic == "FLDENV [3000h]");
        REQUIRE(instr.hexBytes == "D9 26 00 30");
    }

    SECTION("disassembleAt: FNSAVE [3100h]") {
        mem.write8(0x100, 0xDD);
        mem.write8(0x101, 0x36);
        mem.write8(0x102, 0x00);
        mem.write8(0x103, 0x31);

        auto instr = disasm.disassembleAt(0x100);

        REQUIRE(instr.address == 0x100);
        REQUIRE(instr.length == 4);
        REQUIRE(instr.mnemonic == "FNSAVE [3100h]");
        REQUIRE(instr.hexBytes == "DD 36 00 31");
    }

    SECTION("disassembleAt: FRSTOR [3100h]") {
        mem.write8(0x100, 0xDD);
        mem.write8(0x101, 0x26);
        mem.write8(0x102, 0x00);
        mem.write8(0x103, 0x31);

        auto instr = disasm.disassembleAt(0x100);

        REQUIRE(instr.address == 0x100);
        REQUIRE(instr.length == 4);
        REQUIRE(instr.mnemonic == "FRSTOR [3100h]");
        REQUIRE(instr.hexBytes == "DD 26 00 31");
    }

    SECTION("disassembleAt: FNSTENV [00003000h] with 32-bit addressing") {
        mem.write8(0x100, 0x67);
        mem.write8(0x101, 0xD9);
        mem.write8(0x102, 0x35);
        mem.write8(0x103, 0x00);
        mem.write8(0x104, 0x30);
        mem.write8(0x105, 0x00);
        mem.write8(0x106, 0x00);

        auto instr = disasm.disassembleAt(0x100);

        REQUIRE(instr.address == 0x100);
        REQUIRE(instr.length == 7);
        REQUIRE(instr.mnemonic == "FNSTENV [00003000h]");
        REQUIRE(instr.hexBytes == "67 D9 35 00 30 00 00");
    }
}

TEST_CASE("Disassembler - Range", "[Disassembler]") {
    memory::MemoryBus mem;
    cpu::Disassembler disasm(mem);

    SECTION("disassembleRange: Multiple instructions") {
        // MOV AX, 1234h -> B8 34 12
        mem.write8(0x100, 0xB8);
        mem.write8(0x101, 0x34);
        mem.write8(0x102, 0x12);

        // ADD AL, 5 -> 04 05
        mem.write8(0x103, 0x04);
        mem.write8(0x104, 0x05);

        // SUB AX, DX -> 29 D0
        mem.write8(0x105, 0x29);
        mem.write8(0x106, 0xD0);

        auto instrs = disasm.disassembleRange(0x100, 3);

        REQUIRE(instrs.size() == 3);

        REQUIRE(instrs[0].address == 0x100);
        REQUIRE(instrs[0].length == 3);
        REQUIRE(instrs[0].mnemonic == "MOV AX, 1234h");
        REQUIRE(instrs[0].hexBytes == "B8 34 12");

        REQUIRE(instrs[1].address == 0x103);
        REQUIRE(instrs[1].length == 2);
        REQUIRE(instrs[1].mnemonic == "ADD AL, 05h");
        REQUIRE(instrs[1].hexBytes == "04 05");

        REQUIRE(instrs[2].address == 0x105);
        REQUIRE(instrs[2].length == 2);
        REQUIRE(instrs[2].mnemonic == "SUB AX, DX");
        REQUIRE(instrs[2].hexBytes == "29 D0");
    }
}

TEST_CASE("Disassembler - Context (Around)", "[Disassembler]") {
    memory::MemoryBus mem;
    cpu::Disassembler disasm(mem);

    SECTION("disassembleAround: Instructions before and after") {
        // Fill a region with NOPs (0x90) so scanning backwards has a predictable sled
        for (uint32_t i = 0x100; i < 0x200; ++i) {
            mem.write8(i, 0x90);
        }

        // At 0x150: MOV AX, 1234h -> B8 34 12
        mem.write8(0x150, 0xB8);
        mem.write8(0x151, 0x34);
        mem.write8(0x152, 0x12);

        // At 0x153: ADD AL, 5 -> 04 05
        mem.write8(0x153, 0x04);
        mem.write8(0x154, 0x05);

        // At 0x155: SUB AX, DX -> 29 D0
        mem.write8(0x155, 0x29);
        mem.write8(0x156, 0xD0);

        // We want 2 instructions before 0x153 (NOP at 0x14F, and MOV AX at 0x150)
        // Center is 0x153.
        // And 2 instructions after center (so ADD AL at 0x153, and SUB AX at 0x155)

        // Since it's a heuristic backward scan, the exact NOP boundary might vary,
        // but it should definitely find the MOV AX if we ask for `before=2`.
        auto instrs = disasm.disassembleAround(0x153, 2, 2);

        // Expect: NOP, MOV AX, ADD AL, SUB AX
        REQUIRE(instrs.size() == 4);

        REQUIRE(instrs[0].address == 0x14F);
        REQUIRE(instrs[0].mnemonic == "NOP");

        REQUIRE(instrs[1].address == 0x150);
        REQUIRE(instrs[1].mnemonic == "MOV AX, 1234h");

        REQUIRE(instrs[2].address == 0x153);
        REQUIRE(instrs[2].mnemonic == "ADD AL, 05h");

        REQUIRE(instrs[3].address == 0x155);
        REQUIRE(instrs[3].mnemonic == "SUB AX, DX");
    }
}
