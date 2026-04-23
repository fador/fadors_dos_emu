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
