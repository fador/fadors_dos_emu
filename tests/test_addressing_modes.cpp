#include <catch2/catch_test_macros.hpp>
#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "memory/MemoryBus.hpp"
#include "hw/IOBus.hpp"
#include "hw/BIOS.hpp"
#include "hw/DOS.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIT8254.hpp"

using namespace fador::cpu;
using namespace fador::memory;
using namespace fador::hw;

TEST_CASE("CPU: Addressing Modes (16-bit)", "[Decoder][Addressing]") {
    CPU cpu;
    MemoryBus memory;
    IOBus iobus;
    KeyboardController kbd;
    PIT8254 pit;
    BIOS bios(cpu, memory, kbd, pit);
    DOS dos(cpu, memory);
    InstructionDecoder decoder(cpu, memory, iobus, bios, dos);

    cpu.setReg16(BX, 0x1000);
    cpu.setReg16(SI, 0x0100);
    cpu.setReg16(DI, 0x0010);
    cpu.setReg16(BP, 0x2000);
    cpu.setSegReg(DS, 0x3000);
    cpu.setSegReg(SS, 0x4000);

    SECTION("BX + SI") {
        ModRM modrm = {0, 0, 0}; // [BX + SI]
        uint32_t addr = decoder.getEffectiveAddress16(modrm);
        REQUIRE(addr == (0x3000 << 4) + 0x1100);
    }

    SECTION("BX + DI") {
        ModRM modrm = {0, 0, 1}; // [BX + DI]
        uint32_t addr = decoder.getEffectiveAddress16(modrm);
        REQUIRE(addr == (0x3000 << 4) + 0x1010);
    }

    SECTION("BP + SI (Default SS)") {
        ModRM modrm = {0, 0, 2}; // [BP + SI]
        uint32_t addr = decoder.getEffectiveAddress16(modrm);
        REQUIRE(addr == (0x4000 << 4) + 0x2100);
    }

    SECTION("BP + DI (Default SS)") {
        ModRM modrm = {0, 0, 3}; // [BP + DI]
        uint32_t addr = decoder.getEffectiveAddress16(modrm);
        REQUIRE(addr == (0x4000 << 4) + 0x2010);
    }

    SECTION("SI") {
        ModRM modrm = {0, 0, 4}; // [SI]
        uint32_t addr = decoder.getEffectiveAddress16(modrm);
        REQUIRE(addr == (0x3000 << 4) + 0x0100);
    }

    SECTION("DI") {
        ModRM modrm = {0, 0, 5}; // [DI]
        uint32_t addr = decoder.getEffectiveAddress16(modrm);
        REQUIRE(addr == (0x3000 << 4) + 0x0010);
    }

    SECTION("BP (Default SS)") {
        ModRM modrm = {1, 0, 6}; // [BP + disp8]
        // Push disp8 to memory
        uint32_t codeAddr = (cpu.getSegReg(CS) << 4) + cpu.getEIP();
        memory.write8(codeAddr, 0x05);
        
        uint32_t addr = decoder.getEffectiveAddress16(modrm);
        REQUIRE(addr == (0x4000 << 4) + 0x2005);
    }

    SECTION("Direct Address") {
        ModRM modrm = {0, 0, 6}; // [disp16]
        uint32_t codeAddr = (cpu.getSegReg(CS) << 4) + cpu.getEIP();
        memory.write16(codeAddr, 0x1234);
        
        uint32_t addr = decoder.getEffectiveAddress16(modrm);
        REQUIRE(addr == (0x3000 << 4) + 0x1234);
    }
}

TEST_CASE("CPU: Addressing Modes (32-bit)", "[Decoder][Addressing]") {
    CPU cpu;
    MemoryBus memory;
    IOBus iobus;
    KeyboardController kbd;
    PIT8254 pit;
    BIOS bios(cpu, memory, kbd, pit);
    DOS dos(cpu, memory);
    InstructionDecoder decoder(cpu, memory, iobus, bios, dos);

    cpu.setReg32(EBX, 0x10000);
    cpu.setReg32(ESI, 0x1000);
    cpu.setReg32(ESP, 0x20000);
    cpu.setSegReg(DS, 0x3000);
    cpu.setSegReg(SS, 0x4000);

    SECTION("Simple register [EBX]") {
        ModRM modrm = {0, 0, 3}; // [EBX]
        uint32_t addr = decoder.getEffectiveAddress32(modrm);
        REQUIRE(addr == (0x3000 << 4) + 0x10000);
    }

    SECTION("SIB [EBX + ESI * 4]") {
        ModRM modrm = {0, 0, 4}; // SIB follows
        uint32_t codeAddr = (cpu.getSegReg(CS) << 4) + cpu.getEIP();
        // SIB: scale=2 (x4), index=6 (ESI), base=3 (EBX)
        // 0x93 = 10 010 011 -> scale=2, index=2 (EDX? wait)
        // scale=2 (bits 6-7), index=ESI=6 (bits 3-5), base=EBX=3 (bits 0-2)
        // 10 110 011 = 0xB3
        memory.write8(codeAddr, 0xB3);
        
        uint32_t addr = decoder.getEffectiveAddress32(modrm);
        REQUIRE(addr == (0x3000 << 4) + (0x10000 + (0x1000 << 2)));
    }

    SECTION("ESP default SS") {
        ModRM modrm = {0, 0, 4}; // SIB follows
        uint32_t codeAddr = (cpu.getSegReg(CS) << 4) + cpu.getEIP();
        // SIB: scale=0, index=4 (none), base=ESP=4
        // 00 100 100 = 0x24
        memory.write8(codeAddr, 0x24);
        
        uint32_t addr = decoder.getEffectiveAddress32(modrm);
        REQUIRE(addr == (0x4000 << 4) + 0x20000);
    }
}
