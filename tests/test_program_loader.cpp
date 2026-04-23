#include "test_framework.hpp"
#include <fstream>
#include <cstdio>
#include <vector>
#include "cpu/CPU.hpp"
#include "memory/MemoryBus.hpp"
#include "memory/himem/HIMEM.hpp"
#include "hw/DOS.hpp"
#include "hw/ProgramLoader.hpp"

using namespace fador;

TEST_CASE("ProgramLoader Emulation", "[ProgramLoader]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::DOS dos(cpu, mem);
    memory::HIMEM* himem = dos.getHIMEM();
    hw::ProgramLoader loader(cpu, mem, himem);

    SECTION("COM File Loading - Valid File") {
        // Create a dummy .COM file (INT 20h)
        {
            std::ofstream ofs("dummy.com", std::ios::binary);
            uint8_t code[] = { 0xCD, 0x20 };
            ofs.write(reinterpret_cast<const char*>(code), sizeof(code));
        }

        REQUIRE(loader.loadCOM("dummy.com", 0x1000, "arg1 arg2"));

        // Verify PSP
        REQUIRE(mem.read8(0x10000) == 0xCD); // INT 20h
        REQUIRE(mem.read8(0x10001) == 0x20);

        // Verify Command Tail
        // Length of " arg1 arg2" is 10
        REQUIRE(mem.read8(0x10080) == 10);
        REQUIRE(mem.read8(0x10081) == ' ');
        REQUIRE(mem.read8(0x10082) == 'a');

        // Verify Code at 100h
        REQUIRE(mem.read8(0x10100) == 0xCD); // INT 20h
        REQUIRE(mem.read8(0x10101) == 0x20);

        // Verify CPU state
        REQUIRE(cpu.getSegReg(cpu::CS) == 0x1000);
        REQUIRE(cpu.getEIP() == 0x100);

        std::remove("dummy.com");
    }

    SECTION("COM File Loading - Invalid File") {
        REQUIRE(!loader.loadCOM("nonexistent.com", 0x1000));
    }

    SECTION("EXE File Loading - Valid MZ Header") {
        // Create a dummy .EXE file (MZ header + INT 20h)
        {
            std::ofstream ofs("dummy.exe", std::ios::binary);
            uint8_t exe[] = {
                'M', 'Z',        // Signature
                0x22, 0x00,      // Bytes in last page (34 bytes total)
                0x01, 0x00,      // Pages in file (1 page)
                0x00, 0x00,      // Relocations
                0x02, 0x00,      // Header size in paragraphs (32 bytes)
                0x00, 0x00,      // Minimum extra paragraphs
                0x00, 0x00,      // Maximum extra paragraphs
                0x00, 0x00,      // Initial SS
                0xFE, 0xFF,      // Initial SP
                0x00, 0x00,      // Checksum
                0x00, 0x00,      // Initial IP
                0x00, 0x00,      // Initial CS
                0x1C, 0x00,      // Relocation table offset
                0x00, 0x00,      // Overlay number
                0x00, 0x00, 0x00, 0x00, // Padding
                0xCD, 0x20       // INT 20h (code at offset 32)
            };
            ofs.write(reinterpret_cast<const char*>(exe), sizeof(exe));
        }

        REQUIRE(loader.loadEXE("dummy.exe", 0x1000, dos, "arg1"));

        // Verify PSP
        REQUIRE(mem.read8(0x10000) == 0xCD); // INT 20h

        // Command Tail length of " arg1" is 5
        REQUIRE(mem.read8(0x10080) == 5);
        REQUIRE(mem.read8(0x10081) == ' ');

        // Verify Code (Loaded at PSP + 10h = 0x1010)
        REQUIRE(mem.read8(0x10100) == 0xCD); // INT 20h
        REQUIRE(mem.read8(0x10101) == 0x20);

        // Verify CPU state
        REQUIRE(cpu.getSegReg(cpu::CS) == 0x1010);
        REQUIRE(cpu.getEIP() == 0x0000);
        REQUIRE(cpu.getSegReg(cpu::SS) == 0x1010);
        REQUIRE(cpu.getReg16(cpu::SP) == 0xFFFE);

        std::remove("dummy.exe");
    }

    SECTION("EXE File Loading - Invalid Signature") {
        {
            std::ofstream ofs("invalid.exe", std::ios::binary);
            uint8_t exe[] = {
                'M', 'A',        // Invalid Signature
                0x00, 0x00
            };
            ofs.write(reinterpret_cast<const char*>(exe), sizeof(exe));
        }

        REQUIRE(!loader.loadEXE("invalid.exe", 0x1000, dos));

        std::remove("invalid.exe");
    }
}
