#include "test_framework.hpp"
#include "hw/DOS.hpp"
#include "cpu/CPU.hpp"
#include "memory/MemoryBus.hpp"

using namespace fador::hw;
using namespace fador::cpu;
using namespace fador::memory;

TEST_CASE("DOS: Drive Information", "[DOS][Drive]") {
    CPU cpu;
    MemoryBus memory;
    DOS dos(cpu, memory);

    dos.initialize();

    SECTION("Select and Get Drive") {
        // Get Default (Initial should be 2 for C:)
        cpu.setReg8(AH, 0x19);
        dos.handleInterrupt(0x21);
        REQUIRE(cpu.getReg8(AL) == 2);

        // Select Drive 0 (A:)
        cpu.setReg8(AH, 0x0E);
        cpu.setReg8(DL, 0);
        dos.handleInterrupt(0x21);
        REQUIRE(cpu.getReg8(AL) >= 1);

        // Verify it changed
        cpu.setReg8(AH, 0x19);
        dos.handleInterrupt(0x21);
        REQUIRE(cpu.getReg8(AL) == 0);
    }

    SECTION("Get Free Disk Space") {
        // Get space for default drive
        cpu.setReg8(AH, 0x36);
        cpu.setReg8(DL, 0);
        dos.handleInterrupt(0x21);

        REQUIRE(!(cpu.getEFLAGS() & FLAG_CARRY));
        REQUIRE(cpu.getReg16(AX) == 32);     // sectors per cluster
        REQUIRE(cpu.getReg16(BX) == 32768);  // free clusters
        REQUIRE(cpu.getReg16(CX) == 512);    // bytes per sector
        REQUIRE(cpu.getReg16(DX) == 65535);  // total clusters
    }
}
