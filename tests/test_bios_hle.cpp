#include "test_framework.hpp"
#include "../src/cpu/CPU.hpp"
#include "../src/memory/MemoryBus.hpp"
#include "../src/hw/KeyboardController.hpp"
#include "../src/hw/PIT8254.hpp"
#include "../src/hw/BIOS.hpp"

using namespace fador;

struct DummyKbd : hw::KeyboardController { DummyKbd() : KeyboardController() {} };
struct DummyPit : hw::PIT8254 { DummyPit() : PIT8254() {} };

TEST_CASE("BIOS HLE INT 11h/12h/14h/15h/17h/2Fh/1Bh-1Fh stubs work", "[bios][hle]") {
    memory::MemoryBus mem;
    cpu::CPU cpu;
    DummyKbd kbd;
    DummyPit pit;
    hw::BIOS bios(cpu, mem, kbd, pit);

    cpu.setReg8(cpu::AH, 0); cpu.setReg8(cpu::AL, 0); cpu.setReg16(cpu::AX, 0);

    SECTION("INT 11h returns plausible equipment list") {
        REQUIRE(bios.handleInterrupt(0x11));
        REQUIRE(cpu.getReg16(cpu::AX) == 0x002F);
    }
    SECTION("INT 12h returns 640KB memory") {
        REQUIRE(bios.handleInterrupt(0x12));
        REQUIRE(cpu.getReg16(cpu::AX) == 640);
    }
    SECTION("INT 14h returns no error") {
        cpu.setReg8(cpu::AH, 0xFF);
        REQUIRE(bios.handleInterrupt(0x14));
        REQUIRE((cpu.getReg8(cpu::AH) == 0 && !(cpu.getEFLAGS() & cpu::FLAG_CARRY)));
    }
    SECTION("INT 15h returns not supported") {
        cpu.setReg8(cpu::AH, 0);
        REQUIRE(bios.handleInterrupt(0x15));
        REQUIRE(cpu.getReg8(cpu::AH) == 0x86);
        REQUIRE(cpu.getEFLAGS() & cpu::FLAG_CARRY);
    }
    SECTION("INT 17h returns no error") {
        cpu.setReg8(cpu::AH, 0xFF);
        REQUIRE(bios.handleInterrupt(0x17));
        REQUIRE((cpu.getReg8(cpu::AH) == 0 && !(cpu.getEFLAGS() & cpu::FLAG_CARRY)));
    }
    SECTION("INT 2Fh returns not present") {
        cpu.setReg8(cpu::AL, 0xFF);
        REQUIRE(bios.handleInterrupt(0x2F));
        REQUIRE(cpu.getReg8(cpu::AL) == 0);
    }
    SECTION("INT 1Bh-1Fh are no-ops") {
        for (uint8_t v = 0x1B; v <= 0x1F; ++v) {
            REQUIRE(bios.handleInterrupt(v));
        }
    }
}
