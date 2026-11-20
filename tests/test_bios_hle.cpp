#include "test_framework.hpp"
#include "../src/cpu/CPU.hpp"
#include "../src/memory/MemoryBus.hpp"
#include "../src/hw/KeyboardController.hpp"
#include "../src/hw/PIT8254.hpp"
#include "../src/hw/PIC8259.hpp"
#include "../src/hw/BIOS.hpp"

using namespace fador;

struct DummyKbd : hw::KeyboardController { DummyKbd() : KeyboardController() {} };
struct DummyPit : hw::PIT8254 { DummyPit() : PIT8254() {} };
struct DummyPic : hw::PIC8259 { DummyPic() : PIC8259(true) {} };

TEST_CASE("BIOS HLE INT 11h/12h/14h/15h/17h/2Fh/1Bh-1Fh basics", "[bios][hle]") {
    memory::MemoryBus mem;
    cpu::CPU cpu;
    DummyKbd kbd;
    DummyPit pit;
    DummyPic pic;
    hw::BIOS bios(cpu, mem, kbd, pit, pic);

    cpu.setReg8(cpu::AH, 0); cpu.setReg8(cpu::AL, 0); cpu.setReg16(cpu::AX, 0);

    SECTION("INT 11h returns the equipment word") {
        bios.initialize();
        REQUIRE(bios.handleInterrupt(0x11));
        REQUIRE(cpu.getReg16(cpu::AX) == 0x002F); // no serial / parallel ports
        bios.setPortAddresses({0x3F8, 0x2F8, 0, 0}, {0x378, 0, 0});
        REQUIRE(bios.handleInterrupt(0x11));
        REQUIRE(cpu.getReg16(cpu::AX) == (0x002F | (2 << 9) | (1 << 14)));
        REQUIRE(mem.read16(0x400) == 0x3F8);
        REQUIRE(mem.read16(0x402) == 0x2F8);
        REQUIRE(mem.read16(0x404) == 0);
        REQUIRE(mem.read16(0x408) == 0x378);
        // Ports are listed without gaps: COM2 alone becomes the first entry.
        bios.setPortAddresses({0, 0x2F8, 0, 0}, {0, 0, 0});
        REQUIRE(mem.read16(0x400) == 0x2F8);
        REQUIRE(mem.read16(0x402) == 0);
        REQUIRE(mem.read16(0x408) == 0);
        REQUIRE(mem.read16(0x410) == (0x002F | (1 << 9)));
        REQUIRE(mem.read8(0x47C) == 1);  // COM time-outs
        REQUIRE(mem.read8(0x478) == 20); // printer time-outs
    }
    SECTION("INT 12h returns 640KB memory") {
        REQUIRE(bios.handleInterrupt(0x12));
        REQUIRE(cpu.getReg16(cpu::AX) == 640);
    }
    SECTION("INT 14h without a port leaves the registers unchanged") {
        cpu.setReg16(cpu::AX, 0x0341);
        cpu.setReg16(cpu::DX, 0);
        REQUIRE(bios.handleInterrupt(0x14));
        REQUIRE(cpu.getReg16(cpu::AX) == 0x0341);
    }
    SECTION("INT 15h returns not supported") {
        cpu.setReg8(cpu::AH, 0);
        REQUIRE(bios.handleInterrupt(0x15));
        REQUIRE(cpu.getReg8(cpu::AH) == 0x86);
        REQUIRE(cpu.getEFLAGS() & cpu::FLAG_CARRY);
    }
    SECTION("INT 17h without a port leaves the registers unchanged") {
        cpu.setReg16(cpu::AX, 0x0241);
        cpu.setReg16(cpu::DX, 0);
        REQUIRE(bios.handleInterrupt(0x17));
        REQUIRE(cpu.getReg16(cpu::AX) == 0x0241);
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

TEST_CASE("BIOS HLE INT 09h keyboard IRQ with status check", "[bios][hle][kbd]") {
    memory::MemoryBus mem;
    cpu::CPU cpu;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);

    SECTION("INT 09h with data in hardware buffer reads scancode") {
        // Push a scancode to the hardware buffer
        kbd.pushScancode(0x1E); // 'A' make code
        REQUIRE(kbd.read8(0x64) & 0x01); // Status: Output Buffer Full

        // Handle INT 09h via HLE — should read the scancode and drain buffer
        REQUIRE(bios.handleInterrupt(0x09));

        // Buffer should now be empty
        REQUIRE(!(kbd.read8(0x64) & 0x01));
    }

    SECTION("INT 09h with empty hardware buffer does not crash") {
        // No scancode pushed — buffer is empty
        REQUIRE(!(kbd.read8(0x64) & 0x01));

        // handleKeyboardIRQ should check status and NOT read stale data
        REQUIRE(bios.handleInterrupt(0x09));

        // Buffer should still be empty, no side effects
        REQUIRE(!(kbd.read8(0x64) & 0x01));
    }

    SECTION("INT 09h with multiple scancodes drains one per IRQ") {
        kbd.pushScancode(0x1E); // 'A'
        kbd.pushScancode(0x30); // 'B'

        // First IRQ: drains first scancode
        REQUIRE(kbd.read8(0x64) & 0x01);
        REQUIRE(bios.handleInterrupt(0x09));
        // Second scancode still pending
        REQUIRE(kbd.read8(0x64) & 0x01);

        // Second IRQ: drains second scancode
        REQUIRE(bios.handleInterrupt(0x09));
        REQUIRE(!(kbd.read8(0x64) & 0x01));
    }
}
