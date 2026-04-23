#include "test_framework.hpp"
#include "hw/PIC8259.hpp"
#include "hw/PIT8254.hpp"
#include "hw/KeyboardController.hpp"

using namespace fador::hw;

TEST_CASE("Hardware: PIC8259A", "[HW]") {
    PIC8259 pic(true); // Master

    SECTION("Initialization sequence") {
        pic.write8(0x20, 0x11); // ICW1
        pic.write8(0x21, 0x20); // ICW2: Vector 0x20
        pic.write8(0x21, 0x04); // ICW3
        pic.write8(0x21, 0x01); // ICW4
        
        REQUIRE(pic.getPendingInterrupt() == -1);
    }

    SECTION("IRQ Masking") {
        // Must initialize first to set vector base to 0x20
        pic.write8(0x20, 0x11); // ICW1
        pic.write8(0x21, 0x20); // ICW2: Vector 0x20
        pic.write8(0x21, 0x04); // ICW3
        pic.write8(0x21, 0x01); // ICW4

        pic.write8(0x21, 0xFE); // Mask all but IRQ 0
        pic.raiseIRQ(0);
        REQUIRE(pic.getPendingInterrupt() == 0x20);
        
        pic.raiseIRQ(1); // Masked
        REQUIRE(pic.getPendingInterrupt() == 0x20);
    }
}

TEST_CASE("Hardware: PIT8254", "[HW]") {
    PIT8254 pit;

    SECTION("Counter LSB/MSB access") {
        pit.write8(0x43, 0x36); // Ch0, LSB/MSB, Mode 3
        pit.write8(0x40, 0x00);
        pit.write8(0x40, 0x10); // Reload = 0x1000
    }
}

TEST_CASE("Hardware: Keyboard Controller", "[HW]") {
    KeyboardController kbd;

    SECTION("Self-test and buffer") {
        kbd.write8(0x64, 0xAA); // Self-test
        REQUIRE(kbd.read8(0x64) & 0x01); // Status: Output Buffer Full
        REQUIRE(kbd.read8(0x60) == 0x55); // Response
        REQUIRE(!(kbd.read8(0x64) & 0x01)); // Status: Buffer empty
    }
}
