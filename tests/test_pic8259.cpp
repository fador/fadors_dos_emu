#include "test_framework.hpp"
#include "hw/PIC8259.hpp"

using namespace fador::hw;

TEST_CASE("PIC8259: Initialization and Configuration", "[PIC]") {
    PIC8259 pic(true); // Master PIC
    PIC8259 slave(false); // Slave PIC

    SECTION("Master PIC Default Vectors") {
        // Without ICW2, the default in the constructor for master is 0x08
        // Need to push it through a dummy init to test
        pic.write8(0x20, 0x11); // ICW1
        pic.write8(0x21, 0x08); // ICW2: Vector 0x08
        pic.write8(0x21, 0x04); // ICW3
        pic.write8(0x21, 0x01); // ICW4

        pic.raiseIRQ(0);
        REQUIRE(pic.getPendingInterrupt() == 0x08);
    }

    SECTION("Slave PIC Default Vectors") {
        slave.write8(0xA0, 0x11); // ICW1
        slave.write8(0xA1, 0x70); // ICW2: Vector 0x70
        slave.write8(0xA1, 0x02); // ICW3
        slave.write8(0xA1, 0x01); // ICW4

        slave.raiseIRQ(0);
        REQUIRE(slave.getPendingInterrupt() == 0x70);
    }

    SECTION("ICW sequence overrides base vector") {
        pic.write8(0x20, 0x11); // ICW1
        pic.write8(0x21, 0x20); // ICW2: Vector 0x20
        pic.write8(0x21, 0x04); // ICW3
        pic.write8(0x21, 0x01); // ICW4

        pic.raiseIRQ(0);
        REQUIRE(pic.getPendingInterrupt() == 0x20);
        pic.acknowledgeInterrupt();

        pic.raiseIRQ(3);
        REQUIRE(pic.getPendingInterrupt() == 0x23);
    }
}

TEST_CASE("PIC8259: IRQ Masking (IMR)", "[PIC]") {
    PIC8259 pic(true);

    // Initialize to base 0x20
    pic.write8(0x20, 0x11); // ICW1
    pic.write8(0x21, 0x20); // ICW2
    pic.write8(0x21, 0x04); // ICW3
    pic.write8(0x21, 0x01); // ICW4

    SECTION("All unmasked initially") {
        REQUIRE(pic.read8(0x21) == 0x00);
    }

    SECTION("Mask specific IRQs") {
        pic.write8(0x21, 0x05); // Mask IRQ 0 (bit 0) and IRQ 2 (bit 2)
        REQUIRE(pic.read8(0x21) == 0x05);

        // Raise masked IRQ
        pic.raiseIRQ(0);
        REQUIRE(pic.getPendingInterrupt() == -1); // Masked

        // Raise unmasked IRQ
        pic.raiseIRQ(1);
        REQUIRE(pic.getPendingInterrupt() == 0x21); // Unmasked, should be pending
    }

    SECTION("Raise before mask") {
        pic.raiseIRQ(0);
        REQUIRE(pic.getPendingInterrupt() == 0x20);

        pic.write8(0x21, 0x01); // Mask IRQ 0
        REQUIRE(pic.getPendingInterrupt() == -1); // Now hidden by mask

        pic.write8(0x21, 0x00); // Unmask IRQ 0
        REQUIRE(pic.getPendingInterrupt() == 0x20); // Request should still be there
    }
}

TEST_CASE("PIC8259: Priority and Fully Nested Mode", "[PIC]") {
    PIC8259 pic(true);

    // Initialize to base 0x20
    pic.write8(0x20, 0x11); // ICW1
    pic.write8(0x21, 0x20); // ICW2
    pic.write8(0x21, 0x04); // ICW3
    pic.write8(0x21, 0x01); // ICW4

    SECTION("Lower IRQ has higher priority") {
        pic.raiseIRQ(3);
        pic.raiseIRQ(1);

        // Even though 3 was raised first, 1 should be returned
        REQUIRE(pic.getPendingInterrupt() == 0x21);
    }

    SECTION("In-service IRQ blocks lower priority ones") {
        pic.raiseIRQ(2);
        REQUIRE(pic.getPendingInterrupt() == 0x22);

        pic.acknowledgeInterrupt(); // ISR bit 2 is now set

        // Attempt to raise a lower priority IRQ (higher number)
        pic.raiseIRQ(4);
        // It should NOT be pending because IRQ 2 is in service
        REQUIRE(pic.getPendingInterrupt() == -1);

        // Attempt to raise a higher priority IRQ (lower number)
        pic.raiseIRQ(1);
        // It SHOULD be pending because it's higher priority than the one in service
        REQUIRE(pic.getPendingInterrupt() == 0x21);
    }
}

TEST_CASE("PIC8259: End of Interrupt (EOI)", "[PIC]") {
    PIC8259 pic(true);

    // Initialize to base 0x20
    pic.write8(0x20, 0x11); // ICW1
    pic.write8(0x21, 0x20); // ICW2
    pic.write8(0x21, 0x04); // ICW3
    pic.write8(0x21, 0x01); // ICW4

    SECTION("Simple EOI clears highest priority in-service bit") {
        pic.raiseIRQ(2);
        pic.acknowledgeInterrupt(); // ISR bit 2 is set

        pic.raiseIRQ(5);
        // Not pending because 2 is in service
        REQUIRE(pic.getPendingInterrupt() == -1);

        // Send non-specific EOI (OCW2)
        pic.write8(0x20, 0x20); // 0x20 is Non-specific EOI

        // Now IRQ 5 should be pending because 2 is no longer in service
        REQUIRE(pic.getPendingInterrupt() == 0x25);
    }
}

TEST_CASE("PIC8259: IRR Read", "[PIC]") {
    PIC8259 pic(true);

    // The emulator currently only supports reading IRR (Interrupt Request Register) on port 0x20
    pic.raiseIRQ(1);
    pic.raiseIRQ(4);

    REQUIRE(pic.read8(0x20) == 0x12); // Bit 1 and Bit 4 set
}
