#include "test_framework.hpp"
#include "hw/PIT8254.hpp"

using namespace fador::hw;

TEST_CASE("Hardware: PIT8254", "[HW][PIT]") {
    PIT8254 pit;

    SECTION("Initial State") {
        // Should default to 0xFFFF reload, LSB/MSB mode.
        // Reading without configuring should return LSB then MSB of 0xFFFF.
        uint8_t lsb = pit.read8(0x40);
        uint8_t msb = pit.read8(0x40);
        REQUIRE(lsb == 0xFF);
        REQUIRE(msb == 0xFF);
        REQUIRE(pit.checkPendingIRQ0() == false);
    }

    SECTION("Write/Read Access Modes (LSB/MSB)") {
        // Setup Channel 0 for LSB then MSB
        pit.write8(0x43, 0x30); // Channel 0 (00), Access LSB/MSB (11), Mode 0 (000)

        pit.write8(0x40, 0x34); // LSB
        pit.write8(0x40, 0x12); // MSB

        // Latch Channel 0
        pit.write8(0x43, 0x00); // Channel 0 (00), Latch (00)

        uint8_t lsb = pit.read8(0x40);
        uint8_t msb = pit.read8(0x40);

        REQUIRE(lsb == 0x34);
        REQUIRE(msb == 0x12);
    }

    SECTION("Write/Read Access Modes (LSB only)") {
        // Setup Channel 1 for LSB only
        pit.write8(0x43, 0x50); // Channel 1 (01), Access LSB (01), Mode 0 (000)

        pit.write8(0x41, 0x78); // LSB

        // Latch Channel 1
        pit.write8(0x43, 0x40);

        uint8_t lsb = pit.read8(0x41);
        uint8_t second_read = pit.read8(0x41); // Should still return LSB

        REQUIRE(lsb == 0x78);
        REQUIRE(second_read == 0x78);
    }

    SECTION("Write/Read Access Modes (MSB only)") {
        // Setup Channel 2 for MSB only
        pit.write8(0x43, 0xA0); // Channel 2 (10), Access MSB (10), Mode 0 (000)

        pit.write8(0x42, 0x56); // MSB

        // Latch Channel 2
        pit.write8(0x43, 0x80);

        uint8_t msb = pit.read8(0x42);
        uint8_t second_read = pit.read8(0x42); // Should still return MSB

        REQUIRE(msb == 0x56);
        REQUIRE(second_read == 0x56);
    }

    SECTION("Timer Ticks and IRQ0 Generation") {
        // Channel 0 is IRQ0. Setup to Mode 0, LSB/MSB
        pit.write8(0x43, 0x30);

        // Reload = 0x0005
        pit.write8(0x40, 0x05);
        pit.write8(0x40, 0x00);

        REQUIRE(pit.checkPendingIRQ0() == false);

        // CYCLES_PER_PIT_TICK = 16.
        // We need 5 ticks. 5 * 16 = 80 cycles.
        pit.addCycles(16 * 4); // 4 ticks
        REQUIRE(pit.checkPendingIRQ0() == false);

        pit.addCycles(16); // 1 tick -> total 5 ticks
        REQUIRE(pit.checkPendingIRQ0() == true);

        // Check IRQ0 is cleared after reading
        REQUIRE(pit.checkPendingIRQ0() == false);
    }

    SECTION("Timer Countdown Latch") {
        pit.write8(0x43, 0x30); // Channel 0, LSB/MSB
        pit.write8(0x40, 0x0A); // Reload = 10 (0x0A)
        pit.write8(0x40, 0x00); //

        pit.addCycles(16 * 3); // 3 ticks, count should be 7

        pit.write8(0x43, 0x00); // Latch Channel 0

        // Even if we add more cycles, latched value should stay 7
        pit.addCycles(16 * 2);

        uint8_t lsb = pit.read8(0x40);
        uint8_t msb = pit.read8(0x40);

        REQUIRE(lsb == 0x07);
        REQUIRE(msb == 0x00);

        // Next read unlatches, so let's latch again and read current count
        pit.write8(0x43, 0x00); // Latch again
        lsb = pit.read8(0x40);
        msb = pit.read8(0x40);

        // It was 10. We ticked 3, latched 7. We ticked 2 more. Count should be 5.
        REQUIRE(lsb == 0x05);
        REQUIRE(msb == 0x00);
    }
}
