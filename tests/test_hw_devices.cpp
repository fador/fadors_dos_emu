#include "test_framework.hpp"
#include "hw/IOBus.hpp"
#include "hw/PIC8259.hpp"
#include "hw/PIT8254.hpp"
#include "hw/KeyboardController.hpp"

using namespace fador::hw;

TEST_CASE("Hardware: IOBus and Generic Devices", "[HW]") {
    IOBus bus;
    
    class MockDevice : public IODevice {
    public:
        uint8_t val = 0;
        uint8_t read8(uint16_t) override { return val; }
        void write8(uint16_t, uint8_t v) override { val = v; }
    };

    MockDevice dev;
    bus.registerDevice(0x10, 0x10, &dev);

    SECTION("IOBus dispatching") {
        bus.write8(0x10, 0x42);
        REQUIRE(bus.read8(0x10) == 0x42);
        REQUIRE(dev.val == 0x42);
    }

    SECTION("Unmapped ports") {
        REQUIRE(bus.read8(0xFF) == 0xFF);
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
