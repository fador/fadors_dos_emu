#include "test_framework.hpp"
#include "hw/IOBus.hpp"
#include "hw/PIC8259.hpp"
#include "hw/PIT8254.hpp"
#include "hw/KeyboardController.hpp"
#include <chrono>

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

    SECTION("Repeated IRQ edges survive acknowledge and EOI") {
        pic.write8(0x20, 0x11); // ICW1
        pic.write8(0x21, 0x20); // ICW2: Vector 0x20
        pic.write8(0x21, 0x04); // ICW3
        pic.write8(0x21, 0x01); // ICW4
        pic.write8(0x21, 0x00); // Unmask all

        pic.raiseIRQ(0);
        pic.raiseIRQ(0);

        REQUIRE(pic.getPendingInterrupt() == 0x20);

        pic.acknowledgeInterrupt();
        REQUIRE(pic.getPendingInterrupt() == -1);

        pic.write8(0x20, 0x20); // EOI
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

    SECTION("Realtime pulse accumulation keeps every elapsed IRQ0 pulse") {
        pit.write8(0x43, 0x36); // Ch0, LSB/MSB, Mode 3
        pit.write8(0x40, 0x50);
        pit.write8(0x40, 0xC3); // Reload = 50000

        pit.advanceTime(std::chrono::milliseconds(126));

        REQUIRE(pit.checkPendingIRQ0());
        REQUIRE(pit.checkPendingIRQ0());
        REQUIRE(pit.checkPendingIRQ0());
        REQUIRE(!pit.checkPendingIRQ0());
    }

    SECTION("PIT8254 Accuracy and Sub-tick Remainder") {
        // Set 100Hz (Reload = 1193182 / 100 = 11932)
        pit.write8(0x43, 0x36);
        uint16_t reload = 11932;
        pit.write8(0x40, reload & 0xFF);
        pit.write8(0x40, reload >> 8);

        // Advance 1 second
        pit.advanceTime(std::chrono::seconds(1));

        int ticks = 0;
        while (pit.checkPendingIRQ0()) ticks++;

        // 1193181.818 / 11932 = 99.998... so should be 99 or 100
        REQUIRE(ticks >= 99);
        REQUIRE(ticks <= 101);

        // Sub-tick accumulation: Advance 100ms in 1ms steps
        pit.write8(0x43, 0x36);
        pit.write8(0x40, 0x00);
        pit.write8(0x40, 0x00); // 65536 reload (~54.9ms per tick)

        int totalTicks = 0;
        for (int i = 0; i < 100; ++i) {
            pit.advanceTime(std::chrono::milliseconds(1));
            while (pit.checkPendingIRQ0()) totalTicks++;
        }
        REQUIRE(totalTicks >= 1);
        REQUIRE(totalTicks <= 2);
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

    SECTION("Port 0x61 PPI Port B Refresh Bit") {
        uint8_t r1 = kbd.read8(0x61);
        bool toggled = false;
        uint8_t last = r1 & 0x10;
        for (int i = 0; i < 10; ++i) {
            uint8_t val = kbd.read8(0x61) & 0x10;
            if (val != last) {
                toggled = true;
                break;
            }
        }
        REQUIRE(toggled);
    }
}
