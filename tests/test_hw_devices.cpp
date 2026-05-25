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

TEST_CASE("Hardware: Keyboard Controller status register isolation", "[HW][KBD]") {
    KeyboardController kbd;

    // Verify that popKey() (BIOS buffer) does NOT affect the hardware
    // status register bit 0 (Output Buffer Full).  These are independent
    // buffers — the hardware buffer is for port 0x60/INT 09h, the BIOS
    // buffer is for INT 16h.
    SECTION("popKey does not clear hardware status bit") {
        // Push a scancode to the hardware buffer (sets status bit 0)
        kbd.pushScancode(0x1E); // 'A' make code
        REQUIRE(kbd.read8(0x64) & 0x01); // Status: Output Buffer Full

        // Push a key to the BIOS buffer (INT 16h)
        kbd.pushKey('a', 0x1E);

        // Pop from BIOS buffer — must NOT clear hardware status bit 0
        auto [ascii, sc] = kbd.popKey();
        REQUIRE(ascii == 'a');
        REQUIRE(sc == 0x1E);
        REQUIRE(kbd.read8(0x64) & 0x01); // Hardware buffer still has data

        // Drain hardware buffer
        uint8_t hw = kbd.read8(0x60);
        REQUIRE(hw == 0x1E);
        REQUIRE(!(kbd.read8(0x64) & 0x01)); // Now both empty
    }

    SECTION("Hardware buffer empty returns last scancode, not 0") {
        // When hardware buffer is empty, reading port 0x60 returns
        // the last scancode (bus hold behavior), not 0.
        uint8_t empty = kbd.read8(0x60);
        REQUIRE(empty == 0x00); // Initial state: 0

        kbd.pushScancode(0x2C); // 'Z' make code
        uint8_t z = kbd.read8(0x60);
        REQUIRE(z == 0x2C);

        // Buffer now empty, should return last value (0x2C)
        uint8_t stale = kbd.read8(0x60);
        REQUIRE(stale == 0x2C); // Stale, not 0
    }
}

TEST_CASE("Hardware: Keyboard Controller buffer limits", "[HW][KBD]") {
    KeyboardController kbd;

    SECTION("Hardware scancode buffer respects max size") {
        // Fill beyond kMaxHWScanBuffer (64) — oldest entries dropped
        for (int i = 0; i < 80; ++i) {
            kbd.pushScancode(static_cast<uint8_t>(i & 0xFF));
        }
        // Buffer should have at most 64 entries; reading them all
        int count = 0;
        while (kbd.read8(0x64) & 0x01) {
            kbd.read8(0x60);
            ++count;
        }
        REQUIRE(count <= 64);
    }

    SECTION("BIOS key buffer respects max size") {
        // Fill beyond kMaxKeyBuffer (32) — oldest entries dropped
        for (int i = 0; i < 50; ++i) {
            kbd.pushKey(static_cast<uint8_t>('A' + (i % 26)),
                        static_cast<uint8_t>(0x1E + (i % 26)));
        }
        int count = 0;
        while (kbd.hasKey()) {
            kbd.popKey();
            ++count;
        }
        REQUIRE(count <= 32);
    }
}
