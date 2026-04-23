#include "test_framework.hpp"
#include "hw/IOBus.hpp"
#include <map>

using namespace fador::hw;

class TestIODevice : public IODevice {
public:
    std::map<uint16_t, uint8_t> memory;
    int read8_count = 0;
    int write8_count = 0;
    int read16_count = 0;
    int write16_count = 0;
    int read32_count = 0;
    int write32_count = 0;

    uint8_t read8(uint16_t port) override {
        read8_count++;
        return memory[port];
    }

    void write8(uint16_t port, uint8_t value) override {
        write8_count++;
        memory[port] = value;
    }

    uint16_t read16(uint16_t port) override {
        read16_count++;
        return IODevice::read16(port);
    }

    void write16(uint16_t port, uint16_t value) override {
        write16_count++;
        IODevice::write16(port, value);
    }

    uint32_t read32(uint16_t port) override {
        read32_count++;
        return IODevice::read32(port);
    }

    void write32(uint16_t port, uint32_t value) override {
        write32_count++;
        IODevice::write32(port, value);
    }

    void reset_counts() {
        read8_count = 0;
        write8_count = 0;
        read16_count = 0;
        write16_count = 0;
        read32_count = 0;
        write32_count = 0;
    }
};

TEST_CASE("IOBus: Device Registration", "[IOBus]") {
    IOBus bus;
    TestIODevice dev1;
    TestIODevice dev2;

    bus.registerDevice(0x10, 0x1F, &dev1);
    bus.registerDevice(0x20, 0x20, &dev2);

    SECTION("Read from registered device") {
        dev1.memory[0x10] = 0xAA;
        dev1.memory[0x1F] = 0xBB;
        dev2.memory[0x20] = 0xCC;

        REQUIRE(bus.read8(0x10) == 0xAA);
        REQUIRE(bus.read8(0x1F) == 0xBB);
        REQUIRE(bus.read8(0x20) == 0xCC);
        REQUIRE(dev1.read8_count == 2);
        REQUIRE(dev2.read8_count == 1);
    }

    SECTION("Write to registered device") {
        bus.write8(0x15, 0xDD);
        bus.write8(0x20, 0xEE);

        REQUIRE(dev1.memory[0x15] == 0xDD);
        REQUIRE(dev2.memory[0x20] == 0xEE);
        REQUIRE(dev1.write8_count == 1);
        REQUIRE(dev2.write8_count == 1);
    }

    SECTION("Unmapped ports return 0xFF") {
        REQUIRE(bus.read8(0x0F) == 0xFF);
        REQUIRE(bus.read8(0x21) == 0xFF);
        REQUIRE(bus.read8(0xFFFF) == 0xFF);
        REQUIRE(dev1.read8_count == 0);
        REQUIRE(dev2.read8_count == 0);
    }

    SECTION("Writes to unmapped ports are ignored") {
        bus.write8(0x0F, 0xAA);
        REQUIRE(dev1.write8_count == 0);
        REQUIRE(dev2.write8_count == 0);
    }
}

TEST_CASE("IOBus: Multi-byte Operations on Mapped Ports", "[IOBus]") {
    IOBus bus;
    TestIODevice dev;
    bus.registerDevice(0x100, 0x10F, &dev);

    SECTION("16-bit operations") {
        bus.write16(0x100, 0x1234);
        REQUIRE(dev.write16_count == 1);
        REQUIRE(dev.write8_count == 2); // IODevice::write16 calls write8 twice
        REQUIRE(dev.memory[0x100] == 0x34);
        REQUIRE(dev.memory[0x101] == 0x12);

        dev.reset_counts();

        uint16_t val = bus.read16(0x100);
        REQUIRE(val == 0x1234);
        REQUIRE(dev.read16_count == 1);
        REQUIRE(dev.read8_count == 2);
    }

    SECTION("32-bit operations") {
        bus.write32(0x104, 0xDEADBEEF);
        REQUIRE(dev.write32_count == 1);
        REQUIRE(dev.write16_count == 2);
        REQUIRE(dev.write8_count == 4);
        REQUIRE(dev.memory[0x104] == 0xEF);
        REQUIRE(dev.memory[0x105] == 0xBE);
        REQUIRE(dev.memory[0x106] == 0xAD);
        REQUIRE(dev.memory[0x107] == 0xDE);

        dev.reset_counts();

        uint32_t val = bus.read32(0x104);
        REQUIRE(val == 0xDEADBEEF);
        REQUIRE(dev.read32_count == 1);
        REQUIRE(dev.read16_count == 2);
        REQUIRE(dev.read8_count == 4);
    }
}

TEST_CASE("IOBus: Multi-byte Operations with Unmapped/Cross-boundary Ports", "[IOBus]") {
    IOBus bus;
    TestIODevice dev;
    // Mapped port is 0x100 only
    bus.registerDevice(0x100, 0x100, &dev);

    SECTION("16-bit read unmapped lower, mapped upper") {
        // Port 0xFF is unmapped, Port 0x100 is mapped
        dev.memory[0x100] = 0xAA;
        uint16_t val = bus.read16(0xFF);
        // IOBus::read16(0xFF) -> no device at 0xFF
        // Fallback: read8(0xFF) | (read8(0x100) << 8)
        // read8(0xFF) -> 0xFF
        // read8(0x100) -> 0xAA
        REQUIRE(val == 0xAAFF);
        REQUIRE(dev.read8_count == 1);
    }

    SECTION("16-bit write unmapped lower, mapped upper") {
        bus.write16(0xFF, 0xAABB);
        // IOBus::write16(0xFF, 0xAABB) -> no device at 0xFF
        // Fallback: write8(0xFF, 0xBB), write8(0x100, 0xAA)
        REQUIRE(dev.memory[0x100] == 0xAA);
        REQUIRE(dev.write8_count == 1);
    }

    SECTION("32-bit read crossing boundary") {
        // Mapped: 0x100
        // Unmapped: 0xFE, 0xFF, 0x101
        dev.memory[0x100] = 0x55;
        uint32_t val = bus.read32(0xFE);
        // read32(0xFE) -> no device at 0xFE
        // Fallback: read16(0xFE) | (read16(0x100) << 16)
        // read16(0xFE) -> read8(0xFE) | read8(0xFF) << 8 -> 0xFFFF
        // read16(0x100) -> device->read16(0x100) -> read8(0x100) | read8(0x101) << 8
        // read8(0x100) -> 0x55
        // read8(0x101) -> returns 0
        REQUIRE(val == 0x0055FFFF);
        REQUIRE(dev.read16_count == 1);
        REQUIRE(dev.read8_count == 2);
    }

    SECTION("32-bit write crossing boundary") {
        bus.write32(0xFE, 0x11223344);
        // write32(0xFE) -> no device at 0xFE
        // Fallback: write16(0xFE, 0x3344), write16(0x100, 0x1122)
        // write16(0xFE) -> write8(0xFE, 0x44), write8(0xFF, 0x33) -> unmapped
        // write16(0x100) -> device->write16(0x100, 0x1122) -> write8(0x100, 0x22), write8(0x101, 0x11)
        REQUIRE(dev.memory[0x100] == 0x22);
        REQUIRE(dev.memory[0x101] == 0x11);
        REQUIRE(dev.write16_count == 1);
        REQUIRE(dev.write8_count == 2);
    }
}
