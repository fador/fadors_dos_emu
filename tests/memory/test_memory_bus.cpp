#include "../test_framework.hpp"
#include "memory/MemoryBus.hpp"
#include "hw/VGAController.hpp"

using namespace fador::memory;
using namespace fador::hw;

TEST_CASE("MemoryBus: Basic Initialization", "[MemoryBus]") {
    MemoryBus bus;

    SECTION("Memory size is correct") {
        REQUIRE(MemoryBus::MEMORY_SIZE == 64 * 1024 * 1024);
    }

    SECTION("IVT and BDA are zero initialized") {
        for (uint32_t i = 0; i < 0x500; ++i) {
            REQUIRE(bus.read8(i) == 0);
        }
    }

    SECTION("A20 gate is disabled by default") {
        REQUIRE(bus.isA20Enabled() == false);
    }
}

TEST_CASE("MemoryBus: Read and Write 8/16/32-bit", "[MemoryBus]") {
    MemoryBus bus;

    SECTION("8-bit Read/Write") {
        bus.write8(0x1000, 0xAB);
        REQUIRE(bus.read8(0x1000) == 0xAB);
    }

    SECTION("16-bit Read/Write") {
        bus.write16(0x1000, 0xABCD);
        REQUIRE(bus.read16(0x1000) == 0xABCD);
        REQUIRE(bus.read8(0x1000) == 0xCD);
        REQUIRE(bus.read8(0x1001) == 0xAB);
    }

    SECTION("32-bit Read/Write") {
        bus.write32(0x1000, 0x12345678);
        REQUIRE(bus.read32(0x1000) == 0x12345678);
        REQUIRE(bus.read16(0x1000) == 0x5678);
        REQUIRE(bus.read16(0x1002) == 0x1234);
        REQUIRE(bus.read8(0x1000) == 0x78);
        REQUIRE(bus.read8(0x1001) == 0x56);
        REQUIRE(bus.read8(0x1002) == 0x34);
        REQUIRE(bus.read8(0x1003) == 0x12);
    }
}

TEST_CASE("MemoryBus: A20 Gate Handling", "[MemoryBus]") {
    MemoryBus bus;

    // Address 0x100000 (1MB mark)
    uint32_t addrHigh = 0x100000;
    uint32_t addrLow  = 0x000000;

    SECTION("With A20 disabled, addresses wrap around at 1MB") {
        bus.setA20(false);
        bus.write8(addrLow, 0x11);

        // Write to 1MB mark, should wrap to 0 due to 20-bit mask (0xFFFFF)
        bus.write8(addrHigh, 0x22);

        REQUIRE(bus.read8(addrLow) == 0x22); // Value was overwritten
        REQUIRE(bus.read8(addrHigh) == 0x22); // Reads from wrapped address too
    }

    SECTION("With A20 enabled, addresses do not wrap") {
        bus.setA20(true);
        bus.write8(addrLow, 0x11);
        bus.write8(addrHigh, 0x22);

        REQUIRE(bus.read8(addrLow) == 0x11);
        REQUIRE(bus.read8(addrHigh) == 0x22);
    }
}

TEST_CASE("MemoryBus: Out of Bounds Access", "[MemoryBus]") {
    MemoryBus bus;
    bus.setA20(true); // Enable A20 to access beyond 1MB normally

    uint32_t outOfBounds = MemoryBus::MEMORY_SIZE + 0x1000;

    SECTION("Out of bounds read returns 0xFF") {
        REQUIRE(bus.read8(outOfBounds) == 0xFF);
        REQUIRE(bus.read16(outOfBounds) == 0xFFFF);
        REQUIRE(bus.read32(outOfBounds) == 0xFFFFFFFF);
    }

    SECTION("Out of bounds write does not crash") {
        // Just verify it doesn't throw or crash
        bus.write8(outOfBounds, 0x12);
        bus.write16(outOfBounds, 0x1234);
        bus.write32(outOfBounds, 0x12345678);
    }
}

TEST_CASE("MemoryBus: Direct Access", "[MemoryBus]") {
    MemoryBus bus;

    SECTION("Direct access inside bounds returns pointer") {
        uint8_t* ptr = bus.directAccess(0x1000);
        REQUIRE(ptr != nullptr);

        *ptr = 0x99;
        REQUIRE(bus.read8(0x1000) == 0x99);
    }

    SECTION("Direct access out of bounds returns nullptr") {
        uint8_t* ptr = bus.directAccess(MemoryBus::MEMORY_SIZE + 0x1000);
        REQUIRE(ptr == nullptr);
    }
}
