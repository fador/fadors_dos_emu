#include "test_framework.hpp"
#include "hw/DMA8237.hpp"

using namespace fador::hw;

TEST_CASE("Hardware: DMA8237", "[HW][DMA]") {
    DMA8237 dma;

    SECTION("Initial State & Master Clear (0x0D)") {
        // Just writing to Master Clear should not crash and should reset flip-flop
        dma.write8(0x0D, 0x00);

        // After Master Clear, flip-flop is reset (LSB first)
        dma.write8(0x00, 0x34); // CH0 Base Address LSB
        dma.write8(0x00, 0x12); // CH0 Base Address MSB

        // Read back to verify
        // Read resets flip flop on every call in DMA8237? Wait, let's reset it to be sure
        dma.write8(0x0C, 0x00); // Clear Byte Pointer
        uint8_t lsb = dma.read8(0x00);
        uint8_t msb = dma.read8(0x00);
        REQUIRE(lsb == 0x34);
        REQUIRE(msb == 0x12);

        // Status should be 0
        REQUIRE(dma.read8(0x08) == 0x00);
    }

    SECTION("Base Address and Count Registers (0x00-0x07) & Clear Byte Pointer (0x0C)") {
        dma.write8(0x0C, 0x00); // Reset flip-flop

        // CH1 Base Count (port 0x03)
        dma.write8(0x03, 0x56); // LSB
        dma.write8(0x03, 0x78); // MSB

        dma.write8(0x0C, 0x00); // Reset flip-flop
        uint8_t count_lsb = dma.read8(0x03);
        uint8_t count_msb = dma.read8(0x03);

        REQUIRE(count_lsb == 0x56);
        REQUIRE(count_msb == 0x78);
        REQUIRE(dma.getChannelCount(1) == 0x7856);
    }

    SECTION("Page Registers (0x81-0x83)") {
        // CH1 Page is port 0x83
        dma.write8(0x83, 0xAB);
        // CH2 Page is port 0x81
        dma.write8(0x81, 0xCD);
        // CH3 Page is port 0x82
        dma.write8(0x82, 0xEF);

        REQUIRE(dma.read8(0x83) == 0xAB);
        REQUIRE(dma.read8(0x81) == 0xCD);
        REQUIRE(dma.read8(0x82) == 0xEF);

        dma.write8(0x0C, 0x00);
        dma.write8(0x02, 0x34); // CH1 Addr LSB
        dma.write8(0x02, 0x12); // CH1 Addr MSB

        REQUIRE(dma.getChannelAddress(1) == 0xAB1234);

        dma.write8(0x0C, 0x00);
        dma.write8(0x04, 0x78); // CH2 Addr LSB
        dma.write8(0x04, 0x56); // CH2 Addr MSB
        REQUIRE(dma.getChannelAddress(2) == 0xCD5678);

        dma.write8(0x0C, 0x00);
        dma.write8(0x06, 0xBC); // CH3 Addr LSB
        dma.write8(0x06, 0x9A); // CH3 Addr MSB
        REQUIRE(dma.getChannelAddress(3) == 0xEF9ABC);
    }

    SECTION("Mode Register (0x0B) & Mask Registers") {
        // Single Mask Register
        dma.write8(0x0A, 0x05); // Set mask for CH1 (0x04 | 0x01)

        // Write Multiple Mask Register
        dma.write8(0x0F, 0x0F); // Mask all

        // Mode Register
        dma.write8(0x0B, 0x12); // CH2 (0x02), Auto-init (0x10)
        REQUIRE(dma.isAutoInit(2) == true);
        REQUIRE(dma.isAutoInit(1) == false);

        dma.write8(0x0B, 0x21); // CH1 (0x01), Decrement mode (0x20)
        REQUIRE(dma.isAutoInit(1) == false);
    }

    SECTION("Acknowledge Transfer - Increment") {
        dma.write8(0x0C, 0x00);
        dma.write8(0x00, 0x00); // CH0 Addr LSB
        dma.write8(0x00, 0x10); // CH0 Addr MSB (0x1000)
        dma.write8(0x01, 0x02); // CH0 Count LSB
        dma.write8(0x01, 0x00); // CH0 Count MSB (2)
        dma.write8(0x80, 0x00); // Dummy page

        dma.write8(0x0B, 0x00); // CH0, Increment mode, No Auto-Init

        REQUIRE(dma.getChannelCount(0) == 2);

        bool tc1 = dma.acknowledgeTransfer(0, 1);
        REQUIRE(tc1 == false);
        REQUIRE(dma.getChannelCount(0) == 1);
        REQUIRE(dma.getChannelAddress(0) == 0x1001); // Incremented
        REQUIRE(dma.read8(0x08) == 0x00); // Status

        bool tc2 = dma.acknowledgeTransfer(0, 1);
        REQUIRE(tc2 == false);
        REQUIRE(dma.getChannelCount(0) == 0);
        REQUIRE(dma.getChannelAddress(0) == 0x1002);

        bool tc3 = dma.acknowledgeTransfer(0, 1);
        REQUIRE(tc3 == true); // Reached terminal count
        REQUIRE(dma.getChannelCount(0) == 0xFFFF); // Wraps around
        REQUIRE(dma.getChannelAddress(0) == 0x1003);
        REQUIRE(dma.read8(0x08) == 0x01); // Status TC bit for CH0 set
    }

    SECTION("Acknowledge Transfer - Decrement & Auto-Init") {
        dma.write8(0x0C, 0x00);
        dma.write8(0x02, 0x00); // CH1 Addr LSB
        dma.write8(0x02, 0x20); // CH1 Addr MSB (0x2000)
        dma.write8(0x03, 0x01); // CH1 Count LSB
        dma.write8(0x03, 0x00); // CH1 Count MSB (1)
        dma.write8(0x83, 0x00); // CH1 Page

        dma.write8(0x0B, 0x31); // CH1 (0x01), Auto-init (0x10) | Decrement (0x20)

        bool tc1 = dma.acknowledgeTransfer(1, 1);
        REQUIRE(tc1 == false);
        REQUIRE(dma.getChannelCount(1) == 0);
        REQUIRE(dma.getChannelAddress(1) == 0x1FFF); // Decremented

        bool tc2 = dma.acknowledgeTransfer(1, 1);
        REQUIRE(tc2 == true); // Reached terminal count

        // Auto-initialized back to base values
        REQUIRE(dma.getChannelCount(1) == 1);
        REQUIRE(dma.getChannelAddress(1) == 0x2000);
    }
}
