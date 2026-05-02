#include "../test_framework.hpp"
#include "hw/VGAController.hpp"
#include "memory/MemoryBus.hpp"

using namespace fador::hw;
using namespace fador::memory;

TEST_CASE("Hardware: VGA Controller", "[HW][VGA]") {
    MemoryBus memory;
    VGAController vga(memory);

    SECTION("DAC state and index behavior") {
        // Test initial DAC state
        REQUIRE(vga.read8(0x3C7) == 0x03);

        // Test writing to DAC Write index (0x3C8)
        vga.write8(0x3C8, 0x10);
        REQUIRE(vga.read8(0x3C8) == 0x10);

        // Test DAC color cycle (0x3C9) for writing
        vga.write8(0x3C9, 0x11); // R
        vga.write8(0x3C9, 0x22); // G
        vga.write8(0x3C9, 0x33); // B

        // Verify index incremented
        REQUIRE(vga.read8(0x3C8) == 0x11);

        // Test DAC color cycle for reading
        vga.write8(0x3C7, 0x10); // DAC Read index

        REQUIRE(vga.read8(0x3C9) == 0x11); // R
        REQUIRE(vga.read8(0x3C9) == 0x22); // G
        REQUIRE(vga.read8(0x3C9) == 0x33); // B

        // Note: read8(0x3C7) always returns 0x03 for DAC state, so we can't test read index directly
    }

    SECTION("Sequencer registers") {
        // Map mask register
        vga.write8(0x3C4, 0x02);
        vga.write8(0x3C5, 0x0A);
        REQUIRE(vga.read8(0x3C4) == 0x02);
        REQUIRE(vga.read8(0x3C5) == 0x0A);
        REQUIRE(vga.getMapMask() == 0x0A);

        // Chain-4 register
        vga.write8(0x3C4, 0x04);
        vga.write8(0x3C5, 0x08); // set chain-4
        REQUIRE(vga.read8(0x3C5) == 0x08);
        REQUIRE(vga.isChain4() == true);

        vga.write8(0x3C5, 0x00); // clear chain-4
        REQUIRE(vga.read8(0x3C5) == 0x00);
        REQUIRE(vga.isChain4() == false);
    }

    SECTION("Graphics Controller registers") {
        // Read map select register
        vga.write8(0x3CE, 0x04);
        vga.write8(0x3CF, 0x02);
        REQUIRE(vga.read8(0x3CE) == 0x04);
        REQUIRE(vga.read8(0x3CF) == 0x02);
        REQUIRE(vga.getReadMapSelect() == 0x02);

        vga.write8(0x3CE, 0x00);
        vga.write8(0x3CF, 0x0A);
        REQUIRE(vga.read8(0x3CF) == 0x0A);

        vga.write8(0x3CE, 0x03);
        vga.write8(0x3CF, 0x12);
        REQUIRE(vga.read8(0x3CF) == 0x12);

        vga.write8(0x3CE, 0x08);
        vga.write8(0x3CF, 0xF0);
        REQUIRE(vga.read8(0x3CF) == 0xF0);
    }

    SECTION("CRTC registers") {
        vga.write8(0x3D4, 0x0C);
        vga.write8(0x3D5, 0x12);
        REQUIRE(vga.read8(0x3D4) == 0x0C);
        REQUIRE(vga.read8(0x3D5) == 0x12);

        vga.write8(0x3D4, 0x0D);
        vga.write8(0x3D5, 0x34);
        REQUIRE(vga.read8(0x3D5) == 0x34);
    }

    SECTION("Retrace Simulation") {
        // Simulate reading 0x3DA
        // In the implementation, m_retraceReadCount increments.
        // It's display state for the first 32 reads, then retrace for 4.

        // Let's read 32 times. All should be display mode (0x00).
        for (int i = 0; i < 31; ++i) { // after 31 reads count is 31, % 36 = 31 < 32
            REQUIRE(vga.read8(0x3DA) == 0x00);
        }

        // 32nd read: count becomes 32, % 36 = 32 >= 32 (retrace mode)
        REQUIRE(vga.read8(0x3DA) == 0x09);
        REQUIRE(vga.read8(0x3DA) == 0x09);
        REQUIRE(vga.read8(0x3DA) == 0x09);
        REQUIRE(vga.read8(0x3DA) == 0x09);

        // 36th read: count becomes 36, % 36 = 0 < 32 (display mode again)
        REQUIRE(vga.read8(0x3DA) == 0x00);
    }

    SECTION("Plane-aware VRAM access") {
        SECTION("Chain-4 mode") {
            // Enable chain-4
            vga.write8(0x3C4, 0x04);
            vga.write8(0x3C5, 0x08);

            // Write 0xAA at offset 0
            vga.planeWrite8(0x0000, 0xAA);
            REQUIRE(vga.planeRead8(0x0000) == 0xAA);
            REQUIRE(vga.readPlane(0, 0) == 0xAA); // plane 0, offset 0

            // Write 0xBB at offset 1
            vga.planeWrite8(0x0001, 0xBB);
            REQUIRE(vga.planeRead8(0x0001) == 0xBB);
            REQUIRE(vga.readPlane(1, 0) == 0xBB); // plane 1, offset 0

            // Write 0xCC at offset 5
            vga.planeWrite8(0x0005, 0xCC);
            REQUIRE(vga.planeRead8(0x0005) == 0xCC);
            REQUIRE(vga.readPlane(1, 1) == 0xCC); // plane 1, offset 1
        }

        SECTION("Mode-X / unchained mode") {
            // Disable chain-4
            vga.write8(0x3C4, 0x04);
            vga.write8(0x3C5, 0x00);

            // Set map mask to plane 0 and 2 (binary 0101 = 5)
            vga.write8(0x3C4, 0x02);
            vga.write8(0x3C5, 0x05);

            // Write 0xDD at offset 0
            vga.planeWrite8(0x0000, 0xDD);

            // Check that it wrote to planes 0 and 2 at offset 0
            REQUIRE(vga.readPlane(0, 0) == 0xDD);
            REQUIRE(vga.readPlane(1, 0) == 0x00); // untouched
            REQUIRE(vga.readPlane(2, 0) == 0xDD);
            REQUIRE(vga.readPlane(3, 0) == 0x00); // untouched

            // Set read map select to plane 2
            vga.write8(0x3CE, 0x04);
            vga.write8(0x3CF, 0x02);

            // Read from offset 0
            REQUIRE(vga.planeRead8(0x0000) == 0xDD);

            // Set read map select to plane 1
            vga.write8(0x3CE, 0x04);
            vga.write8(0x3CF, 0x01);

            // Read from offset 0
            REQUIRE(vga.planeRead8(0x0000) == 0x00);
        }

        SECTION("Write mode 0 applies bit mask against latched data") {
            vga.write8(0x3C4, 0x04);
            vga.write8(0x3C5, 0x00);
            vga.write8(0x3C4, 0x02);
            vga.write8(0x3C5, 0x01);

            vga.planeWrite8(0x0000, 0xF0);
            REQUIRE(vga.planeRead8(0x0000) == 0xF0);

            vga.write8(0x3CE, 0x08);
            vga.write8(0x3CF, 0x0F);
            vga.planeWrite8(0x0000, 0xAA);

            REQUIRE(vga.readPlane(0, 0) == 0xFA);
        }

        SECTION("Write mode 1 copies the latched source byte") {
            vga.write8(0x3C4, 0x04);
            vga.write8(0x3C5, 0x00);

            for (int plane = 0; plane < 4; ++plane) {
                vga.write8(0x3C4, 0x02);
                vga.write8(0x3C5, 1 << plane);
                vga.planeWrite8(0x0000, static_cast<uint8_t>(0x11 * (plane + 1)));
            }

            vga.write8(0x3CE, 0x04);
            vga.write8(0x3CF, 0x00);
            REQUIRE(vga.planeRead8(0x0000) == 0x11);

            vga.write8(0x3C4, 0x02);
            vga.write8(0x3C5, 0x0F);
            vga.write8(0x3CE, 0x05);
            vga.write8(0x3CF, 0x01);
            vga.planeWrite8(0x0001, 0xFF);

            REQUIRE(vga.readPlane(0, 1) == 0x11);
            REQUIRE(vga.readPlane(1, 1) == 0x22);
            REQUIRE(vga.readPlane(2, 1) == 0x33);
            REQUIRE(vga.readPlane(3, 1) == 0x44);
        }

        SECTION("Write mode 2 expands host low nibble across planes") {
            vga.write8(0x3C4, 0x04);
            vga.write8(0x3C5, 0x00);
            vga.write8(0x3C4, 0x02);
            vga.write8(0x3C5, 0x0F);

            vga.planeWrite8(0x0002, 0x55);
            REQUIRE(vga.planeRead8(0x0002) == 0x55);

            vga.write8(0x3CE, 0x05);
            vga.write8(0x3CF, 0x02);
            vga.write8(0x3CE, 0x08);
            vga.write8(0x3CF, 0x0F);
            vga.planeWrite8(0x0002, 0x05);

            REQUIRE(vga.readPlane(0, 2) == 0x5F);
            REQUIRE(vga.readPlane(1, 2) == 0x50);
            REQUIRE(vga.readPlane(2, 2) == 0x5F);
            REQUIRE(vga.readPlane(3, 2) == 0x50);
        }

        SECTION("Write mode 3 uses rotated host data as the write mask") {
            vga.write8(0x3C4, 0x04);
            vga.write8(0x3C5, 0x00);
            vga.write8(0x3C4, 0x02);
            vga.write8(0x3C5, 0x03);

            vga.planeWrite8(0x0003, 0x0F);
            REQUIRE(vga.planeRead8(0x0003) == 0x0F);

            vga.write8(0x3CE, 0x00);
            vga.write8(0x3CF, 0x01);
            vga.write8(0x3CE, 0x08);
            vga.write8(0x3CF, 0xF0);
            vga.write8(0x3CE, 0x05);
            vga.write8(0x3CF, 0x03);
            vga.planeWrite8(0x0003, 0xCC);

            REQUIRE(vga.readPlane(0, 3) == 0xCF);
            REQUIRE(vga.readPlane(1, 3) == 0x0F);
        }
    }
}
