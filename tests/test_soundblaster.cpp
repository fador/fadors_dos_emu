#include "test_framework.hpp"
#include "hw/audio/SoundBlaster.hpp"
#include "hw/DMA8237.hpp"
#include "memory/MemoryBus.hpp"

using namespace fador::hw;
using namespace fador::hw::audio;

TEST_CASE("Hardware: SoundBlaster", "[HW][Audio]") {
    fador::memory::MemoryBus memory;
    DMA8237 dma;
    SoundBlaster sb(memory, dma, 44100.0f);

    SECTION("Initialization and Basic Ports") {
        // Test DSP Write Status (0xC) - should not be busy initially (bit 7 clear)
        REQUIRE((sb.read8(0x220 + 0xC) & 0x80) == 0x00);

        // Test DSP Data Available Status (0xE) - should be empty initially (bit 7 clear)
        REQUIRE((sb.read8(0x220 + 0xE) & 0x80) == 0x00);

        // Read from empty DSP Data port
        REQUIRE(sb.read8(0x220 + 0xA) == 0xFF);
    }

    SECTION("DSP Reset Sequence") {
        // Write 1 to Reset port (0x6)
        sb.write8(0x220 + 0x6, 1);

        // Data available should still be empty
        REQUIRE((sb.read8(0x220 + 0xE) & 0x80) == 0x00);

        // Write 0 to Reset port (0x6)
        sb.write8(0x220 + 0x6, 0);

        // Data available should now be set (0xAA ready byte)
        REQUIRE((sb.read8(0x220 + 0xE) & 0x80) == 0x80);

        // Read Data port should return 0xAA
        REQUIRE(sb.read8(0x220 + 0xA) == 0xAA);

        // Queue should be empty again
        REQUIRE((sb.read8(0x220 + 0xE) & 0x80) == 0x00);
    }

    SECTION("DSP Commands - Get Version (0xE1)") {
        // Ensure ready state
        sb.write8(0x220 + 0x6, 1);
        sb.write8(0x220 + 0x6, 0);
        sb.read8(0x220 + 0xA); // Clear 0xAA

        // Write Command 0xE1 (Get DSP Version)
        sb.write8(0x220 + 0xC, 0xE1);

        // Should have 2 bytes available
        REQUIRE((sb.read8(0x220 + 0xE) & 0x80) == 0x80);

        // Version for Sound Blaster Pro is 3.02
        REQUIRE(sb.read8(0x220 + 0xA) == 0x03); // Major version
        REQUIRE(sb.read8(0x220 + 0xA) == 0x02); // Minor version

        // Queue empty
        REQUIRE((sb.read8(0x220 + 0xE) & 0x80) == 0x00);
    }

    SECTION("DSP Commands - Speaker Control (0xD1/0xD3)") {
        // This is mainly testing that the commands are processed without crashing
        // Since speakerOn is internal state, we can't observe it directly easily
        // without generating samples.
        sb.write8(0x220 + 0xC, 0xD1); // Speaker ON
        sb.write8(0x220 + 0xC, 0xD3); // Speaker OFF
    }

    SECTION("DSP Commands - Set Time Constant (0x40)") {
        sb.write8(0x220 + 0xC, 0x40); // Command
        sb.write8(0x220 + 0xC, 0xCC); // Data (204)
        // Rate = 1000000 / (256 - 204) = 1000000 / 52 = ~19230 Hz
        // We verify it doesn't crash. Internal state changes.
    }

    SECTION("DSP Commands - Direct DAC Output (0x10)") {
        sb.write8(0x220 + 0xC, 0xD1); // Speaker ON

        sb.write8(0x220 + 0xC, 0x10); // Direct output command
        sb.write8(0x220 + 0xC, 0xFF); // Max value (255)

        // Generate a sample and verify it's not silence
        float buffer[2] = {0.0f, 0.0f};
        sb.generateSamples(buffer, 1);

        // 255 -> (255 - 128) / 128 = 127/128 = ~0.99
        // generated value is accumulated, last sample * 0.2f
        // 0.99 * 0.2 = ~0.198
        REQUIRE(buffer[0] > 0.1f);
        REQUIRE(buffer[1] > 0.1f);
    }

    SECTION("DSP Commands - Single-cycle DMA Output (0x14)") {
        // Setup DMA CH1 for transfer
        dma.write8(0x0C, 0x00); // Clear byte pointer
        dma.write8(0x02, 0x00); // CH1 Base Addr LSB
        dma.write8(0x02, 0x10); // CH1 Base Addr MSB (0x1000)
        dma.write8(0x03, 0x01); // CH1 Count LSB (1)
        dma.write8(0x03, 0x00); // CH1 Count MSB (0) => 2 bytes
        dma.write8(0x83, 0x00); // CH1 Page 0
        dma.write8(0x0B, 0x49); // CH1, Single transfer, Read, Increment

        // Put some data in memory at 0x1000
        memory.write8(0x1000, 0xC0); // ~192 value
        memory.write8(0x1001, 0x40); // ~64 value

        bool irqTriggered = false;
        sb.setIRQCallback([&]() { irqTriggered = true; });

        sb.write8(0x220 + 0xC, 0xD1); // Speaker ON

        // Set sample rate to match system rate to process 1 sample per step
        sb.write8(0x220 + 0xC, 0x40); // Time constant
        sb.write8(0x220 + 0xC, 256 - (1000000 / 44100)); // ~44100 Hz

        sb.write8(0x220 + 0xC, 0x14); // 8-bit output single-cycle DMA
        sb.write8(0x220 + 0xC, 0x01); // Length LSB (1 => 2 bytes)
        sb.write8(0x220 + 0xC, 0x00); // Length MSB

        float buffer[4] = {0.0f};

        // Process first sample
        sb.generateSamples(buffer, 1);

        // 0xC0 (192) -> (192 - 128) / 128 = 64/128 = 0.5
        // 0.5 * 0.2f = 0.1f
        REQUIRE(buffer[0] > 0.05f);
        REQUIRE(buffer[0] < 0.15f);
        REQUIRE(irqTriggered == false);

        // Process second sample
        sb.generateSamples(buffer + 2, 1);

        // 0x40 (64) -> (64 - 128) / 128 = -64/128 = -0.5
        // -0.5 * 0.2f = -0.1f
        REQUIRE(buffer[2] < -0.05f);
        REQUIRE(buffer[2] > -0.15f);

        // IRQ should be triggered after reading 2 bytes
        REQUIRE(irqTriggered == true);
    }
}
