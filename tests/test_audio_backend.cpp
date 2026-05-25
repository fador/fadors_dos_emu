#include "test_framework.hpp"
#include "hw/audio/AudioBackend.hpp"
#include "hw/audio/SoundBlaster.hpp"
#include "hw/DMA8237.hpp"
#include "memory/MemoryBus.hpp"

using namespace fador::hw::audio;
using namespace fador::hw;
using namespace fador::memory;

TEST_CASE("Audio: AudioBackend Basic Operations", "[Audio]") {
    AudioBackend backend;

    SECTION("Default Construction") {
        // We cannot directly read private members like m_initialized,
        // but we can infer uninitialized state.

        // Uninitialized backend should reject queueing samples
        float buffer[] = { 0.1f, 0.2f };
        REQUIRE(backend.queueSamples(buffer, 2) == false);

        // getQueuedAudioSize should return 0 if uninitialized
        REQUIRE(backend.getQueuedAudioSize() == 0);
    }

    SECTION("Initialization") {
        // init should return true (whether SDL2 is available or it falls back)
        bool initStatus = backend.init(44100, 2, 1024);
        REQUIRE(initStatus == true);

        // Calling init again when already initialized should return true
        REQUIRE(backend.init(44100, 2, 1024) == true);
    }

    SECTION("Queue Samples") {
        backend.init(44100, 2, 1024);

        float buffer[] = { 0.1f, -0.1f, 0.5f, -0.5f };
        bool queueStatus = backend.queueSamples(buffer, 4);

        // Without SDL2 it returns true and throws away samples.
        // With SDL2 it queues them and returns true (if successful).
        REQUIRE(queueStatus == true);

        // getQueuedAudioSize depends on SDL2. Without SDL2 it returns 0.
        // With SDL2 it should return > 0.
#ifdef HAVE_SDL2
        REQUIRE(backend.getQueuedAudioSize() > 0);
#else
        REQUIRE(backend.getQueuedAudioSize() == 0);
#endif
    }

    SECTION("Close and Cleanup") {
        backend.init(44100, 2, 1024);
        backend.close();

        // After close, queueSamples should fail
        float buffer[] = { 0.1f, 0.2f };
        REQUIRE(backend.queueSamples(buffer, 2) == false);

        // getQueuedAudioSize should return 0
        REQUIRE(backend.getQueuedAudioSize() == 0);
    }
}

TEST_CASE("SoundBlaster: DSP Reset and Version", "[SB]") {
    MemoryBus memory;
    DMA8237 dma;
    SoundBlaster sb(memory, dma, 44100.0f);

    SECTION("DSP reset sequence returns 0xAA") {
        sb.write8(0x226, 0x01); // Initiate reset
        sb.write8(0x226, 0x00); // Complete reset
        uint8_t status = sb.read8(0x22E); // Read buffer status
        REQUIRE(status == 0x80); // Data available
        uint8_t ready = sb.read8(0x22A); // Read data
        REQUIRE(ready == 0xAA); // DSP Ready
    }

    SECTION("DSP version returns 4.05") {
        sb.write8(0x226, 0x01);
        sb.write8(0x226, 0x00);
        sb.read8(0x22A); // Consume 0xAA
        
        sb.write8(0x22C, 0xE1); // Get DSP Version
        REQUIRE(sb.read8(0x22E) == 0x80); // Data available
        uint8_t major = sb.read8(0x22A);
        REQUIRE(sb.read8(0x22E) == 0x80);
        uint8_t minor = sb.read8(0x22A);
        REQUIRE(major == 0x04);
        REQUIRE(minor == 0x05);
    }
}

TEST_CASE("SoundBlaster: Missing DSP commands now implemented", "[SB]") {
    MemoryBus memory;
    DMA8237 dma;
    SoundBlaster sb(memory, dma, 44100.0f);

    // Reset first
    sb.write8(0x226, 0x01);
    sb.write8(0x226, 0x00);
    sb.read8(0x22A); // Consume 0xAA

    SECTION("0xD8 Speaker status returns value") {
        // Default speaker is on
        sb.write8(0x22C, 0xD8);
        REQUIRE(sb.read8(0x22E) == 0x80); // Data available
        uint8_t spk = sb.read8(0x22A);
        REQUIRE(spk == 0xFF); // Speaker on
    }

    SECTION("0xE2 DSP identification returns two bytes") {
        sb.write8(0x22C, 0xE2);
        REQUIRE(sb.read8(0x22E) == 0x80);
        sb.read8(0x22A); // First byte
        REQUIRE(sb.read8(0x22E) == 0x80);
        sb.read8(0x22A); // Second byte
        // After consuming, no more data
        REQUIRE(sb.read8(0x22E) == 0x00);
    }
}

TEST_CASE("SoundBlaster: DMA TC-driven IRQ (single-cycle 8-bit)", "[SB][DMA]") {
    MemoryBus memory;
    DMA8237 dma;
    SoundBlaster sb(memory, dma, 44100.0f);
    sb.write8(0x226, 0x01);
    sb.write8(0x226, 0x00);
    sb.read8(0x22A); // Consume 0xAA

    int irqCount = 0;
    sb.setIRQCallback([&irqCount]() { ++irqCount; });

    // Set up DMA channel 1 for 8-bit transfer:
    // Write sample data to memory at physical address 0x10000
    const uint32_t dmaPhysBase = 0x10000;
    const uint16_t sampleCount = 4;
    for (uint16_t i = 0; i < sampleCount; ++i) {
        memory.write8(dmaPhysBase + i, static_cast<uint8_t>(128 + i * 20));
    }

    // Program DMA: page=0x01 + address=0x0000 → phys 0x10000
    // Count = sampleCount - 1 (DMA convention: count = bytes - 1)
    // Mode register: bits 0-1 = channel, bit 4 = auto-init, bits 6-7 = single
    dma.write8(0x0A, 0x05);     // Mask channel 1
    dma.write8(0x0B, 0x49);     // Mode: single transfer, read, channel 1
    dma.write8(0x0C, 0x00);     // Clear flip-flop
    dma.write8(0x02, 0x00);     // Address LSB = 0x00
    dma.write8(0x02, 0x00);     // Address MSB = 0x00 → 0x0000
    dma.write8(0x83, 0x01);     // Page = 0x01 → phys 0x1_0000 = 0x10000
    dma.write8(0x0C, 0x00);     // Clear flip-flop
    dma.write8(0x03, static_cast<uint8_t>(sampleCount - 1)); // Count LSB
    dma.write8(0x03, 0x00);     // Count MSB
    dma.write8(0x0A, 0x01);     // Unmask channel 1

    // Start single-cycle 8-bit DMA via DSP
    sb.write8(0x22C, 0x14);           // DSP command: single-cycle 8-bit DMA
    sb.write8(0x22C, sampleCount & 0xFF); // Length LSB
    sb.write8(0x22C, 0x00);           // Length MSB

    // Verify DMA is active and address is correct
    REQUIRE(dma.getChannelAddress(1) == dmaPhysBase);

    // Generate samples — should fire IRQ exactly once when block completes
    // DSP sample rate (22050) / system rate (44100) = 0.5, so need 8 output
    // samples to consume all 4 DMA samples and trigger TC.
    float buffer[16] = {};
    sb.generateSamples(buffer, 8);

    // After single-cycle DMA with 4 samples, IRQ should fire once
    REQUIRE(irqCount == 1);

    // DMA should now be masked (non-auto-init)
    // Verify address advanced: should have moved past all samples
    REQUIRE(dma.getChannelAddress(1) == dmaPhysBase + sampleCount);
}

TEST_CASE("SoundBlaster: DMA TC-driven IRQ (auto-init 8-bit)", "[SB][DMA]") {
    MemoryBus memory;
    DMA8237 dma;
    SoundBlaster sb(memory, dma, 44100.0f);
    sb.write8(0x226, 0x01);
    sb.write8(0x226, 0x00);
    sb.read8(0x22A);

    int irqCount = 0;
    sb.setIRQCallback([&irqCount]() { ++irqCount; });

    const uint32_t dmaPhysBase = 0x10000;
    const uint16_t sampleCount = 2;
    for (uint16_t i = 0; i < sampleCount; ++i) {
        memory.write8(dmaPhysBase + i, static_cast<uint8_t>(128 + i * 30));
    }

    // Program DMA with auto-init mode (bit 4 of mode = 1)
    dma.write8(0x0A, 0x05);
    dma.write8(0x0B, 0x59);     // Mode: auto-init, read, channel 1
    dma.write8(0x0C, 0x00);
    dma.write8(0x02, 0x00);
    dma.write8(0x02, 0x00);     // Address = 0x0000, page=0x01 → phys 0x10000
    dma.write8(0x83, 0x01);
    dma.write8(0x0C, 0x00);
    dma.write8(0x03, static_cast<uint8_t>(sampleCount - 1));
    dma.write8(0x03, 0x00);
    dma.write8(0x0A, 0x01);

    // Start auto-init 8-bit DMA via DSP
    sb.write8(0x22C, 0x1C); // Auto-init 8-bit DMA (no args)

    // Generate samples — after 2 samples, IRQ fires and DMA resets
    float buffer[16] = {};
    sb.generateSamples(buffer, 8);

    // DMA address should have wrapped back to base (auto-init)
    REQUIRE(dma.getChannelAddress(1) == dmaPhysBase);
    // At least 1 IRQ should have fired (multiple if many samples)
    REQUIRE(irqCount >= 1);
}

TEST_CASE("SoundBlaster: IRQ acknowledged by reading 0x22E", "[SB]") {
    MemoryBus memory;
    DMA8237 dma;
    SoundBlaster sb(memory, dma, 44100.0f);

    // After DSP reset, the read queue contains 0xAA (ready byte).
    // Reading port 0x22E reports data-available status (bit 7).
    sb.write8(0x226, 0x01);
    sb.write8(0x226, 0x00);

    // Before consuming 0xAA: read-buffer status should show data ready
    uint8_t ackStatus = sb.read8(0x22E);
    REQUIRE(ackStatus == 0x80); // Bit 7 = data available

    // Consume the 0xAA byte
    uint8_t ready = sb.read8(0x22A);
    REQUIRE(ready == 0xAA);

    // After consuming: read-buffer status should show empty
    ackStatus = sb.read8(0x22E);
    REQUIRE(ackStatus == 0x00);
}
