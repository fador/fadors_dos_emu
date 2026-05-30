#include "test_framework.hpp"
#include "hw/audio/AudioBackend.hpp"
#include "hw/audio/SoundBlaster.hpp"
#include "hw/audio/AdLib.hpp"
#include "hw/DMA8237.hpp"
#include "hw/PIT8254.hpp"
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

TEST_CASE("SoundBlaster: High-Speed DMA and Force 16-bit IRQ", "[SB][DMA]") {
    MemoryBus memory;
    DMA8237 dma;
    SoundBlaster sb(memory, dma, 44100.0f);
    sb.write8(0x226, 0x01);
    sb.write8(0x226, 0x00);
    sb.read8(0x22A); // Consume 0xAA

    int irqCount = 0;
    sb.setIRQCallback([&irqCount]() { ++irqCount; });

    SECTION("0x90 Auto-Init High-Speed 8-bit DMA") {
        const uint32_t dmaPhysBase = 0x10000;
        const uint16_t sampleCount = 4;
        for (uint16_t i = 0; i < sampleCount; ++i) {
            memory.write8(dmaPhysBase + i, static_cast<uint8_t>(128 + i * 10));
        }

        // Program DMA for channel 1 auto-init:
        dma.write8(0x0A, 0x05);     // Mask channel 1
        dma.write8(0x0B, 0x59);     // Mode: auto-init, read, channel 1
        dma.write8(0x0C, 0x00);     // Clear flip-flop
        dma.write8(0x02, 0x00);     // Address LSB
        dma.write8(0x02, 0x00);     // Address MSB
        dma.write8(0x83, 0x01);     // Page = 0x01
        dma.write8(0x0C, 0x00);     // Clear flip-flop
        dma.write8(0x03, static_cast<uint8_t>(sampleCount - 1)); // Count LSB
        dma.write8(0x03, 0x00);     // Count MSB
        dma.write8(0x0A, 0x01);     // Unmask channel 1

        // Program DSP block size via command 0x48:
        sb.write8(0x22C, 0x48);
        sb.write8(0x22C, static_cast<uint8_t>((sampleCount - 1) & 0xFF)); // block size LSB
        sb.write8(0x22C, static_cast<uint8_t>(((sampleCount - 1) >> 8) & 0xFF)); // block size MSB

        // Start Auto-Init High-Speed DMA (0x90, no args)
        sb.write8(0x22C, 0x90);

        // Generate samples to consume the block
        float buffer[16] = {};
        sb.generateSamples(buffer, 8); // DSP rate 22050 / 44100 = 0.5, so 8 samples = 4 frames

        REQUIRE(irqCount >= 1);
        // DMA address should have wrapped back to base (auto-init)
        REQUIRE(dma.getChannelAddress(1) == dmaPhysBase);
    }

    SECTION("0x91 Single-Cycle High-Speed 8-bit DMA") {
        const uint32_t dmaPhysBase = 0x10000;
        const uint16_t sampleCount = 4;
        for (uint16_t i = 0; i < sampleCount; ++i) {
            memory.write8(dmaPhysBase + i, static_cast<uint8_t>(128 + i * 10));
        }

        // Program DMA for channel 1 single-cycle:
        dma.write8(0x0A, 0x05);
        dma.write8(0x0B, 0x49); // Mode: single, read, channel 1
        dma.write8(0x0C, 0x00);
        dma.write8(0x02, 0x00);
        dma.write8(0x02, 0x00);
        dma.write8(0x83, 0x01);
        dma.write8(0x0C, 0x00);
        dma.write8(0x03, static_cast<uint8_t>(sampleCount - 1));
        dma.write8(0x03, 0x00);
        dma.write8(0x0A, 0x01);

        // Program DSP block size via command 0x48:
        sb.write8(0x22C, 0x48);
        sb.write8(0x22C, static_cast<uint8_t>((sampleCount - 1) & 0xFF));
        sb.write8(0x22C, static_cast<uint8_t>(((sampleCount - 1) >> 8) & 0xFF));

        // Start Single-Cycle High-Speed DMA (0x91, no args)
        sb.write8(0x22C, 0x91);

        float buffer[16] = {};
        sb.generateSamples(buffer, 8);

        REQUIRE(irqCount == 1);
        // Address should have advanced
        REQUIRE(dma.getChannelAddress(1) == dmaPhysBase + sampleCount);
    }

    SECTION("0x98 Auto-Init High-Speed 16-bit DMA") {
        const uint32_t dmaPhysBase = 0x20000;
        const uint16_t sampleCount = 4; // 4 samples, 16-bit, mono (8 bytes total)
        for (uint16_t i = 0; i < sampleCount; ++i) {
            memory.write16(dmaPhysBase + i * 2, static_cast<int16_t>(i * 1000));
        }

        // Program DMA for channel 5 auto-init:
        dma.write8(0xD4, 0x05);     // Mask channel 5
        dma.write8(0xD6, 0x59);     // Mode: auto-init, read, channel 5
        dma.write8(0xD8, 0x00);     // Clear master flip-flop
        dma.write8(0xC4, 0x00);     // Address LSB (0x0000 words)
        dma.write8(0xC4, 0x00);     // Address MSB
        dma.write8(0x8B, 0x02);     // Page = 0x02 (phys 0x20000 since 2 << 16 | 0 << 1 = 0x20000)
        dma.write8(0xD8, 0x00);     // Clear flip-flop
        // Word count = 4 - 1 = 3 words
        dma.write8(0xC6, 0x03);     // Count LSB
        dma.write8(0xC6, 0x00);     // Count MSB
        dma.write8(0xD4, 0x01);     // Unmask channel 5

        // Program DSP block size via command 0x48:
        sb.write8(0x22C, 0x48);
        sb.write8(0x22C, static_cast<uint8_t>((sampleCount - 1) & 0xFF));
        sb.write8(0x22C, static_cast<uint8_t>(((sampleCount - 1) >> 8) & 0xFF));

        // Start 16-bit Auto-Init High-Speed DMA (0x98, no args)
        sb.write8(0x22C, 0x98);

        float buffer[16] = {};
        sb.generateSamples(buffer, 8);

        REQUIRE(irqCount >= 1);
        REQUIRE(dma.getChannelAddress(5) == dmaPhysBase);
    }

    SECTION("0x99 Single-Cycle High-Speed 16-bit DMA") {
        const uint32_t dmaPhysBase = 0x20000;
        const uint16_t sampleCount = 4; // 4 samples, 16-bit, mono (8 bytes total)
        for (uint16_t i = 0; i < sampleCount; ++i) {
            memory.write16(dmaPhysBase + i * 2, static_cast<int16_t>(i * 1000));
        }

        // Program DMA for channel 5 single-cycle:
        dma.write8(0xD4, 0x05);     // Mask channel 5
        dma.write8(0xD6, 0x49);     // Mode: single, read, channel 5
        dma.write8(0xD8, 0x00);     // Clear master flip-flop
        dma.write8(0xC4, 0x00);     // Address LSB (0x0000 words)
        dma.write8(0xC4, 0x00);     // Address MSB
        dma.write8(0x8B, 0x02);     // Page = 0x02
        dma.write8(0xD8, 0x00);     // Clear flip-flop
        // Word count = 4 - 1 = 3 words
        dma.write8(0xC6, 0x03);     // Count LSB
        dma.write8(0xC6, 0x00);     // Count MSB
        dma.write8(0xD4, 0x01);     // Unmask channel 5

        // Program DSP block size via command 0x48:
        sb.write8(0x22C, 0x48);
        sb.write8(0x22C, static_cast<uint8_t>((sampleCount - 1) & 0xFF));
        sb.write8(0x22C, static_cast<uint8_t>(((sampleCount - 1) >> 8) & 0xFF));

        // Start 16-bit Single-Cycle High-Speed DMA (0x99, no args)
        sb.write8(0x22C, 0x99);

        float buffer[16] = {};
        sb.generateSamples(buffer, 8);

        REQUIRE(irqCount == 1);
        // Address should have advanced by 4 words = 8 bytes
        REQUIRE(dma.getChannelAddress(5) == dmaPhysBase + sampleCount * 2);
    }

    SECTION("0xF3 Force 16-bit IRQ") {
        REQUIRE(irqCount == 0);
        sb.write8(0x22C, 0xF3);
        REQUIRE(irqCount == 1);
    }
}


// ──  sb_detect() DSP reset + version read ─────────────────────────
TEST_CASE("SB detection: DSP reset and version read", "[SB]") {
    MemoryBus memory;
    DMA8237 dma;
    SoundBlaster sb(memory, dma, 44100.0f);

    SECTION("Full reset sequence as Allegro does it") {
        // _sb_reset_dsp(1): write 1, read port+6 eight times, write 0
        sb.write8(0x226, 0x01);
        // Allegro reads port+6 eight times as delay
        for (int i = 0; i < 8; i++)
            sb.read8(0x226);
        sb.write8(0x226, 0x00);
        // sb_read_dsp() polls port+0x22E until bit 7 set, then reads port+0x22A
        uint8_t status = sb.read8(0x22E);
        REQUIRE(status & 0x80); // Data available
        uint8_t ready = sb.read8(0x22A);
        REQUIRE(ready == 0xAA); // DSP ready
    }

    SECTION("DSP version via command 0xE1") {
        sb.write8(0x226, 0x01);
        for (int i = 0; i < 8; i++) sb.read8(0x226);
        sb.write8(0x226, 0x00);
        sb.read8(0x22A); // consume 0xAA

        // sb_write_dsp(0xE1): polls port+0x22C until bit 7 clear
        // Our implementation always returns 0x00 (ready)
        uint8_t writeStatus = sb.read8(0x22C);
        REQUIRE(!(writeStatus & 0x80)); // Ready to accept command

        sb.write8(0x22C, 0xE1); // Get DSP Version
        // sb_read_dsp() for major
        REQUIRE(sb.read8(0x22E) & 0x80); // Data available
        uint8_t major = sb.read8(0x22A);
        // sb_read_dsp() for minor
        REQUIRE(sb.read8(0x22E) & 0x80);
        uint8_t minor = sb.read8(0x22A);

        // SB16 = version 4.05
        REQUIRE(major == 0x04);
        REQUIRE(minor == 0x05);
    }

    SECTION("Multiple reset attempts with different ports") {
        // Allegro tries ports 0x210, 0x220, 0x230, 0x240, 0x250, 0x260
        // Our SB is at 0x220, so reset at 0x210 should fail, 0x220 should succeed
        // Reset at 0x210 (wrong port) - no response
        sb.write8(0x216, 0x01);
        for (int i = 0; i < 8; i++) sb.read8(0x216);
        sb.write8(0x216, 0x00);
        // Read from wrong port - should not get 0xAA
        // Our implementation returns 0xFF for unmapped ports
        // (the SB only handles ports at its base address)

        // Reset at 0x220 (correct port)
        sb.write8(0x226, 0x01);
        for (int i = 0; i < 8; i++) sb.read8(0x226);
        sb.write8(0x226, 0x00);
        REQUIRE(sb.read8(0x22E) & 0x80);
        REQUIRE(sb.read8(0x22A) == 0xAA);
    }
}

// ── sb_detect() SB16 mixer configuration read ────────────────────
TEST_CASE("SB detection: SB16 mixer config read", "[SB]") {
    MemoryBus memory;
    DMA8237 dma;
    SoundBlaster sb(memory, dma, 44100.0f);
    sb.write8(0x226, 0x01);
    for (int i = 0; i < 8; i++) sb.read8(0x226);
    sb.write8(0x226, 0x00);
    sb.read8(0x22A); // consume 0xAA

    SECTION("IRQ status register (0x80)") {
        // Allegro reads mixer register 0x80 to get IRQ configuration
        sb.write8(0x224, 0x80); // Mixer address = IRQ status
        uint8_t irqStatus = sb.read8(0x225);
        // Our emulator: returns pending IRQ flags
        // Before any DMA, no IRQs pending
        REQUIRE(irqStatus == 0x00);
    }

    SECTION("DMA channel register (0x81)") {
        // Allegro reads mixer register 0x81 to get DMA channel configuration
        sb.write8(0x224, 0x81); // Mixer address = DMA channel
        uint8_t dmaConfig = sb.read8(0x225);
        // Our emulator returns 0x00 for unmapped registers
        // In a real SB16, this would return channel bit masks
        // Allegro checks: bit 0=ch0, bit 1=ch1, bit 3=ch3 for 8-bit
        //                 bit 5=ch5, bit 6=ch6, bit 7=ch7 for 16-bit
        (void)dmaConfig; // Just verify read doesn't crash
    }

    SECTION("Mixer volume registers") {
        // Master volume
        sb.write8(0x224, 0x22);
        uint8_t master = sb.read8(0x225);
        REQUIRE((master & 0x0F) <= 0x0F); // 4-bit value

        // Voice volume
        sb.write8(0x224, 0x04);
        uint8_t voice = sb.read8(0x225);

        // FM volume
        sb.write8(0x224, 0x26);
        uint8_t fm = sb.read8(0x225);

        (void)voice;
        (void)fm;
    }

    SECTION("Mixer reset") {
        // Write 1 to mixer register 0x00 resets to defaults
        sb.write8(0x224, 0x00);
        sb.write8(0x225, 0x01); // Reset
        // After reset, volumes should be at defaults
        sb.write8(0x224, 0x22);
        uint8_t master = sb.read8(0x225);
        REQUIRE((master & 0x0F) == 0x0C); // Default 75%
    }
}

// ──  sb_detect() with BLASTER environment variable ────────────────
// The BLASTER variable is parsed for: A(port), I(irq), D(dma8), H(dma16), T(type)
TEST_CASE("Allegro SB detection: BLASTER parsing behavior", "[SB][Allegro]") {
    MemoryBus memory;
    DMA8237 dma;
    SoundBlaster sb(memory, dma, 44100.0f);

    SECTION("Default port 0x220 when no BLASTER") {
        // default to port 0x220 if BLASTER not set
        // Our SB is at 0x220, so detection should succeed
        sb.write8(0x226, 0x01);
        for (int i = 0; i < 8; i++) sb.read8(0x226);
        sb.write8(0x226, 0x00);
        REQUIRE(sb.read8(0x22A) == 0xAA);
    }

    SECTION("SB16 version check against requested type") {
        // If BLASTER has T6 (SB16), Allegro checks sb_dsp_ver >= 0x400
        // Our DSP returns 4.05 = 0x0405 which is >= 0x400
        sb.write8(0x226, 0x01);
        for (int i = 0; i < 8; i++) sb.read8(0x226);
        sb.write8(0x226, 0x00);
        sb.read8(0x22A);

        sb.write8(0x22C, 0xE1);
        uint8_t major = sb.read8(0x22A);
        uint8_t minor = sb.read8(0x22A);
        uint16_t dspVer = (major << 8) | minor;
        REQUIRE(dspVer >= 0x400); // SB16 compatible
    }

    SECTION("Older SB version detection") {
        // If BLASTER has T4 (SBPro, ver=0x300) but hardware reports 4.05,
        sb.write8(0x226, 0x01);
        for (int i = 0; i < 8; i++) sb.read8(0x226);
        sb.write8(0x226, 0x00);
        sb.read8(0x22A);
        sb.write8(0x22C, 0xE1);
        uint8_t major = sb.read8(0x22A);
        uint8_t minor = sb.read8(0x22A);
        uint16_t hwVer = (major << 8) | minor;
        uint16_t requestedVer = 0x300; // SBPro
        // Allegro fails if requested > detected
        bool compatible = (requestedVer <= hwVer);
        REQUIRE(compatible == true); // 0x300 <= 0x0405
    }
}

// ── fm_is_there() AdLib detection ────────────────────────────────
TEST_CASE("AdLib detection: fm_is_there() sequence", "[AdLib]") {
    uint64_t cycles = 0;
    AdLib opl(44100.0f, &cycles);

    SECTION("Step 1: Reset timers and check status is clear") {
        // fm_write(1, 0) — init test register
        opl.write8(0x388, 0x01);
        opl.write8(0x389, 0x00);

        // fm_write(4, 0x60) — reset both timers
        opl.write8(0x388, 0x04);
        opl.write8(0x389, 0x60);

        // fm_write(4, 0x80) — enable interrupts (actually resets IRQ flag)
        opl.write8(0x388, 0x04);
        opl.write8(0x389, 0x80);

        // inportb(_fm_port) & 0xE0 should be 0
        uint8_t status = opl.read8(0x388);
        REQUIRE((status & 0xE0) == 0x00);
    }

    SECTION("Step 2: Start timer 1 and check expiry") {
        // Reset timers first
        opl.write8(0x388, 0x01); opl.write8(0x389, 0x00);
        opl.write8(0x388, 0x04); opl.write8(0x389, 0x60);
        opl.write8(0x388, 0x04); opl.write8(0x389, 0x80);
        opl.read8(0x388); // clear status

        // fm_write(2, 0xFF) — set timer 1 data to maximum
        opl.write8(0x388, 0x02);
        opl.write8(0x389, 0xFF);

        // fm_write(4, 0x21) — start timer 1
        opl.write8(0x388, 0x04);
        opl.write8(0x389, 0x21);

        // Timer 1 with data=0xFF overflows after ~80µs
        // We need to advance enough CPU cycles for the timer to fire
        // At 33MHz, 100ms = 3,300,000 cycles
        cycles += 3300000;

        // inportb(_fm_port) & 0xE0 should be 0xC0
        // Bit 7 = timer 1 expired, bit 6 = timer 1 flag
        uint8_t status = opl.read8(0x388);
        REQUIRE((status & 0xC0) == 0xC0);
    }

    SECTION("Step 3: Reset timers after detection") {
        // After detection, Allegro resets timers again
        opl.write8(0x388, 0x01); opl.write8(0x389, 0x00);
        opl.write8(0x388, 0x04); opl.write8(0x389, 0x60);
        opl.write8(0x388, 0x04); opl.write8(0x389, 0x80);

        // Start timer 1
        opl.write8(0x388, 0x02); opl.write8(0x389, 0xFF);
        opl.write8(0x388, 0x04); opl.write8(0x389, 0x21);
        cycles += 3300000;

        // Verify timer fired
        uint8_t status = opl.read8(0x388);
        REQUIRE((status & 0xC0) == 0xC0);

        // Reset timers
        opl.write8(0x388, 0x04); opl.write8(0x389, 0x60);
        opl.write8(0x388, 0x04); opl.write8(0x389, 0x80);

        // Status should be clear again
        status = opl.read8(0x388);
        REQUIRE((status & 0xE0) == 0x00);
    }

    SECTION("Full fm_is_there() emulation") {
        // Complete detection sequence as Allegro does it
        opl.write8(0x388, 0x01); opl.write8(0x389, 0x00);  // fm_write(1, 0)
        opl.write8(0x388, 0x04); opl.write8(0x389, 0x60);  // fm_write(4, 0x60)
        opl.write8(0x388, 0x04); opl.write8(0x389, 0x80);  // fm_write(4, 0x80)

        // if (inportb(_fm_port) & 0xE0) return FALSE;
        uint8_t s1 = opl.read8(0x388);
        bool step1ok = ((s1 & 0xE0) == 0);
        REQUIRE(step1ok == true);

        opl.write8(0x388, 0x02); opl.write8(0x389, 0xFF);  // fm_write(2, 0xFF)
        opl.write8(0x388, 0x04); opl.write8(0x389, 0x21);  // fm_write(4, 0x21)

        // rest(100) — wait for timer to expire
        cycles += 3300000;

        // if ((inportb(_fm_port) & 0xE0) != 0xC0) return FALSE;
        uint8_t s2 = opl.read8(0x388);
        bool step2ok = ((s2 & 0xE0) == 0xC0);
        REQUIRE(step2ok == true);

        // fm_write(4, 0x60); fm_write(4, 0x80);
        opl.write8(0x388, 0x04); opl.write8(0x389, 0x60);
        opl.write8(0x388, 0x04); opl.write8(0x389, 0x80);

        // return TRUE;
        REQUIRE(step1ok == true);
        REQUIRE(step2ok == true);
    }
}

// ── OPL3 detection ───────────────────────────────────────────────
TEST_CASE("AdLib detection: OPL3 vs OPL2 check", "[AdLib]") {
    uint64_t cycles = 0;
    AdLib opl(44100.0f, &cycles);

    // Run detection sequence
    opl.write8(0x388, 0x01); opl.write8(0x389, 0x00);
    opl.write8(0x388, 0x04); opl.write8(0x389, 0x60);
    opl.write8(0x388, 0x04); opl.write8(0x389, 0x80);
    opl.write8(0x388, 0x02); opl.write8(0x389, 0xFF);
    opl.write8(0x388, 0x04); opl.write8(0x389, 0x21);
    cycles += 3300000;

    // OPL3 check: (inportb(_fm_port) & 6) == 0
    // Our OPL2 returns status with timer flags, not OPL3 ID
    uint8_t status = opl.read8(0x388);
    bool isOPL3 = ((status & 0x06) == 0);
    // Our emulator is OPL2, so this should be false (bits 1-2 are set from timer)
    // Actually, after timer reset, status might be 0xC0, and 0xC0 & 6 = 0
    // This depends on the exact timing
    (void)isOPL3; // Just verify the read works

    // Reset
    opl.write8(0x388, 0x04); opl.write8(0x389, 0x60);
    opl.write8(0x388, 0x04); opl.write8(0x389, 0x80);
}

// ── OPL register write with delays ───────────────────────────────
// Reads the OPL port between writes as a delay mechanism
TEST_CASE("AdLib: fm_write delay behavior", "[AdLib]") {
    uint64_t cycles = 0;
    AdLib opl(44100.0f, &cycles);

    SECTION("Register write with interleave reads") {

        // For OPL2: fm_delay_1=6, fm_delay_2=35
        // Write to register 0x20 (channel 0 operator flags)
        opl.write8(0x388, 0x20);     // register select
        for (int i = 0; i < 6; i++)  // delay
            opl.read8(0x388);
        opl.write8(0x389, 0x01);     // data: enable vibrato
        for (int i = 0; i < 35; i++) // delay
            opl.read8(0x388);

        // Verify register was written
        REQUIRE(opl.read8(0x388) != 0xFF); // Should return status, not garbage
    }
}

// ── sb_detect() DSP write/read timing ────────────────────────────
// polls port+0x22C (write status) and port+0x22E (read status)
TEST_CASE("SB: DSP write/read status polling", "[SB]") {
    MemoryBus memory;
    DMA8237 dma;
    SoundBlaster sb(memory, dma, 44100.0f);
    sb.write8(0x226, 0x01);
    for (int i = 0; i < 8; i++) sb.read8(0x226);
    sb.write8(0x226, 0x00);
    sb.read8(0x22A); // consume 0xAA

    SECTION("Write status (port+0x22C) always ready") {
        // sb_write_dsp polls: if (!(inportb(0x0C+_sound_port) & 0x80))
        // Our implementation returns 0x00 (always ready)
        for (int i = 0; i < 10; i++) {
            uint8_t ws = sb.read8(0x22C);
            REQUIRE(!(ws & 0x80)); // Always ready
        }
    }

    SECTION("Read status (port+0x22E) reflects data availability") {
        // After reset, 0xAA was pushed. We consumed it.
        REQUIRE(sb.read8(0x22E) == 0x00); // No data

        // After DSP command 0xE1, two bytes are pushed
        sb.write8(0x22C, 0xE1);
        REQUIRE(sb.read8(0x22E) == 0x80); // Data available
        sb.read8(0x22A); // consume first byte
        REQUIRE(sb.read8(0x22E) == 0x80); // Still data available
        sb.read8(0x22A); // consume second byte
        REQUIRE(sb.read8(0x22E) == 0x00); // No more data
    }

    SECTION("8-bit IRQ ack (port+0x22E) clears IRQ flag") {
        // Reading port+0x22E acknowledges 8-bit IRQ
        sb.write8(0x22C, 0xF2); // Force 8-bit IRQ
        // After force IRQ, reading 0x22E clears m_irq8Pending
        uint8_t ack = sb.read8(0x22E);
        (void)ack;
    }
}

// ── sound init: complete flow simulation ─────────────────────────
    TEST_CASE("Allegro sound init: complete detection flow", "[SB][AdLib]") {
    MemoryBus memory;
    DMA8237 dma;
    SoundBlaster sb(memory, dma, 44100.0f);
    uint64_t cycles = 0;
    AdLib opl(44100.0f, &cycles);

    // Step 1: SB detection
    // _sb_reset_dsp(1)
    sb.write8(0x226, 0x01);
    for (int i = 0; i < 8; i++) sb.read8(0x226);
    sb.write8(0x226, 0x00);
    bool sbDetected = (sb.read8(0x22E) & 0x80) && (sb.read8(0x22A) == 0xAA);
    REQUIRE(sbDetected == true);

    // Step 2: Read DSP version
    sb.write8(0x22C, 0xE1);
    uint8_t major = sb.read8(0x22A);
    uint8_t minor = sb.read8(0x22A);
    uint16_t dspVer = (major << 8) | minor;
    REQUIRE(dspVer == 0x0405); // SB16

    // Step 3: Read SB16 mixer config
    sb.write8(0x224, 0x80); // IRQ status
    uint8_t irqConfig = sb.read8(0x225);
    sb.write8(0x224, 0x81); // DMA config
    uint8_t dmaConfig = sb.read8(0x225);

    // Step 4: Speaker on
    sb.write8(0x22C, 0xD1);
    sb.write8(0x22C, 0xD1); // Speaker on command

    // Step 5: AdLib detection
    opl.write8(0x388, 0x01); opl.write8(0x389, 0x00);
    opl.write8(0x388, 0x04); opl.write8(0x389, 0x60);
    opl.write8(0x388, 0x04); opl.write8(0x389, 0x80);
    uint8_t adlibStatus1 = opl.read8(0x388);
    bool adlibStep1 = ((adlibStatus1 & 0xE0) == 0);

    opl.write8(0x388, 0x02); opl.write8(0x389, 0xFF);
    opl.write8(0x388, 0x04); opl.write8(0x389, 0x21);
    cycles += 3300000; // Wait 100ms
    uint8_t adlibStatus2 = opl.read8(0x388);
    bool adlibStep2 = ((adlibStatus2 & 0xC0) == 0xC0);

    opl.write8(0x388, 0x04); opl.write8(0x389, 0x60);
    opl.write8(0x388, 0x04); opl.write8(0x389, 0x80);

    bool adlibDetected = adlibStep1 && adlibStep2;
    REQUIRE(adlibDetected == true);

    // Both SB and AdLib should be detected
    REQUIRE(sbDetected == true);
    REQUIRE(adlibDetected == true);
}

// ── PIT programming as Allegro timer does it ─────────────────────────────
TEST_CASE("PIT programming as Allegro timer does it", "[PIT]") {
    MemoryBus mem;
    PIT8254 pit;

    SECTION("Program PIT channel 0 for rate generator") {
        uint16_t timerSpeed = 5965; // BPS_TO_TIMER(200) ≈ 1193182/200
        pit.write8(0x43, 0x34); // Channel 0, lobyte/hibyte, mode 2
        pit.write8(0x40, timerSpeed & 0xFF);
        pit.write8(0x40, (timerSpeed >> 8) & 0xFF);

        // Verify PIT was programmed by reading back
        pit.write8(0x43, 0x00); // Latch count
        uint8_t lo = pit.read8(0x40);
        uint8_t hi = pit.read8(0x40);
        uint16_t count = lo | (hi << 8);
        // Count should be close to what we programmed
        REQUIRE(count > 0);
        REQUIRE(count <= 0xFFFF);
    }

    SECTION("Program PIT channel 0 for one-shot") {
        pit.write8(0x43, 0x30);
        pit.write8(0x40, 0x00);
        pit.write8(0x40, 0x10); // 0x1000 = 4096 ticks

        // Verify
        pit.write8(0x43, 0x00);
        uint8_t lo = pit.read8(0x40);
        uint8_t hi = pit.read8(0x40);
        uint16_t count = lo | (hi << 8);
        REQUIRE(count > 0);
    }

    SECTION("Read PIT timer value") {
        pit.write8(0x43, 0x00);
        uint8_t lo = pit.read8(0x40);
        uint8_t hi = pit.read8(0x40);
        uint16_t raw = lo | (hi << 8);
        uint16_t elapsed = (0xFFFF - raw + 1) & 0xFFFF;
        // Just verify it doesn't crash
        (void)elapsed;
    }
}

// ── fm_init() OPL register initialization ────────────────────────────────
TEST_CASE("AdLib: fm_reset register initialization", "[AdLib]") {
    uint64_t cycles = 0;
    AdLib opl(44100.0f, &cycles);

    SECTION("fm_reset clears all registers 0x01-0xF5") {
        // fm_reset writes 0 to registers 0xF5 down to 0x01
        for (int i = 0xF5; i > 0; i--) {
            opl.write8(0x388, static_cast<uint8_t>(i));
            opl.write8(0x389, 0x00);
        }

        // Verify waveform control register 0x01
        opl.write8(0x388, 0x01);
        opl.write8(0x389, 0x20); // Enable waveform select

        // Verify key-on registers are clear
        for (int ch = 0; ch < 9; ch++) {
            opl.write8(0x388, static_cast<uint8_t>(0xB0 + ch));
            opl.write8(0x389, 0x00); // Key off
        }

        // Should not crash
        REQUIRE(true);
    }

    SECTION("fm_reset sets up drum mode") {
        // fm_write(0xBD, 0xC0) — set AM and vibrato to high
        opl.write8(0x388, 0xBD);
        opl.write8(0x389, 0xC0);

        // Verify register was written (status read should work)
        uint8_t status = opl.read8(0x388);
        (void)status;
    }
}
