#include "test_framework.hpp"
#include "hw/audio/AdLib.hpp"
#include <cmath>

TEST_CASE("AdLib Basic Functionality", "[audio][adlib]") {
    fador::hw::audio::AdLib adlib(44100.0f);

    SECTION("Initialization") {
        REQUIRE(adlib.read8(0x388) == 0); // Check initial status
    }

    SECTION("Register Write and Timers") {
        adlib.write8(0x388, 0x02); // Select Timer 1 register
        adlib.write8(0x389, 0xFF); // Write max value to Timer 1

        adlib.write8(0x388, 0x04); // Select Timer Control
        adlib.write8(0x389, 0x01); // Start Timer 1

        adlib.updateTimers(0.0001); // Update by 100us, timer should expire (80us res)
        REQUIRE((adlib.read8(0x388) & 0xC0) == 0xC0); // Check IRQ and T1 expired flags
    }

    SECTION("Generate Audio Samples") {
        // Setup a simple beep on Channel 0
        adlib.write8(0x388, 0x20); // AM/VIB/EG/KSR/MULT for Modulator
        adlib.write8(0x389, 0x01);
        adlib.write8(0x388, 0x23); // AM/VIB/EG/KSR/MULT for Carrier
        adlib.write8(0x389, 0x01);

        adlib.write8(0x388, 0x40); // KSL/TL for Modulator
        adlib.write8(0x389, 0x00);
        adlib.write8(0x388, 0x43); // KSL/TL for Carrier
        adlib.write8(0x389, 0x00);

        adlib.write8(0x388, 0x60); // AR/DR for Modulator
        adlib.write8(0x389, 0xF0); // Fast attack
        adlib.write8(0x388, 0x63); // AR/DR for Carrier
        adlib.write8(0x389, 0xF0); // Fast attack

        adlib.write8(0x388, 0x80); // SL/RR for Modulator
        adlib.write8(0x389, 0x00);
        adlib.write8(0x388, 0x83); // SL/RR for Carrier
        adlib.write8(0x389, 0x00);

        adlib.write8(0x388, 0xA0); // F-Number LSB for Ch 0
        adlib.write8(0x389, 0x90);
        adlib.write8(0x388, 0xB0); // Key-On, Block, F-Number MSB for Ch 0
        adlib.write8(0x389, 0x20 | 0x04 | 0x02); // Key On, Block 1, MSB 2

        float buffer[100];
        for (int i=0; i<100; i++) buffer[i] = 0.0f;

        adlib.generateSamples(buffer, 50); // 50 stereo pairs = 100 floats

        // Verify audio is generated (not completely silent)
        bool hasAudio = false;
        for (int i=0; i<100; i++) {
            if (std::abs(buffer[i]) > 0.0001f) {
                hasAudio = true;
                break;
            }
        }
        REQUIRE(hasAudio);
    }
}
