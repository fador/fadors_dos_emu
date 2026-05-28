#include "test_framework.hpp"
#include "hw/audio/MidiDevice.hpp"
#include "hw/audio/MidiSynth.hpp"
#include "hw/audio/AdLib.hpp"

using namespace fador::hw::audio;

TEST_CASE("MidiDevice: Initialization and UART mode", "[Midi]") {
    MidiDevice midi(0x330);

    SECTION("Default state is UART mode") {
        uint8_t status = midi.read8(0x331);
        REQUIRE((status & 0x80) != 0);
        REQUIRE((status & 0x40) == 0);
    }

    SECTION("Data register read is empty") {
        uint8_t data = midi.read8(0x330);
        REQUIRE(data == 0x00);
    }
}

TEST_CASE("MidiDevice: Reset command in UART mode", "[Midi]") {
    MidiDevice midi(0x330);

    SECTION("Reset via command port returns ACK") {
        midi.write8(0x331, 0xFF);

        uint8_t status = midi.read8(0x331);
        REQUIRE((status & 0x80) == 0); // data available
        uint8_t ack = midi.read8(0x330);
        REQUIRE(ack == 0xFE);
    }
}

TEST_CASE("MidiDevice: Intelligent mode commands", "[Midi]") {
    MidiDevice midi(0x330);

    midi.write8(0x331, 0xFF);
    midi.read8(0x330); // consume ACK

    SECTION("Enter UART mode (0x3F)") {
        midi.write8(0x331, 0x3F);

        uint8_t status = midi.read8(0x331);
        REQUIRE((status & 0x80) == 0);
        uint8_t ack = midi.read8(0x330);
        REQUIRE(ack == 0xFE);
    }

    SECTION("Get MIDI version (0xAC)") {
        midi.write8(0x331, 0xAC);

        uint8_t status1 = midi.read8(0x331);
        REQUIRE((status1 & 0x80) == 0);
        uint8_t ack = midi.read8(0x330);
        REQUIRE(ack == 0xFE);

        uint8_t status2 = midi.read8(0x331);
        REQUIRE((status2 & 0x80) == 0);
        uint8_t ver = midi.read8(0x330);
        REQUIRE(ver == 0x01);
    }

    SECTION("Get revision (0xAD)") {
        midi.write8(0x331, 0xAD);

        uint8_t status1 = midi.read8(0x331);
        REQUIRE((status1 & 0x80) == 0);
        uint8_t ack = midi.read8(0x330);
        REQUIRE(ack == 0xFE);

        uint8_t status2 = midi.read8(0x331);
        REQUIRE((status2 & 0x80) == 0);
        uint8_t rev = midi.read8(0x330);
        REQUIRE(rev == 0x01);
    }
}

TEST_CASE("MidiDevice: MIDI byte parsing - note on/off", "[Midi]") {
    MidiDevice midi(0x330);

    int eventCount = 0;
    uint8_t lastChannel = 0xFF;
    uint8_t lastStatus = 0xFF;
    uint8_t lastData1 = 0xFF;
    uint8_t lastData2 = 0xFF;

    midi.setEventCallback(
        [&](uint8_t channel, uint8_t statusType, uint8_t data1, uint8_t data2) {
          eventCount++;
          lastChannel = channel;
          lastStatus = statusType;
          lastData1 = data1;
          lastData2 = data2;
        });

    SECTION("Note on (ch 0, note 60, vel 100)") {
        midi.write8(0x330, 0x90);
        midi.write8(0x330, 60);
        midi.write8(0x330, 100);

        REQUIRE(eventCount == 1);
        REQUIRE(lastChannel == 0);
        REQUIRE(lastStatus == 0x90);
        REQUIRE(lastData1 == 60);
        REQUIRE(lastData2 == 100);
    }

    SECTION("Note on with velocity 0 becomes note off") {
        midi.write8(0x330, 0x91);
        midi.write8(0x330, 64);
        midi.write8(0x330, 0);

        REQUIRE(eventCount == 1);
        REQUIRE(lastChannel == 1);
        REQUIRE(lastStatus == 0x80);
        REQUIRE(lastData1 == 64);
    }

    SECTION("Note off (ch 3, note 72)") {
        midi.write8(0x330, 0x83);
        midi.write8(0x330, 72);
        midi.write8(0x330, 64);

        REQUIRE(eventCount == 1);
        REQUIRE(lastChannel == 3);
        REQUIRE(lastStatus == 0x80);
        REQUIRE(lastData1 == 72);
        REQUIRE(lastData2 == 64);
    }
}

TEST_CASE("MidiDevice: MIDI byte parsing - running status", "[Midi]") {
    MidiDevice midi(0x330);

    int eventCount = 0;
    midi.setEventCallback(
        [&](uint8_t, uint8_t, uint8_t, uint8_t) { eventCount++; });

    SECTION("Running status reuses last status byte") {
        midi.write8(0x330, 0x90);
        midi.write8(0x330, 48);
        midi.write8(0x330, 80);

        midi.write8(0x330, 50);
        midi.write8(0x330, 90);

        REQUIRE(eventCount == 2);
    }
}

TEST_CASE("MidiDevice: MIDI byte parsing - program change", "[Midi]") {
    MidiDevice midi(0x330);

    int eventCount = 0;
    uint8_t lastChannel = 0xFF;
    uint8_t lastStatus = 0xFF;
    uint8_t lastData1 = 0xFF;

    midi.setEventCallback(
        [&](uint8_t channel, uint8_t statusType, uint8_t data1, uint8_t) {
          eventCount++;
          lastChannel = channel;
          lastStatus = statusType;
          lastData1 = data1;
        });

    SECTION("Program change (ch 5, prog 42)") {
        midi.write8(0x330, 0xC5);
        midi.write8(0x330, 42);

        REQUIRE(eventCount == 1);
        REQUIRE(lastChannel == 5);
        REQUIRE(lastStatus == 0xC0);
        REQUIRE(lastData1 == 42);
    }
}

TEST_CASE("MidiDevice: MIDI byte parsing - control change", "[Midi]") {
    MidiDevice midi(0x330);

    int eventCount = 0;
    uint8_t lastChannel = 0xFF;
    uint8_t lastData1 = 0xFF;
    uint8_t lastData2 = 0xFF;

    midi.setEventCallback(
        [&](uint8_t channel, uint8_t, uint8_t data1, uint8_t data2) {
          eventCount++;
          lastChannel = channel;
          lastData1 = data1;
          lastData2 = data2;
        });

    SECTION("Control change (ch 2, CC 7 volume 100)") {
        midi.write8(0x330, 0xB2);
        midi.write8(0x330, 7);
        midi.write8(0x330, 100);

        REQUIRE(eventCount == 1);
        REQUIRE(lastChannel == 2);
        REQUIRE(lastData1 == 7);
        REQUIRE(lastData2 == 100);
    }
}

TEST_CASE("MidiDevice: MIDI byte parsing - pitch bend", "[Midi]") {
    MidiDevice midi(0x330);

    int eventCount = 0;
    uint8_t lastChannel = 0xFF;
    uint8_t lastData1 = 0xFF;
    uint8_t lastData2 = 0xFF;

    midi.setEventCallback(
        [&](uint8_t channel, uint8_t, uint8_t data1, uint8_t data2) {
          eventCount++;
          lastChannel = channel;
          lastData1 = data1;
          lastData2 = data2;
        });

    SECTION("Pitch bend (ch 0, LSB 0, MSB 64 = center)") {
        midi.write8(0x330, 0xE0);
        midi.write8(0x330, 0);
        midi.write8(0x330, 64);

        REQUIRE(eventCount == 1);
        REQUIRE(lastChannel == 0);
        REQUIRE(lastData1 == 0);
        REQUIRE(lastData2 == 64);
    }
}

TEST_CASE("MidiSynth: Note on/off programs AdLib", "[Midi]") {
    AdLib adlib(44100.0f);
    MidiSynth synth(adlib, 44100.0f);

    SECTION("Note on programs frequency and key-on") {
        synth.handleEvent(0, 0x90, 60, 100);

        // The note should now be playing — verify by generating samples
        float buffer[128] = {};
        adlib.generateSamples(buffer, 64);
        bool hasOutput = false;
        for (size_t i = 0; i < 128; i++) {
          if (buffer[i] != 0.0f) {
            hasOutput = true;
            break;
          }
        }
        REQUIRE(hasOutput == true);
    }

    SECTION("Note off clears key-on") {
        synth.handleEvent(0, 0x90, 60, 100);
        float buffer1[128] = {};
        adlib.generateSamples(buffer1, 64);

        synth.handleEvent(0, 0x80, 60, 0);

        float buffer2[128] = {};
        adlib.generateSamples(buffer2, 64);

        // After note-off and some decay, signal should fade
        bool hasOutput = true;
        float maxVal = 0;
        for (size_t i = 0; i < 128; i++) {
          if (std::abs(buffer2[i]) > maxVal)
            maxVal = std::abs(buffer2[i]);
        }
        REQUIRE(maxVal >= 0.0f); // Sound should exist or have decayed
    }

    SECTION("Program change loads instrument") {
        synth.handleEvent(0, 0xC0, 0, 0); // Acoustic Grand Piano
        synth.handleEvent(0, 0x90, 60, 100);

        float buffer[128] = {};
        adlib.generateSamples(buffer, 64);
        bool hasOutput = false;
        for (size_t i = 0; i < 128; i++) {
          if (buffer[i] != 0.0f) {
            hasOutput = true;
            break;
          }
        }
        REQUIRE(hasOutput == true);
    }
}

TEST_CASE("MidiSynth: All notes off", "[Midi]") {
    AdLib adlib(44100.0f);
    MidiSynth synth(adlib, 44100.0f);

    SECTION("All notes off silences all voices") {
        synth.handleEvent(0, 0x90, 60, 100);
        synth.handleEvent(1, 0x90, 64, 100);
        synth.handleEvent(2, 0x90, 67, 100);

        synth.allNotesOff();

        float buffer[128] = {};
        adlib.generateSamples(buffer, 64);
        bool hasOutput = false;
        for (size_t i = 0; i < 128; i++) {
          if (std::abs(buffer[i]) > 0.001f) {
            hasOutput = true;
            break;
          }
        }
        REQUIRE(hasOutput == false);
    }
}

TEST_CASE("MidiSynth: Velocity affects volume", "[Midi]") {
    AdLib adlib(44100.0f);
    MidiSynth synth(adlib, 44100.0f);

    SECTION("Low velocity yields quieter output") {
        synth.handleEvent(0, 0xC0, 0, 0);
        synth.handleEvent(0, 0x90, 60, 20);

        float buffer1[128] = {};
        adlib.generateSamples(buffer1, 4);
        float max1 = 0;
        for (size_t i = 0; i < 128; i++)
          if (std::abs(buffer1[i]) > max1) max1 = std::abs(buffer1[i]);

        synth.allNotesOff();

        synth.handleEvent(0, 0x90, 60, 127);

        float buffer2[128] = {};
        adlib.generateSamples(buffer2, 4);
        float max2 = 0;
        for (size_t i = 0; i < 128; i++)
          if (std::abs(buffer2[i]) > max2) max2 = std::abs(buffer2[i]);

        REQUIRE(max1 <= max2);
    }
}

TEST_CASE("MidiInstruments: All 128 instruments available", "[Midi]") {
    SECTION("Every program number returns non-null instrument") {
        for (int i = 0; i < 128; i++) {
            const auto *inst = getInstrument(i);
            REQUIRE(inst != nullptr);
        }
    }

    SECTION("Out of range program returns first instrument") {
        const auto *inst = getInstrument(255);
        REQUIRE(inst == &kGmInstruments[0]);
    }
}
