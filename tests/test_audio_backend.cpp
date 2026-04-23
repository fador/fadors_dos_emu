#include "test_framework.hpp"
#include "hw/audio/AudioBackend.hpp"

using namespace fador::hw::audio;

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
