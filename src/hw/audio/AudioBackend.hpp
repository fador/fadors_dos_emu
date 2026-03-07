#pragma once

#include <cstdint>
#include <vector>

namespace fador::hw::audio {

class AudioBackend {
public:
    AudioBackend();
    ~AudioBackend();

    // Initializes the audio device (using SDL2 if available)
    bool init(int sampleRate = 44100, int channels = 2, int bufferSize = 1024);
    void close();

    // Queues stereo float samples (interleaved)
    // Returns true if samples were successfully queued.
    bool queueSamples(const float* buffer, size_t count);

    // Get the amount of queued bytes
    uint32_t getQueuedAudioSize();

private:
#ifdef HAVE_SDL2
    uint32_t m_deviceId;
#endif
    bool m_initialized;
    int m_sampleRate;
    int m_channels;
};

} // namespace fador::hw::audio
