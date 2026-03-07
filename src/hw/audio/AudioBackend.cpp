#include "AudioBackend.hpp"
#include "../../utils/Logger.hpp"

#ifdef HAVE_SDL2
#include <SDL.h>
#endif

namespace fador::hw::audio {

AudioBackend::AudioBackend()
    : m_initialized(false), m_sampleRate(44100), m_channels(2) {
#ifdef HAVE_SDL2
  m_deviceId = 0;
#endif
}

AudioBackend::~AudioBackend() { close(); }

bool AudioBackend::init(int sampleRate, int channels, int bufferSize) {
  if (m_initialized) {
    return true;
  }

  m_sampleRate = sampleRate;
  m_channels = channels;

#ifdef HAVE_SDL2
  // We assume SDL_Init(SDL_INIT_AUDIO) has been or will be called by the
  // frontend. However, to be safe, we can initialize it here if it's not
  // initialized.
  if (SDL_InitSubSystem(SDL_INIT_AUDIO) < 0) {
    LOG_ERROR("SDL_InitSubSystem(SDL_INIT_AUDIO) failed: ", SDL_GetError());
    return false;
  }

  SDL_AudioSpec desiredSpec;
  SDL_AudioSpec obtainedSpec;

  SDL_zero(desiredSpec);
  desiredSpec.freq = sampleRate;
  desiredSpec.format =
      AUDIO_F32SYS; // We will generate 32-bit float audio internally
  desiredSpec.channels = channels;
  desiredSpec.samples = bufferSize;
  desiredSpec.callback =
      nullptr; // We use SDL_QueueAudio instead of a callback for simplicity

  m_deviceId = SDL_OpenAudioDevice(nullptr, 0, &desiredSpec, &obtainedSpec, 0);
  if (m_deviceId == 0) {
    LOG_ERROR("Failed to open SDL audio device: ", SDL_GetError());
    return false;
  }

  if (obtainedSpec.format != AUDIO_F32SYS) {
    LOG_WARN(
        "SDL audio device didn't give us F32 format. We might hear garbage.");
  }

  LOG_INFO("Audio device initialized. Freq: ", obtainedSpec.freq,
           ", Channels: ", (int)obtainedSpec.channels,
           ", Samples: ", obtainedSpec.samples);

  SDL_PauseAudioDevice(m_deviceId, 0); // Start playback
  m_initialized = true;
  return true;
#else
  LOG_INFO("Audio logic initialized without native rendering backend.");
  m_initialized = true;
  return true;
#endif
}

void AudioBackend::close() {
#ifdef HAVE_SDL2
  if (m_deviceId != 0) {
    SDL_CloseAudioDevice(m_deviceId);
    m_deviceId = 0;
  }
#endif
  m_initialized = false;
}

bool AudioBackend::queueSamples(const float *buffer, size_t count) {
  if (!m_initialized) {
    return false;
  }

#ifdef HAVE_SDL2
  // count is the number of floats to queue.
  int ret = SDL_QueueAudio(m_deviceId, buffer, count * sizeof(float));
  return (ret == 0);
#else
  // Throw away samples when no audio backend
  (void)buffer;
  (void)count;
  return true;
#endif
}

uint32_t AudioBackend::getQueuedAudioSize() {
#ifdef HAVE_SDL2
  if (m_deviceId != 0) {
    return SDL_GetQueuedAudioSize(m_deviceId);
  }
#endif
  return 0;
}

} // namespace fador::hw::audio
