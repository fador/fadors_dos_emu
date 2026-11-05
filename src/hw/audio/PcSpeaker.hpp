#pragma once
#include <chrono>
#include <cstdint>
#include <deque>

namespace fador::hw::audio {

// PC speaker: port 61h bits 0 (PIT channel 2 gate) and 1 (speaker data)
// combined with PIT channel 2.  Tones (channel 2 in mode 3) are rendered
// as square waves; bit-banged output (port 61h bit 1 toggled directly,
// including 1-bit PWM sound) is rendered from time-stamped level changes.
class PcSpeaker {
public:
  using Clock = std::chrono::steady_clock;

  PcSpeaker(float sampleRate = 44100.0f);

  void port61Written(uint8_t value);
  void channel2Programmed(uint16_t reload, uint8_t mode);

  // Mix numSamples stereo frames ending "now" into buffer.
  void generateSamples(float *buffer, size_t numSamples);

private:
  struct Event {
    double time;   // seconds since construction
    bool gate;     // port 61h bit 0
    bool data;     // port 61h bit 1
    uint16_t reload;
    uint8_t mode;
  };
  double now() const;
  float levelAt(const Event &state, double t) const;

  float m_sampleRate;
  Clock::time_point m_start;
  std::deque<Event> m_events;
  Event m_state{0.0, false, false, 0, 3};
  double m_renderTime = 0.0;
  float m_filtered = 0.0f;
  bool m_active = false; // any speaker activity since the last silence
};

} // namespace fador::hw::audio
