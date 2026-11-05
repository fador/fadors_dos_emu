#include "PcSpeaker.hpp"
#include <algorithm>
#include <cmath>

namespace fador::hw::audio {

namespace {
constexpr double kPitHz = 1193181.8181818181;
constexpr float kVolume = 0.25f;
} // namespace

PcSpeaker::PcSpeaker(float sampleRate)
    : m_sampleRate(sampleRate), m_start(Clock::now()) {}

double PcSpeaker::now() const {
  return std::chrono::duration<double>(Clock::now() - m_start).count();
}

void PcSpeaker::port61Written(uint8_t value) {
  Event e = m_events.empty() ? m_state : m_events.back();
  const bool gate = (value & 1) != 0, data = (value & 2) != 0;
  if (e.gate == gate && e.data == data)
    return;
  e.time = now();
  e.gate = gate;
  e.data = data;
  m_events.push_back(e);
  m_active = true;
}

void PcSpeaker::channel2Programmed(uint16_t reload, uint8_t mode) {
  Event e = m_events.empty() ? m_state : m_events.back();
  e.time = now();
  e.reload = reload;
  e.mode = mode;
  m_events.push_back(e);
  m_active = true;
}

// Speaker output for a given state at time t (-1..1 before volume).
float PcSpeaker::levelAt(const Event &s, double t) const {
  if (!s.data)
    return 0.0f; // speaker disconnected: the cone rests
  if (!s.gate)
    return 1.0f; // gate low: channel 2 output stays high (bit-banging)
  const uint32_t reload = s.reload ? s.reload : 0x10000;
  const double freq = kPitHz / reload;
  if ((s.mode & 3) == 3 || (s.mode & 3) == 2) {
    if (freq > 16000.0)
      return 0.5f; // ultrasonic carrier: averages out
    const double phase = std::fmod((t - s.time) * freq, 1.0);
    return phase < 0.5 ? 1.0f : -1.0f;
  }
  // Modes 0/1 (one-shot): output low until the count runs out, then high.
  // Rewriting the count every sample is 1-bit PWM ("RealSound", MOD
  // players): the duty cycle is the sample value.
  return (t - s.time) * kPitHz >= reload ? 1.0f : -1.0f;
}

void PcSpeaker::generateSamples(float *buffer, size_t numSamples) {
  const double end = now();
  const double dt = 1.0 / m_sampleRate;
  double t = end - numSamples * dt;
  if (m_renderTime < t - 0.25 || m_renderTime > end)
    m_renderTime = t; // resynchronise after stalls
  t = std::max(t, m_renderTime);
  if (!m_active && m_events.empty() && m_filtered == 0.0f) {
    m_renderTime = end;
    return;
  }
  for (size_t i = 0; i < numSamples; ++i) {
    const double ts = end - (numSamples - i) * dt;
    // Average over the sample period: PWM and bit-banged output switch far
    // faster than the output rate.
    constexpr int kSub = 8;
    float level = 0.0f;
    for (int k = 0; k < kSub; ++k) {
      const double tk = ts - dt + (k + 0.5) * dt / kSub;
      while (!m_events.empty() && m_events.front().time <= tk) {
        m_state = m_events.front();
        m_events.pop_front();
      }
      level += levelAt(m_state, tk);
    }
    level /= kSub;
    // Remove the DC component of a speaker that is left switched on.
    m_filtered += (level - m_filtered) * 0.0005f;
    const float out = (level - m_filtered) * kVolume;
    buffer[i * 2 + 0] += out;
    buffer[i * 2 + 1] += out;
  }
  m_renderTime = end;
  if (m_events.empty() && !m_state.data && std::fabs(m_filtered) < 1e-4f) {
    m_filtered = 0.0f;
    m_active = false;
  }
  // Drop events that are too old to ever be rendered.
  while (m_events.size() > 100000)
    m_events.pop_front();
}

} // namespace fador::hw::audio
