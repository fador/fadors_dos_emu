#include "LptDac.hpp"
#include "../../utils/simd/Simd.hpp"
#include <cmath>

namespace fador::hw::audio {

namespace {
constexpr size_t kMaxPendingEvents = 1u << 20;
// One-pole DC blocker, cut-off about 7 Hz at 44.1 kHz.
constexpr float kDcPole = 0.999f;

float dacLevel(uint8_t v) { return (static_cast<float>(v) - 128.0f) / 128.0f; }
} // namespace

// ── DacRenderer ───────────────────────────────────────────────────────

DacRenderer::DacRenderer(float sampleRate, float gain)
    : m_sampleRate(sampleRate), m_gain(gain) {}

void DacRenderer::setLevel(uint64_t nowNs, float level) {
  if (m_pending.size() >= kMaxPendingEvents) {
    m_level = m_pending.front().level; // nobody renders: keep the newest
    m_pending.pop_front();
  }
  m_pending.push_back({nowNs, level});
}

void DacRenderer::render(float *buffer, size_t frames, uint64_t endNs) {
  if (frames == 0)
    return;
  const double dt = 1e9 / m_sampleRate;
  const double end = static_cast<double>(endNs);
  const double start = end - static_cast<double>(frames) * dt;
  if (m_renderedNs < 0.0 || m_renderedNs < start - 0.25e9 ||
      m_renderedNs > end) {
    // First call or after a stall: changes before this window are history.
    while (!m_pending.empty() &&
           static_cast<double>(m_pending.front().ns) <= start) {
      m_level = m_pending.front().level;
      m_pending.pop_front();
    }
  }
  m_renderedNs = end;
  if (m_pending.empty() && m_level == 0.0f && m_hpIn == 0.0f &&
      std::fabs(m_hpOut) < 1e-6f) {
    m_hpOut = 0.0f;
    return; // silent
  }

  m_mono.resize(frames);
  for (size_t i = 0; i < frames; ++i) {
    const double t0 = start + static_cast<double>(i) * dt;
    const double t1 = t0 + dt;
    double cur = t0, acc = 0.0;
    while (!m_pending.empty() &&
           static_cast<double>(m_pending.front().ns) < t1) {
      const double et = static_cast<double>(m_pending.front().ns);
      if (et > cur) {
        acc += m_level * (et - cur);
        cur = et;
      }
      m_level = m_pending.front().level;
      m_pending.pop_front();
    }
    acc += m_level * (t1 - cur);
    float x = static_cast<float>(acc / dt);
    if (m_dcBlock) {
      const float y = x - m_hpIn + kDcPole * m_hpOut;
      m_hpIn = x;
      m_hpOut = y;
      x = y;
    }
    m_mono[i] = x;
  }
  if (!m_dcBlock) {
    m_hpIn = 0.0f;
    m_hpOut = 0.0f;
  }
  m_stereo.resize(frames * 2);
  for (size_t i = 0; i < frames; ++i)
    m_stereo[2 * i] = m_stereo[2 * i + 1] = m_mono[i];
  utils::simd::kernels().mixAddF32(buffer, m_stereo.data(), frames * 2,
                                   m_gain);
}

// ── Covox ─────────────────────────────────────────────────────────────

void CovoxDac::dataWritten(uint8_t value, uint64_t nowNs) {
  m_dac.setLevel(nowNs, dacLevel(value));
}

uint8_t CovoxDac::status(uint8_t data, uint64_t nowNs) {
  (void)data;
  (void)nowNs;
  return ParallelPort::kFloatingStatus;
}

void CovoxDac::generateSamples(float *buffer, size_t frames, uint64_t endNs) {
  m_dac.render(buffer, frames, endNs);
}

// ── Disney Sound Source ───────────────────────────────────────────────

void DisneySoundSource::sync(uint64_t nowNs) {
  uint64_t tick = tickNs(m_nextTick);
  while (m_count > 0 && tick <= nowNs) {
    m_dac.setLevel(tick, dacLevel(m_fifo[m_head]));
    m_head = (m_head + 1) % kFifoSize;
    --m_count;
    tick = tickNs(++m_nextTick);
  }
  if (m_count == 0 && tick <= nowNs)
    m_nextTick = nowNs * 7 / 1'000'000 + 1; // the clock runs on, unused
}

size_t DisneySoundSource::fifoLevel(uint64_t nowNs) {
  sync(nowNs);
  return m_count;
}

void DisneySoundSource::controlWritten(uint8_t oldValue, uint8_t newValue,
                                       uint8_t data, uint64_t nowNs) {
  if (!(oldValue & 0x08) || (newValue & 0x08))
    return;
  // SELECT IN pulse: latch the data lines into the FIFO.
  sync(nowNs);
  if (m_count < kFifoSize) {
    m_fifo[(m_head + m_count) % kFifoSize] = data;
    ++m_count;
  }
}

uint8_t DisneySoundSource::status(uint8_t data, uint64_t nowNs) {
  (void)data;
  sync(nowNs);
  return static_cast<uint8_t>(0x07 | (m_count >= kFifoSize ? 0x40 : 0x00));
}

void DisneySoundSource::generateSamples(float *buffer, size_t frames,
                                        uint64_t endNs) {
  sync(endNs);
  m_dac.render(buffer, frames, endNs);
}

} // namespace fador::hw::audio
