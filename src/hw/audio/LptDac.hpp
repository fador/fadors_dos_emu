#pragma once

#include "../ParallelPort.hpp"
#include <array>
#include <cstdint>
#include <deque>
#include <vector>

namespace fador::hw::audio {

/// Renders a DAC whose output level changes at time-stamped instants into
/// 44.1 kHz frames (each frame is the average level over its period, so
/// DACs written faster than the output rate are filtered properly), with
/// an optional DC blocker standing in for the amplifier's AC coupling.
class DacRenderer {
public:
  explicit DacRenderer(float sampleRate = 44100.0f, float gain = 0.5f);

  /// Output level (-1..1) from time nowNs on.
  void setLevel(uint64_t nowNs, float level);
  /// Mixes frames ending at endNs into a stereo buffer.
  void render(float *buffer, size_t frames, uint64_t endNs);
  void setDcBlock(bool on) { m_dcBlock = on; }
  float level() const { return m_pending.empty() ? m_level : m_pending.back().level; }

private:
  struct Event {
    uint64_t ns;
    float level;
  };
  float m_sampleRate;
  float m_gain;
  bool m_dcBlock = true;
  std::deque<Event> m_pending;
  float m_level = 0.0f;   // level at m_renderedNs
  double m_renderedNs = -1.0;
  float m_hpIn = 0.0f, m_hpOut = 0.0f;
  std::vector<float> m_mono, m_stereo;
};

/// Covox Speech Thing: an 8-bit resistor-ladder DAC on the data lines.
/// Every write to the data port sets the output level ((v - 128) / 128)
/// at that instant of emulated time.  The status lines are not connected.
class CovoxDac : public ParallelDevice {
public:
  CovoxDac() = default;
  void dataWritten(uint8_t value, uint64_t nowNs) override;
  uint8_t status(uint8_t data, uint64_t nowNs) override;
  void generateSamples(float *buffer, size_t frames, uint64_t endNs) override;
  std::string describe() const override { return "covox"; }
  DacRenderer &renderer() { return m_dac; }

private:
  DacRenderer m_dac;
};

/// Disney Sound Source: an 8-bit DAC behind a 16-byte FIFO that is played
/// at a fixed 7 kHz.  A sample is put on the data lines and latched into
/// the FIFO by pulsing SELECT IN (control register bit 3 going from 1 to 0,
/// e.g. OUT 0Ch then 04h to base+2).  Status bit 6 (ACK#) is high while
/// the FIFO is full; programs detect the DSS by writing more than 16
/// samples, checking that bit 6 is set, waiting a few milliseconds and
/// checking that it has cleared as the FIFO drained.
class DisneySoundSource : public ParallelDevice {
public:
  static constexpr size_t kFifoSize = 16;
  static constexpr uint32_t kRateHz = 7000;

  DisneySoundSource() = default;
  void controlWritten(uint8_t oldValue, uint8_t newValue, uint8_t data,
                      uint64_t nowNs) override;
  uint8_t status(uint8_t data, uint64_t nowNs) override;
  void generateSamples(float *buffer, size_t frames, uint64_t endNs) override;
  std::string describe() const override { return "disney"; }

  size_t fifoLevel(uint64_t nowNs);
  DacRenderer &renderer() { return m_dac; }
  /// Time of the k-th 7 kHz FIFO clock.
  static uint64_t tickNs(uint64_t k) { return k * 1'000'000 / 7; }

private:
  void sync(uint64_t nowNs);

  DacRenderer m_dac;
  std::array<uint8_t, kFifoSize> m_fifo{};
  size_t m_head = 0, m_count = 0;
  uint64_t m_nextTick = 0; // index of the next FIFO clock
};

} // namespace fador::hw::audio
