#include "AdLib.hpp"
#include "../../utils/Logger.hpp"
#include "ymfm_opl.h"
#include <algorithm>
#include <cmath>

namespace fador::hw::audio {

// Offset from register base (0x20, 0x40, etc) to operator (0 to 17)
const int AdLib::opOffsets[18] = {
    0x00, 0x01, 0x02, 0x08, 0x09, 0x0A, 0x10, 0x11, 0x12, // Slot 1 (Modulators)
    0x03, 0x04, 0x05, 0x0B, 0x0C, 0x0D, 0x13, 0x14, 0x15  // Slot 2 (Carriers)
};

namespace {
constexpr double kOplClock = 14318180.0; // YMF262 master clock
}

// ymfm glue: the chip asks us to run its two timers; they are counted in
// emulated seconds and expire through engine_timer_expired().
class OplChip : public ymfm::ymfm_interface {
public:
  OplChip() : chip(*this) { chip.reset(); }

  void ymfm_set_timer(uint32_t tnum, int32_t duration) override {
    if (tnum >= 2)
      return;
    if (duration < 0) {
      timerActive[tnum] = false;
    } else {
      timerActive[tnum] = true;
      // Keep any overshoot from the expiry that re-armed the timer.
      timerLeft[tnum] = duration / kOplClock + carry[tnum];
    }
  }

  void advance(double dt) {
    for (uint32_t t = 0; t < 2; ++t) {
      if (!timerActive[t])
        continue;
      timerLeft[t] -= dt;
      int guard = 0;
      while (timerActive[t] && timerLeft[t] <= 0 && guard++ < 1000) {
        carry[t] = timerLeft[t];
        timerActive[t] = false;
        m_engine->engine_timer_expired(t); // may re-arm via ymfm_set_timer
        carry[t] = 0;
      }
    }
  }

  ymfm::ymf262 chip;
  std::array<bool, 2> timerActive{};
  std::array<double, 2> timerLeft{};
  std::array<double, 2> carry{};
};

AdLib::AdLib(float sampleRate, const uint64_t *cpuCycles)
    : m_chip(std::make_unique<OplChip>()), m_sampleRate(sampleRate),
      m_cpuCycles(cpuCycles) {
  m_chipRate = m_chip->chip.sample_rate(static_cast<uint32_t>(kOplClock));
  if (m_cpuCycles)
    m_lastTimerCycles = *m_cpuCycles;
  LOG_INFO("AdLib/OPL3 (ymfm YMF262) initialized. Output rate: ", sampleRate,
           " Hz, chip rate: ", m_chipRate, " Hz");
}

AdLib::~AdLib() = default;

void AdLib::advanceFromCycles() {
  if (!m_cpuCycles)
    return;
  const uint64_t now = *m_cpuCycles;
  if (now != m_lastTimerCycles) {
    m_chip->advance(static_cast<double>(now - m_lastTimerCycles) / CPU_FREQ);
    m_lastTimerCycles = now;
  }
}

uint8_t AdLib::read8(uint16_t port) {
  switch (port & 3) {
  case 0:
  case 2:
    advanceFromCycles();
    m_chip->advance(kIoReadTime);
    return m_chip->chip.read_status();
  default:
    return 0xFF;
  }
}

void AdLib::write8(uint16_t port, uint8_t value) {
  switch (port & 3) {
  case 0:
    m_address = value;
    break;
  case 2:
    m_address = static_cast<uint16_t>(0x100 | value);
    break;
  default:
    writeRegister(m_address, value);
    break;
  }
}

void AdLib::writeRegister(uint16_t reg, uint8_t val) {
  reg &= 0x1FF;
  m_registers[reg] = val;
  if (reg & 0x100)
    m_chip->chip.write_address_hi(static_cast<uint8_t>(reg));
  else
    m_chip->chip.write_address(static_cast<uint8_t>(reg));
  m_chip->chip.write_data(val);
}

void AdLib::updateTimers(double dt) {
  advanceFromCycles();
  m_chip->advance(dt);
}

void AdLib::generateSamples(float *buffer, size_t numSamples) {
  // Chip output is 16-bit range; scale so a few loud voices stay below
  // full scale when mixed with digital audio.
  constexpr float kScale = 1.0f / 32768.0f;
  const double step = m_chipRate / m_sampleRate;
  ymfm::ymf262::output_data out;
  for (size_t i = 0; i < numSamples; ++i) {
    m_resamplePos += step;
    while (m_resamplePos >= 1.0) {
      m_resamplePos -= 1.0;
      m_prev = m_next;
      m_chip->chip.generate(&out, 1);
      m_next[0] = static_cast<float>(out.data[0] + out.data[2]) * kScale;
      m_next[1] = static_cast<float>(out.data[1] + out.data[3]) * kScale;
    }
    const float f = static_cast<float>(m_resamplePos);
    buffer[i * 2 + 0] += m_prev[0] + (m_next[0] - m_prev[0]) * f;
    buffer[i * 2 + 1] += m_prev[1] + (m_next[1] - m_prev[1]) * f;
  }
}

} // namespace fador::hw::audio
