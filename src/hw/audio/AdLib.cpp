#include "AdLib.hpp"
#include "../../utils/Logger.hpp"
#include <algorithm>
#include <cmath>

namespace fador::hw::audio {

// Offset from register base (0x20, 0x40, etc) to operator (0 to 17)
const int AdLib::opOffsets[18] = {
    0x00, 0x01, 0x02, 0x08, 0x09, 0x0A, 0x10, 0x11, 0x12, // Slot 1 (Modulators)
    0x03, 0x04, 0x05, 0x0B, 0x0C, 0x0D, 0x13, 0x14, 0x15  // Slot 2 (Carriers)
};

AdLib::AdLib(float sampleRate, const uint64_t *cpuCycles)
    : m_sampleRate(sampleRate), m_cpuCycles(cpuCycles) {
  if (m_cpuCycles)
    m_lastTimerCycles = *m_cpuCycles;
  LOG_INFO("AdLib (OPL2) initialized. Sample rate: ", sampleRate);
}

uint8_t AdLib::read8(uint16_t port) {
  if (port == 0x388) {
    // Advance OPL timers based on elapsed CPU cycles so that polling
    // loops (e.g. DMX OPL detection) see timer expiry without waiting
    // for the audio generation callback.
    if (m_cpuCycles) {
      uint64_t now = *m_cpuCycles;
      uint64_t elapsed = now - m_lastTimerCycles;
      if (elapsed > 0) {
        double dt = static_cast<double>(elapsed) / CPU_FREQ;
        updateTimers(dt);
        m_lastTimerCycles = now;
      }
    }
    return m_status;
  }
  return 0xFF;
}

void AdLib::write8(uint16_t port, uint8_t value) {
  if (port == 0x388) {
    m_currentRegister = value;
  } else if (port == 0x389) {
    writeRegister(m_currentRegister, value);
  }
}

void AdLib::writeRegister(uint8_t reg, uint8_t val) {
  m_registers[reg] = val;

  if (reg == 0x01) {
    m_waveformEnable = (val & 0x20) != 0;
  } else if (reg == 0x02) {
    m_timer1Data = val;
  } else if (reg == 0x03) {
    m_timer2Data = val;
  } else if (reg == 0x04) {
    m_timerControl = val;
    if (val & 0x80) { // IRQ Reset
      m_status = 0;
    } else {
      // Masking and starting timers
    }
  } else if (reg >= 0x20 && reg <= 0x35) {
    for (int i = 0; i < 18; i++) {
      if (reg - 0x20 == opOffsets[i]) {
        int ch = i % 9;
        Operator &op = (i < 9) ? m_channels[ch].op1 : m_channels[ch].op2;
        op.flags20 = val;
        int rawMult = val & 0x0F;
        op.mult = (rawMult == 0) ? 0.5f : (float)rawMult;
      }
    }
  } else if (reg >= 0x40 && reg <= 0x55) {
    for (int i = 0; i < 18; i++) {
      if (reg - 0x40 == opOffsets[i]) {
        int ch = i % 9;
        Operator &op = (i < 9) ? m_channels[ch].op1 : m_channels[ch].op2;
        op.flags40 = val;
        int tl = val & 0x3F;
        op.totalLevel = (float)tl / 63.0f;
      }
    }
  } else if (reg >= 0x60 && reg <= 0x75) {
    for (int i = 0; i < 18; i++) {
      if (reg - 0x60 == opOffsets[i]) {
        int ch = i % 9;
        Operator &op = (i < 9) ? m_channels[ch].op1 : m_channels[ch].op2;
        op.flags60 = val;
        // Attack limits 0.001 to ~10s
        int ar = (val >> 4) & 0x0F;
        op.attackRate =
            (ar == 0) ? 0.0f : 1.0f / (powf(2.0f, (15 - ar)) * 0.001f);
        int dr = val & 0x0F;
        op.decayRate =
            (dr == 0) ? 0.0f : 1.0f / (powf(2.0f, (15 - dr)) * 0.01f);
      }
    }
  } else if (reg >= 0x80 && reg <= 0x95) {
    for (int i = 0; i < 18; i++) {
      if (reg - 0x80 == opOffsets[i]) {
        int ch = i % 9;
        Operator &op = (i < 9) ? m_channels[ch].op1 : m_channels[ch].op2;
        op.flags80 = val;
        int sl = (val >> 4) & 0x0F; // 0=0dB to 15=-93dB
        op.sustainLevel = 1.0f - ((float)sl / 15.0f);
        int rr = val & 0x0F;
        op.releaseRate =
            (rr == 0) ? 0.0f : 1.0f / (powf(2.0f, (15 - rr)) * 0.01f);
      }
    }
  } else if (reg >= 0xA0 && reg <= 0xA8) {
    int ch = reg - 0xA0;
    m_channels[ch].fnum = (m_channels[ch].fnum & 0x0300) | val;
  } else if (reg >= 0xB0 && reg <= 0xB8) {
    int ch = reg - 0xB0;
    m_channels[ch].fnum = (m_channels[ch].fnum & 0x00FF) | ((val & 0x03) << 8);
    m_channels[ch].block = (val >> 2) & 0x07;
    bool keyOn = (val & 0x20) != 0;

    if (keyOn && !m_channels[ch].keyOn) {
      // Key pressed
      m_channels[ch].op1.state = EnvelopeState::ATTACK;
      m_channels[ch].op2.state = EnvelopeState::ATTACK;
      m_channels[ch].op1.phase = 0;
      m_channels[ch].op2.phase = 0;
    } else if (!keyOn && m_channels[ch].keyOn) {
      // Key released
      m_channels[ch].op1.state = EnvelopeState::RELEASE;
      m_channels[ch].op2.state = EnvelopeState::RELEASE;
    }
    m_channels[ch].keyOn = keyOn;
  } else if (reg >= 0xC0 && reg <= 0xC8) {
    int ch = reg - 0xC0;
    m_channels[ch].feedback = (val >> 1) & 0x07;
    m_channels[ch].connection = (val & 0x01) != 0;
  } else if (reg >= 0xE0 && reg <= 0xF5) {
    for (int i = 0; i < 18; i++) {
      if (reg - 0xE0 == opOffsets[i]) {
        int ch = i % 9;
        Operator &op = (i < 9) ? m_channels[ch].op1 : m_channels[ch].op2;
        op.waveform = val & 0x03;
      }
    }
  }
}

void AdLib::updateTimers(double dt) {
  // Sync the cycle-based timer so read8 doesn't double-count this interval
  if (m_cpuCycles)
    m_lastTimerCycles = *m_cpuCycles;

  if (!(m_timerControl & 0x80)) {
    if (!(m_timerControl & 0x40)) { // Timer 1 unmasked
      if (m_timerControl & 0x01) {  // Timer 1 started
        m_timer1Counter += dt;
        double t1Period = 0.00008 * (256 - m_timer1Data); // 80us resolution
        if (m_timer1Counter >= t1Period) {
          m_status |= 0x40; // T1 expired
          m_status |= 0x80; // IRQ
          m_timer1Counter -= t1Period;
        }
      }
    }
    if (!(m_timerControl & 0x20)) { // Timer 2 unmasked
      if (m_timerControl & 0x02) {  // Timer 2 started
        m_timer2Counter += dt;
        double t2Period = 0.00032 * (256 - m_timer2Data); // 320us resolution
        if (m_timer2Counter >= t2Period) {
          m_status |= 0x20; // T2 expired
          m_status |= 0x80; // IRQ
          m_timer2Counter -= t2Period;
        }
      }
    }
  }
}

float AdLib::computePhaseStep(uint16_t fnum, uint8_t block, float mult) {
  // Standard OPL2 frequency formula: F = (FNUM * 50000) / (2^20 / 2^BLOCK)
  // Approximate phase step for given sample rate:
  float freq = (fnum * powf(2.0f, block) * 49716.0f) / 1048576.0f;
  return (freq * mult) / m_sampleRate;
}

void AdLib::updateEnvelope(Operator &op, bool keyOn, double dt, uint16_t fnum,
                           uint8_t block) {
  // Quick approximation of ADSR
  switch (op.state) {
  case EnvelopeState::OFF:
    op.currentLevel = 0.0f;
    break;
  case EnvelopeState::ATTACK:
    if (op.attackRate > 0)
      op.currentLevel += (float)(dt * op.attackRate);
    else
      op.currentLevel = 1.0f;

    if (op.currentLevel >= 1.0f) {
      op.currentLevel = 1.0f;
      op.state = EnvelopeState::DECAY;
    }
    break;
  case EnvelopeState::DECAY:
    if (op.decayRate > 0)
      op.currentLevel -= (float)(dt * op.decayRate);
    if (op.currentLevel <= op.sustainLevel) {
      op.currentLevel = op.sustainLevel;
      op.state = EnvelopeState::SUSTAIN;
    }
    break;
  case EnvelopeState::SUSTAIN:
    op.currentLevel = op.sustainLevel;
    break;
  case EnvelopeState::RELEASE:
    if (op.releaseRate > 0)
      op.currentLevel -= (float)(dt * op.releaseRate);
    else
      op.currentLevel = 0.0f;

    if (op.currentLevel <= 0.0f) {
      op.currentLevel = 0.0f;
      op.state = EnvelopeState::OFF;
    }
    break;
  }
}

// Generates correct sine table mappings based on waveform selection register
float getWaveform(float phase, uint8_t ws, bool wse) {
  float s = sinf(phase * 2.0f * 3.1415926535f);
  if (!wse)
    return s; // standard sine
  switch (ws) {
  case 0:
    return s; // sine
  case 1:
    return (phase >= 0.5f) ? 0.0f : s; // half sine
  case 2:
    return fabsf(s); // absolute sine
  case 3:
    return (phase >= 0.5f) ? 0.0f
                           : fabsf(s); // quarter sine (incorrect but close)
  }
  return s;
}

float AdLib::renderChannel(int ch) {
  Channel &c = m_channels[ch];
  if (c.op2.state == EnvelopeState::OFF && c.op1.state == EnvelopeState::OFF) {
    return 0.0f;
  }

  double dt = 1.0 / m_sampleRate;
  updateEnvelope(c.op1, c.keyOn, dt, c.fnum, c.block);
  updateEnvelope(c.op2, c.keyOn, dt, c.fnum, c.block);

  c.op1.phase += computePhaseStep(c.fnum, c.block, c.op1.mult);
  c.op2.phase += computePhaseStep(c.fnum, c.block, c.op2.mult);
  if (c.op1.phase >= 1.0f)
    c.op1.phase -= 1.0f;
  if (c.op2.phase >= 1.0f)
    c.op2.phase -= 1.0f;

  float op1Out =
      getWaveform(c.op1.phase + (c.op1LastOutput * c.feedback * 0.1f),
                  c.op1.waveform, m_waveformEnable);
  op1Out *= c.op1.currentLevel *
            (1.0f - c.op1.totalLevel); // Simple logarithmic TL approximation
  c.op1LastOutput = op1Out;

  float out = 0.0f;
  if (c.connection) {
    // Additive
    float op2Out = getWaveform(c.op2.phase, c.op2.waveform, m_waveformEnable);
    op2Out *= c.op2.currentLevel * (1.0f - c.op2.totalLevel);
    out = (op1Out + op2Out) * 0.5f;
  } else {
    // FM
    float op2Out = getWaveform(c.op2.phase + op1Out * 0.5f, c.op2.waveform,
                               m_waveformEnable);
    op2Out *= c.op2.currentLevel * (1.0f - c.op2.totalLevel);
    out = op2Out;
  }
  return out;
}

void AdLib::generateSamples(float *buffer, size_t numSamples) {
  for (size_t i = 0; i < numSamples; i++) {
    float mix = 0.0f;
    for (int ch = 0; ch < 9; ch++) {
      mix += renderChannel(ch);
    }
    mix *= 0.1f; // Master volume padding to prevent clipping

    // Output as interleaved stereo
    buffer[i * 2 + 0] += mix;
    buffer[i * 2 + 1] += mix;
    m_time += 1.0 / m_sampleRate;
  }
}

} // namespace fador::hw::audio
