#include "SoundBlaster.hpp"
#include "AdLib.hpp"
#include "../../utils/Logger.hpp"
#include <algorithm>
#include <cmath>

namespace fador::hw::audio {

namespace {
// SB16 mixer registers (volumes are 5 bits in bits 7-3)
constexpr uint8_t kMasterL = 0x30, kMasterR = 0x31;
constexpr uint8_t kVoiceL = 0x32, kVoiceR = 0x33;
constexpr uint8_t kFmL = 0x34, kFmR = 0x35;
constexpr uint8_t kCdL = 0x36, kCdR = 0x37;
constexpr uint8_t kLineL = 0x38, kLineR = 0x39;
constexpr uint8_t kMic = 0x3A, kSpeaker = 0x3B;
constexpr uint8_t kOutSwitch = 0x3C;
constexpr size_t kMaxQueuedFrames = 8192; // ~186 ms at 44.1 kHz
} // namespace

SoundBlaster::SoundBlaster(fador::memory::MemoryBus &memory, DMA8237 &dma,
                           float systemSampleRate, uint16_t basePort, int irq,
                           int dmaChannel8, int dmaChannel16)
    : m_memory(memory), m_dma(dma), m_basePort(basePort), m_irq(irq),
      m_dmaChannel8(dmaChannel8), m_dmaChannel16(dmaChannel16),
      m_outputRate(systemSampleRate) {
  resetMixer();
  LOG_INFO("SoundBlaster 16 (DSP 4.05) at ", std::hex, m_basePort, std::dec,
           " IRQ=", m_irq, " DMA8=", m_dmaChannel8, " DMA16=", m_dmaChannel16);
}

// ─── Mixer ──────────────────────────────────────────────────────────────

void SoundBlaster::resetMixer() {
  m_mixer.fill(0);
  // Full volume: games that never touch the mixer must still be audible
  // (the analog output stage of a real card makes up for its -14 dB reset
  // default).
  for (uint8_t r : {kMasterL, kMasterR, kVoiceL, kVoiceR, kFmL, kFmR, kCdL,
                    kCdR, kLineL, kLineR})
    m_mixer[r] = 0xF8;
  m_mixer[kOutSwitch] = 0x1F;
  m_mixer[0x3D] = 0x15;
  m_mixer[0x3E] = 0x0B;
  m_mixer[0x44] = m_mixer[0x45] = m_mixer[0x46] = m_mixer[0x47] = 0x80;
  m_mixer[0x0E] = 0x00; // SB Pro: mono, filter on
}

float SoundBlaster::volumeGain(uint8_t sb16Value) {
  const int v = sb16Value >> 3;
  if (v == 0)
    return 0.0f;
  return std::pow(10.0f, -(31 - v) * 2.0f / 20.0f); // 2 dB steps
}

uint8_t SoundBlaster::readMixer(uint8_t reg) const {
  auto pro = [&](uint8_t l, uint8_t r) {
    return static_cast<uint8_t>((m_mixer[l] & 0xF0) | (m_mixer[r] >> 4));
  };
  switch (reg) {
  case 0x04: return pro(kVoiceL, kVoiceR);
  case 0x0A: return static_cast<uint8_t>(m_mixer[kMic] >> 5);
  case 0x0E: return m_mixer[0x0E];
  case 0x22: return pro(kMasterL, kMasterR);
  case 0x26: return pro(kFmL, kFmR);
  case 0x28: return pro(kCdL, kCdR);
  case 0x2E: return pro(kLineL, kLineR);
  case 0x80: // IRQ select
    switch (m_irq) {
    case 2: case 9: return 0x01;
    case 5: return 0x02;
    case 7: return 0x04;
    case 10: return 0x08;
    default: return 0x00;
    }
  case 0x81: // DMA select
    return static_cast<uint8_t>((1u << m_dmaChannel8) |
                                (m_dmaChannel16 >= 4 ? 1u << m_dmaChannel16 : 0u));
  case 0x82: // IRQ status: 8-bit DMA, 16-bit DMA, MPU-401
    return static_cast<uint8_t>((m_irq8Pending ? 1 : 0) |
                                (m_irq16Pending ? 2 : 0));
  default:
    return m_mixer[reg];
  }
}

void SoundBlaster::writeMixer(uint8_t reg, uint8_t value) {
  // SB Pro registers: high nibble left, low nibble right (4-bit volumes)
  auto pro = [&](uint8_t l, uint8_t r) {
    m_mixer[l] = static_cast<uint8_t>((value & 0xF0) | 0x08);
    m_mixer[r] = static_cast<uint8_t>(((value & 0x0F) << 4) | 0x08);
  };
  switch (reg) {
  case 0x00: resetMixer(); break;
  case 0x04: pro(kVoiceL, kVoiceR); break;
  case 0x0A: m_mixer[kMic] = static_cast<uint8_t>((value & 7) << 5); break;
  case 0x0E: m_mixer[0x0E] = value; break; // bit 1: stereo, bit 5: filter off
  case 0x22: pro(kMasterL, kMasterR); break;
  case 0x26: pro(kFmL, kFmR); break;
  case 0x28: pro(kCdL, kCdR); break;
  case 0x2E: pro(kLineL, kLineR); break;
  case 0x80: case 0x81: case 0x82: break; // jumper settings / status
  default: m_mixer[reg] = value; break;
  }
}

// ─── Port I/O ───────────────────────────────────────────────────────────

uint8_t SoundBlaster::read8(uint16_t port) {
  const uint16_t off = static_cast<uint16_t>(port - m_basePort);
  switch (off) {
  case 0x0: case 0x1: case 0x2: case 0x3:
    return m_fm ? m_fm->read8(static_cast<uint16_t>(0x388 + off)) : 0xFF;
  case 0x8: case 0x9:
    return m_fm ? m_fm->read8(static_cast<uint16_t>(0x388 + off - 8)) : 0xFF;
  case 0x4: return m_mixerIndex;
  case 0x5: return readMixer(m_mixerIndex);
  case 0x6: return 0xFF;
  case 0xA:
    if (!m_readQueue.empty()) {
      m_lastRead = m_readQueue.front();
      m_readQueue.pop();
    }
    return m_lastRead;
  case 0xC: // write buffer status: bit 7 clear = ready for a command
    return 0x00;
  case 0xE: // read buffer status; reading acknowledges the 8-bit IRQ
    m_irq8Pending = false;
    return static_cast<uint8_t>(m_readQueue.empty() ? 0x00 : 0x80);
  case 0xF: // acknowledge the 16-bit IRQ
    m_irq16Pending = false;
    return 0xFF;
  default:
    return 0xFF;
  }
}

void SoundBlaster::write8(uint16_t port, uint8_t value) {
  const uint16_t off = static_cast<uint16_t>(port - m_basePort);
  switch (off) {
  case 0x0: case 0x1: case 0x2: case 0x3:
    if (m_fm)
      m_fm->write8(static_cast<uint16_t>(0x388 + off), value);
    break;
  case 0x8: case 0x9:
    if (m_fm)
      m_fm->write8(static_cast<uint16_t>(0x388 + off - 8), value);
    break;
  case 0x4: m_mixerIndex = value; break;
  case 0x5: writeMixer(m_mixerIndex, value); break;
  case 0x6:
    if (value & 1) {
      m_resetLatch = true;
    } else if (m_resetLatch) {
      m_resetLatch = false;
      dspReset();
    }
    break;
  case 0xC:
    dspWrite(value);
    break;
  default:
    break;
  }
}

// ─── DSP ────────────────────────────────────────────────────────────────

void SoundBlaster::dspReset() {
  std::queue<uint8_t>().swap(m_readQueue);
  m_readQueue.push(0xAA);
  m_argsNeeded = 0;
  m_args.clear();
  m_dmaRunning = false;
  m_dmaPaused = false;
  m_highSpeed = false;
  m_exitAutoInit = false;
  m_silenceLeft = 0;
  m_irq8Pending = false;
  m_irq16Pending = false;
  m_speakerOn = true; // SB16: DMA output does not depend on it anyway
  LOG_DEBUG("[SB] DSP reset");
}

int SoundBlaster::argumentCount(uint8_t cmd) const {
  if ((cmd & 0xF0) == 0xB0 || (cmd & 0xF0) == 0xC0)
    return 3;
  switch (cmd) {
  case 0x10: case 0x38: case 0x40: case 0xE0: case 0xE2: case 0xE4:
  case 0xF9:
    return 1;
  case 0x14: case 0x16: case 0x17: case 0x24: case 0x41: case 0x42:
  case 0x48: case 0x74: case 0x75: case 0x76: case 0x77: case 0x80:
    return 2;
  default:
    return 0;
  }
}

void SoundBlaster::dspWrite(uint8_t value) {
  if (m_argsNeeded > 0) {
    m_args.push_back(value);
    if (--m_argsNeeded == 0)
      processCommand();
    return;
  }
  m_command = value;
  m_args.clear();
  m_argsNeeded = argumentCount(value);
  if (m_argsNeeded == 0)
    processCommand();
}

void SoundBlaster::startDma(bool is16, bool autoInit, bool stereo,
                            bool isSigned, uint32_t length, bool input,
                            bool highSpeed) {
  m_dma16 = is16;
  m_autoInit = autoInit;
  m_stereo = stereo;
  m_signed = isSigned;
  m_input = input;
  m_highSpeed = highSpeed;
  m_blockSize = std::max<uint32_t>(length, 1);
  m_blockLeft = m_blockSize;
  m_dmaRunning = true;
  m_dmaPaused = false;
  m_exitAutoInit = false;
  m_halfRate = false;
  m_frameAcc = 0.0;
  LOG_DEBUG("[SB] DMA ", is16 ? "16" : "8", "-bit ", input ? "input" : "output",
            autoInit ? " auto-init" : " single-cycle", stereo ? " stereo" : "",
            " block=", m_blockSize, " rate=", m_rate);
}

void SoundBlaster::processCommand() {
  const uint8_t cmd = m_command;
  auto arg = [&](size_t i) -> uint32_t { return i < m_args.size() ? m_args[i] : 0; };
  const bool proStereo = (m_mixer[0x0E] & 0x02) != 0;

  if ((cmd & 0xF0) == 0xB0 || (cmd & 0xF0) == 0xC0) {
    // SB16: Bx = 16-bit, Cx = 8-bit; bit 3 = input, bit 2 = auto-init;
    // mode bit 5 = stereo, bit 4 = signed; length = samples - 1.
    const uint8_t mode = static_cast<uint8_t>(arg(0));
    const uint32_t len = (arg(1) | (arg(2) << 8)) + 1;
    startDma((cmd & 0xF0) == 0xB0, (cmd & 0x04) != 0, (mode & 0x20) != 0,
             (mode & 0x10) != 0, len, (cmd & 0x08) != 0, false);
    return;
  }

  switch (cmd) {
  case 0x10: { // direct DAC, 8-bit
    const float v = (static_cast<float>(arg(0)) - 128.0f) / 128.0f;
    m_dacPending.push_back(v);
    if (m_dacPending.size() > 65536)
      m_dacPending.erase(m_dacPending.begin(), m_dacPending.begin() + 32768);
    break;
  }
  case 0x14: // 8-bit single-cycle output
  case 0x16: case 0x17: // ADPCM (played as 8-bit; rarely used)
  case 0x74: case 0x75: case 0x76: case 0x77:
    startDma(false, false, false, false, (arg(0) | (arg(1) << 8)) + 1, false,
             false);
    break;
  case 0x1C: // 8-bit auto-init output
  case 0x1F: case 0x7D: case 0x7F:
    startDma(false, true, proStereo, false, m_legacyBlock, false, false);
    m_halfRate = proStereo;
    break;
  case 0x20: // direct ADC: silence
    m_readQueue.push(0x80);
    break;
  case 0x24: // 8-bit single-cycle input
    startDma(false, false, false, false, (arg(0) | (arg(1) << 8)) + 1, true,
             false);
    break;
  case 0x2C: // 8-bit auto-init input
    startDma(false, true, false, false, m_legacyBlock, true, false);
    break;
  case 0x30: case 0x31: case 0x34: case 0x35: case 0x36: case 0x37:
  case 0x38: // MIDI: not connected
    break;
  case 0x40: { // time constant
    const uint32_t tc = arg(0);
    m_rate = 1000000.0f / static_cast<float>(256 - std::min<uint32_t>(tc, 255));
    break;
  }
  case 0x41: case 0x42: { // SB16 sample rate (high byte first)
    const uint32_t rate = (arg(0) << 8) | arg(1);
    m_rate = static_cast<float>(std::clamp<uint32_t>(rate, 4000, 48000));
    break;
  }
  case 0x45: // continue 8-bit auto-init
  case 0x47: // continue 16-bit auto-init
    m_dmaPaused = false;
    m_exitAutoInit = false;
    break;
  case 0x48: // block size for 0x1C / 0x90 / 0x91
    m_legacyBlock = (arg(0) | (arg(1) << 8)) + 1;
    break;
  case 0x80: // silence for N samples, then IRQ
    m_silenceLeft = (arg(0) | (arg(1) << 8)) + 1;
    m_silenceAcc = 0.0;
    break;
  case 0x90: // 8-bit high-speed auto-init output
    startDma(false, true, proStereo, false, m_legacyBlock, false, true);
    m_halfRate = proStereo;
    break;
  case 0x91: // 8-bit high-speed single-cycle output
    startDma(false, false, proStereo, false, m_legacyBlock, false, true);
    m_halfRate = proStereo;
    break;
  case 0x98: // 8-bit high-speed auto-init input
    startDma(false, true, false, false, m_legacyBlock, true, true);
    break;
  case 0x99: // 8-bit high-speed single-cycle input
    startDma(false, false, false, false, m_legacyBlock, true, true);
    break;
  case 0xA0: case 0xA8: // SB Pro input mode: mono / stereo
    break;
  case 0xD0: case 0xD5: // pause DMA
    m_dmaPaused = true;
    break;
  case 0xD4: case 0xD6: // continue DMA
    m_dmaPaused = false;
    break;
  case 0xD1: m_speakerOn = true; break;
  case 0xD3: m_speakerOn = false; break;
  case 0xD8: m_readQueue.push(m_speakerOn ? 0xFF : 0x00); break;
  case 0xD9: case 0xDA: // exit auto-init after the current block
    m_exitAutoInit = true;
    break;
  case 0xE0: // DSP identification: inverted data byte
    m_readQueue.push(static_cast<uint8_t>(~arg(0)));
    break;
  case 0xE1: // DSP version 4.05 (Sound Blaster 16)
    m_readQueue.push(0x04);
    m_readQueue.push(0x05);
    break;
  case 0xE2: // DMA identification (result goes to memory by DMA; rarely used)
    m_identifyByte = static_cast<uint8_t>(arg(0));
    break;
  case 0xE3: {
    static const char copyright[] = "COPYRIGHT (C) CREATIVE TECHNOLOGY LTD, 1992.";
    for (char c : copyright)
      m_readQueue.push(static_cast<uint8_t>(c)); // includes the NUL
    break;
  }
  case 0xE4: m_testRegister = static_cast<uint8_t>(arg(0)); break;
  case 0xE8: m_readQueue.push(m_testRegister); break;
  case 0xF2: triggerIRQ(false); break;
  case 0xF3: triggerIRQ(true); break;
  case 0xF8: m_readQueue.push(0x00); break;
  case 0xF9: m_readQueue.push(0x00); break;
  case 0xFB: case 0xFC: case 0xFD: m_readQueue.push(0x00); break;
  default:
    LOG_DEBUG("[SB] unhandled DSP command 0x", std::hex, static_cast<int>(cmd));
    break;
  }
}

void SoundBlaster::triggerIRQ(bool is16bit) {
  if (is16bit)
    m_irq16Pending = true;
  else
    m_irq8Pending = true;
  if (m_irqCallback)
    m_irqCallback();
}

// ─── DMA engine and output ──────────────────────────────────────────────

void SoundBlaster::fetchFrame() {
  const int channel = m_dma16 ? m_dmaChannel16 : m_dmaChannel8;
  const int channels = m_stereo ? 2 : 1;
  m_prev = m_cur;
  for (int c = 0; c < channels && m_dmaRunning; ++c) {
    const uint32_t addr = m_dma.getChannelAddress(channel);
    float v = 0.0f;
    if (m_input) {
      if (m_dma16)
        m_memory.write16(addr, 0);
      else
        m_memory.write8(addr, 0x80);
    } else if (m_dma16) {
      uint16_t raw = m_memory.read16(addr);
      if (!m_signed)
        raw ^= 0x8000;
      v = static_cast<float>(static_cast<int16_t>(raw)) / 32768.0f;
    } else {
      uint8_t raw = m_memory.read8(addr);
      if (m_signed)
        raw ^= 0x80;
      v = (static_cast<float>(raw) - 128.0f) / 128.0f;
    }
    m_dma.acknowledgeTransfer(channel, m_dma16 ? 2 : 1);
    if (m_stereo)
      m_cur[c] = v;
    else
      m_cur[0] = m_cur[1] = v;

    if (m_blockLeft > 0)
      --m_blockLeft;
    if (m_blockLeft == 0) {
      triggerIRQ(m_dma16);
      if (m_autoInit && !m_exitAutoInit) {
        m_blockLeft = m_blockSize;
      } else {
        m_dmaRunning = false;
        m_exitAutoInit = false;
      }
    }
  }
  if (m_input)
    m_cur = {0.0f, 0.0f};
}

void SoundBlaster::produce(size_t frames) {
  if (frames == 0)
    return;
  if (m_gainKey != (m_mixer[kMasterL] | m_mixer[kMasterR] << 8 |
                    m_mixer[kVoiceL] << 16 | uint32_t(m_mixer[kVoiceR]) << 24)) {
    m_gainKey = m_mixer[kMasterL] | m_mixer[kMasterR] << 8 |
                m_mixer[kVoiceL] << 16 | uint32_t(m_mixer[kVoiceR]) << 24;
    m_gain[0] = volumeGain(m_mixer[kMasterL]) * volumeGain(m_mixer[kVoiceL]);
    m_gain[1] = volumeGain(m_mixer[kMasterR]) * volumeGain(m_mixer[kVoiceR]);
  }
  const float master[2] = {m_gain[0], m_gain[1]};
  const float voice[2] = {1.0f, 1.0f};
  // Direct DAC writes since the last call are spread over these frames.
  const size_t dacCount = m_dacPending.size();
  for (size_t i = 0; i < frames; ++i) {
    std::array<float, 2> s{0.0f, 0.0f};
    if (m_dmaRunning && !m_dmaPaused) {
      // SB Pro stereo via the legacy commands: the rate is the byte rate.
      const double frameRate = m_halfRate ? m_rate / 2.0 : m_rate;
      m_frameAcc += frameRate / m_outputRate;
      while (m_frameAcc >= 1.0 && m_dmaRunning) {
        m_frameAcc -= 1.0;
        fetchFrame();
      }
      const float f = static_cast<float>(std::min(m_frameAcc, 1.0));
      s[0] = m_prev[0] + (m_cur[0] - m_prev[0]) * f;
      s[1] = m_prev[1] + (m_cur[1] - m_prev[1]) * f;
    }
    if (m_silenceLeft > 0) {
      m_silenceAcc += m_rate / m_outputRate;
      while (m_silenceAcc >= 1.0 && m_silenceLeft > 0) {
        m_silenceAcc -= 1.0;
        if (--m_silenceLeft == 0)
          triggerIRQ(false);
      }
    }
    if (dacCount > 0) {
      m_dacLevel = m_dacPending[std::min(dacCount - 1, i * dacCount / frames)];
      m_dacIdleFrames = 0;
    } else if (m_dacLevel != 0.0f && ++m_dacIdleFrames > 4410) {
      m_dacLevel *= 0.999f; // let a stale DC level from the DAC fade out
      if (std::fabs(m_dacLevel) < 1e-4f)
        m_dacLevel = 0.0f;
    }
    s[0] += m_dacLevel;
    s[1] += m_dacLevel;
    m_out.push_back({s[0] * voice[0] * master[0], s[1] * voice[1] * master[1]});
  }
  m_dacPending.clear();
  while (m_out.size() > kMaxQueuedFrames)
    m_out.pop_front();
}

void SoundBlaster::advance(double seconds) {
  m_timeDriven = true;
  if (seconds <= 0)
    return;
  m_outAcc += seconds * m_outputRate;
  const auto frames = static_cast<size_t>(m_outAcc);
  m_outAcc -= static_cast<double>(frames);
  produce(std::min<size_t>(frames, kMaxQueuedFrames));
}

void SoundBlaster::generateSamples(float *buffer, size_t numSamples) {
  // Without advance() (unit tests) the DSP is clocked by the output.  With
  // it, emulated time alone moves the DSP: an underrun is padded with
  // silence instead of running the DMA ahead of the program's timeline,
  // and a backlog beyond ~100 ms is dropped to keep the latency bounded.
  if (!m_timeDriven && m_out.size() < numSamples)
    produce(numSamples - m_out.size());
  constexpr size_t kMaxLatencyFrames = 4410;
  while (m_out.size() > numSamples + kMaxLatencyFrames)
    m_out.pop_front();
  const size_t available = std::min(numSamples, m_out.size());
  const size_t start = numSamples - available; // pad at the front
  for (size_t i = 0; i < available; ++i) {
    const auto &f = m_out.front();
    buffer[(start + i) * 2 + 0] += f[0];
    buffer[(start + i) * 2 + 1] += f[1];
    m_out.pop_front();
  }
}

} // namespace fador::hw::audio
