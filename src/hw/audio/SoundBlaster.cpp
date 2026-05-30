#include "SoundBlaster.hpp"
#include "../../utils/Logger.hpp"
#include <algorithm>

namespace fador::hw::audio {

SoundBlaster::SoundBlaster(fador::memory::MemoryBus &memory, DMA8237 &dma,
                           float systemSampleRate, uint16_t basePort, int irq,
                           int dmaChannel8, int dmaChannel16)
    : m_memory(memory), m_dma(dma), m_systemSampleRate(systemSampleRate),
      m_basePort(basePort), m_irq(irq), m_dmaChannel8(dmaChannel8),
      m_dmaChannel16(dmaChannel16) {
  LOG_INFO("SoundBlaster DSP initialized at base port ", std::hex, m_basePort,
           " IRQ=", std::dec, m_irq, " DMA8=", m_dmaChannel8,
           " DMA16=", m_dmaChannel16);
}

// ─── Mixer register constants ───────────────────────────────────────────
namespace MixerReg {
  constexpr uint8_t RESET           = 0x00;
  constexpr uint8_t DAC_LEVEL       = 0x04;
  constexpr uint8_t VOICE_LEFT      = 0x04; // alias
  constexpr uint8_t MIC_LEVEL       = 0x0A;
  constexpr uint8_t OUTPUT_SWITCH   = 0x0E;
  constexpr uint8_t INPUT_SWITCH    = 0x0F;
  constexpr uint8_t MASTER_LEFT     = 0x22;
  constexpr uint8_t MASTER_RIGHT    = 0x23;
  constexpr uint8_t VOICE_VOL_LEFT  = 0x24; // also 0x04
  constexpr uint8_t VOICE_VOL_RIGHT = 0x25;
  constexpr uint8_t MIDI_LEFT       = 0x26;
  constexpr uint8_t MIDI_RIGHT      = 0x27;
  constexpr uint8_t CD_LEFT         = 0x28;
  constexpr uint8_t CD_RIGHT        = 0x29;
  constexpr uint8_t LINEIN_LEFT     = 0x2E;
  constexpr uint8_t LINEIN_RIGHT    = 0x2F;
  constexpr uint8_t PC_SPEAKER      = 0x30;
  constexpr uint8_t TREBLE_LEFT     = 0x31;
  constexpr uint8_t TREBLE_RIGHT    = 0x32;
  constexpr uint8_t BASS_LEFT       = 0x33;
  constexpr uint8_t BASS_RIGHT      = 0x34;
  constexpr uint8_t RECORD_SOURCE   = 0x36;
  constexpr uint8_t RECORD_GAIN_L   = 0x38;
  constexpr uint8_t RECORD_GAIN_R   = 0x39;
  constexpr uint8_t MIC_GAIN        = 0x3B;
  constexpr uint8_t AGC             = 0x3C;
  constexpr uint8_t IRQ_STATUS      = 0x80;
  constexpr uint8_t DMA_STATUS      = 0x81;
  constexpr uint8_t IRQ_CONFIG      = 0x80;
  constexpr uint8_t DMA_CONFIG      = 0x81;
}

// ─── Mixer volume conversion ────────────────────────────────────────────
// SB16 mixer volumes: 4-bit nibble (0x0–0xF) → 0.0–1.0
float SoundBlaster::mixerVolumeToFloat(uint8_t nibble) {
  return static_cast<float>(nibble & 0x0F) / 15.0f;
}

// ─── Mixer register read/write ──────────────────────────────────────────
uint8_t SoundBlaster::readMixerRegister(uint8_t reg) const {
  switch (reg) {
  case MixerReg::RESET:
    return 0x00;
  case MixerReg::DAC_LEVEL:
    return (m_mixer.voiceLeft << 4) | m_mixer.voiceRight;
  case MixerReg::MIC_LEVEL:
    return m_mixer.micLevel & 0x0F;
  case MixerReg::OUTPUT_SWITCH:
    return m_mixer.outputMixerSwitch;
  case MixerReg::INPUT_SWITCH:
    return m_mixer.inputMixerSwitch;
  case MixerReg::MASTER_LEFT:
    return m_mixer.masterLeft & 0x0F;
  case MixerReg::MASTER_RIGHT:
    return m_mixer.masterRight & 0x0F;
  case MixerReg::VOICE_VOL_LEFT:
    return m_mixer.voiceLeft & 0x0F;
  case MixerReg::VOICE_VOL_RIGHT:
    return m_mixer.voiceRight & 0x0F;
  case MixerReg::MIDI_LEFT:
    return m_mixer.midiLeft & 0x0F;
  case MixerReg::MIDI_RIGHT:
    return m_mixer.midiRight & 0x0F;
  case MixerReg::CD_LEFT:
    return m_mixer.cdLeft & 0x0F;
  case MixerReg::CD_RIGHT:
    return m_mixer.cdRight & 0x0F;
  case MixerReg::LINEIN_LEFT:
    return m_mixer.lineInLeft & 0x0F;
  case MixerReg::LINEIN_RIGHT:
    return m_mixer.lineInRight & 0x0F;
  case MixerReg::PC_SPEAKER:
    return m_mixer.pcSpeaker & 0x0F;
  case MixerReg::TREBLE_LEFT:
    return m_mixer.trebleLeft & 0x0F;
  case MixerReg::TREBLE_RIGHT:
    return m_mixer.trebleRight & 0x0F;
  case MixerReg::BASS_LEFT:
    return m_mixer.bassLeft & 0x0F;
  case MixerReg::BASS_RIGHT:
    return m_mixer.bassRight & 0x0F;
  case MixerReg::RECORD_SOURCE:
    return m_mixer.recordSource;
  case MixerReg::RECORD_GAIN_L:
    return (m_mixer.recordGainLeft & 0x0F) << 4;
  case MixerReg::RECORD_GAIN_R:
    return (m_mixer.recordGainRight & 0x0F) << 4;
  case MixerReg::MIC_GAIN:
    return m_mixer.micGain & 0x0F;
  case MixerReg::AGC:
    return m_mixer.agc & 0x01;
  case MixerReg::IRQ_STATUS: {
    uint8_t status = 0x00;
    if (m_irq8Pending)  status |= 0x01; // bit 0: 8-bit IRQ
    if (m_irq16Pending) status |= 0x02; // bit 1: 16-bit IRQ
    return status;
  }
  default:
    return 0x00;
  }
}

void SoundBlaster::writeMixerRegister(uint8_t reg, uint8_t value) {
  switch (reg) {
  case MixerReg::RESET:
    if (value & 0x01) {
      m_mixer = MixerState{};
      LOG_DEBUG("[SB] Mixer reset to defaults");
    }
    break;
  case MixerReg::DAC_LEVEL:
    m_mixer.voiceLeft  = (value >> 4) & 0x0F;
    m_mixer.voiceRight = value & 0x0F;
    break;
  case MixerReg::MIC_LEVEL:
    m_mixer.micLevel = value & 0x0F;
    break;
  case MixerReg::OUTPUT_SWITCH:
    m_mixer.outputMixerSwitch = value;
    break;
  case MixerReg::INPUT_SWITCH:
    m_mixer.inputMixerSwitch = value;
    break;
  case MixerReg::MASTER_LEFT:
    m_mixer.masterLeft = value & 0x0F;
    break;
  case MixerReg::MASTER_RIGHT:
    m_mixer.masterRight = value & 0x0F;
    break;
  case MixerReg::VOICE_VOL_LEFT:
    m_mixer.voiceLeft = value & 0x0F;
    break;
  case MixerReg::VOICE_VOL_RIGHT:
    m_mixer.voiceRight = value & 0x0F;
    break;
  case MixerReg::MIDI_LEFT:
    m_mixer.midiLeft = value & 0x0F;
    break;
  case MixerReg::MIDI_RIGHT:
    m_mixer.midiRight = value & 0x0F;
    break;
  case MixerReg::CD_LEFT:
    m_mixer.cdLeft = value & 0x0F;
    break;
  case MixerReg::CD_RIGHT:
    m_mixer.cdRight = value & 0x0F;
    break;
  case MixerReg::LINEIN_LEFT:
    m_mixer.lineInLeft = value & 0x0F;
    break;
  case MixerReg::LINEIN_RIGHT:
    m_mixer.lineInRight = value & 0x0F;
    break;
  case MixerReg::PC_SPEAKER:
    m_mixer.pcSpeaker = value & 0x0F;
    break;
  case MixerReg::TREBLE_LEFT:
    m_mixer.trebleLeft = value & 0x0F;
    break;
  case MixerReg::TREBLE_RIGHT:
    m_mixer.trebleRight = value & 0x0F;
    break;
  case MixerReg::BASS_LEFT:
    m_mixer.bassLeft = value & 0x0F;
    break;
  case MixerReg::BASS_RIGHT:
    m_mixer.bassRight = value & 0x0F;
    break;
  case MixerReg::RECORD_SOURCE:
    m_mixer.recordSource = value & 0x07;
    break;
  case MixerReg::RECORD_GAIN_L:
    m_mixer.recordGainLeft = (value >> 4) & 0x0F;
    break;
  case MixerReg::RECORD_GAIN_R:
    m_mixer.recordGainRight = (value >> 4) & 0x0F;
    break;
  case MixerReg::MIC_GAIN:
    m_mixer.micGain = value & 0x0F;
    break;
  case MixerReg::AGC:
    m_mixer.agc = value & 0x01;
    break;
  case MixerReg::IRQ_CONFIG:
  case MixerReg::DMA_CONFIG:
    // SB16 IRQ/DMA config — read-only in practice
    break;
  default:
    LOG_DEBUG("[SB] Write to unknown mixer register 0x", std::hex,
              (int)reg, " = 0x", (int)value);
    break;
  }
}

// ─── Port I/O ───────────────────────────────────────────────────────────
uint8_t SoundBlaster::read8(uint16_t port) {
  uint16_t off = port - m_basePort;

  if (off == 0x04) { // Mixer Address
    return m_mixerIndex;
  } else if (off == 0x05) { // Mixer Data
    return readMixerRegister(m_mixerIndex);
  } else if (off == 0x0A) { // DSP Read Data (read) / DSP Data Write (write)
    if (!m_readQueue.empty()) {
      uint8_t val = m_readQueue.front();
      m_readQueue.pop();
      return val;
    }
    return 0xFF;
  } else if (off == 0x0C) { // DSP Write Status / Read-Buffer mirror
    // Bit 7 = 1 if read data available (for programs that poll 0x22C
    // instead of 0x22E, like WINGS.EXE); also always clear = ready for
    // writing (bit 7 = 0 for write-buffer-empty).
    return m_readQueue.empty() ? 0x00 : 0x80;
  } else if (off == 0x0E) { // Read-Buffer Status / 8-bit IRQ Ack
    m_irq8Pending = false;
    if (m_irqCallback) {
      // Lower the IRQ line
    }
    return m_readQueue.empty() ? 0x00 : 0x80;
  } else if (off == 0x0F) { // 16-bit IRQ Ack
    m_irq16Pending = false;
    return 0x00;
  }
  return 0xFF;
}

void SoundBlaster::write8(uint16_t port, uint8_t value) {
  uint16_t off = port - m_basePort;

  if (off == 0x04) { // Mixer Address
    m_mixerIndex = value;
  } else if (off == 0x05) { // Mixer Data
    writeMixerRegister(m_mixerIndex, value);
  } else if (off == 0x06) { // DSP Reset
    if (value == 1) {
      m_resetState = true;
      LOG_INFO("[SB] Reset Initiated");
    } else if (value == 0 && m_resetState) {
      m_resetState = false;
      while (!m_readQueue.empty())
        m_readQueue.pop();
      while (!m_writeQueue.empty())
        m_writeQueue.pop();
      m_readQueue.push(0xAA); // DSP Ready flag
      LOG_INFO("[SB] Reset Completed - Sent 0xAA");
      m_expectedArgs = 0;
      m_dmaActive = false;
      m_highSpeed = false;
      m_16bit = false;
      m_irq8Pending = false;
      m_irq16Pending = false;
    }
  } else if (off == 0x0A || off == 0x0C) { // DSP Write Command/Data (both ports)
    // Port 0x22A is the alternate command port used by some programs
    // (e.g. WINGS.EXE) instead of the standard 0x22C.
    if (m_expectedArgs > 0) {
      m_writeQueue.push(value);
      m_expectedArgs--;
      if (m_expectedArgs == 0) {
        processCommand(m_currentCommand);
      }
    } else {
      m_currentCommand = value;
      switch (m_currentCommand) {
      case 0x10:
        m_expectedArgs = 1;
        break;
      case 0x14:
        m_expectedArgs = 2;
        break;
      case 0x1C:
        m_expectedArgs = 0;
        processCommand(m_currentCommand);
        break;
      case 0x40:
        m_expectedArgs = 1;
        break;
      case 0x41:
      case 0x42:
        m_expectedArgs = 2;
        break;
      case 0x48:
        m_expectedArgs = 2;
        break;
      case 0xB0: case 0xB1: case 0xB2: case 0xB3:
      case 0xB4: case 0xB5: case 0xB6: case 0xB7:
      case 0xB8: case 0xB9: case 0xBA: case 0xBB:
      case 0xBC: case 0xBD: case 0xBE: case 0xBF:
        m_expectedArgs = 3;
        break;
      case 0xC0: case 0xC1: case 0xC2: case 0xC3:
      case 0xC4: case 0xC5: case 0xC6: case 0xC7:
      case 0xC8: case 0xC9: case 0xCA: case 0xCB:
      case 0xCC: case 0xCD: case 0xCE: case 0xCF:
        m_expectedArgs = 3;
        break;
      case 0x90: case 0x91: case 0x98: case 0x99:
        m_expectedArgs = 0;
        processCommand(m_currentCommand);
        break;
      case 0xE0:
        m_expectedArgs = 0;
        processCommand(m_currentCommand);
        break;
      case 0xE1:
        m_expectedArgs = 0;
        processCommand(m_currentCommand);
        break;
      case 0xE2:
        m_expectedArgs = 0;
        processCommand(m_currentCommand);
        break;
      case 0xE3:
        m_expectedArgs = 0;
        processCommand(m_currentCommand);
        break;
      case 0xE4:
        m_expectedArgs = 1;
        break;
      case 0xE8:
        m_expectedArgs = 0;
        processCommand(m_currentCommand);
        break;
      case 0xF0:
      case 0xF1:
      case 0xF2:
      case 0xF3:
        m_expectedArgs = 0;
        processCommand(m_currentCommand);
        break;
      case 0xF8:
        m_expectedArgs = 0;
        processCommand(m_currentCommand);
        break;
      default:
        processCommand(m_currentCommand);
        break;
      }
    }
  }
}

// ─── Command processing ─────────────────────────────────────────────────
void SoundBlaster::processCommand(uint8_t cmd) {
  switch (cmd) {
  case 0x10: { // Direct DAC output
    uint8_t sample = m_writeQueue.front();
    m_writeQueue.pop();
    if (m_speakerOn) {
      float val = ((float)sample - 128.0f) / 128.0f;
      m_lastSampleL = val;
      m_lastSampleR = val;
    }
    break;
  }
  case 0x14: { // 8-bit single-cycle DMA
    uint8_t lenLo = m_writeQueue.front();
    m_writeQueue.pop();
    uint8_t lenHi = m_writeQueue.front();
    m_writeQueue.pop();
    m_dmaLength = (lenHi << 8) | lenLo;
    m_dmaBaseLength = m_dmaLength;
    m_autoInitDma = false;
    m_dmaActive = true;
    m_16bit = false;
    m_highSpeed = false;
    LOG_INFO("[SB] 8-bit Single-cycle DMA Length=", m_dmaLength);
    break;
  }
  case 0x1C: { // 8-bit auto-init DMA
    m_dmaLength = m_dmaBaseLength;
    m_autoInitDma = true;
    m_dmaActive = true;
    m_16bit = false;
    m_highSpeed = false;
    LOG_INFO("[SB] 8-bit Auto-init DMA Length=", m_dmaBaseLength);
    break;
  }
  case 0x40: { // Set Time Constant
    uint8_t tc = m_writeQueue.front();
    m_writeQueue.pop();
    if (tc == 0) {
      LOG_WARN("[SB] Time constant 0, clamping to 1");
      tc = 1;
    }
    m_dspSampleRate = 1000000.0f / (256.0f - static_cast<float>(tc));
    LOG_DEBUG("[SB] Time Constant ", (int)tc, " -> ", m_dspSampleRate, " Hz");
    break;
  }
  case 0x41: // Set Output Sample Rate (SB16)
  case 0x42: { // Set Input Sample Rate (SB16)
    uint8_t hi = m_writeQueue.front();
    m_writeQueue.pop();
    uint8_t lo = m_writeQueue.front();
    m_writeQueue.pop();
    uint16_t rate = (hi << 8) | lo;
    if (rate < 5000) rate = 5000;
    if (rate > 44100) rate = 44100;
    m_dspSampleRate = static_cast<float>(rate);
    LOG_INFO("[SB] Sample Rate set to ", m_dspSampleRate);
    break;
  }
  case 0x48: { // Set Block size
    uint8_t lo = m_writeQueue.front();
    m_writeQueue.pop();
    uint8_t hi = m_writeQueue.front();
    m_writeQueue.pop();
    m_dmaBaseLength = (hi << 8) | lo;
    m_dmaLength = m_dmaBaseLength;
    LOG_INFO("[SB] Block size set to ", m_dmaBaseLength);
    break;
  }
  case 0xD0: // Halt 8-bit DMA
    m_dmaActive = false;
    break;
  case 0xD1: // Speaker On
    m_speakerOn = true;
    break;
  case 0xD3: // Speaker Off
    m_speakerOn = false;
    break;
  case 0xD4: // Continue 8-bit DMA
    m_dmaActive = true;
    break;
  case 0xD5: // Halt 16-bit DMA
    m_dmaActive = false;
    break;
  case 0xD6: // Continue 16-bit DMA
    m_dmaActive = true;
    break;
  case 0xD8: { // Speaker status
    m_readQueue.push(m_speakerOn ? 0xFF : 0x00);
    break;
  }
  case 0xD9: // Exit auto-init 16-bit DMA
    m_autoInitDma = false;
    break;
  case 0xDA: // Exit auto-init 8-bit DMA
    m_autoInitDma = false;
    break;

  // ── 16-bit DMA commands (SB16) ────────────────────────────────────
  case 0xB0: case 0xB1: case 0xB2: case 0xB3:
  case 0xB4: case 0xB5: case 0xB6: case 0xB7:
  case 0xB8: case 0xB9: case 0xBA: case 0xBB:
  case 0xBC: case 0xBD: case 0xBE: case 0xBF: {
    uint8_t mode = m_writeQueue.front();
    m_writeQueue.pop();
    uint8_t lo = m_writeQueue.front();
    m_writeQueue.pop();
    uint8_t hi = m_writeQueue.front();
    m_writeQueue.pop();

    m_autoInitDma = (cmd & 0x04) != 0;
    m_dmaActive = true;
    m_16bit = true;
    m_highSpeed = false;
    m_stereo = (mode & 0x20) != 0;
    m_dmaBaseLength = (hi << 8) | lo;
    m_dmaLength = m_dmaBaseLength;
    m_sampleAccumulator = 0.0f;
    m_prevSampleL = 0.0f;
    m_prevSampleR = 0.0f;
    LOG_INFO("[SB] 16-bit DMA Mode=0x", std::hex, (int)mode,
             " Length=", std::dec, m_dmaLength);
    break;
  }

  // ── 8-bit DMA commands (SB16) ─────────────────────────────────────
  case 0xC0: case 0xC1: case 0xC2: case 0xC3:
  case 0xC4: case 0xC5: case 0xC6: case 0xC7:
  case 0xC8: case 0xC9: case 0xCA: case 0xCB:
  case 0xCC: case 0xCD: case 0xCE: case 0xCF: {
    uint8_t mode = m_writeQueue.front();
    m_writeQueue.pop();
    uint8_t lo = m_writeQueue.front();
    m_writeQueue.pop();
    uint8_t hi = m_writeQueue.front();
    m_writeQueue.pop();

    m_autoInitDma = (cmd & 0x04) != 0;
    m_dmaActive = true;
    m_16bit = false;
    m_highSpeed = false;
    m_stereo = (mode & 0x20) != 0;
    m_dmaBaseLength = (hi << 8) | lo;
    m_dmaLength = m_dmaBaseLength;
    m_sampleAccumulator = 0.0f;
    m_prevSampleL = 0.0f;
    m_prevSampleR = 0.0f;
    LOG_INFO("[SB] 8-bit DMA Mode=0x", std::hex, (int)mode,
             " Length=", std::dec, m_dmaLength);
    break;
  }

  // ── High-speed DMA ────────────────────────────────────────────────
  case 0x90: { // 8-bit auto-init high-speed
    m_dmaLength = m_dmaBaseLength;
    m_autoInitDma = true;
    m_dmaActive = true;
    m_16bit = false;
    m_highSpeed = true;
    LOG_INFO("[SB] 8-bit Auto-init High-Speed DMA Length=", m_dmaBaseLength);
    break;
  }
  case 0x91: { // 8-bit single-cycle high-speed
    m_dmaLength = m_dmaBaseLength;
    m_autoInitDma = false;
    m_dmaActive = true;
    m_16bit = false;
    m_highSpeed = true;
    LOG_INFO("[SB] 8-bit Single-cycle High-Speed DMA Length=", m_dmaLength);
    break;
  }
  case 0x98: { // 16-bit auto-init high-speed
    m_dmaLength = m_dmaBaseLength;
    m_autoInitDma = true;
    m_dmaActive = true;
    m_16bit = true;
    m_highSpeed = true;
    LOG_INFO("[SB] 16-bit Auto-init High-Speed DMA Length=", m_dmaBaseLength);
    break;
  }
  case 0x99: { // 16-bit single-cycle high-speed
    m_dmaLength = m_dmaBaseLength;
    m_autoInitDma = false;
    m_dmaActive = true;
    m_16bit = true;
    m_highSpeed = true;
    LOG_INFO("[SB] 16-bit Single-cycle High-Speed DMA Length=", m_dmaLength);
    break;
  }

  // ── Identity ──────────────────────────────────────────────────────
  case 0xE0: { // Identify DSP (inverted byte)
    uint8_t inv = m_writeQueue.empty() ? 0 : m_writeQueue.front();
    if (!m_writeQueue.empty()) m_writeQueue.pop();
    m_readQueue.push(~inv);
    break;
  }
  case 0xE1: { // Get DSP Version
    m_readQueue.push(0x04); // v4.05 = SB16
    m_readQueue.push(0x05);
    break;
  }
  case 0xE2: { // DSP identification (SB16)
    // Return 2-byte copyright string fragment
    static const char copyright[] = "COPYRIGHT (C) CREATIVE TECHNOLOGY LTD, 1992.";
    uint16_t idx = 0;
    m_readQueue.push(static_cast<uint8_t>(copyright[idx]));
    m_readQueue.push(static_cast<uint8_t>(copyright[idx + 1]));
    break;
  }
  case 0xE3: { // DSP Copyright string (40 bytes)
    static const char copyright[] = "COPYRIGHT (C) CREATIVE TECHNOLOGY LTD, 1992.";
    for (int i = 0; i < 40; ++i) {
      m_readQueue.push(static_cast<uint8_t>(copyright[i]));
    }
    break;
  }
  case 0xE4: { // Write test register
    uint8_t val = m_writeQueue.front();
    m_writeQueue.pop();
    m_dspTestReg = val;
    break;
  }
  case 0xE8: { // Read test register
    m_readQueue.push(m_dspTestReg);
    break;
  }

  // ── IRQ control ───────────────────────────────────────────────────
  case 0xF0: // Sine generator (undocumented, many SB16 don't support)
    break;
  case 0xF1: // IRQ reset (clear both IRQs)
    m_irq8Pending = false;
    m_irq16Pending = false;
    break;
  case 0xF2: { // Force 8-bit IRQ
    triggerIRQ(false);
    break;
  }
  case 0xF3: { // Force 16-bit IRQ
    triggerIRQ(true);
    break;
  }
  case 0xF8: { // Undocumented — returns 0x00 on SB16
    m_readQueue.push(0x00);
    break;
  }

  default:
    LOG_DEBUG("[SB] Unimplemented command: 0x", std::hex, (int)cmd);
    while (!m_writeQueue.empty())
      m_writeQueue.pop();
    break;
  }
}

// ─── IRQ triggering ─────────────────────────────────────────────────────
void SoundBlaster::triggerIRQ(bool is16bit) {
  if (is16bit) {
    m_irq16Pending = true;
  } else {
    m_irq8Pending = true;
  }
  if (m_irqCallback) {
    m_irqCallback();
  }
}

// ─── Audio generation ───────────────────────────────────────────────────
void SoundBlaster::generateSamples(float *buffer, size_t numSamples) {
  float samplesPerSystemSample = m_dspSampleRate / m_systemSampleRate;

  // Pre-compute mixer volume levels
  float masterL = mixerVolumeToFloat(m_mixer.masterLeft);
  float masterR = mixerVolumeToFloat(m_mixer.masterRight);
  float voiceL  = mixerVolumeToFloat(m_mixer.voiceLeft);
  float voiceR  = mixerVolumeToFloat(m_mixer.voiceRight);

  for (size_t i = 0; i < numSamples; i++) {
    m_sampleAccumulator += samplesPerSystemSample;

    // Fetch new samples from DMA
    while (m_sampleAccumulator >= 1.0f) {
      if (!m_dmaActive)
        break;

      int dmaChannel = m_16bit ? m_dmaChannel16 : m_dmaChannel8;
      uint32_t addr = m_dma.getChannelAddress(dmaChannel);
      uint16_t transferBytes = 0;
      float newL = m_lastSampleL;
      float newR = m_lastSampleR;

      if (m_16bit) {
        if (m_stereo) {
          // SB16 16-bit stereo: word-interleaved (L, R)
          int16_t sL = static_cast<int16_t>(m_memory.read16(addr));
          int16_t sR = static_cast<int16_t>(m_memory.read16(addr + 2));
          newL = static_cast<float>(sL) / 32768.0f;
          newR = static_cast<float>(sR) / 32768.0f;
          transferBytes = 4;
        } else {
          int16_t sL = static_cast<int16_t>(m_memory.read16(addr));
          newL = static_cast<float>(sL) / 32768.0f;
          newR = newL;
          transferBytes = 2;
        }
      } else {
        if (m_stereo) {
          // SB Pro / SB16 8-bit stereo: byte-interleaved (L, R)
          uint8_t sL = m_memory.read8(addr);
          uint8_t sR = m_memory.read8(addr + 1);
          newL = (static_cast<float>(sL) - 128.0f) / 128.0f;
          newR = (static_cast<float>(sR) - 128.0f) / 128.0f;
          transferBytes = 2;
        } else {
          uint8_t sL = m_memory.read8(addr);
          newL = (static_cast<float>(sL) - 128.0f) / 128.0f;
          newR = newL;
          transferBytes = 1;
        }
      }

      // Store previous sample for interpolation
      m_prevSampleL = m_lastSampleL;
      m_prevSampleR = m_lastSampleR;
      m_lastSampleL = newL;
      m_lastSampleR = newR;

      bool tc = m_dma.acknowledgeTransfer(dmaChannel, transferBytes);

      // Decrement DSP block counter by number of sample-words consumed
      // For 16-bit: each channel sample = 1 word; for 8-bit: each = 1 byte
      // The counter counts samples per channel, not bytes
      uint16_t samplesConsumed = 1;
      if (m_16bit && m_stereo) samplesConsumed = 1; // counter counts per channel
      else if (m_stereo) samplesConsumed = 1;        // same for 8-bit stereo

      bool dspTc = false;
      if (m_dmaLength == 0) {
        dspTc = true;
        if (m_autoInitDma) {
          m_dmaLength = m_dmaBaseLength;
        }
      } else {
        m_dmaLength--;
      }

      // Fire IRQ on DMA TC — the DMA channel reached terminal count
      if (tc) {
        triggerIRQ(m_16bit);
        if (!m_autoInitDma) {
          m_dmaActive = false;
        }
      }

      // Fire IRQ on DSP block boundary — independently of DMA TC
      if (dspTc && !tc) {
        triggerIRQ(m_16bit);
      }

      m_sampleAccumulator -= 1.0f;
    }

    if (m_dmaActive && m_speakerOn) {
      // Linear interpolation between previous and current sample
      float frac = m_sampleAccumulator;
      if (frac < 0.0f) frac = 0.0f;
      if (frac > 1.0f) frac = 1.0f;

      float interpL = m_prevSampleL + (m_lastSampleL - m_prevSampleL) * (1.0f - frac);
      float interpR = m_prevSampleR + (m_lastSampleR - m_prevSampleR) * (1.0f - frac);

      // Apply voice volume then master volume
      float outL = interpL * voiceL * masterL;
      float outR = interpR * voiceR * masterR;

      buffer[i * 2 + 0] += outL;
      buffer[i * 2 + 1] += outR;
    }
  }
}

} // namespace fador::hw::audio
