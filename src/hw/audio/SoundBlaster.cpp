#include "SoundBlaster.hpp"
#include "../../utils/Logger.hpp"

namespace fador::hw::audio {

SoundBlaster::SoundBlaster(fador::memory::MemoryBus &memory, DMA8237 &dma,
                           float systemSampleRate)
    : m_memory(memory), m_dma(dma), m_systemSampleRate(systemSampleRate) {
  LOG_INFO("SoundBlaster DSP initialized at base port ", std::hex, m_basePort);
}

uint8_t SoundBlaster::read8(uint16_t port) {
  if (port == m_basePort + 0xA) { // DSP Read Data (0x22A)
    if (!m_readQueue.empty()) {
      uint8_t val = m_readQueue.front();
      m_readQueue.pop();
      return val;
    }
    return 0xFF;
  } else if (port == m_basePort + 0xC) { // DSP Write Status (0x22C)
    return 0x00;                         // Bit 7 clear = not busy
  } else if (port == m_basePort + 0xE) { // DSP Read-Buffer Status / IRQ Ack (0x22E)
    m_irqPending = false; // Reading port 0x22E acknowledges the SB IRQ
    return m_readQueue.empty() ? 0x00 : 0x80;
  } else if (port == m_basePort + 0x4) { // Mixer Address (0x224)
    return m_mixerIndex;
  } else if (port == m_basePort + 0x5) { // Mixer Data (0x225)
    return m_mixerRegs[m_mixerIndex];
  }
  return 0xFF;
}

void SoundBlaster::write8(uint16_t port, uint8_t value) {
  if (port == m_basePort + 0x4) { // Mixer Address (0x224)
    m_mixerIndex = value;
  } else if (port == m_basePort + 0x5) { // Mixer Data (0x225)
    m_mixerRegs[m_mixerIndex] = value;
  } else if (port == m_basePort + 0x6) { // DSP Reset (0x226)
    if (value == 1) {
      m_resetState = true;
    } else if (value == 0 && m_resetState) {
      m_resetState = false;
      while (!m_readQueue.empty())
        m_readQueue.pop();
      while (!m_writeQueue.empty())
        m_writeQueue.pop();
      m_readQueue.push(0xAA); // DSP Ready flag
      m_expectedArgs = 0;
      m_dmaActive = false;
    }
  } else if (port == m_basePort + 0xC) { // DSP Write Command/Data
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
        break; // Direct Mode 8-bit DAC play (1 byte of data)
      case 0x14:
        m_expectedArgs = 2;
        break; // Single-cycle DMA (8-bit), expects length (LSB, MSB)
      case 0x1C:
        m_expectedArgs = 0;
        processCommand(m_currentCommand);
        break; // Auto-init DMA (8-bit)
      case 0x40:
        m_expectedArgs = 1;
        break; // Set Time Constant
      case 0x48:
        m_expectedArgs = 2;
        break; // DSP Block size (for auto-init)
      case 0xE1:
        m_expectedArgs = 0;
        processCommand(m_currentCommand);
        break; // Get DSP Version
      default:
        // No arguments needed by default
        processCommand(m_currentCommand);
        break;
      }
    }
  }
}

void SoundBlaster::processCommand(uint8_t cmd) {
  switch (cmd) {
  case 0x10: { // Direct output
    uint8_t sample = m_writeQueue.front();
    m_writeQueue.pop();
    if (m_speakerOn) {
      m_lastSample = ((float)sample - 128.0f) / 128.0f;
    }
    break;
  }
  case 0x14: { // 8-bit output single-cycle DMA
    uint8_t lenLo = m_writeQueue.front();
    m_writeQueue.pop();
    uint8_t lenHi = m_writeQueue.front();
    m_writeQueue.pop();
    m_dmaLength = (lenHi << 8) | lenLo;
    m_dmaBaseLength = m_dmaLength;
    m_autoInitDma = false;
    m_dmaActive = true;
    m_16bit = false;
    break;
  }
  case 0x1C: { // 8-bit output auto-init DMA
    m_autoInitDma = true;
    m_dmaActive = true;
    m_16bit = false;
    break;
  }
  case 0x40: { // Set Time Constant
    uint8_t tc = m_writeQueue.front();
    m_writeQueue.pop();
    // DSP rate = 1000000 / (256 - tc)
    m_dspSampleRate = 1000000.0f / (256.0f - (float)tc);
    LOG_DEBUG("SB Time Constant set to ", (int)tc, " (", m_dspSampleRate,
              " Hz)");
    break;
  }
  case 0x48: { // Set block size
    uint8_t lenLo = m_writeQueue.front();
    m_writeQueue.pop();
    uint8_t lenHi = m_writeQueue.front();
    m_writeQueue.pop();
    m_dmaLength = (lenHi << 8) | lenLo;
    m_dmaBaseLength = m_dmaLength;
    break;
  }
  case 0xD0: // Halt DMA (pause 8-bit)
    m_dmaActive = false;
    break;
  case 0xD1: // Turn on speaker
    m_speakerOn = true;
    break;
  case 0xD3: // Turn off speaker
    m_speakerOn = false;
    break;
  case 0xD4: // Continue DMA (resume 8-bit)
    m_dmaActive = true;
    break;
  case 0xDA: // Exit auto-init 8-bit DMA
    m_autoInitDma = false;
    break;
  case 0xE1:                // Get DSP Version
    m_readQueue.push(0x03); // Return v3.02 (Sound Blaster Pro)
    m_readQueue.push(0x02);
    break;
  case 0x90: // 8-bit high speed auto-init
  case 0x91: // 8-bit high speed single-cycle
    m_autoInitDma = (cmd == 0x90);
    m_dmaActive = true;
    m_16bit = false;
    break;
  default:
    LOG_DEBUG("Unimplemented SB command: ", std::hex, (int)cmd);
    while (!m_writeQueue.empty())
      m_writeQueue.pop();
    break;
  }
}

void SoundBlaster::triggerIRQ() {
  m_irqPending = true;
  if (m_irqCallback) {
    m_irqCallback();
  }
}

void SoundBlaster::generateSamples(float *buffer, size_t numSamples) {
  if (!m_speakerOn || !m_dmaActive) {
    // Output silence or hold last sample
    for (size_t i = 0; i < numSamples; i++) {
      buffer[i * 2 + 0] += m_lastSample * 0.2f; // Left
      buffer[i * 2 + 1] += m_lastSample * 0.2f; // Right
    }
    return;
  }

  // Step the DMA based on output sample rate ratio
  float samplesPerSystemSample = m_dspSampleRate / m_systemSampleRate;

  for (size_t i = 0; i < numSamples; i++) {
    m_sampleAccumulator += samplesPerSystemSample;

    // Fetch new samples from memory if we consumed one DSP sample frame
    while (m_sampleAccumulator >= 1.0f) {
      if (!m_dmaActive)
        break;

      int dmaChannel = m_16bit ? 5 : 1;
      uint32_t addr = m_dma.getChannelAddress(dmaChannel);

      uint8_t dmaSample = m_memory.read8(addr);
      m_lastSample = ((float)dmaSample - 128.0f) / 128.0f;

      bool tc = m_dma.acknowledgeTransfer(dmaChannel, 1);
      if (tc || m_dmaLength == 0) {
        triggerIRQ();
        if (!m_autoInitDma) {
          m_dmaActive = false;
        } else {
          m_dmaLength = m_dmaBaseLength;
        }
      } else {
        m_dmaLength--;
      }

      m_sampleAccumulator -= 1.0f;
    }

    // Output current held sample
    buffer[i * 2 + 0] += m_lastSample * 0.2f;
    buffer[i * 2 + 1] += m_lastSample * 0.2f;
  }
}

} // namespace fador::hw::audio
