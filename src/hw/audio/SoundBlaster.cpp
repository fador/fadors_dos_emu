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
    m_irqPending = false;                // Reading port 0x22E acknowledges 8-bit IRQ
    return m_readQueue.empty() ? 0x00 : 0x80;
  } else if (port == m_basePort + 0xF) { // 16-bit IRQ Ack (0x22F)
    m_irqPending = false;                // Reading port 0x22F acknowledges 16-bit IRQ
    return 0x00;
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
      case 0x41:
      case 0x42:
        m_expectedArgs = 2;
        break; // Set Sample Rate (SB16)
      case 0x48:
        m_expectedArgs = 2;
        break; // DSP Block size (for auto-init)
      case 0xB0:
      case 0xB1:
      case 0xB2:
      case 0xB3:
      case 0xB4:
      case 0xB5:
      case 0xB6:
      case 0xB7:
      case 0xB8:
      case 0xB9:
      case 0xBA:
      case 0xBB:
      case 0xBC:
      case 0xBD:
      case 0xBE:
      case 0xBF:
        m_expectedArgs = 3;
        break; // 16-bit DMA
      case 0xC0:
      case 0xC1:
      case 0xC2:
      case 0xC3:
      case 0xC4:
      case 0xC5:
      case 0xC6:
      case 0xC7:
      case 0xC8:
      case 0xC9:
      case 0xCA:
      case 0xCB:
      case 0xCC:
      case 0xCD:
      case 0xCE:
      case 0xCF:
        m_expectedArgs = 3;
        break; // 8-bit DMA (SB16)
      case 0xE1:
        m_expectedArgs = 0;
        processCommand(m_currentCommand);
        break; // Get DSP Version
      case 0xF2:
        m_expectedArgs = 0;
        processCommand(m_currentCommand);
        break; // Force IRQ
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
    LOG_INFO("[SB] DMA Start (Single-cycle) Channel=1 Length=", m_dmaLength);
    break;
  }
  case 0x1C: { // 8-bit output auto-init DMA
    m_autoInitDma = true;
    m_dmaActive = true;
    m_16bit = false;
    LOG_INFO("[SB] DMA Start (Auto-init) Channel=1 Length=", m_dmaBaseLength);
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
    m_readQueue.push(0x04); // Return v4.05 (Sound Blaster 16)
    m_readQueue.push(0x05);
    break;
  case 0x41: // Set Output Sample Rate (SB16)
  case 0x42: { // Set Input Sample Rate (SB16)
    uint8_t hi = m_writeQueue.front();
    m_writeQueue.pop();
    uint8_t lo = m_writeQueue.front();
    m_writeQueue.pop();
    m_dspSampleRate = static_cast<float>((hi << 8) | lo);
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
  case 0xB0:
  case 0xB1:
  case 0xB2:
  case 0xB3:
  case 0xB4:
  case 0xB5:
  case 0xB6:
  case 0xB7:
  case 0xB8:
  case 0xB9:
  case 0xBA:
  case 0xBB:
  case 0xBC:
  case 0xBD:
  case 0xBE:
  case 0xBF: { // 16-bit output
    uint8_t mode = m_writeQueue.front();
    m_writeQueue.pop();
    uint8_t lo = m_writeQueue.front();
    m_writeQueue.pop();
    uint8_t hi = m_writeQueue.front();
    m_writeQueue.pop();

    m_autoInitDma = (cmd & 0x04) != 0;
    m_dmaActive = true;
    m_16bit = true;
    m_stereo = (mode & 0x20) != 0;
    m_dmaBaseLength = (hi << 8) | lo;
    m_dmaLength = m_dmaBaseLength;
    LOG_INFO("[SB] 16-bit DMA Start Mode=0x", std::hex, (int)mode, " Length=", m_dmaLength);
    break;
  }
  case 0xC0:
  case 0xC1:
  case 0xC2:
  case 0xC3:
  case 0xC4:
  case 0xC5:
  case 0xC6:
  case 0xC7:
  case 0xC8:
  case 0xC9:
  case 0xCA:
  case 0xCB:
  case 0xCC:
  case 0xCD:
  case 0xCE:
  case 0xCF: { // 8-bit output (SB16 style)
    uint8_t mode = m_writeQueue.front();
    m_writeQueue.pop();
    uint8_t lo = m_writeQueue.front();
    m_writeQueue.pop();
    uint8_t hi = m_writeQueue.front();
    m_writeQueue.pop();

    m_autoInitDma = (cmd & 0x04) != 0;
    m_dmaActive = true;
    m_16bit = false;
    m_stereo = (mode & 0x20) != 0;
    m_dmaBaseLength = (hi << 8) | lo;
    m_dmaLength = m_dmaBaseLength;
    LOG_INFO("[SB] 8-bit DMA Start Mode=0x", std::hex, (int)mode, " Length=", m_dmaLength);
    break;
  }
  case 0xF2: { // Force IRQ
    triggerIRQ();
    break;
  }
  case 0xD9: // Exit auto-init 16-bit DMA
    m_autoInitDma = false;
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
  float left = m_lastSample;
  float right = m_lastSample;

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

      if (m_16bit) {
        // 16-bit DMA
        int16_t sL = static_cast<int16_t>(m_memory.read16(addr));
        left = static_cast<float>(sL) / 32768.0f;
        m_dma.acknowledgeTransfer(dmaChannel, 2);
        
        if (m_stereo) {
          addr = m_dma.getChannelAddress(dmaChannel);
          int16_t sR = static_cast<int16_t>(m_memory.read16(addr));
          right = static_cast<float>(sR) / 32768.0f;
          m_dma.acknowledgeTransfer(dmaChannel, 2);
        } else {
          right = left;
        }
      } else {
        // 8-bit DMA
        uint8_t sL = m_memory.read8(addr);
        left = (static_cast<float>(sL) - 128.0f) / 128.0f;
        m_dma.acknowledgeTransfer(dmaChannel, 1);

        if (m_stereo) {
          addr = m_dma.getChannelAddress(dmaChannel);
          uint8_t sR = m_memory.read8(addr);
          right = (static_cast<float>(sR) - 128.0f) / 128.0f;
          m_dma.acknowledgeTransfer(dmaChannel, 1);
        } else {
          right = left;
        }
      }
      
      m_lastSample = (left + right) * 0.5f; 

      if (m_dmaLength == 0) {
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

    if (m_dmaActive && m_speakerOn) {
       buffer[i * 2 + 0] += left * 0.8f;
       buffer[i * 2 + 1] += right * 0.8f;
    }
  }
}

} // namespace fador::hw::audio
