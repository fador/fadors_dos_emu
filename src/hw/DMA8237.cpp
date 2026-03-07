#include "DMA8237.hpp"
#include "../utils/Logger.hpp"

namespace fador::hw {

DMA8237::DMA8237() {
  LOG_INFO("Initialized 8237 DMA Controller (Channels 0-3)");
}

uint8_t DMA8237::read8(uint16_t port) {
  if (port >= 0x00 && port <= 0x07) {
    int ch = port / 2;
    bool isCount = (port % 2) != 0;
    uint16_t val =
        isCount ? m_channels[ch].currentCount : m_channels[ch].currentAddress;

    uint8_t ret = m_flipflop ? (val >> 8) : (val & 0xFF);
    m_flipflop = !m_flipflop;
    return ret;
  } else if (port == 0x08) { // Status register
    // simplified, just returning 0
    return m_status;
  } else if (port >= 0x81 && port <= 0x83) { // Page registers
    int ch = (port == 0x81) ? 2 : (port == 0x82) ? 3 : 1;
    return m_channels[ch].page;
  }
  return 0xFF; // Unhandled
}

void DMA8237::write8(uint16_t port, uint8_t value) {
  if (port >= 0x00 && port <= 0x07) {
    int ch = port / 2;
    bool isCount = (port % 2) != 0;

    if (isCount) {
      if (!m_flipflop) {
        m_channels[ch].baseCount = (m_channels[ch].baseCount & 0xFF00) | value;
      } else {
        m_channels[ch].baseCount =
            (m_channels[ch].baseCount & 0x00FF) | ((uint16_t)value << 8);
        m_channels[ch].currentCount = m_channels[ch].baseCount;
      }
    } else {
      if (!m_flipflop) {
        m_channels[ch].baseAddress =
            (m_channels[ch].baseAddress & 0xFF00) | value;
      } else {
        m_channels[ch].baseAddress =
            (m_channels[ch].baseAddress & 0x00FF) | ((uint16_t)value << 8);
        m_channels[ch].currentAddress = m_channels[ch].baseAddress;
      }
    }
    m_flipflop = !m_flipflop;
  } else if (port == 0x0A) { // Single Mask Register
    int ch = value & 0x03;
    m_channels[ch].mask = (value & 0x04) != 0;
  } else if (port == 0x0B) { // Mode Register
    int ch = value & 0x03;
    m_channels[ch].mode = value;
  } else if (port == 0x0C) { // Clear Byte Pointer (Flip-Flop)
    m_flipflop = false;
  } else if (port == 0x0D) { // Master Clear
    m_flipflop = false;
    m_status = 0;
    for (auto &c : m_channels) {
      c.mask = true;
    }
  } else if (port == 0x0F) { // Write Multiple Mask Register
    m_channels[0].mask = (value & 0x01) != 0;
    m_channels[1].mask = (value & 0x02) != 0;
    m_channels[2].mask = (value & 0x04) != 0;
    m_channels[3].mask = (value & 0x08) != 0;
  } else if (port >= 0x81 && port <= 0x83) { // Page Registers
    int ch = (port == 0x81) ? 2 : (port == 0x82) ? 3 : 1;
    m_channels[ch].page = value;
  }
}

uint32_t DMA8237::getChannelAddress(int channel) {
  if (channel < 0 || channel > 3)
    return 0;
  return ((uint32_t)m_channels[channel].page << 16) |
         m_channels[channel].currentAddress;
}

uint16_t DMA8237::getChannelCount(int channel) {
  if (channel < 0 || channel > 3)
    return 0;
  return m_channels[channel].currentCount;
}

bool DMA8237::acknowledgeTransfer(int channel, uint16_t bytes) {
  if (channel < 0 || channel > 3)
    return false;

  // Address increments or decrements (bit 5 of mode specifies decrement)
  if (m_channels[channel].mode & 0x20) {
    m_channels[channel].currentAddress -= bytes;
  } else {
    m_channels[channel].currentAddress += bytes;
  }

  bool tc = false;
  if (m_channels[channel].currentCount < bytes) {
    m_channels[channel].currentCount = 0xFFFF; // Wraps underflow
    tc = true;
  } else {
    m_channels[channel].currentCount -= bytes;
    if (m_channels[channel].currentCount == 0xFFFF)
      tc = true;
  }

  if (tc) {
    m_status |= (1 << channel); // Set TC bit in status
    if (isAutoInit(channel)) {
      m_channels[channel].currentAddress = m_channels[channel].baseAddress;
      m_channels[channel].currentCount = m_channels[channel].baseCount;
    } else {
      m_channels[channel].mask = true; // Mask out the channel on TC
    }
    return true;
  }
  return false;
}

bool DMA8237::isAutoInit(int channel) {
  if (channel < 0 || channel > 3)
    return false;
  return (m_channels[channel].mode & 0x10) != 0;
}

} // namespace fador::hw
