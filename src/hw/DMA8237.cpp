#include "DMA8237.hpp"
#include "../utils/Logger.hpp"

namespace fador::hw {

DMA8237::DMA8237() {
  LOG_INFO("Initialized 8237 DMA Controller (Channels 0-7)");
}

uint8_t DMA8237::read8(uint16_t port) {
  // Slave (Ch 0-3)
  if (port >= 0x00 && port <= 0x07) {
    int ch = port / 2;
    bool isCount = (port % 2) != 0;
    uint16_t val = isCount ? m_channels[ch].currentCount : m_channels[ch].currentAddress;
    uint8_t ret = m_flipflop ? (val >> 8) : (val & 0xFF);
    m_flipflop = !m_flipflop;
    return ret;
  } else if (port == 0x08) {
    return m_status;
  }
  // Master (Ch 4-7)
  else if (port >= 0xC0 && port <= 0xCE && (port % 2 == 0)) {
    int ch = 4 + (port - 0xC0) / 4;
    bool isCount = (port % 4) != 0; // Wait, port 0xC0=Addr, 0xC2=Count, 0xC4=Addr...
    // Actually for master: 0xC0=Ch4Addr, 0xC2=Ch4Count, 0xC4=Ch5Addr, 0xC6=Ch5Count...
    ch = 4 + (port - 0xC0) / 4;
    isCount = ((port - 0xC0) % 4) != 0;
    uint16_t val = isCount ? m_channels[ch].currentCount : m_channels[ch].currentAddress;
    uint8_t ret = m_flipflopMaster ? (val >> 8) : (val & 0xFF);
    m_flipflopMaster = !m_flipflopMaster;
    return ret;
  } else if (port == 0xD0) {
    return m_statusMaster;
  }
  // Page registers
  else if (port >= 0x81 && port <= 0x8F) {
    int ch = -1;
    switch (port) {
      case 0x87: ch = 0; break;
      case 0x83: ch = 1; break;
      case 0x81: ch = 2; break;
      case 0x82: ch = 3; break;
      case 0x8B: ch = 5; break;
      case 0x89: ch = 6; break;
      case 0x8A: ch = 7; break;
      case 0x8F: ch = 4; break;
    }
    if (ch != -1) return m_channels[ch].page;
  }
  return 0xFF;
}

void DMA8237::write8(uint16_t port, uint8_t value) {
  // Slave (Ch 0-3)
  if (port >= 0x00 && port <= 0x07) {
    int ch = port / 2;
    bool isCount = (port % 2) != 0;
    if (isCount) {
      if (!m_flipflop) m_channels[ch].baseCount = (m_channels[ch].baseCount & 0xFF00) | value;
      else { m_channels[ch].baseCount = (m_channels[ch].baseCount & 0x00FF) | ((uint16_t)value << 8); m_channels[ch].currentCount = m_channels[ch].baseCount; }
    } else {
      if (!m_flipflop) m_channels[ch].baseAddress = (m_channels[ch].baseAddress & 0xFF00) | value;
      else { m_channels[ch].baseAddress = (m_channels[ch].baseAddress & 0x00FF) | ((uint16_t)value << 8); m_channels[ch].currentAddress = m_channels[ch].baseAddress; }
    }
    m_flipflop = !m_flipflop;
  } else if (port == 0x0A) { // Single Mask
    m_channels[value & 0x03].mask = (value & 0x04) != 0;
  } else if (port == 0x0B) { // Mode
    m_channels[value & 0x03].mode = value;
  } else if (port == 0x0C) { // Clear Flip-Flop
    m_flipflop = false;
  } else if (port == 0x0D) { // Master Clear
    m_flipflop = false; m_status = 0;
    for (int i=0; i<4; i++) m_channels[i].mask = true;
  }
  // Master (Ch 4-7)
  else if (port >= 0xC0 && port <= 0xCE && (port % 2 == 0)) {
    int ch = 4 + (port - 0xC0) / 4;
    bool isCount = ((port - 0xC0) % 4) != 0;
    if (isCount) {
      if (!m_flipflopMaster) m_channels[ch].baseCount = (m_channels[ch].baseCount & 0xFF00) | value;
      else { m_channels[ch].baseCount = (m_channels[ch].baseCount & 0x00FF) | ((uint16_t)value << 8); m_channels[ch].currentCount = m_channels[ch].baseCount; }
    } else {
      if (!m_flipflopMaster) m_channels[ch].baseAddress = (m_channels[ch].baseAddress & 0xFF00) | value;
      else { m_channels[ch].baseAddress = (m_channels[ch].baseAddress & 0x00FF) | ((uint16_t)value << 8); m_channels[ch].currentAddress = m_channels[ch].baseAddress; }
    }
    m_flipflopMaster = !m_flipflopMaster;
  } else if (port == 0xD4) { // Single Mask Master
    m_channels[4 + (value & 0x03)].mask = (value & 0x04) != 0;
  } else if (port == 0xD6) { // Mode Master
    m_channels[4 + (value & 0x03)].mode = value;
  } else if (port == 0xD8) { // Clear Flip-Flop Master
    m_flipflopMaster = false;
  } else if (port == 0xDA) { // Master Clear Master
    m_flipflopMaster = false; m_statusMaster = 0;
    for (int i=4; i<8; i++) m_channels[i].mask = true;
  }
  // Page registers
  else if (port >= 0x81 && port <= 0x8F) {
    int ch = -1;
    switch (port) {
      case 0x87: ch = 0; break;
      case 0x83: ch = 1; break;
      case 0x81: ch = 2; break;
      case 0x82: ch = 3; break;
      case 0x8B: ch = 5; break;
      case 0x89: ch = 6; break;
      case 0x8A: ch = 7; break;
      case 0x8F: ch = 4; break;
    }
    if (ch != -1) m_channels[ch].page = value;
  }
}

uint32_t DMA8237::getChannelAddress(int channel) {
  if (channel < 0 || channel > 7) return 0;
  if (channel >= 4) {
    // 16-bit DMA: address is in units of 16-bit words.
    // Linear address = (page << 16) | (currentAddress << 1).
    // Note: bit 0 of page is ignored/undefined usually, but for some PCs it's part of the address.
    // In AT, page register bit 0 is effectively address bit 16.
    return ((uint32_t)m_channels[channel].page << 16) | (static_cast<uint32_t>(m_channels[channel].currentAddress) << 1);
  }
  return ((uint32_t)m_channels[channel].page << 16) | m_channels[channel].currentAddress;
}

uint16_t DMA8237::getChannelCount(int channel) {
  if (channel < 0 || channel > 7) return 0;
  return m_channels[channel].currentCount;
}

bool DMA8237::acknowledgeTransfer(int channel, uint16_t bytes) {
  if (channel < 0 || channel > 7) return false;
  uint16_t units = (channel >= 4) ? (bytes / 2) : bytes;
  if (units == 0 && bytes > 0) units = 1; // Minimum 1 unit

  if (m_channels[channel].mode & 0x20) m_channels[channel].currentAddress -= units;
  else m_channels[channel].currentAddress += units;

  bool tc = false;
  if (m_channels[channel].currentCount < units) { m_channels[channel].currentCount = 0xFFFF; tc = true; }
  else { m_channels[channel].currentCount -= units; if (m_channels[channel].currentCount == 0xFFFF) tc = true; }

  if (tc) {
    if (channel < 4) m_status |= (1 << channel);
    else m_statusMaster |= (1 << (channel - 4));
    if (isAutoInit(channel)) {
      m_channels[channel].currentAddress = m_channels[channel].baseAddress;
      m_channels[channel].currentCount = m_channels[channel].baseCount;
    } else m_channels[channel].mask = true;
    return true;
  }
  return false;
}

bool DMA8237::isAutoInit(int channel) {
  if (channel < 0 || channel > 7) return false;
  return (m_channels[channel].mode & 0x10) != 0;
}

} // namespace fador::hw
