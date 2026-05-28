#include "MidiDevice.hpp"
#include "../../utils/Logger.hpp"

namespace fador::hw::audio {

MidiDevice::MidiDevice(uint16_t basePort)
    : m_basePort(basePort) {
  LOG_INFO("MPU-401 MIDI device at ports ", std::hex, m_basePort,
           "-", m_basePort + 1);
}

uint8_t MidiDevice::read8(uint16_t port) {
  if (port == m_basePort + 1) {
    uint8_t status = 0;
    if (m_readQueue.empty())
      status |= 0x80; // bit 7 = no data available
    else
      status &= ~0x80; // data available
    status &= ~0x40;    // bit 6 = ready to receive (always)
    return status;
  } else if (port == m_basePort) {
    if (!m_readQueue.empty()) {
      uint8_t val = m_readQueue.front();
      m_readQueue.pop();
      return val;
    }
    return 0x00;
  }
  return 0xFF;
}

void MidiDevice::write8(uint16_t port, uint8_t value) {
  if (port == m_basePort + 1) {
    if (m_uartMode) {
      if (value == 0xFF) {
        resetDevice();
      }
      return;
    }
    handleCommand(value);
  } else if (port == m_basePort) {
    if (m_uartMode) {
      handleMidiByte(value);
    } else {
      if (m_expectingParam && m_paramsReceived < m_paramsExpected) {
        m_paramBuf[m_paramsReceived++] = value;
        if (m_paramsReceived >= m_paramsExpected) {
          m_expectingParam = false;
          sendAck();
        }
      } else {
        handleMidiByte(value);
      }
    }
  }
}

void MidiDevice::handleMidiByte(uint8_t byte) {
  if (m_sysexActive) {
    if (byte == 0xF7) {
      m_sysexActive = false;
    }
    return;
  }

  if (byte >= 0xF8) {
    return;
  }

  if (byte == 0xF0) {
    m_sysexActive = true;
    return;
  }

  if (byte == 0xF7) {
    return;
  }

  if (byte & 0x80) {
    m_runningStatus = byte;
    m_dataIndex = 0;

    uint8_t statusType = byte & 0xF0;

    switch (statusType) {
    case 0x80: // Note off
    case 0x90: // Note on
    case 0xB0: // Control change
    case 0xE0: // Pitch bend
      m_midiDataLen = 2;
      break;
    case 0xC0: // Program change
    case 0xD0: // Channel pressure
      m_midiDataLen = 1;
      break;
    default:
      m_midiDataLen = 0;
      break;
    }
  } else {
    if (m_runningStatus == 0)
      return;

    m_midiBuf[m_dataIndex++] = byte;

    if (m_dataIndex >= m_midiDataLen) {
      uint8_t channel = m_runningStatus & 0x0F;
      uint8_t statusType = m_runningStatus & 0xF0;
      uint8_t data1 = (m_midiDataLen >= 1) ? m_midiBuf[0] : 0;
      uint8_t data2 = (m_midiDataLen >= 2) ? m_midiBuf[1] : 0;

      // Note-on with velocity 0 is a note-off
      if (statusType == 0x90 && data2 == 0) {
        statusType = 0x80;
      }

      sendEvent(channel, statusType, data1, data2);
      m_dataIndex = 0;
    }
  }
}

void MidiDevice::sendEvent(uint8_t channel, uint8_t statusType, uint8_t data1,
                           uint8_t data2) {
  if (m_eventCallback) {
    m_eventCallback(channel, statusType, data1, data2);
  }
}

void MidiDevice::handleCommand(uint8_t cmd) {
  m_currentCommand = cmd;
  m_paramsExpected = 0;
  m_paramsReceived = 0;

  switch (cmd) {
  case 0x3F: // Enter UART mode
    sendAck();
    enterUartMode();
    return;
  case 0xFF: // Reset
    sendAck();
    resetDevice();
    return;
  case 0x01: // MIDI stop
  case 0x02: // MIDI start
  case 0x03: // MIDI continue
  case 0x15: // Stop all
  case 0x34: // Timing bytes
  case 0x35: // Enable mode messages
  case 0x38: // Enable system common
  case 0x39: // Enable real time
  case 0x3C: // CLS sync
  case 0x3D: // SMPTE sync
  case 0x80: // MIDI sync
  case 0x81: // FSK sync
  case 0x82: // MIDI sync
  case 0x83: // Enable metronome
  case 0x84: // Disable metronome
  case 0x87: // Enable pitch/controller
  case 0x8A: // Disable data in stopped
  case 0x8B: // Enable data in stopped
  case 0x8C: // Disable measure end
  case 0x91: // Enable ext MIDI ctrl
  case 0x94: // Disable clock to host
  case 0x95: // Enable clock to host
  case 0x97: // Enable sysex
  case 0xD0: // ???
  case 0xDF: // ???
    sendAck();
    break;
  case 0xAC: // Get MIDI version
    sendAck();
    appendReadData(0x01); // v1.0
    break;
  case 0xAD: // Get revision
    sendAck();
    appendReadData(0x01); // rev 1
    break;
  case 0xC0: case 0xC1: case 0xC2: case 0xC3:
  case 0xC4: case 0xC5: case 0xC6: case 0xC7:
  case 0xC8: case 0xC9: case 0xCA: case 0xCB:
  case 0xCC: case 0xCD: case 0xCE: case 0xCF:
    sendAck();
    break;
  case 0xE0: // Set tempo
    m_expectingParam = true;
    m_paramsExpected = 1;
    break;
  case 0xE4: // Set clocks per click
    m_expectingParam = true;
    m_paramsExpected = 1;
    break;
  case 0xE6: // Set beats per measure
    m_expectingParam = true;
    m_paramsExpected = 1;
    break;
  case 0xE7: // Send all clocks to host
    sendAck();
    appendReadData(0x04);
    break;
  default:
    sendAck();
    break;
  }
}

void MidiDevice::sendAck() {
  appendReadData(0xFE);
}

void MidiDevice::enterUartMode() {
  m_uartMode = true;
  LOG_DEBUG("[MPU-401] Entered UART mode");
}

void MidiDevice::resetDevice() {
  m_uartMode = true;
  m_expectingParam = false;
  m_paramsExpected = 0;
  m_paramsReceived = 0;
  m_runningStatus = 0;
  m_dataIndex = 0;
  m_midiDataLen = 0;
  m_sysexActive = false;
  while (!m_readQueue.empty())
    m_readQueue.pop();
  sendAck();
  LOG_DEBUG("[MPU-401] Reset");
}

void MidiDevice::appendReadData(uint8_t byte) {
  m_readQueue.push(byte);
}

} // namespace fador::hw::audio
