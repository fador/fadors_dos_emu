#pragma once

#include "../IODevice.hpp"
#include <cstdint>
#include <functional>
#include <queue>

namespace fador::hw::audio {

class MidiDevice : public IODevice {
public:
  using MidiEventCallback =
      std::function<void(uint8_t channel, uint8_t statusType, uint8_t data1,
                         uint8_t data2)>;

  MidiDevice(uint16_t basePort = 0x330);
  ~MidiDevice() override = default;

  uint8_t read8(uint16_t port) override;
  void write8(uint16_t port, uint8_t value) override;

  void setEventCallback(MidiEventCallback cb) { m_eventCallback = cb; }
  void setIRQCallback(std::function<void()> cb) { m_irqCallback = cb; }

private:
  void handleMidiByte(uint8_t byte);
  void sendEvent(uint8_t channel, uint8_t statusType, uint8_t data1,
                 uint8_t data2);
  void handleCommand(uint8_t cmd);
  void sendAck();
  void enterUartMode();
  void resetDevice();
  void appendReadData(uint8_t byte);

  uint16_t m_basePort;

  // MPU-401 mode
  bool m_uartMode = true;

  // Intelligent mode state
  bool m_expectingParam = false;
  uint8_t m_currentCommand = 0;
  int m_paramsExpected = 0;
  int m_paramsReceived = 0;
  uint8_t m_paramBuf[4] = {};

  // Read queue (for data going to CPU, e.g. ACK, version)
  std::queue<uint8_t> m_readQueue;

  // MIDI parser state
  uint8_t m_runningStatus = 0;
  int m_dataIndex = 0;
  uint8_t m_midiBuf[2] = {};
  int m_midiDataLen = 0;

  // SysEx state
  bool m_sysexActive = false;

  std::function<void()> m_irqCallback;
  MidiEventCallback m_eventCallback;
};

} // namespace fador::hw::audio
