#pragma once

#include "IODevice.hpp"
#include <array>
#include <cstdint>

namespace fador::hw {

class DMA8237 : public IODevice {
public:
  DMA8237();
  ~DMA8237() override = default;

  uint8_t read8(uint16_t port) override;
  void write8(uint16_t port, uint8_t value) override;

  // Direct interface for peripheral devices (like Sound Blaster) to do DMA
  // transfers. Returns the current physical memory address for the specified
  // channel.
  uint32_t getChannelAddress(int channel);

  // Returns the current remaining count for the specified channel.
  uint16_t getChannelCount(int channel);

  // Advances the channel state by 'bytes'. Returns true if the channel reached
  // Terminal Count (TC).
  bool acknowledgeTransfer(int channel, uint16_t bytes = 1);

  // Determines if a channel is configured for auto-initialize.
  bool isAutoInit(int channel);

private:
  struct Channel {
    uint16_t baseAddress = 0;
    uint16_t currentAddress = 0;
    uint16_t baseCount = 0;
    uint16_t currentCount = 0;
    uint8_t page = 0;
    uint8_t mode = 0;
    bool mask = true; // initially masked
  };

  std::array<Channel, 8> m_channels; // Channel 0-3 (8-bit), 4-7 (16-bit)
  bool m_flipflop = false;           // low/high byte toggle (slave)
  bool m_flipflopMaster = false;     // low/high byte toggle (master)
  uint8_t m_status = 0;
  uint8_t m_statusMaster = 0;
};

} // namespace fador::hw
