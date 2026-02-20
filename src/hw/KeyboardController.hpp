#pragma once
#include "IODevice.hpp"
#include <cstdint>
#include <queue>

namespace fador::hw {

class KeyboardController : public IODevice {
public:
    KeyboardController();
    ~KeyboardController() = default;

    uint8_t read8(uint16_t port) override;
    void write8(uint16_t port, uint8_t value) override;

    void pushScancode(uint8_t scancode);

private:
    std::queue<uint8_t> m_buffer;
    uint8_t m_status;
    uint8_t m_commandByte;
};

} // namespace fador::hw
