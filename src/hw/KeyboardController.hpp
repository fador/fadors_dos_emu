#pragma once
#include "IODevice.hpp"
#include <cstdint>
#include <queue>

namespace fador::memory { class MemoryBus; }

namespace fador::hw {

class KeyboardController : public IODevice {
public:
    KeyboardController();
    ~KeyboardController() = default;

    uint8_t read8(uint16_t port) override;
    void write8(uint16_t port, uint8_t value) override;

    void pushScancode(uint8_t scancode);

    void setMemoryBus(fador::memory::MemoryBus* memory) { m_memory = memory; }

private:
    fador::memory::MemoryBus* m_memory{nullptr};
    std::queue<uint8_t> m_buffer;
    uint8_t m_status;
    uint8_t m_commandByte;
    uint8_t m_lastCommand{0};
};

} // namespace fador::hw
