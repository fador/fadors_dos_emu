#pragma once
#include "IODevice.hpp"
#include <cstdint>
#include <queue>
#include <utility>

namespace fador::memory { class MemoryBus; }

namespace fador::hw {

class KeyboardController : public IODevice {
public:
    KeyboardController();
    ~KeyboardController() = default;

    uint8_t read8(uint16_t port) override;
    void write8(uint16_t port, uint8_t value) override;

    void pushScancode(uint8_t scancode);
    void pushKey(uint8_t ascii, uint8_t scancode);

    // BIOS-level key access (for INT 16h)
    bool hasKey() const { return !m_keyBuffer.empty(); }
    std::pair<uint8_t, uint8_t> peekKey() const;  // {ascii, scancode}
    std::pair<uint8_t, uint8_t> popKey();          // {ascii, scancode}

    void setMemoryBus(fador::memory::MemoryBus* memory) { m_memory = memory; }

private:
    struct KeyEntry { uint8_t ascii; uint8_t scancode; };

    fador::memory::MemoryBus* m_memory{nullptr};
    std::queue<KeyEntry> m_keyBuffer;
    uint8_t m_status;
    uint8_t m_commandByte;
    uint8_t m_lastCommand{0};
};

} // namespace fador::hw
