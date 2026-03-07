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
    // Push make scancode + auto-queue break scancode (for hardware INT 9)
    // Also pushes to BIOS buffer for INT 16h
    void pushKeyWithBreak(uint8_t ascii, uint8_t scancode);

    // Separate make/break for backends with proper key-up events (SDL)
    // pushMakeKey: pushes make scancode to HW buffer + IRQ, and to BIOS buffer
    void pushMakeKey(uint8_t ascii, uint8_t scancode);
    // pushBreakKey: pushes break scancode to HW buffer + IRQ only
    void pushBreakKey(uint8_t scancode);
    // Extended-key variants: push 0xE0 prefix byte before the scancode (for arrow keys, etc.)
    void pushMakeKeyExtended(uint8_t ascii, uint8_t scancode);
    void pushBreakKeyExtended(uint8_t scancode);

    // BIOS-level key access (for INT 16h)
    bool hasKey() const { return !m_keyBuffer.empty(); }
    std::pair<uint8_t, uint8_t> peekKey() const;  // {ascii, scancode}
    std::pair<uint8_t, uint8_t> popKey();          // {ascii, scancode}

    // Hardware IRQ1 pending (for INT 9 injection)
    bool checkPendingIRQ() { if (m_pendingIRQCount > 0) { --m_pendingIRQCount; return true; } return false; }

    void setMemoryBus(fador::memory::MemoryBus* memory) { m_memory = memory; }

private:
    struct KeyEntry { uint8_t ascii; uint8_t scancode; };
    int m_pendingIRQCount{0};

    fador::memory::MemoryBus* m_memory{nullptr};
    std::queue<KeyEntry> m_keyBuffer;         // BIOS-level buffer (for INT 16h)
    std::queue<uint8_t> m_hwScanBuffer;       // Hardware scancode buffer (for port 0x60 / INT 9)
    uint8_t m_lastScancode{0};                // Last scancode read from port 0x60
    uint8_t m_status;
    uint8_t m_commandByte;
    uint8_t m_lastCommand{0};
};

} // namespace fador::hw
