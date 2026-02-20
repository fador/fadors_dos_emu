#include "KeyboardController.hpp"
#include "../utils/Logger.hpp"

namespace fador::hw {

KeyboardController::KeyboardController()
    : m_status(0x1C) // Self-test passed, etc.
    , m_commandByte(0) {
}

uint8_t KeyboardController::read8(uint16_t port) {
    if (port == 0x60) { // Data Port
        if (m_buffer.empty()) return 0;
        uint8_t val = m_buffer.front();
        m_buffer.pop();
        if (m_buffer.empty()) m_status &= ~0x01; // Clear Output Buffer Full
        return val;
    } else if (port == 0x64) { // Status Port
        return m_status;
    }
    return 0;
}

void KeyboardController::write8(uint16_t port, uint8_t value) {
    if (port == 0x60) { // Data Port
        LOG_DEBUG("KBD: Data write 0x", std::hex, (int)value);
    } else if (port == 0x64) { // Command Port
        LOG_DEBUG("KBD: Command write 0x", std::hex, (int)value);
        if (value == 0xAA) { // Self-test
            pushScancode(0x55); 
        } else if (value == 0xAD) { // Disable keyboard
            m_status |= 0x10;
        } else if (value == 0xAE) { // Enable keyboard
            m_status &= ~0x10;
        }
    }
}

void KeyboardController::pushScancode(uint8_t scancode) {
    m_buffer.push(scancode);
    m_status |= 0x01; // Set Output Buffer Full
}

} // namespace fador::hw
