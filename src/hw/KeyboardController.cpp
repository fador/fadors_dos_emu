#include "KeyboardController.hpp"
#include "../utils/Logger.hpp"
#include "../memory/MemoryBus.hpp"

namespace fador::hw {

KeyboardController::KeyboardController()
    : m_status(0x1C) // Self-test passed, etc.
    , m_commandByte(0) {
}

uint8_t KeyboardController::read8(uint16_t port) {
    if (port == 0x60) { // Data Port
        if (m_keyBuffer.empty()) return 0;
        uint8_t val = m_keyBuffer.front().scancode;
        m_keyBuffer.pop();
        if (m_keyBuffer.empty()) m_status &= ~0x01; // Clear Output Buffer Full
        return val;
    } else if (port == 0x64) { // Status Port
        return m_status;
    }
    return 0;
}

void KeyboardController::write8(uint16_t port, uint8_t value) {
    if (port == 0x60) { // Data Port
        LOG_DEBUG("KBD: Data write 0x", std::hex, (int)value);
        if (m_lastCommand == 0xD1) {
            bool a20 = (value & 0x02) != 0;
            if (m_memory) {
                m_memory->setA20(a20);
                LOG_INFO("KBD: A20 Gate ", a20 ? "Enabled" : "Disabled", " via Output Port write");
            }
            m_lastCommand = 0;
        }
    } else if (port == 0x64) { // Command Port
        LOG_DEBUG("KBD: Command write 0x", std::hex, (int)value);
        if (value == 0xAA) { // Self-test
            pushScancode(0x55); 
        } else if (value == 0xAD) { // Disable keyboard
            m_status |= 0x10;
        } else if (value == 0xAE) { // Enable keyboard
            m_status &= ~0x10;
        } else if (value == 0xD1) { // Write Output Port
            m_lastCommand = 0xD1;
        }
    }
}

void KeyboardController::pushScancode(uint8_t scancode) {
    m_keyBuffer.push({0, scancode});
    m_status |= 0x01; // Set Output Buffer Full
}

void KeyboardController::pushKey(uint8_t ascii, uint8_t scancode) {
    m_keyBuffer.push({ascii, scancode});
    m_status |= 0x01;
}

std::pair<uint8_t, uint8_t> KeyboardController::peekKey() const {
    if (m_keyBuffer.empty()) return {0, 0};
    return {m_keyBuffer.front().ascii, m_keyBuffer.front().scancode};
}

std::pair<uint8_t, uint8_t> KeyboardController::popKey() {
    if (m_keyBuffer.empty()) return {0, 0};
    auto entry = m_keyBuffer.front();
    m_keyBuffer.pop();
    if (m_keyBuffer.empty()) m_status &= ~0x01;
    return {entry.ascii, entry.scancode};
}

} // namespace fador::hw
