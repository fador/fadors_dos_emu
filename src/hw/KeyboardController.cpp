#include "KeyboardController.hpp"
#include "../memory/MemoryBus.hpp"
#include "../utils/Logger.hpp"

namespace fador::hw {

KeyboardController::KeyboardController()
    : m_status(0x1C) // Self-test passed, etc.
      ,
      m_commandByte(0) {}

uint8_t KeyboardController::read8(uint16_t port) {
  if (port == 0x60) { // Data Port — returns hardware scancode
    if (!m_hwScanBuffer.empty()) {
      m_lastScancode = m_hwScanBuffer.front();
      m_hwScanBuffer.pop();
      if (m_hwScanBuffer.empty())
        m_status &= ~0x01;
    }
    return m_lastScancode;
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
        if (a20) {
          LOG_INFO("KBD: A20 Gate Enabled via Output Port write (val: 0x",
                   std::hex, (int)value, ")");
        } else {
          LOG_INFO("KBD: A20 Gate Disabled via Output Port write (val: 0x",
                   std::hex, (int)value, ")");
        }
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
    } else if (value == 0xD0) { // Read Output Port
      // Return output port value: bit 1 is A20, bit 0 is system reset (always
      // 0) Bit 1 = A20, Bit 6 = Keyboard clock, Bit 7 = Keyboard data
      uint8_t outputPort = 0xDF; // Default: A20 enabled, keyboard enabled
      if (m_memory && !m_memory->isA20Enabled()) {
        outputPort &= ~0x02;
      }
      pushScancode(outputPort);
    } else if (value == 0xD1) { // Write Output Port
      m_lastCommand = 0xD1;
    }
  }
}

void KeyboardController::pushScancode(uint8_t scancode) {
  m_hwScanBuffer.push(scancode);
  m_status |= 0x01; // Set Output Buffer Full
}

void KeyboardController::pushKey(uint8_t ascii, uint8_t scancode) {
  m_keyBuffer.push({ascii, scancode});
}

void KeyboardController::pushKeyWithBreak(uint8_t ascii, uint8_t scancode) {
  // Push make code to hardware buffer (for port 0x60 / INT 9)
  m_hwScanBuffer.push(scancode);
  m_status |= 0x01;
  ++m_pendingIRQCount;
  // Push break code to hardware buffer
  m_hwScanBuffer.push(static_cast<uint8_t>(scancode | 0x80));
  ++m_pendingIRQCount;
  // Also push to BIOS buffer so INT 16h still works
  m_keyBuffer.push({ascii, scancode});
}

void KeyboardController::pushMakeKey(uint8_t ascii, uint8_t scancode) {
  m_hwScanBuffer.push(scancode);
  m_status |= 0x01;
  ++m_pendingIRQCount;
  m_keyBuffer.push({ascii, scancode});
}

void KeyboardController::pushBreakKey(uint8_t scancode) {
  m_hwScanBuffer.push(static_cast<uint8_t>(scancode | 0x80));
  m_status |= 0x01;
  ++m_pendingIRQCount;
}

void KeyboardController::pushMakeKeyExtended(uint8_t ascii, uint8_t scancode) {
  // Push 0xE0 prefix byte (triggers its own INT 9)
  m_hwScanBuffer.push(0xE0);
  m_status |= 0x01;
  ++m_pendingIRQCount;
  // Push actual make scancode
  m_hwScanBuffer.push(scancode);
  m_status |= 0x01;
  ++m_pendingIRQCount;
  // Also push to BIOS buffer for INT 16h
  m_keyBuffer.push({ascii, scancode});
}

void KeyboardController::pushBreakKeyExtended(uint8_t scancode) {
  // Push 0xE0 prefix byte
  m_hwScanBuffer.push(0xE0);
  m_status |= 0x01;
  ++m_pendingIRQCount;
  // Push break scancode
  m_hwScanBuffer.push(static_cast<uint8_t>(scancode | 0x80));
  m_status |= 0x01;
  ++m_pendingIRQCount;
}

std::pair<uint8_t, uint8_t> KeyboardController::peekKey() const {
  if (m_keyBuffer.empty())
    return {0, 0};
  return {m_keyBuffer.front().ascii, m_keyBuffer.front().scancode};
}

std::pair<uint8_t, uint8_t> KeyboardController::popKey() {
  if (m_keyBuffer.empty())
    return {0, 0};
  auto entry = m_keyBuffer.front();
  m_keyBuffer.pop();
  if (m_keyBuffer.empty())
    m_status &= ~0x01;
  return {entry.ascii, entry.scancode};
}

} // namespace fador::hw
