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
      m_hwScanBuffer.pop_front();
      // A make code reaching the program: the BIOS-level key it belongs
      // to has now been through INT 9.
      if (!(m_lastScancode & 0x80)) {
        for (auto &k : m_keyBuffer) {
          if (k.hwPending && k.scancode == m_lastScancode) {
            k.hwPending = false;
            break;
          }
        }
      }
      if (m_hwScanBuffer.empty())
        m_status &= ~0x01;
    }
    // Return last known scancode even when buffer is drained
    return m_lastScancode;
  } else if (port == 0x64) { // Status Port
    return m_status;
  } else if (port == 0x61) { // PPI Port B
    // Bit 4 toggles on every few reads to simulate DRAM refresh timing (~15us)
    // Games often use this bit for high-resolution delay loops.
    m_refreshCounter++;
    if ((m_refreshCounter & 0x01) == 0) {
      m_port61 ^= 0x10;
    }
    return m_port61;
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
  } else if (port == 0x61) { // PPI Port B
    // Bits 0-3 and 7 are R/W. Bits 4-6 are read-only.
    m_port61 = (m_port61 & 0x70) | (value & 0x8F);
    if (m_port61Callback)
      m_port61Callback(value);
  }
}

void KeyboardController::pushScancode(uint8_t scancode) {
  if (m_hwScanBuffer.size() >= kMaxHWScanBuffer) {
    m_hwScanBuffer.pop_front(); // Drop oldest to make room (hardware overrun)
  }
  m_hwScanBuffer.push_back(scancode);
  m_status |= 0x01; // Set Output Buffer Full
}

void KeyboardController::pushKey(uint8_t ascii, uint8_t scancode) {
  pushKeyEntry(ascii, scancode, false, false);
}

void KeyboardController::pushKeyEntry(uint8_t ascii, uint8_t scancode,
                                      bool hwPending, bool extended) {
  if (m_keyBuffer.size() >= kMaxKeyBuffer) {
    m_keyBuffer.pop_front(); // Drop oldest (BIOS buffer full)
  }
  m_keyBuffer.push_back({ascii, scancode, hwPending, extended});
}

// INT 16h took a key whose IRQ 1 has not run yet.  On real hardware the
// BIOS INT 9 handler would already have read the make code, so remove it
// from the controller; otherwise it arrives later as a fresh key press, for
// example in an INT 9 handler that a program installs after reading the
// key ("press any key to stop").
void KeyboardController::withdrawMakeCode(const KeyEntry &entry) {
  for (auto it = m_hwScanBuffer.begin(); it != m_hwScanBuffer.end(); ++it) {
    if (*it != entry.scancode)
      continue;
    int removed = 1;
    if (entry.extended && it != m_hwScanBuffer.begin() && *(it - 1) == 0xE0) {
      it = m_hwScanBuffer.erase(it - 1);
      ++removed;
    }
    m_hwScanBuffer.erase(it);
    for (; removed > 0; --removed) {
      if (m_pendingIRQCount > 0)
        --m_pendingIRQCount;
      else // IRQ already raised: its port 60h read sees a key release
        m_lastScancode = static_cast<uint8_t>(entry.scancode | 0x80);
    }
    if (m_hwScanBuffer.empty())
      m_status &= ~0x01;
    return;
  }
}

void KeyboardController::pushKeyWithBreak(uint8_t ascii, uint8_t scancode) {
  // Push make code to hardware buffer (for port 0x60 / INT 9)
  pushScancode(scancode);
  ++m_pendingIRQCount;
  // Push break code to hardware buffer
  pushScancode(static_cast<uint8_t>(scancode | 0x80));
  ++m_pendingIRQCount;
  // Also push to BIOS buffer so INT 16h still works
  LOG_DEBUG("KBD: pushKeyWithBreak sc=0x", std::hex, (int)scancode,
            " ascii=0x", std::hex, (int)ascii);
  pushKeyEntry(ascii, scancode, true, false);
}

void KeyboardController::pushMakeKey(uint8_t ascii, uint8_t scancode) {
  LOG_DEBUG("KBD: pushMakeKey sc=0x", std::hex, (int)scancode,
            " ascii=0x", std::hex, (int)ascii);
  pushScancode(scancode);
  ++m_pendingIRQCount;
  pushKeyEntry(ascii, scancode, true, false);
}

void KeyboardController::pushBreakKey(uint8_t scancode) {
  pushScancode(static_cast<uint8_t>(scancode | 0x80));
  ++m_pendingIRQCount;
}

void KeyboardController::pushMakeKeyExtended(uint8_t ascii, uint8_t scancode) {
  // Push 0xE0 prefix byte (triggers its own INT 9)
  pushScancode(0xE0);
  ++m_pendingIRQCount;
  // Push actual make scancode
  pushScancode(scancode);
  ++m_pendingIRQCount;
  // Also push to BIOS buffer for INT 16h
  LOG_DEBUG("KBD: pushMakeKeyExtended sc=0x", std::hex, (int)scancode,
            " ascii=0x", std::hex, (int)ascii);
  pushKeyEntry(ascii, scancode, true, true);
}

void KeyboardController::pushBreakKeyExtended(uint8_t scancode) {
  // Push 0xE0 prefix byte
  pushScancode(0xE0);
  ++m_pendingIRQCount;
  // Push break scancode
  pushScancode(static_cast<uint8_t>(scancode | 0x80));
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
  m_keyBuffer.pop_front();
  if (entry.hwPending) {
    withdrawMakeCode(entry);
    return {entry.ascii, entry.scancode}; // INT 9 never queued it in the BDA
  }
  // The BIOS INT 9 handler also queued this key in the BDA ring buffer
  // (0040:001E); consume it there too, or programs that test for a key by
  // comparing the BDA head and tail pointers see it forever.
  if (m_memory) {
    const uint16_t head = m_memory->read16(0x41A);
    const uint16_t tail = m_memory->read16(0x41C);
    if (head != tail && head >= 0x1E && head < 0x3E) {
      const uint16_t next = static_cast<uint16_t>(head + 2 >= 0x3E ? 0x1E : head + 2);
      m_memory->write16(0x41A, next);
    }
  }
  // Do NOT clear m_status bit 0 here — that bit reflects the hardware
  // output buffer (port 0x60), not the BIOS key buffer consumed by INT 16h.
  // Clearing it would falsely signal "no data" to programs that poll port 0x64.
  return {entry.ascii, entry.scancode};
}

} // namespace fador::hw
