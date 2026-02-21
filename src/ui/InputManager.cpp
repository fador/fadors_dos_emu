#include "InputManager.hpp"
#ifdef _WIN32
#include <conio.h>
#endif
#include <iostream>

namespace fador::ui {

InputManager::InputManager(hw::KeyboardController& kbd)
    : m_kbd(kbd) {
}

bool InputManager::pollInput() {
#ifdef _WIN32
    if (_kbhit()) {
        int ch = _getch();
        if (ch == 0 || ch == 0xE0) {
            // Extended key
            int ext = _getch();
            handleKey(ext | 0x100, true);
        } else {
            handleKey(ch, true);
        }
        return true;
    }
    return false;
#else
    // TODO: Implement non-blocking input for Linux using termios or ncurses
    return false;
#endif
}

void InputManager::handleKey(int key, bool pressed) {
    uint8_t scancode = 0;
    
    // Simple mapping for demonstration
    // In a full implementation, we'd have a large switch or map
    switch (key) {
        case 27:   scancode = 0x01; break; // ESC
        case '1':  scancode = 0x02; break;
        case '2':  scancode = 0x03; break;
        case 'w':  scancode = 0x11; break;
        case 'a':  scancode = 0x1E; break;
        case 's':  scancode = 0x1F; break;
        case 'd':  scancode = 0x20; break;
        case 13:   scancode = 0x1C; break; // Enter
        case ' ':  scancode = 0x39; break; // Space
        case 8:    scancode = 0x0E; break; // Backspace
        
        // Arrow keys (Extended)
        case 0x148: scancode = 0x48; break; // Up
        case 0x150: scancode = 0x50; break; // Down
        case 0x14B: scancode = 0x4B; break; // Left
        case 0x14D: scancode = 0x4D; break; // Right
    }

    if (scancode != 0) {
        if (pressed) {
            m_kbd.pushScancode(scancode);
        } else {
            m_kbd.pushScancode(scancode | 0x80); // Break code
        }
    }
}

} // namespace fador::ui
