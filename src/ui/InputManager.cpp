#include "InputManager.hpp"
#ifdef _WIN32
#include <conio.h>
#else
#include <unistd.h>
#include <sys/select.h>
#endif
#include <iostream>

namespace fador::ui {

InputManager::InputManager(hw::KeyboardController& kbd)
    : m_kbd(kbd) {
#ifndef _WIN32
    // Save current terminal settings and enable raw (non-canonical, no-echo) mode
    // so we can read individual keypresses without waiting for Enter.
    if (tcgetattr(STDIN_FILENO, &m_origTermios) == 0) {
        struct termios raw = m_origTermios;
        raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0; // non-blocking
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) == 0) {
            m_rawMode = true;
        }
    }
#endif
}

InputManager::~InputManager() {
#ifndef _WIN32
    if (m_rawMode) {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &m_origTermios);
    }
#endif
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
    // POSIX non-blocking: check if stdin has data available with zero timeout
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    struct timeval tv = {0, 0};

    if (select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv) > 0) {
        unsigned char buf[4] = {};
        ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
        if (n > 0) {
            if (n >= 3 && buf[0] == 0x1B && buf[1] == '[') {
                // ANSI escape sequence for arrow keys / navigation
                switch (buf[2]) {
                    case 'A': handleKey(0x148, true); break; // Up
                    case 'B': handleKey(0x150, true); break; // Down
                    case 'C': handleKey(0x14D, true); break; // Right
                    case 'D': handleKey(0x14B, true); break; // Left
                    default:  break;
                }
            } else if (n == 1 && buf[0] == 0x1B) {
                handleKey(27, true); // Lone ESC
            } else {
                handleKey(static_cast<int>(buf[0]), true);
            }
            return true;
        }
    }
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
