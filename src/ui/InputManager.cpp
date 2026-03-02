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
    uint8_t ascii = 0;

    // Full ASCII-to-PC/XT scancode mapping
    switch (key) {
        case 27:            scancode = 0x01; ascii = 27;  break; // ESC
        case '1': case '!': scancode = 0x02; ascii = static_cast<uint8_t>(key); break;
        case '2': case '@': scancode = 0x03; ascii = static_cast<uint8_t>(key); break;
        case '3': case '#': scancode = 0x04; ascii = static_cast<uint8_t>(key); break;
        case '4': case '$': scancode = 0x05; ascii = static_cast<uint8_t>(key); break;
        case '5': case '%': scancode = 0x06; ascii = static_cast<uint8_t>(key); break;
        case '6': case '^': scancode = 0x07; ascii = static_cast<uint8_t>(key); break;
        case '7': case '&': scancode = 0x08; ascii = static_cast<uint8_t>(key); break;
        case '8': case '*': scancode = 0x09; ascii = static_cast<uint8_t>(key); break;
        case '9': case '(': scancode = 0x0A; ascii = static_cast<uint8_t>(key); break;
        case '0': case ')': scancode = 0x0B; ascii = static_cast<uint8_t>(key); break;
        case '-': case '_': scancode = 0x0C; ascii = static_cast<uint8_t>(key); break;
        case '=': case '+': scancode = 0x0D; ascii = static_cast<uint8_t>(key); break;
        case 8:   case 127: scancode = 0x0E; ascii = 8;   break; // Backspace (127 = DEL on some terminals)
        case 9:             scancode = 0x0F; ascii = 9;   break; // Tab
        case 'q': case 'Q': scancode = 0x10; ascii = static_cast<uint8_t>(key); break;
        case 'w': case 'W': scancode = 0x11; ascii = static_cast<uint8_t>(key); break;
        case 'e': case 'E': scancode = 0x12; ascii = static_cast<uint8_t>(key); break;
        case 'r': case 'R': scancode = 0x13; ascii = static_cast<uint8_t>(key); break;
        case 't': case 'T': scancode = 0x14; ascii = static_cast<uint8_t>(key); break;
        case 'y': case 'Y': scancode = 0x15; ascii = static_cast<uint8_t>(key); break;
        case 'u': case 'U': scancode = 0x16; ascii = static_cast<uint8_t>(key); break;
        case 'i': case 'I': scancode = 0x17; ascii = static_cast<uint8_t>(key); break;
        case 'o': case 'O': scancode = 0x18; ascii = static_cast<uint8_t>(key); break;
        case 'p': case 'P': scancode = 0x19; ascii = static_cast<uint8_t>(key); break;
        case '[': case '{': scancode = 0x1A; ascii = static_cast<uint8_t>(key); break;
        case ']': case '}': scancode = 0x1B; ascii = static_cast<uint8_t>(key); break;
        case 13:  case 10:  scancode = 0x1C; ascii = 13;  break; // Enter
        case 'a': case 'A': scancode = 0x1E; ascii = static_cast<uint8_t>(key); break;
        case 's': case 'S': scancode = 0x1F; ascii = static_cast<uint8_t>(key); break;
        case 'd': case 'D': scancode = 0x20; ascii = static_cast<uint8_t>(key); break;
        case 'f': case 'F': scancode = 0x21; ascii = static_cast<uint8_t>(key); break;
        case 'g': case 'G': scancode = 0x22; ascii = static_cast<uint8_t>(key); break;
        case 'h': case 'H': scancode = 0x23; ascii = static_cast<uint8_t>(key); break;
        case 'j': case 'J': scancode = 0x24; ascii = static_cast<uint8_t>(key); break;
        case 'k': case 'K': scancode = 0x25; ascii = static_cast<uint8_t>(key); break;
        case 'l': case 'L': scancode = 0x26; ascii = static_cast<uint8_t>(key); break;
        case ';': case ':': scancode = 0x27; ascii = static_cast<uint8_t>(key); break;
        case '\'': case '"': scancode = 0x28; ascii = static_cast<uint8_t>(key); break;
        case '`': case '~': scancode = 0x29; ascii = static_cast<uint8_t>(key); break;
        case '\\': case '|': scancode = 0x2B; ascii = static_cast<uint8_t>(key); break;
        case 'z': case 'Z': scancode = 0x2C; ascii = static_cast<uint8_t>(key); break;
        case 'x': case 'X': scancode = 0x2D; ascii = static_cast<uint8_t>(key); break;
        case 'c': case 'C': scancode = 0x2E; ascii = static_cast<uint8_t>(key); break;
        case 'v': case 'V': scancode = 0x2F; ascii = static_cast<uint8_t>(key); break;
        case 'b': case 'B': scancode = 0x30; ascii = static_cast<uint8_t>(key); break;
        case 'n': case 'N': scancode = 0x31; ascii = static_cast<uint8_t>(key); break;
        case 'm': case 'M': scancode = 0x32; ascii = static_cast<uint8_t>(key); break;
        case ',': case '<': scancode = 0x33; ascii = static_cast<uint8_t>(key); break;
        case '.': case '>': scancode = 0x34; ascii = static_cast<uint8_t>(key); break;
        case '/': case '?': scancode = 0x35; ascii = static_cast<uint8_t>(key); break;
        case ' ':           scancode = 0x39; ascii = ' '; break;
        // Arrow keys (extended - no ASCII)
        case 0x148: scancode = 0x48; ascii = 0; break; // Up
        case 0x150: scancode = 0x50; ascii = 0; break; // Down
        case 0x14B: scancode = 0x4B; ascii = 0; break; // Left
        case 0x14D: scancode = 0x4D; ascii = 0; break; // Right
    }

    if (scancode != 0) {
        if (pressed) {
            m_kbd.pushKey(ascii, scancode);
        } else {
            m_kbd.pushKey(0, scancode | 0x80); // Break code
        }
    }
}

} // namespace fador::ui
