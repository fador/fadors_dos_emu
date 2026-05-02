#include "InputManager.hpp"
#include "../hw/BIOS.hpp"
#include "../utils/Logger.hpp"
#ifdef _WIN32
#include <conio.h>
#else
#include <unistd.h>
#include <sys/select.h>
#endif
#include <iostream>
#include <cstring>

namespace fador::ui {

InputManager::InputManager(hw::KeyboardController& kbd)
    : m_kbd(kbd) {
#ifndef _WIN32
    // Save current terminal settings and enable raw (non-canonical, no-echo) mode
    // so we can read individual keypresses without waiting for Enter.
    if (tcgetattr(STDIN_FILENO, &m_origTermios) == 0) {
        struct termios raw = m_origTermios;
        raw.c_iflag &= ~static_cast<tcflag_t>(ICRNL | IXON | IXOFF | BRKINT | INPCK | ISTRIP);
        raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO | ISIG | IEXTEN);
        raw.c_cc[VMIN]  = 0; // non-blocking
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) == 0) {
            m_rawMode = true;
            // Enable SGR mouse tracking (button events + motion while pressed)
            std::cout << "\033[?1000h" // basic mouse tracking
                      << "\033[?1002h" // motion while button held
                      << "\033[?1006h" // SGR extended mode
                      << std::flush;
        }
    }
#endif
}

InputManager::~InputManager() {
#ifndef _WIN32
    if (m_rawMode) {
        // Disable mouse tracking before restoring terminal
        std::cout << "\033[?1006l"
                  << "\033[?1002l"
                  << "\033[?1000l"
                  << std::flush;
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &m_origTermios);
    }
#endif
}

bool InputManager::pollInput() {
    releaseTimedOutKeys();
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
        unsigned char buf[64] = {};
        ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
        if (n > 0) {
            if (buf[0] == 0x1B && n >= 2) {
                if (buf[1] == '[') {
                    // SGR mouse: ESC [ < Cb ; Cx ; Cy M/m
                    if (n >= 3 && buf[2] == '<') {
                        int cb = 0, cx = 0, cy = 0;
                        int idx = 3, field = 0;
                        while (idx < n && buf[idx] != 'M' && buf[idx] != 'm') {
                            if (buf[idx] == ';') { field++; idx++; continue; }
                            if (buf[idx] >= '0' && buf[idx] <= '9') {
                                if (field == 0) {
                                    if (cb < 1000000) cb = cb * 10 + (buf[idx] - '0');
                                } else if (field == 1) {
                                    if (cx < 1000000) cx = cx * 10 + (buf[idx] - '0');
                                } else if (field == 2) {
                                    if (cy < 1000000) cy = cy * 10 + (buf[idx] - '0');
                                }
                            }
                            idx++;
                        }
                        bool pressed = (idx < n && buf[idx] == 'M');
                        int button = cb & 0x03; // 0=left, 1=middle, 2=right
                        bool isMotion = (cb & 32) != 0;
                        if (isMotion) {
                            // Motion event — update position only
                            handleMouseEvent(-1, cx, cy, false);
                        } else {
                            handleMouseEvent(button, cx, cy, pressed);
                        }
                    }
                    // Arrow keys: ESC [ A/B/C/D
                    else if (n >= 3 && buf[2] >= 'A' && buf[2] <= 'D') {
                        // Arrow keys: ESC [ A/B/C/D
                        switch (buf[2]) {
                            case 'A': handleKey(0x148, true); break; // Up
                            case 'B': handleKey(0x150, true); break; // Down
                            case 'C': handleKey(0x14D, true); break; // Right
                            case 'D': handleKey(0x14B, true); break; // Left
                        }
                    } else if (n >= 3 && buf[2] == 'H') {
                        handleKey(0x147, true); // Home
                    } else if (n >= 3 && buf[2] == 'F') {
                        handleKey(0x14F, true); // End
                    } else if (n >= 4 && buf[n-1] == '~') {
                        // ESC [ <num> ~ sequences
                        int code = 0;
                        for (int i = 2; i < n - 1; ++i) {
                            if (buf[i] >= '0' && buf[i] <= '9') {
                                if (code < 1000000) {
                                    code = code * 10 + (buf[i] - '0');
                                }
                            }
                            else if (buf[i] == ';') break; // modifier follows
                        }
                        switch (code) {
                            case 1:  handleKey(0x147, true); break; // Home
                            case 2:  handleKey(0x152, true); break; // Insert
                            case 3:  handleKey(0x153, true); break; // Delete
                            case 4:  handleKey(0x14F, true); break; // End
                            case 5:  handleKey(0x149, true); break; // PageUp
                            case 6:  handleKey(0x151, true); break; // PageDown
                            case 11: handleKey(0x13B, true); break; // F1
                            case 12: handleKey(0x13C, true); break; // F2
                            case 13: handleKey(0x13D, true); break; // F3
                            case 14: handleKey(0x13E, true); break; // F4
                            case 15: handleKey(0x13F, true); break; // F5
                            case 17: handleKey(0x140, true); break; // F6
                            case 18: handleKey(0x141, true); break; // F7
                            case 19: handleKey(0x142, true); break; // F8
                            case 20: handleKey(0x143, true); break; // F9
                            case 21: handleKey(0x144, true); break; // F10
                            case 23: handleKey(0x185, true); break; // F11
                            case 24: handleKey(0x186, true); break; // F12
                        }
                    }
                } else if (buf[1] == 'O') {
                    // SS3 sequences: ESC O P/Q/R/S (F1-F4 on many terminals)
                    if (n >= 3) {
                        switch (buf[2]) {
                            case 'P': handleKey(0x13B, true); break; // F1
                            case 'Q': handleKey(0x13C, true); break; // F2
                            case 'R': handleKey(0x13D, true); break; // F3
                            case 'S': handleKey(0x13E, true); break; // F4
                            case 'H': handleKey(0x147, true); break; // Home
                            case 'F': handleKey(0x14F, true); break; // End
                        }
                    }
                } else {
                    // ESC + printable character → Alt+key combination
                    // Terminal sends ESC followed by the key for Alt+letter
                    if (n == 2 && buf[1] >= 0x20 && buf[1] < 0x7F) {
                        handleAltKey(buf[1]);
                    } else {
                        // Lone ESC or ESC + something we don't handle
                        handleKey(27, true);
                    }
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

    LOG_DEBUG("InputManager::handleKey key=", key, " pressed=", pressed);

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
        // Ctrl+letter (0x01-0x1A) not caught by explicit cases above
        default:
            if (key >= 1 && key <= 26) {
                static const uint8_t ctrlScancodes[26] = {
                    0x1E,0x30,0x2E,0x20,0x12,0x21,0x22,0x23,0x17,0x24,
                    0x25,0x26,0x32,0x31,0x18,0x19,0x10,0x13,0x1F,0x14,
                    0x16,0x2F,0x11,0x2D,0x15,0x2C
                };
                scancode = ctrlScancodes[key - 1];
                ascii = static_cast<uint8_t>(key);
            }
            break;
        // Arrow keys (extended - no ASCII)
        case 0x148: scancode = 0x48; ascii = 0; break; // Up
        case 0x150: scancode = 0x50; ascii = 0; break; // Down
        case 0x14B: scancode = 0x4B; ascii = 0; break; // Left
        case 0x14D: scancode = 0x4D; ascii = 0; break; // Right
        // Navigation keys (extended - no ASCII)
        case 0x147: scancode = 0x47; ascii = 0; break; // Home
        case 0x14F: scancode = 0x4F; ascii = 0; break; // End
        case 0x149: scancode = 0x49; ascii = 0; break; // PageUp
        case 0x151: scancode = 0x51; ascii = 0; break; // PageDown
        case 0x152: scancode = 0x52; ascii = 0; break; // Insert
        case 0x153: scancode = 0x53; ascii = 0; break; // Delete
        // Function keys (extended - no ASCII)
        case 0x13B: scancode = 0x3B; ascii = 0; break; // F1
        case 0x13C: scancode = 0x3C; ascii = 0; break; // F2
        case 0x13D: scancode = 0x3D; ascii = 0; break; // F3
        case 0x13E: scancode = 0x3E; ascii = 0; break; // F4
        case 0x13F: scancode = 0x3F; ascii = 0; break; // F5
        case 0x140: scancode = 0x40; ascii = 0; break; // F6
        case 0x141: scancode = 0x41; ascii = 0; break; // F7
        case 0x142: scancode = 0x42; ascii = 0; break; // F8
        case 0x143: scancode = 0x43; ascii = 0; break; // F9
        case 0x144: scancode = 0x44; ascii = 0; break; // F10
        case 0x185: scancode = 0x57; ascii = 0; break; // F11
        case 0x186: scancode = 0x58; ascii = 0; break; // F12
    }

    if (scancode != 0 && pressed) {
        bool extended = (key >= 0x100); // extended keys have 0x100 flag
        LOG_DEBUG("InputManager: mapped -> scancode=0x", std::hex, (int)scancode,
                  " ascii=0x", std::hex, (int)ascii, " extended=", extended);
        auto it = m_heldKeys.find(scancode);
        if (it != m_heldKeys.end()) {
            // Key is already held — just refresh the timestamp.
            it->second.pressTime = std::chrono::steady_clock::now();
        } else {
            // New key press — push make code.
            if (extended) {
                m_kbd.pushMakeKeyExtended(ascii, scancode);
            } else {
                m_kbd.pushMakeKey(ascii, scancode);
            }
            m_heldKeys[scancode] = { scancode, extended,
                                     std::chrono::steady_clock::now() };
        }
    }
}

void InputManager::handleMouseEvent(int button, int col, int row, bool pressed) {
    if (!m_bios) return;
    auto& ms = m_bios->mouseState();
    // Terminal coordinates are 1-based; convert to DOS virtual pixel coords
    // In text mode, each cell = 8x8 pixels for INT 33h
    ms.x = static_cast<int16_t>((col - 1) * 8);
    ms.y = static_cast<int16_t>((row - 1) * 8);

    if (button >= 0 && button <= 2) {
        // Map terminal button: 0=left, 1=middle, 2=right
        // DOS button bits: 0=left, 1=right, 2=middle
        int dosBtn = (button == 1) ? 2 : (button == 2) ? 1 : 0;
        if (pressed) {
            ms.buttons |= (1 << dosBtn);
            ms.pressCount[dosBtn]++;
            ms.lastPressX[dosBtn] = ms.x;
            ms.lastPressY[dosBtn] = ms.y;
        } else {
            ms.buttons &= ~(1 << dosBtn);
            ms.releaseCount[dosBtn]++;
            ms.lastReleaseX[dosBtn] = ms.x;
            ms.lastReleaseY[dosBtn] = ms.y;
        }
    }
}

void InputManager::handleAltKey(unsigned char ch) {
    // Alt+letter in DOS returns ascii=0 and a special scan code.
    // Map the printable character to its PC/XT scancode for Alt combinations.
    static const uint8_t altScancodes[26] = {
        0x1E, 0x30, 0x2E, 0x20, 0x12, 0x21, 0x22, 0x23, 0x17, 0x24, // A-J
        0x25, 0x26, 0x32, 0x31, 0x18, 0x19, 0x10, 0x13, 0x1F, 0x14, // K-T
        0x16, 0x2F, 0x11, 0x2D, 0x15, 0x2C                          // U-Z
    };
    uint8_t scancode = 0;
    if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z')) {
        int idx = (ch | 0x20) - 'a'; // Case-insensitive index
        scancode = altScancodes[idx];
    } else if (ch >= '1' && ch <= '9') {
        scancode = static_cast<uint8_t>(0x78 + (ch - '1')); // Alt+1=0x78 .. Alt+9=0x80
    } else if (ch == '0') {
        scancode = 0x81; // Alt+0
    } else if (ch == '-') {
        scancode = 0x82;
    } else if (ch == '=') {
        scancode = 0x83;
    }
    if (scancode != 0) {
        auto it = m_heldKeys.find(scancode);
        if (it != m_heldKeys.end()) {
            it->second.pressTime = std::chrono::steady_clock::now();
        } else {
            m_kbd.pushMakeKey(0, scancode);
            m_heldKeys[scancode] = { scancode, false,
                                     std::chrono::steady_clock::now() };
        }
    }
}

void InputManager::releaseTimedOutKeys() {
    auto now = std::chrono::steady_clock::now();
    for (auto it = m_heldKeys.begin(); it != m_heldKeys.end(); ) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - it->second.pressTime).count();
        if (elapsed >= KEY_HOLD_MS) {
            if (it->second.extended) {
                m_kbd.pushBreakKeyExtended(it->second.scancode);
            } else {
                m_kbd.pushBreakKey(it->second.scancode);
            }
            it = m_heldKeys.erase(it);
        } else {
            ++it;
        }
    }
}

} // namespace fador::ui
