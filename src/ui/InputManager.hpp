#pragma once
#include <cstdint>
#include "../hw/KeyboardController.hpp"

#ifndef _WIN32
#include <termios.h>
#endif

namespace fador::ui {

class InputManager {
public:
    InputManager(hw::KeyboardController& kbd);
    ~InputManager();

    // Checks for host keyboard input and pushes scancodes to the controller
    // Returns true if input was processed
    bool pollInput();

private:
    hw::KeyboardController& m_kbd;

    // Maps ASCII/Special keys to PC/XT scancodes
    void handleKey(int key, bool pressed);

#ifndef _WIN32
    struct termios m_origTermios;
    bool m_rawMode = false;
#endif
};

} // namespace fador::ui
