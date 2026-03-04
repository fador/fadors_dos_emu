#pragma once
#include <cstdint>
#include "../hw/KeyboardController.hpp"

#ifndef _WIN32
#include <termios.h>
#endif

namespace fador::hw { class BIOS; }

namespace fador::ui {

class InputManager {
public:
    InputManager(hw::KeyboardController& kbd);
    ~InputManager();

    // Set BIOS reference for mouse state updates
    void setBIOS(hw::BIOS& bios) { m_bios = &bios; }

    // Checks for host keyboard input and pushes scancodes to the controller
    // Returns true if input was processed
    bool pollInput();

private:
    hw::KeyboardController& m_kbd;
    hw::BIOS* m_bios = nullptr;

    // Maps ASCII/Special keys to PC/XT scancodes
    void handleKey(int key, bool pressed);
    void handleMouseEvent(int button, int col, int row, bool pressed);

#ifndef _WIN32
    struct termios m_origTermios;
    bool m_rawMode = false;
#endif
};

} // namespace fador::ui
