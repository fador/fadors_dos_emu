#pragma once
#include <cstdint>
#include "../hw/KeyboardController.hpp"

namespace fador::ui {

class InputManager {
public:
    InputManager(hw::KeyboardController& kbd);
    ~InputManager() = default;

    // Checks for host keyboard input and pushes scancodes to the controller
    // Returns true if input was processed
    bool pollInput();

private:
    hw::KeyboardController& m_kbd;

    // Maps ASCII/Special keys to PC/XT scancodes
    void handleKey(int key, bool pressed);
    
    // Windows-specific non-blocking input check
    bool kbhit();
    int getch();
};

} // namespace fador::ui
