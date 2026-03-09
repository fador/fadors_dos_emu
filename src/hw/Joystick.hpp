#pragma once

#include "IODevice.hpp"

namespace fador::hw {

// A stub Joystick interface for the legacy PC Gameport (Port 0x201).
// Without this, the floating I/O bus reads 0xFF, which games like Liero
// interpret as the analog sticks being pegged in the bottom right corner
// infinitely (because the discharge duration timeout counter maxes out).
// Returning 0xF0 tells DOS games that buttons are released and axes are
// discharged.
class Joystick : public IODevice {
public:
  uint8_t read8(uint16_t /*port*/) override {
    // Upper 4 bits: Buttons (0 = pressed, 1 = released) -> 1111 (F)
    // Lower 4 bits: Axes (0 = discharged/centered, 1 = charging) -> 0000 (0)
    return 0xF0;
  }

  void write8(uint16_t /*port*/, uint8_t /*value*/) override {
    // Writing to 0x201 usually initiates the joystick RC-circuit discharge
    // cycle. We do nothing since our dummy axes are always immediately
    // discharged.
  }
};

} // namespace fador::hw
