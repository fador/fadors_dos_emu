#pragma once

#include "IODevice.hpp"

#include "../cpu/CPU.hpp"

namespace fador::hw {

// A stub Joystick interface for the legacy PC Gameport (Port 0x201).
// Without this, the floating I/O bus reads 0xFF, which games like Liero
// interpret as the analog sticks being pegged in the bottom right corner.
// Returning 0xF0 instantly means the sticks are pegged top left.
// To emulate a centered joystick, we decay the axes after a short duration.
class Joystick : public IODevice {
private:
  fador::cpu::CPU *m_cpu = nullptr;
  uint64_t m_lastWriteCycles = 0;
  // 5000 cycles represents roughly a centered joypad axis on a 4.77MHz - 33MHz
  // system.
  const uint64_t CENTER_CYCLES = 5000;

public:
  void setCPU(fador::cpu::CPU *cpu) { m_cpu = cpu; }

  uint8_t read8(uint16_t /*port*/) override {
    if (!m_cpu)
      return 0xF0;

    uint64_t elapsed = m_cpu->getCycles() - m_lastWriteCycles;

    // Upper 4 bits: Buttons (0 = pressed, 1 = released) -> 1111 (F)
    uint8_t result = 0xF0;

    // Lower 4 bits: Axes (0 = discharged, 1 = charging)
    if (elapsed < CENTER_CYCLES) {
      result |= 0x0F; // Axes still high
    }

    return result;
  }

  void write8(uint16_t /*port*/, uint8_t /*value*/) override {
    // Writing to 0x201 initiates the joystick RC-circuit discharge cycle.
    if (m_cpu) {
      m_lastWriteCycles = m_cpu->getCycles();
    }
  }
};

} // namespace fador::hw
