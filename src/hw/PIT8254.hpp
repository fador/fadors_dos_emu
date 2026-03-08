#pragma once
#include "IODevice.hpp"
#include <cstdint>

namespace fador::hw {

class PIT8254 : public IODevice {
public:
    PIT8254();
    ~PIT8254() = default;

    uint8_t read8(uint16_t port) override;
    void write8(uint16_t port, uint8_t value) override;

    void update(); // Called by main loop to process timer ticks
    bool checkPendingIRQ0();

    // Advance virtual clock by one instruction (~4 CPU cycles).
    void addCycles(uint32_t cycles = 4);

private:
    struct Channel {
        uint16_t reload;
        uint16_t count;
        uint8_t mode;
        uint8_t accessMode;
        bool hiByteNext;
        bool latched;
        uint16_t latchedValue;
    };

    Channel m_channels[3];
    bool m_irq0Pending;

    // Instruction-based virtual clock
    uint64_t m_cycleAccum = 0;

    // Assume ~20 MHz 386: 20 MHz / 1.193182 MHz ≈ 16.8 CPU cycles per PIT tick
    static constexpr uint32_t CYCLES_PER_PIT_TICK = 16;

    static constexpr double BASE_FREQ = 1193182.0;

    void advanceTicks(uint32_t ticks);
};

} // namespace fador::hw
