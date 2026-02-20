#pragma once
#include "IODevice.hpp"
#include <cstdint>
#include <chrono>

namespace fador::hw {

class PIT8254 : public IODevice {
public:
    PIT8254();
    ~PIT8254() = default;

    uint8_t read8(uint16_t port) override;
    void write8(uint16_t port, uint8_t value) override;

    void update(); // Called by main loop to process timer ticks
    bool checkPendingIRQ0();

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
    std::chrono::steady_clock::time_point m_lastUpdate;
    bool m_irq0Pending;

    static constexpr double BASE_FREQ = 1193182.0;
};

} // namespace fador::hw
