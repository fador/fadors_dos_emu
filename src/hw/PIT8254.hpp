#pragma once
#include "IODevice.hpp"
#include <chrono>
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
    void advanceTime(std::chrono::nanoseconds elapsed);

    // Compatibility shim for legacy call sites. PIT timing is wall-clock based.
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
    uint64_t m_pendingIRQ0 = 0;
    std::chrono::steady_clock::time_point m_lastUpdate;
    uint64_t m_subTickRemainder = 0;

    static constexpr uint64_t BASE_FREQ_HZ = 1193182;
    static constexpr uint64_t NANOS_PER_SECOND = 1000000000ULL;

    void advanceTicks(uint64_t ticks);
    void syncToRealtime();
};

} // namespace fador::hw
