#include "PIT8254.hpp"
#include "../utils/Logger.hpp"

namespace fador::hw {

PIT8254::PIT8254() : m_irq0Pending(false) {
    for (int i = 0; i < 3; ++i) {
        m_channels[i].reload = 0xFFFF;
        m_channels[i].count = 0xFFFF;
        m_channels[i].mode = 3; // Square wave
        m_channels[i].accessMode = 3; // LSB/MSB
        m_channels[i].hiByteNext = false;
        m_channels[i].latched = false;
    }
    m_lastUpdate = std::chrono::steady_clock::now();
}

uint8_t PIT8254::read8(uint16_t port) {
    uint8_t channelIdx = port & 3;
    if (channelIdx > 2) return 0xFF;

    auto& ch = m_channels[channelIdx];
    uint16_t val = ch.latched ? ch.latchedValue : ch.count;
    
    uint8_t ret;
    if (ch.accessMode == 1) { // LSB only
        ret = val & 0xFF;
    } else if (ch.accessMode == 2) { // MSB only
        ret = (val >> 8) & 0xFF;
    } else { // LSB then MSB
        if (ch.hiByteNext) {
            ret = (val >> 8) & 0xFF;
            ch.hiByteNext = false;
            ch.latched = false; // Release latch after full read
        } else {
            ret = val & 0xFF;
            ch.hiByteNext = true;
        }
    }
    return ret;
}

void PIT8254::write8(uint16_t port, uint8_t value) {
    uint8_t channelIdx = port & 3;
    if (channelIdx == 3) { // Control Word
        uint8_t target = (value >> 6) & 3;
        if (target > 2) return; // Read-back not implemented

        auto& ch = m_channels[target];
        uint8_t access = (value >> 4) & 3;
        if (access == 0) { // Latch
            ch.latched = true;
            ch.latchedValue = ch.count;
        } else {
            ch.accessMode = access;
            ch.mode = (value >> 1) & 7;
            ch.hiByteNext = false;
        }
    } else {
        auto& ch = m_channels[channelIdx];
        if (ch.accessMode == 1) { // LSB
            ch.reload = (ch.reload & 0xFF00) | value;
            ch.count = ch.reload;
        } else if (ch.accessMode == 2) { // MSB
            ch.reload = (ch.reload & 0x00FF) | (static_cast<uint16_t>(value) << 8);
            ch.count = ch.reload;
        } else { // LSB then MSB
            if (ch.hiByteNext) {
                ch.reload = (ch.reload & 0x00FF) | (static_cast<uint16_t>(value) << 8);
                ch.count = ch.reload;
                ch.hiByteNext = false;
            } else {
                ch.reload = (ch.reload & 0xFF00) | value;
                ch.hiByteNext = true;
            }
        }
    }
}

void PIT8254::update() {
    auto now = std::chrono::steady_clock::now();
    double elapsedSec = std::chrono::duration<double>(now - m_lastUpdate).count();
    m_lastUpdate = now;

    uint32_t ticks = static_cast<uint32_t>(elapsedSec * BASE_FREQ);
    if (ticks == 0) return;

    auto& ch0 = m_channels[0];
    uint32_t currentCount = ch0.count;
    if (currentCount == 0) currentCount = 0x10000;

    if (ticks >= currentCount) {
        m_irq0Pending = true;
        ch0.count = ch0.reload - static_cast<uint16_t>(ticks % (ch0.reload ? ch0.reload : 0x10000));
    } else {
        ch0.count -= static_cast<uint16_t>(ticks);
    }
}

bool PIT8254::checkPendingIRQ0() {
    bool ret = m_irq0Pending;
    m_irq0Pending = false;
    return ret;
}

} // namespace fador::hw
