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

void PIT8254::addCycles(uint32_t cycles) {
    m_cycleAccum += cycles;
    if (m_cycleAccum >= CYCLES_PER_PIT_TICK) {
        uint32_t pitTicks = m_cycleAccum / CYCLES_PER_PIT_TICK;
        m_cycleAccum %= CYCLES_PER_PIT_TICK;
        advanceTicks(pitTicks);
    }
}

void PIT8254::advanceTicks(uint32_t ticks) {
    if (ticks == 0) return;

    for (int i = 0; i < 3; ++i) {
        auto& ch = m_channels[i];
        uint32_t reload = ch.reload ? ch.reload : 0x10000;
        uint32_t count = ch.count ? ch.count : 0x10000;

        if (ticks >= count) {
            if (i == 0) m_irq0Pending = true;
            uint32_t remaining = (ticks - count) % reload;
            ch.count = static_cast<uint16_t>(reload - remaining);
        } else {
            ch.count = static_cast<uint16_t>(count - ticks);
        }
    }
}

void PIT8254::update() {
    // This method is kept for backward compatibility but the cycle-based
    // addCycles() is now the primary timing mechanism.
}

bool PIT8254::checkPendingIRQ0() {
    bool ret = m_irq0Pending;
    m_irq0Pending = false;
    return ret;
}

} // namespace fador::hw
