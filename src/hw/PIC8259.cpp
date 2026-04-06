#include "PIC8259.hpp"
#include "../utils/Logger.hpp"

namespace fador::hw {

PIC8259::PIC8259(bool master)
    : m_master(master)
    , m_baseVector(master ? 0x08 : 0x70)
    , m_mask(0x00)
    , m_request(0)
    , m_service(0)
    , m_icwStep(0)
    , m_initializing(false) {
}

uint8_t PIC8259::read8(uint16_t port) {
    uint8_t offset = port & 1;
    if (offset == 0) {
        return m_request; // Simplified: Return IRR
    } else {
        return m_mask;
    }
}

void PIC8259::write8(uint16_t port, uint8_t value) {
    uint8_t offset = port & 1;
    
    if (offset == 0) {
        if (value & 0x10) { // ICW1
            m_initializing = true;
            m_icwStep = 1;
            LOG_DEBUG("PIC ", (m_master ? "Master" : "Slave"), ": Initialization started (ICW1: 0x", std::hex, (int)value, ")");
        } else if ((value & 0x18) == 0x00) { // OCW2
            if (value & 0x20) { // EOI
                m_service &= m_service - 1; // Clear highest bit in ISR (Simple EOI)
            }
        }
    } else {
        if (m_initializing) {
            m_icwStep++;
            if (m_icwStep == 2) { // ICW2: Base Vector
                m_baseVector = value & 0xF8;
            } else if (m_icwStep == 3) { // ICW3: Cascade
                // Ignore for now
            } else if (m_icwStep == 4) { // ICW4: Mode
                m_initializing = false;
                LOG_DEBUG("PIC ", (m_master ? "Master" : "Slave"), ": Initialization complete. Base vector: 0x", std::hex, (int)m_baseVector);
            }
        } else {
            m_mask = value;
        }
    }
}

void PIC8259::raiseIRQ(uint8_t irq) {
    if (!(m_mask & (1 << irq))) {
        m_request |= (1 << irq);
    }
}

int PIC8259::getPendingInterrupt() {
    uint8_t pending = m_request & ~m_mask;
    if (pending == 0) return -1;
    
    // Fully nested mode: only deliver if no equal-or-higher-priority IRQ
    // is already in service (lower IRQ number = higher priority).
    for (int i = 0; i < 8; ++i) {
        if (pending & (1 << i)) {
            uint8_t higherOrEqualMask = static_cast<uint8_t>((1u << (i + 1)) - 1);
            if (m_service & higherOrEqualMask)
                return -1;
            return m_baseVector + i;
        }
    }
    return -1;
}

void PIC8259::acknowledgeInterrupt() {
    int irq = -1;
    uint8_t pending = m_request & ~m_mask;
    for (int i = 0; i < 8; ++i) {
        if (pending & (1 << i)) {
            irq = i;
            break;
        }
    }
    
    if (irq != -1) {
        m_request &= ~(1 << irq);
        m_service |= (1 << irq);
    }
}

} // namespace fador::hw
