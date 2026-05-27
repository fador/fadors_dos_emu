#include "PIC8259.hpp"
#include "../utils/Logger.hpp"

namespace fador::hw {

PIC8259::PIC8259(bool master)
    : m_master(master)
    , m_baseVector(master ? 0x08 : 0x70)
    , m_mask(0xFF)
    , m_request(0)
    , m_service(0)
    , m_icwStep(0)
    , m_initializing(false)
    , m_autoEoi(false)
    , m_priorityAdd(0)
    , m_readRegSelect(0) // Default to IRR
    , m_specialMaskMode(false)
    , m_specialFullyNestedMode(false)
    , m_pollMode(false) {
}

int PIC8259::getHighestPriorityIRQ(uint8_t mask) {
    if (mask == 0) return -1;
    for (int i = 0; i < 8; ++i) {
        int irq = (i + m_priorityAdd + 1) & 7; // priorityAdd is the lowest priority IRQ
        if (mask & (1 << irq)) {
            return irq;
        }
    }
    return -1;
}

uint8_t PIC8259::read8(uint16_t port) {
    uint8_t offset = port & 1;
    if (offset == 0) {
        if (m_pollMode) {
            m_pollMode = false;
            int irq = getPendingInterrupt();
            if (irq != -1) {
                irq -= m_baseVector;
                m_service |= (1 << irq);
                m_request &= ~(1 << irq);
                return 0x80 | irq;
            }
            return 0;
        }
        return m_readRegSelect == 0 ? m_request : m_service;
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
            m_mask = 0;
            m_service = 0;
            m_priorityAdd = 7;
            m_specialMaskMode = false;
            m_pollMode = false;
            m_readRegSelect = 0;
            LOG_DEBUG("PIC ", (m_master ? "Master" : "Slave"), ": Initialization started (ICW1: 0x", std::hex, (int)value, ")");
        } else if ((value & 0x18) == 0x00) { // OCW2
            uint8_t cmd = value >> 5;
            uint8_t irq = value & 7;

            switch (cmd) {
                case 1: // Non-specific EOI
                {
                    int highestService = getHighestPriorityIRQ(m_service);
                    if (highestService != -1) {
                        m_service &= ~(1 << highestService);
                    }
                    break;
                }
                case 3: // Specific EOI
                    m_service &= ~(1 << irq);
                    break;
                case 5: // Rotate on non-specific EOI
                {
                    int highestService = getHighestPriorityIRQ(m_service);
                    if (highestService != -1) {
                        m_service &= ~(1 << highestService);
                        m_priorityAdd = highestService;
                    }
                    break;
                }
                case 4: // Rotate in auto EOI mode (set)
                    // Auto EOI rotation not fully implemented, ignore for now
                    break;
                case 0: // Rotate in auto EOI mode (clear)
                    break;
                case 7: // Rotate on specific EOI
                    m_service &= ~(1 << irq);
                    m_priorityAdd = irq;
                    break;
                case 6: // Set priority
                    m_priorityAdd = irq;
                    break;
            }
        } else if ((value & 0x18) == 0x08) { // OCW3
            if (value & 0x04) {
                m_pollMode = true;
            }
            if (value & 0x02) {
                m_readRegSelect = value & 0x01; // 0 = IRR, 1 = ISR
            }
            if (value & 0x40) {
                m_specialMaskMode = (value & 0x20) != 0;
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
                m_autoEoi = (value & 0x02) != 0;
                m_specialFullyNestedMode = (value & 0x10) != 0;
                m_initializing = false;
                LOG_DEBUG("PIC ", (m_master ? "Master" : "Slave"), ": Initialization complete. Base vector: 0x", std::hex, (int)m_baseVector);
            }
        } else {
            m_mask = value;
        }
    }
}

void PIC8259::raiseIRQ(uint8_t irq) {
    if (irq >= 8) return;

    ++m_requestCounts[irq];
    m_request |= static_cast<uint8_t>(1u << irq);
}

int PIC8259::getPendingInterrupt() {
    uint8_t pending = m_request & ~m_mask;
    if (pending == 0) return -1;
    
    int highestPending = getHighestPriorityIRQ(pending);
    if (highestPending == -1) return -1;

    int highestService = getHighestPriorityIRQ(m_service);

    if (highestService != -1) {
        if (m_specialMaskMode) {
            // In Special Mask Mode, interrupts are not inhibited by lower priority ones in service
        } else {
            // Priority formula relative to priorityAdd (which is the lowest priority)
            // Priority 0 is highest, 7 is lowest.
            int pendingPriority = (highestPending - m_priorityAdd - 1) & 7;
            int servicePriority = (highestService - m_priorityAdd - 1) & 7;

            if (pendingPriority >= servicePriority) {
                if (!(m_specialFullyNestedMode && highestPending == highestService)) {
                    return -1;
                }
            }
        }
    }

    return m_baseVector + highestPending;
}

void PIC8259::acknowledgeInterrupt() {
    uint8_t pending = m_request & ~m_mask;
    int irq = getHighestPriorityIRQ(pending);
    
    if (irq != -1) {
        if (m_requestCounts[irq] > 0) {
            --m_requestCounts[irq];
        }
        if (m_requestCounts[irq] == 0) {
            m_request &= static_cast<uint8_t>(~(1u << irq));
        }
        if (!m_autoEoi) {
            m_service |= static_cast<uint8_t>(1u << irq);
        }
    }
}

void PIC8259::unmaskIRQ(uint8_t irq) {
    if (irq >= 8) return;
    m_mask &= static_cast<uint8_t>(~(1u << irq));
}

bool PIC8259::isIRQInService(uint8_t irq) const {
    if (irq >= 8) return false;
    return (m_service & (1u << irq)) != 0;
}

} // namespace fador::hw
