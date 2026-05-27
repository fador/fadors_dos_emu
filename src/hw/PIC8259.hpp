#pragma once
#include "IODevice.hpp"
#include <array>
#include <cstdint>

namespace fador::hw {

class PIC8259 : public IODevice {
public:
    PIC8259(bool master);
    ~PIC8259() = default;

    uint8_t read8(uint16_t port) override;
    void write8(uint16_t port, uint8_t value) override;

    void raiseIRQ(uint8_t irq);
    int getPendingInterrupt();
    void acknowledgeInterrupt();

    // Unmask an interrupt line (clear the mask bit)
    void unmaskIRQ(uint8_t irq);

    // Check if an IRQ is currently in service (ISR bit set)
    bool isIRQInService(uint8_t irq) const;

private:
    bool m_master;
    uint8_t m_baseVector;
    uint8_t m_mask;
    uint8_t m_request;
    uint8_t m_service;
    std::array<uint32_t, 8> m_requestCounts{};
    
    // Initialization state machine
    uint8_t m_icwStep;
    bool m_initializing;

    // Additional state
    bool m_autoEoi;
    uint8_t m_priorityAdd;
    uint8_t m_readRegSelect; // 0 = IRR, 1 = ISR
    bool m_specialMaskMode;
    bool m_specialFullyNestedMode;
    bool m_pollMode;

    int getHighestPriorityIRQ(uint8_t mask);
};

} // namespace fador::hw
