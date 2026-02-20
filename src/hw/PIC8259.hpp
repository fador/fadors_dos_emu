#pragma once
#include "IODevice.hpp"
#include <cstdint>

namespace fador::hw {

class PIC8259 : public IODevice {
public:
    PIC8259(bool master);
    ~PIC8259() = default;

    uint8_t read8(uint16_t port) override;
    void write8(uint16_t port, uint8_t value) override;

    // TODO: IRQ handling logic
    void raiseIRQ(uint8_t irq);
    int getPendingInterrupt();
    void acknowledgeInterrupt();

private:
    bool m_master;
    uint8_t m_baseVector;
    uint8_t m_mask;
    uint8_t m_request;
    uint8_t m_service;
    
    // Initialization state machine
    uint8_t m_icwStep;
    bool m_initializing;
};

} // namespace fador::hw
