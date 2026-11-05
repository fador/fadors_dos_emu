#pragma once
#include <cstdint>
#include <memory>
#include <vector>
#include "IODevice.hpp"

namespace fador::hw {

class IOBus {
public:
    IOBus() = default;
    ~IOBus() = default;

    void registerDevice(uint16_t startPort, uint16_t endPort, IODevice* device);

    uint8_t read8(uint16_t port);
    void write8(uint16_t port, uint8_t value);

    uint16_t read16(uint16_t port);
    void write16(uint16_t port, uint16_t value);

    uint32_t read32(uint16_t port);
    void write32(uint16_t port, uint32_t value);

    // Log every byte read from or written to ports first..last (--io-trace).
    void setTraceRange(uint16_t first, uint16_t last) {
        m_traceFirst = first;
        m_traceLast = last;
        m_trace = true;
    }

private:
    std::vector<IODevice*> m_devices = std::vector<IODevice*>(0x10000, nullptr);
    bool m_trace = false;
    uint16_t m_traceFirst = 0, m_traceLast = 0;

    IODevice* getDevice(uint16_t port) { return m_devices[port]; }
    bool traced(uint16_t port) const {
        return m_trace && port >= m_traceFirst && port <= m_traceLast;
    }
};

} // namespace fador::hw
