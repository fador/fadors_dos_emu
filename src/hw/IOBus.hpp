#pragma once
#include <cstdint>
#include <map>
#include <memory>
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

private:
    std::map<uint16_t, IODevice*> m_deviceMap;
    
    IODevice* getDevice(uint16_t port);
};

} // namespace fador::hw
