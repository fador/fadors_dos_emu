#include "IOBus.hpp"
#include "../utils/Logger.hpp"

namespace fador::hw {

void IOBus::registerDevice(uint16_t startPort, uint16_t endPort, IODevice* device) {
    for (uint16_t port = startPort; port <= endPort; ++port) {
        m_deviceMap[port] = device;
    }
    LOG_DEBUG("IOBus: Registered device for ports 0x", std::hex, startPort, "-0x", endPort);
}

IODevice* IOBus::getDevice(uint16_t port) {
    auto it = m_deviceMap.find(port);
    if (it != m_deviceMap.end()) {
        return it->second;
    }
    return nullptr;
}

uint8_t IOBus::read8(uint16_t port) {
    if (auto device = getDevice(port)) {
        return device->read8(port);
    }
    LOG_TRACE("IOBus: Read from unmapped port 0x", std::hex, port);
    return 0xFF;
}

void IOBus::write8(uint16_t port, uint8_t value) {
    if (auto device = getDevice(port)) {
        device->write8(port, value);
        return;
    }
    LOG_TRACE("IOBus: Write to unmapped port 0x", std::hex, port, " val 0x", (int)value);
}

uint16_t IOBus::read16(uint16_t port) {
    if (auto device = getDevice(port)) {
        return device->read16(port);
    }
    return read8(port) | (static_cast<uint16_t>(read8(port + 1)) << 8);
}

void IOBus::write16(uint16_t port, uint16_t value) {
    if (auto device = getDevice(port)) {
        device->write16(port, value);
        return;
    }
    write8(port, static_cast<uint8_t>(value & 0xFF));
    write8(port + 1, static_cast<uint8_t>(value >> 8));
}

uint32_t IOBus::read32(uint16_t port) {
    if (auto device = getDevice(port)) {
        return device->read32(port);
    }
    return read16(port) | (static_cast<uint32_t>(read16(port + 2)) << 16);
}

void IOBus::write32(uint16_t port, uint32_t value) {
    if (auto device = getDevice(port)) {
        device->write32(port, value);
        return;
    }
    write16(port, static_cast<uint16_t>(value & 0xFFFF));
    write16(port + 2, static_cast<uint16_t>(value >> 16));
}

} // namespace fador::hw
