#include "IOBus.hpp"
#include "../utils/Logger.hpp"
#include <cstdio>

namespace fador::hw {

void IOBus::registerDevice(uint16_t startPort, uint16_t endPort, IODevice* device) {
    for (uint16_t port = startPort; port <= endPort; ++port) {
        m_devices[port] = device;
        if (port == 0xFFFF)
            break;
    }
    LOG_DEBUG("IOBus: Registered device for ports 0x", std::hex, startPort, "-0x", endPort);
}

uint8_t IOBus::read8(uint16_t port) {
    uint8_t value = 0xFF;
    if (auto device = getDevice(port))
        value = device->read8(port);
    else
        LOG_TRACE("IOBus: Read from unmapped port 0x", std::hex, port);
    if (traced(port))
        std::fprintf(stderr, "IO: in  %04X -> %02X  %s\n", port, value,
                     m_traceContext ? m_traceContext().c_str() : "");
    return value;
}

void IOBus::write8(uint16_t port, uint8_t value) {
    if (traced(port))
        std::fprintf(stderr, "IO: out %04X <- %02X  %s\n", port, value,
                     m_traceContext ? m_traceContext().c_str() : "");
    if (auto device = getDevice(port)) {
        device->write8(port, value);
        return;
    }
    LOG_TRACE("IOBus: Write to unmapped port 0x", std::hex, port, " val 0x", (int)value);
}

uint16_t IOBus::read16(uint16_t port) {
    if (auto device = getDevice(port)) {
        const uint16_t value = device->read16(port);
        if (traced(port))
            std::fprintf(stderr, "IO: inw %04X -> %04X  %s\n", port, value,
                         m_traceContext ? m_traceContext().c_str() : "");
        return value;
    }
    return read8(port) | (static_cast<uint16_t>(read8(port + 1)) << 8);
}

void IOBus::write16(uint16_t port, uint16_t value) {
    if (auto device = getDevice(port)) {
        if (traced(port))
            std::fprintf(stderr, "IO: outw %04X <- %04X  %s\n", port, value,
                         m_traceContext ? m_traceContext().c_str() : "");
        device->write16(port, value);
        return;
    }
    write8(port, static_cast<uint8_t>(value & 0xFF));
    write8(port + 1, static_cast<uint8_t>(value >> 8));
}

uint32_t IOBus::read32(uint16_t port) {
    if (auto device = getDevice(port)) {
        const uint32_t value = device->read32(port);
        if (traced(port))
            std::fprintf(stderr, "IO: ind %04X -> %08X  %s\n", port, value,
                         m_traceContext ? m_traceContext().c_str() : "");
        return value;
    }
    return read16(port) | (static_cast<uint32_t>(read16(port + 2)) << 16);
}

void IOBus::write32(uint16_t port, uint32_t value) {
    if (auto device = getDevice(port)) {
        if (traced(port))
            std::fprintf(stderr, "IO: outd %04X <- %08X  %s\n", port, value,
                         m_traceContext ? m_traceContext().c_str() : "");
        device->write32(port, value);
        return;
    }
    write16(port, static_cast<uint16_t>(value & 0xFFFF));
    write16(port + 2, static_cast<uint16_t>(value >> 16));
}

} // namespace fador::hw
