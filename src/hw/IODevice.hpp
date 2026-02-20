#pragma once
#include <cstdint>

namespace fador::hw {

class IODevice {
public:
    virtual ~IODevice() = default;

    virtual uint8_t read8(uint16_t port) = 0;
    virtual void write8(uint16_t port, uint8_t value) = 0;

    // Optional overrides for 16/32 bit if device supports them natively
    virtual uint16_t read16(uint16_t port) {
        return read8(port) | (static_cast<uint16_t>(read8(port + 1)) << 8);
    }
    virtual void write16(uint16_t port, uint16_t value) {
        write8(port, static_cast<uint8_t>(value & 0xFF));
        write8(port + 1, static_cast<uint8_t>((value >> 8) & 0xFF));
    }
    virtual uint32_t read32(uint16_t port) {
        return read16(port) | (static_cast<uint32_t>(read16(port + 2)) << 16);
    }
    virtual void write32(uint16_t port, uint32_t value) {
        write16(port, static_cast<uint16_t>(value & 0xFFFF));
        write16(port + 2, static_cast<uint16_t>((value >> 16) & 0xFFFF));
    }
};

} // namespace fador::hw
