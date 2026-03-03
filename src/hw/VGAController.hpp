#pragma once
#include "IODevice.hpp"
#include "../memory/MemoryBus.hpp"

namespace fador::hw {

// VGA I/O port emulation for DAC palette access, retrace status,
// Sequencer (plane mask) and Graphics Controller (read map select).
// The palette is stored in memory at PALETTE_BASE (256 * 3 bytes, 6-bit RGB).
class VGAController : public IODevice {
public:
    explicit VGAController(memory::MemoryBus& memory);

    uint8_t read8(uint16_t port) override;
    void write8(uint16_t port, uint8_t value) override;

    // Plane state for direct VRAM access in planar modes
    uint8_t getMapMask() const { return m_seqMapMask; }
    uint8_t getReadMapSelect() const { return m_gcReadMap; }

    static constexpr uint32_t PALETTE_BASE = 0xE0000;

private:
    memory::MemoryBus& m_memory;
    uint8_t m_dacWriteIndex = 0;
    uint8_t m_dacReadIndex = 0;
    uint8_t m_dacColorCycle = 0; // 0=R, 1=G, 2=B
    bool m_verticalRetrace = false;

    // Sequencer registers (port 0x3C4/0x3C5)
    uint8_t m_seqIndex = 0;
    uint8_t m_seqMapMask = 0x0F; // register 2: which planes to write

    // Graphics Controller registers (port 0x3CE/0x3CF)
    uint8_t m_gcIndex = 0;
    uint8_t m_gcReadMap = 0;     // register 4: which plane to read

    // CRTC registers (port 0x3D4/0x3D5) — mostly stubs
    uint8_t m_crtcIndex = 0;
    uint8_t m_crtcRegs[0x19] = {};
};

} // namespace fador::hw
