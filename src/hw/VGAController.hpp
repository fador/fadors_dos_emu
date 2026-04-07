#pragma once
#include "IODevice.hpp"
#include "../memory/MemoryBus.hpp"
#include <chrono>

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
    bool isChain4() const { return m_chain4; }

    // Plane-aware VRAM access (called by MemoryBus for 0xA0000-0xAFFFF)
    void planeWrite8(uint32_t offset, uint8_t value);
    uint8_t planeRead8(uint32_t offset) const;

    // Read a specific plane byte (for renderer use)
    uint8_t readPlane(int plane, uint32_t offset) const {
        return m_planes[plane & 3][offset & 0xFFFF];
    }

    // CRTC start address (for page flipping / display offset)
    uint32_t getDisplayStart() const {
        return (uint32_t(m_crtcRegs[0x0C]) << 8) | m_crtcRegs[0x0D];
    }

    static constexpr uint32_t PALETTE_BASE = 0xE0000;
    static constexpr uint32_t VGA_WINDOW_START = 0xA0000;
    static constexpr uint32_t VGA_WINDOW_SIZE = 0x10000; // 64KB

private:
    memory::MemoryBus& m_memory;
    uint8_t m_dacWriteIndex = 0;
    uint8_t m_dacReadIndex = 0;
    uint8_t m_dacColorCycle = 0; // 0=R, 1=G, 2=B

    // VGA timing: simulate retrace using port read counter.
    // After a certain number of reads, transition between retrace/display.
    uint32_t m_retraceReadCount = 0;

    // Sequencer registers (port 0x3C4/0x3C5)
    uint8_t m_seqIndex = 0;
    uint8_t m_seqMapMask = 0x0F; // register 2: which planes to write
    bool m_chain4 = true;        // register 4 bit 3: Chain-4 mode

    // Graphics Controller registers (port 0x3CE/0x3CF)
    uint8_t m_gcIndex = 0;
    uint8_t m_gcReadMap = 0;     // register 4: which plane to read

    // CRTC registers (port 0x3D4/0x3D5) — mostly stubs
    uint8_t m_crtcIndex = 0;
    uint8_t m_crtcRegs[0x19] = {};

    // VGA plane memory (4 planes × 64KB each = 256KB total)
    uint8_t m_planes[4][65536] = {};
};

} // namespace fador::hw
