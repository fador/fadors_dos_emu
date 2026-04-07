#include "VGAController.hpp"
#include "../utils/Logger.hpp"

namespace fador::hw {

VGAController::VGAController(memory::MemoryBus& memory)
    : m_memory(memory) {}

uint8_t VGAController::read8(uint16_t port) {
    switch (port) {
        // --- DAC ---
        case 0x3C7: return 0x03; // DAC state
        case 0x3C8: return m_dacWriteIndex;
        case 0x3C9: {
            uint8_t val = m_memory.read8(PALETTE_BASE + m_dacReadIndex * 3 + m_dacColorCycle);
            if (++m_dacColorCycle >= 3) { m_dacColorCycle = 0; m_dacReadIndex++; }
            return val;
        }

        // --- Sequencer ---
        case 0x3C4: return m_seqIndex;
        case 0x3C5:
            if (m_seqIndex == 2) return m_seqMapMask;
            if (m_seqIndex == 4) return m_chain4 ? 0x08 : 0x00;
            return 0;

        // --- Graphics Controller ---
        case 0x3CE: return m_gcIndex;
        case 0x3CF:
            if (m_gcIndex == 4) return m_gcReadMap;
            return 0;

        // --- CRTC ---
        case 0x3D4: return m_crtcIndex;
        case 0x3D5:
            if (m_crtcIndex < 0x19) return m_crtcRegs[m_crtcIndex];
            return 0;

        // --- Input Status ---
        case 0x3DA: {
            // Bit 0: Display Enable (1 during retrace)
            // Bit 3: Vertical Retrace (1 during retrace)
            // Programs busy-wait in tight loops reading this port.
            // Simulate by cycling through display/retrace states based on
            // read count rather than wall clock (our CPU is much slower
            // than real hardware). After ~32 reads in display period,
            // enter retrace for ~4 reads, then cycle back.
            m_retraceReadCount++;
            bool inRetrace = (m_retraceReadCount % 36) >= 32;
            return inRetrace ? 0x09 : 0x00;
        }

        default:
            LOG_TRACE("VGA: Read port 0x", std::hex, port);
            return 0;
    }
}

void VGAController::write8(uint16_t port, uint8_t value) {
    switch (port) {
        // --- DAC ---
        case 0x3C7:
            m_dacReadIndex = value;
            m_dacColorCycle = 0;
            break;
        case 0x3C8:
            m_dacWriteIndex = value;
            m_dacColorCycle = 0;
            break;
        case 0x3C9:
            m_memory.write8(PALETTE_BASE + m_dacWriteIndex * 3 + m_dacColorCycle, value & 0x3F);
            if (++m_dacColorCycle >= 3) { m_dacColorCycle = 0; m_dacWriteIndex++; }
            break;

        // --- Sequencer ---
        case 0x3C4: m_seqIndex = value; break;
        case 0x3C5:
            if (m_seqIndex == 2) m_seqMapMask = value & 0x0F;
            else if (m_seqIndex == 4) m_chain4 = (value & 0x08) != 0;
            break;

        // --- Graphics Controller ---
        case 0x3CE: m_gcIndex = value; break;
        case 0x3CF:
            if (m_gcIndex == 4) m_gcReadMap = value & 0x03;
            break;

        // --- CRTC ---
        case 0x3D4: m_crtcIndex = value; break;
        case 0x3D5:
            if (m_crtcIndex < 0x19) m_crtcRegs[m_crtcIndex] = value;
            break;

        default:
            LOG_TRACE("VGA: Write port 0x", std::hex, port, " = 0x", (int)value);
            break;
    }
}

} // namespace fador::hw

// ── Plane-aware VRAM access ──────────────────────────────────────────

void fador::hw::VGAController::planeWrite8(uint32_t offset, uint8_t value) {
    if (m_chain4) {
        // Chain-4: address bits [1:0] select plane, [15:2] select offset
        uint8_t plane = offset & 3;
        uint32_t planeOff = offset >> 2;
        m_planes[plane][planeOff & 0xFFFF] = value;
    } else {
        // Mode-X / unchained: write to all planes selected by mapMask
        uint32_t planeOff = offset & 0xFFFF;
        for (int p = 0; p < 4; ++p) {
            if (m_seqMapMask & (1 << p))
                m_planes[p][planeOff] = value;
        }
    }
}

uint8_t fador::hw::VGAController::planeRead8(uint32_t offset) const {
    if (m_chain4) {
        // Chain-4: address bits [1:0] select plane, [15:2] select offset
        uint8_t plane = offset & 3;
        uint32_t planeOff = offset >> 2;
        return m_planes[plane][planeOff & 0xFFFF];
    } else {
        // Mode-X / unchained: read from the plane selected by readMapSelect
        return m_planes[m_gcReadMap & 3][offset & 0xFFFF];
    }
}
