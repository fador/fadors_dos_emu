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
            // Bit 0: Display Enable (set during h-retrace and v-retrace)
            // Bit 3: Vertical Retrace
            // Toggle both on every read to satisfy programs waiting for either bit.
            m_verticalRetrace = !m_verticalRetrace;
            return m_verticalRetrace ? 0x09 : 0x00;
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
