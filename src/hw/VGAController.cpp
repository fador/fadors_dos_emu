#include "VGAController.hpp"
#include "../utils/Logger.hpp"

namespace fador::hw {

namespace {

uint8_t rotateRight(uint8_t value, unsigned count) {
    count &= 7;
    if (count == 0) {
        return value;
    }

    return static_cast<uint8_t>((value >> count) | (value << (8 - count)));
}

uint8_t expandPlaneBit(uint8_t value, int plane) {
    return (value & (1u << plane)) != 0 ? 0xFF : 0x00;
}

uint8_t applyLogicalOp(uint8_t op, uint8_t input, uint8_t latch) {
    switch (op & 0x03) {
        case 0x01: return input & latch;
        case 0x02: return input | latch;
        case 0x03: return input ^ latch;
        default: return input;
    }
}

} // namespace

VGAController::VGAController(memory::MemoryBus& memory)
    : m_memory(memory) {}

void VGAController::loadLatches(uint32_t planeOffset) const {
    uint32_t index = planeOffset & 0xFFFF;
    for (int plane = 0; plane < 4; ++plane) {
        m_latches[plane] = m_planes[plane][index];
    }
}

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
            switch (m_gcIndex) {
                case 0x00: return m_gcSetReset;
                case 0x01: return m_gcEnableSetReset;
                case 0x02: return m_gcColorCompare;
                case 0x03: return m_gcDataRotate;
                case 0x04: return m_gcReadMap;
                case 0x05: return m_gcMode;
                case 0x06: return m_gcMisc;
                case 0x07: return m_gcColorDontCare;
                case 0x08: return m_gcBitMask;
                default: return 0;
            }

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
#if FADOR_ENABLE_DEBUG_DIAGNOSTICS
            LOG_INFO("VGA SEQ[", std::hex, int(m_seqIndex), "]=0x",
                     int(value), " mapMask=0x", int(m_seqMapMask),
                     " chain4=", m_chain4 ? 1 : 0);
#endif
            break;

        // --- Graphics Controller ---
        case 0x3CE: m_gcIndex = value; break;
        case 0x3CF:
            switch (m_gcIndex) {
                case 0x00: m_gcSetReset = value & 0x0F; break;
                case 0x01: m_gcEnableSetReset = value & 0x0F; break;
                case 0x02: m_gcColorCompare = value & 0x0F; break;
                case 0x03: m_gcDataRotate = value & 0x1F; break;
                case 0x04: m_gcReadMap = value & 0x03; break;
                case 0x05: m_gcMode = value & 0x7F; break;
                case 0x06: m_gcMisc = value & 0x0F; break;
                case 0x07: m_gcColorDontCare = value & 0x0F; break;
                case 0x08: m_gcBitMask = value; break;
                default: break;
            }
#if FADOR_ENABLE_DEBUG_DIAGNOSTICS
            LOG_INFO("VGA GC[", std::hex, int(m_gcIndex), "]=0x", int(value),
                     " readMap=0x", int(m_gcReadMap),
                     " mode=0x", int(m_gcMode),
                     " bitMask=0x", int(m_gcBitMask));
#endif
            break;

        // --- CRTC ---
        case 0x3D4: m_crtcIndex = value; break;
        case 0x3D5:
            if (m_crtcIndex < 0x19) m_crtcRegs[m_crtcIndex] = value;
#if FADOR_ENABLE_DEBUG_DIAGNOSTICS
            LOG_INFO("VGA CRTC[", std::hex, int(m_crtcIndex), "]=0x",
                     int(value), " displayStart=0x", getDisplayStart());
#endif
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
        if ((m_seqMapMask & (1u << plane)) != 0) {
            m_planes[plane][planeOff & 0xFFFF] = value;
        }
    } else {
        uint32_t planeOff = offset & 0xFFFF;
        uint8_t rotateCount = m_gcDataRotate & 0x07;
        uint8_t logicalOp = (m_gcDataRotate >> 3) & 0x03;
        uint8_t rotated = rotateRight(value, rotateCount);

        auto commitPlane = [&](int plane, uint8_t source, uint8_t mask) {
            if ((m_seqMapMask & (1u << plane)) == 0) {
                return;
            }

            uint8_t latch = m_latches[plane];
            uint8_t combined = static_cast<uint8_t>((source & mask) |
                                                    (latch & ~mask));
            m_planes[plane][planeOff] = combined;
        };

        switch (m_gcMode & 0x03) {
            case 0x00:
                for (int plane = 0; plane < 4; ++plane) {
                    uint8_t source = (m_gcEnableSetReset & (1u << plane)) != 0
                                   ? expandPlaneBit(m_gcSetReset, plane)
                                   : rotated;
                    uint8_t logical = applyLogicalOp(logicalOp, source, m_latches[plane]);
                    commitPlane(plane, logical, m_gcBitMask);
                }
                break;

            case 0x01:
                for (int plane = 0; plane < 4; ++plane) {
                    if ((m_seqMapMask & (1u << plane)) != 0) {
                        m_planes[plane][planeOff] = m_latches[plane];
                    }
                }
                break;

            case 0x02:
                for (int plane = 0; plane < 4; ++plane) {
                    uint8_t source = expandPlaneBit(value, plane);
                    uint8_t logical = applyLogicalOp(logicalOp, source, m_latches[plane]);
                    commitPlane(plane, logical, m_gcBitMask);
                }
                break;

            case 0x03: {
                uint8_t mode3Mask = rotated & m_gcBitMask;
                for (int plane = 0; plane < 4; ++plane) {
                    uint8_t source = expandPlaneBit(m_gcSetReset, plane);
                    commitPlane(plane, source, mode3Mask);
                }
                break;
            }
        }
    }
}

uint8_t fador::hw::VGAController::planeRead8(uint32_t offset) const {
    uint32_t planeOff = m_chain4 ? (offset >> 2) : (offset & 0xFFFF);
    loadLatches(planeOff);

    if ((m_gcMode & 0x08) != 0) {
        uint8_t compare = 0;
        for (int bit = 0; bit < 8; ++bit) {
            bool match = true;
            for (int plane = 0; plane < 4; ++plane) {
                if ((m_gcColorDontCare & (1u << plane)) == 0) {
                    continue;
                }

                bool latchBit = (m_latches[plane] & (1u << bit)) != 0;
                bool compareBit = (m_gcColorCompare & (1u << plane)) != 0;
                if (latchBit != compareBit) {
                    match = false;
                    break;
                }
            }

            if (match) {
                compare |= static_cast<uint8_t>(1u << bit);
            }
        }

        return compare;
    }

    if (m_chain4) {
        // Chain-4: address bits [1:0] select plane, [15:2] select offset
        uint8_t plane = offset & 3;
        return m_latches[plane];
    } else {
        // Mode-X / unchained: read from the plane selected by readMapSelect
        return m_latches[m_gcReadMap & 3];
    }
}
