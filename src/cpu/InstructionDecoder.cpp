#include "InstructionDecoder.hpp"
#include "../utils/Logger.hpp"

namespace fador::cpu {

InstructionDecoder::InstructionDecoder(CPU& cpu, memory::MemoryBus& memory)
    : m_cpu(cpu)
    , m_memory(memory)
    , m_hasPrefix66(false)
    , m_hasPrefix67(false)
    , m_hasRepnz(false)
    , m_hasRepz(false)
    , m_segmentOverride(0xFF) {
}

uint8_t InstructionDecoder::fetch8() {
    uint32_t eip = m_cpu.getEIP();
    uint32_t csBase = m_cpu.getSegReg(CS) << 4; // Basic Real Mode Calculation
    uint8_t val = m_memory.read8(csBase + eip);
    m_cpu.setEIP(eip + 1);
    return val;
}

uint16_t InstructionDecoder::fetch16() {
    uint32_t eip = m_cpu.getEIP();
    uint32_t csBase = m_cpu.getSegReg(CS) << 4;
    uint16_t val = m_memory.read16(csBase + eip);
    m_cpu.setEIP(eip + 2);
    return val;
}

uint32_t InstructionDecoder::fetch32() {
    uint32_t eip = m_cpu.getEIP();
    uint32_t csBase = m_cpu.getSegReg(CS) << 4;
    uint32_t val = m_memory.read32(csBase + eip);
    m_cpu.setEIP(eip + 4);
    return val;
}

ModRM InstructionDecoder::decodeModRM(uint8_t byte) {
    return {
        static_cast<uint8_t>((byte >> 6) & 0x03),
        static_cast<uint8_t>((byte >> 3) & 0x07),
        static_cast<uint8_t>(byte & 0x07)
    };
}

SIB InstructionDecoder::decodeSIB(uint8_t byte) {
    return {
        static_cast<uint8_t>((byte >> 6) & 0x03),
        static_cast<uint8_t>((byte >> 3) & 0x07),
        static_cast<uint8_t>(byte & 0x07)
    };
}

// NOTE: Placeholder simplified implementations for now
uint32_t InstructionDecoder::getEffectiveAddress16(const ModRM& modrm) {
    // 16-bit addressing mode
    LOG_WARN("getEffectiveAddress16 not fully implemented");
    return 0;
}

uint32_t InstructionDecoder::getEffectiveAddress32(const ModRM& modrm) {
    // 32-bit addressing mode
    LOG_WARN("getEffectiveAddress32 not fully implemented");
    return 0;
}

uint32_t InstructionDecoder::readModRM32(const ModRM& modrm) {
    if (modrm.mod == 3) return m_cpu.getReg32(modrm.rm);
    uint32_t addr = m_hasPrefix67 ? getEffectiveAddress32(modrm) : getEffectiveAddress16(modrm);
    return m_memory.read32(addr);
}

uint16_t InstructionDecoder::readModRM16(const ModRM& modrm) {
    if (modrm.mod == 3) return m_cpu.getReg16(modrm.rm);
    uint32_t addr = m_hasPrefix67 ? getEffectiveAddress32(modrm) : getEffectiveAddress16(modrm);
    return m_memory.read16(addr);
}

uint8_t InstructionDecoder::readModRM8(const ModRM& modrm) {
    if (modrm.mod == 3) return m_cpu.getReg8(modrm.rm);
    uint32_t addr = m_hasPrefix67 ? getEffectiveAddress32(modrm) : getEffectiveAddress16(modrm);
    return m_memory.read8(addr);
}

void InstructionDecoder::writeModRM32(const ModRM& modrm, uint32_t value) {
    if (modrm.mod == 3) {
        m_cpu.setReg32(modrm.rm, value);
    } else {
        uint32_t addr = m_hasPrefix67 ? getEffectiveAddress32(modrm) : getEffectiveAddress16(modrm);
        m_memory.write32(addr, value);
    }
}

void InstructionDecoder::writeModRM16(const ModRM& modrm, uint16_t value) {
    if (modrm.mod == 3) {
        m_cpu.setReg16(modrm.rm, value);
    } else {
        uint32_t addr = m_hasPrefix67 ? getEffectiveAddress32(modrm) : getEffectiveAddress16(modrm);
        m_memory.write16(addr, value);
    }
}

void InstructionDecoder::writeModRM8(const ModRM& modrm, uint8_t value) {
    if (modrm.mod == 3) {
        m_cpu.setReg8(modrm.rm, value);
    } else {
        uint32_t addr = m_hasPrefix67 ? getEffectiveAddress32(modrm) : getEffectiveAddress16(modrm);
        m_memory.write8(addr, value);
    }
}

void InstructionDecoder::step() {
    m_hasPrefix66 = false;
    m_hasPrefix67 = false;
    m_hasRepnz = false;
    m_hasRepz = false;
    m_segmentOverride = 0xFF;

    uint8_t opcode = fetch8();
    
    // Process prefixes Loop (simplified)
    while (true) {
        switch (opcode) {
            case 0x66: m_hasPrefix66 = true; opcode = fetch8(); continue;
            case 0x67: m_hasPrefix67 = true; opcode = fetch8(); continue;
            case 0xF2: m_hasRepnz = true; opcode = fetch8(); continue;
            case 0xF3: m_hasRepz = true; opcode = fetch8(); continue;
            case 0x2E: m_segmentOverride = CS; opcode = fetch8(); continue;
            case 0x36: m_segmentOverride = SS; opcode = fetch8(); continue;
            case 0x3E: m_segmentOverride = DS; opcode = fetch8(); continue;
            case 0x26: m_segmentOverride = ES; opcode = fetch8(); continue;
            case 0x64: m_segmentOverride = FS; opcode = fetch8(); continue;
            case 0x65: m_segmentOverride = GS; opcode = fetch8(); continue;
            default: break; // No more prefixes
        }
        break;
    }

    if (opcode == 0x0F) {
        executeOpcode0F(fetch8());
    } else {
        executeOpcode(opcode);
    }
}

void InstructionDecoder::executeOpcode(uint8_t opcode) {
    switch (opcode) {
        case 0x90: // NOP
            break;
            
        case 0xCC: // INT3
            LOG_DEBUG("INT3 Exception triggered.");
            break;

        // Basic ADD
        case 0x00: { // ADD r/m8, r8
            ModRM modrm = decodeModRM(fetch8());
            uint8_t rmVal = readModRM8(modrm);
            uint8_t regVal = m_cpu.getReg8(modrm.reg);
            uint8_t res = rmVal + regVal;
            // TODO: EFLAGS
            writeModRM8(modrm, res);
            break;
        }
        case 0x01: { // ADD r/m16/32, r16/32
            ModRM modrm = decodeModRM(fetch8());
            if (m_hasPrefix66) { // 32-bit
                uint32_t rmVal = readModRM32(modrm);
                uint32_t regVal = m_cpu.getReg32(modrm.reg);
                writeModRM32(modrm, rmVal + regVal);
            } else { // 16-bit default in real mode
                uint16_t rmVal = readModRM16(modrm);
                uint16_t regVal = m_cpu.getReg16(modrm.reg);
                writeModRM16(modrm, rmVal + regVal);
            }
            break;
        }

        case 0xC3: { // RET (Near)
            if (m_hasPrefix66) {
                uint32_t esp = m_cpu.getReg32(ESP);
                uint32_t eip = m_memory.read32((m_cpu.getSegReg(SS) << 4) + esp);
                m_cpu.setReg32(ESP, esp + 4);
                m_cpu.setEIP(eip);
            } else {
                uint16_t sp = m_cpu.getReg16(SP);
                uint16_t ip = m_memory.read16((m_cpu.getSegReg(SS) << 4) + sp);
                m_cpu.setReg16(SP, sp + 2);
                m_cpu.setEIP(ip);
            }
            break;
        }
        case 0xE8: { // CALL rel16/32
            if (m_hasPrefix66) {
                int32_t rel32 = static_cast<int32_t>(fetch32());
                uint32_t esp = m_cpu.getReg32(ESP) - 4;
                m_cpu.setReg32(ESP, esp);
                m_memory.write32((m_cpu.getSegReg(SS) << 4) + esp, m_cpu.getEIP());
                m_cpu.setEIP(m_cpu.getEIP() + rel32);
            } else {
                int16_t rel16 = static_cast<int16_t>(fetch16());
                uint16_t sp = m_cpu.getReg16(SP) - 2;
                m_cpu.setReg16(SP, sp);
                m_memory.write16((m_cpu.getSegReg(SS) << 4) + sp, static_cast<uint16_t>(m_cpu.getEIP()));
                m_cpu.setEIP(static_cast<uint16_t>(m_cpu.getEIP() + rel16));
            }
            break;
        }
        case 0xE9: { // JMP rel16/32
            if (m_hasPrefix66) {
                int32_t rel32 = static_cast<int32_t>(fetch32());
                m_cpu.setEIP(m_cpu.getEIP() + rel32);
            } else {
                int16_t rel16 = static_cast<int16_t>(fetch16());
                m_cpu.setEIP(static_cast<uint16_t>(m_cpu.getEIP() + rel16));
            }
            break;
        }
        case 0xEB: { // JMP rel8
            int8_t rel8 = static_cast<int8_t>(fetch8());
            if (m_hasPrefix66) {
                m_cpu.setEIP(m_cpu.getEIP() + rel8);
            } else {
                m_cpu.setEIP(static_cast<uint16_t>(m_cpu.getEIP() + rel8));
            }
            break;
        }

        default:
            LOG_WARN("Unknown opcode 0x", std::hex, static_cast<int>(opcode), " at CS:EIP ", 
                     m_cpu.getSegReg(CS), ":", m_cpu.getEIP() - 1);
            break;
    }
}

void InstructionDecoder::executeOpcode0F(uint8_t opcode) {
    LOG_WARN("0x0F 0x", std::hex, static_cast<int>(opcode), " not implemented.");
}

} // namespace fador::cpu
