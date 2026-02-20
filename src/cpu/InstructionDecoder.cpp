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
            triggerInterrupt(3);
            break;
            
        case 0xCD: { // INT imm8
            uint8_t vector = fetch8();
            triggerInterrupt(vector);
            break;
        }

        case 0xCE: { // INTO
            if (m_cpu.getEFLAGS() & 0x0800) { // Overflow flag (OF) is bit 11
                triggerInterrupt(4);
            }
            break;
        }

        case 0xCF: { // IRET (Real mode for now)
            if (m_hasPrefix66) { // 32-bit IRET
                uint32_t esp = m_cpu.getReg32(ESP);
                m_cpu.setEIP(m_memory.read32((m_cpu.getSegReg(SS) << 4) + esp));
                m_cpu.setSegReg(CS, m_memory.read32((m_cpu.getSegReg(SS) << 4) + esp + 4) & 0xFFFF);
                m_cpu.setEFLAGS(m_memory.read32((m_cpu.getSegReg(SS) << 4) + esp + 8));
                m_cpu.setReg32(ESP, esp + 12);
            } else { // 16-bit IRET
                uint16_t sp = m_cpu.getReg16(SP);
                m_cpu.setEIP(m_memory.read16((m_cpu.getSegReg(SS) << 4) + sp));
                m_cpu.setSegReg(CS, m_memory.read16((m_cpu.getSegReg(SS) << 4) + sp + 2));
                
                uint32_t currentFlags = m_cpu.getEFLAGS();
                uint16_t poppedFlags = m_memory.read16((m_cpu.getSegReg(SS) << 4) + sp + 4);
                m_cpu.setEFLAGS((currentFlags & 0xFFFF0000) | poppedFlags);
                
                m_cpu.setReg16(SP, sp + 6);
            }
            break;
        }

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

        case 0x50: case 0x51: case 0x52: case 0x53: case 0x54: case 0x55: case 0x56: case 0x57: { // PUSH r16/r32
            uint8_t opReg = opcode - 0x50;
            if (m_hasPrefix66) {
                uint32_t val = m_cpu.getReg32(opReg);
                uint32_t esp = m_cpu.getReg32(ESP) - 4;
                m_cpu.setReg32(ESP, esp);
                m_memory.write32((m_cpu.getSegReg(SS) << 4) + esp, val);
            } else {
                uint16_t val = m_cpu.getReg16(opReg);
                uint16_t sp = m_cpu.getReg16(SP) - 2;
                m_cpu.setReg16(SP, sp);
                m_memory.write16((m_cpu.getSegReg(SS) << 4) + sp, val);
            }
            break;
        }
        case 0x58: case 0x59: case 0x5A: case 0x5B: case 0x5C: case 0x5D: case 0x5E: case 0x5F: { // POP r16/r32
            uint8_t opReg = opcode - 0x58;
            if (m_hasPrefix66) {
                uint32_t esp = m_cpu.getReg32(ESP);
                uint32_t val = m_memory.read32((m_cpu.getSegReg(SS) << 4) + esp);
                m_cpu.setReg32(ESP, esp + 4);
                m_cpu.setReg32(opReg, val);
            } else {
                uint16_t sp = m_cpu.getReg16(SP);
                uint16_t val = m_memory.read16((m_cpu.getSegReg(SS) << 4) + sp);
                m_cpu.setReg16(SP, sp + 2);
                m_cpu.setReg16(opReg, val);
            }
            break;
        }
        // MOV r8, imm8
        case 0xB0: case 0xB1: case 0xB2: case 0xB3: case 0xB4: case 0xB5: case 0xB6: case 0xB7: {
            uint8_t opReg = opcode - 0xB0;
            m_cpu.setReg8(opReg, fetch8());
            break;
        }
        // MOV r16/32, imm16/32
        case 0xB8: case 0xB9: case 0xBA: case 0xBB: case 0xBC: case 0xBD: case 0xBE: case 0xBF: {
            uint8_t opReg = opcode - 0xB8;
            if (m_hasPrefix66) {
                m_cpu.setReg32(opReg, fetch32());
            } else {
                m_cpu.setReg16(opReg, fetch16());
            }
            break;
        }
        case 0x88: { // MOV r/m8, r8
            ModRM modrm = decodeModRM(fetch8());
            writeModRM8(modrm, m_cpu.getReg8(modrm.reg));
            break;
        }
        case 0x89: { // MOV r/m16/32, r16/32
            ModRM modrm = decodeModRM(fetch8());
            if (m_hasPrefix66) {
                writeModRM32(modrm, m_cpu.getReg32(modrm.reg));
            } else {
                writeModRM16(modrm, m_cpu.getReg16(modrm.reg));
            }
            break;
        }
        case 0x8A: { // MOV r8, r/m8
            ModRM modrm = decodeModRM(fetch8());
            m_cpu.setReg8(modrm.reg, readModRM8(modrm));
            break;
        }
        case 0x8B: { // MOV r16/32, r/m16/32
            ModRM modrm = decodeModRM(fetch8());
            if (m_hasPrefix66) {
                m_cpu.setReg32(modrm.reg, readModRM32(modrm));
            } else {
                m_cpu.setReg16(modrm.reg, readModRM16(modrm));
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

void InstructionDecoder::triggerInterrupt(uint8_t vector) {
    // 16-bit Real Mode Interrupt handling
    // 1. Push FLAGS, CS, IP
    // 2. Clear IF and TF flags (not fully implemented flag handling yet)
    // 3. Jump to IVT[vector]

    if (m_hasPrefix66) {
        LOG_WARN("32-bit protected mode interrupts not strictly implemented yet");
    }

    uint16_t sp = m_cpu.getReg16(SP);
    
    // Push FLAGS
    sp -= 2;
    m_memory.write16((m_cpu.getSegReg(SS) << 4) + sp, static_cast<uint16_t>(m_cpu.getEFLAGS() & 0xFFFF));
    // Push CS
    sp -= 2;
    m_memory.write16((m_cpu.getSegReg(SS) << 4) + sp, m_cpu.getSegReg(CS));
    // Push IP
    sp -= 2;
    m_memory.write16((m_cpu.getSegReg(SS) << 4) + sp, static_cast<uint16_t>(m_cpu.getEIP() & 0xFFFF));

    m_cpu.setReg16(SP, sp);

    // Read new CS:IP from IVT
    uint16_t newIP = m_memory.read16(vector * 4);
    uint16_t newCS = m_memory.read16((vector * 4) + 2);
    
    m_cpu.setSegReg(CS, newCS);
    m_cpu.setEIP(newIP);

    LOG_DEBUG("Triggered Interrupt 0x", std::hex, static_cast<int>(vector), 
              " Vector -> CS:EIP = 0x", newCS, ":0x", newIP);
}

} // namespace fador::cpu
