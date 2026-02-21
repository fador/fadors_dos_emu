#include "InstructionDecoder.hpp"
#include "../hw/IOBus.hpp"
#include "../hw/BIOS.hpp"
#include "../hw/DOS.hpp"
#include "../utils/Logger.hpp"

namespace fador::cpu {

InstructionDecoder::InstructionDecoder(CPU& cpu, memory::MemoryBus& memory, hw::IOBus& iobus, hw::BIOS& bios, hw::DOS& dos)
    : m_cpu(cpu)
    , m_memory(memory)
    , m_iobus(iobus)
    , m_bios(bios)
    , m_dos(dos)
    , m_hasPrefix66(false)
    , m_hasPrefix67(false)
    , m_hasRepnz(false)
    , m_hasRepz(false)
    , m_segmentOverride(0xFF)
    , m_currentEA(0)
    , m_eaResolved(false) 
{
    m_cpu.setMemoryBus(&m_memory);
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

// Calculates effective address based on ModRM and SIB
uint32_t InstructionDecoder::getEffectiveAddress16(const ModRM& modrm) {
    if (m_eaResolved) return m_currentEA;

    uint16_t addr = 0;
    uint8_t defaultSeg = DS;

    if (modrm.mod == 0 && modrm.rm == 6) {
        addr = fetch16();
    } else {
        switch (modrm.rm) {
            case 0: addr = m_cpu.getReg16(BX) + m_cpu.getReg16(SI); break;
            case 1: addr = m_cpu.getReg16(BX) + m_cpu.getReg16(DI); break;
            case 2: addr = m_cpu.getReg16(BP) + m_cpu.getReg16(SI); defaultSeg = SS; break;
            case 3: addr = m_cpu.getReg16(BP) + m_cpu.getReg16(DI); defaultSeg = SS; break;
            case 4: addr = m_cpu.getReg16(SI); break;
            case 5: addr = m_cpu.getReg16(DI); break;
            case 6: addr = m_cpu.getReg16(BP); defaultSeg = SS; break;
            case 7: addr = m_cpu.getReg16(BX); break;
        }

        if (modrm.mod == 1) {
            addr += static_cast<int8_t>(fetch8());
        } else if (modrm.mod == 2) {
            addr += static_cast<int16_t>(fetch16());
        }
    }

    uint8_t seg = (m_segmentOverride != 0xFF) ? m_segmentOverride : static_cast<uint8_t>(defaultSeg);
    m_currentEA = (m_cpu.getSegReg(static_cast<SegRegIndex>(seg)) << 4) + addr;
    m_eaResolved = true;
    return m_currentEA;
}

uint32_t InstructionDecoder::getEffectiveAddress32(const ModRM& modrm) {
    if (m_eaResolved) return m_currentEA;

    uint32_t addr = 0;
    uint8_t defaultSeg = DS;

    if (modrm.rm == 4) { // SIB byte follows
        SIB sib = decodeSIB(fetch8());
        uint32_t base = 0;
        if (sib.base == 5) {
            if (modrm.mod == 0) base = fetch32();
            else {
                base = m_cpu.getReg32(EBP);
                defaultSeg = SS;
            }
        } else {
            base = m_cpu.getReg32(sib.base);
            if (sib.base == ESP || sib.base == EBP) defaultSeg = SS;
        }

        uint32_t index = (sib.index == 4) ? 0 : m_cpu.getReg32(sib.index);
        addr = base + (index << sib.scale);
    } else {
        if (modrm.mod == 0 && modrm.rm == 5) {
            addr = fetch32();
        } else {
            addr = m_cpu.getReg32(modrm.rm);
            if (modrm.rm == ESP || modrm.rm == EBP) defaultSeg = SS;
        }
    }

    if (modrm.mod == 1) {
        addr += static_cast<int8_t>(fetch8());
    } else if (modrm.mod == 2) {
        addr += fetch32();
    }

    uint8_t seg = (m_segmentOverride != 0xFF) ? m_segmentOverride : defaultSeg;
    m_currentEA = (m_cpu.getSegReg(static_cast<SegRegIndex>(seg)) << 4) + addr;
    m_eaResolved = true;
    return m_currentEA;
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
    m_eaResolved = false;

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

        // Immediate AL/AX/EAX opcodes
        case 0x04: m_cpu.setReg8(AL, m_cpu.getReg8(AL) + fetch8()); break; // ADD AL, imm8
        case 0x05: { // ADD AX/EAX, imm16/32
            if (m_hasPrefix66) m_cpu.setReg32(EAX, m_cpu.getReg32(EAX) + fetch32());
            else m_cpu.setReg16(AX, m_cpu.getReg16(AX) + fetch16());
            break;
        }
        case 0x0C: m_cpu.setReg8(AL, m_cpu.getReg8(AL) | fetch8()); break; // OR AL, imm8
        case 0x0D: { // OR AX/EAX, imm16/32
            if (m_hasPrefix66) m_cpu.setReg32(EAX, m_cpu.getReg32(EAX) | fetch32());
            else m_cpu.setReg16(AX, m_cpu.getReg16(AX) | fetch16());
            break;
        }
        case 0x14: { // ADC AL, imm8
            uint8_t cf = (m_cpu.getEFLAGS() & FLAG_CARRY) ? 1 : 0;
            m_cpu.setReg8(AL, m_cpu.getReg8(AL) + fetch8() + cf);
            break;
        }
        case 0x15: { // ADC AX/EAX, imm16/32
            uint8_t cf = (m_cpu.getEFLAGS() & FLAG_CARRY) ? 1 : 0;
            if (m_hasPrefix66) m_cpu.setReg32(EAX, m_cpu.getReg32(EAX) + fetch32() + cf);
            else m_cpu.setReg16(AX, m_cpu.getReg16(AX) + fetch16() + cf);
            break;
        }
        case 0x1C: { // SBB AL, imm8
            uint8_t cf = (m_cpu.getEFLAGS() & FLAG_CARRY) ? 1 : 0;
            m_cpu.setReg8(AL, m_cpu.getReg8(AL) - (fetch8() + cf));
            break;
        }
        case 0x1D: { // SBB AX/EAX, imm16/32
            uint8_t cf = (m_cpu.getEFLAGS() & FLAG_CARRY) ? 1 : 0;
            if (m_hasPrefix66) m_cpu.setReg32(EAX, m_cpu.getReg32(EAX) - (fetch32() + cf));
            else m_cpu.setReg16(AX, m_cpu.getReg16(AX) - (fetch16() + cf));
            break;
        }
        case 0x24: m_cpu.setReg8(AL, m_cpu.getReg8(AL) & fetch8()); break; // AND AL, imm8
        case 0x25: { // AND AX/EAX, imm16/32
            if (m_hasPrefix66) m_cpu.setReg32(EAX, m_cpu.getReg32(EAX) & fetch32());
            else m_cpu.setReg16(AX, m_cpu.getReg16(AX) & fetch16());
            break;
        }
        case 0x2C: m_cpu.setReg8(AL, m_cpu.getReg8(AL) - fetch8()); break; // SUB AL, imm8
        case 0x2D: { // SUB AX/EAX, imm16/32
            if (m_hasPrefix66) m_cpu.setReg32(EAX, m_cpu.getReg32(EAX) - fetch32());
            else m_cpu.setReg16(AX, m_cpu.getReg16(AX) - fetch16());
            break;
        }
        case 0x34: m_cpu.setReg8(AL, m_cpu.getReg8(AL) ^ fetch8()); break; // XOR AL, imm8
        case 0x35: { // XOR AX/EAX, imm16/32
            if (m_hasPrefix66) m_cpu.setReg32(EAX, m_cpu.getReg32(EAX) ^ fetch32());
            else m_cpu.setReg16(AX, m_cpu.getReg16(AX) ^ fetch16());
            break;
        }
        case 0x3C: { // CMP AL, imm8
            fetch8(); // TODO: flags
            break;
        }
        case 0x3D: { // CMP AX/EAX, imm16/32
            if (m_hasPrefix66) fetch32(); else fetch16(); // TODO: flags
            break;
        }
        case 0x3F: LOG_DEBUG("AAS (0x3F) - stubbed"); break;
        case 0xD4: fetch8(); LOG_DEBUG("AAM (0xD4) - stubbed"); break;
        case 0xD5: fetch8(); LOG_DEBUG("AAD (0xD5) - stubbed"); break;
        case 0x0E: m_cpu.push16(m_cpu.getSegReg(CS)); break;
        case 0x16: m_cpu.push16(m_cpu.getSegReg(SS)); break;
        case 0x17: m_cpu.setSegReg(SS, m_cpu.pop16()); break;
        case 0x1E: m_cpu.push16(m_cpu.getSegReg(DS)); break;
        case 0x1F: m_cpu.setSegReg(DS, m_cpu.pop16()); break;

        // ADD
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
            if (m_hasPrefix66) {
                uint32_t rmVal = readModRM32(modrm);
                uint32_t regVal = m_cpu.getReg32(modrm.reg);
                uint32_t res = rmVal + regVal;
                writeModRM32(modrm, res);
            } else {
                uint16_t rmVal = readModRM16(modrm);
                uint16_t regVal = m_cpu.getReg16(modrm.reg);
                uint16_t res = rmVal + regVal;
                writeModRM16(modrm, res);
            }
            break;
        }
        case 0x02: { // ADD r8, r/m8
            ModRM modrm = decodeModRM(fetch8());
            uint8_t rmVal = readModRM8(modrm);
            uint8_t regVal = m_cpu.getReg8(modrm.reg);
            uint8_t res = regVal + rmVal;
            m_cpu.setReg8(modrm.reg, res);
            break;
        }
        case 0x03: { // ADD r16/32, r/m16/32
            ModRM modrm = decodeModRM(fetch8());
            if (m_hasPrefix66) {
                uint32_t rmVal = readModRM32(modrm);
                uint32_t regVal = m_cpu.getReg32(modrm.reg);
                uint32_t res = regVal + rmVal;
                m_cpu.setReg32(modrm.reg, res);
            } else {
                uint16_t rmVal = readModRM16(modrm);
                uint16_t regVal = m_cpu.getReg16(modrm.reg);
                uint16_t res = regVal + rmVal;
                m_cpu.setReg16(modrm.reg, res);
            }
            break;
        }

        // ADC
        case 0x10: { // ADC r/m8, r8
            ModRM modrm = decodeModRM(fetch8());
            uint8_t rmVal = readModRM8(modrm);
            uint8_t regVal = m_cpu.getReg8(modrm.reg);
            uint8_t cf = (m_cpu.getEFLAGS() & FLAG_CARRY) ? 1 : 0;
            writeModRM8(modrm, rmVal + regVal + cf);
            break;
        }
        case 0x11: { // ADC r/m16/32, r16/32
            ModRM modrm = decodeModRM(fetch8());
            uint8_t cf = (m_cpu.getEFLAGS() & FLAG_CARRY) ? 1 : 0;
            if (m_hasPrefix66) writeModRM32(modrm, readModRM32(modrm) + m_cpu.getReg32(modrm.reg) + cf);
            else writeModRM16(modrm, readModRM16(modrm) + m_cpu.getReg16(modrm.reg) + cf);
            break;
        }
        case 0x12: { // ADC r8, r/m8
            ModRM modrm = decodeModRM(fetch8());
            uint8_t cf = (m_cpu.getEFLAGS() & FLAG_CARRY) ? 1 : 0;
            m_cpu.setReg8(modrm.reg, m_cpu.getReg8(modrm.reg) + readModRM8(modrm) + cf);
            break;
        }
        case 0x13: { // ADC r16/32, r/m16/32
            ModRM modrm = decodeModRM(fetch8());
            uint8_t cf = (m_cpu.getEFLAGS() & FLAG_CARRY) ? 1 : 0;
            if (m_hasPrefix66) m_cpu.setReg32(modrm.reg, m_cpu.getReg32(modrm.reg) + readModRM32(modrm) + cf);
            else m_cpu.setReg16(modrm.reg, m_cpu.getReg16(modrm.reg) + readModRM16(modrm) + cf);
            break;
        }

        // SBB
        case 0x18: { // SBB r/m8, r8
            ModRM modrm = decodeModRM(fetch8());
            uint8_t rmVal = readModRM8(modrm);
            uint8_t regVal = m_cpu.getReg8(modrm.reg);
            uint8_t cf = (m_cpu.getEFLAGS() & FLAG_CARRY) ? 1 : 0;
            writeModRM8(modrm, rmVal - (regVal + cf));
            break;
        }
        case 0x19: { // SBB r/m16/32, r16/32
            ModRM modrm = decodeModRM(fetch8());
            uint8_t cf = (m_cpu.getEFLAGS() & FLAG_CARRY) ? 1 : 0;
            if (m_hasPrefix66) writeModRM32(modrm, readModRM32(modrm) - (m_cpu.getReg32(modrm.reg) + cf));
            else writeModRM16(modrm, readModRM16(modrm) - (m_cpu.getReg16(modrm.reg) + cf));
            break;
        }
        case 0x1A: { // SBB r8, r/m8
            ModRM modrm = decodeModRM(fetch8());
            uint8_t cf = (m_cpu.getEFLAGS() & FLAG_CARRY) ? 1 : 0;
            m_cpu.setReg8(modrm.reg, m_cpu.getReg8(modrm.reg) - (readModRM8(modrm) + cf));
            break;
        }
        case 0x1B: { // SBB r16/32, r/m16/32
            ModRM modrm = decodeModRM(fetch8());
            uint8_t cf = (m_cpu.getEFLAGS() & FLAG_CARRY) ? 1 : 0;
            if (m_hasPrefix66) m_cpu.setReg32(modrm.reg, m_cpu.getReg32(modrm.reg) - (readModRM32(modrm) + cf));
            else m_cpu.setReg16(modrm.reg, m_cpu.getReg16(modrm.reg) - (readModRM16(modrm) + cf));
            break;
        }

        // OR
        case 0x08: { // OR r/m8, r8
            ModRM modrm = decodeModRM(fetch8());
            uint8_t rmVal = readModRM8(modrm);
            uint8_t regVal = m_cpu.getReg8(modrm.reg);
            uint8_t res = rmVal | regVal;
            // TODO: EFLAGS
            writeModRM8(modrm, res);
            break;
        }
        case 0x09: { // OR r/m16/32, r16/32
            ModRM modrm = decodeModRM(fetch8());
            if (m_hasPrefix66) {
                uint32_t rmVal = readModRM32(modrm);
                uint32_t regVal = m_cpu.getReg32(modrm.reg);
                uint32_t res = rmVal | regVal;
                writeModRM32(modrm, res);
            } else {
                uint16_t rmVal = readModRM16(modrm);
                uint16_t regVal = m_cpu.getReg16(modrm.reg);
                uint16_t res = rmVal | regVal;
                writeModRM16(modrm, res);
            }
            break;
        }
        case 0x0A: { // OR r8, r/m8
            ModRM modrm = decodeModRM(fetch8());
            uint8_t rmVal = readModRM8(modrm);
            uint8_t regVal = m_cpu.getReg8(modrm.reg);
            uint8_t res = regVal | rmVal;
            m_cpu.setReg8(modrm.reg, res);
            break;
        }
        case 0x0B: { // OR r16/32, r/m16/32
            ModRM modrm = decodeModRM(fetch8());
            if (m_hasPrefix66) {
                uint32_t rmVal = readModRM32(modrm);
                uint32_t regVal = m_cpu.getReg32(modrm.reg);
                uint32_t res = regVal | rmVal;
                m_cpu.setReg32(modrm.reg, res);
            } else {
                uint16_t rmVal = readModRM16(modrm);
                uint16_t regVal = m_cpu.getReg16(modrm.reg);
                uint16_t res = regVal | rmVal;
                m_cpu.setReg16(modrm.reg, res);
            }
            break;
        }

        // AND
        case 0x20: { // AND r/m8, r8
            ModRM modrm = decodeModRM(fetch8());
            uint8_t rmVal = readModRM8(modrm);
            uint8_t regVal = m_cpu.getReg8(modrm.reg);
            uint8_t res = rmVal & regVal;
            // TODO: EFLAGS
            writeModRM8(modrm, res);
            break;
        }
        case 0x21: { // AND r/m16/32, r16/32
            ModRM modrm = decodeModRM(fetch8());
            if (m_hasPrefix66) {
                uint32_t rmVal = readModRM32(modrm);
                uint32_t regVal = m_cpu.getReg32(modrm.reg);
                uint32_t res = rmVal & regVal;
                writeModRM32(modrm, res);
            } else {
                uint16_t rmVal = readModRM16(modrm);
                uint16_t regVal = m_cpu.getReg16(modrm.reg);
                uint16_t res = rmVal & regVal;
                writeModRM16(modrm, res);
            }
            break;
        }
        case 0x22: { // AND r8, r/m8
            ModRM modrm = decodeModRM(fetch8());
            uint8_t rmVal = readModRM8(modrm);
            uint8_t regVal = m_cpu.getReg8(modrm.reg);
            uint8_t res = regVal & rmVal;
            m_cpu.setReg8(modrm.reg, res);
            break;
        }
        case 0x23: { // AND r16/32, r/m16/32
            ModRM modrm = decodeModRM(fetch8());
            if (m_hasPrefix66) {
                uint32_t rmVal = readModRM32(modrm);
                uint32_t regVal = m_cpu.getReg32(modrm.reg);
                uint32_t res = regVal & rmVal;
                m_cpu.setReg32(modrm.reg, res);
            } else {
                uint16_t rmVal = readModRM16(modrm);
                uint16_t regVal = m_cpu.getReg16(modrm.reg);
                uint16_t res = regVal & rmVal;
                m_cpu.setReg16(modrm.reg, res);
            }
            break;
        }

        // SUB
        case 0x28: { // SUB r/m8, r8
            ModRM modrm = decodeModRM(fetch8());
            uint8_t rmVal = readModRM8(modrm);
            uint8_t regVal = m_cpu.getReg8(modrm.reg);
            uint8_t res = rmVal - regVal;
            // TODO: EFLAGS
            writeModRM8(modrm, res);
            break;
        }
        case 0x29: { // SUB r/m16/32, r16/32
            ModRM modrm = decodeModRM(fetch8());
            if (m_hasPrefix66) {
                uint32_t rmVal = readModRM32(modrm);
                uint32_t regVal = m_cpu.getReg32(modrm.reg);
                uint32_t res = rmVal - regVal;
                writeModRM32(modrm, res);
            } else {
                uint16_t rmVal = readModRM16(modrm);
                uint16_t regVal = m_cpu.getReg16(modrm.reg);
                uint16_t res = rmVal - regVal;
                writeModRM16(modrm, res);
            }
            break;
        }
        case 0x2A: { // SUB r8, r/m8
            ModRM modrm = decodeModRM(fetch8());
            uint8_t rmVal = readModRM8(modrm);
            uint8_t regVal = m_cpu.getReg8(modrm.reg);
            uint8_t res = regVal - rmVal;
            m_cpu.setReg8(modrm.reg, res);
            break;
        }
        case 0x2B: { // SUB r16/32, r/m16/32
            ModRM modrm = decodeModRM(fetch8());
            if (m_hasPrefix66) {
                uint32_t rmVal = readModRM32(modrm);
                uint32_t regVal = m_cpu.getReg32(modrm.reg);
                uint32_t res = regVal - rmVal;
                m_cpu.setReg32(modrm.reg, res);
            } else {
                uint16_t rmVal = readModRM16(modrm);
                uint16_t regVal = m_cpu.getReg16(modrm.reg);
                uint16_t res = regVal - rmVal;
                m_cpu.setReg16(modrm.reg, res);
            }
            break;
        }

        // INC register 16/32
        case 0x40: case 0x41: case 0x42: case 0x43:
        case 0x44: case 0x45: case 0x46: case 0x47: {
            uint8_t reg = opcode & 0x07;
            if (m_hasPrefix66) m_cpu.setReg32(reg, m_cpu.getReg32(reg) + 1);
            else m_cpu.setReg16(reg, m_cpu.getReg16(reg) + 1);
            break;
        }
        // DEC register 16/32
        case 0x48: case 0x49: case 0x4A: case 0x4B:
        case 0x4C: case 0x4D: case 0x4E: case 0x4F: {
            uint8_t reg = opcode & 0x07;
            if (m_hasPrefix66) m_cpu.setReg32(reg, m_cpu.getReg32(reg) - 1);
            else m_cpu.setReg16(reg, m_cpu.getReg16(reg) - 1);
            break;
        }

        // PUSH register 16/32
        case 0x50: case 0x51: case 0x52: case 0x53:
        case 0x54: case 0x55: case 0x56: case 0x57: {
            uint8_t reg = opcode & 0x07;
            if (m_hasPrefix66) m_cpu.push32(m_cpu.getReg32(reg));
            else m_cpu.push16(m_cpu.getReg16(reg));
            break;
        }
        // POP register 16/32
        case 0x58: case 0x59: case 0x5A: case 0x5B:
        case 0x5C: case 0x5D: case 0x5E: case 0x5F: {
            uint8_t reg = opcode & 0x07;
            if (m_hasPrefix66) m_cpu.setReg32(reg, m_cpu.pop32());
            else m_cpu.setReg16(reg, m_cpu.pop16());
            break;
        }

        // Short Jumps 0x70 - 0x7F
        case 0x70: case 0x71: case 0x72: case 0x73:
        case 0x74: case 0x75: case 0x76: case 0x77:
        case 0x78: case 0x79: case 0x7A: case 0x7B:
        case 0x7C: case 0x7D: case 0x7E: case 0x7F: {
            int8_t disp = static_cast<int8_t>(fetch8());
            bool jump = false;
            uint32_t flags = m_cpu.getEFLAGS();
            bool zf = flags & 0x0040;
            bool cf = flags & 0x0001;
            bool sf = flags & 0x0080;
            bool of = flags & 0x0800;
            bool pf = flags & 0x0004;

            switch (opcode) {
                case 0x70: jump = of; break; // JO
                case 0x71: jump = !of; break; // JNO
                case 0x72: jump = cf; break; // JB / JNAE / JC
                case 0x73: jump = !cf; break; // JNB / JAE / JNC
                case 0x74: jump = zf; break; // JZ / JE
                case 0x75: jump = !zf; break; // JNZ / JNE
                case 0x76: jump = cf || zf; break; // JBE / JNA
                case 0x77: jump = !cf && !zf; break; // JNBE / JA
                case 0x78: jump = sf; break; // JS
                case 0x79: jump = !sf; break; // JNS
                case 0x7A: jump = pf; break; // JP / JPE
                case 0x7B: jump = !pf; break; // JNP / JPO
                case 0x7C: jump = sf != of; break; // JL / JNGE
                case 0x7D: jump = sf == of; break; // JNL / JGE
                case 0x7E: jump = zf || (sf != of); break; // JLE / JNG
                case 0x7F: jump = !zf && (sf == of); break; // JNLE / JG
            }
            if (jump) m_cpu.setEIP(m_cpu.getEIP() + disp);
            break;
        }

        // MOV
        case 0x88: { // MOV r/m8, r8
            ModRM modrm = decodeModRM(fetch8());
            writeModRM8(modrm, m_cpu.getReg8(modrm.reg));
            break;
        }
        case 0x89: { // MOV r/m16/32, r16/32
            ModRM modrm = decodeModRM(fetch8());
            if (m_hasPrefix66) writeModRM32(modrm, m_cpu.getReg32(modrm.reg));
            else writeModRM16(modrm, m_cpu.getReg16(modrm.reg));
            break;
        }
        case 0x8A: { // MOV r8, r/m8
            ModRM modrm = decodeModRM(fetch8());
            m_cpu.setReg8(modrm.reg, readModRM8(modrm));
            break;
        }
        case 0x8B: { // MOV r16/32, r/m16/32
            ModRM modrm = decodeModRM(fetch8());
            if (m_hasPrefix66) m_cpu.setReg32(modrm.reg, readModRM32(modrm));
            else m_cpu.setReg16(modrm.reg, readModRM16(modrm));
            break;
        }
        case 0x8C: { // MOV r/m16, Sreg
            ModRM modrm = decodeModRM(fetch8());
            writeModRM16(modrm, m_cpu.getSegReg(static_cast<SegRegIndex>(modrm.reg)));
            break;
        }
        case 0xF1: { // INT1 / ICEBP
            LOG_DEBUG("ICEBP (0xF1) treated as NOP");
            break;
        }

        case 0x8D: { // LEA r16/32, m
            ModRM modrm = decodeModRM(fetch8());
            uint32_t addr = (m_hasPrefix67) ? getEffectiveAddress32(modrm) : getEffectiveAddress16(modrm);
            // LEA only uses the offset, not the segment base
            uint16_t offset = static_cast<uint16_t>(addr & 0xFFFF);
            if (m_hasPrefix66) m_cpu.setReg32(modrm.reg, addr); // This is simplified
            else m_cpu.setReg16(modrm.reg, offset);
            break;
        }
        case 0x8E: { // MOV Sreg, r/m16
            ModRM modrm = decodeModRM(fetch8());
            m_cpu.setSegReg(static_cast<SegRegIndex>(modrm.reg), readModRM16(modrm));
            break;
        }

        case 0xC7: { // MOV r/m16/32, imm16/32
            ModRM modrm = decodeModRM(fetch8());
            if (m_hasPrefix66) writeModRM32(modrm, fetch32());
            else writeModRM16(modrm, fetch16());
            break;
        }


        // LOOPs
        case 0xE0: case 0xE1: case 0xE2: case 0xE3: {
            int8_t disp = static_cast<int8_t>(fetch8());
            uint32_t count = m_hasPrefix67 ? m_cpu.getReg32(ECX) : m_cpu.getReg16(CX);
            bool jump = false;
            if (opcode == 0xE3) { // JCXZ/JECXZ
                jump = (count == 0);
            } else {
                count--;
                if (m_hasPrefix67) m_cpu.setReg32(ECX, count);
                else m_cpu.setReg16(CX, static_cast<uint16_t>(count));
                
                if (opcode == 0xE2) jump = (count != 0); // LOOP
                else if (opcode == 0xE1) jump = (count != 0 && (m_cpu.getEFLAGS() & FLAG_ZERO)); // LOOPE
                else if (opcode == 0xE0) jump = (count != 0 && !(m_cpu.getEFLAGS() & FLAG_ZERO)); // LOOPNE
            }
            if (jump) m_cpu.setEIP(m_cpu.getEIP() + disp);
            break;
        }

        // ENTER / LEAVE
        case 0xC8: { // ENTER imm16, imm8
            uint16_t size = fetch16();
            uint8_t level = fetch8() & 0x1F;
            if (m_hasPrefix66) {
                m_cpu.push32(m_cpu.getReg32(EBP));
                uint32_t framePtr = m_cpu.getReg32(ESP);
                if (level > 0) LOG_WARN("ENTER level > 0 not implemented");
                m_cpu.setReg32(EBP, framePtr);
                m_cpu.setReg32(ESP, framePtr - size);
            } else {
                m_cpu.push16(m_cpu.getReg16(BP));
                uint16_t framePtr = m_cpu.getReg16(SP);
                if (level > 0) LOG_WARN("ENTER level > 0 not implemented");
                m_cpu.setReg16(BP, framePtr);
                m_cpu.setReg16(SP, framePtr - size);
            }
            break;
        }
        case 0xC9: { // LEAVE
            if (m_hasPrefix66) {
                m_cpu.setReg32(ESP, m_cpu.getReg32(EBP));
                m_cpu.setReg32(EBP, m_cpu.pop32());
            } else {
                m_cpu.setReg16(SP, m_cpu.getReg16(BP));
                m_cpu.setReg16(BP, m_cpu.pop16());
            }
            break;
        }

        case 0xCB: { // RETF
            uint32_t ip = 0, cs = 0;
            if (m_hasPrefix66) {
                ip = m_cpu.pop32();
                cs = m_cpu.pop32() & 0xFFFF;
            } else {
                ip = m_cpu.pop16();
                cs = m_cpu.pop16();
            }
            m_cpu.setEIP(ip);
            m_cpu.setSegReg(CS, static_cast<uint16_t>(cs));
            break;
        }

        case 0xA0: { // MOV AL, [moffs8]
            uint32_t addr = (m_hasPrefix67) ? fetch32() : fetch16();
            uint8_t seg = (m_segmentOverride != 0xFF) ? m_segmentOverride : static_cast<uint8_t>(DS);
            m_cpu.setReg8(AL, m_memory.read8((m_cpu.getSegReg(static_cast<SegRegIndex>(seg)) << 4) + addr));
            break;
        }
        case 0xA1: { // MOV AX/EAX, [moffs16/32]
            uint32_t addr = (m_hasPrefix67) ? fetch32() : fetch16();
            uint8_t seg = (m_segmentOverride != 0xFF) ? m_segmentOverride : static_cast<uint8_t>(DS);
            uint32_t finalAddr = (m_cpu.getSegReg(static_cast<SegRegIndex>(seg)) << 4) + addr;
            if (m_hasPrefix66) m_cpu.setReg32(EAX, m_memory.read32(finalAddr));
            else m_cpu.setReg16(AX, m_memory.read16(finalAddr));
            break;
        }
        case 0xA2: { // MOV [moffs8], AL
            uint32_t addr = (m_hasPrefix67) ? fetch32() : fetch16();
            uint8_t seg = (m_segmentOverride != 0xFF) ? m_segmentOverride : static_cast<uint8_t>(DS);
            m_memory.write8((m_cpu.getSegReg(static_cast<SegRegIndex>(seg)) << 4) + addr, m_cpu.getReg8(AL));
            break;
        }
        case 0xA3: { // MOV [moffs16/32], AX/EAX
            uint32_t addr = (m_hasPrefix67) ? fetch32() : fetch16();
            uint8_t seg = (m_segmentOverride != 0xFF) ? m_segmentOverride : static_cast<uint8_t>(DS);
            uint32_t finalAddr = (m_cpu.getSegReg(static_cast<SegRegIndex>(seg)) << 4) + addr;
            if (m_hasPrefix66) m_memory.write32(finalAddr, m_cpu.getReg32(EAX));
            else m_memory.write16(finalAddr, m_cpu.getReg16(AX));
            break;
        }

        // XOR
        case 0x30: { // XOR r/m8, r8
            ModRM modrm = decodeModRM(fetch8());
            uint8_t rmVal = readModRM8(modrm);
            uint8_t regVal = m_cpu.getReg8(modrm.reg);
            uint8_t res = rmVal ^ regVal;
            // TODO: EFLAGS
            writeModRM8(modrm, res);
            break;
        }
        case 0x31: { // XOR r/m16/32, r16/32
            ModRM modrm = decodeModRM(fetch8());
            if (m_hasPrefix66) {
                uint32_t rmVal = readModRM32(modrm);
                uint32_t regVal = m_cpu.getReg32(modrm.reg);
                uint32_t res = rmVal ^ regVal;
                writeModRM32(modrm, res);
            } else {
                uint16_t rmVal = readModRM16(modrm);
                uint16_t regVal = m_cpu.getReg16(modrm.reg);
                uint16_t res = rmVal ^ regVal;
                writeModRM16(modrm, res);
            }
            break;
        }
        case 0x32: { // XOR r8, r/m8
            ModRM modrm = decodeModRM(fetch8());
            uint8_t rmVal = readModRM8(modrm);
            uint8_t regVal = m_cpu.getReg8(modrm.reg);
            uint8_t res = regVal ^ rmVal;
            m_cpu.setReg8(modrm.reg, res);
            break;
        }
        case 0x33: { // XOR r16/32, r/m16/32
            ModRM modrm = decodeModRM(fetch8());
            if (m_hasPrefix66) {
                uint32_t rmVal = readModRM32(modrm);
                uint32_t regVal = m_cpu.getReg32(modrm.reg);
                uint32_t res = regVal ^ rmVal;
                m_cpu.setReg32(modrm.reg, res);
            } else {
                uint16_t rmVal = readModRM16(modrm);
                uint16_t regVal = m_cpu.getReg16(modrm.reg);
                uint16_t res = regVal ^ rmVal;
                m_cpu.setReg16(modrm.reg, res);
            }
            break;
        }

        // CMP
        case 0x38: { // CMP r/m8, r8
            ModRM modrm = decodeModRM(fetch8());
            uint8_t rmVal = readModRM8(modrm);
            uint8_t regVal = m_cpu.getReg8(modrm.reg);
            uint8_t res = rmVal - regVal;
            // TODO: EFLAGS
            (void)res;
            break;
        }
        case 0x39: { // CMP r/m16/32, r16/32
            ModRM modrm = decodeModRM(fetch8());
            if (m_hasPrefix66) {
                uint32_t rmVal = readModRM32(modrm);
                uint32_t regVal = m_cpu.getReg32(modrm.reg);
                uint32_t res = rmVal - regVal;
                (void)res;
            } else {
                uint16_t rmVal = readModRM16(modrm);
                uint16_t regVal = m_cpu.getReg16(modrm.reg);
                uint16_t res = rmVal - regVal;
                (void)res;
            }
            break;
        }
        case 0x3A: { // CMP r8, r/m8
            ModRM modrm = decodeModRM(fetch8());
            uint8_t rmVal = readModRM8(modrm);
            uint8_t regVal = m_cpu.getReg8(modrm.reg);
            uint8_t res = regVal - rmVal;
            (void)res;
            break;
        }
        case 0x3B: { // CMP r16/32, r/m16/32
            ModRM modrm = decodeModRM(fetch8());
            if (m_hasPrefix66) {
                uint32_t rmVal = readModRM32(modrm);
                uint32_t regVal = m_cpu.getReg32(modrm.reg);
                uint32_t res = regVal - rmVal;
                (void)res;
            } else {
                uint16_t rmVal = readModRM16(modrm);
                uint16_t regVal = m_cpu.getReg16(modrm.reg);
                uint16_t res = regVal - rmVal;
                (void)res;
            }
            break;
        }

        // Group 1: 0x80, 0x81, 0x82, 0x83
        case 0x80: case 0x81: case 0x82: case 0x83: {
            ModRM modrm = decodeModRM(fetch8());
            uint32_t val1 = 0;
            uint32_t val2 = 0;
            bool is8 = (opcode == 0x80 || opcode == 0x82);
            bool signExt = (opcode == 0x83);

            if (is8) {
                val1 = readModRM8(modrm);
                val2 = fetch8();
            } else if (signExt) {
                if (m_hasPrefix66) {
                    val1 = readModRM32(modrm);
                    val2 = static_cast<int32_t>(static_cast<int8_t>(fetch8()));
                } else {
                    val1 = readModRM16(modrm);
                    val2 = static_cast<int16_t>(static_cast<int8_t>(fetch8()));
                }
            } else {
                if (m_hasPrefix66) {
                    val1 = readModRM32(modrm);
                    val2 = fetch32();
                } else {
                    val1 = readModRM16(modrm);
                    val2 = fetch16();
                }
            }

            uint32_t res = 0;
            switch (modrm.reg) {
                case 0: res = val1 + val2; break; // ADD
                case 1: res = val1 | val2; break; // OR
                case 2: res = val1 + val2; break; // ADC (simplified)
                case 3: res = val1 - val2; break; // SBB (simplified)
                case 4: res = val1 & val2; break; // AND
                case 5: res = val1 - val2; break; // SUB
                case 6: res = val1 ^ val2; break; // XOR
                case 7: res = val1 - val2; break; // CMP
            }

            if (modrm.reg != 7) {
                if (is8) writeModRM8(modrm, static_cast<uint8_t>(res));
                else if (m_hasPrefix66) writeModRM32(modrm, res);
                else writeModRM16(modrm, static_cast<uint16_t>(res));
            }
            break;
        }

        // Group 3: 0xF6, 0xF7
        case 0xF6:
        case 0xF7: {
            ModRM modrm = decodeModRM(fetch8());
            if (opcode == 0xF6) {
                uint8_t rmVal = readModRM8(modrm);
                switch (modrm.reg) {
                    case 0: case 1: { // TEST rm8, imm8
                        uint8_t imm = fetch8();
                        (void)(rmVal & imm);
                        break;
                    }
                    case 2: writeModRM8(modrm, ~rmVal); break; // NOT
                    case 3: writeModRM8(modrm, static_cast<uint8_t>(0 - rmVal)); break; // NEG
                    case 4: { // MUL
                        uint16_t res = static_cast<uint16_t>(m_cpu.getReg8(AL)) * rmVal;
                        m_cpu.setReg16(AX, res);
                        break;
                    }
                    case 5: { // IMUL
                        int16_t res = static_cast<int16_t>(static_cast<int8_t>(m_cpu.getReg8(AL))) * static_cast<int8_t>(rmVal);
                        m_cpu.setReg16(AX, static_cast<uint16_t>(res));
                        break;
                    }
                    case 6: { // DIV
                        if (rmVal == 0) triggerInterrupt(0);
                        else {
                            uint16_t ax = m_cpu.getReg16(AX);
                            m_cpu.setReg8(AL, static_cast<uint8_t>(ax / rmVal));
                            m_cpu.setReg8(AH, static_cast<uint8_t>(ax % rmVal));
                        }
                        break;
                    }
                    case 7: { // IDIV
                        if (rmVal == 0) triggerInterrupt(0);
                        else {
                            int16_t ax = static_cast<int16_t>(m_cpu.getReg16(AX));
                            int8_t div = static_cast<int8_t>(rmVal);
                            m_cpu.setReg8(AL, static_cast<uint8_t>(ax / div));
                            m_cpu.setReg8(AH, static_cast<uint8_t>(ax % div));
                        }
                        break;
                    }
                }
            } else {
                if (m_hasPrefix66) {
                    uint32_t rmVal = readModRM32(modrm);
                    switch (modrm.reg) {
                        case 0: case 1: { // TEST rm32, imm32
                            uint32_t imm = fetch32();
                            (void)(rmVal & imm);
                            break;
                        }
                        case 2: writeModRM32(modrm, ~rmVal); break; // NOT
                        case 3: writeModRM32(modrm, 0 - rmVal); break; // NEG
                        case 4: { // MUL
                            uint64_t res = static_cast<uint64_t>(m_cpu.getReg32(EAX)) * rmVal;
                            m_cpu.setReg32(EAX, static_cast<uint32_t>(res));
                            m_cpu.setReg32(EDX, static_cast<uint32_t>(res >> 32));
                            break;
                        }
                        case 7: { // IDIV
                            if (rmVal == 0) triggerInterrupt(0);
                            else {
                                int64_t edxeax = (static_cast<int64_t>(m_cpu.getReg32(EDX)) << 32) | m_cpu.getReg32(EAX);
                                m_cpu.setReg32(EAX, static_cast<uint32_t>(edxeax / static_cast<int32_t>(rmVal)));
                                m_cpu.setReg32(EDX, static_cast<uint32_t>(edxeax % static_cast<int32_t>(rmVal)));
                            }
                            break;
                        }
                        default: LOG_WARN("Group 3 r32 reg ", (int)modrm.reg, " not fully implemented"); break;
                    }
                } else {
                    uint16_t rmVal = readModRM16(modrm);
                    switch (modrm.reg) {
                        case 0: case 1: { // TEST rm16, imm16
                            uint16_t imm = fetch16();
                            (void)(rmVal & imm);
                            break;
                        }
                        case 2: writeModRM16(modrm, ~rmVal); break; // NOT
                        case 3: writeModRM16(modrm, 0 - rmVal); break; // NEG
                        case 4: { // MUL
                            uint32_t res = static_cast<uint32_t>(m_cpu.getReg16(AX)) * rmVal;
                            m_cpu.setReg16(AX, static_cast<uint16_t>(res));
                            m_cpu.setReg16(DX, static_cast<uint16_t>(res >> 16));
                            break;
                        }
                        case 7: { // IDIV
                            if (rmVal == 0) triggerInterrupt(0);
                            else {
                                int32_t dxax = (static_cast<int16_t>(m_cpu.getReg16(DX)) << 16) | m_cpu.getReg16(AX);
                                m_cpu.setReg16(AX, static_cast<uint16_t>(dxax / static_cast<int16_t>(rmVal)));
                                m_cpu.setReg16(DX, static_cast<uint16_t>(dxax % static_cast<int16_t>(rmVal)));
                            }
                            break;
                        }
                        default: LOG_WARN("Group 3 r16 reg ", (int)modrm.reg, " not fully implemented"); break;
                    }
                }
            }
            break;
        }

        // MOVSB / MOVSW / MOVSD
        case 0xA4:
        case 0xA5: {
            uint32_t count = m_hasPrefix67 ? m_cpu.getReg32(ECX) : m_cpu.getReg16(CX);
            bool rep = m_hasRepz || m_hasRepnz;
            
            auto do_movs = [&]() {
                uint32_t si = m_hasPrefix67 ? m_cpu.getReg32(ESI) : m_cpu.getReg16(SI);
                uint32_t di = m_hasPrefix67 ? m_cpu.getReg32(EDI) : m_cpu.getReg16(DI);
                
                uint8_t dsSeg = (m_segmentOverride != 0xFF) ? m_segmentOverride : static_cast<uint8_t>(DS);
                uint32_t srcAddr = (m_cpu.getSegReg(dsSeg) << 4) + si;
                uint32_t dstAddr = (m_cpu.getSegReg(ES) << 4) + di;
                
                int df = (m_cpu.getEFLAGS() & FLAG_DIRECTION) ? -1 : 1;
                uint32_t step = 0;

                if (opcode == 0xA4) { // MOVSB
                    m_memory.write8(dstAddr, m_memory.read8(srcAddr));
                    step = 1;
                } else if (m_hasPrefix66) { // MOVSD (32-bit)
                    m_memory.write32(dstAddr, m_memory.read32(srcAddr));
                    step = 4;
                } else { // MOVSW (16-bit)
                    m_memory.write16(dstAddr, m_memory.read16(srcAddr));
                    step = 2;
                }
                
                uint32_t delta = step * df;
                if (m_hasPrefix67) {
                    m_cpu.setReg32(ESI, si + delta);
                    m_cpu.setReg32(EDI, di + delta);
                } else {
                    m_cpu.setReg16(SI, (si + delta) & 0xFFFF);
                    m_cpu.setReg16(DI, (di + delta) & 0xFFFF);
                }
            };

            if (rep) {
                while (count > 0) {
                    do_movs();
                    count--;
                }
                if (m_hasPrefix67) m_cpu.setReg32(ECX, 0);
                else m_cpu.setReg16(CX, 0);
            } else {
                do_movs();
            }
            break;
        }

        // STOSB / STOSW / STOSD
        case 0xAA:
        case 0xAB: {
            uint32_t count = m_hasPrefix67 ? m_cpu.getReg32(ECX) : m_cpu.getReg16(CX);
            bool rep = m_hasRepz || m_hasRepnz;
            
            auto do_stos = [&]() {
                uint32_t di = m_hasPrefix67 ? m_cpu.getReg32(EDI) : m_cpu.getReg16(DI);
                uint32_t dstAddr = (m_cpu.getSegReg(ES) << 4) + di;
                
                int df = (m_cpu.getEFLAGS() & FLAG_DIRECTION) ? -1 : 1;
                uint32_t step = 0;

                if (opcode == 0xAA) { // STOSB
                    m_memory.write8(dstAddr, m_cpu.getReg8(AL));
                    step = 1;
                } else if (m_hasPrefix66) { // STOSD (32-bit)
                    m_memory.write32(dstAddr, m_cpu.getReg32(EAX));
                    step = 4;
                } else { // STOSW (16-bit)
                    m_memory.write16(dstAddr, m_cpu.getReg16(AX));
                    step = 2;
                }
                
                uint32_t delta = step * df;
                if (m_hasPrefix67) {
                    m_cpu.setReg32(EDI, di + delta);
                } else {
                    m_cpu.setReg16(DI, (di + delta) & 0xFFFF);
                }
            };

            if (rep) {
                while (count > 0) {
                    do_stos();
                    count--;
                }
                if (m_hasPrefix67) m_cpu.setReg32(ECX, 0);
                else m_cpu.setReg16(CX, 0);
            } else {
                do_stos();
            }
            break;
        }

        // LODSB / LODSW / LODSD
        case 0xAC:
        case 0xAD: {
            uint32_t count = m_hasPrefix67 ? m_cpu.getReg32(ECX) : m_cpu.getReg16(CX);
            bool rep = m_hasRepz || m_hasRepnz;
            
            auto do_lods = [&]() {
                uint32_t si = m_hasPrefix67 ? m_cpu.getReg32(ESI) : m_cpu.getReg16(SI);
                uint8_t dsSeg = (m_segmentOverride != 0xFF) ? m_segmentOverride : static_cast<uint8_t>(DS);
                uint32_t srcAddr = (m_cpu.getSegReg(dsSeg) << 4) + si;
                
                int df = (m_cpu.getEFLAGS() & FLAG_DIRECTION) ? -1 : 1;
                uint32_t step = 0;

                if (opcode == 0xAC) { // LODSB
                    m_cpu.setReg8(AL, m_memory.read8(srcAddr));
                    step = 1;
                } else if (m_hasPrefix66) { // LODSD (32-bit)
                    m_cpu.setReg32(EAX, m_memory.read32(srcAddr));
                    step = 4;
                } else { // LODSW (16-bit)
                    m_cpu.setReg16(AX, m_memory.read16(srcAddr));
                    step = 2;
                }
                
                uint32_t delta = step * df;
                if (m_hasPrefix67) {
                    m_cpu.setReg32(ESI, si + delta);
                } else {
                    m_cpu.setReg16(SI, (si + delta) & 0xFFFF);
                }
            };

            if (rep) {
                while (count > 0) {
                    do_lods();
                    count--;
                }
                if (m_hasPrefix67) m_cpu.setReg32(ECX, 0);
                else m_cpu.setReg16(CX, 0);
            } else {
                do_lods();
            }
            break;
        }

        // Group 2: 0xC0, 0xC1, 0xD0, 0xD1, 0xD2, 0xD3
        case 0xC0: case 0xC1: case 0xD0: case 0xD1: case 0xD2: case 0xD3: {
            ModRM modrm = decodeModRM(fetch8());
            uint8_t count = 0;
            if (opcode == 0xC0 || opcode == 0xC1) count = fetch8();
            else if (opcode == 0xD0 || opcode == 0xD1) count = 1;
            else count = m_cpu.getReg8(CL);

            auto shift = [&](uint32_t val, uint8_t c, int size) {
                if (c == 0) return val;
                uint32_t mask = (size == 8) ? 0xFF : (size == 16) ? 0xFFFF : 0xFFFFFFFF;
                val &= mask;
                c &= 0x1F;
                switch (modrm.reg) {
                    case 4: case 6: return (val << c) & mask; // SHL / SAL
                    case 5: return (val >> c) & mask; // SHR
                    case 7: { // SAR
                        int32_t sval = (size == 8) ? static_cast<int8_t>(val) : 
                                       (size == 16) ? static_cast<int16_t>(val) : 
                                       static_cast<int32_t>(val);
                        return static_cast<uint32_t>(sval >> c) & mask;
                    }
                    default: LOG_WARN("Shift/Rotate reg ", (int)modrm.reg, " not fully handled"); return val;
                }
            };

            if (opcode == 0xC0 || opcode == 0xD0 || opcode == 0xD2) {
                uint8_t val = readModRM8(modrm);
                writeModRM8(modrm, static_cast<uint8_t>(shift(val, count, 8)));
            } else if (m_hasPrefix66) {
                uint32_t val = readModRM32(modrm);
                writeModRM32(modrm, shift(val, count, 32));
            } else {
                uint16_t val = readModRM16(modrm);
                writeModRM16(modrm, static_cast<uint16_t>(shift(val, count, 16)));
            }
            break;
        }

        case 0x87: { // XCHG r/m, reg
            ModRM modrm = decodeModRM(fetch8());
            if (m_hasPrefix66) {
                uint32_t rmVal = readModRM32(modrm);
                uint32_t regVal = m_cpu.getReg32(modrm.reg);
                writeModRM32(modrm, regVal);
                m_cpu.setReg32(modrm.reg, rmVal);
            } else {
                uint16_t rmVal = readModRM16(modrm);
                uint16_t regVal = m_cpu.getReg16(modrm.reg);
                writeModRM16(modrm, regVal);
                m_cpu.setReg16(modrm.reg, rmVal);
            }
            break;
        }

        case 0x98: { // CBW / CWDE
            if (m_hasPrefix66) {
                m_cpu.setReg32(EAX, static_cast<int32_t>(static_cast<int16_t>(m_cpu.getReg16(AX))));
            } else {
                m_cpu.setReg16(AX, static_cast<int16_t>(static_cast<int8_t>(m_cpu.getReg8(AL))));
            }
            break;
        }

        case 0x99: { // CWD / CDQ
            if (m_hasPrefix66) {
                int32_t eax = static_cast<int32_t>(m_cpu.getReg32(EAX));
                m_cpu.setReg32(EDX, (eax < 0) ? 0xFFFFFFFF : 0);
            } else {
                int16_t ax = static_cast<int16_t>(m_cpu.getReg16(AX));
                m_cpu.setReg16(DX, (ax < 0) ? 0xFFFF : 0);
            }
            break;
        }

        case 0xF9: // STC
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_CARRY);
            break;

        case 0xFC: // CLD
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~FLAG_DIRECTION);
            break;

        case 0xFD: // STD
            m_cpu.setEFLAGS(m_cpu.getEFLAGS() | FLAG_DIRECTION);
            break;

        // Group 4/5: 0xFE, 0xFF
        case 0xFE:
        case 0xFF: {
            ModRM modrm = decodeModRM(fetch8());
            if (opcode == 0xFE) { // Group 4
                if (modrm.reg == 0) writeModRM8(modrm, readModRM8(modrm) + 1); // INC rm8
                else if (modrm.reg == 1) writeModRM8(modrm, readModRM8(modrm) - 1); // DEC rm8
                break;
            }
            // Group 5
            if (m_hasPrefix66) {
                uint32_t val = (modrm.reg < 2 || modrm.reg == 6) ? readModRM32(modrm) : 0;
                switch (modrm.reg) {
                    case 0: writeModRM32(modrm, val + 1); break; // INC rm32
                    case 1: writeModRM32(modrm, val - 1); break; // DEC rm32
                    case 2: { // CALL NEAR indirect
                        uint32_t target = readModRM32(modrm);
                        m_cpu.push32(m_cpu.getEIP()); 
                        m_cpu.setEIP(target); 
                        break; 
                    }
                    case 4: { // JMP NEAR indirect
                        uint32_t target = readModRM32(modrm);
                        m_cpu.setEIP(target); 
                        break;
                    }
                    case 6: m_cpu.push32(val); break; // PUSH rm32
                }
            } else {
                uint16_t val = (modrm.reg < 2 || modrm.reg == 6) ? readModRM16(modrm) : 0;
                switch (modrm.reg) {
                    case 0: writeModRM16(modrm, val + 1); break; // INC rm16
                    case 1: writeModRM16(modrm, val - 1); break; // DEC rm16
                    case 2: { // CALL NEAR indirect
                        uint16_t target = readModRM16(modrm);
                        m_cpu.push16(static_cast<uint16_t>(m_cpu.getEIP())); 
                        m_cpu.setEIP(target); 
                        break;
                    }
                    case 4: { // JMP NEAR indirect
                        uint16_t target = readModRM16(modrm);
                        m_cpu.setEIP(target); 
                        break;
                    }
                    case 6: m_cpu.push16(val); break; // PUSH rm16
                }
            }
            break;
        }

        // FPU stub
        case 0xD8: case 0xD9: case 0xDA: case 0xDB:
        case 0xDC: case 0xDD: case 0xDE: case 0xDF: {
            ModRM modrm = decodeModRM(fetch8());
            (void)modrm;
            LOG_WARN("FPU opcode stubbed: 0x", std::hex, (int)opcode);
            break;
        }

        // IN
        case 0xE4: { // IN AL, imm8
            uint8_t port = fetch8();
            m_cpu.setReg8(AL, m_iobus.read8(port));
            break;
        }
        case 0xE5: { // IN AX/EAX, imm8
            uint8_t port = fetch8();
            if (m_hasPrefix66) {
                m_cpu.setReg32(EAX, m_iobus.read32(port));
            } else {
                m_cpu.setReg16(AX, m_iobus.read16(port));
            }
            break;
        }
        case 0xEC: { // IN AL, DX
            uint16_t port = m_cpu.getReg16(DX);
            m_cpu.setReg8(AL, m_iobus.read8(port));
            break;
        }
        case 0xED: { // IN AX/EAX, DX
            uint16_t port = m_cpu.getReg16(DX);
            if (m_hasPrefix66) {
                m_cpu.setReg32(EAX, m_iobus.read32(port));
            } else {
                m_cpu.setReg16(AX, m_iobus.read16(port));
            }
            break;
        }

        // OUT
        case 0xE6: { // OUT imm8, AL
            uint8_t port = fetch8();
            m_iobus.write8(port, m_cpu.getReg8(AL));
            break;
        }
        case 0xE7: { // OUT imm8, AX/EAX
            uint8_t port = fetch8();
            if (m_hasPrefix66) {
                m_iobus.write32(port, m_cpu.getReg32(EAX));
            } else {
                m_iobus.write16(port, m_cpu.getReg16(AX));
            }
            break;
        }
        case 0xEE: { // OUT DX, AL
            uint16_t port = m_cpu.getReg16(DX);
            m_iobus.write8(port, m_cpu.getReg8(AL));
            break;
        }
        case 0xEF: { // OUT DX, AX/EAX
            uint16_t port = m_cpu.getReg16(DX);
            if (m_hasPrefix66) {
                m_iobus.write32(port, m_cpu.getReg32(EAX));
            } else {
                m_iobus.write16(port, m_cpu.getReg16(AX));
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
                m_cpu.push32(m_cpu.getEIP());
                m_cpu.setEIP(m_cpu.getEIP() + rel32);
            } else {
                int16_t rel16 = static_cast<int16_t>(fetch16());
                m_cpu.push16(static_cast<uint16_t>(m_cpu.getEIP()));
                m_cpu.setEIP(static_cast<uint16_t>(m_cpu.getEIP() + rel16));
            }
            break;
        }
        case 0xE9: { // JMP rel16/32
            if (m_hasPrefix66) m_cpu.setEIP(m_cpu.getEIP() + fetch32());
            else m_cpu.setEIP(static_cast<uint16_t>(m_cpu.getEIP() + fetch16()));
            break;
        }
        case 0xEB: { // JMP rel8
            int8_t rel8 = static_cast<int8_t>(fetch8());
            m_cpu.setEIP(m_cpu.getEIP() + rel8);
            break;
        }

        // Immediate moves
        case 0xB0: case 0xB1: case 0xB2: case 0xB3:
        case 0xB4: case 0xB5: case 0xB6: case 0xB7: {
            m_cpu.setReg8(opcode & 0x07, fetch8());
            break;
        }
        case 0xB8: case 0xB9: case 0xBA: case 0xBB:
        case 0xBC: case 0xBD: case 0xBE: case 0xBF: {
            uint8_t reg = opcode & 0x07;
            if (m_hasPrefix66) m_cpu.setReg32(reg, fetch32());
            else m_cpu.setReg16(reg, fetch16());
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
    if (m_dos.handleInterrupt(vector)) {
        LOG_DEBUG("DOS: Handled Interrupt 0x", std::hex, static_cast<int>(vector), " via HLE");
        return;
    }
    if (m_bios.handleInterrupt(vector)) {
        LOG_DEBUG("BIOS: Handled Interrupt 0x", std::hex, static_cast<int>(vector), " via HLE");
        return;
    }

    if (m_hasPrefix66) {
        LOG_WARN("32-bit interrupts not fully implemented");
    }

    m_cpu.push16(static_cast<uint16_t>(m_cpu.getEFLAGS() & 0xFFFF));
    m_cpu.push16(m_cpu.getSegReg(CS));
    m_cpu.push16(static_cast<uint16_t>(m_cpu.getEIP() & 0xFFFF));

    uint16_t newIP = m_memory.read16(vector * 4);
    uint16_t newCS = m_memory.read16((vector * 4) + 2);
    
    m_cpu.setSegReg(CS, newCS);
    m_cpu.setEIP(newIP);

    LOG_DEBUG("Triggered Interrupt 0x", std::hex, static_cast<int>(vector), 
              " Vector -> CS:EIP = 0x", newCS, ":0x", newIP);
}

} // namespace fador::cpu
