#include "Disassembler.hpp"
#include <sstream>
#include <iomanip>
#include <cstring>

namespace fador::cpu {

// ── Cursor ──────────────────────────────────────────────────────────────────

uint8_t Disassembler::Cursor::read8() { return mem.read8(pos++); }
uint16_t Disassembler::Cursor::read16() { uint16_t v = mem.read16(pos); pos += 2; return v; }
uint32_t Disassembler::Cursor::read32() { uint32_t v = mem.read32(pos); pos += 4; return v; }

// ── Helpers ─────────────────────────────────────────────────────────────────

const char* Disassembler::reg8Name(uint8_t idx) {
    static const char* names[] = {"AL","CL","DL","BL","AH","CH","DH","BH"};
    return names[idx & 7];
}
const char* Disassembler::reg16Name(uint8_t idx) {
    static const char* names[] = {"AX","CX","DX","BX","SP","BP","SI","DI"};
    return names[idx & 7];
}
const char* Disassembler::reg32Name(uint8_t idx) {
    static const char* names[] = {"EAX","ECX","EDX","EBX","ESP","EBP","ESI","EDI"};
    return names[idx & 7];
}
const char* Disassembler::segRegName(uint8_t idx) {
    static const char* names[] = {"ES","CS","SS","DS","FS","GS"};
    return (idx < 6) ? names[idx] : "??";
}
const char* Disassembler::ccName(uint8_t cc) {
    static const char* names[] = {
        "O","NO","B","NB","Z","NZ","BE","A",
        "S","NS","P","NP","L","GE","LE","G"
    };
    return names[cc & 0xF];
}

Disassembler::ModRM Disassembler::decodeModRM(uint8_t b) {
    return { static_cast<uint8_t>((b >> 6) & 3),
             static_cast<uint8_t>((b >> 3) & 7),
             static_cast<uint8_t>(b & 7) };
}
Disassembler::SIB Disassembler::decodeSIB(uint8_t b) {
    return { static_cast<uint8_t>((b >> 6) & 3),
             static_cast<uint8_t>((b >> 3) & 7),
             static_cast<uint8_t>(b & 7) };
}

std::string Disassembler::hexStr8(uint8_t v) {
    std::ostringstream s; s << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << (int)v << 'h'; return s.str();
}
std::string Disassembler::hexStr16(uint16_t v) {
    std::ostringstream s; s << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << v << 'h'; return s.str();
}
std::string Disassembler::hexStr32(uint32_t v) {
    std::ostringstream s; s << std::uppercase << std::hex << std::setw(8) << std::setfill('0') << v << 'h'; return s.str();
}

// ── ModR/M formatting ──────────────────────────────────────────────────────

std::string Disassembler::formatModRM16(Cursor& c, const ModRM& m, int opSize, uint8_t segOvr) const {
    if (m.mod == 3) {
        if (opSize == 8)  return reg8Name(m.rm);
        if (opSize == 32) return reg32Name(m.rm);
        return reg16Name(m.rm);
    }

    std::string prefix;
    if (segOvr != 0xFF) { prefix = std::string(segRegName(segOvr)) + ":"; }

    std::string sizeTag;
    if (opSize == 8) sizeTag = "BYTE PTR ";
    else if (opSize == 32) sizeTag = "DWORD PTR ";
    else sizeTag = "WORD PTR ";

    std::string base;
    if (m.mod == 0 && m.rm == 6) {
        uint16_t disp = c.read16();
        return sizeTag + "[" + prefix + hexStr16(disp) + "]";
    }

    static const char* bases[] = {"BX+SI","BX+DI","BP+SI","BP+DI","SI","DI","BP","BX"};
    base = bases[m.rm];

    if (m.mod == 0) return sizeTag + "[" + prefix + base + "]";
    if (m.mod == 1) {
        int8_t d = static_cast<int8_t>(c.read8());
        if (d >= 0) return sizeTag + "[" + prefix + base + "+" + hexStr8(static_cast<uint8_t>(d)) + "]";
        return sizeTag + "[" + prefix + base + "-" + hexStr8(static_cast<uint8_t>(-d)) + "]";
    }
    // mod == 2
    uint16_t d = c.read16();
    return sizeTag + "[" + prefix + base + "+" + hexStr16(d) + "]";
}

std::string Disassembler::formatModRM32(Cursor& c, const ModRM& m, int opSize, uint8_t segOvr) const {
    if (m.mod == 3) {
        if (opSize == 8)  return reg8Name(m.rm);
        if (opSize == 32) return reg32Name(m.rm);
        return reg16Name(m.rm);
    }

    std::string prefix;
    if (segOvr != 0xFF) { prefix = std::string(segRegName(segOvr)) + ":"; }

    std::string sizeTag;
    if (opSize == 8) sizeTag = "BYTE PTR ";
    else if (opSize == 32) sizeTag = "DWORD PTR ";
    else sizeTag = "WORD PTR ";

    auto buildSIB = [&]() -> std::string {
        SIB sib = decodeSIB(c.read8());
        std::string idx;
        if (sib.index != 4) {
            idx = reg32Name(sib.index);
            if (sib.scale > 0) idx += "*" + std::to_string(1 << sib.scale);
        }
        std::string b;
        if (sib.base == 5 && m.mod == 0) {
            uint32_t d = c.read32();
            b = hexStr32(d);
        } else {
            b = reg32Name(sib.base);
        }
        if (idx.empty()) return b;
        return b + "+" + idx;
    };

    if (m.mod == 0 && m.rm == 5) {
        uint32_t d = c.read32();
        return sizeTag + "[" + prefix + hexStr32(d) + "]";
    }

    std::string base;
    if (m.rm == 4) {
        base = buildSIB();
    } else {
        base = reg32Name(m.rm);
    }

    if (m.mod == 0) return sizeTag + "[" + prefix + base + "]";
    if (m.mod == 1) {
        int8_t d = static_cast<int8_t>(c.read8());
        if (d >= 0) return sizeTag + "[" + prefix + base + "+" + hexStr8(static_cast<uint8_t>(d)) + "]";
        return sizeTag + "[" + prefix + base + "-" + hexStr8(static_cast<uint8_t>(-d)) + "]";
    }
    // mod == 2
    uint32_t d = c.read32();
    return sizeTag + "[" + prefix + base + "+" + hexStr32(d) + "]";
}

std::string Disassembler::formatRM(Cursor& c, const ModRM& m, int opSize, bool addr32, uint8_t segOvr) const {
    if (addr32) return formatModRM32(c, m, opSize, segOvr);
    return formatModRM16(c, m, opSize, segOvr);
}

// ── Constructor ─────────────────────────────────────────────────────────────

Disassembler::Disassembler(const memory::MemoryBus& memory) : m_memory(memory) {}

// ── Public API ──────────────────────────────────────────────────────────────

DisassembledInstruction Disassembler::disassembleAt(uint32_t linearAddr) const {
    Cursor c{m_memory, linearAddr, linearAddr};

    // Decode prefixes
    bool p66 = false, p67 = false, repz = false, repnz = false;
    uint8_t segOvr = 0xFF;
    std::string prefixStr;

    for (;;) {
        uint8_t b = c.read8();
        switch (b) {
            case 0x66: p66 = true; continue;
            case 0x67: p67 = true; continue;
            case 0xF0: prefixStr += "LOCK "; continue;
            case 0xF2: repnz = true; continue;
            case 0xF3: repz = true; continue;
            case 0x26: segOvr = 0; continue; // ES
            case 0x2E: segOvr = 1; continue; // CS
            case 0x36: segOvr = 2; continue; // SS
            case 0x3E: segOvr = 3; continue; // DS
            case 0x64: segOvr = 4; continue; // FS
            case 0x65: segOvr = 5; continue; // GS
            default:
                // Not a prefix; this is the opcode
                {
                    std::string mnem = disasmOpcode(c, b, p66, p67, repz, repnz, segOvr);
                    if (repz && mnem.find("REP") == std::string::npos) prefixStr += "REPZ ";
                    if (repnz && mnem.find("REP") == std::string::npos) prefixStr += "REPNZ ";
                    mnem = prefixStr + mnem;

                    // Build hex bytes string
                    uint32_t len = c.bytesConsumed();
                    std::ostringstream hex;
                    for (uint32_t i = 0; i < len; ++i) {
                        if (i) hex << ' ';
                        hex << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
                            << (int)m_memory.read8(linearAddr + i);
                    }

                    return {linearAddr, len, mnem, hex.str()};
                }
        }
    }
}

std::vector<DisassembledInstruction> Disassembler::disassembleRange(uint32_t linearAddr, uint32_t count) const {
    std::vector<DisassembledInstruction> result;
    result.reserve(count);
    uint32_t addr = linearAddr;
    for (uint32_t i = 0; i < count; ++i) {
        auto instr = disassembleAt(addr);
        addr += instr.length;
        result.push_back(std::move(instr));
    }
    return result;
}

std::vector<DisassembledInstruction> Disassembler::disassembleAround(uint32_t centerAddr,
                                                                       uint32_t before,
                                                                       uint32_t after) const {
    // For instructions before the center we scan back heuristically:
    // try to start from 64 bytes before and disassemble forward until we
    // reach the centre, then keep the last `before` instructions.
    std::vector<DisassembledInstruction> prolog;
    if (before > 0) {
        // Start scanning from up to 128 bytes before center
        uint32_t scanStart = (centerAddr > 128) ? centerAddr - 128 : 0;
        uint32_t addr = scanStart;
        while (addr < centerAddr) {
            auto instr = disassembleAt(addr);
            if (addr + instr.length > centerAddr) break;
            prolog.push_back(std::move(instr));
            addr += prolog.back().length;
        }
        // Keep only the last `before` entries
        if (prolog.size() > before) {
            prolog.erase(prolog.begin(), prolog.end() - before);
        }
    }

    // Disassemble from center forward
    auto forward = disassembleRange(centerAddr, after);

    // Combine
    prolog.insert(prolog.end(), forward.begin(), forward.end());
    return prolog;
}

// ── ALU / Group mnemonics ───────────────────────────────────────────────────

static const char* aluNames[] = {"ADD","OR","ADC","SBB","AND","SUB","XOR","CMP"};
static const char* shiftNames[] = {"ROL","ROR","RCL","RCR","SHL","SHR","SAL","SAR"};
static const char* grp3Names[] = {"TEST","TEST","NOT","NEG","MUL","IMUL","DIV","IDIV"};
static const char* grp45Names_16[] = {"INC","DEC","CALL","CALL FAR","JMP","JMP FAR","PUSH","???"};

// ── Single-byte opcode disassembly ──────────────────────────────────────────

std::string Disassembler::disasmOpcode(Cursor& c, uint8_t op, bool p66, bool p67,
                                        bool repz, bool repnz, uint8_t segOvr) const {
    // Determine default operand size
    int opSize16 = p66 ? 32 : 16;
    auto regW = [&](uint8_t idx) -> const char* { return p66 ? reg32Name(idx) : reg16Name(idx); };
    auto immW = [&](Cursor& cc) -> std::string { return p66 ? hexStr32(cc.read32()) : hexStr16(cc.read16()); };

    // Two-byte escape
    if (op == 0x0F) {
        uint8_t op2 = c.read8();
        return disasmOpcode0F(c, op2, p66, p67, segOvr);
    }

    // ── ALU r/m8, r8 (00,08,10,18,20,28,30,38) ──
    if ((op & 0xC7) == 0x00) {
        uint8_t aluIdx = (op >> 3) & 7;
        ModRM m = decodeModRM(c.read8());
        return std::string(aluNames[aluIdx]) + " " + formatRM(c, m, 8, p67, segOvr) + ", " + reg8Name(m.reg);
    }
    // ALU r/m16, r16 (01,09,11,19,21,29,31,39)
    if ((op & 0xC7) == 0x01) {
        uint8_t aluIdx = (op >> 3) & 7;
        ModRM m = decodeModRM(c.read8());
        return std::string(aluNames[aluIdx]) + " " + formatRM(c, m, opSize16, p67, segOvr) + ", " + regW(m.reg);
    }
    // ALU r8, r/m8 (02,0A,12,1A,22,2A,32,3A)
    if ((op & 0xC7) == 0x02) {
        uint8_t aluIdx = (op >> 3) & 7;
        ModRM m = decodeModRM(c.read8());
        return std::string(aluNames[aluIdx]) + " " + reg8Name(m.reg) + ", " + formatRM(c, m, 8, p67, segOvr);
    }
    // ALU r16, r/m16 (03,0B,13,1B,23,2B,33,3B)
    if ((op & 0xC7) == 0x03) {
        uint8_t aluIdx = (op >> 3) & 7;
        ModRM m = decodeModRM(c.read8());
        return std::string(aluNames[aluIdx]) + " " + regW(m.reg) + ", " + formatRM(c, m, opSize16, p67, segOvr);
    }
    // ALU AL, imm8 (04,0C,14,1C,24,2C,34,3C)
    if ((op & 0xC7) == 0x04) {
        uint8_t aluIdx = (op >> 3) & 7;
        return std::string(aluNames[aluIdx]) + " AL, " + hexStr8(c.read8());
    }
    // ALU AX, imm16 (05,0D,15,1D,25,2D,35,3D)
    if ((op & 0xC7) == 0x05) {
        uint8_t aluIdx = (op >> 3) & 7;
        return std::string(aluNames[aluIdx]) + " " + regW(0) + ", " + immW(c);
    }

    // ── Segment PUSH/POP ──
    if (op == 0x06) return "PUSH ES";
    if (op == 0x07) return "POP ES";
    if (op == 0x0E) return "PUSH CS";
    if (op == 0x16) return "PUSH SS";
    if (op == 0x17) return "POP SS";
    if (op == 0x1E) return "PUSH DS";
    if (op == 0x1F) return "POP DS";

    // ── BCD/ASCII adjust ──
    if (op == 0x27) return "DAA";
    if (op == 0x2F) return "DAS";
    if (op == 0x37) return "AAA";
    if (op == 0x3F) return "AAS";
    if (op == 0xD4) { c.read8(); return "AAM"; }
    if (op == 0xD5) { c.read8(); return "AAD"; }

    // ── INC/DEC reg16 ──
    if (op >= 0x40 && op <= 0x47) return std::string("INC ") + regW(op & 7);
    if (op >= 0x48 && op <= 0x4F) return std::string("DEC ") + regW(op & 7);

    // ── PUSH/POP reg16 ──
    if (op >= 0x50 && op <= 0x57) return std::string("PUSH ") + regW(op & 7);
    if (op >= 0x58 && op <= 0x5F) return std::string("POP ") + regW(op & 7);

    // ── PUSHA/POPA ──
    if (op == 0x60) return p66 ? "PUSHAD" : "PUSHA";
    if (op == 0x61) return p66 ? "POPAD" : "POPA";

    // ── BOUND / ARPL stubs ──
    if (op == 0x62) { ModRM m = decodeModRM(c.read8()); (void)formatRM(c, m, opSize16, p67, segOvr); return "BOUND"; }
    if (op == 0x63) { ModRM m = decodeModRM(c.read8()); (void)formatRM(c, m, 16, p67, segOvr); return "ARPL"; }

    // ── IMUL r, r/m, imm ──
    if (op == 0x69) {
        ModRM m = decodeModRM(c.read8());
        std::string rm = formatRM(c, m, opSize16, p67, segOvr);
        return "IMUL " + std::string(regW(m.reg)) + ", " + rm + ", " + immW(c);
    }
    if (op == 0x6B) {
        ModRM m = decodeModRM(c.read8());
        std::string rm = formatRM(c, m, opSize16, p67, segOvr);
        return "IMUL " + std::string(regW(m.reg)) + ", " + rm + ", " + hexStr8(c.read8());
    }

    // ── PUSH imm ──
    if (op == 0x68) return "PUSH " + immW(c);
    if (op == 0x6A) return "PUSH " + hexStr8(c.read8());

    // ── INS/OUTS stubs ──
    if (op == 0x6C) return "INSB";
    if (op == 0x6D) return p66 ? "INSD" : "INSW";
    if (op == 0x6E) return "OUTSB";
    if (op == 0x6F) return p66 ? "OUTSD" : "OUTSW";

    // ── Jcc short ──
    if (op >= 0x70 && op <= 0x7F) {
        int8_t d = static_cast<int8_t>(c.read8());
        uint32_t target = c.pos + d; // c.pos is already past the displacement
        return std::string("J") + ccName(op & 0xF) + " " + hexStr16(static_cast<uint16_t>(target));
    }

    // ── Group 1 (80-83) ──
    if (op >= 0x80 && op <= 0x83) {
        ModRM m = decodeModRM(c.read8());
        int sz = (op == 0x80 || op == 0x82) ? 8 : opSize16;
        std::string rm = formatRM(c, m, sz, p67, segOvr);
        std::string imm;
        if (op == 0x80 || op == 0x82) imm = hexStr8(c.read8());
        else if (op == 0x83) imm = hexStr8(c.read8());
        else imm = (p66 ? hexStr32(c.read32()) : hexStr16(c.read16()));
        return std::string(aluNames[m.reg]) + " " + rm + ", " + imm;
    }

    // ── TEST ──
    if (op == 0x84) { ModRM m = decodeModRM(c.read8()); return "TEST " + formatRM(c, m, 8, p67, segOvr) + ", " + reg8Name(m.reg); }
    if (op == 0x85) { ModRM m = decodeModRM(c.read8()); return "TEST " + formatRM(c, m, opSize16, p67, segOvr) + ", " + regW(m.reg); }

    // ── XCHG ──
    if (op == 0x86) { ModRM m = decodeModRM(c.read8()); return "XCHG " + formatRM(c, m, 8, p67, segOvr) + ", " + reg8Name(m.reg); }
    if (op == 0x87) { ModRM m = decodeModRM(c.read8()); return "XCHG " + formatRM(c, m, opSize16, p67, segOvr) + ", " + regW(m.reg); }

    // ── MOV r/m, r | r, r/m ──
    if (op == 0x88) { ModRM m = decodeModRM(c.read8()); return "MOV " + formatRM(c, m, 8, p67, segOvr) + ", " + reg8Name(m.reg); }
    if (op == 0x89) { ModRM m = decodeModRM(c.read8()); return "MOV " + formatRM(c, m, opSize16, p67, segOvr) + ", " + regW(m.reg); }
    if (op == 0x8A) { ModRM m = decodeModRM(c.read8()); return "MOV " + std::string(reg8Name(m.reg)) + ", " + formatRM(c, m, 8, p67, segOvr); }
    if (op == 0x8B) { ModRM m = decodeModRM(c.read8()); return "MOV " + std::string(regW(m.reg)) + ", " + formatRM(c, m, opSize16, p67, segOvr); }
    if (op == 0x8C) { ModRM m = decodeModRM(c.read8()); return "MOV " + formatRM(c, m, 16, p67, segOvr) + ", " + segRegName(m.reg); }
    if (op == 0x8E) { ModRM m = decodeModRM(c.read8()); return "MOV " + std::string(segRegName(m.reg)) + ", " + formatRM(c, m, 16, p67, segOvr); }

    // ── LEA ──
    if (op == 0x8D) {
        ModRM m = decodeModRM(c.read8());
        return "LEA " + std::string(regW(m.reg)) + ", " + formatRM(c, m, opSize16, p67, segOvr);
    }

    // ── POP r/m ──
    if (op == 0x8F) {
        ModRM m = decodeModRM(c.read8());
        return "POP " + formatRM(c, m, opSize16, p67, segOvr);
    }

    // ── NOP / XCHG AX, reg ──
    if (op == 0x90) return "NOP";
    if (op >= 0x91 && op <= 0x97) return "XCHG " + std::string(regW(0)) + ", " + regW(op & 7);

    // ── CBW/CWD etc ──
    if (op == 0x98) return p66 ? "CWDE" : "CBW";
    if (op == 0x99) return p66 ? "CDQ" : "CWD";
    if (op == 0x9A) { uint16_t ip = c.read16(); uint16_t cs = c.read16(); return "CALL " + hexStr16(cs) + ":" + hexStr16(ip); }
    if (op == 0x9B) return "FWAIT";
    if (op == 0x9C) return p66 ? "PUSHFD" : "PUSHF";
    if (op == 0x9D) return p66 ? "POPFD" : "POPF";
    if (op == 0x9E) return "SAHF";
    if (op == 0x9F) return "LAHF";

    // ── MOV AL/AX, moffs ──
    if (op == 0xA0) return "MOV AL, [" + hexStr16(c.read16()) + "]";
    if (op == 0xA1) return "MOV " + std::string(regW(0)) + ", [" + hexStr16(c.read16()) + "]";
    if (op == 0xA2) return "MOV [" + hexStr16(c.read16()) + "], AL";
    if (op == 0xA3) return "MOV [" + hexStr16(c.read16()) + "], " + regW(0);

    // ── String ops (with REP prefix handling) ──
    if (op == 0xA4) { return repz ? "REP MOVSB" : "MOVSB"; }
    if (op == 0xA5) { return repz ? (p66 ? "REP MOVSD" : "REP MOVSW") : (p66 ? "MOVSD" : "MOVSW"); }
    if (op == 0xA6) { return (repz ? "REPE CMPSB" : (repnz ? "REPNE CMPSB" : "CMPSB")); }
    if (op == 0xA7) { return (repz ? (p66 ? "REPE CMPSD" : "REPE CMPSW") : (repnz ? (p66 ? "REPNE CMPSD" : "REPNE CMPSW") : (p66 ? "CMPSD" : "CMPSW"))); }
    if (op == 0xA8) return "TEST AL, " + hexStr8(c.read8());
    if (op == 0xA9) return "TEST " + std::string(regW(0)) + ", " + immW(c);
    if (op == 0xAA) { return repz ? "REP STOSB" : "STOSB"; }
    if (op == 0xAB) { return repz ? (p66 ? "REP STOSD" : "REP STOSW") : (p66 ? "STOSD" : "STOSW"); }
    if (op == 0xAC) { return repz ? "REP LODSB" : "LODSB"; }
    if (op == 0xAD) { return repz ? (p66 ? "REP LODSD" : "REP LODSW") : (p66 ? "LODSD" : "LODSW"); }
    if (op == 0xAE) { return (repz ? "REPE SCASB" : (repnz ? "REPNE SCASB" : "SCASB")); }
    if (op == 0xAF) { return (repz ? (p66 ? "REPE SCASD" : "REPE SCASW") : (repnz ? (p66 ? "REPNE SCASD" : "REPNE SCASW") : (p66 ? "SCASD" : "SCASW"))); }

    // ── MOV reg, imm ──
    if (op >= 0xB0 && op <= 0xB7) return "MOV " + std::string(reg8Name(op & 7)) + ", " + hexStr8(c.read8());
    if (op >= 0xB8 && op <= 0xBF) return "MOV " + std::string(regW(op & 7)) + ", " + immW(c);

    // ── Shift/Rotate ──
    if (op == 0xC0 || op == 0xC1 || op == 0xD0 || op == 0xD1 || op == 0xD2 || op == 0xD3) {
        ModRM m = decodeModRM(c.read8());
        int sz = (op == 0xC0 || op == 0xD0 || op == 0xD2) ? 8 : opSize16;
        std::string rm = formatRM(c, m, sz, p67, segOvr);
        std::string count;
        if (op == 0xC0 || op == 0xC1) count = hexStr8(c.read8());
        else if (op == 0xD0 || op == 0xD1) count = "1";
        else count = "CL";
        return std::string(shiftNames[m.reg]) + " " + rm + ", " + count;
    }

    // ── RET ──
    if (op == 0xC2) return "RET " + hexStr16(c.read16());
    if (op == 0xC3) return "RET";
    if (op == 0xCA) return "RETF " + hexStr16(c.read16());
    if (op == 0xCB) return "RETF";

    // ── LES/LDS ──
    if (op == 0xC4) { ModRM m = decodeModRM(c.read8()); return "LES " + std::string(regW(m.reg)) + ", " + formatRM(c, m, opSize16, p67, segOvr); }
    if (op == 0xC5) { ModRM m = decodeModRM(c.read8()); return "LDS " + std::string(regW(m.reg)) + ", " + formatRM(c, m, opSize16, p67, segOvr); }

    // ── MOV r/m, imm ──
    if (op == 0xC6) { ModRM m = decodeModRM(c.read8()); std::string rm = formatRM(c, m, 8, p67, segOvr); return "MOV " + rm + ", " + hexStr8(c.read8()); }
    if (op == 0xC7) { ModRM m = decodeModRM(c.read8()); std::string rm = formatRM(c, m, opSize16, p67, segOvr); return "MOV " + rm + ", " + immW(c); }

    // ── ENTER/LEAVE ──
    if (op == 0xC8) { uint16_t sz = c.read16(); uint8_t nest = c.read8(); return "ENTER " + hexStr16(sz) + ", " + hexStr8(nest); }
    if (op == 0xC9) return "LEAVE";

    // ── INT ──
    if (op == 0xCC) return "INT3";
    if (op == 0xCD) return "INT " + hexStr8(c.read8());
    if (op == 0xCE) return "INTO";
    if (op == 0xCF) return "IRET";

    // ── XLAT ──
    if (op == 0xD7) return "XLAT";

    // ── FPU partial decoding ──
    if (op >= 0xD8 && op <= 0xDF) {
        ModRM m = decodeModRM(c.read8());

        auto formatMemNoSize = [&](const ModRM& modrm) {
            std::string rm = formatRM(c, modrm, p67 ? 32 : 16, p67, segOvr);
            static const char* sizeTags[] = {"BYTE PTR ", "WORD PTR ", "DWORD PTR "};
            for (const char* sizeTag : sizeTags) {
                if (rm.rfind(sizeTag, 0) == 0) {
                    rm.erase(0, std::strlen(sizeTag));
                    break;
                }
            }
            return rm;
        };

        if (m.mod != 3) {
            if (op == 0xD9) {
                if (m.reg == 4) return "FLDENV " + formatMemNoSize(m);
                if (m.reg == 6) return "FNSTENV " + formatMemNoSize(m);
            }
            if (op == 0xDD) {
                if (m.reg == 4) return "FRSTOR " + formatMemNoSize(m);
                if (m.reg == 6) return "FNSAVE " + formatMemNoSize(m);
            }
        }

        (void)formatRM(c, m, 32, p67, segOvr); // consume any displacement
        std::ostringstream s; s << "FPU_" << std::hex << (int)op;
        return s.str();
    }

    // ── LOOP/JCXZ ──
    if (op >= 0xE0 && op <= 0xE3) {
        static const char* loopNames[] = {"LOOPNE","LOOPE","LOOP","JCXZ"};
        int8_t d = static_cast<int8_t>(c.read8());
        uint32_t target = c.pos + d;
        return std::string(loopNames[op - 0xE0]) + " " + hexStr16(static_cast<uint16_t>(target));
    }

    // ── I/O ──
    if (op == 0xE4) return "IN AL, " + hexStr8(c.read8());
    if (op == 0xE5) return "IN " + std::string(regW(0)) + ", " + hexStr8(c.read8());
    if (op == 0xE6) return "OUT " + hexStr8(c.read8()) + ", AL";
    if (op == 0xE7) return "OUT " + hexStr8(c.read8()) + ", " + regW(0);
    if (op == 0xEC) return "IN AL, DX";
    if (op == 0xED) return std::string("IN ") + regW(0) + ", DX";
    if (op == 0xEE) return "OUT DX, AL";
    if (op == 0xEF) return std::string("OUT DX, ") + regW(0);

    // ── CALL/JMP ──
    if (op == 0xE8) {
        if (p66) { int32_t rel = static_cast<int32_t>(c.read32()); return "CALL " + hexStr32(c.pos + rel); }
        int16_t rel = static_cast<int16_t>(c.read16());
        return "CALL " + hexStr16(static_cast<uint16_t>(c.pos + rel));
    }
    if (op == 0xE9) {
        if (p66) { int32_t rel = static_cast<int32_t>(c.read32()); return "JMP " + hexStr32(c.pos + rel); }
        int16_t rel = static_cast<int16_t>(c.read16());
        return "JMP " + hexStr16(static_cast<uint16_t>(c.pos + rel));
    }
    if (op == 0xEA) { uint16_t ip = c.read16(); uint16_t cs = c.read16(); return "JMP " + hexStr16(cs) + ":" + hexStr16(ip); }
    if (op == 0xEB) {
        int8_t d = static_cast<int8_t>(c.read8());
        return "JMP SHORT " + hexStr16(static_cast<uint16_t>(c.pos + d));
    }

    // ── Flag ops ──
    if (op == 0xF5) return "CMC";
    if (op == 0xF8) return "CLC";
    if (op == 0xF9) return "STC";
    if (op == 0xFA) return "CLI";
    if (op == 0xFB) return "STI";
    if (op == 0xFC) return "CLD";
    if (op == 0xFD) return "STD";

    // ── Group 3 (F6/F7): TEST/NOT/NEG/MUL/IMUL/DIV/IDIV ──
    if (op == 0xF6 || op == 0xF7) {
        int sz = (op == 0xF6) ? 8 : opSize16;
        ModRM m = decodeModRM(c.read8());
        std::string rm = formatRM(c, m, sz, p67, segOvr);
        if (m.reg == 0 || m.reg == 1) {
            // TEST r/m, imm
            std::string imm = (op == 0xF6) ? hexStr8(c.read8()) : immW(c);
            return "TEST " + rm + ", " + imm;
        }
        return std::string(grp3Names[m.reg]) + " " + rm;
    }

    // ── Group 4/5 (FE/FF) ──
    if (op == 0xFE) {
        ModRM m = decodeModRM(c.read8());
        std::string rm = formatRM(c, m, 8, p67, segOvr);
        if (m.reg == 0) return "INC " + rm;
        if (m.reg == 1) return "DEC " + rm;
        return "??? " + rm;
    }
    if (op == 0xFF) {
        ModRM m = decodeModRM(c.read8());
        std::string rm = formatRM(c, m, opSize16, p67, segOvr);
        return std::string(grp45Names_16[m.reg]) + " " + rm;
    }

    // ── Misc ──
    if (op == 0xD6) return "SALC";
    if (op == 0xF1) return "INT1";
    if (op == 0xF4) return "HLT";

    // Unknown
    std::ostringstream s;
    s << "DB " << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << (int)op << 'h';
    return s.str();
}

// ── Two-byte (0F xx) opcode disassembly ─────────────────────────────────────

std::string Disassembler::disasmOpcode0F(Cursor& c, uint8_t op, bool p66, bool p67,
                                          uint8_t segOvr) const {
    int opSize16 = p66 ? 32 : 16;
    auto regW = [&](uint8_t idx) -> const char* { return p66 ? reg32Name(idx) : reg16Name(idx); };

    // ── Group 6/7 ──
    if (op == 0x00) {
        ModRM m = decodeModRM(c.read8());
        std::string rm = formatRM(c, m, 16, p67, segOvr);
        if (m.reg == 0) return "SLDT " + rm;
        if (m.reg == 1) return "STR " + rm;
        return "GRP6/" + std::to_string(m.reg) + " " + rm;
    }
    if (op == 0x01) {
        ModRM m = decodeModRM(c.read8());
        std::string rm = formatRM(c, m, 16, p67, segOvr);
        if (m.reg == 4) return "SMSW " + rm;
        return "GRP7/" + std::to_string(m.reg) + " " + rm;
    }
    if (op == 0x02) { ModRM m = decodeModRM(c.read8()); return "LAR " + std::string(regW(m.reg)) + ", " + formatRM(c, m, 16, p67, segOvr); }
    if (op == 0x03) { ModRM m = decodeModRM(c.read8()); return "LSL " + std::string(regW(m.reg)) + ", " + formatRM(c, m, 16, p67, segOvr); }
    if (op == 0x06) return "CLTS";
    if (op == 0x08) return "INVD";
    if (op == 0x09) return "WBINVD";
    if (op == 0x0B) return "UD2";

    // ── SSE stubs ──
    if (op == 0x10 || op == 0x11) { c.read8(); return "MOVUPS (SSE stub)"; }
    if (op == 0x35) return "SYSEXIT";
    if (op == 0x77) return "EMMS";
    if (op == 0xEE) return "FEMMS";

    // ── Jcc near ──
    if (op >= 0x80 && op <= 0x8F) {
        uint32_t rel = p66 ? c.read32() : c.read16();
        uint32_t target;
        if (p66) target = c.pos + rel;
        else target = static_cast<uint16_t>(c.pos + rel);
        return std::string("J") + ccName(op & 0xF) + " " + (p66 ? hexStr32(target) : hexStr16(static_cast<uint16_t>(target)));
    }

    // ── SETcc ──
    if (op >= 0x90 && op <= 0x9F) {
        ModRM m = decodeModRM(c.read8());
        return std::string("SET") + ccName(op & 0xF) + " " + formatRM(c, m, 8, p67, segOvr);
    }

    // ── CMOVcc ──
    if (op >= 0x40 && op <= 0x4F) {
        ModRM m = decodeModRM(c.read8());
        return std::string("CMOV") + ccName(op & 0xF) + " " + regW(m.reg) + ", " + formatRM(c, m, opSize16, p67, segOvr);
    }

    // ── PUSH/POP FS/GS ──
    if (op == 0xA0) return "PUSH FS";
    if (op == 0xA1) return "POP FS";
    if (op == 0xA8) return "PUSH GS";
    if (op == 0xA9) return "POP GS";

    // ── BT/BTS/BTR/BTC r/m, r ──
    if (op == 0xA3) { ModRM m = decodeModRM(c.read8()); return "BT " + formatRM(c, m, opSize16, p67, segOvr) + ", " + regW(m.reg); }
    if (op == 0xAB) { ModRM m = decodeModRM(c.read8()); return "BTS " + formatRM(c, m, opSize16, p67, segOvr) + ", " + regW(m.reg); }
    if (op == 0xB3) { ModRM m = decodeModRM(c.read8()); return "BTR " + formatRM(c, m, opSize16, p67, segOvr) + ", " + regW(m.reg); }
    if (op == 0xBB) { ModRM m = decodeModRM(c.read8()); return "BTC " + formatRM(c, m, opSize16, p67, segOvr) + ", " + regW(m.reg); }

    // ── Group 8 (BA) ──
    if (op == 0xBA) {
        ModRM m = decodeModRM(c.read8());
        std::string rm = formatRM(c, m, opSize16, p67, segOvr);
        uint8_t imm = c.read8();
        static const char* names[] = {"???","???","???","???","BT","BTS","BTR","BTC"};
        return std::string(names[m.reg]) + " " + rm + ", " + hexStr8(imm);
    }

    // ── SHLD/SHRD ──
    if (op == 0xA4) { ModRM m = decodeModRM(c.read8()); return "SHLD " + formatRM(c, m, opSize16, p67, segOvr) + ", " + regW(m.reg) + ", " + hexStr8(c.read8()); }
    if (op == 0xA5) { ModRM m = decodeModRM(c.read8()); return "SHLD " + formatRM(c, m, opSize16, p67, segOvr) + ", " + regW(m.reg) + ", CL"; }
    if (op == 0xAC) { ModRM m = decodeModRM(c.read8()); return "SHRD " + formatRM(c, m, opSize16, p67, segOvr) + ", " + regW(m.reg) + ", " + hexStr8(c.read8()); }
    if (op == 0xAD) { ModRM m = decodeModRM(c.read8()); return "SHRD " + formatRM(c, m, opSize16, p67, segOvr) + ", " + regW(m.reg) + ", CL"; }

    // ── BSF/BSR ──
    if (op == 0xBC) { ModRM m = decodeModRM(c.read8()); return "BSF " + std::string(regW(m.reg)) + ", " + formatRM(c, m, opSize16, p67, segOvr); }
    if (op == 0xBD) { ModRM m = decodeModRM(c.read8()); return "BSR " + std::string(regW(m.reg)) + ", " + formatRM(c, m, opSize16, p67, segOvr); }

    // ── BSWAP ──
    if (op >= 0xC8 && op <= 0xCF) {
        return "BSWAP " + std::string(reg32Name(op & 0x7));
    }

    // ── LSS/LFS/LGS ──
    if (op == 0xB2) { ModRM m = decodeModRM(c.read8()); return "LSS " + std::string(regW(m.reg)) + ", " + formatRM(c, m, opSize16, p67, segOvr); }
    if (op == 0xB4) { ModRM m = decodeModRM(c.read8()); return "LFS " + std::string(regW(m.reg)) + ", " + formatRM(c, m, opSize16, p67, segOvr); }
    if (op == 0xB5) { ModRM m = decodeModRM(c.read8()); return "LGS " + std::string(regW(m.reg)) + ", " + formatRM(c, m, opSize16, p67, segOvr); }

    // ── IMUL r, rm ──
    if (op == 0xAF) { ModRM m = decodeModRM(c.read8()); return "IMUL " + std::string(regW(m.reg)) + ", " + formatRM(c, m, opSize16, p67, segOvr); }

    // ── MOVZX/MOVSX ──
    if (op == 0xB6) { ModRM m = decodeModRM(c.read8()); return "MOVZX " + std::string(regW(m.reg)) + ", " + formatRM(c, m, 8, p67, segOvr); }
    if (op == 0xB7) { ModRM m = decodeModRM(c.read8()); return "MOVZX " + std::string(regW(m.reg)) + ", " + formatRM(c, m, 16, p67, segOvr); }
    if (op == 0xBE) { ModRM m = decodeModRM(c.read8()); return "MOVSX " + std::string(regW(m.reg)) + ", " + formatRM(c, m, 8, p67, segOvr); }
    if (op == 0xBF) { ModRM m = decodeModRM(c.read8()); return "MOVSX " + std::string(regW(m.reg)) + ", " + formatRM(c, m, 16, p67, segOvr); }

    // Unknown 0F opcode – consume assumed ModRM byte
    c.read8();
    std::ostringstream s;
    s << "DB 0Fh, " << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << (int)op << 'h';
    return s.str();
}

} // namespace fador::cpu
