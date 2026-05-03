#include "Assembler.hpp"
#include <algorithm>
#include <cctype>
#include <sstream>
#include <iostream>
#include <unordered_map>

namespace fador::cpu {

// ═══════════════════════════════════════════════════════════════════
//  Utility
// ═══════════════════════════════════════════════════════════════════

std::string Assembler::toUpper(const std::string& s) {
    std::string r = s;
    std::transform(r.begin(), r.end(), r.begin(), ::toupper);
    return r;
}

// ═══════════════════════════════════════════════════════════════════
//  Register name tables
// ═══════════════════════════════════════════════════════════════════

bool Assembler::isReg8(const std::string& s, uint8_t& idx) {
    static const char* names[] = {"AL","CL","DL","BL","AH","CH","DH","BH"};
    for (int i = 0; i < 8; ++i) if (s == names[i]) { idx = i; return true; }
    return false;
}
bool Assembler::isReg16(const std::string& s, uint8_t& idx) {
    static const char* names[] = {"AX","CX","DX","BX","SP","BP","SI","DI"};
    for (int i = 0; i < 8; ++i) if (s == names[i]) { idx = i; return true; }
    return false;
}
bool Assembler::isReg32(const std::string& s, uint8_t& idx) {
    static const char* names[] = {"EAX","ECX","EDX","EBX","ESP","EBP","ESI","EDI"};
    for (int i = 0; i < 8; ++i) if (s == names[i]) { idx = i; return true; }
    return false;
}
bool Assembler::isSegReg(const std::string& s, uint8_t& idx) {
    static const char* names[] = {"ES","CS","SS","DS","FS","GS"};
    for (int i = 0; i < 6; ++i) if (s == names[i]) { idx = i; return true; }
    return false;
}

// ═══════════════════════════════════════════════════════════════════
//  Tokeniser
// ═══════════════════════════════════════════════════════════════════

std::vector<Assembler::Token> Assembler::tokenize(const std::string& line) const {
    std::vector<Token> toks;
    size_t i = 0;
    while (i < line.size()) {
        char c = line[i];
        if (std::isspace(static_cast<unsigned char>(c))) { ++i; continue; }
        if (c == ';') break; // Comment
        if (c == ',') { toks.push_back({TokKind::Comma, ","}); ++i; continue; }
        if (c == ':') { toks.push_back({TokKind::Colon, ":"}); ++i; continue; }
        if (c == '[') { toks.push_back({TokKind::LBracket, "["}); ++i; continue; }
        if (c == ']') { toks.push_back({TokKind::RBracket, "]"}); ++i; continue; }
        if (c == '+') { toks.push_back({TokKind::Plus, "+"}); ++i; continue; }
        if (c == '-') { toks.push_back({TokKind::Minus, "-"}); ++i; continue; }
        if (c == '*') { toks.push_back({TokKind::Star, "*"}); ++i; continue; }

        // Number: hex (0x.., ..h), decimal, or bare hex bytes like '0F' / 'FF'
        // Treat as number if it starts with a digit, or if it starts with
        // two hex digits that are *not* followed by an alpha mnemonic character
        // (this avoids treating mnemonics like "ADD", "DEC", "CBW" as numbers).
        bool looksLikeNumber = false;
        if (std::isdigit(static_cast<unsigned char>(c))) looksLikeNumber = true;
        else if (std::isxdigit(static_cast<unsigned char>(c)) &&
                 (i + 1 < line.size()) && std::isxdigit(static_cast<unsigned char>(line[i+1]))) {
            // Allow if either the token ends after two hex digits, the next
            // character is non-alpha (space/punct), or it's a hex-suffix 'h/H'.
            if (i + 2 >= line.size()) looksLikeNumber = true;
            else {
                unsigned char third = static_cast<unsigned char>(line[i+2]);
                if (!std::isalpha(third) || third == 'h' || third == 'H') looksLikeNumber = true;
            }
        }
        if (looksLikeNumber) {
            size_t start = i;
            while (i < line.size() && std::isxdigit(static_cast<unsigned char>(line[i])))
                ++i;
            std::string numStr = line.substr(start, i - start);
            uint32_t val = 0;
            bool isHex = false;
            // Handle explicit hex suffix 'h' (e.g., 0Ah)
            if (i < line.size() && (line[i] == 'h' || line[i] == 'H')) {
                isHex = true; ++i;
            }
            // 0x prefix inside the token (e.g., 0x1A)
            else if (numStr.size() > 2 && numStr[0] == '0' && (numStr[1] == 'x' || numStr[1] == 'X')) {
                isHex = true;
                numStr = numStr.substr(2);
            }
            // Treat short tokens (1-2 hex digits) as hex bytes (e.g., '0F', '21'),
            // otherwise if token contains any hex letter (A-F) treat as hex.
            else {
                if (numStr.size() <= 2) {
                    isHex = true;
                } else {
                    for (char ch : numStr) {
                        if (std::isalpha(static_cast<unsigned char>(ch))) { isHex = true; break; }
                    }
                }
            }
            if (isHex) val = std::stoul(numStr, nullptr, 16);
            else val = std::stoul(numStr, nullptr, 10);
            toks.push_back({TokKind::Imm, numStr, val});
            continue;
        }

        // Identifier / register / keyword
        if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
            size_t start = i;
            while (i < line.size() && (std::isalnum(static_cast<unsigned char>(line[i])) || line[i] == '_'))
                ++i;
            std::string word = toUpper(line.substr(start, i - start));
            // Check for hex literal ending in 'H' already consumed as part of ident
            // e.g. "0Ah" — first char '0' would be caught by digit branch above

            uint8_t ridx = 0;
            if (isReg8(word, ridx))        toks.push_back({TokKind::Reg8, word, ridx});
            else if (isReg16(word, ridx))   toks.push_back({TokKind::Reg16, word, ridx});
            else if (isReg32(word, ridx))   toks.push_back({TokKind::Reg32, word, ridx});
            else if (isSegReg(word, ridx))  toks.push_back({TokKind::SegReg, word, ridx});
            else if (word == "BYTE" || word == "DB")
                toks.push_back({TokKind::SizeBytePtr, word});
            else if (word == "WORD" || word == "DW")
                toks.push_back({TokKind::SizeWord, word});
            else if (word == "DWORD" || word == "DD")
                toks.push_back({TokKind::SizeDwordPtr, word});
            else if (word == "PTR") {} // Consumed silently (BYTE PTR → just BYTE)
            else if (word == "FAR")
                toks.push_back({TokKind::Far, word});
            else if (word == "SHORT")
                toks.push_back({TokKind::Short, word});
            else
                toks.push_back({TokKind::Ident, word});
            continue;
        }

        // Skip unknown
        ++i;
    }
    toks.push_back({TokKind::Eol, ""});
    return toks;
}

// ═══════════════════════════════════════════════════════════════════
//  Operand parser
// ═══════════════════════════════════════════════════════════════════

bool Assembler::parseOperand(const std::vector<Token>& toks, size_t& pos, Operand& out) const {
    if (pos >= toks.size() || toks[pos].kind == TokKind::Eol) return false;

    // Size hint prefix  (BYTE PTR, WORD PTR, DWORD PTR before '[')
    if (toks[pos].kind == TokKind::SizeBytePtr)   { out.sizeHint = 1; ++pos; }
    else if (toks[pos].kind == TokKind::SizeWord)  { out.sizeHint = 2; ++pos; }
    else if (toks[pos].kind == TokKind::SizeDwordPtr) { out.sizeHint = 4; ++pos; }

    // Check for segment override  (e.g. ES: before [])
    if (pos + 1 < toks.size() && toks[pos].kind == TokKind::SegReg &&
        toks[pos+1].kind == TokKind::Colon) {
        out.segOverride = true;
        out.segReg = static_cast<uint8_t>(toks[pos].value);
        pos += 2; // skip "ES:"
    }

    // Short / Far prefix
    bool forceShort = false;
    bool forceFar = false;
    if (pos < toks.size() && toks[pos].kind == TokKind::Short) { forceShort = true; ++pos; }
    if (pos < toks.size() && toks[pos].kind == TokKind::Far) { forceFar = true; ++pos; }
    (void)forceShort; (void)forceFar;

    const auto& t = toks[pos];

    // Register operands
    if (t.kind == TokKind::Reg8) {
        out.kind = OpKind::Reg8; out.reg = static_cast<uint8_t>(t.value); ++pos;
        return true;
    }
    if (t.kind == TokKind::Reg16) {
        out.kind = OpKind::Reg16; out.reg = static_cast<uint8_t>(t.value); ++pos;
        return true;
    }
    if (t.kind == TokKind::Reg32) {
        out.kind = OpKind::Reg32; out.reg = static_cast<uint8_t>(t.value); ++pos;
        return true;
    }
    if (t.kind == TokKind::SegReg) {
        out.kind = OpKind::SegReg; out.reg = static_cast<uint8_t>(t.value); ++pos;
        return true;
    }

    // Memory operand: [base + index + disp]
    if (t.kind == TokKind::LBracket) {
        ++pos;
        out.kind = OpKind::Mem;
        out.baseReg = 0xFF;
        out.indexReg = 0xFF;
        out.imm = 0;
        out.hasDisp = false;
        bool negate = false;

        while (pos < toks.size() && toks[pos].kind != TokKind::RBracket) {
            const auto& ct = toks[pos];
            if (ct.kind == TokKind::Plus)  { negate = false; ++pos; continue; }
            if (ct.kind == TokKind::Minus) { negate = true;  ++pos; continue; }

            if (ct.kind == TokKind::Reg16) {
                if (out.baseReg == 0xFF) out.baseReg = static_cast<uint8_t>(ct.value);
                else out.indexReg = static_cast<uint8_t>(ct.value);
                ++pos;
            } else if (ct.kind == TokKind::Imm) {
                out.hasDisp = true;
                if (negate) out.imm -= ct.value; else out.imm += ct.value;
                negate = false;
                ++pos;
            } else {
                ++pos; // skip unknown
            }
        }
        if (pos < toks.size() && toks[pos].kind == TokKind::RBracket) ++pos;
        // If only displacement, no register
        if (out.baseReg == 0xFF && out.indexReg == 0xFF) out.hasDisp = true;
        return true;
    }

    // Immediate
    if (t.kind == TokKind::Imm) {
        out.kind = OpKind::Imm; out.imm = t.value; ++pos;
        // Check for seg:off pattern  (e.g. 1234h:5678h)
        if (pos < toks.size() && toks[pos].kind == TokKind::Colon &&
            pos + 1 < toks.size() && toks[pos+1].kind == TokKind::Imm) {
            // Far pointer — pack seg:off as imm = (seg << 16) | off
            uint32_t seg = out.imm;
            ++pos; // skip ':'
            out.imm = (seg << 16) | toks[pos].value;
            ++pos;
        }
        return true;
    }

    return false;
}

// ═══════════════════════════════════════════════════════════════════
//  ModR/M encoding (16-bit addressing only for real mode)
// ═══════════════════════════════════════════════════════════════════

uint8_t Assembler::encodeModRM(const Operand& rm, uint8_t regField, std::vector<uint8_t>& out) const {
    if (rm.kind == OpKind::Reg8 || rm.kind == OpKind::Reg16 || rm.kind == OpKind::Reg32) {
        // mod=11, rm=register
        out.push_back(0xC0 | (regField << 3) | rm.reg);
        return 0xC0;
    }

    // Memory operand — 16-bit addressing modes
    // Encoding table for base+index:
    //   BX+SI=0, BX+DI=1, BP+SI=2, BP+DI=3, SI=4, DI=5, BP=6, BX=7
    //   Direct (no base)=6 with mod=00

    uint8_t rmCode = 0;
    bool directAddr = false;
    uint8_t base = rm.baseReg;
    uint8_t idx = rm.indexReg;

    // Normalise: if idx is BX/BP and base is SI/DI, swap them
    if (idx != 0xFF && base != 0xFF) {
        // BX=3, BP=5, SI=6, DI=7
        if ((base == 6 || base == 7) && (idx == 3 || idx == 5)) {
            std::swap(base, idx);
        }
    }

    if (base == 0xFF && idx == 0xFF) {
        // Direct address [disp16]
        directAddr = true;
        rmCode = 6;
    } else if (base == 3 && idx == 6) rmCode = 0; // [BX+SI]
    else if (base == 3 && idx == 7)   rmCode = 1; // [BX+DI]
    else if (base == 5 && idx == 6)   rmCode = 2; // [BP+SI]
    else if (base == 5 && idx == 7)   rmCode = 3; // [BP+DI]
    else if (base == 6 && idx == 0xFF) rmCode = 4; // [SI]
    else if (base == 7 && idx == 0xFF) rmCode = 5; // [DI]
    else if (base == 5 && idx == 0xFF) rmCode = 6; // [BP+disp]
    else if (base == 3 && idx == 0xFF) rmCode = 7; // [BX]
    else {
        // Unsupported combination
        return 0xFF;
    }

    if (directAddr) {
        // mod=00, rm=110, disp16
        out.push_back(0x00 | (regField << 3) | 6);
        emitImm16(out, rm.imm);
        return 0;
    }

    // Determine mod based on displacement
    bool needsDisp = rm.hasDisp && rm.imm != 0;
    // Special: [BP] with no disp needs disp8=0 (mod=00 rm=110 is direct addr)
    if (base == 5 && idx == 0xFF && !rm.hasDisp) needsDisp = true;

    if (!needsDisp && !(base == 5 && idx == 0xFF)) {
        // mod=00
        out.push_back(0x00 | (regField << 3) | rmCode);
    } else if (rm.imm == 0 || (static_cast<int32_t>(rm.imm) >= -128 && static_cast<int32_t>(rm.imm) <= 127)) {
        // mod=01, disp8
        out.push_back(0x40 | (regField << 3) | rmCode);
        out.push_back(static_cast<uint8_t>(rm.imm));
    } else {
        // mod=10, disp16
        out.push_back(0x80 | (regField << 3) | rmCode);
        emitImm16(out, rm.imm);
    }

    return out.back() & 0xC0;
}

void Assembler::emitImm8(std::vector<uint8_t>& out, uint32_t val) const {
    out.push_back(static_cast<uint8_t>(val & 0xFF));
}
void Assembler::emitImm16(std::vector<uint8_t>& out, uint32_t val) const {
    out.push_back(static_cast<uint8_t>(val & 0xFF));
    out.push_back(static_cast<uint8_t>((val >> 8) & 0xFF));
}
void Assembler::emitImm32(std::vector<uint8_t>& out, uint32_t val) const {
    out.push_back(static_cast<uint8_t>(val & 0xFF));
    out.push_back(static_cast<uint8_t>((val >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>((val >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((val >> 24) & 0xFF));
}

// ═══════════════════════════════════════════════════════════════════
//  ALU encoder (ADD,OR,ADC,SBB,AND,SUB,XOR,CMP)
// ═══════════════════════════════════════════════════════════════════

AsmResult Assembler::encodeALU(const std::string& mnem, uint8_t aluOp,
                               const Operand& dst, const Operand& src, uint32_t) const {
    AsmResult r;

    // ALU r/m8, imm8    (0x80 /op ib) or AL,imm8 (short form 0x04+op*8)
    // ALU r/m16, imm16  (0x81 /op iw) or AX,imm16 (short form 0x05+op*8)
    // ALU r/m16, imm8   (0x83 /op ib sign-extended)
    // ALU r/m8, r8      (0x00+op*8)
    // ALU r/m16, r16    (0x01+op*8)
    // ALU r8, r/m8      (0x02+op*8)
    // ALU r16, r/m16    (0x03+op*8)

    if (dst.kind == OpKind::Reg8 && src.kind == OpKind::Imm) {
        if (dst.reg == 0) { // AL short form
            r.bytes.push_back(0x04 + aluOp * 8);
            emitImm8(r.bytes, src.imm);
        } else {
            r.bytes.push_back(0x80);
            r.bytes.push_back(0xC0 | (aluOp << 3) | dst.reg);
            emitImm8(r.bytes, src.imm);
        }
    } else if (dst.kind == OpKind::Reg16 && src.kind == OpKind::Imm) {
        if (dst.reg == 0) { // AX short form
            r.bytes.push_back(0x05 + aluOp * 8);
            emitImm16(r.bytes, src.imm);
        } else if (src.imm <= 0x7F || (src.imm >= 0xFF80)) {
            // Sign-extended imm8
            r.bytes.push_back(0x83);
            r.bytes.push_back(0xC0 | (aluOp << 3) | dst.reg);
            emitImm8(r.bytes, src.imm);
        } else {
            r.bytes.push_back(0x81);
            r.bytes.push_back(0xC0 | (aluOp << 3) | dst.reg);
            emitImm16(r.bytes, src.imm);
        }
    } else if ((dst.kind == OpKind::Mem || dst.kind == OpKind::Reg8) && src.kind == OpKind::Imm && dst.sizeHint == 1) {
        r.bytes.push_back(0x80);
        encodeModRM(dst, aluOp, r.bytes);
        emitImm8(r.bytes, src.imm);
    } else if ((dst.kind == OpKind::Mem || dst.kind == OpKind::Reg16) && src.kind == OpKind::Imm) {
        if (src.imm <= 0x7F || src.imm >= 0xFF80) {
            r.bytes.push_back(0x83);
            encodeModRM(dst, aluOp, r.bytes);
            emitImm8(r.bytes, src.imm);
        } else {
            r.bytes.push_back(0x81);
            encodeModRM(dst, aluOp, r.bytes);
            emitImm16(r.bytes, src.imm);
        }
    } else if ((dst.kind == OpKind::Mem || dst.kind == OpKind::Reg8) && src.kind == OpKind::Reg8) {
        r.bytes.push_back(0x00 + aluOp * 8);
        encodeModRM(dst, src.reg, r.bytes);
    } else if ((dst.kind == OpKind::Mem || dst.kind == OpKind::Reg16) && src.kind == OpKind::Reg16) {
        r.bytes.push_back(0x01 + aluOp * 8);
        encodeModRM(dst, src.reg, r.bytes);
    } else if (dst.kind == OpKind::Reg8 && (src.kind == OpKind::Mem || src.kind == OpKind::Reg8)) {
        r.bytes.push_back(0x02 + aluOp * 8);
        encodeModRM(src, dst.reg, r.bytes);
    } else if (dst.kind == OpKind::Reg16 && (src.kind == OpKind::Mem || src.kind == OpKind::Reg16)) {
        r.bytes.push_back(0x03 + aluOp * 8);
        encodeModRM(src, dst.reg, r.bytes);
    } else {
        r.error = "Invalid operands for " + mnem;
    }
    return r;
}

// ═══════════════════════════════════════════════════════════════════
//  Shift/Rotate encoder (ROL,ROR,RCL,RCR,SHL,SHR,SAR)
// ═══════════════════════════════════════════════════════════════════

AsmResult Assembler::encodeShift(const std::string& mnem, uint8_t subOp,
                                 const Operand& dst, const Operand& src) const {
    AsmResult r;
    bool is8 = (dst.kind == OpKind::Reg8 || dst.sizeHint == 1);

    if (src.kind == OpKind::Imm && src.imm == 1) {
        r.bytes.push_back(is8 ? 0xD0 : 0xD1);
        encodeModRM(dst, subOp, r.bytes);
    } else if (src.kind == OpKind::Reg8 && src.reg == 1) { // CL
        r.bytes.push_back(is8 ? 0xD2 : 0xD3);
        encodeModRM(dst, subOp, r.bytes);
    } else if (src.kind == OpKind::Imm) {
        r.bytes.push_back(is8 ? 0xC0 : 0xC1);
        encodeModRM(dst, subOp, r.bytes);
        emitImm8(r.bytes, src.imm);
    } else {
        r.error = "Invalid operands for " + mnem;
    }
    return r;
}

// ═══════════════════════════════════════════════════════════════════
//  Jcc encoder
// ═══════════════════════════════════════════════════════════════════

AsmResult Assembler::encodeJcc(uint8_t cc, const Operand& target, uint32_t origin) const {
    AsmResult r;
    if (target.kind != OpKind::Imm) { r.error = "Jcc requires immediate target"; return r; }

    int32_t disp = static_cast<int32_t>(target.imm) - static_cast<int32_t>(origin + 2);
    if (disp >= -128 && disp <= 127) {
        r.bytes.push_back(0x70 + cc);
        r.bytes.push_back(static_cast<uint8_t>(disp));
    } else {
        // Near conditional jump (0F 8x rw)
        disp = static_cast<int32_t>(target.imm) - static_cast<int32_t>(origin + 4);
        r.bytes.push_back(0x0F);
        r.bytes.push_back(0x80 + cc);
        emitImm16(r.bytes, static_cast<uint32_t>(disp));
    }
    return r;
}

// ═══════════════════════════════════════════════════════════════════
//  Main assemble entry point
// ═══════════════════════════════════════════════════════════════════

AsmResult Assembler::assembleLine(const std::string& line, uint32_t origin) const {
    AsmResult result;
    auto toks = tokenize(line);
    if (toks.empty() || toks[0].kind == TokKind::Eol) return result; // Empty line

    // Skip label if present (IDENT followed by COLON)
    size_t pos = 0;
    if (toks.size() >= 2 && toks[0].kind == TokKind::Ident && toks[1].kind == TokKind::Colon)
        pos = 2;

    if (pos >= toks.size() || toks[pos].kind == TokKind::Eol) return result;

    // Prefix handling
    std::vector<uint8_t> prefixes;
    while (pos < toks.size() && toks[pos].kind == TokKind::Ident) {
        const auto& w = toks[pos].text;
        if      (w == "REP" || w == "REPE" || w == "REPZ")  { prefixes.push_back(0xF3); ++pos; }
        else if (w == "REPNE" || w == "REPNZ") { prefixes.push_back(0xF2); ++pos; }
        else if (w == "LOCK")                  { prefixes.push_back(0xF0); ++pos; }
        else break;
    }

    if (pos >= toks.size() || toks[pos].kind == TokKind::Eol) return result;

    std::string mnem;
    // Allow size tokens (DB/DW/DD) and bare byte sequences (e.g. "0F FF 21")
    if (toks[pos].kind == TokKind::Ident) {
        mnem = toks[pos].text;
        ++pos;
    } else if (toks[pos].kind == TokKind::SizeBytePtr) {
        mnem = "DB";
        ++pos;
    } else if (toks[pos].kind == TokKind::SizeWord) {
        mnem = "DW";
        ++pos;
    } else if (toks[pos].kind == TokKind::SizeDwordPtr) {
        mnem = "DD";
        ++pos;
    } else if (toks[pos].kind == TokKind::Imm) {
        // Bare byte sequence shorthand
        mnem = "DB";
    } else {
        result.error = "Expected mnemonic";
        return result;
    }

    // Special-case data directives: parse successive immediate tokens
    if (mnem == "DB" || mnem == "DW" || mnem == "DD") {
        while (pos < toks.size() && toks[pos].kind != TokKind::Eol) {
            if (toks[pos].kind == TokKind::Comma) { ++pos; continue; }
            if (toks[pos].kind == TokKind::Imm) {
                if (mnem == "DB") result.bytes.push_back(static_cast<uint8_t>(toks[pos].value));
                else if (mnem == "DW") emitImm16(result.bytes, toks[pos].value);
                else if (mnem == "DD") emitImm32(result.bytes, toks[pos].value);
                ++pos;
                continue;
            }
            // Unknown token in data directive — skip it
            ++pos;
        }
        return result;
    }

    // Parse up to 2 operands for non-data mnemonics
    Operand op1, op2;
    bool hasOp1 = false, hasOp2 = false;
    if (pos < toks.size() && toks[pos].kind != TokKind::Eol) {
        hasOp1 = parseOperand(toks, pos, op1);
    }
    if (hasOp1 && pos < toks.size() && toks[pos].kind == TokKind::Comma) {
        ++pos; // skip comma
        hasOp2 = parseOperand(toks, pos, op2);
    }

    // Debugging: print token/operand parse for suspicious mnemonics
    if (mnem == "CBW" || mnem == "ADD" || mnem == "DEC" || mnem == "CALL") {
        std::cerr << "ASM_DEBUG: line='" << line << "' mnem=" << mnem << " tokens=[";
        for (const auto &t : toks) {
            std::cerr << "(" << static_cast<int>(t.kind) << ":" << t.text << ":" << t.value << "),";
        }
        std::cerr << "] pos=" << pos << " hasOp1=" << hasOp1 << " hasOp2=" << hasOp2;
        if (hasOp1) std::cerr << " op1(kind=" << static_cast<int>(op1.kind) << ",reg=" << int(op1.reg)
                               << ",imm=" << op1.imm << ",size=" << int(op1.sizeHint) << ")";
        if (hasOp2) std::cerr << " op2(kind=" << static_cast<int>(op2.kind) << ",reg=" << int(op2.reg)
                               << ",imm=" << op2.imm << ",size=" << int(op2.sizeHint) << ")";
        std::cerr << "\n";
    }

    // Segment override prefix from operands
    const Operand& memOp = (op1.kind == OpKind::Mem) ? op1 : op2;
    if (memOp.segOverride) {
        static const uint8_t segPrefixes[] = {0x26, 0x2E, 0x36, 0x3E, 0x64, 0x65};
        if (memOp.segReg < 6)
            prefixes.push_back(segPrefixes[memOp.segReg]);
    }

    // ── Encode instruction ──
    auto encodedResult = [&]() -> AsmResult {
        AsmResult r;

        // ── ALU ──
        static const std::unordered_map<std::string, uint8_t> aluOps = {
            {"ADD",0}, {"OR",1}, {"ADC",2}, {"SBB",3},
            {"AND",4}, {"SUB",5}, {"XOR",6}, {"CMP",7}
        };
        if (auto it = aluOps.find(mnem); it != aluOps.end()) {
            return encodeALU(mnem, it->second, op1, op2, origin);
        }

        // ── Shifts ──
        static const std::unordered_map<std::string, uint8_t> shiftOps = {
            {"ROL",0}, {"ROR",1}, {"RCL",2}, {"RCR",3},
            {"SHL",4}, {"SAL",4}, {"SHR",5}, {"SAR",7}
        };
        if (auto it = shiftOps.find(mnem); it != shiftOps.end()) {
            return encodeShift(mnem, it->second, op1, op2);
        }

        // ── Conditional jumps ──
        static const std::unordered_map<std::string, uint8_t> jccOps = {
            {"JO",0},  {"JNO",1}, {"JB",2},  {"JC",2},  {"JNAE",2},
            {"JNB",3}, {"JNC",3}, {"JAE",3}, {"JZ",4},  {"JE",4},
            {"JNZ",5}, {"JNE",5}, {"JBE",6}, {"JNA",6}, {"JA",7},
            {"JNBE",7},{"JS",8},  {"JNS",9}, {"JP",0xA},{"JPE",0xA},
            {"JNP",0xB},{"JPO",0xB},{"JL",0xC},{"JNGE",0xC},
            {"JGE",0xD},{"JNL",0xD},{"JLE",0xE},{"JNG",0xE},
            {"JG",0xF},{"JNLE",0xF}
        };
        if (auto it = jccOps.find(mnem); it != jccOps.end()) {
            return encodeJcc(it->second, op1, origin);
        }

        // ── LOOP/JCXZ ──
        if (mnem == "LOOP" || mnem == "LOOPE" || mnem == "LOOPZ" ||
            mnem == "LOOPNE" || mnem == "LOOPNZ" || mnem == "JCXZ") {
            if (op1.kind != OpKind::Imm) { r.error = mnem + " requires immediate target"; return r; }
            uint8_t opcode = 0xE2;
            if (mnem == "LOOPNE" || mnem == "LOOPNZ") opcode = 0xE0;
            else if (mnem == "LOOPE" || mnem == "LOOPZ") opcode = 0xE1;
            else if (mnem == "JCXZ") opcode = 0xE3;
            int32_t disp = static_cast<int32_t>(op1.imm) - static_cast<int32_t>(origin + 2);
            if (disp < -128 || disp > 127) { r.error = mnem + " target out of range"; return r; }
            r.bytes.push_back(opcode);
            r.bytes.push_back(static_cast<uint8_t>(disp));
            return r;
        }

        // ── MOV ──
        if (mnem == "MOV") {
            // MOV r/m8, r8
            if ((op1.kind == OpKind::Mem || op1.kind == OpKind::Reg8) && op2.kind == OpKind::Reg8) {
                r.bytes.push_back(0x88);
                encodeModRM(op1, op2.reg, r.bytes);
            }
            // MOV r/m16, r16
            else if ((op1.kind == OpKind::Mem || op1.kind == OpKind::Reg16) && op2.kind == OpKind::Reg16) {
                r.bytes.push_back(0x89);
                encodeModRM(op1, op2.reg, r.bytes);
            }
            // MOV r8, r/m8
            else if (op1.kind == OpKind::Reg8 && (op2.kind == OpKind::Mem || op2.kind == OpKind::Reg8)) {
                r.bytes.push_back(0x8A);
                encodeModRM(op2, op1.reg, r.bytes);
            }
            // MOV r16, r/m16
            else if (op1.kind == OpKind::Reg16 && (op2.kind == OpKind::Mem || op2.kind == OpKind::Reg16)) {
                r.bytes.push_back(0x8B);
                encodeModRM(op2, op1.reg, r.bytes);
            }
            // MOV r/m16, Sreg
            else if ((op1.kind == OpKind::Mem || op1.kind == OpKind::Reg16) && op2.kind == OpKind::SegReg) {
                r.bytes.push_back(0x8C);
                encodeModRM(op1, op2.reg, r.bytes);
            }
            // MOV Sreg, r/m16
            else if (op1.kind == OpKind::SegReg && (op2.kind == OpKind::Mem || op2.kind == OpKind::Reg16)) {
                r.bytes.push_back(0x8E);
                encodeModRM(op2, op1.reg, r.bytes);
            }
            // MOV r8, imm8
            else if (op1.kind == OpKind::Reg8 && op2.kind == OpKind::Imm) {
                r.bytes.push_back(0xB0 + op1.reg);
                emitImm8(r.bytes, op2.imm);
            }
            // MOV r16, imm16
            else if (op1.kind == OpKind::Reg16 && op2.kind == OpKind::Imm) {
                r.bytes.push_back(0xB8 + op1.reg);
                emitImm16(r.bytes, op2.imm);
            }
            // MOV r/m8, imm8  (memory dest)
            else if (op1.kind == OpKind::Mem && op2.kind == OpKind::Imm && op1.sizeHint == 1) {
                r.bytes.push_back(0xC6);
                encodeModRM(op1, 0, r.bytes);
                emitImm8(r.bytes, op2.imm);
            }
            // MOV r/m16, imm16 (memory dest)
            else if (op1.kind == OpKind::Mem && op2.kind == OpKind::Imm) {
                r.bytes.push_back(0xC7);
                encodeModRM(op1, 0, r.bytes);
                emitImm16(r.bytes, op2.imm);
            }
            else {
                r.error = "Invalid operands for MOV";
            }
            return r;
        }

        // ── TEST ──
        if (mnem == "TEST") {
            if (op1.kind == OpKind::Reg8 && op1.reg == 0 && op2.kind == OpKind::Imm) {
                r.bytes.push_back(0xA8); emitImm8(r.bytes, op2.imm);
            } else if (op1.kind == OpKind::Reg16 && op1.reg == 0 && op2.kind == OpKind::Imm) {
                r.bytes.push_back(0xA9); emitImm16(r.bytes, op2.imm);
            } else if ((op1.kind == OpKind::Reg8 || op1.kind == OpKind::Mem) && op2.kind == OpKind::Imm) {
                r.bytes.push_back(0xF6);
                encodeModRM(op1, 0, r.bytes);
                emitImm8(r.bytes, op2.imm);
            } else if ((op1.kind == OpKind::Reg16 || op1.kind == OpKind::Mem) && op2.kind == OpKind::Imm) {
                r.bytes.push_back(0xF7);
                encodeModRM(op1, 0, r.bytes);
                emitImm16(r.bytes, op2.imm);
            } else if ((op1.kind == OpKind::Reg8 || op1.kind == OpKind::Mem) && op2.kind == OpKind::Reg8) {
                r.bytes.push_back(0x84);
                encodeModRM(op1, op2.reg, r.bytes);
            } else if ((op1.kind == OpKind::Reg16 || op1.kind == OpKind::Mem) && op2.kind == OpKind::Reg16) {
                r.bytes.push_back(0x85);
                encodeModRM(op1, op2.reg, r.bytes);
            } else {
                r.error = "Invalid operands for TEST";
            }
            return r;
        }

        // ── XCHG ──
        if (mnem == "XCHG") {
            if (op1.kind == OpKind::Reg16 && op2.kind == OpKind::Reg16) {
                if (op1.reg == 0) { r.bytes.push_back(0x90 + op2.reg); }
                else if (op2.reg == 0) { r.bytes.push_back(0x90 + op1.reg); }
                else { r.bytes.push_back(0x87); encodeModRM(op2, op1.reg, r.bytes); }
            } else if (op1.kind == OpKind::Reg8 && op2.kind == OpKind::Reg8) {
                r.bytes.push_back(0x86); encodeModRM(op2, op1.reg, r.bytes);
            } else {
                r.error = "Invalid operands for XCHG";
            }
            return r;
        }

        // ── LEA ──
        if (mnem == "LEA") {
            if (op1.kind == OpKind::Reg16 && op2.kind == OpKind::Mem) {
                r.bytes.push_back(0x8D);
                encodeModRM(op2, op1.reg, r.bytes);
            } else {
                r.error = "Invalid operands for LEA";
            }
            return r;
        }

        // ── INC/DEC ──
        if (mnem == "INC") {
            if (op1.kind == OpKind::Reg16) { r.bytes.push_back(0x40 + op1.reg); }
            else if (op1.kind == OpKind::Reg8 || (op1.kind == OpKind::Mem && op1.sizeHint == 1)) {
                r.bytes.push_back(0xFE); encodeModRM(op1, 0, r.bytes);
            } else if (op1.kind == OpKind::Mem) {
                r.bytes.push_back(0xFF); encodeModRM(op1, 0, r.bytes);
            } else { r.error = "Invalid operands for INC"; }
            return r;
        }
        if (mnem == "DEC") {
            if (op1.kind == OpKind::Reg16) { r.bytes.push_back(0x48 + op1.reg); }
            else if (op1.kind == OpKind::Reg8 || (op1.kind == OpKind::Mem && op1.sizeHint == 1)) {
                r.bytes.push_back(0xFE); encodeModRM(op1, 1, r.bytes);
            } else if (op1.kind == OpKind::Mem) {
                r.bytes.push_back(0xFF); encodeModRM(op1, 1, r.bytes);
            } else { r.error = "Invalid operands for DEC"; }
            return r;
        }

        // ── NOT/NEG ──
        if (mnem == "NOT") {
            if (op1.kind == OpKind::Reg8 || op1.sizeHint == 1) {
                r.bytes.push_back(0xF6); encodeModRM(op1, 2, r.bytes);
            } else {
                r.bytes.push_back(0xF7); encodeModRM(op1, 2, r.bytes);
            }
            return r;
        }
        if (mnem == "NEG") {
            if (op1.kind == OpKind::Reg8 || op1.sizeHint == 1) {
                r.bytes.push_back(0xF6); encodeModRM(op1, 3, r.bytes);
            } else {
                r.bytes.push_back(0xF7); encodeModRM(op1, 3, r.bytes);
            }
            return r;
        }

        // ── MUL/IMUL/DIV/IDIV ──
        if (mnem == "MUL" || mnem == "IMUL" || mnem == "DIV" || mnem == "IDIV") {
            uint8_t subOp = 4; // MUL
            if (mnem == "IMUL") subOp = 5;
            if (mnem == "DIV")  subOp = 6;
            if (mnem == "IDIV") subOp = 7;

            // Single operand form: MUL r/m
            if (hasOp1 && !hasOp2) {
                if (op1.kind == OpKind::Reg8 || op1.sizeHint == 1) {
                    r.bytes.push_back(0xF6); encodeModRM(op1, subOp, r.bytes);
                } else {
                    r.bytes.push_back(0xF7); encodeModRM(op1, subOp, r.bytes);
                }
                return r;
            }
            // IMUL r16, r/m16 (0F AF)
            if (mnem == "IMUL" && hasOp2 && op2.kind != OpKind::Imm) {
                r.bytes.push_back(0x0F);
                r.bytes.push_back(0xAF);
                encodeModRM(op2, op1.reg, r.bytes);
                return r;
            }
            // IMUL r16, r/m16, imm
            if (mnem == "IMUL" && hasOp2 && op2.kind == OpKind::Imm) {
                if (op2.imm <= 0x7F || op2.imm >= 0xFF80) {
                    r.bytes.push_back(0x6B);
                    encodeModRM(op1, op1.reg, r.bytes);
                    emitImm8(r.bytes, op2.imm);
                } else {
                    r.bytes.push_back(0x69);
                    encodeModRM(op1, op1.reg, r.bytes);
                    emitImm16(r.bytes, op2.imm);
                }
                return r;
            }
            r.error = "Invalid operands for " + mnem;
            return r;
        }

        // ── PUSH/POP ──
        if (mnem == "PUSH") {
            if (op1.kind == OpKind::Reg16) { r.bytes.push_back(0x50 + op1.reg); }
            else if (op1.kind == OpKind::SegReg) {
                static const uint8_t pushSeg[] = {0x06, 0x0E, 0x16, 0x1E}; // ES,CS,SS,DS
                if (op1.reg < 4) r.bytes.push_back(pushSeg[op1.reg]);
                else if (op1.reg == 4) { r.bytes.push_back(0x0F); r.bytes.push_back(0xA0); } // FS
                else if (op1.reg == 5) { r.bytes.push_back(0x0F); r.bytes.push_back(0xA8); } // GS
            }
            else if (op1.kind == OpKind::Imm) {
                if (op1.imm <= 0x7F || op1.imm >= 0xFF80) {
                    r.bytes.push_back(0x6A); emitImm8(r.bytes, op1.imm);
                } else {
                    r.bytes.push_back(0x68); emitImm16(r.bytes, op1.imm);
                }
            }
            else if (op1.kind == OpKind::Mem) {
                r.bytes.push_back(0xFF); encodeModRM(op1, 6, r.bytes);
            }
            else { r.error = "Invalid operands for PUSH"; }
            return r;
        }
        if (mnem == "POP") {
            if (op1.kind == OpKind::Reg16) { r.bytes.push_back(0x58 + op1.reg); }
            else if (op1.kind == OpKind::SegReg) {
                // ES=0x07, CS=invalid, SS=0x17, DS=0x1F
                if (op1.reg == 0) r.bytes.push_back(0x07);       // POP ES
                else if (op1.reg == 2) r.bytes.push_back(0x17);  // POP SS
                else if (op1.reg == 3) r.bytes.push_back(0x1F);  // POP DS
                else if (op1.reg == 4) { r.bytes.push_back(0x0F); r.bytes.push_back(0xA1); } // FS
                else if (op1.reg == 5) { r.bytes.push_back(0x0F); r.bytes.push_back(0xA9); } // GS
                else { r.error = "Cannot POP CS"; }
            }
            else if (op1.kind == OpKind::Mem) {
                r.bytes.push_back(0x8F); encodeModRM(op1, 0, r.bytes);
            }
            else { r.error = "Invalid operands for POP"; }
            return r;
        }

        // ── JMP ──
        if (mnem == "JMP") {
            if (op1.kind == OpKind::Imm) {
                // Check for far jump (seg:off packed as (seg<<16)|off)
                if (op1.imm > 0xFFFF) {
                    // JMP far seg:off
                    r.bytes.push_back(0xEA);
                    emitImm16(r.bytes, op1.imm & 0xFFFF);       // offset
                    emitImm16(r.bytes, (op1.imm >> 16) & 0xFFFF); // segment
                } else {
                    int32_t disp = static_cast<int32_t>(op1.imm) - static_cast<int32_t>(origin + 2);
                    if (disp >= -128 && disp <= 127) {
                        r.bytes.push_back(0xEB);
                        r.bytes.push_back(static_cast<uint8_t>(disp));
                    } else {
                        disp = static_cast<int32_t>(op1.imm) - static_cast<int32_t>(origin + 3);
                        r.bytes.push_back(0xE9);
                        emitImm16(r.bytes, static_cast<uint32_t>(disp));
                    }
                }
            } else if (op1.kind == OpKind::Reg16 || op1.kind == OpKind::Mem) {
                r.bytes.push_back(0xFF);
                encodeModRM(op1, 4, r.bytes);
            } else { r.error = "Invalid operands for JMP"; }
            return r;
        }

        // ── CALL ──
        if (mnem == "CALL") {
            if (op1.kind == OpKind::Imm) {
                if (op1.imm > 0xFFFF) {
                    // CALL far seg:off
                    r.bytes.push_back(0x9A);
                    emitImm16(r.bytes, op1.imm & 0xFFFF);
                    emitImm16(r.bytes, (op1.imm >> 16) & 0xFFFF);
                } else {
                    int32_t disp = static_cast<int32_t>(op1.imm) - static_cast<int32_t>(origin + 3);
                    r.bytes.push_back(0xE8);
                    emitImm16(r.bytes, static_cast<uint32_t>(disp));
                }
            } else if (op1.kind == OpKind::Reg16 || op1.kind == OpKind::Mem) {
                r.bytes.push_back(0xFF);
                encodeModRM(op1, 2, r.bytes);
            } else { r.error = "Invalid operands for CALL"; }
            return r;
        }

        // ── RET/RETF ──
        if (mnem == "RET" || mnem == "RETN") {
            if (hasOp1 && op1.kind == OpKind::Imm) {
                r.bytes.push_back(0xC2); emitImm16(r.bytes, op1.imm);
            } else {
                r.bytes.push_back(0xC3);
            }
            return r;
        }
        if (mnem == "RETF") {
            if (hasOp1 && op1.kind == OpKind::Imm) {
                r.bytes.push_back(0xCA); emitImm16(r.bytes, op1.imm);
            } else {
                r.bytes.push_back(0xCB);
            }
            return r;
        }

        // ── INT ──
        if (mnem == "INT") {
            if (op1.kind == OpKind::Imm) {
                if (op1.imm == 3) { r.bytes.push_back(0xCC); }
                else { r.bytes.push_back(0xCD); emitImm8(r.bytes, op1.imm); }
            } else { r.error = "INT requires immediate operand"; }
            return r;
        }

        // ── HLE_INT ──
        if (mnem == "HLE_INT") {
            if (op1.kind == OpKind::Imm) {
                r.bytes.push_back(0x0F);
                r.bytes.push_back(0xFF);
                emitImm8(r.bytes, op1.imm);
            } else { r.error = "HLE_INT requires immediate operand"; }
            return r;
        }

        // ── No-operand instructions ──
        static const std::unordered_map<std::string, std::vector<uint8_t>> noOpInstrs = {
            {"NOP",  {0x90}},
            {"HLT",  {0xF4}},
            {"CMC",  {0xF5}},
            {"CLC",  {0xF8}},
            {"STC",  {0xF9}},
            {"CLI",  {0xFA}},
            {"STI",  {0xFB}},
            {"CLD",  {0xFC}},
            {"STD",  {0xFD}},
            {"SAHF", {0x9E}},
            {"LAHF", {0x9F}},
            {"CBW",  {0x98}},
            {"CWD",  {0x99}},
            {"CWDE", {0x66, 0x98}},
            {"CDQ",  {0x66, 0x99}},
            {"AAA",  {0x37}},
            {"AAS",  {0x3F}},
            {"DAA",  {0x27}},
            {"DAS",  {0x2F}},
            {"AAM",  {0xD4, 0x0A}},
            {"AAD",  {0xD5, 0x0A}},
            {"PUSHA",{0x60}},
            {"PUSHAD",{0x66, 0x60}},
            {"POPA", {0x61}},
            {"POPAD",{0x66, 0x61}},
            {"PUSHF",{0x9C}},
            {"PUSHFD",{0x66, 0x9C}},
            {"POPF", {0x9D}},
            {"POPFD",{0x66, 0x9D}},
            {"IRET", {0xCF}},
            {"INTO", {0xCE}},
            {"LEAVE",{0xC9}},
            {"MOVSB",{0xA4}},
            {"MOVSW",{0xA5}},
            {"CMPSB",{0xA6}},
            {"CMPSW",{0xA7}},
            {"STOSB",{0xAA}},
            {"STOSW",{0xAB}},
            {"LODSB",{0xAC}},
            {"LODSW",{0xAD}},
            {"SCASB",{0xAE}},
            {"SCASW",{0xAF}},
            {"INSB", {0x6C}},
            {"INSW", {0x6D}},
            {"OUTSB",{0x6E}},
            {"OUTSW",{0x6F}},
            {"FWAIT",{0x9B}},
            {"WAIT", {0x9B}},
        };
        if (auto it = noOpInstrs.find(mnem); it != noOpInstrs.end()) {
            r.bytes = it->second;
            return r;
        }

        // ── ENTER ──
        if (mnem == "ENTER") {
            if (hasOp1 && hasOp2 && op1.kind == OpKind::Imm && op2.kind == OpKind::Imm) {
                r.bytes.push_back(0xC8);
                emitImm16(r.bytes, op1.imm);
                emitImm8(r.bytes, op2.imm);
            } else { r.error = "ENTER requires two immediate operands"; }
            return r;
        }

        // ── IN/OUT ──
        if (mnem == "IN") {
            if (op1.kind == OpKind::Reg8 && op1.reg == 0 && op2.kind == OpKind::Imm) {
                r.bytes.push_back(0xE4); emitImm8(r.bytes, op2.imm);
            } else if (op1.kind == OpKind::Reg16 && op1.reg == 0 && op2.kind == OpKind::Imm) {
                r.bytes.push_back(0xE5); emitImm8(r.bytes, op2.imm);
            } else if (op1.kind == OpKind::Reg8 && op1.reg == 0 &&
                       op2.kind == OpKind::Reg16 && op2.reg == 2) {
                r.bytes.push_back(0xEC); // IN AL, DX
            } else if (op1.kind == OpKind::Reg16 && op1.reg == 0 &&
                       op2.kind == OpKind::Reg16 && op2.reg == 2) {
                r.bytes.push_back(0xED); // IN AX, DX
            } else { r.error = "Invalid operands for IN"; }
            return r;
        }
        if (mnem == "OUT") {
            if (op1.kind == OpKind::Imm && op2.kind == OpKind::Reg8 && op2.reg == 0) {
                r.bytes.push_back(0xE6); emitImm8(r.bytes, op1.imm);
            } else if (op1.kind == OpKind::Imm && op2.kind == OpKind::Reg16 && op2.reg == 0) {
                r.bytes.push_back(0xE7); emitImm8(r.bytes, op1.imm);
            } else if (op1.kind == OpKind::Reg16 && op1.reg == 2 &&
                       op2.kind == OpKind::Reg8 && op2.reg == 0) {
                r.bytes.push_back(0xEE); // OUT DX, AL
            } else if (op1.kind == OpKind::Reg16 && op1.reg == 2 &&
                       op2.kind == OpKind::Reg16 && op2.reg == 0) {
                r.bytes.push_back(0xEF); // OUT DX, AX
            } else { r.error = "Invalid operands for OUT"; }
            return r;
        }

        // ── LES/LDS ──
        if (mnem == "LES") {
            if (op1.kind == OpKind::Reg16 && op2.kind == OpKind::Mem) {
                r.bytes.push_back(0xC4); encodeModRM(op2, op1.reg, r.bytes);
            } else { r.error = "Invalid operands for LES"; }
            return r;
        }
        if (mnem == "LDS") {
            if (op1.kind == OpKind::Reg16 && op2.kind == OpKind::Mem) {
                r.bytes.push_back(0xC5); encodeModRM(op2, op1.reg, r.bytes);
            } else { r.error = "Invalid operands for LDS"; }
            return r;
        }

        // ── x87 environment/state save-restore ──
        if (mnem == "FLDENV" || mnem == "FNSTENV" || mnem == "FRSTOR" || mnem == "FNSAVE") {
            if (op1.kind != OpKind::Mem || hasOp2) {
                r.error = mnem + " requires one memory operand";
                return r;
            }

            if (op1.sizeHint == 4) {
                r.bytes.push_back(0x66);
            } else if (op1.sizeHint != 0 && op1.sizeHint != 2) {
                r.error = mnem + " supports WORD (16-bit env/state) or DWORD (32-bit env/state) memory operands";
                return r;
            }

            if (mnem == "FLDENV") {
                r.bytes.push_back(0xD9);
                encodeModRM(op1, 4, r.bytes);
            } else if (mnem == "FNSTENV") {
                r.bytes.push_back(0xD9);
                encodeModRM(op1, 6, r.bytes);
            } else if (mnem == "FRSTOR") {
                r.bytes.push_back(0xDD);
                encodeModRM(op1, 4, r.bytes);
            } else {
                r.bytes.push_back(0xDD);
                encodeModRM(op1, 6, r.bytes);
            }
            return r;
        }

        // ── MOVZX/MOVSX (0F Bx) ──
        if (mnem == "MOVZX") {
            if (op1.kind == OpKind::Reg16 && (op2.kind == OpKind::Reg8 || op2.kind == OpKind::Mem)) {
                r.bytes.push_back(0x0F); r.bytes.push_back(0xB6);
                encodeModRM(op2, op1.reg, r.bytes);
            } else { r.error = "Invalid operands for MOVZX"; }
            return r;
        }
        if (mnem == "MOVSX") {
            if (op1.kind == OpKind::Reg16 && (op2.kind == OpKind::Reg8 || op2.kind == OpKind::Mem)) {
                r.bytes.push_back(0x0F); r.bytes.push_back(0xBE);
                encodeModRM(op2, op1.reg, r.bytes);
            } else { r.error = "Invalid operands for MOVSX"; }
            return r;
        }

        // ── BT/BTS/BTR/BTC ──
        static const std::unordered_map<std::string, std::pair<uint8_t,uint8_t>> btOps = {
            {"BT", {0xA3, 4}}, {"BTS", {0xAB, 5}}, {"BTR", {0xB3, 6}}, {"BTC", {0xBB, 7}}
        };
        if (auto it = btOps.find(mnem); it != btOps.end()) {
            if (op2.kind == OpKind::Imm) {
                r.bytes.push_back(0x0F); r.bytes.push_back(0xBA);
                encodeModRM(op1, it->second.second, r.bytes);
                emitImm8(r.bytes, op2.imm);
            } else if (op2.kind == OpKind::Reg16) {
                r.bytes.push_back(0x0F); r.bytes.push_back(it->second.first);
                encodeModRM(op1, op2.reg, r.bytes);
            } else { r.error = "Invalid operands for " + mnem; }
            return r;
        }

        // ── BSF/BSR ──
        if (mnem == "BSF") {
            r.bytes.push_back(0x0F); r.bytes.push_back(0xBC);
            encodeModRM(op2, op1.reg, r.bytes);
            return r;
        }
        if (mnem == "BSR") {
            r.bytes.push_back(0x0F); r.bytes.push_back(0xBD);
            encodeModRM(op2, op1.reg, r.bytes);
            return r;
        }

        if (mnem == "BSWAP") {
            if (op1.kind == OpKind::Reg32 && !hasOp2) {
                r.bytes.push_back(0x0F);
                r.bytes.push_back(static_cast<uint8_t>(0xC8 + op1.reg));
            } else { r.error = "BSWAP requires one 32-bit register operand"; }
            return r;
        }

        // ── SHLD/SHRD ──
        if (mnem == "SHLD" || mnem == "SHRD") {
            r.bytes.push_back(0x0F);
            // We need to handle 3-operand forms but we only parsed 2.
            // For simplicity: SHLD r/m, r, imm8  or  SHLD r/m, r, CL
            // The "r" is in op2 and we need a 3rd operand parsed...
            // For now, just stub it as an error if missing.
            r.error = mnem + ": 3-operand form not yet supported in assembler";
            return r;
        }

        r.error = "Unknown mnemonic: " + mnem;
        return r;
    }();

    if (!encodedResult.error.empty()) return encodedResult;

    // Debug: show final encoded bytes for suspicious mnemonics
    if (mnem == "CBW" || mnem == "ADD" || mnem == "DEC" || mnem == "CALL") {
        std::cerr << "ASM_DEBUG_ENCODE: mnem=" << mnem << " encoded=[";
        for (uint8_t b : encodedResult.bytes) std::cerr << std::hex << (int)b << " ";
        std::cerr << std::dec << "]\n";
    }

    // Prepend prefixes
    result.bytes = prefixes;
    result.bytes.insert(result.bytes.end(), encodedResult.bytes.begin(), encodedResult.bytes.end());
    return result;
}

// ═══════════════════════════════════════════════════════════════════
//  Multi-line assembler
// ═══════════════════════════════════════════════════════════════════

AsmResult Assembler::assembleBlock(const std::string& text, uint32_t origin) const {
    AsmResult result;
    std::istringstream stream(text);
    std::string line;
    uint32_t addr = origin;
    int lineNum = 0;

    while (std::getline(stream, line)) {
        ++lineNum;
        // Also split on semicolons (as statement separator, not just comment)
        // We treat ';' as comment already, so each physical line is one statement.
        
        // Trim
        auto trimmed = line;
        while (!trimmed.empty() && std::isspace(static_cast<unsigned char>(trimmed.front())))
            trimmed.erase(trimmed.begin());
        while (!trimmed.empty() && std::isspace(static_cast<unsigned char>(trimmed.back())))
            trimmed.pop_back();
        if (trimmed.empty() || trimmed[0] == ';') continue;

        auto lr = assembleLine(trimmed, addr);
        if (!lr.error.empty()) {
            result.error = "Line " + std::to_string(lineNum) + ": " + lr.error + " (" + trimmed + ")";
            return result;
        }
        result.bytes.insert(result.bytes.end(), lr.bytes.begin(), lr.bytes.end());
        addr += static_cast<uint32_t>(lr.bytes.size());
    }
    return result;
}

} // namespace fador::cpu
