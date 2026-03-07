#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <optional>

namespace fador::cpu {

// Simple x86 real-mode assembler that converts a single mnemonic line into
// machine-code bytes.  Supports the subset of 8086/386 instructions
// implemented in InstructionDecoder.
//
// Usage:
//   Assembler asm;
//   auto result = asm.assemble("MOV AX, 1234h", 0x100);
//   // result->bytes contains the encoded instruction

struct AsmResult {
    std::vector<uint8_t> bytes;
    std::string error;       // Non-empty on failure
};

class Assembler {
public:
    // Assemble one instruction.  |origin| is the current CS:IP linear
    // address, needed to compute relative branch displacements.
    AsmResult assembleLine(const std::string& line, uint32_t origin) const;

    // Assemble multiple lines (separated by newline or semicolons).
    // Returns concatenated bytes on success or an error for the first
    // failing line.
    AsmResult assembleBlock(const std::string& text, uint32_t origin) const;

private:
    // ── Token types ──
    enum class TokKind { Reg8, Reg16, Reg32, SegReg, Imm, Mem, Label, Ident, Comma, Colon,
                         LBracket, RBracket, Plus, Minus, Star, SizeWord, SizeBytePtr,
                         SizeDwordPtr, Far, Short, Eol };
    struct Token {
        TokKind kind;
        std::string text;
        uint32_t value = 0;   // For Imm, Reg*, SegReg
    };

    // Tokeniser
    std::vector<Token> tokenize(const std::string& line) const;
    static bool isReg8(const std::string& s, uint8_t& idx);
    static bool isReg16(const std::string& s, uint8_t& idx);
    static bool isReg32(const std::string& s, uint8_t& idx);
    static bool isSegReg(const std::string& s, uint8_t& idx);

    // Operand representation
    enum class OpKind { None, Reg8, Reg16, Reg32, SegReg, Imm, Mem };
    struct Operand {
        OpKind kind = OpKind::None;
        uint8_t reg = 0;           // Register index
        uint32_t imm = 0;          // Immediate / displacement
        bool hasDisp = false;
        uint8_t baseReg = 0xFF;    // 0xFF = none
        uint8_t indexReg = 0xFF;   // 0xFF = none
        uint8_t sizeHint = 0;     // 0 = unspecified, 1 = byte, 2 = word, 4 = dword
        bool segOverride = false;
        uint8_t segReg = 0xFF;
    };
    bool parseOperand(const std::vector<Token>& toks, size_t& pos, Operand& out) const;

    // Encoding helpers
    uint8_t encodeModRM(const Operand& rm, uint8_t regField, std::vector<uint8_t>& out) const;
    void emitImm8(std::vector<uint8_t>& out, uint32_t val) const;
    void emitImm16(std::vector<uint8_t>& out, uint32_t val) const;
    void emitImm32(std::vector<uint8_t>& out, uint32_t val) const;

    // Pattern matchers for instruction encoding
    AsmResult encodeALU(const std::string& mnem, uint8_t aluOp,
                        const Operand& dst, const Operand& src, uint32_t origin) const;
    AsmResult encodeShift(const std::string& mnem, uint8_t subOp,
                          const Operand& dst, const Operand& src) const;
    AsmResult encodeJcc(uint8_t cc, const Operand& target, uint32_t origin) const;

    static std::string toUpper(const std::string& s);
};

} // namespace fador::cpu
