#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include "../memory/MemoryBus.hpp"

namespace fador::cpu {

struct DisassembledInstruction {
    uint32_t address;       // Linear address of the instruction
    uint32_t length;        // Number of bytes consumed
    std::string mnemonic;   // Full disassembly text (e.g. "MOV AX, 1234")
    std::string hexBytes;   // Raw bytes in hex (e.g. "B8 34 12")
};

// Read-only x86 real-mode disassembler.  Reads bytes from a MemoryBus
// and produces human-readable assembly text without modifying any state.
class Disassembler {
public:
    explicit Disassembler(const memory::MemoryBus& memory);

    // Disassemble a single instruction at the given linear address.
    DisassembledInstruction disassembleAt(uint32_t linearAddr) const;

    // Disassemble `count` sequential instructions starting at `linearAddr`.
    std::vector<DisassembledInstruction> disassembleRange(uint32_t linearAddr, uint32_t count) const;

    // Convenience: disassemble around an address (context before + after).
    // Returns instructions with `before` instructions before `center` and
    // `after` instructions after (including the one at `center`).
    std::vector<DisassembledInstruction> disassembleAround(uint32_t centerAddr,
                                                           uint32_t before,
                                                           uint32_t after) const;

private:
    const memory::MemoryBus& m_memory;

    // Internal cursor used during a single disassemble call
    struct Cursor {
        const memory::MemoryBus& mem;
        uint32_t pos;
        uint32_t start;

        uint8_t  read8();
        uint16_t read16();
        uint32_t read32();
        uint32_t bytesConsumed() const { return pos - start; }
    };

    static const char* reg8Name(uint8_t idx);
    static const char* reg16Name(uint8_t idx);
    static const char* reg32Name(uint8_t idx);
    static const char* segRegName(uint8_t idx);
    static const char* ccName(uint8_t cc);

    struct ModRM { uint8_t mod, reg, rm; };
    struct SIB   { uint8_t scale, index, base; };

    static ModRM decodeModRM(uint8_t b);
    static SIB   decodeSIB(uint8_t b);

    // Format a ModR/M operand as a string.
    std::string formatModRM16(Cursor& c, const ModRM& m, int opSize, uint8_t segOverride) const;
    std::string formatModRM32(Cursor& c, const ModRM& m, int opSize, uint8_t segOverride) const;
    std::string formatRM(Cursor& c, const ModRM& m, int opSize, bool addr32, uint8_t segOverride) const;

    static std::string hexStr8(uint8_t v);
    static std::string hexStr16(uint16_t v);
    static std::string hexStr32(uint32_t v);

    // Top-level single-byte opcode disassembly
    std::string disasmOpcode(Cursor& c, uint8_t opcode, bool prefix66, bool prefix67,
                             bool repz, bool repnz, uint8_t segOverride) const;
    std::string disasmOpcode0F(Cursor& c, uint8_t opcode, bool prefix66, bool prefix67,
                               uint8_t segOverride) const;
};

} // namespace fador::cpu
