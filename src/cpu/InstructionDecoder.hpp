#pragma once

#include <cstdint>
#include <tuple>
#include "../memory/MemoryBus.hpp"
#include "CPU.hpp"

namespace fador::cpu {

struct ModRM {
    uint8_t mod;
    uint8_t reg;
    uint8_t rm;
};

struct SIB {
    uint8_t scale;
    uint8_t index;
    uint8_t base;
};

// Provides helper functions for fetching CPU instructions and processing prefixes
class InstructionDecoder {
public:
    InstructionDecoder(CPU& cpu, memory::MemoryBus& memory);

    // Main execution loop single step
    void step();

private:
    CPU& m_cpu;
    memory::MemoryBus& m_memory;

    // Prefixes state for current instruction
    bool m_hasPrefix66; // Operand size override
    bool m_hasPrefix67; // Address size override
    bool m_hasRepnz;
    bool m_hasRepz;
    uint8_t m_segmentOverride; // SegRegIndex or 0xFF for none

    // Fetches next byte from memory through CS:EIP
    uint8_t fetch8();
    uint16_t fetch16();
    uint32_t fetch32();

    ModRM decodeModRM(uint8_t byte);
    SIB decodeSIB(uint8_t byte);

    // Calculates effective address based on ModRM and SIB
    uint32_t getEffectiveAddress16(const ModRM& modrm);
    uint32_t getEffectiveAddress32(const ModRM& modrm);

    // Helpers to resolve operand values
    uint32_t readModRM32(const ModRM& modrm);
    uint16_t readModRM16(const ModRM& modrm);
    uint8_t  readModRM8(const ModRM& modrm);

    void writeModRM32(const ModRM& modrm, uint32_t value);
    void writeModRM16(const ModRM& modrm, uint16_t value);
    void writeModRM8(const ModRM& modrm, uint8_t value);

    // Instruction Handlers
    void executeOpcode(uint8_t opcode);
    void executeOpcode0F(uint8_t opcode); // Two-byte opcodes
    
    // Interrupt and Exception handling
    void triggerInterrupt(uint8_t vector);
};

} // namespace fador::cpu
