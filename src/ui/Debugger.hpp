#pragma once
#include <string>
#include <vector>
#include "../cpu/CPU.hpp"
#include "../cpu/InstructionDecoder.hpp"
#include "../cpu/Disassembler.hpp"
#include "../cpu/Assembler.hpp"
#include "../memory/MemoryBus.hpp"

namespace fador::ui {

class Debugger {
public:
    Debugger(cpu::CPU& cpu, memory::MemoryBus& memory, cpu::InstructionDecoder& decoder);
    ~Debugger() = default;

    // Starts the interactive debugger loop
    // Returns false if the user wants to quit the emulator entirely
    bool run();

private:
    cpu::CPU& m_cpu;
    memory::MemoryBus& m_memory;
    cpu::InstructionDecoder& m_decoder;
    cpu::Disassembler m_disasm;
    cpu::Assembler m_asm;

    void printRegisters();
    void dumpMemory(uint32_t address, uint32_t count);
    void assembleMode(uint32_t startAddr);

    std::vector<std::string> split(const std::string& s, char delimiter);

public:
    void disassemble(uint32_t address, uint32_t count);
    // Assemble one line and write to memory; returns bytes written (0 on error)
    uint32_t assembleAndWrite(const std::string& asmLine, uint32_t address);
    // Print registers and surrounding disassembly (used by --stop-after)
    void dumpState(uint32_t contextLines = 5);
};

} // namespace fador::ui
