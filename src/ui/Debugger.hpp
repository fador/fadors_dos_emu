#pragma once
#include <string>
#include <vector>
#include "../cpu/CPU.hpp"
#include "../cpu/InstructionDecoder.hpp"
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

    void printRegisters();
    void dumpMemory(uint32_t address, uint32_t count);
    void disassemble(uint32_t address, uint32_t count);
    
    std::vector<std::string> split(const std::string& s, char delimiter);
};

} // namespace fador::ui
