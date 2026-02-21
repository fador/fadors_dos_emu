#pragma once
#include <string>
#include <cstdint>
#include "../cpu/CPU.hpp"
#include "../memory/MemoryBus.hpp"

namespace fador::hw {

class ProgramLoader {
public:
    ProgramLoader(cpu::CPU& cpu, memory::MemoryBus& memory);
    ~ProgramLoader() = default;

    // Loads a flat .COM file into memory at segment:0x100
    bool loadCOM(const std::string& path, uint16_t segment);

    // Loads an .EXE file, parses MZ header, relocates, and sets initial CS:IP / SS:SP
    bool loadEXE(const std::string& path, uint16_t segment);

private:
    cpu::CPU& m_cpu;
    memory::MemoryBus& m_memory;

    // Creates a 256-byte Program Segment Prefix at the given segment
    void createPSP(uint16_t segment);
};

} // namespace fador::hw
