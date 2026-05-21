#pragma once
#include <string>
#include <cstdint>
#include "../cpu/CPU.hpp"
#include "../memory/MemoryBus.hpp"

namespace fador::memory {
class HIMEM;
}

namespace fador::hw {
class DOS;

class ProgramLoader {
public:
    ProgramLoader(cpu::CPU& cpu, memory::MemoryBus& memory, memory::HIMEM* himem = nullptr);
    ~ProgramLoader() = default;

    // Loads a flat .COM file into memory at segment:0x100
    bool loadCOM(const std::string& path, uint16_t segment, DOS& dos, const std::string& args = "", uint16_t envSeg = 0);

    // Loads an .EXE file, parses MZ header, relocates, and sets initial CS:IP / SS:SP
    bool loadEXE(const std::string& path, uint16_t segment, DOS& dos, const std::string& args = "", bool useHimem = false, uint16_t envSeg = 0);

private:
    cpu::CPU& m_cpu;
    memory::MemoryBus& m_memory;
    memory::HIMEM* m_himem;

    // Creates a 256-byte Program Segment Prefix at the given segment
    void createPSP(uint16_t segment, const std::string& args, const std::string& programPath, uint16_t envSeg = 0);
};

} // namespace fador::hw
