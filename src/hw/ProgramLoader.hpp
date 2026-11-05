#pragma once
#include <string>
#include <cstdint>
#include <utility>
#include <vector>
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

    // Environment variables given to directly launched programs (and the
    // shell) in addition to PATH/LIB/INCLUDE, in order.  Defaults:
    //   BLASTER   the emulated Sound Blaster's settings
    //   CAUSEWAY  DPMI: CauseWay would otherwise switch to raw mode and
    //             use ring transitions this emulator does not model
    // setEnvironmentVariable() replaces or adds one (--env=NAME=VALUE).
    using EnvList = std::vector<std::pair<std::string, std::string>>;
    static EnvList& defaultEnvironment();
    static void setEnvironmentVariable(const std::string& name, const std::string& value);

private:
    cpu::CPU& m_cpu;
    memory::MemoryBus& m_memory;
    memory::HIMEM* m_himem;

    // Creates a 256-byte Program Segment Prefix at the given segment, returns drive validities for FCB1/2
    std::pair<bool, bool> createPSP(uint16_t segment, const std::string& args, const std::string& programPath, uint16_t envSeg = 0);
};

} // namespace fador::hw
