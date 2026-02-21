#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include "../cpu/CPU.hpp"
#include "../memory/MemoryBus.hpp"

namespace fador::hw {

class DOS {
public:
    DOS(cpu::CPU& cpu, memory::MemoryBus& memory);
    ~DOS() = default;

    // Returns true if the interrupt was handled by HLE
    bool handleInterrupt(uint8_t vector);

    // Initialization (PSP setup, etc.)
    void initialize();

private:
    cpu::CPU& m_cpu;
    memory::MemoryBus& m_memory;

    // File handle emulation
    struct FileHandle {
        std::string path;
        std::fstream stream;
    };
    std::vector<std::unique_ptr<FileHandle>> m_fileHandles;

    void handleDOSService(); // INT 21h
    void terminateProcess(uint8_t exitCode); // AH=4Ch or INT 20h

    // Helpers
    std::string readDOSString(uint32_t address); // Read '$' terminated string
    std::string readFilename(uint32_t address);  // Read null-terminated string
};

} // namespace fador::hw
