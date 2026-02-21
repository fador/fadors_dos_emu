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
    std::string m_currentDir = "."; // Emulated C:\ starting at current host dir
    uint8_t m_currentDrive = 2;     // 0=A, 1=B, 2=C... default to C:
    uint32_t m_dtaPtr = 0x00000000; // Pointer to DTA (segmented)

    // File handle emulation
    struct FileHandle {
        std::string path;
        std::fstream stream;
    };
    std::vector<std::unique_ptr<FileHandle>> m_fileHandles;
    
    // Memory Control Block (MCB)
    struct MCB {
        uint8_t type;     // 'M' (0x4D) or 'Z' (0x5A)
        uint16_t owner;   // 0x0000 if free, 0x0008 if system, or PSP segment
        uint16_t size;    // size in paragraphs (16 bytes)
        uint8_t reserved[3];
        char name[8];
    };
    static constexpr uint16_t FIRST_MCB_SEGMENT = 0x0700;
    static constexpr uint16_t LAST_PARA = 0x9FFF;

    void handleDOSService(); // INT 21h
    void handleMemoryManagement(); // AH=48h, 49h, 4Ah
    void handleDirectoryService(); // AH=39h, 3Ah, 3Bh, 47h
    void handleDriveService();     // AH=0Eh, 19h, 36h
    void handleDirectorySearch();  // AH=4Eh, 4Fh
    void terminateProcess(uint8_t exitCode); // AH=4Ch or INT 20h

    // Helpers
    MCB readMCB(uint16_t segment);
    void writeMCB(uint16_t segment, const MCB& mcb);
    void dumpMCBChain();

    std::string readDOSString(uint32_t address); // Read '$' terminated string
    std::string readFilename(uint32_t address);  // Read null-terminated string
};

} // namespace fador::hw
