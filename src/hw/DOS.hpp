#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include "../cpu/CPU.hpp"
#include "../memory/MemoryBus.hpp"
#include "../memory/himem/HIMEM.hpp"

namespace fador::hw {

class DOS {
public:
    DOS(cpu::CPU& cpu, memory::MemoryBus& memory);
    ~DOS() = default;

    // Returns true if the interrupt was handled by HLE
    bool handleInterrupt(uint8_t vector);

    // Initialization (PSP setup, etc.)
    void initialize();

    bool isTerminated() const { return m_terminated; }
    uint8_t getExitCode() const { return m_exitCode; }
    
    // VROOMM Overlay Support
    struct NESegment {
        uint16_t fileOffsetSector; // Offset in sectors from start of file
        uint16_t length;           // Length in bytes (0 = 64KB)
        uint16_t flags;
        uint16_t minAlloc;         // Minimum paragraphs to allocate
        uint16_t loadedSegment;    // Actual emulated segment, 0 if not loaded
    };
    void setNEInfo(const std::string& path, uint16_t alignShift, const std::vector<NESegment>& segments, uint16_t initialLoadSegment);

private:
    // HIMEM (XMS) support
    std::unique_ptr<memory::HIMEM> m_himem;
    cpu::CPU& m_cpu;
    memory::MemoryBus& m_memory;
    std::string m_currentDir = "."; // Emulated C:\ starting at current host dir
    uint8_t m_currentDrive = 2;     // 0=A, 1=B, 2=C... default to C:
    uint32_t m_dtaPtr = 0x00000000; // Pointer to DTA (segmented)
    bool m_terminated = false;
    uint8_t m_exitCode = 0;
    uint16_t m_pspSegment = 0x1000;
    // VROOMM State
    std::string m_programPath;
    uint16_t m_neAlignShift = 0;
    uint16_t m_neInitialLoadSegment = 0;
    std::vector<NESegment> m_neSegments;

    uint16_t loadOverlaySegment(uint16_t segIndex);

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
