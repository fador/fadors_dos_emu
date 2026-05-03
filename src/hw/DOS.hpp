#pragma once
#include "../cpu/CPU.hpp"
#include "../memory/MemoryBus.hpp"
#include "../memory/himem/HIMEM.hpp"
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace fador::hw {

class KeyboardController;
class DPMI;

class DOS {
public:
  DOS(cpu::CPU &cpu, memory::MemoryBus &memory);
  ~DOS() = default;

  // Returns true if the interrupt was handled by HLE
  bool handleInterrupt(uint8_t vector);

  // Keyboard access for INT 21h console input functions
  void setKeyboard(KeyboardController &kbd) { m_kbd = &kbd; }
  void setInputPollCallback(std::function<void()> cb) {
    m_pollInput = std::move(cb);
  }
  void setIdleCallback(std::function<void()> cb) {
    m_idleCallback = std::move(cb);
  }

  // DPMI host wiring
  void setDPMI(DPMI *dpmi) { m_dpmi = dpmi; }

  // Access to HIMEM (XMS) for BIOS dispatch wiring
  memory::HIMEM *getHIMEM() { return m_himem.get(); }

  // Initialization (PSP setup, etc.)
  void initialize();

  bool isTerminated() const { return m_terminated; }
  uint8_t getExitCode() const { return m_exitCode; }
  uint16_t getPSPSegment() const { return m_pspSegment; }
  void setPSPSegment(uint16_t seg) { m_pspSegment = seg; }

  // VROOMM Overlay Support
  struct NESegment {
    uint16_t fileOffsetSector; // Offset in sectors from start of file
    uint16_t length;           // Length in bytes (0 = 64KB)
    uint16_t flags;
    uint16_t minAlloc;      // Minimum paragraphs to allocate
    uint16_t loadedSegment; // Actual emulated segment, 0 if not loaded
  };
  struct FBOVOverlay {
    uint16_t overlayId;     // Borland overlay identifier used by INT 3Fh
    uint32_t fileOffset;    // Absolute file offset of the FBOV block
    uint32_t fileSize;      // Size of the FBOV block in bytes
    uint32_t auxOffset;     // FBOV header auxiliary offset field
    uint32_t auxCount;      // FBOV header auxiliary count field
    uint16_t loadedSegment; // Actual emulated segment, 0 if not loaded
  };
  void clearOverlayInfo();
  void setNEInfo(const std::string &path, uint16_t alignShift,
                 const std::vector<NESegment> &segments,
                 uint16_t initialLoadSegment);
  void setFBOVInfo(const std::string &path,
                   const std::vector<FBOVOverlay> &overlays);

  // Set the working directory to the program's parent directory
  void setProgramDir(const std::string &programPath);

private:
  // HIMEM (XMS) support
  std::unique_ptr<memory::HIMEM> m_himem;
  DPMI *m_dpmi = nullptr;
  cpu::CPU &m_cpu;
  memory::MemoryBus &m_memory;
  std::string m_currentDir = "."; // Host filesystem working directory
  std::string m_hostRootDir =
      ".";                     // Host path that maps to DOS drive root (C:\)
  std::string m_dosCurrentDir; // DOS-style current dir relative to drive root
                               // (no leading \)
  uint8_t m_currentDrive = 2;  // 0=A, 1=B, 2=C... default to C:
  uint32_t m_dtaPtr = 0x00000000; // Pointer to DTA (segmented)
  bool m_terminated = false;
  uint8_t m_exitCode = 0;
  bool m_ctrlBreakCheck = false;
  uint16_t m_pspSegment = 0x1000;
  uint16_t m_allocationStrategy = 0; // Default: Best fit
  bool m_umbLinked = false;
  KeyboardController *m_kbd = nullptr;
  std::function<void()> m_pollInput;
  std::function<void()> m_idleCallback;
  // VROOMM State
  std::string m_programPath;
  uint16_t m_neAlignShift = 0;
  uint16_t m_neInitialLoadSegment = 0;
  std::vector<NESegment> m_neSegments;
  std::vector<FBOVOverlay> m_fbovOverlays;

  uint16_t loadOverlaySegment(uint16_t segIndex);
  uint16_t loadFBOVOverlay(size_t overlayIndex);

  // File handle emulation
  struct FileHandle {
    enum class Kind {
      File,
      EMSDevice,
    };

    std::string path;
    Kind kind = Kind::File;
    std::fstream stream;

    bool isEMSDevice() const { return kind == Kind::EMSDevice; }
  };
  std::vector<std::shared_ptr<FileHandle>> m_fileHandles;

  // Memory Control Block (MCB)
  struct MCB {
    uint8_t type;   // 'M' (0x4D) or 'Z' (0x5A)
    uint16_t owner; // 0x0000 if free, 0x0008 if system, or PSP segment
    uint16_t size;  // size in paragraphs (16 bytes)
    uint8_t reserved[3];
    char name[8];
  };
  static constexpr uint16_t FIRST_MCB_SEGMENT = 0x0800;
  static constexpr uint16_t LAST_PARA = 0x9FFF;

  void handleDOSService();                 // INT 21h
  void handleMemoryManagement();           // AH=48h, 49h, 4Ah
  void handleFileService();                // AH=3Ch-43h, 45h, 46h, 56h, 57h
  void handleDirectoryService();           // AH=39h, 3Ah, 3Bh, 47h
  void handleDriveService();               // AH=0Eh, 19h, 36h
  void handleDirectorySearch();            // AH=4Eh, 4Fh
  void terminateProcess(uint8_t exitCode); // AH=4Ch or INT 20h

  // FindFirst/FindNext search state
  std::vector<std::filesystem::path> m_searchResults;
  size_t m_searchIndex = 0;
  std::string m_searchPattern;
  std::string m_searchDir;

  // Helpers
  MCB readMCB(uint16_t segment);
  void writeMCB(uint16_t segment, const MCB &mcb);
  void dumpMCBChain();

  std::string readDOSString(uint32_t address); // Read '$' terminated string
  std::string readFilename(uint32_t address);  // Read null-terminated string
  void writeCharToVRAM(uint8_t c); // Teletype-style output to B800 VRAM
  std::string resolvePath(
      const std::string &path); // Resolve relative paths against m_currentDir
  std::string
  hostToDosPath(const std::string &hostPath); // Convert host path to DOS path
  std::string
  dosToHostPath(const std::string &dosPath); // Convert DOS path to host path
};

} // namespace fador::hw
