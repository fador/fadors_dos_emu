#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include "../cpu/CPU.hpp"
#include "../memory/MemoryBus.hpp"

namespace fador::hw {

class KeyboardController;
class PIT8254;

class BIOS {
public:
    BIOS(cpu::CPU& cpu, memory::MemoryBus& memory, KeyboardController& kbd, PIT8254& pit);
    ~BIOS() = default;

    // Returns true if the interrupt was handled by HLE
    bool handleInterrupt(uint8_t vector);

    // Set up IVT vectors and BDA defaults
    void initialize();

    // Load a raw disk image for floppy 0 (0x00)
    bool loadDiskImage(const std::string& path);

private:
    cpu::CPU& m_cpu;
    memory::MemoryBus& m_memory;
    KeyboardController& m_kbd;
    PIT8254& m_pit;

    // Simple Floppy emulation (1.44MB)
    std::vector<uint8_t> m_floppyData;
    bool m_floppyLoaded = false;

    void handleVideoService();      // INT 10h
    void handleKeyboardService();   // INT 16h
    void handleTimeService();       // INT 1Ah
    void handleDiskService();       // INT 13h
};

} // namespace fador::hw
