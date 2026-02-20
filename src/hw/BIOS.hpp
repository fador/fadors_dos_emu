#pragma once
#include <cstdint>
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

private:
    cpu::CPU& m_cpu;
    memory::MemoryBus& m_memory;
    KeyboardController& m_kbd;
    PIT8254& m_pit;

    void handleVideoService();      // INT 10h
    void handleKeyboardService();   // INT 16h
    void handleTimeService();       // INT 1Ah
    void handleDiskService();       // INT 13h
};

} // namespace fador::hw
