#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <functional>
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

    // Set a callback that polls host input; called by INT 16h when blocking.
    void setInputPollCallback(std::function<void()> cb) { m_pollInput = std::move(cb); }

    // Mouse state (updated by InputManager, read by INT 33h)
    struct MouseState {
        int16_t x = 0, y = 0;       // Virtual screen coordinates (pixel units)
        uint16_t buttons = 0;        // Bit 0=left, 1=right, 2=middle
        bool installed = false;
        bool visible = false;
        uint16_t pressCount[3] = {};
        uint16_t releaseCount[3] = {};
        int16_t lastPressX[3] = {}, lastPressY[3] = {};
        int16_t lastReleaseX[3] = {}, lastReleaseY[3] = {};
    };
    MouseState& mouseState() { return m_mouse; }

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

    std::function<void()> m_pollInput;
    MouseState m_mouse;

    void handleVideoService();      // INT 10h
    void handleKeyboardService();   // INT 16h
    void handleMouseService();      // INT 33h
    void handleTimeService();       // INT 1Ah
    void handleDiskService();       // INT 13h
};

} // namespace fador::hw
