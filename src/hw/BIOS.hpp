#pragma once
#include <array>
#include <cstdint>
#include <vector>
#include <string>
#include <functional>
#include "../cpu/CPU.hpp"
#include "../memory/MemoryBus.hpp"

namespace fador::memory { class HIMEM; }

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
    void setIdleCallback(std::function<void()> cb) { m_idleCallback = std::move(cb); }

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
        int16_t mickeysX = 0, mickeysY = 0; // Accumulated motion counters (mickeys)
        int16_t minX = 0, maxX = 639;       // Horizontal bounds (set via AH=07h)
        int16_t minY = 0, maxY = 199;       // Vertical bounds (set via AH=08h)
    };
    MouseState& mouseState() { return m_mouse; }

    // Set up IVT vectors and BDA defaults
    void initialize();

    // Check if an IVT entry still points to the BIOS-default handler.
    // Returns false when a program has hooked the vector via INT 21h/AH=25h.
    bool isOriginalIVT(uint8_t vector, uint16_t cs, uint32_t eip) const;

    // HLE callback stub constants – each vector gets a unique IRET stub
    // at F000:(HLE_STUB_BASE + vector) so we can detect program hooks.
    static constexpr uint16_t HLE_STUB_SEG  = 0xF000;
    static constexpr uint16_t HLE_STUB_BASE = 0x0100;

    // Load a raw disk image for floppy 0 (0x00)
    bool loadDiskImage(const std::string& path);

    // Set HIMEM (XMS) driver for INT 2Fh detection and XMS function dispatch
    void setHIMEM(memory::HIMEM* himem) { m_himem = himem; }

    // XMS entry point stub location in ROM (F000:XMS_ENTRY_OFFSET)
    // Code at this address: INT E0h; RETF
    static constexpr uint16_t XMS_ENTRY_OFFSET = 0x0040;

private:
    // Original IVT entries written during initialize().
    // Indexed by vector number; first = offset, second = segment.
    std::array<std::pair<uint16_t, uint16_t>, 256> m_originalIVT{};
    cpu::CPU& m_cpu;
    memory::MemoryBus& m_memory;
    KeyboardController& m_kbd;
    PIT8254& m_pit;


    // Simple Floppy emulation (1.44MB)
    std::vector<uint8_t> m_floppyData;
    bool m_floppyLoaded = false;

    std::function<void()> m_pollInput;
    std::function<void()> m_idleCallback;
    MouseState m_mouse;

    // INT 33h event handler callback
    uint16_t m_mouseCallbackMask = 0;
    uint16_t m_mouseCallbackSeg = 0;
    uint16_t m_mouseCallbackOff = 0;

    void handleVideoService();      // INT 10h
    void handleKeyboardService();   // INT 16h
    void handleMouseService();      // INT 33h
    void handleTimeService();       // INT 1Ah
    void handleDiskService();       // INT 13h
    void handleSystemService();     // INT 15h
    void handleXMSDispatch();       // INT E0h (XMS far-call entry)

    memory::HIMEM* m_himem = nullptr;
};

} // namespace fador::hw
