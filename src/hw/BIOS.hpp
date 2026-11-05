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
class PIC8259;
class PIT8254;
class VGAController;

class BIOS {
public:
    BIOS(cpu::CPU& cpu, memory::MemoryBus& memory, KeyboardController& kbd, PIT8254& pit, PIC8259& pic);
    ~BIOS() = default;

    // Returns true if the interrupt was handled by HLE
    bool handleInterrupt(uint8_t vector);

    // Set VGA controller for mode-specific register programming
    void setVGA(VGAController* vga) { m_vga = vga; }

    // Send End-Of-Interrupt for a hardware interrupt vector.
    // Used when the D-bit guard suppresses thunk dispatch but BIOS has
    // no HLE handler — the EOI must still be sent so the PIC unblocks.
    void sendEOI(uint8_t vector);

    // Set slave PIC (for IRQ8-15 EOI through port A0h)
    void setSlavePIC(PIC8259* slave) { m_slavePic = slave; }

    // Ctrl+Alt+Del reset flag
    bool shouldReset() const { return m_resetRequested; }
    void clearReset() { m_resetRequested = false; }

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

    // VESA BIOS Extensions (VBE 2.0) state.  mode == 0 means a standard
    // VGA mode (selected via INT 10h AH=00h) is active.
    struct VbeState {
        uint16_t mode = 0;          // VBE mode number (without flag bits)
        uint16_t width = 0;
        uint16_t height = 0;
        uint8_t bpp = 0;            // 8, 15, 16 or 32
        uint32_t pitch = 0;         // bytes per scan line
        uint32_t startX = 0;        // display start (4F07h)
        uint32_t startY = 0;
        uint32_t bank = 0;          // window A position, 64 KiB granularity
        bool linear = false;        // LFB requested (bit 14 of the mode)
    };
    const VbeState& vbeState() const { return m_vbe; }

    // VBE mode table entry (also used by tests and renderers)
    struct VbeModeInfo {
        uint16_t mode;
        uint16_t width;
        uint16_t height;
        uint8_t bpp;
    };
    static const VbeModeInfo* findVbeMode(uint16_t mode);

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
    static constexpr uint16_t EMS_PAGE_FRAME_SEGMENT = 0xD000;
    static constexpr uint16_t EMS_PAGE_SIZE = 0x4000;
    static constexpr uint8_t EMS_PHYSICAL_PAGE_COUNT = 4;
    static constexpr uint16_t EMS_TOTAL_PAGES = 256;
    static constexpr uint16_t EMS_PRIVATE_API_OFFSET = 0x0070;
    static constexpr uint32_t EMS_IMPORT_RECORD_PHYS = 0xF1100;
    // VBE ROM data (OEM strings, mode list, window function) at F000:C000
    static constexpr uint16_t VBE_ROM_OFFSET = 0xC000;
    // Trap vectors used by the VBE 2.0 protected-mode interface (4F0Ah).
    // The PM code is "0F FF <vector>; RET", executed with a near CALL by
    // the application, so these vectors return without an IRET frame.
    static constexpr uint8_t VBE_PM_SET_WINDOW = 0xE5;
    static constexpr uint8_t VBE_PM_SET_DISPLAY_START = 0xE6;
    static constexpr uint8_t VBE_PM_SET_PALETTE = 0xE7;
    static bool isCallStyleTrap(uint8_t vector) {
        return vector >= VBE_PM_SET_WINDOW && vector <= VBE_PM_SET_PALETTE;
    }

private:
    // Original IVT entries written during initialize().
    // Indexed by vector number; first = offset, second = segment.
    std::array<std::pair<uint16_t, uint16_t>, 256> m_originalIVT{};
    cpu::CPU& m_cpu;
    memory::MemoryBus& m_memory;
    KeyboardController& m_kbd;
    PIT8254& m_pit;
    PIC8259& m_pic;
    PIC8259* m_slavePic = nullptr;
    bool m_resetRequested = false;


    // Simple Floppy emulation (1.44MB)
    std::vector<uint8_t> m_floppyData;
    bool m_floppyLoaded = false;

    std::function<void()> m_pollInput;
    std::function<void()> m_idleCallback;
    MouseState m_mouse;
    bool m_e0Flag = false;

    // INT 33h event handler callback
    uint16_t m_mouseCallbackMask = 0;
    uint16_t m_mouseCallbackSeg = 0;
    uint16_t m_mouseCallbackOff = 0;

    void handleVideoService();      // INT 10h
    void handleVbeService();        // INT 10h AH=4Fh (VESA BIOS Extensions)
    void handleVbeProtectedMode(uint8_t vector); // 4F0Ah PM entry points
    void initializeVbeRom();        // VBE info strings / mode list in ROM
    void resetVbeState();           // back to a standard VGA mode
    VbeState m_vbe;
    void handleKeyboardIRQ();       // INT 09h
    void handleKeyboardService();   // INT 16h
    static uint8_t scancodeToAscii(uint8_t scancode, uint8_t shiftFlags);
    void handleMouseService();      // INT 33h
    void handleTimeService();       // INT 1Ah
    void handleDiskService();       // INT 13h
    void handleSystemService();     // INT 15h
    void handleEMSService();        // INT 67h
    void handleXMSDispatch();       // INT E0h (XMS far-call entry)

    // Program VGA sequencer/GC registers to match the requested video mode
    void programVGARegisters(uint8_t mode);

    VGAController* m_vga = nullptr;

    struct EMSMapping {
        uint16_t handle = 0;
        uint16_t logicalPage = 0xFFFF;
    };
    struct EMSHandle {
        bool allocated = false;
        bool hasSavedMapping = false;
        std::vector<std::vector<uint8_t>> pages;
        std::array<EMSMapping, EMS_PHYSICAL_PAGE_COUNT> savedMappings{};
    };

    void initializeEMS();
    void flushEMSPhysicalPage(uint8_t physicalPage);
    void loadEMSPhysicalPage(uint8_t physicalPage);
    size_t countEMSAllocatedPages() const;
    void updateEMSImportRecord();

    memory::HIMEM* m_himem = nullptr;
    std::vector<EMSHandle> m_emsHandles;
    std::array<EMSMapping, EMS_PHYSICAL_PAGE_COUNT> m_emsMappings{};
};

} // namespace fador::hw
