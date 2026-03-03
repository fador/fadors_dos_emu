#pragma once
#include <cstdint>
#include <array>

namespace fador::hw {

// Video memory layout types
enum class VMemLayout : uint8_t {
    Text,         // Character/attribute pairs at B8000h
    CGA2,         // 1bpp CGA interleaved at B8000h (640 wide)
    CGA4,         // 2bpp CGA interleaved at B8000h (320 wide)
    Planar,       // 4-plane EGA/VGA at A0000h
    Linear256     // 1 byte/pixel at A0000h (mode 13h)
};

struct VideoModeInfo {
    uint8_t  mode;
    uint16_t width;       // pixels (graphics) or columns (text)
    uint16_t height;      // pixels (graphics) or rows (text)
    uint8_t  bpp;         // bits per pixel (text: 0)
    uint16_t colors;      // max simultaneous colors (text: 16)
    uint8_t  textCols;    // terminal columns (text modes)
    uint8_t  textRows;    // terminal rows (text modes)
    uint32_t vramBase;    // start of framebuffer
    uint32_t pageSize;    // bytes per page
    VMemLayout layout;
    bool     isGraphics;
};

// All standard DOS video modes
inline constexpr std::array<VideoModeInfo, 15> kVideoModes = {{
    //mode  w     h   bpp col tcol trow vramBase   pgSize          layout          gfx
    {0x00, 360, 400,  0, 16, 40, 25, 0xB8000, 40*25*2,   VMemLayout::Text,      false},
    {0x01, 360, 400,  0, 16, 40, 25, 0xB8000, 40*25*2,   VMemLayout::Text,      false},
    {0x02, 720, 400,  0, 16, 80, 25, 0xB8000, 80*25*2,   VMemLayout::Text,      false},
    {0x03, 720, 400,  0, 16, 80, 25, 0xB8000, 80*25*2,   VMemLayout::Text,      false},
    {0x04, 320, 200,  2,  4, 40, 25, 0xB8000, 16384,     VMemLayout::CGA4,      true },
    {0x05, 320, 200,  2,  4, 40, 25, 0xB8000, 16384,     VMemLayout::CGA4,      true },
    {0x06, 640, 200,  1,  2, 80, 25, 0xB8000, 16384,     VMemLayout::CGA2,      true },
    {0x07, 720, 400,  0, 16, 80, 25, 0xB0000, 80*25*2,   VMemLayout::Text,      false},
    {0x0D, 320, 200,  4, 16, 40, 25, 0xA0000, 32000,     VMemLayout::Planar,    true },
    {0x0E, 640, 200,  4, 16, 80, 25, 0xA0000, 64000,     VMemLayout::Planar,    true },
    {0x0F, 640, 350,  2,  4, 80, 25, 0xA0000, 28000,     VMemLayout::Planar,    true },
    {0x10, 640, 350,  4, 16, 80, 25, 0xA0000, 112000,    VMemLayout::Planar,    true },
    {0x11, 640, 480,  1,  2, 80, 30, 0xA0000, 38400,     VMemLayout::Planar,    true },
    {0x12, 640, 480,  4, 16, 80, 30, 0xA0000, 153600,    VMemLayout::Planar,    true },
    {0x13, 320, 200,  8,256, 40, 25, 0xA0000, 64000,     VMemLayout::Linear256, true },
}};

inline const VideoModeInfo* findVideoMode(uint8_t mode) {
    for (auto& m : kVideoModes) {
        if (m.mode == mode) return &m;
    }
    return nullptr;
}

} // namespace fador::hw
