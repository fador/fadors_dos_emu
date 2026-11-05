// VESA BIOS Extensions 2.0 (INT 10h AH=4Fh).
//
// Provides banked (A000h window, 64 KiB granularity) and linear framebuffer
// access to MemoryBus::VBE_VRAM_SIZE bytes of SVGA video memory located at
// physical MemoryBus::VBE_LFB_BASE.  Protected-mode programs map the linear
// framebuffer with DPMI 0800h; in this flat (non-paged) emulator the linear
// address equals the physical one.
#include "BIOS.hpp"
#include "VGAController.hpp"
#include "../utils/Logger.hpp"

#include <array>
#include <cstring>

namespace fador::hw {

namespace {

using memory::MemoryBus;

constexpr std::array<BIOS::VbeModeInfo, 20> kVbeModes = {{
    // mode    w     h    bpp
    {0x100, 640, 400, 8},
    {0x101, 640, 480, 8},
    {0x103, 800, 600, 8},
    {0x105, 1024, 768, 8},
    {0x10D, 320, 200, 15},
    {0x10E, 320, 200, 16},
    {0x10F, 320, 200, 32},
    {0x110, 640, 480, 15},
    {0x111, 640, 480, 16},
    {0x112, 640, 480, 32},
    {0x113, 800, 600, 15},
    {0x114, 800, 600, 16},
    {0x115, 800, 600, 32},
    {0x116, 1024, 768, 15},
    {0x117, 1024, 768, 16},
    {0x118, 1024, 768, 32},
    // Common non-standard low-resolution modes (as offered by e.g. DOSBox)
    {0x150, 320, 200, 8},
    {0x151, 320, 240, 8},
    {0x152, 320, 400, 8},
    {0x153, 320, 480, 8},
}};

constexpr uint16_t kVbeOk = 0x004F;
constexpr uint16_t kVbeFailed = 0x014F;
constexpr uint16_t kVbeInvalid = 0x034F; // invalid in current video mode

uint32_t bytesPerPixel(uint8_t bpp) { return (bpp + 7u) / 8u; }

// Offsets of the data written into the VBE ROM area (F000:VBE_ROM_OFFSET)
constexpr uint16_t kOemString = 0x00;     // "Fador VBE"
constexpr uint16_t kVendorName = 0x20;
constexpr uint16_t kProductName = 0x40;
constexpr uint16_t kProductRev = 0x60;
constexpr uint16_t kModeList = 0x80;      // word list, 0xFFFF terminated
constexpr uint16_t kWindowFunc = 0xE0;    // far-callable bank switch
constexpr uint16_t kPmTable = 0x100;      // 4F0Ah protected-mode table

} // namespace

const BIOS::VbeModeInfo *BIOS::findVbeMode(uint16_t mode) {
  mode &= 0x1FF;
  for (const auto &m : kVbeModes)
    if (m.mode == mode)
      return &m;
  return nullptr;
}

void BIOS::initializeVbeRom() {
  const uint32_t base = (static_cast<uint32_t>(HLE_STUB_SEG) << 4) + VBE_ROM_OFFSET;
  auto writeString = [&](uint16_t off, const char *text) {
    for (size_t i = 0; text[i]; ++i)
      m_memory.write8(base + off + static_cast<uint32_t>(i),
                      static_cast<uint8_t>(text[i]));
    m_memory.write8(base + off + static_cast<uint32_t>(std::strlen(text)), 0);
  };
  writeString(kOemString, "Fador SVGA VBE 2.0");
  writeString(kVendorName, "Fador");
  writeString(kProductName, "Fador DOS Emulator SVGA");
  writeString(kProductRev, "1.0");

  uint32_t p = base + kModeList;
  for (const auto &m : kVbeModes) {
    m_memory.write16(p, m.mode);
    p += 2;
  }
  m_memory.write16(p, 0xFFFF);

  // VBE 2.0 protected-mode interface table (returned by 4F0Ah):
  //   +0 offset of SetWindow, +2 SetDisplayStart, +4 SetPrimaryPalette,
  //   +6 offset of the I/O-port / memory list (both empty, FFFFh-terminated)
  // Each entry point is "0F FF <trap>; C3" so it works wherever the client
  // copies the table.
  {
    const uint32_t t = base + kPmTable;
    m_memory.write16(t + 0, 0x0010);
    m_memory.write16(t + 2, 0x0014);
    m_memory.write16(t + 4, 0x0018);
    m_memory.write16(t + 6, 0x000C);
    m_memory.write16(t + 0x0C, 0xFFFF); // port list terminator
    m_memory.write16(t + 0x0E, 0xFFFF); // memory list terminator
    const uint8_t traps[3] = {VBE_PM_SET_WINDOW, VBE_PM_SET_DISPLAY_START,
                              VBE_PM_SET_PALETTE};
    for (int i = 0; i < 3; ++i) {
      const uint32_t code = t + 0x10 + i * 4;
      m_memory.write8(code + 0, 0x0F);
      m_memory.write8(code + 1, 0xFF);
      m_memory.write8(code + 2, traps[i]);
      m_memory.write8(code + 3, 0xC3); // RET
    }
  }

  // Real-mode window function (ModeInfo.WinFuncPtr): programs FAR CALL it
  // with BX/DX as for 4F05h.  PUSH AX; MOV AX,4F05h; INT 10h; POP AX; RETF
  const uint8_t winFunc[] = {0x50, 0xB8, 0x05, 0x4F, 0xCD, 0x10, 0x58, 0xCB};
  for (size_t i = 0; i < sizeof(winFunc); ++i)
    m_memory.write8(base + kWindowFunc + static_cast<uint32_t>(i), winFunc[i]);
}

void BIOS::resetVbeState() {
  m_vbe = VbeState{};
  if (m_vga)
    m_vga->setSvgaWindow(false, 0);
}

void BIOS::handleVbeService() {
  const uint8_t al = m_cpu.getReg8(cpu::AL);
  // Real-mode buffer address ES:DI (DPMI clients reach us through
  // 0300h "simulate real-mode interrupt", so ES is a paragraph).
  const uint32_t esdi = (static_cast<uint32_t>(m_cpu.getSegReg(cpu::ES)) << 4) +
                        m_cpu.getReg16(cpu::DI);
  const uint32_t romSegOff = static_cast<uint32_t>(HLE_STUB_SEG) << 16;
  auto farRom = [&](uint16_t off) {
    return romSegOff | static_cast<uint32_t>(VBE_ROM_OFFSET + off);
  };

  switch (al) {
  case 0x00: { // Return VBE controller information
    const bool vbe2 = m_memory.read8(esdi + 0) == 'V' &&
                      m_memory.read8(esdi + 1) == 'B' &&
                      m_memory.read8(esdi + 2) == 'E' &&
                      m_memory.read8(esdi + 3) == '2';
    const uint32_t size = vbe2 ? 512 : 256;
    for (uint32_t i = 0; i < size; ++i)
      m_memory.write8(esdi + i, 0);
    m_memory.write8(esdi + 0, 'V');
    m_memory.write8(esdi + 1, 'E');
    m_memory.write8(esdi + 2, 'S');
    m_memory.write8(esdi + 3, 'A');
    m_memory.write16(esdi + 0x04, 0x0200);               // VBE 2.0
    m_memory.write32(esdi + 0x06, farRom(kOemString));   // OEM string
    m_memory.write32(esdi + 0x0A, 0x00000000);           // capabilities: 6-bit DAC
    m_memory.write32(esdi + 0x0E, farRom(kModeList));    // video mode list
    m_memory.write16(esdi + 0x12,
                     static_cast<uint16_t>(MemoryBus::VBE_VRAM_SIZE >> 16));
    if (vbe2) {
      m_memory.write16(esdi + 0x14, 0x0100);             // OEM software rev
      m_memory.write32(esdi + 0x16, farRom(kVendorName));
      m_memory.write32(esdi + 0x1A, farRom(kProductName));
      m_memory.write32(esdi + 0x1E, farRom(kProductRev));
    }
    m_cpu.setReg16(cpu::AX, kVbeOk);
    LOG_VIDEO("VBE 4F00h: controller info (", vbe2 ? "VBE2" : "VESA", ")");
    break;
  }
  case 0x01: { // Return VBE mode information
    const uint16_t modeNum = m_cpu.getReg16(cpu::CX) & 0x1FF;
    const VbeModeInfo *mi = findVbeMode(modeNum);
    if (!mi) {
      m_cpu.setReg16(cpu::AX, kVbeFailed);
      break;
    }
    const uint32_t bpp = bytesPerPixel(mi->bpp);
    const uint32_t pitch = mi->width * bpp;
    const uint32_t pageSize = pitch * mi->height;
    for (uint32_t i = 0; i < 256; ++i)
      m_memory.write8(esdi + i, 0);
    // supported | optional info | colour | graphics | LFB available
    m_memory.write16(esdi + 0x00, 0x009B);
    m_memory.write8(esdi + 0x02, 0x07);   // window A: exists, readable, writable
    m_memory.write8(esdi + 0x03, 0x00);   // window B: absent
    m_memory.write16(esdi + 0x04, 64);    // granularity (KiB)
    m_memory.write16(esdi + 0x06, 64);    // window size (KiB)
    m_memory.write16(esdi + 0x08, 0xA000);
    m_memory.write16(esdi + 0x0A, 0x0000);
    m_memory.write32(esdi + 0x0C, farRom(kWindowFunc));
    m_memory.write16(esdi + 0x10, static_cast<uint16_t>(pitch));
    m_memory.write16(esdi + 0x12, mi->width);
    m_memory.write16(esdi + 0x14, mi->height);
    m_memory.write8(esdi + 0x16, 8);      // char cell width
    m_memory.write8(esdi + 0x17, 16);     // char cell height
    m_memory.write8(esdi + 0x18, 1);      // planes
    m_memory.write8(esdi + 0x19, mi->bpp);
    m_memory.write8(esdi + 0x1A, 1);      // banks
    m_memory.write8(esdi + 0x1B, mi->bpp == 8 ? 4 : 6); // packed / direct
    m_memory.write8(esdi + 0x1C, 0);      // bank size
    uint32_t pages = pageSize ? MemoryBus::VBE_VRAM_SIZE / pageSize : 1;
    if (pages > 256)
      pages = 256;
    m_memory.write8(esdi + 0x1D, static_cast<uint8_t>(pages - 1));
    m_memory.write8(esdi + 0x1E, 1);      // reserved, must be 1
    if (mi->bpp == 15) {
      const uint8_t masks[8] = {5, 10, 5, 5, 5, 0, 1, 15};
      for (int i = 0; i < 8; ++i)
        m_memory.write8(esdi + 0x1F + i, masks[i]);
    } else if (mi->bpp == 16) {
      const uint8_t masks[8] = {5, 11, 6, 5, 5, 0, 0, 0};
      for (int i = 0; i < 8; ++i)
        m_memory.write8(esdi + 0x1F + i, masks[i]);
    } else if (mi->bpp == 32) {
      const uint8_t masks[8] = {8, 16, 8, 8, 8, 0, 8, 24};
      for (int i = 0; i < 8; ++i)
        m_memory.write8(esdi + 0x1F + i, masks[i]);
    }
    m_memory.write32(esdi + 0x28, MemoryBus::VBE_LFB_BASE); // PhysBasePtr
    m_memory.write32(esdi + 0x2C, 0);
    m_memory.write16(esdi + 0x30, 0);
    m_cpu.setReg16(cpu::AX, kVbeOk);
    break;
  }
  case 0x02: { // Set VBE mode
    const uint16_t bx = m_cpu.getReg16(cpu::BX);
    const uint16_t modeNum = bx & 0x1FF;
    if (modeNum < 0x100) {
      // Standard VGA mode through the VBE interface
      resetVbeState();
      m_cpu.setReg16(cpu::AX, static_cast<uint16_t>(
                                  (bx & 0x8000 ? 0x80 : 0x00) | modeNum));
      handleVideoService();
      m_cpu.setReg16(cpu::AX, kVbeOk);
      break;
    }
    const VbeModeInfo *mi = findVbeMode(modeNum);
    if (!mi) {
      m_cpu.setReg16(cpu::AX, kVbeFailed);
      break;
    }
    m_vbe = VbeState{};
    m_vbe.mode = modeNum;
    m_vbe.width = mi->width;
    m_vbe.height = mi->height;
    m_vbe.bpp = mi->bpp;
    m_vbe.pitch = mi->width * bytesPerPixel(mi->bpp);
    m_vbe.linear = (bx & 0x4000) != 0;
    if (!(bx & 0x8000)) {
      if (uint8_t *vram = m_memory.directAccess(MemoryBus::VBE_LFB_BASE))
        std::memset(vram, 0, MemoryBus::VBE_VRAM_SIZE);
    }
    if (m_vga)
      m_vga->setSvgaWindow(true, MemoryBus::VBE_LFB_BASE);
    // Keep the BDA consistent for software that inspects it: report a
    // graphics mode and a text geometry derived from the 8x16 font.
    m_memory.write8(0x449, 0x13);
    m_memory.write16(0x44A, static_cast<uint16_t>(mi->width / 8));
    m_memory.write8(0x484, static_cast<uint8_t>(mi->height / 16 - 1));
    m_cpu.setReg16(cpu::AX, kVbeOk);
    LOG_VIDEO("VBE 4F02h: mode 0x", std::hex, modeNum, std::dec, " ",
              mi->width, "x", mi->height, "x", (int)mi->bpp,
              m_vbe.linear ? " LFB" : " banked");
    break;
  }
  case 0x03: // Return current VBE mode
    m_cpu.setReg16(cpu::BX, m_vbe.mode ? static_cast<uint16_t>(
                                             m_vbe.mode |
                                             (m_vbe.linear ? 0x4000 : 0))
                                       : m_memory.read8(0x449));
    m_cpu.setReg16(cpu::AX, kVbeOk);
    break;
  case 0x04: { // Save/restore state
    const uint8_t dl = m_cpu.getReg8(cpu::DL);
    if (dl == 0x00)
      m_cpu.setReg16(cpu::BX, 1); // one 64-byte block is enough
    m_cpu.setReg16(cpu::AX, kVbeOk);
    break;
  }
  case 0x05: { // Display window control
    const uint8_t bh = m_cpu.getReg8(cpu::BH);
    const uint8_t bl = m_cpu.getReg8(cpu::BL);
    if (m_vbe.mode == 0 || bl != 0) {
      m_cpu.setReg16(cpu::AX, kVbeInvalid);
      break;
    }
    if (bh == 0x00) {
      uint32_t bank = m_cpu.getReg16(cpu::DX);
      if (static_cast<uint64_t>(bank) * 0x10000u >= MemoryBus::VBE_VRAM_SIZE) {
        m_cpu.setReg16(cpu::AX, kVbeFailed);
        break;
      }
      m_vbe.bank = bank;
      if (m_vga)
        m_vga->setSvgaWindow(true, MemoryBus::VBE_LFB_BASE + bank * 0x10000u);
    } else {
      m_cpu.setReg16(cpu::DX, static_cast<uint16_t>(m_vbe.bank));
    }
    m_cpu.setReg16(cpu::AX, kVbeOk);
    break;
  }
  case 0x06: { // Set/get logical scan line length
    if (m_vbe.mode == 0) {
      m_cpu.setReg16(cpu::AX, kVbeInvalid);
      break;
    }
    const uint8_t bl = m_cpu.getReg8(cpu::BL);
    const uint32_t bpp = bytesPerPixel(m_vbe.bpp);
    if (bl == 0x00 || bl == 0x02) {
      uint32_t req = m_cpu.getReg16(cpu::CX);
      uint32_t pitch = (bl == 0x00) ? req * bpp : req;
      pitch = (pitch + 3u) & ~3u;
      if (pitch < static_cast<uint32_t>(m_vbe.width) * bpp ||
          pitch * m_vbe.height > MemoryBus::VBE_VRAM_SIZE) {
        m_cpu.setReg16(cpu::AX, kVbeFailed);
        break;
      }
      m_vbe.pitch = pitch;
    }
    if (bl == 0x03) {
      uint32_t maxPitch = MemoryBus::VBE_VRAM_SIZE / m_vbe.height;
      if (maxPitch > 0xFFFF)
        maxPitch = 0xFFFF;
      m_cpu.setReg16(cpu::BX, static_cast<uint16_t>(maxPitch));
      m_cpu.setReg16(cpu::CX, static_cast<uint16_t>(maxPitch / bpp));
    } else {
      m_cpu.setReg16(cpu::BX, static_cast<uint16_t>(m_vbe.pitch));
      m_cpu.setReg16(cpu::CX, static_cast<uint16_t>(m_vbe.pitch / bpp));
      uint32_t lines = MemoryBus::VBE_VRAM_SIZE / m_vbe.pitch;
      m_cpu.setReg16(cpu::DX, static_cast<uint16_t>(lines > 0xFFFF ? 0xFFFF : lines));
    }
    m_cpu.setReg16(cpu::AX, kVbeOk);
    break;
  }
  case 0x07: { // Set/get display start
    if (m_vbe.mode == 0) {
      m_cpu.setReg16(cpu::AX, kVbeInvalid);
      break;
    }
    const uint8_t bl = m_cpu.getReg8(cpu::BL);
    if (bl == 0x00 || bl == 0x80) {
      m_vbe.startX = m_cpu.getReg16(cpu::CX);
      m_vbe.startY = m_cpu.getReg16(cpu::DX);
    } else if (bl == 0x01) {
      m_cpu.setReg8(cpu::BH, 0);
      m_cpu.setReg16(cpu::CX, static_cast<uint16_t>(m_vbe.startX));
      m_cpu.setReg16(cpu::DX, static_cast<uint16_t>(m_vbe.startY));
    } else {
      m_cpu.setReg16(cpu::AX, kVbeFailed);
      break;
    }
    m_cpu.setReg16(cpu::AX, kVbeOk);
    break;
  }
  case 0x08: { // DAC palette format: fixed 6 bits per primary
    const bool setRequest = m_cpu.getReg8(cpu::BL) == 0x00;
    const bool wants6 = m_cpu.getReg8(cpu::BH) == 6;
    m_cpu.setReg8(cpu::BH, 6);
    m_cpu.setReg16(cpu::AX, (setRequest && !wants6) ? kVbeFailed : kVbeOk);
    break;
  }
  case 0x09: { // Set/get palette data (B, G, R, pad per entry)
    const uint8_t bl = m_cpu.getReg8(cpu::BL) & 0x7F;
    const uint16_t count = m_cpu.getReg16(cpu::CX);
    const uint16_t first = m_cpu.getReg16(cpu::DX);
    uint8_t *pal = m_memory.dacPalette();
    for (uint32_t i = 0; i < count && first + i < 256; ++i) {
      const uint32_t entry = esdi + i * 4;
      uint8_t *rgb = pal + (first + i) * 3;
      if (bl == 0x00) {
        rgb[2] = m_memory.read8(entry + 0) & 0x3F;
        rgb[1] = m_memory.read8(entry + 1) & 0x3F;
        rgb[0] = m_memory.read8(entry + 2) & 0x3F;
      } else if (bl == 0x01) {
        m_memory.write8(entry + 0, rgb[2]);
        m_memory.write8(entry + 1, rgb[1]);
        m_memory.write8(entry + 2, rgb[0]);
        m_memory.write8(entry + 3, 0);
      }
    }
    m_cpu.setReg16(cpu::AX, (bl <= 0x01) ? kVbeOk : kVbeFailed);
    break;
  }
  case 0x0A: // Return VBE protected-mode interface
    if (m_cpu.getReg8(cpu::BL) != 0x00) {
      m_cpu.setReg16(cpu::AX, kVbeFailed);
      break;
    }
    m_cpu.setSegReg(cpu::ES, HLE_STUB_SEG);
    m_cpu.setReg16(cpu::DI, static_cast<uint16_t>(VBE_ROM_OFFSET + kPmTable));
    m_cpu.setReg16(cpu::CX, 0x001C); // table length
    m_cpu.setReg16(cpu::AX, kVbeOk);
    break;
  default:
    // 4F10h (power management),
    // 4F15h (DDC) and anything newer: report "function call failed" so
    // callers fall back to the real-mode/INT 10h paths.
    LOG_DEBUG("VBE: unsupported function AX=0x", std::hex,
              m_cpu.getReg16(cpu::AX));
    m_cpu.setReg16(cpu::AX, kVbeFailed);
    break;
  }
}

// VBE 2.0 protected-mode entry points.  Called with the client's PM
// registers; the display-start address is in plane units (bytes / 4).
void BIOS::handleVbeProtectedMode(uint8_t vector) {
  if (m_vbe.mode == 0)
    return;
  switch (vector) {
  case VBE_PM_SET_WINDOW: {
    if (m_cpu.getReg8(cpu::BL) != 0)
      return;
    uint32_t bank = m_cpu.getReg16(cpu::DX);
    if (static_cast<uint64_t>(bank) * 0x10000u >= MemoryBus::VBE_VRAM_SIZE)
      return;
    m_vbe.bank = bank;
    if (m_vga)
      m_vga->setSvgaWindow(true, MemoryBus::VBE_LFB_BASE + bank * 0x10000u);
    break;
  }
  case VBE_PM_SET_DISPLAY_START: {
    const uint32_t planeAddr = (static_cast<uint32_t>(m_cpu.getReg16(cpu::DX)) << 16) |
                               m_cpu.getReg16(cpu::CX);
    const uint32_t byteAddr = planeAddr * 4;
    if (m_vbe.pitch != 0) {
      const uint32_t bpp = bytesPerPixel(m_vbe.bpp);
      m_vbe.startY = byteAddr / m_vbe.pitch;
      m_vbe.startX = (byteAddr % m_vbe.pitch) / bpp;
    }
    break;
  }
  case VBE_PM_SET_PALETTE: {
    const uint16_t count = m_cpu.getReg16(cpu::CX);
    const uint16_t first = m_cpu.getReg16(cpu::DX);
    const uint32_t table = m_cpu.getSegBase(cpu::ES) +
                           (m_cpu.is32BitCode() ? m_cpu.getReg32(cpu::EDI)
                                                : m_cpu.getReg16(cpu::DI));
    uint8_t *pal = m_memory.dacPalette();
    for (uint32_t i = 0; i < count && first + i < 256; ++i) {
      uint8_t *rgb = pal + (first + i) * 3;
      rgb[2] = m_memory.read8(table + i * 4 + 0) & 0x3F;
      rgb[1] = m_memory.read8(table + i * 4 + 1) & 0x3F;
      rgb[0] = m_memory.read8(table + i * 4 + 2) & 0x3F;
    }
    break;
  }
  default:
    break;
  }
}

} // namespace fador::hw
