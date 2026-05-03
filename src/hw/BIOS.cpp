#include "BIOS.hpp"
#include "../memory/himem/HIMEM.hpp"
#include "../utils/Logger.hpp"
#include "KeyboardController.hpp"
#include "PIC8259.hpp"
#include "PIT8254.hpp"
#include "VideoMode.hpp"
#include <algorithm>
#include <cstring>
#include <fstream>
#include <thread>

namespace fador::hw {

BIOS::BIOS(cpu::CPU &cpu, memory::MemoryBus &memory, KeyboardController &kbd,
           PIT8254 &pit, PIC8259 &pic)
    : m_cpu(cpu), m_memory(memory), m_kbd(kbd), m_pit(pit), m_pic(pic) {}

bool BIOS::loadDiskImage(const std::string &path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    LOG_ERROR("BIOS: Failed to open disk image: ", path);
    return false;
  }

  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);

  m_floppyData.resize(static_cast<size_t>(size));
  if (file.read(reinterpret_cast<char *>(m_floppyData.data()), size)) {
    m_floppyLoaded = true;
    LOG_INFO("BIOS: Loaded floppy image ", path, " (", size, " bytes)");
    return true;
  }
  return false;
}

void BIOS::sendEOI(uint8_t vector) {
  // Vectors 0x08-0x0F are master PIC (IRQ 0-7)
  // Vectors 0x70-0x77 are slave PIC (IRQ 8-15)
  if (vector >= 0x70 && vector <= 0x77) {
    m_pic.write8(0xA0, 0x20); // slave EOI
  }
  if ((vector >= 0x08 && vector <= 0x0F) ||
      (vector >= 0x70 && vector <= 0x77)) {
    m_pic.write8(0x20, 0x20); // master EOI
  }
}

bool BIOS::handleInterrupt(uint8_t vector) {
  switch (vector) {
  case 0x10:
    handleVideoService();
    return true;
  case 0x13:
    handleDiskService();
    return true;
  case 0x16:
    handleKeyboardService();
    return true;
  case 0x1A:
    handleTimeService();
    return true;
  // HLE stubs for common BIOS/system interrupts
  case 0x11:                         // Equipment List
    m_cpu.setReg16(cpu::AX, 0x002F); // Typical: FDD, video, RAM, etc.
    LOG_DEBUG("BIOS INT 11h: Equipment List (stubbed)");
    return true;
  case 0x12:                      // Memory Size
    m_cpu.setReg16(cpu::AX, 640); // 640KB conventional
    LOG_DEBUG("BIOS INT 12h: Memory Size (stubbed)");
    return true;
  case 0x14:                   // Serial Port
    m_cpu.setReg8(cpu::AH, 0); // No error
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    LOG_DEBUG("BIOS INT 14h: Serial Port (stubbed)");
    return true;
  case 0x15: // System Services
    handleSystemService();
    return true;
  case 0x17:                   // Printer
    m_cpu.setReg8(cpu::AH, 0); // No error
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    LOG_DEBUG("BIOS INT 17h: Printer (stubbed)");
    return true;
  case 0x2F: { // Multiplex
    uint16_t ax = m_cpu.getReg16(cpu::AX);
    if (ax == 0x4300) {
      // XMS installation check
      if (m_himem) {
        m_cpu.setReg8(cpu::AL, 0x80); // XMS driver installed
        LOG_DEBUG("INT 2Fh AX=4300h: XMS installed");
      } else {
        m_cpu.setReg8(cpu::AL, 0x00); // Not installed
        LOG_DEBUG("INT 2Fh AX=4300h: XMS not installed");
      }
    } else if (ax == 0x4310) {
      // Get XMS driver entry point
      if (m_himem) {
        m_cpu.setSegReg(cpu::ES, HLE_STUB_SEG);
        m_cpu.setReg16(cpu::BX, XMS_ENTRY_OFFSET);
        LOG_DEBUG("INT 2Fh AX=4310h: XMS entry at ", std::hex, HLE_STUB_SEG,
                  ":", XMS_ENTRY_OFFSET);
      } else {
        m_cpu.setReg8(cpu::AL, 0x00);
      }
    } else if (ax == 0x1600) {
      // Enhanced Windows installation check
      m_cpu.setReg8(cpu::AL, 0x00); // Neither Windows 3.x nor Windows 9x
      LOG_DEBUG("INT 2Fh AX=1600h: Windows check (stubbed)");
    } else if (ax == 0x1680) {
      // Release time slice
      m_cpu.setReg8(cpu::AL, 0x00);
      LOG_DEBUG("INT 2Fh AX=1680h: Release time slice");
    } else if (ax == 0x1686) {
      // Get CPU mode
      m_cpu.setReg16(cpu::AX, 0x0000); // Real mode
      LOG_DEBUG("INT 2Fh AX=1686h: Get CPU Mode -> Real Mode");
    } else {
      LOG_DEBUG("BIOS: INT 2Fh Multiplex (stubbed), AX=0x", std::hex, ax);
      m_cpu.setReg8(cpu::AL, 0x00); // Not supported
    }
    return true;
  }
  case 0x08: { // IRQ0 – Timer Tick (HLE path, matches machine code at F000:0200)
    uint32_t lo = m_memory.read16(0x46C);
    lo++;
    m_memory.write16(0x46C, static_cast<uint16_t>(lo & 0xFFFF));
    if ((lo & 0xFFFF) == 0)
      m_memory.write16(0x46E, m_memory.read16(0x46E) + 1);
    // INT 1Ch (user timer hook) is a no-op stub in our HLE.
    // Send EOI to master PIC so the ISR bit is cleared and future
    // timer ticks can be delivered.
    m_pic.write8(0x20, 0x20);
    return true;
  }
  case 0x1B: // Ctrl-Break
  case 0x1C: // Timer Tick
  case 0x1D: // Video Parameters
  case 0x1E: // Diskette Params
  case 0x1F: // Graphics Char Table
    LOG_DEBUG("BIOS INT ", std::hex, (int)vector, ": System stub (no-op)");
    return true;
  case 0x28: // DOS Idle
    return true;
  case 0x33: // Mouse driver
    handleMouseService();
    return true;
  case 0xE0: // XMS far-call dispatch (our internal HLE vector)
    handleXMSDispatch();
    return true;
  case 0x67: // EMS / EMM386
    handleEMSService();
    return true;
  }
  return false;
}

void BIOS::handleMouseService() {
  uint16_t ax = m_cpu.getReg16(cpu::AX);
  LOG_DEBUG("INT 33h: AX=0x", std::hex, ax);
  switch (ax) {
  case 0x0000: // Reset / detect
    m_mouse.installed = true;
    m_mouse.x = 0;
    m_mouse.y = 0;
    m_mouse.buttons = 0;
    m_mouse.visible = false;
    m_mouse.mickeysX = 0;
    m_mouse.mickeysY = 0;
    m_mouse.minX = 0;
    m_mouse.maxX = 639;
    m_mouse.minY = 0;
    m_mouse.maxY = 199;
    for (int i = 0; i < 3; i++) {
      m_mouse.pressCount[i] = 0;
      m_mouse.releaseCount[i] = 0;
    }
    m_cpu.setReg16(cpu::AX, 0xFFFF); // Installed
    m_cpu.setReg16(cpu::BX, 0x0002); // 2 buttons
    m_mouseCallbackMask = 0;
    m_mouseCallbackSeg = 0;
    m_mouseCallbackOff = 0;
    LOG_DEBUG("INT 33h: Mouse reset, driver installed");
    break;
  case 0x0001: // Show cursor
    m_mouse.visible = true;
    break;
  case 0x0002: // Hide cursor
    m_mouse.visible = false;
    break;
  case 0x0003: // Get position and button status
    m_cpu.setReg16(cpu::BX, m_mouse.buttons);
    m_cpu.setReg16(cpu::CX, static_cast<uint16_t>(m_mouse.x));
    m_cpu.setReg16(cpu::DX, static_cast<uint16_t>(m_mouse.y));
    break;
  case 0x0004: // Set position
    m_mouse.x = static_cast<int16_t>(m_cpu.getReg16(cpu::CX));
    m_mouse.y = static_cast<int16_t>(m_cpu.getReg16(cpu::DX));
    if (m_mouse.x < m_mouse.minX)
      m_mouse.x = m_mouse.minX;
    if (m_mouse.x > m_mouse.maxX)
      m_mouse.x = m_mouse.maxX;
    if (m_mouse.y < m_mouse.minY)
      m_mouse.y = m_mouse.minY;
    if (m_mouse.y > m_mouse.maxY)
      m_mouse.y = m_mouse.maxY;
    break;
  case 0x0005: { // Get button press info
    uint16_t btn = m_cpu.getReg16(cpu::BX);
    if (btn > 2)
      btn = 0;
    m_cpu.setReg16(cpu::AX, m_mouse.buttons);
    m_cpu.setReg16(cpu::BX, m_mouse.pressCount[btn]);
    m_cpu.setReg16(cpu::CX, static_cast<uint16_t>(m_mouse.lastPressX[btn]));
    m_cpu.setReg16(cpu::DX, static_cast<uint16_t>(m_mouse.lastPressY[btn]));
    m_mouse.pressCount[btn] = 0;
    break;
  }
  case 0x0006: { // Get button release info
    uint16_t btn = m_cpu.getReg16(cpu::BX);
    if (btn > 2)
      btn = 0;
    m_cpu.setReg16(cpu::AX, m_mouse.buttons);
    m_cpu.setReg16(cpu::BX, m_mouse.releaseCount[btn]);
    m_cpu.setReg16(cpu::CX, static_cast<uint16_t>(m_mouse.lastReleaseX[btn]));
    m_cpu.setReg16(cpu::DX, static_cast<uint16_t>(m_mouse.lastReleaseY[btn]));
    m_mouse.releaseCount[btn] = 0;
    break;
  }
  case 0x0007: // Set horizontal bounds
    m_mouse.minX = static_cast<int16_t>(m_cpu.getReg16(cpu::CX));
    m_mouse.maxX = static_cast<int16_t>(m_cpu.getReg16(cpu::DX));
    LOG_DEBUG("INT 33h: Set horizontal bounds ", m_mouse.minX, "-",
              m_mouse.maxX);
    break;
  case 0x0008: // Set vertical bounds
    m_mouse.minY = static_cast<int16_t>(m_cpu.getReg16(cpu::CX));
    m_mouse.maxY = static_cast<int16_t>(m_cpu.getReg16(cpu::DX));
    LOG_DEBUG("INT 33h: Set vertical bounds ", m_mouse.minY, "-", m_mouse.maxY);
    break;
  case 0x000A: // Set text cursor
    break;
  case 0x000B: // Read motion counters
    m_cpu.setReg16(cpu::CX, static_cast<uint16_t>(m_mouse.mickeysX));
    m_cpu.setReg16(cpu::DX, static_cast<uint16_t>(m_mouse.mickeysY));
    m_mouse.mickeysX = 0;
    m_mouse.mickeysY = 0;
    break;
  case 0x000C: { // Set event handler
    m_mouseCallbackMask = m_cpu.getReg16(cpu::CX);
    m_mouseCallbackSeg = m_cpu.getSegReg(cpu::ES);
    m_mouseCallbackOff = m_cpu.getReg16(cpu::DX);
    LOG_DEBUG("INT 33h: Set event handler mask=0x", std::hex,
              m_mouseCallbackMask, " addr=", m_mouseCallbackSeg, ":",
              m_mouseCallbackOff);
    break;
  }
  case 0x0014: { // Swap event handlers
    uint16_t oldMask = m_mouseCallbackMask;
    uint16_t oldSeg = m_mouseCallbackSeg;
    uint16_t oldOff = m_mouseCallbackOff;
    m_mouseCallbackMask = m_cpu.getReg16(cpu::CX);
    m_mouseCallbackSeg = m_cpu.getSegReg(cpu::ES);
    m_mouseCallbackOff = m_cpu.getReg16(cpu::DX);
    m_cpu.setReg16(cpu::CX, oldMask);
    m_cpu.setSegReg(cpu::ES, oldSeg);
    m_cpu.setReg16(cpu::DX, oldOff);
    break;
  }
  case 0x000F: // Set mickey/pixel ratio
  case 0x0010: // Set exclusive area
  case 0x0013: // Set double-speed threshold
    break;
  case 0x0015: // Get driver storage size
    m_cpu.setReg16(cpu::BX, 64);
    break;
  case 0x001A: // Set sensitivity
  case 0x001B: // Get sensitivity
    if (ax == 0x001B) {
      m_cpu.setReg16(cpu::BX, 50); // horizontal
      m_cpu.setReg16(cpu::CX, 50); // vertical
      m_cpu.setReg16(cpu::DX, 50); // threshold
    }
    break;
  case 0x0021: // Software reset
    m_mouse.installed = true;
    m_cpu.setReg16(cpu::AX, 0xFFFF); // Installed
    m_cpu.setReg16(cpu::BX, 0x0002); // 2 buttons
    break;
  default:
    LOG_DEBUG("INT 33h: Unhandled function AX=0x", std::hex, ax);
    break;
  }
}

void BIOS::handleVideoService() {
  uint8_t ah = m_cpu.getReg8(cpu::AH);
  switch (ah) {
  case 0x00: {                                  // Set Video Mode
    uint8_t al = m_cpu.getReg8(cpu::AL) & 0x7F; // bit 7 = don't clear if set
    bool clearScreen = !(m_cpu.getReg8(cpu::AL) & 0x80);
    const auto *mi = findVideoMode(al);
    m_memory.write8(0x449, al); // BDA: current mode
    m_memory.write8(0x462, 0);  // BDA: active page = 0
    m_memory.write16(0x44E, 0); // BDA: page offset = 0
    // Reset cursor for all 8 pages
    for (int p = 0; p < 8; ++p) {
      m_memory.write8(0x450 + p * 2, 0);     // col
      m_memory.write8(0x450 + p * 2 + 1, 0); // row
    }
    if (mi) {
      m_memory.write16(0x44A, mi->textCols);
      m_memory.write8(0x484, mi->textRows - 1);
      m_memory.write16(0x44C, static_cast<uint16_t>(mi->pageSize));
      if (clearScreen) {
        if (mi->isGraphics) {
          std::memset(m_memory.directAccess(mi->vramBase), 0, mi->pageSize);
        } else {
          uint32_t cells = static_cast<uint32_t>(mi->textCols) * mi->textRows;
          for (uint32_t i = 0; i < cells * 2; i += 2) {
            m_memory.write8(mi->vramBase + i, ' ');
            m_memory.write8(mi->vramBase + i + 1, 0x07);
          }
        }
      }
    } else {
      // Unknown mode — default to 80x25 text
      m_memory.write16(0x44A, 80);
      m_memory.write8(0x484, 24);
      m_memory.write16(0x44C, 80 * 25 * 2);
    }
    LOG_VIDEO("BIOS INT 10h: Set Video Mode 0x", std::hex, (int)al);
    break;
  }
  case 0x0F: {                                     // Get Video Mode
    m_cpu.setReg8(cpu::AL, m_memory.read8(0x449)); // Current mode
    m_cpu.setReg8(cpu::AH,
                  static_cast<uint8_t>(m_memory.read16(0x44A))); // Columns
    m_cpu.setReg8(cpu::BH, m_memory.read8(0x462));               // Active page
    break;
  }
  case 0x01: { // Set Cursor Type
    m_memory.write16(0x460, m_cpu.getReg16(cpu::CX));
    LOG_DEBUG("BIOS INT 10h: Set Cursor Type CX=", m_cpu.getReg16(cpu::CX));
    break;
  }
  case 0x02: { // Set Cursor Position
    uint8_t page = m_cpu.getReg8(cpu::BH);
    uint8_t dh = m_cpu.getReg8(cpu::DH); // Row
    uint8_t dl = m_cpu.getReg8(cpu::DL); // Col
    if (page > 7)
      page = 0;
    m_memory.write8(0x450 + page * 2, dl);
    m_memory.write8(0x450 + page * 2 + 1, dh);
    LOG_DEBUG("BIOS INT 10h: Set Cursor Position page=", (int)page, " to ",
              (int)dh, ":", (int)dl);
    break;
  }
  case 0x03: { // Get Cursor Position/Type
    uint8_t page = m_cpu.getReg8(cpu::BH);
    if (page > 7)
      page = 0;
    m_cpu.setReg8(cpu::DL, m_memory.read8(0x450 + page * 2));
    m_cpu.setReg8(cpu::DH, m_memory.read8(0x450 + page * 2 + 1));
    m_cpu.setReg16(cpu::CX, m_memory.read16(0x460));
    break;
  }
  case 0x04: {                 // Read Light Pen Position
    m_cpu.setReg8(cpu::AH, 0); // Not triggered
    LOG_DEBUG("BIOS INT 10h: Read Light Pen (stubbed)");
    break;
  }
  case 0x05: { // Set Active Page
    uint8_t page = m_cpu.getReg8(cpu::AL);
    if (page > 7)
      page = 0;
    m_memory.write8(0x462, page);
    m_memory.write16(0x44E,
                     static_cast<uint16_t>(page * m_memory.read16(0x44C)));
    LOG_DEBUG("BIOS INT 10h: Set Active Page ", (int)page);
    break;
  }
  case 0x06:   // Scroll Up
  case 0x07: { // Scroll Down
    uint8_t al = m_cpu.getReg8(cpu::AL);
    uint8_t row1 = m_cpu.getReg8(cpu::CH);
    uint8_t row2 = m_cpu.getReg8(cpu::DH);
    uint8_t col1 = m_cpu.getReg8(cpu::CL);
    uint8_t col2 = m_cpu.getReg8(cpu::DL);
    uint8_t attr = m_cpu.getReg8(cpu::BH);
    uint16_t cols = m_memory.read16(0x44A);
    uint8_t maxRow = m_memory.read8(0x484);
    if (row2 > maxRow)
      row2 = maxRow;
    if (col2 >= cols)
      col2 = static_cast<uint8_t>(cols - 1);
    uint8_t lines = al;
    if (lines == 0 || lines > (row2 - row1 + 1)) {
      for (uint8_t r = row1; r <= row2; ++r) {
        for (uint8_t c = col1; c <= col2; ++c) {
          uint32_t off = (r * cols + c) * 2;
          m_memory.write8(0xB8000 + off, ' ');
          m_memory.write8(0xB8000 + off + 1, attr);
        }
      }
    } else if (ah == 0x06) {
      for (uint8_t r = row1; r <= row2 - lines; ++r) {
        for (uint8_t c = col1; c <= col2; ++c) {
          uint32_t dst = (r * cols + c) * 2;
          uint32_t src = ((r + lines) * cols + c) * 2;
          m_memory.write8(0xB8000 + dst, m_memory.read8(0xB8000 + src));
          m_memory.write8(0xB8000 + dst + 1, m_memory.read8(0xB8000 + src + 1));
        }
      }
      for (uint8_t r = row2 - lines + 1; r <= row2; ++r) {
        for (uint8_t c = col1; c <= col2; ++c) {
          uint32_t off = (r * cols + c) * 2;
          m_memory.write8(0xB8000 + off, ' ');
          m_memory.write8(0xB8000 + off + 1, attr);
        }
      }
    } else {
      for (uint8_t r = row2; r >= row1 + lines; --r) {
        for (uint8_t c = col1; c <= col2; ++c) {
          uint32_t dst = (r * cols + c) * 2;
          uint32_t src = ((r - lines) * cols + c) * 2;
          m_memory.write8(0xB8000 + dst, m_memory.read8(0xB8000 + src));
          m_memory.write8(0xB8000 + dst + 1, m_memory.read8(0xB8000 + src + 1));
        }
      }
      for (uint8_t r = row1; r < row1 + lines; ++r) {
        for (uint8_t c = col1; c <= col2; ++c) {
          uint32_t off = (r * cols + c) * 2;
          m_memory.write8(0xB8000 + off, ' ');
          m_memory.write8(0xB8000 + off + 1, attr);
        }
      }
    }
    LOG_VIDEO("BIOS INT 10h: Scroll ", (ah == 0x06 ? "Up" : "Down"),
              " AL=", (int)al, " Window=", (int)row1, ",", (int)col1, "-",
              (int)row2, ",", (int)col2);
    break;
  }
  case 0x08: { // Read Character and Attribute at Cursor
    uint8_t page = m_cpu.getReg8(cpu::BH);
    if (page > 7)
      page = 0;
    uint16_t cols = m_memory.read16(0x44A);
    uint8_t col = m_memory.read8(0x450 + page * 2);
    uint8_t row = m_memory.read8(0x450 + page * 2 + 1);
    uint32_t off = (row * cols + col) * 2;
    m_cpu.setReg8(cpu::AL, m_memory.read8(0xB8000 + off));
    m_cpu.setReg8(cpu::AH, m_memory.read8(0xB8000 + off + 1));
    break;
  }
  case 0x09: { // Write Character and Attribute at Cursor
    uint8_t c = m_cpu.getReg8(cpu::AL);
    uint8_t attr = m_cpu.getReg8(cpu::BL);
    uint16_t count = m_cpu.getReg16(cpu::CX);
    uint16_t cols = m_memory.read16(0x44A);
    uint8_t maxRow = m_memory.read8(0x484);
    uint8_t cursorCol = m_memory.read8(0x450);
    uint8_t cursorRow = m_memory.read8(0x451);
    for (uint16_t i = 0; i < count; ++i) {
      uint32_t off = (cursorRow * cols + cursorCol) * 2;
      m_memory.write8(0xB8000 + off, c);
      m_memory.write8(0xB8000 + off + 1, attr);
      cursorCol++;
      if (cursorCol >= cols) {
        cursorCol = 0;
        cursorRow++;
      }
      if (cursorRow > maxRow) {
        cursorRow = maxRow;
      }
    }
    // AH=09h does NOT advance the cursor in the BDA
    LOG_VIDEO("BIOS INT 10h AH=09h: Write Char '", (char)c, "' attr=", std::hex,
              (int)attr, " count=", count);
    break;
  }
  case 0x0A: { // Write Character Only at Cursor (no attribute change)
    uint8_t c = m_cpu.getReg8(cpu::AL);
    uint16_t count = m_cpu.getReg16(cpu::CX);
    uint16_t cols = m_memory.read16(0x44A);
    uint8_t maxRow = m_memory.read8(0x484);
    uint8_t cursorCol = m_memory.read8(0x450);
    uint8_t cursorRow = m_memory.read8(0x451);
    for (uint16_t i = 0; i < count; ++i) {
      uint32_t off = (cursorRow * cols + cursorCol) * 2;
      m_memory.write8(0xB8000 + off, c);
      // attribute byte left unchanged
      cursorCol++;
      if (cursorCol >= cols) {
        cursorCol = 0;
        cursorRow++;
      }
      if (cursorRow > maxRow) {
        cursorRow = maxRow;
      }
    }
    LOG_VIDEO("BIOS INT 10h AH=0Ah: Write Char Only '", (char)c,
              "' count=", count);
    break;
  }
  case 0x0B: { // Set Color Palette / Background/Border
    uint8_t bh = m_cpu.getReg8(cpu::BH);
    uint8_t bl = m_cpu.getReg8(cpu::BL);
    LOG_DEBUG("BIOS INT 10h AH=0Bh: Set Palette BH=", (int)bh, " BL=", (int)bl,
              " (stubbed)");
    break;
  }
  case 0x0C: { // Write Graphics Pixel
    uint8_t mode = m_memory.read8(0x449);
    const auto *mi = findVideoMode(mode);
    if (mi && mi->isGraphics) {
      uint8_t color = m_cpu.getReg8(cpu::AL);
      uint16_t x = m_cpu.getReg16(cpu::CX);
      uint16_t y = m_cpu.getReg16(cpu::DX);
      if (x < mi->width && y < mi->height) {
        switch (mi->layout) {
        case VMemLayout::Linear256:
          m_memory.write8(mi->vramBase + y * mi->width + x, color);
          break;
        case VMemLayout::CGA4: {
          uint32_t bank = (y & 1) ? 0x2000 : 0;
          uint32_t off = bank + (y >> 1) * 80 + (x >> 2);
          uint8_t shift = (3 - (x & 3)) * 2;
          uint8_t b = m_memory.read8(mi->vramBase + off);
          b &= ~(0x03 << shift);
          b |= (color & 0x03) << shift;
          m_memory.write8(mi->vramBase + off, b);
          break;
        }
        case VMemLayout::CGA2: {
          uint32_t bank = (y & 1) ? 0x2000 : 0;
          uint32_t off = bank + (y >> 1) * 80 + (x >> 3);
          uint8_t bit = 7 - (x & 7);
          uint8_t b = m_memory.read8(mi->vramBase + off);
          if (color & 1)
            b |= (1 << bit);
          else
            b &= ~(1 << bit);
          m_memory.write8(mi->vramBase + off, b);
          break;
        }
        case VMemLayout::Planar: {
          // Write pixel across 4 planes stored sequentially
          uint32_t byteOff = (y * mi->width + x) / 8;
          uint8_t bit = 7 - (x & 7);
          uint32_t planeSize =
              (static_cast<uint32_t>(mi->width) * mi->height) / 8;
          for (int p = 0; p < 4; ++p) {
            uint8_t b = m_memory.read8(mi->vramBase + p * planeSize + byteOff);
            if (color & (1 << p))
              b |= (1 << bit);
            else
              b &= ~(1 << bit);
            m_memory.write8(mi->vramBase + p * planeSize + byteOff, b);
          }
          break;
        }
        default:
          break;
        }
      }
    }
    LOG_TRACE("BIOS INT 10h AH=0Ch: Write Pixel");
    break;
  }
  case 0x0D: { // Read Graphics Pixel
    uint8_t mode = m_memory.read8(0x449);
    const auto *mi = findVideoMode(mode);
    m_cpu.setReg8(cpu::AL, 0);
    if (mi && mi->isGraphics) {
      uint16_t x = m_cpu.getReg16(cpu::CX);
      uint16_t y = m_cpu.getReg16(cpu::DX);
      if (x < mi->width && y < mi->height) {
        switch (mi->layout) {
        case VMemLayout::Linear256:
          m_cpu.setReg8(cpu::AL,
                        m_memory.read8(mi->vramBase + y * mi->width + x));
          break;
        case VMemLayout::CGA4: {
          uint32_t bank = (y & 1) ? 0x2000 : 0;
          uint32_t off = bank + (y >> 1) * 80 + (x >> 2);
          uint8_t shift = (3 - (x & 3)) * 2;
          m_cpu.setReg8(cpu::AL,
                        (m_memory.read8(mi->vramBase + off) >> shift) & 0x03);
          break;
        }
        case VMemLayout::CGA2: {
          uint32_t bank = (y & 1) ? 0x2000 : 0;
          uint32_t off = bank + (y >> 1) * 80 + (x >> 3);
          uint8_t bit = 7 - (x & 7);
          m_cpu.setReg8(cpu::AL,
                        (m_memory.read8(mi->vramBase + off) >> bit) & 1);
          break;
        }
        case VMemLayout::Planar: {
          uint32_t byteOff = (y * mi->width + x) / 8;
          uint8_t bit = 7 - (x & 7);
          uint32_t planeSize =
              (static_cast<uint32_t>(mi->width) * mi->height) / 8;
          uint8_t pixel = 0;
          for (int p = 0; p < 4; ++p) {
            if (m_memory.read8(mi->vramBase + p * planeSize + byteOff) &
                (1 << bit))
              pixel |= (1 << p);
          }
          m_cpu.setReg8(cpu::AL, pixel);
          break;
        }
        default:
          break;
        }
      }
    }
    LOG_TRACE("BIOS INT 10h AH=0Dh: Read Pixel");
    break;
  }
  case 0x0E: { // Teletype Output
    uint8_t al = m_cpu.getReg8(cpu::AL);
    uint8_t page = m_cpu.getReg8(cpu::BH);
    if (page > 7)
      page = 0;
    uint16_t cols = m_memory.read16(0x44A);
    uint8_t maxRow = m_memory.read8(0x484);
    uint8_t cursorCol = m_memory.read8(0x450 + page * 2);
    uint8_t cursorRow = m_memory.read8(0x450 + page * 2 + 1);

    switch (al) {
    case 0x07: // BEL
      break;
    case 0x08: // Backspace
      if (cursorCol > 0)
        cursorCol--;
      break;
    case 0x0A: // Line Feed
      cursorRow++;
      break;
    case 0x0D: // Carriage Return
      cursorCol = 0;
      break;
    default: { // Printable character
      uint32_t off = (cursorRow * cols + cursorCol) * 2;
      m_memory.write8(0xB8000 + off, al);
      // Keep existing attribute
      if (m_memory.read8(0xB8000 + off + 1) == 0)
        m_memory.write8(0xB8000 + off + 1, 0x07);
      cursorCol++;
      if (cursorCol >= cols) {
        cursorCol = 0;
        cursorRow++;
      }
      break;
    }
    }
    // Scroll up if past last row
    if (cursorRow > maxRow) {
      // Scroll entire screen up one line
      for (uint8_t r = 0; r < maxRow; ++r) {
        for (uint16_t c = 0; c < cols; ++c) {
          uint32_t dst = (r * cols + c) * 2;
          uint32_t src = ((r + 1) * cols + c) * 2;
          m_memory.write8(0xB8000 + dst, m_memory.read8(0xB8000 + src));
          m_memory.write8(0xB8000 + dst + 1, m_memory.read8(0xB8000 + src + 1));
        }
      }
      // Clear last line
      for (uint16_t c = 0; c < cols; ++c) {
        uint32_t off = (maxRow * cols + c) * 2;
        m_memory.write8(0xB8000 + off, ' ');
        m_memory.write8(0xB8000 + off + 1, 0x07);
      }
      cursorRow = maxRow;
    }
    m_memory.write8(0x450 + page * 2, cursorCol);
    m_memory.write8(0x450 + page * 2 + 1, cursorRow);
    break;
  }
  case 0x10: { // DAC/Palette Functions
    uint8_t al = m_cpu.getReg8(cpu::AL);
    constexpr uint32_t PAL = 0xE0000;
    switch (al) {
    case 0x00: // Set Individual Palette Register
      LOG_DEBUG("BIOS INT 10h AH=10h AL=00h: Set Palette Reg BL=",
                (int)m_cpu.getReg8(cpu::BL), " (stubbed)");
      break;
    case 0x01: // Set Overscan (Border) Color
      LOG_DEBUG("BIOS INT 10h AH=10h AL=01h: Set Border Color (stubbed)");
      break;
    case 0x02: // Set All Palette Registers
      LOG_DEBUG("BIOS INT 10h AH=10h AL=02h: Set All Palette (stubbed)");
      break;
    case 0x03: // Toggle Intensity/Blinking
      LOG_DEBUG("BIOS INT 10h AH=10h AL=03h: Toggle Blink BL=",
                (int)m_cpu.getReg8(cpu::BL), " (stubbed)");
      break;
    case 0x07: // Read Individual Palette Register
      m_cpu.setReg8(cpu::BH, m_cpu.getReg8(cpu::BL)); // Echo back
      break;
    case 0x08:                      // Read Overscan Register
      m_cpu.setReg8(cpu::BH, 0x00); // Black border
      break;
    case 0x10: { // Set Individual DAC Register
      uint16_t reg = m_cpu.getReg16(cpu::BX);
      if (reg < 256) {
        m_memory.write8(PAL + reg * 3 + 0, m_cpu.getReg8(cpu::DH) & 0x3F);
        m_memory.write8(PAL + reg * 3 + 1, m_cpu.getReg8(cpu::CH) & 0x3F);
        m_memory.write8(PAL + reg * 3 + 2, m_cpu.getReg8(cpu::CL) & 0x3F);
      }
      LOG_DEBUG("BIOS INT 10h AH=10h AL=10h: Set DAC Reg ", reg);
      break;
    }
    case 0x12: { // Set Block of DAC Registers
      uint16_t first = m_cpu.getReg16(cpu::BX);
      uint16_t count = m_cpu.getReg16(cpu::CX);
      uint16_t es = m_cpu.getSegReg(cpu::ES);
      uint16_t dx = m_cpu.getReg16(cpu::DX);
      uint32_t addr = (es << 4) + dx;
      for (uint16_t i = 0; i < count && (first + i) < 256; ++i) {
        m_memory.write8(PAL + (first + i) * 3 + 0,
                        m_memory.read8(addr + i * 3 + 0) & 0x3F);
        m_memory.write8(PAL + (first + i) * 3 + 1,
                        m_memory.read8(addr + i * 3 + 1) & 0x3F);
        m_memory.write8(PAL + (first + i) * 3 + 2,
                        m_memory.read8(addr + i * 3 + 2) & 0x3F);
      }
      LOG_DEBUG("BIOS INT 10h AH=10h AL=12h: Set DAC Block first=", first,
                " count=", count);
      break;
    }
    case 0x15: { // Read Individual DAC Register
      uint16_t reg = m_cpu.getReg16(cpu::BX);
      if (reg < 256) {
        m_cpu.setReg8(cpu::DH, m_memory.read8(PAL + reg * 3 + 0));
        m_cpu.setReg8(cpu::CH, m_memory.read8(PAL + reg * 3 + 1));
        m_cpu.setReg8(cpu::CL, m_memory.read8(PAL + reg * 3 + 2));
      }
      break;
    }
    case 0x17: { // Read Block of DAC Registers
      uint16_t first = m_cpu.getReg16(cpu::BX);
      uint16_t count = m_cpu.getReg16(cpu::CX);
      uint16_t es = m_cpu.getSegReg(cpu::ES);
      uint16_t dx = m_cpu.getReg16(cpu::DX);
      uint32_t addr = (es << 4) + dx;
      for (uint16_t i = 0; i < count && (first + i) < 256; ++i) {
        m_memory.write8(addr + i * 3 + 0,
                        m_memory.read8(PAL + (first + i) * 3 + 0));
        m_memory.write8(addr + i * 3 + 1,
                        m_memory.read8(PAL + (first + i) * 3 + 1));
        m_memory.write8(addr + i * 3 + 2,
                        m_memory.read8(PAL + (first + i) * 3 + 2));
      }
      LOG_DEBUG("BIOS INT 10h AH=10h AL=17h: Read DAC Block first=", first,
                " count=", count);
      break;
    }
    case 0x1A:                   // Get Color Page State
      m_cpu.setReg8(cpu::BL, 0); // Paging mode 0
      m_cpu.setReg8(cpu::BH, 0); // Current page 0
      break;
    default:
      LOG_DEBUG("BIOS INT 10h AH=10h: Palette subfn AL=", std::hex, (int)al,
                " (stubbed)");
      break;
    }
    break;
  }
  case 0x11: { // Character Generator / Font Services
    uint8_t al = m_cpu.getReg8(cpu::AL);
    switch (al) {
    case 0x30: { // Get Font Information
      uint8_t bh = m_cpu.getReg8(cpu::BH);
      // Return standard 8x16 font info
      m_cpu.setReg16(cpu::CX, 16); // Bytes per character (height)
      m_cpu.setReg8(cpu::DL, 24);  // Rows - 1
      // ES:BP -> font table (point to a dummy area)
      m_cpu.setSegReg(cpu::ES, 0xC000);
      m_cpu.setReg16(cpu::BP, 0x0000);
      LOG_DEBUG("BIOS INT 10h AH=11h AL=30h: Get Font Info pointer=", (int)bh);
      break;
    }
    default:
      LOG_DEBUG("BIOS INT 10h AH=11h: Font subfn AL=", std::hex, (int)al,
                " (stubbed)");
      break;
    }
    break;
  }
  case 0x12: { // Alternate Function Select (Video Subsystem Configuration)
    uint8_t bl = m_cpu.getReg8(cpu::BL);
    switch (bl) {
    case 0x10: {                    // Get EGA Info
      m_cpu.setReg8(cpu::BH, 0x00); // Color mode
      m_cpu.setReg8(cpu::BL, 0x03); // 256K EGA memory
      m_cpu.setReg8(cpu::CH, 0x00); // Feature bits
      m_cpu.setReg8(cpu::CL, 0x09); // Switch settings
      LOG_DEBUG("BIOS INT 10h AH=12h BL=10h: Get EGA Info");
      break;
    }
    case 0x30: { // Set Vertical Resolution
      uint8_t al = m_cpu.getReg8(cpu::AL);
      m_cpu.setReg8(cpu::AL, 0x12); // Function supported
      LOG_DEBUG("BIOS INT 10h AH=12h BL=30h: Set VertRes AL=", (int)al,
                " (stubbed)");
      break;
    }
    case 0x34: { // Cursor Emulation
      m_cpu.setReg8(cpu::AL, 0x12);
      LOG_DEBUG("BIOS INT 10h AH=12h BL=34h: Cursor Emulation (stubbed)");
      break;
    }
    case 0x36: { // Video Refresh Control
      m_cpu.setReg8(cpu::AL, 0x12);
      LOG_DEBUG("BIOS INT 10h AH=12h BL=36h: Video Refresh (stubbed)");
      break;
    }
    default:
      LOG_DEBUG("BIOS INT 10h AH=12h: Alt Select BL=", std::hex, (int)bl,
                " (stubbed)");
      m_cpu.setReg8(cpu::AL, 0x12);
      break;
    }
    break;
  }
  case 0x13: {                           // Write String
    uint8_t al = m_cpu.getReg8(cpu::AL); // Write mode
    uint8_t page = m_cpu.getReg8(cpu::BH);
    uint8_t attr = m_cpu.getReg8(cpu::BL);
    uint16_t count = m_cpu.getReg16(cpu::CX);
    uint8_t row = m_cpu.getReg8(cpu::DH);
    uint8_t col = m_cpu.getReg8(cpu::DL);
    uint16_t es = m_cpu.getSegReg(cpu::ES);
    uint16_t bp = m_cpu.getReg16(cpu::BP);
    uint32_t strAddr = (es << 4) + bp;
    uint16_t cols = m_memory.read16(0x44A);
    uint8_t maxRow = m_memory.read8(0x484);

    if (page > 7)
      page = 0;

    for (uint16_t i = 0; i < count; ++i) {
      uint8_t c;
      uint8_t charAttr = attr;
      if (al & 0x02) {
        // Mode 2/3: string contains char, attr pairs
        c = m_memory.read8(strAddr++);
        charAttr = m_memory.read8(strAddr++);
      } else {
        // Mode 0/1: string is chars only, use BL for attribute
        c = m_memory.read8(strAddr++);
      }

      // Handle control characters
      if (c == 0x0D) {
        col = 0;
        continue;
      }
      if (c == 0x0A) {
        row++;
        continue;
      }
      if (c == 0x08) {
        if (col > 0)
          col--;
        continue;
      }
      if (c == 0x07) {
        continue;
      } // BEL

      uint32_t off = (row * cols + col) * 2;
      m_memory.write8(0xB8000 + off, c);
      m_memory.write8(0xB8000 + off + 1, charAttr);
      col++;
      if (col >= cols) {
        col = 0;
        row++;
      }
      if (row > maxRow) {
        row = maxRow;
      }
    }

    // Modes 1 and 3 update the cursor position
    if (al & 0x01) {
      m_memory.write8(0x450 + page * 2, col);
      m_memory.write8(0x450 + page * 2 + 1, row);
    }
    LOG_VIDEO("BIOS INT 10h AH=13h: Write String mode=", (int)al,
              " count=", count);
    break;
  }
  case 0x1A: { // Get/Set Display Combination Code
    uint8_t al = m_cpu.getReg8(cpu::AL);
    if (al == 0x00) {
      // Get: report VGA color
      m_cpu.setReg8(cpu::AL, 0x1A); // Function supported
      m_cpu.setReg8(cpu::BL, 0x08); // Active: VGA w/ color analog
      m_cpu.setReg8(cpu::BH, 0x00); // Alternate: none
    } else {
      m_cpu.setReg8(cpu::AL, 0x1A);
    }
    LOG_DEBUG("BIOS INT 10h AH=1Ah: Display Combo Code AL=", (int)al);
    break;
  }
  case 0x1B: { // Functionality/State Information
    // ES:DI -> 64-byte buffer to fill
    uint16_t es = m_cpu.getSegReg(cpu::ES);
    uint16_t di = m_cpu.getReg16(cpu::DI);
    uint32_t addr = (es << 4) + di;
    uint16_t cols = m_memory.read16(0x44A);
    uint8_t rows = m_memory.read8(0x484);
    // Zero the buffer
    for (int i = 0; i < 64; ++i)
      m_memory.write8(addr + i, 0);
    // Fill key fields
    m_memory.write8(addr + 0x00, m_memory.read8(0x449)); // Current mode
    m_memory.write16(addr + 0x01, cols);
    m_memory.write16(addr + 0x22, m_memory.read16(0x44C)); // Page size
    m_memory.write8(addr + 0x27, rows);                    // Rows - 1
    m_memory.write16(addr + 0x28, 16);                     // Char height
    m_memory.write8(addr + 0x2A, m_memory.read8(0x462));   // Active page
    m_cpu.setReg8(cpu::AL, 0x1B);                          // Function supported
    LOG_DEBUG("BIOS INT 10h AH=1Bh: State Info (filled)");
    break;
  }
  case 0xFE: { // Get Video Buffer Segment (Borland/TopView API)
    // Programs pass a guessed video segment in ES:DI and expect
    // ES to be updated with the actual segment to write to.
    // For text mode on CGA/EGA/VGA the answer is always B800h.
    m_cpu.setSegReg(cpu::ES, 0xB800);
    LOG_DEBUG("BIOS INT 10h AH=FEh: Get Video Buffer -> ES=B800h");
    break;
  }
  case 0x6F: { // Borland video BIOS extensions
    // Turbo C/Pascal call this to detect extended video capabilities.
    // Return AL != 6Fh to indicate not supported.
    m_cpu.setReg8(cpu::AL, 0x00);
    LOG_DEBUG("BIOS INT 10h AH=6Fh: Borland video extension (not supported)");
    break;
  }
  case 0xFA: {
    // Some software issues this undocumented/extended BIOS video function.
    // Stub it as no-op to avoid unhandled interrupt behavior.
    LOG_DEBUG("BIOS INT 10h AH=FAh: Reserved/unknown function (stubbed)");
    break;
  }
  case 0xEF: {
    // Some programs may call AH=EFh as undocumented extension; treat as NOP.
    m_cpu.setReg8(cpu::AL, 0x00);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    LOG_DEBUG("BIOS INT 10h AH=EFh: Unknown function stubbed");
    break;
  }
  default:
    LOG_WARN("BIOS INT 10h: Unknown function AH=0x", std::hex, (int)ah);
    break;
  }
}

void BIOS::handleKeyboardService() {
  uint8_t ah = m_cpu.getReg8(cpu::AH);
  if (m_kbd.hasKey())
    LOG_DEBUG("BIOS INT 16h AH=0x", std::hex, (int)ah,
              " hasKey=", m_kbd.hasKey());
  switch (ah) {
  case 0x10:   // Enhanced Read (fall through to 00h)
  case 0x00: { // Read Character (Blocking)
    if (!m_kbd.hasKey()) {
      // Poll host input directly while blocking so we don't have
      // to wait for the main loop's periodic poll.
      if (m_pollInput)
        m_pollInput();
      if (!m_kbd.hasKey()) {
        if (m_idleCallback)
          m_idleCallback();
        m_cpu.setEIP(m_cpu.getInstructionStartEIP());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return;
      }
    }
    auto [ascii, scancode] = m_kbd.popKey();
    m_cpu.setReg16(cpu::AX, (static_cast<uint16_t>(scancode) << 8) | ascii);
    LOG_DEBUG("BIOS INT 16h AH=00h: CONSUMED key scan=0x", std::hex,
              (int)scancode, " ascii=0x", (int)ascii, " (AX=0x",
              m_cpu.getReg16(cpu::AX), ")", " CS:IP=", m_cpu.getSegReg(cpu::CS),
              ":", m_cpu.getEIP());
    break;
  }
  case 0x11:   // Enhanced Status (fall through to 01h)
  case 0x01: { // Get Keyboard Status (peek, don't consume)
    if (!m_kbd.hasKey() && m_pollInput)
      m_pollInput();
    if (m_kbd.hasKey()) {
      auto [ascii, scancode] = m_kbd.peekKey();
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() &
                      ~cpu::FLAG_ZERO); // ZF=0 → key available
      m_cpu.setReg16(cpu::AX, (static_cast<uint16_t>(scancode) << 8) | ascii);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_ZERO); // ZF=1 → no key
    }
    break;
  }
  case 0x02: { // Get Shift Flags
    m_cpu.setReg8(cpu::AL, m_memory.read8(0x417));
    break;
  }
  case 0x03: { // Set Typematic Rate/Delay
    // Accept but ignore — no hardware to configure
    LOG_DEBUG("BIOS INT 16h AH=03h: Set Typematic (stubbed)");
    break;
  }
  case 0x05: { // Store Key in Keyboard Buffer
    uint8_t ch = m_cpu.getReg8(cpu::CL);
    uint8_t sc = m_cpu.getReg8(cpu::CH);
    m_kbd.pushKey(ch, sc);
    m_cpu.setReg8(cpu::AL, 0); // Success
    break;
  }
  case 0x12: { // Get Extended Shift Flags
    m_cpu.setReg8(cpu::AL, m_memory.read8(0x417));
    m_cpu.setReg8(cpu::AH, m_memory.read8(0x418));
    break;
  }
  case 0x55: { // Unknown/unused function (stubbed)
    m_cpu.setReg8(cpu::AL, 0x00);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    LOG_DEBUG("BIOS INT 16h AH=55h: Unknown function (stubbed)");
    break;
  }
  default:
    LOG_WARN("BIOS INT 16h: Unknown function AH=0x", std::hex, (int)ah);
    break;
  }
}

void BIOS::initialize() {
  // 1. Setup BDA (BIOS Data Area) at 0x40:0x00
  m_memory.write8(0x449, 0x03);         // Current video mode: 80x25 color text
  m_memory.write16(0x44A, 80);          // Number of columns
  m_memory.write16(0x44C, 80 * 25 * 2); // Page size in bytes
  m_memory.write16(0x44E, 0);           // Current page offset
  // Cursor positions for all 8 pages
  for (int p = 0; p < 8; ++p) {
    m_memory.write8(0x450 + p * 2, 0);     // Col
    m_memory.write8(0x450 + p * 2 + 1, 0); // Row
  }
  m_memory.write16(0x460, 0x0607); // Cursor type: start=6 end=7
  m_memory.write8(0x462, 0);       // Active video page
  m_memory.write16(0x463, 0x3D4);  // CRTC base I/O port (color)
  m_memory.write8(0x484, 24);      // Number of rows - 1
  m_memory.write16(0x485, 16);     // Character height in pixels

  // Keyboard buffer: circular 16-entry queue at 0x40:0x1E-0x3E
  m_memory.write16(0x41A, 0x001E); // Buffer head (offset within seg 0x40)
  m_memory.write16(0x41C, 0x001E); // Buffer tail (head==tail → empty)
  m_memory.write16(0x480, 0x001E); // Buffer start offset
  m_memory.write16(0x482, 0x003E); // Buffer end offset
  // Zero the buffer area itself
  for (uint16_t i = 0x41E; i < 0x43E; i += 2)
    m_memory.write16(i, 0);
  m_memory.write8(0x417, 0);    // Keyboard shift flags byte 1
  m_memory.write8(0x418, 0);    // Keyboard shift flags byte 2
  m_memory.write8(0x496, 0x10); // Enhanced keyboard flag (101-key present)

  // Memory size at 0x40:0x13
  m_memory.write16(0x413, 640); // 640KB conventional memory

  // Initialize VRAM to spaces with default attribute
  for (uint32_t i = 0; i < 80 * 25 * 2; i += 2) {
    m_memory.write8(0xB8000 + i, ' ');
    m_memory.write8(0xB8000 + i + 1, 0x07);
  }

  // Initialize VGA DAC palette at 0xE0000 (256 entries x 3 bytes = 768 bytes)
  // Standard CGA colors first (6-bit VGA values, 0-63)
  static const uint8_t cgaPal[16][3] = {
      {0, 0, 0},    {0, 0, 42},   {0, 42, 0},   {0, 42, 42},
      {42, 0, 0},   {42, 0, 42},  {42, 21, 0},  {42, 42, 42},
      {21, 21, 21}, {21, 21, 63}, {21, 63, 21}, {21, 63, 63},
      {63, 21, 21}, {63, 21, 63}, {63, 63, 21}, {63, 63, 63}};
  constexpr uint32_t PAL = 0xE0000;
  for (int i = 0; i < 16; ++i) {
    m_memory.write8(PAL + i * 3 + 0, cgaPal[i][0]);
    m_memory.write8(PAL + i * 3 + 1, cgaPal[i][1]);
    m_memory.write8(PAL + i * 3 + 2, cgaPal[i][2]);
  }
  // 16-31: Grayscale ramp
  for (int i = 0; i < 16; ++i) {
    uint8_t v = static_cast<uint8_t>(i * 63 / 15);
    m_memory.write8(PAL + (16 + i) * 3 + 0, v);
    m_memory.write8(PAL + (16 + i) * 3 + 1, v);
    m_memory.write8(PAL + (16 + i) * 3 + 2, v);
  }
  // 32-247: 6x6x6 color cube
  for (int i = 0; i < 216; ++i) {
    uint8_t r = static_cast<uint8_t>((i / 36) * 12 + 3);
    uint8_t g = static_cast<uint8_t>(((i % 36) / 6) * 12 + 3);
    uint8_t b = static_cast<uint8_t>((i % 6) * 12 + 3);
    m_memory.write8(PAL + (32 + i) * 3 + 0, r);
    m_memory.write8(PAL + (32 + i) * 3 + 1, g);
    m_memory.write8(PAL + (32 + i) * 3 + 2, b);
  }
  // 248-255: Black
  for (int i = 248; i < 256; ++i) {
    m_memory.write8(PAL + i * 3 + 0, 0);
    m_memory.write8(PAL + i * 3 + 1, 0);
    m_memory.write8(PAL + i * 3 + 2, 0);
  }

  // 2. Setup IVT (Interrupt Vector Table) vectors
  // Each vector gets a unique HLE intercept stub at F000:(0x100 + vector * 4).
  // We write the custom invalid opcode `0x0F 0xFF <vector>` so the CPU traps
  // directly to our HLE handlers instead of silently returning on an IRET.
  for (int v = 0; v < 256; ++v) {
    uint16_t stubOff = HLE_STUB_BASE + static_cast<uint16_t>(v * 4);
    uint32_t stubPhys = (static_cast<uint32_t>(HLE_STUB_SEG) << 4) + stubOff;
    m_memory.write8(stubPhys, 0x0F);                        // Custom HLE Trap
    m_memory.write8(stubPhys + 1, 0xFF);                    // Opcode Extension
    m_memory.write8(stubPhys + 2, static_cast<uint8_t>(v)); // Vector Num
    m_memory.write8(stubPhys + 3, 0xCF); // IRET fallback just in case

    m_memory.write16(v * 4, stubOff);
    m_memory.write16(v * 4 + 2, HLE_STUB_SEG);

    m_originalIVT[v] = {stubOff, HLE_STUB_SEG};
  }

  // INT 8 (IRQ0 / timer tick) handler — real machine code so INT 1Ch
  // chains work naturally when programs hook the user-timer vector.
  //   PUSH DS; PUSH AX; MOV AX,0040; MOV DS,AX;
  //   INC WORD [006C]; JNZ +4; INC WORD [006E];
  //   INT 1C; POP AX; POP DS; MOV AL,20; OUT 20,AL; IRET
  {
    constexpr uint16_t INT8_HANDLER = 0x0200;
    const uint32_t base =
        (static_cast<uint32_t>(HLE_STUB_SEG) << 4) + INT8_HANDLER;
    const uint8_t code[] = {
        0x1E,                   // PUSH DS
        0x50,                   // PUSH AX
        0xB8, 0x40, 0x00,       // MOV AX, 0040h
        0x8E, 0xD8,             // MOV DS, AX
        0xFF, 0x06, 0x6C, 0x00, // INC WORD [006Ch]
        0x75, 0x04,             // JNZ +4
        0xFF, 0x06, 0x6E, 0x00, // INC WORD [006Eh]
        0xCD, 0x1C,             // INT 1Ch
        0x58,                   // POP AX
        0x1F,                   // POP DS
        0xB0, 0x20,             // MOV AL, 20h
        0xE6, 0x20,             // OUT 20h, AL
        0xCF                    // IRET
    };
    for (size_t i = 0; i < sizeof(code); ++i)
      m_memory.write8(base + i, code[i]);
    m_memory.write16(0x08 * 4, INT8_HANDLER);
    m_memory.write16(0x08 * 4 + 2, HLE_STUB_SEG);
    m_originalIVT[0x08] = {INT8_HANDLER, HLE_STUB_SEG};
  }

  // Mouse driver stub at F000:0033 — programs check if INT 33h handler is a
  // bare IRET to detect presence. NOP + IRET avoids false "no driver"
  // detection. HLE intercepts INT 33h before this code ever executes.
  m_memory.write8(0xF0033, 0x90);         // NOP
  m_memory.write8(0xF0034, 0xCF);         // IRET
  m_memory.write16(0x33 * 4, 0x0033);     // INT 33h offset
  m_memory.write16(0x33 * 4 + 2, 0xF000); // INT 33h segment
  m_originalIVT[0x33] = {0x0033, 0xF000};

  // XMS driver entry point stub at F000:XMS_ENTRY_OFFSET.
  // Programs obtain this address via INT 2Fh AX=4310h, then CALL FAR to it.
  // The stub does INT E0h (our HLE dispatch vector) followed by RETF.
  {
    uint32_t xmsPhys =
        (static_cast<uint32_t>(HLE_STUB_SEG) << 4) + XMS_ENTRY_OFFSET;
    m_memory.write8(xmsPhys, 0xCD);     // INT
    m_memory.write8(xmsPhys + 1, 0xE0); // E0h
    m_memory.write8(xmsPhys + 2, 0xCB); // RETF
  }

  // DPMI entry point stub at F000:0050.
  // Programs obtain this address via INT 2Fh AX=1687h, then CALL FAR to it.
  // The 0F FF E1 HLE trap triggers the DPMI mode switch handler.
  {
    uint32_t dpmiPhys =
        (static_cast<uint32_t>(HLE_STUB_SEG) << 4) + 0x0050;
    m_memory.write8(dpmiPhys, 0x0F);     // HLE trap prefix
    m_memory.write8(dpmiPhys + 1, 0xFF); // HLE trap opcode
    m_memory.write8(dpmiPhys + 2, 0xE1); // Vector E1h (DPMI entry)
  }

  // DPMI raw mode switch: PM→RM at F000:005A (flat PM: 0x08:0xF005A)
  {
    uint32_t phys = 0xF005A;
    m_memory.write8(phys, 0x0F);
    m_memory.write8(phys + 1, 0xFF);
    m_memory.write8(phys + 2, 0xE2); // Vector E2h (PM→RM switch)
  }

  // Low-address copy of PM→RM stub at physical 0x500 (reachable as 0x08:0x0500)
  // DOS/4GW uses 16-bit JMP FAR in D=0 code segments; offset must fit in 16 bits.
  {
    m_memory.write8(0x500, 0x0F);
    m_memory.write8(0x501, 0xFF);
    m_memory.write8(0x502, 0xE2); // Vector E2h (PM→RM switch)
  }

  // DPMI raw mode switch: RM→PM at F000:005D
  {
    uint32_t phys = 0xF005D;
    m_memory.write8(phys, 0x0F);
    m_memory.write8(phys + 1, 0xFF);
    m_memory.write8(phys + 2, 0xE3); // Vector E3h (RM→PM switch)
  }

  // DPMI state save/restore stub at F000:0063 (no-op RETF)
  {
    uint32_t phys = 0xF0063;
    m_memory.write8(phys, 0xCB); // RETF (no state to save/restore)
  }

  initializeEMS();

  // IRETD trampoline at physical 0xF0300 — used by the HW interrupt
  // reflection path so that a PM handler called via near CALL can return
  // to the interrupted code through IRETD.
  m_memory.write8(0xF0300, 0xCF); // IRETD (in a D=1 segment, 0xCF = IRETD)

  LOG_INFO("BIOS: BDA and IVT initialized.");
}

void BIOS::initializeEMS() {
  m_emsHandles.clear();
  std::fill(m_emsMappings.begin(), m_emsMappings.end(), EMSMapping{});

  static constexpr char kEMMSignature[] =
      "COMPAQ EXPANDED MEMORY MANAGER 386";
  m_memory.write16((static_cast<uint32_t>(HLE_STUB_SEG) << 4) + 0x0012,
                   EMS_PRIVATE_API_OFFSET);
  const uint32_t sigPhys = (static_cast<uint32_t>(HLE_STUB_SEG) << 4) + 0x0014;
  for (size_t index = 0; index < sizeof(kEMMSignature); ++index) {
    m_memory.write8(sigPhys + static_cast<uint32_t>(index),
                    static_cast<uint8_t>(kEMMSignature[index]));
  }

  const uint32_t apiPhys =
      (static_cast<uint32_t>(HLE_STUB_SEG) << 4) + EMS_PRIVATE_API_OFFSET;
  m_memory.write8(apiPhys, 0xCB); // RETF: private API stub returns immediately

  if (uint8_t *pageFrame =
          m_memory.directAccess(static_cast<uint32_t>(EMS_PAGE_FRAME_SEGMENT)
                                << 4)) {
    std::fill_n(pageFrame,
                static_cast<size_t>(EMS_PAGE_SIZE) * EMS_PHYSICAL_PAGE_COUNT,
                0);
  }

  updateEMSImportRecord();
}

void BIOS::flushEMSPhysicalPage(uint8_t physicalPage) {
  if (physicalPage >= EMS_PHYSICAL_PAGE_COUNT)
    return;

  const EMSMapping &mapping = m_emsMappings[physicalPage];
  if (mapping.handle == 0 || mapping.handle > m_emsHandles.size())
    return;

  EMSHandle &handle = m_emsHandles[mapping.handle - 1];
  if (!handle.allocated || mapping.logicalPage >= handle.pages.size())
    return;

  uint8_t *frame = m_memory.directAccess(
      (static_cast<uint32_t>(EMS_PAGE_FRAME_SEGMENT) << 4) +
      static_cast<uint32_t>(physicalPage) * EMS_PAGE_SIZE);
  if (!frame)
    return;

  std::copy_n(frame, EMS_PAGE_SIZE, handle.pages[mapping.logicalPage].begin());
}

void BIOS::loadEMSPhysicalPage(uint8_t physicalPage) {
  if (physicalPage >= EMS_PHYSICAL_PAGE_COUNT)
    return;

  uint8_t *frame = m_memory.directAccess(
      (static_cast<uint32_t>(EMS_PAGE_FRAME_SEGMENT) << 4) +
      static_cast<uint32_t>(physicalPage) * EMS_PAGE_SIZE);
  if (!frame)
    return;

  const EMSMapping &mapping = m_emsMappings[physicalPage];
  if (mapping.handle == 0 || mapping.handle > m_emsHandles.size()) {
    std::fill_n(frame, EMS_PAGE_SIZE, 0);
    return;
  }

  const EMSHandle &handle = m_emsHandles[mapping.handle - 1];
  if (!handle.allocated || mapping.logicalPage >= handle.pages.size()) {
    std::fill_n(frame, EMS_PAGE_SIZE, 0);
    return;
  }

  std::copy(handle.pages[mapping.logicalPage].begin(),
            handle.pages[mapping.logicalPage].end(), frame);
}

size_t BIOS::countEMSAllocatedPages() const {
  size_t allocatedPages = 0;
  for (const EMSHandle &handle : m_emsHandles) {
    if (handle.allocated)
      allocatedPages += handle.pages.size();
  }
  return allocatedPages;
}

void BIOS::updateEMSImportRecord() {
  static constexpr uint16_t kImportRecordSize = 0x198;
  static constexpr uint16_t kFrameStatusOffset = 0x000A;
  static constexpr uint16_t kFrameStatusSize = 6;
  static constexpr uint16_t kHandleCountOffset = 0x018C;
  static constexpr uint16_t kVersion110Offset = 0x018D;

  for (uint16_t offset = 0; offset < kImportRecordSize; ++offset) {
    m_memory.write8(EMS_IMPORT_RECORD_PHYS + offset, 0);
  }

  m_memory.write16(EMS_IMPORT_RECORD_PHYS + 0x02, kImportRecordSize);
  m_memory.write16(EMS_IMPORT_RECORD_PHYS + 0x04, 0x0110);

  for (uint16_t index = 0; index < 64; ++index) {
    const uint32_t recordAddr = EMS_IMPORT_RECORD_PHYS + kFrameStatusOffset +
                                static_cast<uint32_t>(index) * kFrameStatusSize;
    m_memory.write8(recordAddr + 0, 0x00);
    m_memory.write8(recordAddr + 1, 0xFF);
    m_memory.write16(recordAddr + 2, 0xFFFF);
    m_memory.write8(recordAddr + 4, 0xFF);
    m_memory.write8(recordAddr + 5, 0x00);
  }

  const uint16_t firstFrameRecord = EMS_PAGE_FRAME_SEGMENT >> 10;
  for (uint8_t physicalPage = 0; physicalPage < EMS_PHYSICAL_PAGE_COUNT;
       ++physicalPage) {
    const uint16_t recordIndex = firstFrameRecord + physicalPage;
    const uint32_t recordAddr = EMS_IMPORT_RECORD_PHYS + kFrameStatusOffset +
                                static_cast<uint32_t>(recordIndex) *
                                    kFrameStatusSize;
    const EMSMapping &mapping = m_emsMappings[physicalPage];

    m_memory.write8(recordAddr + 0, 0x03); // EMS frame in standard 64K page frame
    m_memory.write8(recordAddr + 1,
                    mapping.handle == 0
                        ? 0xFF
                        : static_cast<uint8_t>(mapping.handle & 0x00FF));
    m_memory.write16(recordAddr + 2,
                     mapping.handle == 0 ? 0x7FFF : mapping.logicalPage);
    m_memory.write8(recordAddr + 4, physicalPage);
    m_memory.write8(recordAddr + 5, 0x00);
  }

  m_memory.write8(EMS_IMPORT_RECORD_PHYS + 0x018A,
                  static_cast<uint8_t>(EMS_PHYSICAL_PAGE_COUNT * 3 + 4));
  m_memory.write8(EMS_IMPORT_RECORD_PHYS + 0x018B, 0x00); // No UMB descriptors
  m_memory.write8(EMS_IMPORT_RECORD_PHYS + kHandleCountOffset,
                  0x00); // Handle info records omitted for now

  m_memory.write16(EMS_IMPORT_RECORD_PHYS + kVersion110Offset + 0,
                   m_originalIVT[0x67].first);
  m_memory.write16(EMS_IMPORT_RECORD_PHYS + kVersion110Offset + 2,
                   m_originalIVT[0x67].second);
  m_memory.write32(EMS_IMPORT_RECORD_PHYS + kVersion110Offset + 4,
                   0x00000000);
  m_memory.write8(EMS_IMPORT_RECORD_PHYS + kVersion110Offset + 8,
                  0x00); // Free page entry count
  m_memory.write8(EMS_IMPORT_RECORD_PHYS + kVersion110Offset + 9,
                  0x00); // XMS handle count
  m_memory.write8(EMS_IMPORT_RECORD_PHYS + kVersion110Offset + 10,
                  0x00); // Free UMB info count
}

bool BIOS::isOriginalIVT(uint8_t vector, uint16_t cs, uint32_t eip) const {
  auto &orig = m_originalIVT[vector];
  // Real mode match: CS=F000, IP=stub offset
  if (static_cast<uint16_t>(eip) == orig.first && cs == orig.second)
    return true;
  // Protected mode match: flat code selector (base=0), physical offset matches
  // Our PM IDT points to selector 0x08 (flat) at offset F0000h + stub_offset
  uint32_t stubPhys = 0xF0000 + orig.first;
  if (eip == stubPhys)
    return true;
  return false;
}

void BIOS::handleTimeService() {
  uint8_t ah = m_cpu.getReg8(cpu::AH);
  switch (ah) {
  case 0x00: { // Read system timer
    LOG_TRACE("BIOS INT 1Ah: Get System Time");
    // BIOS timer tick count at 0x40:0x6C
    uint32_t ticks = m_memory.read32(0x46C);
    m_cpu.setReg16(cpu::CX, static_cast<uint16_t>(ticks >> 16));
    m_cpu.setReg16(cpu::DX, static_cast<uint16_t>(ticks & 0xFFFF));
    m_cpu.setReg8(cpu::AL, 0); // Midnight flag
    break;
  }
  default:
    LOG_WARN("BIOS INT 1Ah: Unknown function AH=0x", std::hex, (int)ah);
    break;
  }
}

void BIOS::handleDiskService() {
  uint8_t ah = m_cpu.getReg8(cpu::AH);
  uint8_t dl = m_cpu.getReg8(cpu::DL); // Drive

  switch (ah) {
  case 0x00: { // Reset Disk System
    LOG_DEBUG("BIOS INT 13h: Reset Disk System");
    m_cpu.setReg8(cpu::AH, 0); // Success
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    break;
  }
  case 0x02: {                           // Read Sectors
    uint8_t al = m_cpu.getReg8(cpu::AL); // Num sectors
    uint8_t ch = m_cpu.getReg8(cpu::CH); // Cylinder (low 8)
    uint8_t cl = m_cpu.getReg8(cpu::CL); // Sector (1-63) + Cy (high 2)
    uint8_t dh = m_cpu.getReg8(cpu::DH); // Head
    uint16_t es = m_cpu.getSegReg(cpu::ES);
    uint16_t bx = m_cpu.getReg16(cpu::BX);

    uint16_t cylinder = ch | ((cl & 0xC0) << 2);
    uint8_t sector = cl & 0x3F;

    LOG_DEBUG("BIOS INT 13h: Read ", (int)al, " sectors from Drive ", (int)dl,
              " (C:", cylinder, " H:", (int)dh, " S:", (int)sector, ") to ",
              std::hex, es, ":", bx);

    if (dl == 0x00 && m_floppyLoaded) {
      // CHS Mapping for 1.44MB floppy: 80/2/18
      uint32_t lba = ((cylinder * 2 + dh) * 18) + (sector - 1);
      uint32_t dest = (es << 4) + bx;

      for (int i = 0; i < al; ++i) {
        uint32_t srcStart = (lba + i) * 512;
        if (srcStart + 512 > m_floppyData.size()) {
          m_cpu.setReg8(cpu::AH, 0x01); // Invalid parameter
          m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
          return;
        }
        for (int j = 0; j < 512; ++j) {
          m_memory.write8(dest + i * 512 + j, m_floppyData[srcStart + j]);
        }
      }
      m_cpu.setReg8(cpu::AH, 0); // Success
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      m_cpu.setReg8(cpu::AL, al); // AL = sectors read
    } else {
      LOG_WARN("BIOS INT 13h: Drive not found or not loaded: ", (int)dl);
      m_cpu.setReg8(cpu::AH, 0x01); // Invalid function/parameter
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    }
    break;
  }
  case 0x08: {        // Get Drive Parameters
    if (dl == 0x00) { // Floppy 0
      m_cpu.setReg8(cpu::AH, 0);
      m_cpu.setReg8(cpu::BL, 0x04); // 1.44MB
      m_cpu.setReg8(cpu::CH, 79);   // Max cylinder index (low 8 bits)
      // CL: bits 6-7 = high 2 bits of cyl, bits 0-5 = max sector number
      m_cpu.setReg8(cpu::CL, 18); // 18 sectors/track
      m_cpu.setReg8(cpu::DH, 1); // Max head index (0-based, so 1 means 2 heads)
      m_cpu.setReg8(cpu::DL, 1); // Num floppy drives
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      m_cpu.setReg8(cpu::AH, 0x07); // Drive parameter activity failed
    }
    break;
  }
  case 0x15: {                      // Get Disk Type
    if (dl == 0x00) {               // Floppy
      m_cpu.setReg8(cpu::AH, 0x01); // Floppy, no change line
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else {
      m_cpu.setReg8(cpu::AH, 0x00); // Drive not present
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    }
    break;
  }
  default:
    LOG_WARN("BIOS INT 13h: Unknown function AH=0x", std::hex, (int)ah);
    m_cpu.setReg8(cpu::AH, 0x01);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    break;
  }
}

// ── INT 15h — System Services ───────────────────────────────────────────────

void BIOS::handleSystemService() {
  uint8_t ah = m_cpu.getReg8(cpu::AH);
  switch (ah) {
  case 0x24: { // A20 gate support — extended function
    uint8_t al = m_cpu.getReg8(cpu::AL);
    switch (al) {
    case 0x00: // Disable A20
      m_memory.setA20(false);
      m_cpu.setReg8(cpu::AH, 0);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      break;
    case 0x01: // Enable A20
      m_memory.setA20(true);
      m_cpu.setReg8(cpu::AH, 0);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      break;
    case 0x02: // Query A20 status
      m_cpu.setReg8(cpu::AH, 0);
      m_cpu.setReg8(cpu::AL, m_memory.isA20Enabled() ? 1 : 0);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      break;
    case 0x03: // Query A20 support
      m_cpu.setReg8(cpu::AH, 0);
      m_cpu.setReg16(cpu::BX, 0x03); // Keyboard + port 92h supported
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      break;
    default:
      m_cpu.setReg8(cpu::AH, 0x86);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      break;
    }
    LOG_DEBUG("BIOS INT 15h AH=24h AL=", std::hex, (int)al,
              ": A20 gate control");
    break;
  }
  case 0x87: { // Copy Extended Memory Block
    uint16_t count = m_cpu.getReg16(cpu::CX);
    uint32_t len = static_cast<uint32_t>(count) * 2;
    uint16_t es = m_cpu.getSegReg(cpu::ES);
    uint16_t si = m_cpu.getReg16(cpu::SI);
    uint32_t gdtAddr = (static_cast<uint32_t>(es) << 4) + si;

    // Descriptors at 0x10 (source) and 0x18 (dest)
    auto readPhysAddr = [&](uint32_t descAddr) {
      uint32_t base = m_memory.read16(descAddr + 2);
      base |= (static_cast<uint32_t>(m_memory.read8(descAddr + 4)) << 16);
      base |= (static_cast<uint32_t>(m_memory.read8(descAddr + 7)) << 24);
      return base;
    };

    uint32_t srcPhys = readPhysAddr(gdtAddr + 0x10);
    uint32_t dstPhys = readPhysAddr(gdtAddr + 0x18);

    LOG_DEBUG("BIOS INT 15h AH=87h: Copy ", len, " bytes from 0x", std::hex,
              srcPhys, " to 0x", dstPhys);

    for (uint32_t i = 0; i < len; ++i) {
      m_memory.write8(dstPhys + i, m_memory.read8(srcPhys + i));
    }

    m_cpu.setReg8(cpu::AH, 0);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    break;
  }
  case 0x88: { // Get Extended Memory Size (above 1MB, in KB)
    // If an XMS driver is installed, it should "claim" all INT 15h memory.
    // Return 0 if HIMEM is present, otherwise return total.
    uint16_t extKB = 0;
    if (!m_himem) {
      extKB =
          static_cast<uint16_t>((memory::MemoryBus::MEMORY_SIZE / 1024) - 1024);
    }
    m_cpu.setReg16(cpu::AX, extKB);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    break;
  }
  case 0xE8: {
    uint8_t al = m_cpu.getReg8(cpu::AL);
    if (al == 0x01) { // Get memory size for >64M configurations
      // AX/CX = configured memory 1M to 16M, in K
      // BX/DX = configured memory above 16M, in 64K blocks
      uint16_t mem1to16 = 15 * 1024;          // 15MB
      uint16_t memAbove16 = (48 * 1024) / 64; // 48MB / 64K = 768

      m_cpu.setReg16(cpu::AX, mem1to16);
      m_cpu.setReg16(cpu::CX, mem1to16);
      m_cpu.setReg16(cpu::BX, memAbove16);
      m_cpu.setReg16(cpu::DX, memAbove16);

      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DEBUG(
          "BIOS INT 15h AH=E801h: Reported 15MB (AX/CX) and 48MB (BX/DX)");
    } else {
      m_cpu.setReg8(cpu::AH, 0x86);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    }
    break;
  }
  case 0xC0: { // Get System Configuration
    // BIOS Configuration Table at F000:FFD0
    uint32_t tableAddr = 0xFFFD0;
    m_memory.write16(tableAddr, 8);       // Length
    m_memory.write8(tableAddr + 2, 0xFC); // Model: AT
    m_memory.write8(tableAddr + 3, 0x01); // Submodel
    m_memory.write8(tableAddr + 4, 0x00); // Revision
    m_memory.write8(tableAddr + 5,
                    0x70); // Feature 1: EBDA, RTC, Keyboard intercept
    m_memory.write8(tableAddr + 6, 0x00); // Feature 2
    m_memory.write8(tableAddr + 7, 0x00); // Feature 3
    m_memory.write8(tableAddr + 8, 0x00); // Feature 4
    m_memory.write8(tableAddr + 9, 0x00); // Feature 5

    m_cpu.setSegReg(cpu::ES, 0xF000);
    m_cpu.setReg16(cpu::BX, 0xFFD0);
    m_cpu.setReg8(cpu::AH, 0);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    LOG_DEBUG("BIOS INT 15h AH=C0h: Return System Config at F000:FFD0");
    break;
  }
  case 0x86: { // Wait (microseconds in CX:DX)
    // Programs use this for delays. We just return immediately.
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    LOG_DEBUG("BIOS INT 15h AH=86h: Wait (stubbed - no delay)");
    break;
  }
  case 0x41: { // Wait on external event (PS/2)
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    break;
  }
  case 0x4F: { // Keyboard intercept
    // Return carry=1 to indicate key should be processed normally
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    break;
  }
  default:
    LOG_DEBUG("BIOS INT 15h: Unknown function AH=0x", std::hex, (int)ah);
    m_cpu.setReg8(cpu::AH, 0x86); // Function not supported
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    break;
  }
}

void BIOS::handleEMSService() {
  const uint8_t function = m_cpu.getReg8(cpu::AH);

  auto fail = [this](uint8_t status) { m_cpu.setReg8(cpu::AH, status); };
  auto validHandle = [this](uint16_t handle) {
    return handle >= 1 && handle <= m_emsHandles.size() &&
           m_emsHandles[handle - 1].allocated;
  };

  switch (function) {
  case 0x40: // Get Status
    m_cpu.setReg8(cpu::AH, 0x00);
    break;

  case 0x41: // Get Page Frame Address
    m_cpu.setReg16(cpu::BX, EMS_PAGE_FRAME_SEGMENT);
    m_cpu.setReg8(cpu::AH, 0x00);
    break;

  case 0x42: { // Get Unallocated and Total Page Counts
    const uint16_t availablePages = static_cast<uint16_t>(
        EMS_TOTAL_PAGES - static_cast<uint16_t>(countEMSAllocatedPages()));
    m_cpu.setReg16(cpu::BX, availablePages);
    m_cpu.setReg16(cpu::DX, EMS_TOTAL_PAGES);
    m_cpu.setReg8(cpu::AH, 0x00);
    break;
  }

  case 0x43: { // Allocate Pages
    const uint16_t requestedPages = m_cpu.getReg16(cpu::BX);
    const uint16_t availablePages = static_cast<uint16_t>(
        EMS_TOTAL_PAGES - static_cast<uint16_t>(countEMSAllocatedPages()));

    if (requestedPages == 0) {
      fail(0x89);
      break;
    }
    if (requestedPages > availablePages) {
      fail(0x88);
      break;
    }

    size_t handleIndex = 0;
    while (handleIndex < m_emsHandles.size() && m_emsHandles[handleIndex].allocated)
      ++handleIndex;
    if (handleIndex == m_emsHandles.size()) {
      if (handleIndex >= 0xFF) {
        fail(0x85);
        break;
      }
      m_emsHandles.emplace_back();
    }

    EMSHandle &handle = m_emsHandles[handleIndex];
    handle.allocated = true;
    handle.pages.assign(requestedPages,
                        std::vector<uint8_t>(EMS_PAGE_SIZE, 0));

    m_cpu.setReg16(cpu::DX, static_cast<uint16_t>(handleIndex + 1));
    m_cpu.setReg8(cpu::AH, 0x00);
    updateEMSImportRecord();
    break;
  }

  case 0x44: { // Map / Unmap Handle Page
    const uint8_t physicalPage = m_cpu.getReg8(cpu::AL);
    const uint16_t logicalPage = m_cpu.getReg16(cpu::BX);
    const uint16_t handleId = m_cpu.getReg16(cpu::DX);

    if (physicalPage >= EMS_PHYSICAL_PAGE_COUNT) {
      fail(0x8B);
      break;
    }

    flushEMSPhysicalPage(physicalPage);

    if (logicalPage == 0xFFFF) {
      m_emsMappings[physicalPage] = {};
      loadEMSPhysicalPage(physicalPage);
      m_cpu.setReg8(cpu::AH, 0x00);
      updateEMSImportRecord();
      break;
    }

    if (!validHandle(handleId)) {
      fail(0x83);
      break;
    }

    const EMSHandle &handle = m_emsHandles[handleId - 1];
    if (logicalPage >= handle.pages.size()) {
      fail(0x8A);
      break;
    }

    m_emsMappings[physicalPage] = {handleId, logicalPage};
    loadEMSPhysicalPage(physicalPage);
    m_cpu.setReg8(cpu::AH, 0x00);
    updateEMSImportRecord();
    break;
  }

  case 0x45: { // Deallocate Pages
    const uint16_t handleId = m_cpu.getReg16(cpu::DX);
    if (!validHandle(handleId)) {
      fail(0x83);
      break;
    }

    for (uint8_t physicalPage = 0; physicalPage < EMS_PHYSICAL_PAGE_COUNT;
         ++physicalPage) {
      if (m_emsMappings[physicalPage].handle == handleId) {
        flushEMSPhysicalPage(physicalPage);
        m_emsMappings[physicalPage] = {};
        loadEMSPhysicalPage(physicalPage);
      }
    }

    EMSHandle &handle = m_emsHandles[handleId - 1];
    handle.pages.clear();
    handle.allocated = false;

    m_cpu.setReg8(cpu::AH, 0x00);
    updateEMSImportRecord();
    break;
  }

  case 0x46: // Get EMM Version
    m_cpu.setReg8(cpu::AL, 0x40); // EMS 4.0
    m_cpu.setReg8(cpu::AH, 0x00);
    break;

  case 0x4B: { // Get Number of Open Handles
    uint16_t openHandles = 0;
    for (const EMSHandle &handle : m_emsHandles) {
      if (handle.allocated)
        ++openHandles;
    }
    m_cpu.setReg16(cpu::BX, openHandles);
    m_cpu.setReg8(cpu::AH, 0x00);
    break;
  }

  case 0x4C: { // Get Pages Owned by Handle
    const uint16_t handleId = m_cpu.getReg16(cpu::DX);
    if (!validHandle(handleId)) {
      fail(0x83);
      break;
    }
    m_cpu.setReg16(cpu::BX,
                   static_cast<uint16_t>(m_emsHandles[handleId - 1].pages.size()));
    m_cpu.setReg8(cpu::AH, 0x00);
    break;
  }

  default:
    fail(0x84); // Undefined function
    break;
  }
}

// ── INT E0h — XMS Driver Dispatch (HLE) ─────────────────────────────────────

void BIOS::handleXMSDispatch() {
  if (!m_himem) {
    m_cpu.setReg16(cpu::AX, 0);
    m_cpu.setReg8(cpu::BL, 0x80); // Not implemented
    return;
  }

  uint8_t ah = m_cpu.getReg8(cpu::AH);
  LOG_DEBUG("XMS dispatch: AH=0x", std::hex, (int)ah);

  switch (ah) {
  case 0x00: {                                      // Get XMS Version
    m_cpu.setReg16(cpu::AX, m_himem->getVersion()); // XMS spec version (BCD)
    m_cpu.setReg16(cpu::BX,
                   m_himem->getDriverVersion()); // Driver internal revision
    m_cpu.setReg16(cpu::DX, 1);                  // HMA exists
    LOG_DEBUG("XMS 00h: Version=", std::hex, m_himem->getVersion());
    break;
  }
  case 0x01: { // Request HMA
    uint16_t size = m_cpu.getReg16(cpu::DX);
    if (m_himem->requestHMA(size)) {
      m_cpu.setReg16(cpu::AX, 1);
    } else {
      m_cpu.setReg16(cpu::AX, 0);
      m_cpu.setReg8(cpu::BL, m_himem->lastError());
    }
    break;
  }
  case 0x02: { // Release HMA
    if (m_himem->releaseHMA()) {
      m_cpu.setReg16(cpu::AX, 1);
    } else {
      m_cpu.setReg16(cpu::AX, 0);
      m_cpu.setReg8(cpu::BL, m_himem->lastError());
    }
    break;
  }
  case 0x03: { // Global Enable A20
    m_himem->enableA20();
    m_cpu.setReg16(cpu::AX, 1);
    break;
  }
  case 0x04: { // Global Disable A20
    m_himem->disableA20();
    m_cpu.setReg16(cpu::AX, 1);
    break;
  }
  case 0x05: { // Local Enable A20
    m_himem->enableA20();
    m_cpu.setReg16(cpu::AX, 1);
    break;
  }
  case 0x06: { // Local Disable A20
    m_himem->disableA20();
    m_cpu.setReg16(cpu::AX, 1);
    break;
  }
  case 0x07: { // Query A20 State
    m_cpu.setReg16(cpu::AX, m_himem->isA20Enabled() ? 1 : 0);
    break;
  }
  case 0x08: { // Query Free Extended Memory
    uint16_t largestKB = 0;
    uint16_t totalKB = m_himem->queryFreeKB(largestKB);
    m_cpu.setReg16(cpu::AX, largestKB); // Largest free block in KB
    m_cpu.setReg16(cpu::DX, totalKB);   // Total free in KB
    LOG_DEBUG("XMS 08h: Largest=", largestKB, "KB Total=", totalKB, "KB");
    break;
  }
  case 0x09: { // Allocate Extended Memory Block
    uint16_t sizeKB = m_cpu.getReg16(cpu::DX);
    uint16_t handle = m_himem->allocateEMB(sizeKB);
    if (handle) {
      m_cpu.setReg16(cpu::AX, 1);
      m_cpu.setReg16(cpu::DX, handle);
      LOG_DEBUG("XMS 09h: Allocated ", sizeKB, "KB -> handle ", handle);
    } else {
      m_cpu.setReg16(cpu::AX, 0);
      m_cpu.setReg8(cpu::BL, m_himem->lastError());
      LOG_WARN("XMS 09h: Failed to allocate ", sizeKB, "KB");
    }
    break;
  }
  case 0x0A: { // Free Extended Memory Block
    uint16_t handle = m_cpu.getReg16(cpu::DX);
    if (m_himem->freeEMB(handle)) {
      m_cpu.setReg16(cpu::AX, 1);
    } else {
      m_cpu.setReg16(cpu::AX, 0);
      m_cpu.setReg8(cpu::BL, m_himem->lastError());
    }
    break;
  }
  case 0x0B: { // Move Extended Memory Block
    // DS:SI points to an XMS move struct:
    //   DWORD Length, WORD SrcHandle, DWORD SrcOffset, WORD DstHandle, DWORD
    //   DstOffset
    uint16_t ds = m_cpu.getSegReg(cpu::DS);
    uint16_t si = m_cpu.getReg16(cpu::SI);
    uint32_t ptr = (static_cast<uint32_t>(ds) << 4) + si;

    uint32_t length = m_memory.read32(ptr);
    uint16_t srcHandle = m_memory.read16(ptr + 4);
    uint32_t srcOffset = m_memory.read32(ptr + 6);
    uint16_t dstHandle = m_memory.read16(ptr + 10);
    uint32_t dstOffset = m_memory.read32(ptr + 12);

    LOG_DEBUG(
        "XMS 0Bh struct at ", std::hex, ds, ":", si, " (flat ", ptr,
        ") raw[0..19]: ", (int)m_memory.read8(ptr), " ",
        (int)m_memory.read8(ptr + 1), " ", (int)m_memory.read8(ptr + 2), " ",
        (int)m_memory.read8(ptr + 3), " ", (int)m_memory.read8(ptr + 4), " ",
        (int)m_memory.read8(ptr + 5), " ", (int)m_memory.read8(ptr + 6), " ",
        (int)m_memory.read8(ptr + 7), " ", (int)m_memory.read8(ptr + 8), " ",
        (int)m_memory.read8(ptr + 9), " ", (int)m_memory.read8(ptr + 10), " ",
        (int)m_memory.read8(ptr + 11), " ", (int)m_memory.read8(ptr + 12), " ",
        (int)m_memory.read8(ptr + 13), " ", (int)m_memory.read8(ptr + 14), " ",
        (int)m_memory.read8(ptr + 15), " ", (int)m_memory.read8(ptr + 16), " ",
        (int)m_memory.read8(ptr + 17), " ", (int)m_memory.read8(ptr + 18), " ",
        (int)m_memory.read8(ptr + 19));
    LOG_DEBUG("  parsed: len=", std::dec, length, " srcH=", srcHandle,
              " srcOff=0x", std::hex, srcOffset, " dstH=", std::dec, dstHandle,
              " dstOff=0x", std::hex, dstOffset, " retaddr=",
              m_memory.read16((m_cpu.getSegReg(cpu::SS) << 4) +
                              m_cpu.getReg16(cpu::SP) + 2),
              ":",
              m_memory.read16((m_cpu.getSegReg(cpu::SS) << 4) +
                              m_cpu.getReg16(cpu::SP)));

    if (m_himem->moveEMB(length, srcHandle, srcOffset, dstHandle, dstOffset,
                         m_memory)) {
      m_cpu.setReg16(cpu::AX, 1);
      LOG_DEBUG("XMS 0Bh: Move OK len=", length, " src=", srcHandle, ":",
                std::hex, srcOffset, " dst=", dstHandle, ":", dstOffset);
    } else {
      m_cpu.setReg16(cpu::AX, 0);
      m_cpu.setReg8(cpu::BL, m_himem->lastError());
      LOG_WARN("XMS 0Bh: Move FAILED len=", length, " src=", srcHandle, ":",
               std::hex, srcOffset, " dst=", dstHandle, ":", dstOffset,
               " err=", (int)m_himem->lastError());
    }
    break;
  }
  case 0x0C: { // Lock Extended Memory Block
    uint16_t handle = m_cpu.getReg16(cpu::DX);
    uint32_t linearAddr = 0;
    if (m_himem->lockEMB(handle, linearAddr)) {
      m_cpu.setReg16(cpu::AX, 1);
      m_cpu.setReg16(cpu::DX, static_cast<uint16_t>(linearAddr >> 16));
      m_cpu.setReg16(cpu::BX, static_cast<uint16_t>(linearAddr & 0xFFFF));
    } else {
      m_cpu.setReg16(cpu::AX, 0);
      m_cpu.setReg8(cpu::BL, m_himem->lastError());
    }
    break;
  }
  case 0x0D: { // Unlock Extended Memory Block
    uint16_t handle = m_cpu.getReg16(cpu::DX);
    if (m_himem->unlockEMB(handle)) {
      m_cpu.setReg16(cpu::AX, 1);
    } else {
      m_cpu.setReg16(cpu::AX, 0);
      m_cpu.setReg8(cpu::BL, m_himem->lastError());
    }
    break;
  }
  case 0x0E: { // Get Handle Information
    uint16_t handle = m_cpu.getReg16(cpu::DX);
    m_cpu.setReg16(cpu::AX, 1);
    m_cpu.setReg8(cpu::BH, m_himem->getLockCount(handle));
    m_cpu.setReg8(cpu::BL, m_himem->getFreeHandles());
    m_cpu.setReg16(cpu::DX,
                   static_cast<uint16_t>(m_himem->getHandleSizeKB(handle)));
    LOG_DEBUG("XMS 0Eh: Info handle=", handle,
              " size=", m_cpu.getReg16(cpu::DX),
              "KB lock=", (int)m_cpu.getReg8(cpu::BH));
    break;
  }
  case 0x0F: { // Reallocate Extended Memory Block
    uint16_t handle = m_cpu.getReg16(cpu::DX);
    uint16_t newSizeKB = m_cpu.getReg16(cpu::BX);
    LOG_DEBUG("XMS 0Fh: Resize handle=", handle, " newSize=", newSizeKB, "KB");
    if (m_himem->resizeEMB(handle, newSizeKB)) {
      m_cpu.setReg16(cpu::AX, 1);
      LOG_DEBUG("XMS 0Fh: Resize OK");
    } else {
      m_cpu.setReg16(cpu::AX, 0);
      m_cpu.setReg8(cpu::BL, m_himem->lastError());
      LOG_WARN("XMS 0Fh: Resize FAILED err=", (int)m_himem->lastError());
    }
    break;
  }
  default:
    LOG_WARN("XMS: Unknown function AH=0x", std::hex, (int)ah);
    m_cpu.setReg16(cpu::AX, 0);
    m_cpu.setReg8(cpu::BL, 0x80); // Not implemented
    break;
  }
}

} // namespace fador::hw
