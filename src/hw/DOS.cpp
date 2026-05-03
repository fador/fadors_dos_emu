#include "../memory/himem/HIMEM.hpp"

#include "../utils/Logger.hpp"
#include "DOS.hpp"
#include "DPMI.hpp"
#include "KeyboardController.hpp"
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>

namespace fs = std::filesystem;

namespace fador::hw {

namespace {

std::string upperASCII(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) {
                   return static_cast<char>(std::toupper(ch));
                 });
  return value;
}

bool isEMSDeviceProbe(const std::string &filename) {
  const std::string leaf = upperASCII(fs::path(filename).filename().string());
  return leaf == "EMMXXXX0" || leaf == "EMMQXXX0";
}

constexpr uint16_t kEMSPrivateApiOffset = 0x0070;
constexpr uint16_t kEMSPrivateApiSegment = 0xF000;
constexpr uint32_t kEMSImportRecordPhys = 0xF1100;
constexpr uint8_t kEMSImportMajorVersion = 0x01;
constexpr uint8_t kEMSImportMinorVersion = 0x10;

} // namespace

DOS::DOS(cpu::CPU &cpu, memory::MemoryBus &memory)
    : m_cpu(cpu), m_memory(memory) {
  m_himem = std::make_unique<memory::HIMEM>();
}

void DOS::initialize() {
  LOG_INFO("DOS: Kernel initialized.");

  // Setup initial MCB chain
  // Block 1: PSP starting at FIRST_MCB_SEGMENT + 1 (0x801)
  m_pspSegment = FIRST_MCB_SEGMENT + 1;

  // Real DOS allocates ALL available conventional memory to the loaded
  // program (respecting MZ maxAlloc=0xFFFF).  The program then shrinks its
  // block with INT 21h AH=4Ah to release memory for other uses.
  MCB psp;
  psp.type = 'Z'; // Only block — last in chain
  psp.owner = m_pspSegment;
  psp.size = LAST_PARA - m_pspSegment; // All memory up to 640K
  for (int i = 0; i < 8; ++i)
    psp.name[i] = 0;
  writeMCB(FIRST_MCB_SEGMENT, psp);

  // Initialize List of Lists pointer
  // LoL is at 0070:0000 (0x0700). MCB pointer is at LoL-2 (0x06FE)
  m_memory.write16(0x06FE, FIRST_MCB_SEGMENT);

  LOG_INFO("DOS: Initial MCB chain setup. PSP block at 0x", std::hex,
           m_pspSegment, " size 0x", std::hex, psp.size);

  // Default DTA is at offset 0x80 of the initial PSP
  m_dtaPtr = (static_cast<uint32_t>(m_pspSegment) << 16) | 0x0080;
}

bool DOS::handleInterrupt(uint8_t vector) {
  switch (vector) {
  case 0x20: // Terminate
    terminateProcess(0);
    return true;
  case 0x21: // DOS API
    handleDOSService();
    return true;
  case 0x3F: { // Overlay Manager / Generic Proxy (VROOMM / FBOV)
    uint16_t cs = m_cpu.getSegReg(cpu::CS);

    uint32_t addr = m_cpu.getSegBase(cpu::CS) + m_cpu.getEIP();

    // Metadata follows INT 3F: [uint16_t offset, uint16_t segmentIndex]
    uint16_t targetOffset = m_memory.read16(addr);
    uint16_t segIndex = m_memory.read16(addr + 2);
    LOG_DOS("DOS: VROOMM thunk raw: ", std::hex, m_memory.read8(addr), " ",
            m_memory.read8(addr + 1), " ", m_memory.read8(addr + 2), " ",
            m_memory.read8(addr + 3));
    LOG_DOS("DOS: VROOMM interpreted: segIndex=", std::dec, segIndex, " (0x",
            std::hex, segIndex, ") offset=0x", targetOffset);

    uint16_t loadedSeg = 0;
    if (segIndex == 0) {
      if (!m_neSegments.empty())
        loadedSeg = loadOverlaySegment(0);
    } else {
      const uint16_t internalSegIdx = static_cast<uint16_t>(segIndex - 1);
      if (internalSegIdx < m_neSegments.size()) {
        loadedSeg = loadOverlaySegment(internalSegIdx);
      } else {
        auto it = std::find_if(
            m_fbovOverlays.begin(), m_fbovOverlays.end(),
            [segIndex](const FBOVOverlay &overlay) {
              return overlay.overlayId == segIndex;
            });
        if (it != m_fbovOverlays.end()) {
          loadedSeg =
              loadFBOVOverlay(static_cast<size_t>(std::distance(
                  m_fbovOverlays.begin(), it)));
        }
      }
    }

    if (loadedSeg == 0) {
      LOG_ERROR("DOS: VROOMM/FBOV failed to resolve overlay id ", std::dec,
                segIndex, " (NE segments=", m_neSegments.size(),
                " FBOV blocks=", m_fbovOverlays.size(), ")");
      return true;
    }

    // Patch the thunk at [CS:InstructionStartEIP] with a JMP FAR (EA OO OO SS SS)
    uint32_t thunkStart = m_cpu.getSegBase(cpu::CS) + m_cpu.getInstructionStartEIP();
    m_memory.write8(thunkStart, 0xEA); // JMP far
    m_memory.write16(thunkStart + 1, targetOffset);
    m_memory.write16(thunkStart + 3, loadedSeg);

    LOG_DOS("DOS: Thunk at ", std::hex, cs, ":", m_cpu.getInstructionStartEIP(), " patched to JMP ",
            loadedSeg, ":", targetOffset);

    // Rewind EIP to re-execute the patched instruction
    m_cpu.setEIP(m_cpu.getInstructionStartEIP());
    return true;
  }
  case 0x2F: { // Multiplex
    uint16_t ax = m_cpu.getReg16(cpu::AX);
    // Let XMS calls (AX=4300h/4310h) fall through to BIOS handler
    if (ax == 0x4300 || ax == 0x4310)
      return false;
    if (ax == 0x1687) {
      if (m_dpmi) {
        m_dpmi->handleDetect();
        LOG_DEBUG("DOS: INT 2Fh AX=1687h (DPMI query) → DPMI available");
      } else {
        LOG_DEBUG("DOS: INT 2Fh AX=1687h (DPMI query) returning NOT SUPPORTED");
        m_cpu.setReg8(cpu::AL, 0x00);
      }
    } else if (ax == 0x1686) {
      uint16_t mode = (m_cpu.getCR(0) & 1) ? 0x0001 : 0x0000;
      LOG_DEBUG("DOS: INT 2Fh AX=1686h (Get CPU Mode) -> ",
                (mode == 1 ? "Protected Mode" : "Real Mode"));
      m_cpu.setReg16(cpu::AX, mode);
    } else if (ax == 0x1680) {
      // Release time slice (no-op)
      m_cpu.setReg8(cpu::AL, 0x00);
    } else {
      LOG_DOS("DOS: INT 2Fh Multiplex (stubbed), AX=0x", std::hex, ax);
      m_cpu.setReg8(cpu::AL, 0x00);
    }
    return true;
  }
  case 0x31: { // DPMI
    uint16_t ax = m_cpu.getReg16(cpu::AX);
    LOG_DEBUG("[DPMI] INT 31h AX=0x", std::hex, ax);
    if (m_dpmi)
      return m_dpmi->handleInt31();
    return false;
  }
  case 0xE1: { // DPMI entry point (triggered by 0F FF E1 at F000:0050)
    if (m_dpmi)
      return m_dpmi->handleEntry();
    return false;
  }
  case 0xE2: { // DPMI raw mode switch PM→RM
    if (m_dpmi) {
      m_dpmi->handleRawSwitchPMtoRM();
      return true;
    }
    return false;
  }
  case 0xE3: { // DPMI raw mode switch RM→PM
    if (m_dpmi) {
      m_dpmi->handleRawSwitchRMtoPM();
      return true;
    }
    return false;
  }
  }
  return false;
}

void DOS::handleDOSService() {
  uint8_t ah = m_cpu.getReg8(cpu::AH);
  LOG_DEBUG("[DOS] INT 21h AH=0x", std::hex, (int)ah, " AL=0x",
            (int)m_cpu.getReg8(cpu::AL));
  switch (ah) {
  case 0x00: // Terminate Program (same as INT 20h)
    terminateProcess(0);
    return;
  case 0x01: { // Read Character with Echo (blocking)
    if (!m_kbd || !m_kbd->hasKey()) {
      if (m_pollInput)
        m_pollInput();
      if (!m_kbd || !m_kbd->hasKey()) {
        m_cpu.setEIP(m_cpu.getInstructionStartEIP());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return;
      }
    }
    {
      auto [ascii, scancode] = m_kbd->popKey();
      m_cpu.setReg8(cpu::AL, ascii);
      if (ascii >= 0x20)
        writeCharToVRAM(ascii);
    }
    break;
  }
  case 0x02: { // Print Character
    uint8_t c = m_cpu.getReg8(cpu::DL);
    writeCharToVRAM(c);
    break;
  }
  case 0x06: { // Direct Console I/O
    uint8_t dl = m_cpu.getReg8(cpu::DL);
    if (dl != 0xFF) {
      writeCharToVRAM(dl);
    } else {
      if (m_kbd && !m_kbd->hasKey() && m_pollInput)
        m_pollInput();
      if (m_idleCallback)
        m_idleCallback();
      if (m_kbd && m_kbd->hasKey()) {
        auto [ascii, scancode] = m_kbd->popKey();
        m_cpu.setReg8(cpu::AL, ascii);
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_ZERO);
      } else {
        m_cpu.setReg8(cpu::AL, 0);
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_ZERO);
      }
    }
    break;
  }
  case 0x07:   // Direct Character Input (no echo, no Ctrl-C check)
  case 0x08: { // Character Input (no echo, Ctrl-C check)
    if (!m_kbd || !m_kbd->hasKey()) {
      if (m_pollInput)
        m_pollInput();
      if (!m_kbd || !m_kbd->hasKey()) {
        if (m_idleCallback)
          m_idleCallback();
        m_cpu.setEIP(m_cpu.getInstructionStartEIP());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return;
      }
    }
    {
      auto [ascii, scancode] = m_kbd->popKey();
      m_cpu.setReg8(cpu::AL, ascii);
    }
    break;
  }
  case 0x09: {                              // Print String
    uint32_t dx = m_cpu.getReg32(cpu::EDX); // Supports 32-bit offset
    uint32_t addr = m_cpu.getSegBase(cpu::DS) + dx;

    std::string str = readDOSString(addr);
    for (uint8_t c : str)
      writeCharToVRAM(c);
    break;
  }
  case 0x0A: { // Buffered Keyboard Input
    uint32_t dx = m_cpu.getReg32(cpu::EDX);
    uint32_t bufAddr = m_cpu.getSegBase(cpu::DS) + dx;
    uint8_t maxLen = m_memory.read8(bufAddr); // First byte = max chars
    // For now, store empty input (0 chars read, CR terminated)
    m_memory.write8(bufAddr + 1, 0);    // Actual chars read = 0
    m_memory.write8(bufAddr + 2, 0x0D); // CR terminator
    LOG_DOS("DOS: Buffered Input (stubbed, max=", (int)maxLen, ")");
    break;
  }
  case 0x0B: { // Get Stdin Status
    if (m_kbd && !m_kbd->hasKey() && m_pollInput)
      m_pollInput();
    if (m_idleCallback)
      m_idleCallback();
    m_cpu.setReg8(cpu::AL, (m_kbd && m_kbd->hasKey()) ? 0xFF : 0x00);
    break;
  }
  case 0x0C: { // Flush Buffer and Read
    if (m_kbd) {
      while (m_kbd->hasKey())
        m_kbd->popKey();
    }
    uint8_t subFunc = m_cpu.getReg8(cpu::AL);
    if (subFunc == 0x01 || subFunc == 0x06 || subFunc == 0x07 ||
        subFunc == 0x08 || subFunc == 0x0A) {
      m_cpu.setReg8(cpu::AH, subFunc);
      handleDOSService();
    }
    break;
  }
  case 0x1A: { // Set DTA
    uint16_t ds = m_cpu.getSegReg(cpu::DS);
    uint16_t dx = m_cpu.getReg16(cpu::DX);
    m_dtaPtr = (static_cast<uint32_t>(ds) << 16) | dx;
    LOG_DOS("DOS: Set DTA to ", std::hex, ds, ":", dx);
    break;
  }
  case 0x2A: { // Get System Date
    // Return current date
    std::time_t t = std::time(nullptr);
    std::tm *now = std::localtime(&t);
    m_cpu.setReg16(cpu::CX, now->tm_year + 1900); // Year
    m_cpu.setReg8(cpu::DH, now->tm_mon + 1);      // Month
    m_cpu.setReg8(cpu::DL, now->tm_mday);         // Day
    m_cpu.setReg8(cpu::AL,
                  now->tm_wday); // Day of week (0=Sun, 1=Mon, ..., 6=Sat)
    break;
  }
  case 0x2B: { // Set System Date
    // Accept but ignore
    m_cpu.setReg8(cpu::AL, 0); // Success
    LOG_DOS("DOS: Set Date (ignored)");
    break;
  }
  case 0x51:   // Get Current PSP (undocumented)
  case 0x62: { // Get Current PSP
    // In PM, return the PSP selector (not the RM segment).
    // DPMI spec requires INT 21h results to use PM selectors.
    if (m_dpmi && m_dpmi->isActive()) {
      uint16_t pspSel = m_dpmi->getPSPSelector();
      m_cpu.setReg16(cpu::BX, pspSel);
      LOG_DOS("DOS: Get Current PSP (PM) -> sel 0x", std::hex, pspSel);
    } else {
      m_cpu.setReg16(cpu::BX, m_pspSegment);
      LOG_DOS("DOS: Get Current PSP -> 0x", std::hex, m_pspSegment);
    }
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    break;
  }
  case 0x2F: { // Get DTA Address
    uint16_t ds = static_cast<uint16_t>(m_dtaPtr >> 16);
    uint16_t bx = static_cast<uint16_t>(m_dtaPtr & 0xFFFF);
    m_cpu.setSegReg(cpu::ES, ds);
    m_cpu.setReg16(cpu::BX, bx);
    LOG_DOS("DOS: Get DTA -> ", std::hex, ds, ":", bx);
    break;
  }
  case 0x25: { // Set Interrupt Vector
    uint8_t intNum = m_cpu.getReg8(cpu::AL);
    uint16_t ds = m_cpu.getSegReg(cpu::DS);
    uint16_t dx = m_cpu.getReg16(cpu::DX);
    // Write vector
    m_memory.write16(intNum * 4, dx);
    m_memory.write16(intNum * 4 + 2, ds);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    LOG_DOS("DOS: Set INT vector 0x", std::hex, (int)intNum, " to ", ds, ":",
            dx);
    LOG_WARN("DOS: Set INT vector 0x", std::hex, (int)intNum, " to ", ds, ":",
             dx);
    break;
  }
  case 0x35: { // Get Interrupt Vector
    uint8_t intNum = m_cpu.getReg8(cpu::AL);
    uint16_t seg = m_memory.read16(intNum * 4 + 2);
    uint16_t off = m_memory.read16(intNum * 4);

    m_cpu.setSegReg(cpu::ES, seg);
    m_cpu.setReg16(cpu::BX, off);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    LOG_DOS("DOS: Get INT vector 0x", std::hex, (int)intNum, " -> ", seg, ":",
            off);
    break;
  }

  case 0x2E: { // Set/Reset Verify Flag (Not implemented)
    LOG_DOS("DOS: Set/Reset Verify Flag (AH=2Eh) - Not implemented");
    break;
  }
  case 0x0D: { // Disk Reset (Flush Buffers)
    // No-op: emulator has no disk cache to flush
    break;
  }
  case 0x0E: // Select Default Drive
  case 0x19: // Get Current Default Drive
  case 0x36: // Get Free Disk Space
    handleDriveService();
    break;
  case 0x37: { // Get/Set Switch Character
    uint8_t al = m_cpu.getReg8(cpu::AL);
    if (al == 0x00) {
      // Get switch character
      m_cpu.setReg8(cpu::DL, '/');  // Standard DOS switch char
      m_cpu.setReg8(cpu::AL, 0x00); // Success
      LOG_DOS("DOS: Get Switch Character -> '/'");
    } else if (al == 0x01) {
      // Set switch character — accept but ignore
      m_cpu.setReg8(cpu::AL, 0x00);
      LOG_DOS("DOS: Set Switch Character (ignored)");
    } else {
      m_cpu.setReg8(cpu::AL, 0xFF); // Invalid subfunction
    }
    break;
  }
  case 0x38: { // Get Country Dependent Information
    uint16_t dx = m_cpu.getReg16(cpu::DX);
    if (dx != 0xFFFF) {
      uint16_t ds = m_cpu.getSegReg(cpu::DS);
      uint32_t addr = (static_cast<uint32_t>(ds) << 4) + dx;

      // Write 34-byte US country info block
      m_memory.write16(addr + 0x00, 0); // Date format: 0 = MDY
      m_memory.write8(addr + 0x02, '$');
      m_memory.write8(addr + 0x03, 0); // Currency sym
      m_memory.write8(addr + 0x04, 0);
      m_memory.write8(addr + 0x05, 0);
      m_memory.write8(addr + 0x06, 0);
      m_memory.write8(addr + 0x07, ',');
      m_memory.write8(addr + 0x08, 0); // Thousands separator
      m_memory.write8(addr + 0x09, '.');
      m_memory.write8(addr + 0x0A, 0); // Decimal separator
      m_memory.write8(addr + 0x0B, '-');
      m_memory.write8(addr + 0x0C, 0); // Date separator
      m_memory.write8(addr + 0x0D, ':');
      m_memory.write8(addr + 0x0E, 0);  // Time separator
      m_memory.write8(addr + 0x0F, 0);  // Currency format
      m_memory.write8(addr + 0x10, 2);  // Digits after decimal
      m_memory.write8(addr + 0x11, 0);  // Time format: 0 = 12 hour
      m_memory.write32(addr + 0x12, 0); // Case map routine FAR pointer (null)
      m_memory.write8(addr + 0x16, ',');
      m_memory.write8(addr + 0x17, 0); // Data list separator
      for (int i = 0x18; i < 34; ++i)
        m_memory.write8(addr + i, 0); // Reserved

      m_cpu.setReg16(cpu::BX, 1); // US Country Code
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Get Country Information (US) at ", std::hex, ds, ":", dx);
    } else {
      m_cpu.setReg16(cpu::BX, 1); // US Country Code
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Set Country Code (ignored)");
    }
    break;
  }
  case 0x33: { // Get/Set Ctrl-Break Check State
    uint8_t al = m_cpu.getReg8(cpu::AL);
    if (al == 0x00) {
      // Get current state
      m_cpu.setReg8(cpu::DL, m_ctrlBreakCheck ? 1 : 0);
      LOG_DOS("DOS: Get Ctrl-Break Check -> ", (int)m_ctrlBreakCheck);
    } else if (al == 0x01) {
      // Set state
      m_ctrlBreakCheck = (m_cpu.getReg8(cpu::DL) != 0);
      LOG_DOS("DOS: Set Ctrl-Break Check -> ", (int)m_ctrlBreakCheck);
    } else if (al == 0x05) {
      // Get boot drive
      m_cpu.setReg8(cpu::DL, 3); // C: drive
      LOG_DOS("DOS: Get Boot Drive -> C:");
    }
    break;
  }
  case 0x30: { // Get DOS Version
    LOG_DOS("DOS: Get DOS Version (Reported: 5.0)");
    m_cpu.setReg8(cpu::AL, 5);       // Major 5
    m_cpu.setReg8(cpu::AH, 0);       // Minor 0
    m_cpu.setReg16(cpu::BX, 0xFF00); // OEM
    m_cpu.setReg16(cpu::CX, 0x0000);
    break;
  }
  case 0x44: { // IOCTL
    uint8_t al = m_cpu.getReg8(cpu::AL);
    uint16_t bx = m_cpu.getReg16(cpu::BX);
    if (al == 0x00) { // Get Device Info
      // Return device info word: bit 7=1 means character device
      // (stdin/stdout/stderr)
      uint16_t info = 0;
      if (bx == 0)
        info = 0x80 | 0x01; // STDIN  - char device, EOF on input
      else if (bx == 1)
        info = 0x80 | 0x02; // STDOUT - char device
      else if (bx == 2)
        info = 0x80 | 0x02; // STDERR - char device
      else if (bx >= 5 && bx - 5 < m_fileHandles.size() &&
               m_fileHandles[bx - 5]) {
        if (m_fileHandles[bx - 5]->isEMSDevice())
          info = 0x0080; // Character device
        else
          // Disk file: bit7=0 (file), bit11=1 (media not removable),
          // bits 5-0 = drive number (0=A:,2=C:).
          info = 0x0802;
      }
      else {
        m_cpu.setReg16(cpu::AX, 0x06); // Invalid handle
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
        break;
      }
      m_cpu.setReg16(cpu::DX, info);
      m_cpu.setReg16(cpu::AX, 0x0000); // AX destroyed on success
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: IOCTL Get Device Info handle=", bx, " info=0x", std::hex,
              info);
    } else if (al == 0x02) { // IOCTL Read from character device
      const uint16_t cx = m_cpu.getReg16(cpu::CX);
      const uint32_t bufAddr =
          m_cpu.getSegBase(cpu::DS) + m_cpu.getReg16(cpu::DX);

      if (!(bx >= 5 && bx - 5 < m_fileHandles.size() &&
            m_fileHandles[bx - 5])) {
        m_cpu.setReg16(cpu::AX, 0x06); // Invalid handle
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
        break;
      }

      const auto &fh = m_fileHandles[bx - 5];
      if (!fh->isEMSDevice()) {
        m_cpu.setReg16(cpu::AX, 0x01); // Function not supported
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
        LOG_DOS("DOS: IOCTL Read handle=", bx,
                " unsupported for non-device file");
        break;
      }

      if (cx < 6) {
        m_cpu.setReg16(cpu::AX, 0x0D); // Invalid data
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
        LOG_DOS("DOS: EMS IOCTL buffer too small, CX=", cx);
        break;
      }

      const uint8_t subfn = m_memory.read8(bufAddr);
      if (subfn == 0x00) {
        m_memory.write16(bufAddr + 0, 0x0025);
        m_memory.write16(bufAddr + 2, kEMSPrivateApiOffset);
        m_memory.write16(bufAddr + 4, kEMSPrivateApiSegment);
        m_cpu.setReg16(cpu::AX, 0x0006);
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
        LOG_DOS("DOS: EMS IOCTL subfn 00h -> API entry ", std::hex,
                kEMSPrivateApiSegment, ":", kEMSPrivateApiOffset);
      } else if (subfn == 0x01) {
        m_memory.write32(bufAddr + 0, kEMSImportRecordPhys);
        m_memory.write8(bufAddr + 4, kEMSImportMajorVersion);
        m_memory.write8(bufAddr + 5, kEMSImportMinorVersion);
        m_cpu.setReg16(cpu::AX, 0x0006);
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
        LOG_DOS("DOS: EMS IOCTL subfn 01h -> import record phys=0x", std::hex,
                kEMSImportRecordPhys);
      } else {
        m_cpu.setReg16(cpu::AX, 0x01); // Function not supported
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
        LOG_DOS("DOS: EMS IOCTL subfn=0x", std::hex, (int)subfn,
                " not implemented");
      }
    } else {
      // Other IOCTL subfunction – stub as unsupported
      m_cpu.setReg16(cpu::AX, 0x01); // Function not supported
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      LOG_DOS("DOS: IOCTL AL=", (int)al, " not implemented");
    }
    break;
  }
  case 0x3C: // Create File
  case 0x3D: // Open File
  case 0x3E: // Close File
  case 0x3F: // Read File/Device
  case 0x40: // Write File/Device
  case 0x41: // Delete File
  case 0x42: // Move File Pointer
  case 0x43: // Get/Set File Attributes
  case 0x45: // Duplicate File Handle
  case 0x56: // Rename File
  case 0x57: // Get/Set File Date and Time
    handleFileService();
    break;
  case 0x4B: { // Exec
    uint8_t mode = m_cpu.getReg8(cpu::AL);
    uint32_t dx = m_cpu.getReg32(cpu::EDX);
    std::string filename =
        resolvePath(readFilename(m_cpu.getSegBase(cpu::DS) + dx));
    LOG_DOS("DOS: Exec '", filename, "' mode ", (int)mode);
    m_cpu.setReg16(cpu::AX, 0x02); // File not found (simplified stub)
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    break;
  }
  case 0x2C: { // Get System Time
    LOG_DOS("DOS: Get System Time (stub)");
    m_cpu.setReg8(cpu::CH, 12); // Hour
    m_cpu.setReg8(cpu::CL, 0);  // Minute
    m_cpu.setReg8(cpu::DH, 0);  // Second
    m_cpu.setReg8(cpu::DL, 0);  // 1/100 second
    break;
  }

  case 0x4C: { // Terminate with return code
    uint8_t al = m_cpu.getReg8(cpu::AL);
    terminateProcess(al);
    break;
  }
  case 0x48: // Allocate Memory
  case 0x49: // Free Memory
  case 0x4A: // Resize Block
    LOG_WARN("DOS: Memory management AH=0x", std::hex, (int)ah, " ES=0x",
             m_cpu.getSegReg(cpu::ES), " BX=0x", m_cpu.getReg16(cpu::BX));
    handleMemoryManagement();
    break;

  case 0x39: // Create Directory (mkdir)
  case 0x3A: // Remove Directory (rmdir)
  case 0x3B: // Change Directory (chdir)
  case 0x47: // Get Current Directory
    handleDirectoryService();
    break;
  case 0x4E: // Find First
  case 0x4F: // Find Next
    handleDirectorySearch();
    break;
  case 0x34: { // Get InDOS flag address
    // Return ES:BX pointing to the InDOS flag (1 byte)
    // We'll put it at a static location in the BIOS data area or similar.
    m_cpu.setSegReg(cpu::ES, 0x0070);
    m_cpu.setReg16(cpu::BX, 0x0010);
    m_memory.write8(0x0710, 0); // InDOS = 0
    LOG_DOS("DOS: Get InDOS flag at 0070:0010");
    break;
  }
  case 0x52: { // "Get List of Lists" (SYSVARS)
    // Pointer to first MCB is at [ES:BX-2]
    // We'll put the LoL at 0x0050:0020
    uint16_t lolSeg = 0x0050;
    uint16_t lolOff = 0x0020;
    m_cpu.setSegReg(cpu::ES, lolSeg);
    m_cpu.setReg16(cpu::BX, lolOff);

    uint32_t lolPhys = (lolSeg << 4) + lolOff;
    // Set first MCB segment at [LoL-2]
    m_memory.write16(lolPhys - 2, FIRST_MCB_SEGMENT);
    // Other fields (mostly dummy for now)
    // DPB, SFT, etc. - DOS extenders usually only need the MCB pointer.
    LOG_DOS("DOS: Get List of Lists at ", std::hex, lolSeg, ":", lolOff,
            " (First MCB: ", FIRST_MCB_SEGMENT, ")");
    break;
  }
  default:
    LOG_WARN("DOS: Unknown INT 21h function AH=0x", std::hex, (int)ah);
    break;
  }
}
void DOS::terminateProcess(uint8_t exitCode) {
  LOG_INFO("DOS: Process terminated with exit code ", (int)exitCode);

  m_terminated = true;
  m_exitCode = exitCode;
}

std::string DOS::readDOSString(uint32_t address) {
  std::string result;
  while (true) {
    uint8_t c = m_memory.read8(address++);
    if (c == '$' || result.length() > 255)
      break; // DOS strings end with '$'
    result += (char)c;
  }
  return result;
}

std::string DOS::readFilename(uint32_t address) {
  std::string result;
  while (true) {
    uint8_t c = m_memory.read8(address++);
    if (c == 0 || result.length() > 255)
      break;
    result += (char)c;
  }
  return result;
}

void DOS::setProgramDir(const std::string &programPath) {
  fs::path p = fs::absolute(programPath);
  m_programPath = p.string();
  m_currentDir = p.parent_path().string();
  // Drive root = parent of program directory, DOS current dir = program dir
  // name
  fs::path progDir = p.parent_path();
  m_hostRootDir = progDir.parent_path().string();
  if (m_hostRootDir.empty())
    m_hostRootDir = progDir.string();
  // DOS current directory = last component of host path, uppercased
  std::string dirName = progDir.filename().string();
  std::transform(dirName.begin(), dirName.end(), dirName.begin(), ::toupper);
  m_dosCurrentDir = dirName;
  LOG_INFO("DOS: Working directory set to '", m_currentDir, "'");
  LOG_INFO("DOS: Drive root = '", m_hostRootDir, "', DOS dir = '",
           m_dosCurrentDir, "'");
}

std::string DOS::hostToDosPath(const std::string &hostPath) {
  // Convert a host filesystem path to a DOS-style path relative to the drive
  // root
  fs::path host(hostPath);
  fs::path root(m_hostRootDir);
  std::string result;
  // Try to make the path relative to the drive root
  auto rel = fs::relative(host, root);
  if (!rel.empty() && rel.string().find("..") != 0) {
    result = rel.string();
  } else {
    // Fallback: use just the last components
    result = host.filename().string();
  }
  // Convert separators to backslash and uppercase
  std::transform(result.begin(), result.end(), result.begin(), [](char c) {
    return c == '/'
               ? '\\'
               : static_cast<char>(::toupper(static_cast<unsigned char>(c)));
  });
  return result;
}

std::string DOS::dosToHostPath(const std::string &dosPath) {
  // Convert a DOS-style path to a host filesystem path
  std::string path = dosPath;

  // Strip drive letter prefix (e.g., "C:\")
  if (path.size() >= 2 && std::isalpha(static_cast<unsigned char>(path[0])) &&
      path[1] == ':') {
    path = path.substr(2);
  }
  // Convert backslashes to forward slashes
  std::transform(path.begin(), path.end(), path.begin(),
                 [](char c) { return c == '\\' ? '/' : c; });
  // If path starts with /, it's absolute from drive root
  if (!path.empty() && path[0] == '/') {
    return (fs::path(m_hostRootDir) / path.substr(1)).string();
  }
  // Relative path — resolve from current directory
  return (fs::path(m_currentDir) / path).string();
}

std::string DOS::resolvePath(const std::string &path) {
  std::string resolvedStr;
  // Check for DOS drive letter prefix (e.g. "C:\foo")
  if (path.size() >= 2 && std::isalpha(static_cast<unsigned char>(path[0])) &&
      path[1] == ':') {
    resolvedStr = dosToHostPath(path);
  }
  // Check for DOS backslash paths
  else if (!path.empty() && path[0] == '\\') {
    resolvedStr = dosToHostPath(path);
  }
  else {
    // If path is already an absolute host path, use as-is
    fs::path p(path);
    if (p.is_absolute())
      resolvedStr = path;
    else
      // Resolve relative to m_currentDir
      resolvedStr = (fs::path(m_currentDir) / p).string();
  }

  // Security check: ensure the resolved path stays within m_hostRootDir
  fs::path resolvedPath = fs::weakly_canonical(fs::absolute(resolvedStr));
  fs::path rootPath = fs::weakly_canonical(fs::absolute(m_hostRootDir));

  auto [rootEnd, nothing] = std::mismatch(rootPath.begin(), rootPath.end(), resolvedPath.begin(), resolvedPath.end());
  if (rootEnd != rootPath.end()) {
      LOG_WARN("DOS: Path traversal attempt blocked: ", path);
      return rootPath.string();
  }

  return resolvedPath.string();
}

void DOS::writeCharToVRAM(uint8_t c) {
  uint16_t cols = m_memory.read16(0x44A);
  uint8_t maxRow = m_memory.read8(0x484);
  if (cols == 0)
    cols = 80;
  if (maxRow == 0)
    maxRow = 24;

  uint8_t col = m_memory.read8(0x450);
  uint8_t row = m_memory.read8(0x451);

  switch (c) {
  case 0x07:
    break;   // BEL
  case 0x08: // Backspace
    if (col > 0)
      col--;
    break;
  case 0x0A: // Line Feed
    row++;
    break;
  case 0x0D: // Carriage Return
    col = 0;
    break;
  default: {
    uint32_t off = (row * cols + col) * 2;
    m_memory.write8(0xB8000 + off, c);
    uint8_t attr = m_memory.read8(0xB8000 + off + 1);
    if (attr == 0)
      m_memory.write8(0xB8000 + off + 1, 0x07);
    col++;
    if (col >= cols) {
      col = 0;
      row++;
    }
    break;
  }
  }

  // Scroll up if past last row
  if (row > maxRow) {
    for (uint8_t r = 0; r < maxRow; ++r) {
      for (uint16_t cc = 0; cc < cols; ++cc) {
        uint32_t dst = (r * cols + cc) * 2;
        uint32_t src = ((r + 1) * cols + cc) * 2;
        m_memory.write8(0xB8000 + dst, m_memory.read8(0xB8000 + src));
        m_memory.write8(0xB8000 + dst + 1, m_memory.read8(0xB8000 + src + 1));
      }
    }
    for (uint16_t cc = 0; cc < cols; ++cc) {
      uint32_t off = (maxRow * cols + cc) * 2;
      m_memory.write8(0xB8000 + off, ' ');
      m_memory.write8(0xB8000 + off + 1, 0x07);
    }
    row = maxRow;
  }

  m_memory.write8(0x450, col);
  m_memory.write8(0x451, row);
}

DOS::MCB DOS::readMCB(uint16_t segment) {
  uint32_t addr = (segment << 4);
  MCB mcb;
  mcb.type = m_memory.read8(addr + 0);
  mcb.owner = m_memory.read16(addr + 1);
  mcb.size = m_memory.read16(addr + 3);
  for (int i = 0; i < 8; ++i)
    mcb.name[i] = (char)m_memory.read8(addr + 8 + i);
  return mcb;
}

void DOS::writeMCB(uint16_t segment, const MCB &mcb) {
  uint32_t addr = (segment << 4);
  m_memory.write8(addr + 0, mcb.type);
  m_memory.write16(addr + 1, mcb.owner);
  m_memory.write16(addr + 3, mcb.size);
  for (int i = 0; i < 8; ++i)
    m_memory.write8(addr + 8 + i, (uint8_t)mcb.name[i]);
}

void DOS::handleMemoryManagement() {
  uint8_t ah = m_cpu.getReg8(cpu::AH);
  dumpMCBChain();
  if (ah == 0x48) { // Allocate
    uint16_t requested = m_cpu.getReg16(cpu::BX);
    uint16_t current = FIRST_MCB_SEGMENT;
    uint16_t bestFit = 0;
    uint16_t bestFitSize = 0xFFFF;
    uint16_t largestFree = 0;
    int mcbCount = 0;

    while (true) {
      if (mcbCount++ > 1000) {
        LOG_ERROR("DOS: MCB chain corrupted or too long (infinite loop?)");
        break;
      }
      MCB mcb = readMCB(current);
      LOG_DEBUG("DOS: Checking MCB at 0x", std::hex, current,
                " type=", (char)mcb.type, " owner=0x", mcb.owner, " size=0x",
                mcb.size);

      if (mcb.owner == 0) {
        if (mcb.size > largestFree)
          largestFree = mcb.size;
        if (mcb.size >= requested && mcb.size < bestFitSize) {
          bestFit = current;
          bestFitSize = mcb.size;
        }
      }
      if (mcb.type == 'Z')
        break;
      uint32_t next = (uint32_t)current + mcb.size + 1;
      if (next > 0xFFFF || next > LAST_PARA) {
        LOG_DEBUG("DOS: MCB chain ends/wraps at 0x", std::hex, current,
                  " size 0x", mcb.size);
        break;
      }
      current = static_cast<uint16_t>(next);
    }

    if (bestFit != 0) {
      MCB mcb = readMCB(bestFit);
      LOG_DEBUG("DOS: Splitting MCB at 0x", std::hex, bestFit, " requested 0x",
                requested, " total 0x", mcb.size);
      if (mcb.size > requested + 1) { // Split
        MCB next;
        next.type = mcb.type;
        next.owner = 0;
        next.size = mcb.size - requested - 1;

        mcb.type = 'M';
        mcb.size = requested;
        mcb.owner = m_pspSegment;

        writeMCB(bestFit, mcb);
        uint16_t nextMcbSeg = static_cast<uint16_t>(bestFit + requested + 1);
        writeMCB(nextMcbSeg, next);
        LOG_DEBUG("DOS: Created new free MCB at 0x", std::hex, nextMcbSeg,
                  " size 0x", next.size);
      } else {
        mcb.owner = m_pspSegment;
        writeMCB(bestFit, mcb);
      }
      m_cpu.setReg16(cpu::AX, bestFit + 1);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DEBUG("DOS: Allocated 0x", std::hex, requested, " paras at 0x",
                bestFit + 1);
    } else {
      m_cpu.setReg16(cpu::AX, 0x0008);      // Insufficient memory
      m_cpu.setReg16(cpu::BX, largestFree); // Largest available block
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      LOG_WARN("DOS: Failed to allocate ", std::hex, requested,
               " paras (largest free: 0x", largestFree, ")");
      dumpMCBChain();
    }
  } else if (ah == 0x49) { // Free
    uint16_t segment = m_cpu.getSegReg(cpu::ES);
    uint16_t mcbSeg = segment - 1;
    MCB mcb = readMCB(mcbSeg);
    if (mcb.type == 'M' || mcb.type == 'Z') {
      mcb.owner = 0;
      writeMCB(mcbSeg, mcb);

      // Simplified merge: only merge with next if free
      if (mcb.type == 'M') {
        uint16_t nextSeg = mcbSeg + mcb.size + 1;
        MCB next = readMCB(nextSeg);
        if (next.owner == 0) {
          mcb.type = next.type;
          mcb.size = static_cast<uint16_t>(mcb.size + next.size + 1);
          writeMCB(mcbSeg, mcb);
        }
      }
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DEBUG("DOS: Freed segment ", std::hex, segment);
    } else {
      m_cpu.setReg16(cpu::AX, 0x09); // Memory block address invalid
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      LOG_ERROR("DOS: Failed to free invalid segment ", std::hex, segment);
    }
  } else if (ah == 0x4A) { // Resize
    uint16_t segment = m_cpu.getSegReg(cpu::ES);
    uint16_t requested = m_cpu.getReg16(cpu::BX);
    uint16_t mcbSeg = segment - 1;
    MCB mcb = readMCB(mcbSeg);

    if (mcb.size >= requested) { // Shrink
      if (mcb.size > requested + 1) {
        // Compute the size of the new free block
        uint16_t freeSize = mcb.size - requested - 1;
        uint8_t  freeType = mcb.type; // inherit 'M' or 'Z'

        // Merge with the following block if it is also free
        uint16_t newFreeSeg = static_cast<uint16_t>(mcbSeg + requested + 1);
        if (freeType == 'M') {
          uint16_t afterSeg = static_cast<uint16_t>(newFreeSeg + freeSize + 1);
          if (afterSeg <= LAST_PARA) {
            MCB afterMcb = readMCB(afterSeg);
            if (afterMcb.owner == 0) {
              // Merge: absorb the next free block
              freeSize = static_cast<uint16_t>(freeSize + 1 + afterMcb.size);
              freeType = afterMcb.type; // inherit 'Z' if it was last
            }
          }
        }

        MCB next;
        next.type = freeType;
        next.owner = 0;
        next.size = freeSize;

        mcb.type = 'M';
        mcb.size = requested;

        writeMCB(mcbSeg, mcb);
        writeMCB(newFreeSeg, next);
      }
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DEBUG("DOS: Shrunk segment ", std::hex, segment, " to ", requested,
                " paras");
    } else {
      // Expand (only if next is free)
      if (mcb.type == 'M') {
        uint16_t nextSeg = mcbSeg + mcb.size + 1;
        MCB next = readMCB(nextSeg);
        if (next.owner == 0 && (mcb.size + 1 + next.size) >= requested) {
          uint16_t combinedSize = mcb.size + 1 + next.size;
          if (combinedSize > requested + 1) {
            MCB newNext;
            newNext.type = next.type;
            newNext.owner = 0;
            newNext.size = combinedSize - requested - 1;

            mcb.type = 'M';
            mcb.size = requested;
            writeMCB(mcbSeg, mcb);
            writeMCB(static_cast<uint16_t>(mcbSeg + requested + 1), newNext);
          } else {
            mcb.type = next.type;
            mcb.size = static_cast<uint16_t>(combinedSize);
            writeMCB(mcbSeg, mcb);
          }
          m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
          LOG_DEBUG("DOS: Expanded segment ", std::hex, segment, " to ",
                    requested, " paras");
          return;
        }
      }
      m_cpu.setReg16(cpu::AX, 0x08); // Insufficient memory
      uint16_t nextSeg = mcbSeg + mcb.size + 1;
      MCB next = readMCB(nextSeg);
      uint16_t maxPossible = mcb.size;
      if (next.owner == 0)
        maxPossible += 1 + next.size;
      m_cpu.setReg16(cpu::BX, maxPossible);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      LOG_WARN("DOS: Failed to expand segment ", std::hex, segment, " to ",
               requested, " paras (max: 0x", maxPossible, ")");
      dumpMCBChain();
    }
  } else if (ah == 0x58) { // Get/Set Allocation Strategy
    uint8_t al = m_cpu.getReg8(cpu::AL);
    if (al == 0x00) { // Get strategy
      m_cpu.setReg16(cpu::AX, m_allocationStrategy);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DEBUG("DOS: Get Allocation Strategy -> ", m_allocationStrategy);
    } else if (al == 0x01) { // Set strategy
      m_allocationStrategy = m_cpu.getReg16(cpu::BX);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DEBUG("DOS: Set Allocation Strategy to ", m_allocationStrategy);
    } else if (al == 0x02) { // Get UMB link state
      m_cpu.setReg8(cpu::AL, m_umbLinked ? 1 : 0);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DEBUG("DOS: Get UMB Link State -> ", (int)m_umbLinked);
    } else if (al == 0x03) { // Set UMB link state
      m_umbLinked = (m_cpu.getReg8(cpu::BX) != 0);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DEBUG("DOS: Set UMB Link State to ", (int)m_umbLinked);
    } else {
      m_cpu.setReg16(cpu::AX, 0x0001); // Invalid function
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      LOG_WARN("DOS: Unknown AH=58h AL=", std::hex, (int)al);
    }
  }
}

void DOS::handleDirectoryService() {
  uint8_t ah = m_cpu.getReg8(cpu::AH);
  uint16_t dx = m_cpu.getReg16(cpu::DX);
  uint32_t addr = m_cpu.getSegBase(cpu::DS) + dx;

  try {
    if (ah == 0x39) { // MKDIR
      std::string path = resolvePath(readFilename(addr));
      LOG_DEBUG("DOS: MKDIR '", path, "'");
      if (fs::create_directory(path)) {
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
        m_cpu.setReg16(cpu::AX, 0);
      } else {
        m_cpu.setReg16(cpu::AX, 0x03); // Path not found or already exists
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      }
    } else if (ah == 0x3A) { // RMDIR
      std::string path = resolvePath(readFilename(addr));
      LOG_DEBUG("DOS: RMDIR '", path, "'");
      if (fs::remove(path)) {
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
        m_cpu.setReg16(cpu::AX, 0);
      } else {
        m_cpu.setReg16(cpu::AX,
                       0x05); // Access denied (or not empty/doesn't exist)
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      }
    } else if (ah == 0x3B) { // CHDIR
      std::string rawPath = readFilename(addr);
      std::string path = resolvePath(rawPath);
      LOG_DEBUG("DOS: CHDIR raw='", rawPath, "' resolved='", path, "'");
      if (fs::exists(path) && fs::is_directory(path)) {
        m_currentDir = path;
        // Update DOS-style current directory
        m_dosCurrentDir = hostToDosPath(path);
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
        m_cpu.setReg16(cpu::AX, 0);
      } else {
        m_cpu.setReg16(cpu::AX, 0x03); // Path not found
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      }
    } else if (ah == 0x47) { // GETCWD
      uint16_t si = m_cpu.getReg16(cpu::SI);
      uint32_t outAddr = m_cpu.getSegBase(cpu::DS) + si;

      // DOS returns path relative to drive root (no leading backslash, no drive
      // letter)
      std::string path = m_dosCurrentDir;

      LOG_DEBUG("DOS: GETCWD returning '", path, "'");
      for (size_t i = 0; i < path.length(); ++i) {
        m_memory.write8(outAddr + i, static_cast<uint8_t>(path[i]));
      }
      m_memory.write8(outAddr + path.length(), 0); // Null terminator

      m_cpu.setReg16(cpu::AX, 0x0100);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    }
  } catch (const std::exception &e) {
    LOG_ERROR("DOS Directory Error: ", e.what());
    m_cpu.setReg16(cpu::AX, 0x05); // General failure
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
  }
}

void DOS::handleDriveService() {
  uint8_t ah = m_cpu.getReg8(cpu::AH);
  if (ah == 0x0E) { // Select Default Drive
    uint8_t drive = m_cpu.getReg8(cpu::DL);
    m_currentDrive = drive;
    LOG_DEBUG("DOS: Selected drive ", (int)m_currentDrive);
    m_cpu.setReg8(cpu::AL,
                  5); // DOS often returns total logical drives (e.g., 5: A-E)
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
  } else if (ah == 0x19) { // Get Current Default Drive
    m_cpu.setReg8(cpu::AL, m_currentDrive);
    LOG_DEBUG("DOS: Current drive is ", (int)m_currentDrive);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
  } else if (ah == 0x36) { // Get Free Disk Space
    uint8_t drive = m_cpu.getReg8(cpu::DL);
    if (drive == 0)
      drive = m_currentDrive + 1;

    LOG_DEBUG("DOS: Get Free Disk Space for drive ", (int)drive);

    // Mock 1GB total, 500MB free
    // AX = Sectors per cluster
    // BX = Number of free clusters
    // CX = Bytes per sector
    // DX = Total clusters
    m_cpu.setReg16(cpu::AX, 32);    // 32 sectors/cluster (16KB clusters)
    m_cpu.setReg16(cpu::BX, 32768); // 32768 free clusters (512MB)
    m_cpu.setReg16(cpu::CX, 512);   // 512 bytes/sector
    m_cpu.setReg16(cpu::DX, 65535); // 65535 total clusters (~1GB)
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
  }
}

// Case-insensitive DOS wildcard match: '?' matches any single char, '*' matches
// any sequence
static bool dosWildcardMatch(const std::string &pattern,
                             const std::string &name) {
  size_t pi = 0, ni = 0;
  size_t starP = std::string::npos, starN = 0;
  while (ni < name.size()) {
    if (pi < pattern.size() &&
        (pattern[pi] == '?' ||
         std::tolower(static_cast<unsigned char>(pattern[pi])) ==
             std::tolower(static_cast<unsigned char>(name[ni])))) {
      ++pi;
      ++ni;
    } else if (pi < pattern.size() && pattern[pi] == '*') {
      starP = pi++;
      starN = ni;
    } else if (starP != std::string::npos) {
      pi = starP + 1;
      ni = ++starN;
    } else {
      return false;
    }
  }
  while (pi < pattern.size() && pattern[pi] == '*')
    ++pi;
  return pi == pattern.size();
}

void DOS::handleDirectorySearch() {
  uint8_t ah = m_cpu.getReg8(cpu::AH);
  // DTA pointer is stored as a segmented address. In PM, its interpretation
  // is tricky. For now, we assume it's a RM pointer or a legacy PM pointer.
  uint16_t dtaSeg = m_dtaPtr >> 16;
  uint32_t dtaOff = m_dtaPtr & 0xFFFFFFFF;
  // If in PM, we might need more complex logic, but usually DTA is RM-segment
  // compatible in early DOS/4GW.
  uint32_t dtaAddr = (static_cast<uint32_t>(dtaSeg) << 4) + dtaOff;

  auto fillDTA = [&](const fs::path &fullPath, const std::string &dosName) {
    // Byte 0x15: file attributes
    std::error_code ec;
    uint8_t dosAttr = 0x20; // Archive
    if (fs::is_directory(fullPath, ec))
      dosAttr = 0x10;
    auto perms = fs::status(fullPath, ec).permissions();
    if ((perms & fs::perms::owner_write) == fs::perms::none)
      dosAttr |= 0x01;
    m_memory.write8(dtaAddr + 0x15, dosAttr);

    // Bytes 0x16-0x17: file time, 0x18-0x19: file date
    auto ftime = fs::last_write_time(fullPath, ec);
    (void)ftime; // Mark ftime as unused to fix compiler warning
    // Use a plausible default timestamp
    uint16_t dosTime = (12 << 11) | (0 << 5) | 0;           // 12:00:00
    uint16_t dosDate = ((2026 - 1980) << 9) | (3 << 5) | 2; // 2026-03-02
    m_memory.write16(dtaAddr + 0x16, dosTime);
    m_memory.write16(dtaAddr + 0x18, dosDate);

    // Bytes 0x1A-0x1D: file size (32-bit)
    uint32_t fileSize = 0;
    if (fs::is_regular_file(fullPath, ec))
      fileSize = static_cast<uint32_t>(fs::file_size(fullPath, ec));
    m_memory.write32(dtaAddr + 0x1A, fileSize);

    // Bytes 0x1E-0x2A: filename (null-terminated, uppercase 8.3)
    std::string upper = dosName;
    for (auto &ch : upper)
      ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    for (int i = 0; i < 13; ++i) {
      uint8_t c = (static_cast<size_t>(i) < upper.size())
                      ? static_cast<uint8_t>(upper[i])
                      : 0;
      m_memory.write8(dtaAddr + 0x1E + i, c);
    }
  };

  if (ah == 0x4E) { // Find First
    uint16_t dx = m_cpu.getReg16(cpu::DX);
    uint32_t nameAddr = m_cpu.getSegBase(cpu::DS) + dx;
    std::string rawPattern = readFilename(nameAddr);
    uint8_t attr = m_cpu.getReg8(cpu::CL);

    // Separate directory and filename pattern
    fs::path resolved = fs::path(resolvePath(rawPattern));
    std::string searchDir;
    std::string filePattern;
    if (fs::is_directory(resolved)) {
      searchDir = resolved.string();
      filePattern = "*.*";
    } else {
      searchDir = resolved.parent_path().string();
      filePattern = resolved.filename().string();
    }
    if (searchDir.empty())
      searchDir = m_currentDir;

    LOG_DEBUG("DOS: FindFirst raw='", rawPattern, "' dir='", searchDir,
              "' pattern='", filePattern, "' attr 0x", std::hex, (int)attr);

    // Collect matching files
    m_searchResults.clear();
    m_searchIndex = 0;
    m_searchPattern = filePattern;
    m_searchDir = searchDir;

    try {
      for (const auto &entry : fs::directory_iterator(searchDir)) {
        std::string fname = entry.path().filename().string();
        // Skip . and ..
        if (fname == "." || fname == "..")
          continue;
        // Attribute filtering: skip dirs unless attr includes 0x10
        if (fs::is_directory(entry.path()) && !(attr & 0x10))
          continue;

        if (dosWildcardMatch(filePattern, fname)) {
          m_searchResults.push_back(entry.path());
        }
      }
    } catch (...) {
      m_cpu.setReg16(cpu::AX, 0x03); // Path not found
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      return;
    }

    if (m_searchResults.empty()) {
      m_cpu.setReg16(cpu::AX, 0x12); // No more files
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      LOG_DEBUG("DOS: FindFirst no matches for '", filePattern, "' in '",
                searchDir, "'");
      return;
    }

    // Store search state in DTA reserved area
    m_memory.write16(dtaAddr + 0x00, 0); // match index = 0

    fillDTA(m_searchResults[0], m_searchResults[0].filename().string());
    LOG_DOS("DOS: FindFirst matched '", m_searchResults[0].filename().string(),
            "' (", m_searchResults.size(), " total)");

    m_cpu.setReg16(cpu::AX, 0);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);

  } else if (ah == 0x4F) { // Find Next
    uint16_t index = m_memory.read16(dtaAddr + 0x00);
    index++;

    if (index >= m_searchResults.size()) {
      m_cpu.setReg16(cpu::AX, 0x12); // No more files
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      LOG_DEBUG("DOS: FindNext end of results");
      return;
    }

    m_memory.write16(dtaAddr + 0x00, index);

    fillDTA(m_searchResults[index], m_searchResults[index].filename().string());
    LOG_DOS("DOS: FindNext matched '",
            m_searchResults[index].filename().string(), "'");

    m_cpu.setReg16(cpu::AX, 0);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
  }
}

void DOS::dumpMCBChain() {
  uint16_t current = FIRST_MCB_SEGMENT;
  LOG_INFO("--- DOS MCB Chain Dump ---");
  while (true) {
    MCB mcb = readMCB(current);
    LOG_INFO("MCB [", (char)mcb.type, "] Seg: 0x", std::hex, current,
             " Owner: 0x", mcb.owner, " Size: ", std::dec, mcb.size, " paras");
    if (mcb.type == 'Z')
      break;
    current = static_cast<uint16_t>(current + mcb.size + 1);
    if (current >= LAST_PARA)
      break;
  }
}

void DOS::clearOverlayInfo() {
  m_programPath.clear();
  m_neAlignShift = 0;
  m_neInitialLoadSegment = 0;
  m_neSegments.clear();
  m_fbovOverlays.clear();
}

void DOS::setNEInfo(const std::string &path, uint16_t alignShift,
                    const std::vector<NESegment> &segments,
                    uint16_t initialLoadSegment) {
  m_programPath = path;
  m_neAlignShift = alignShift;
  m_neSegments = segments;
  m_neInitialLoadSegment = initialLoadSegment;

  // Mark the first code segment as loaded if it's already in memory
  // (Actually, ProgramLoader loads the first segment of code to
  // m_neInitialLoadSegment)
  if (!m_neSegments.empty()) {
    m_neSegments[0].loadedSegment = m_neInitialLoadSegment;
  }

  LOG_INFO("DOS: VROOMM Registered ", segments.size(), " segments. Initial: 0x",
           std::hex, initialLoadSegment, " AlignShift: ", alignShift);
}

void DOS::setFBOVInfo(const std::string &path,
                      const std::vector<FBOVOverlay> &overlays) {
  m_programPath = path;
  m_fbovOverlays = overlays;

  LOG_INFO("DOS: FBOV Registered ", overlays.size(), " overlay blocks for ",
           path);
  for (const auto &overlay : m_fbovOverlays) {
    LOG_DOS("DOS: FBOV overlay id=", std::dec, overlay.overlayId,
            " fileOff=0x", std::hex, overlay.fileOffset, " size=0x",
            overlay.fileSize, " auxOff=0x", overlay.auxOffset,
            " auxCount=0x", overlay.auxCount);
  }
}

uint16_t DOS::loadOverlaySegment(uint16_t segIndex) {
  LOG_DEBUG("DOS: loadOverlaySegment(", std::dec, segIndex, ")");
  if (segIndex >= m_neSegments.size()) {
    LOG_ERROR("DOS: loadOverlaySegment out of bounds: ", segIndex);
    return 0;
  }
  if (m_neSegments[segIndex].loadedSegment != 0)
    return m_neSegments[segIndex].loadedSegment;

  auto &seg = m_neSegments[segIndex];
  LOG_DOS("DOS: Loading overlay segment ", std::dec, segIndex + 1, " from ",
          m_programPath);

  std::ifstream file(m_programPath, std::ios::binary);
  if (!file) {
    LOG_ERROR("DOS: Failed to open program file for overlay loading: ",
              m_programPath);
    return 0;
  }

  uint32_t fileOffset = static_cast<uint32_t>(seg.fileOffsetSector)
                        << m_neAlignShift;
  uint32_t sizeInFile = seg.length == 0 ? 0x10000 : seg.length;
  uint32_t memSize = seg.minAlloc == 0 ? 0x10000 : seg.minAlloc;
  if (sizeInFile > memSize)
    memSize = sizeInFile;

  LOG_DOS("DOS: Overlay seg ", std::dec, segIndex + 1, ": sector=0x", std::hex,
          seg.fileOffsetSector, " (fileOff=0x", fileOffset, ") len=0x",
          seg.length, " flags=0x", seg.flags, " minAlloc=0x", seg.minAlloc);

  uint16_t paragraphs = (uint16_t)((memSize + 15) / 16);

  // Allocate memory using AH=48h logic
  m_cpu.setReg8(cpu::AH, 0x48);
  m_cpu.setReg16(cpu::BX, paragraphs);
  handleMemoryManagement();

  if (m_cpu.getEFLAGS() & cpu::FLAG_CARRY) {
    LOG_ERROR("DOS: Failed to allocate memory (", std::dec, paragraphs,
              " paras) for overlay segment ", segIndex + 1);
    return 0;
  }

  uint16_t targetSegment = (uint16_t)m_cpu.getReg16(cpu::AX);
  seg.loadedSegment = targetSegment;

  file.seekg(fileOffset, std::ios::beg);
  std::vector<uint8_t> buffer(paragraphs * 16, 0);
  file.read(reinterpret_cast<char *>(buffer.data()), sizeInFile);

  // Check for relocation information (flag 0x0100)
  if (seg.flags & 0x0100) {
    LOG_DOS("DOS: Segment ", std::dec, segIndex + 1,
            " has relocation information.");
    file.seekg(fileOffset + sizeInFile, std::ios::beg);
    uint16_t numRelocs = 0;
    file.read(reinterpret_cast<char *>(&numRelocs), 2);
    LOG_DOS("DOS: Applying ", std::dec, numRelocs, " fixups...");

    for (int i = 0; i < numRelocs; ++i) {
      uint8_t r[8];
      file.read(reinterpret_cast<char *>(r), 8);

      uint8_t sourceType =
          r[0]; // Standard NE: 0=LoByte, 2=Segment, 3=Far Addr, 5=Offset
      uint8_t flags = r[1];
      uint16_t nextOffset = *reinterpret_cast<uint16_t *>(&r[2]);
      uint16_t targetSegIdx =
          *reinterpret_cast<uint16_t *>(&r[4]); // 1-based index
      uint16_t targetOffset = *reinterpret_cast<uint16_t *>(&r[6]);

      uint8_t relocType = flags & 0x03;
      bool additive = (flags & 0x04) != 0;

      if (relocType == 0) { // Internal Reference
        uint16_t actualTargetSeg = 0;
        LOG_DOS("DOS: Reloc Internal index ", std::dec, targetSegIdx,
                " offset 0x", std::hex, targetOffset, " at relOffset 0x",
                nextOffset);

        if (targetSegIdx > 0 && targetSegIdx <= m_neSegments.size()) {
          actualTargetSeg = m_neSegments[targetSegIdx - 1].loadedSegment;
          if (actualTargetSeg == 0)
            actualTargetSeg = loadOverlaySegment(targetSegIdx - 1);
        } else if (targetSegIdx == 255) {
          actualTargetSeg = m_neSegments[segIndex].loadedSegment;
          LOG_DOS("DOS: Reloc mapping segment 255 -> self (0x", std::hex,
                  actualTargetSeg, ")");
        } else {
          LOG_ERROR("DOS: Reloc Internal Ref invalid segment index ", std::dec,
                    targetSegIdx);
          continue;
        }

        if (actualTargetSeg == 0)
          continue;

        int chainCount = 0;
        while (nextOffset != 0xFFFF) {
          LOG_DEBUG("DOS: Reloc chain site: targetSeg=0x", std::hex,
                    actualTargetSeg, " at relOffset=0x", nextOffset);
          if (chainCount++ > 100) {
            LOG_ERROR(
                "DOS: Relocation chain too long or cyclic at relOffset 0x",
                std::hex, nextOffset);
            break;
          }
          if ((size_t)nextOffset >= buffer.size())
            break;

          uint16_t currentSiteValue = 0;
          if ((size_t)nextOffset + 1 < buffer.size()) {
            currentSiteValue =
                *reinterpret_cast<uint16_t *>(&buffer[nextOffset]);
          }

          // For additive relocations, apply the target offset over the existing
          // inline offset.
          uint16_t finalOffset = targetOffset;
          if (additive) {
            finalOffset += currentSiteValue;
          }

          if (sourceType == 2) { // 16-bit Segment (2 bytes)
            if ((size_t)nextOffset + 1 < buffer.size()) {
              buffer[nextOffset] = actualTargetSeg & 0xFF;
              buffer[nextOffset + 1] = (actualTargetSeg >> 8) & 0xFF;
            }
          } else if (sourceType ==
                     3) { // 32-bit Far Address (16-bit offset + 16-bit segment)
            if ((size_t)nextOffset + 3 < buffer.size()) {
              buffer[nextOffset] = finalOffset & 0xFF;
              buffer[nextOffset + 1] = (finalOffset >> 8) & 0xFF;
              buffer[nextOffset + 2] = actualTargetSeg & 0xFF;
              buffer[nextOffset + 3] = (actualTargetSeg >> 8) & 0xFF;
            }
          } else if (sourceType == 5) { // 16-bit Offset (2 bytes)
            if ((size_t)nextOffset + 1 < buffer.size()) {
              buffer[nextOffset] = finalOffset & 0xFF;
              buffer[nextOffset + 1] = (finalOffset >> 8) & 0xFF;
            }
          } else if (sourceType == 0) { // 8-bit LoByte (1 byte)
            buffer[nextOffset] = finalOffset & 0xFF;
          }

          if (additive)
            break;                       // Additive relocations do not chain
          nextOffset = currentSiteValue; // Move to next item in chain
        }
      } else if (relocType == 1 || relocType == 2) {
        // Imported Ordinal / Name - Not implemented yet
      }
    }
  }

  uint32_t targetAddr = (targetSegment << 4);
  for (uint32_t i = 0; i < buffer.size(); ++i) {
    m_memory.write8(targetAddr + i, buffer[i]);
  }

  LOG_DOS("DOS: Loaded overlay segment ", std::dec, segIndex + 1, " to 0x",
          std::hex, targetSegment);
  return targetSegment;
}

uint16_t DOS::loadFBOVOverlay(size_t overlayIndex) {
  LOG_DEBUG("DOS: loadFBOVOverlay(", std::dec, overlayIndex, ")");
  if (overlayIndex >= m_fbovOverlays.size()) {
    LOG_ERROR("DOS: loadFBOVOverlay out of bounds: ", overlayIndex);
    return 0;
  }

  auto &overlay = m_fbovOverlays[overlayIndex];
  if (overlay.loadedSegment != 0)
    return overlay.loadedSegment;
  if (overlay.fileSize == 0) {
    LOG_ERROR("DOS: FBOV overlay id ", std::dec, overlay.overlayId,
              " has zero size");
    return 0;
  }

  const uint16_t paragraphs =
      static_cast<uint16_t>((overlay.fileSize + 15u) / 16u);

  m_cpu.setReg8(cpu::AH, 0x48);
  m_cpu.setReg16(cpu::BX, paragraphs);
  handleMemoryManagement();

  if (m_cpu.getEFLAGS() & cpu::FLAG_CARRY) {
    LOG_ERROR("DOS: Failed to allocate memory (", std::dec, paragraphs,
              " paras) for FBOV overlay id ", overlay.overlayId);
    return 0;
  }

  const uint16_t targetSegment = static_cast<uint16_t>(m_cpu.getReg16(cpu::AX));
  overlay.loadedSegment = targetSegment;

  std::ifstream file(m_programPath, std::ios::binary);
  if (!file) {
    LOG_ERROR("DOS: Failed to open program file for FBOV loading: ",
              m_programPath);
    overlay.loadedSegment = 0;
    return 0;
  }

  file.seekg(static_cast<std::streamoff>(overlay.fileOffset), std::ios::beg);
  std::vector<uint8_t> buffer(static_cast<size_t>(paragraphs) * 16u, 0);
  file.read(reinterpret_cast<char *>(buffer.data()),
            static_cast<std::streamsize>(overlay.fileSize));
  if (static_cast<uint32_t>(file.gcount()) != overlay.fileSize) {
    LOG_ERROR("DOS: Failed to read FBOV overlay id ", std::dec,
              overlay.overlayId, " size=0x", std::hex, overlay.fileSize,
              " from file offset 0x", overlay.fileOffset);
    overlay.loadedSegment = 0;
    return 0;
  }

  const uint32_t targetAddr = static_cast<uint32_t>(targetSegment) << 4u;
  for (uint32_t i = 0; i < buffer.size(); ++i) {
    m_memory.write8(targetAddr + i, buffer[i]);
  }

  LOG_DOS("DOS: Loaded FBOV overlay id ", std::dec, overlay.overlayId,
          " from 0x", std::hex, overlay.fileOffset, " to 0x",
          targetSegment);
  return targetSegment;
}

void DOS::handleFileService() {
  uint8_t ah = m_cpu.getReg8(cpu::AH);
  if (ah == 0x3C) { // Create or Truncate File
    uint16_t dx = m_cpu.getReg16(cpu::DX);
    uint32_t nameAddr = m_cpu.getSegBase(cpu::DS) + dx;
    std::string filename = readFilename(nameAddr);
    std::string hostPath = resolvePath(filename);

    auto fh = std::make_shared<FileHandle>();
    fh->path = hostPath;
    fh->stream.open(hostPath, std::ios::out | std::ios::binary |
                                  std::ios::trunc | std::ios::in);

    if (fh->stream.is_open()) {
      m_fileHandles.push_back(std::move(fh));
      m_cpu.setReg16(cpu::AX,
                     m_fileHandles.size() - 1 + 5); // DOS handles start at 5
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Created file '", hostPath,
              "' handle=", m_fileHandles.size() - 1 + 5);
    } else {
      m_cpu.setReg16(cpu::AX, 0x03); // Path not found
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      LOG_ERROR("DOS: Failed to create file '", hostPath, "'");
    }
  } else if (ah == 0x3D) { // Open Existing File
    uint16_t dx = m_cpu.getReg16(cpu::DX);
    uint8_t accessMode =
        m_cpu.getReg8(cpu::AL) & 0x03; // 0=Read, 1=Write, 2=Read/Write
    uint32_t nameAddr = m_cpu.getSegBase(cpu::DS) + dx;
    std::string filename = readFilename(nameAddr);

    if (isEMSDeviceProbe(filename)) {
      auto fh = std::make_shared<FileHandle>();
      fh->path = upperASCII(fs::path(filename).filename().string());
      fh->kind = FileHandle::Kind::EMSDevice;
      m_fileHandles.push_back(std::move(fh));
      m_cpu.setReg16(cpu::AX,
                     static_cast<uint16_t>(m_fileHandles.size() - 1 + 5));
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Opened EMS device '",
              m_fileHandles.back()->path, "' handle=",
              m_fileHandles.size() - 1 + 5);
      return;
    }

    std::string hostPath = resolvePath(filename);

    auto fh = std::make_shared<FileHandle>();
    fh->path = hostPath;

    // DOS extenders (e.g., DOS/4GW) may reopen their own EXE by a truncated
    // name (like ".EXE") when they can't find the program name from PM.
    // Fall back to the loaded program's host path if the name looks like
    // just an extension and the file doesn't exist.
    if (!fs::exists(hostPath) && !m_programPath.empty()) {
      fs::path fn = fs::path(filename).filename();
      std::string fnStr = fn.string();
      if (!fnStr.empty() && fnStr[0] == '.') {
        hostPath = m_programPath;
        fh->path = hostPath;
        LOG_INFO("DOS: Filename fallback: '", filename, "' -> '",
                 hostPath, "'");
      }
    }

    std::ios::openmode mode = std::ios::binary;
    if (accessMode == 0)
      mode |= std::ios::in;
    else if (accessMode == 1)
      mode |= std::ios::out | std::ios::app;
    else
      mode |= std::ios::in | std::ios::out;

    fh->stream.open(hostPath, mode);

    if (fh->stream.is_open()) {
      m_fileHandles.push_back(std::move(fh));
      m_cpu.setReg16(cpu::AX, m_fileHandles.size() - 1 + 5);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Opened file '", hostPath,
              "' handle=", m_fileHandles.size() - 1 + 5);
    } else {
      m_cpu.setReg16(cpu::AX, 0x02); // File/device not found (EMS absent)
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      LOG_ERROR("DOS: Failed to open file '", hostPath, "'");
    }
  } else if (ah == 0x3E) { // Close File
    uint16_t handle = m_cpu.getReg16(cpu::BX);
    if (handle >= 5 && handle - 5 < m_fileHandles.size() &&
        m_fileHandles[handle - 5]) {
      if (m_fileHandles[handle - 5].use_count() == 1 &&
          !m_fileHandles[handle - 5]->isEMSDevice())
        m_fileHandles[handle - 5]->stream.close();
      m_fileHandles[handle - 5].reset(); // Free slot
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Closed file handle=", handle);
    } else if (handle < 5) { // Standard handles
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Closed standard handle=", handle);
    } else {
      m_cpu.setReg16(cpu::AX, 0x06); // Invalid handle
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    }
  } else if (ah == 0x45) { // Duplicate File Handle
    uint16_t handle = m_cpu.getReg16(cpu::BX);
    if (handle >= 5 && handle - 5 < m_fileHandles.size() &&
        m_fileHandles[handle - 5]) {
      m_fileHandles.push_back(m_fileHandles[handle - 5]);
      m_cpu.setReg16(cpu::AX,
                     static_cast<uint16_t>(m_fileHandles.size() - 1 + 5));
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Duplicated handle=", handle, " -> ",
              m_fileHandles.size() - 1 + 5);
    } else if (handle < 5) {
      m_cpu.setReg16(cpu::AX, handle);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Duplicated standard handle=", handle);
    } else {
      m_cpu.setReg16(cpu::AX, 0x06); // Invalid handle
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      LOG_DOS("DOS: Duplicate handle failed for invalid handle=", handle);
    }
  } else if (ah == 0x3F) { // Read from File or Device
    uint16_t handle = m_cpu.getReg16(cpu::BX);
    uint16_t bytesToRead = m_cpu.getReg16(cpu::CX);
    uint16_t dx = m_cpu.getReg16(cpu::DX);
    uint32_t bufAddr = m_cpu.getSegBase(cpu::DS) + dx;

    if (handle >= 5 && handle - 5 < m_fileHandles.size() &&
        m_fileHandles[handle - 5]) {
      auto &fh = m_fileHandles[handle - 5];
      if (fh->isEMSDevice()) {
        m_cpu.setReg16(cpu::AX, 0x0000);
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
        LOG_DOS("DOS: Read from EMS device handle=", handle,
                " returned 0 bytes");
        return;
      }

      std::vector<char> buf(bytesToRead);
      fh->stream.read(buf.data(), bytesToRead);
      std::streamsize bytesRead = fh->stream.gcount();

      for (std::streamsize i = 0; i < bytesRead; ++i) {
        m_memory.write8(bufAddr + i, buf[i]);
      }

      m_cpu.setReg16(cpu::AX, static_cast<uint16_t>(bytesRead));
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Read ", bytesRead, " bytes from handle=", handle);
    } else if (handle == 0) {     // STDIN
      m_cpu.setReg16(cpu::AX, 0); // Not fully implemented
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else {
      m_cpu.setReg16(cpu::AX, 0x06); // Invalid handle
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    }
  } else if (ah == 0x40) { // Write to File or Device
    uint16_t handle = m_cpu.getReg16(cpu::BX);
    uint16_t bytesToWrite = m_cpu.getReg16(cpu::CX);
    uint16_t dx = m_cpu.getReg16(cpu::DX);
    uint32_t bufAddr = m_cpu.getSegBase(cpu::DS) + dx;

    if (handle == 1 || handle == 2) { // STDOUT / STDERR
      for (uint16_t i = 0; i < bytesToWrite; ++i) {
        writeCharToVRAM(m_memory.read8(bufAddr + i));
      }
      m_cpu.setReg16(cpu::AX, bytesToWrite);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else if (handle >= 5 && handle - 5 < m_fileHandles.size() &&
               m_fileHandles[handle - 5]) {
      auto &fh = m_fileHandles[handle - 5];
      if (fh->isEMSDevice()) {
        m_cpu.setReg16(cpu::AX, bytesToWrite);
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
        LOG_DOS("DOS: Ignored write of ", bytesToWrite,
                " bytes to EMS device handle=", handle);
        return;
      }

      std::vector<char> buf(bytesToWrite);
      for (uint16_t i = 0; i < bytesToWrite; ++i) {
        buf[i] = m_memory.read8(bufAddr + i);
      }
      fh->stream.write(buf.data(), bytesToWrite);

      if (fh->stream.good()) {
        m_cpu.setReg16(cpu::AX, bytesToWrite);
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
        LOG_DOS("DOS: Wrote ", bytesToWrite, " bytes to handle=", handle);
      } else {
        m_cpu.setReg16(cpu::AX, 0x1D); // Write fault
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      }
    } else {
      m_cpu.setReg16(cpu::AX, 0x06); // Invalid handle
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    }
  } else if (ah == 0x41) { // Delete File
    uint16_t dx = m_cpu.getReg16(cpu::DX);
    uint32_t nameAddr = m_cpu.getSegBase(cpu::DS) + dx;
    std::string filename = readFilename(nameAddr);
    std::string hostPath = resolvePath(filename);
    fs::path targetPath(hostPath);
    fs::path parentPath = targetPath.parent_path();
    std::error_code ec;

    if (!fs::exists(targetPath, ec)) {
      const bool parentExists =
          parentPath.empty() || fs::exists(parentPath, ec);
      m_cpu.setReg16(cpu::AX, parentExists ? 0x02 : 0x03);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      LOG_DOS("DOS: Delete file failed for '", hostPath,
              "' (missing target)");
    } else if (fs::is_directory(targetPath, ec)) {
      m_cpu.setReg16(cpu::AX, 0x05); // Access denied
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      LOG_DOS("DOS: Delete file rejected directory '", hostPath, "'");
    } else if (fs::remove(targetPath, ec)) {
      m_cpu.setReg16(cpu::AX, 0x0000);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Deleted file '", hostPath, "'");
    } else {
      m_cpu.setReg16(cpu::AX, 0x05); // Access denied / delete failure
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      LOG_ERROR("DOS: Failed to delete file '", hostPath,
                "' error=", ec.message());
    }
  } else if (ah == 0x42) { // Move File Pointer (Seek)
    uint16_t handle = m_cpu.getReg16(cpu::BX);
    uint8_t method = m_cpu.getReg8(cpu::AL);
    uint16_t cx = m_cpu.getReg16(cpu::CX); // High word of offset
    uint16_t dx = m_cpu.getReg16(cpu::DX); // Low word of offset
    int32_t offset = (cx << 16) | dx;

    if (handle >= 5 && handle - 5 < m_fileHandles.size() &&
        m_fileHandles[handle - 5]) {
      auto &fh = m_fileHandles[handle - 5];
      if (fh->isEMSDevice()) {
        m_cpu.setReg16(cpu::AX, 0x01); // Invalid function for device
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
        LOG_DOS("DOS: Seek not supported on EMS device handle=", handle);
        return;
      }

      std::ios_base::seekdir dir;
      if (method == 0)
        dir = std::ios::beg;
      else if (method == 1)
        dir = std::ios::cur;
      else if (method == 2)
        dir = std::ios::end;
      else {
        m_cpu.setReg16(cpu::AX, 0x01); // Invalid function
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
        return;
      }

      fh->stream.clear(); // Clear EOF flags
      fh->stream.seekg(offset, dir);
      fh->stream.seekp(offset, dir);

      uint32_t newPos = fh->stream.tellg();
      m_cpu.setReg16(cpu::DX, newPos >> 16);
      m_cpu.setReg16(cpu::AX, newPos & 0xFFFF);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Seeked handle=", handle, " to pos=", newPos);
    } else {
      m_cpu.setReg16(cpu::AX, 0x06); // Invalid handle
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    }
  } else if (ah == 0x43) { // Get/Set File Attributes
    uint16_t dx = m_cpu.getReg16(cpu::DX);
    uint8_t al = m_cpu.getReg8(cpu::AL);
    uint32_t nameAddr = m_cpu.getSegBase(cpu::DS) + dx;
    std::string filename = readFilename(nameAddr);
    std::string hostPath = resolvePath(filename);

    std::error_code ec;
    if (al == 0x00) { // Get
      if (fs::exists(hostPath, ec)) {
        uint16_t attr = 0x20; // Archive
        if (fs::is_directory(hostPath, ec))
          attr = 0x10;
        m_cpu.setReg16(cpu::CX, attr);
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
        LOG_DOS("DOS: Get attributes for '", hostPath, "'");
      } else {
        m_cpu.setReg16(cpu::AX, 0x02); // File not found
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      }
    } else if (al == 0x01) { // Set (Stubbed)
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DOS("DOS: Set attributes for '", hostPath, "' (stubbed)");
    } else {
      m_cpu.setReg16(cpu::AX, 0x01); // Invalid function
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    }
  }
}

} // namespace fador::hw
