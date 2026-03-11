#include "DPMI.hpp"
#include "BIOS.hpp"
#include "DOS.hpp"
#include "../utils/Logger.hpp"
#include <cstring>

namespace fador::hw {

// Physical addresses for DPMI tables (must not conflict with HLE stubs,
// VGA palette, or BIOS config table)
static constexpr uint32_t GDT_PHYS = 0xF1000; // 5 entries (40 bytes)
static constexpr uint32_t IDT_PM_PHYS = 0xF0800; // 256 entries (2048 bytes)
static constexpr uint32_t TSS_PHYS = 0xF3000; // 104 bytes
static constexpr uint32_t LDT_PHYS = 0xC0000; // Up to 64KB for 8192 entries

// ── Constructor ──────────────────────────────────────────────────────────────
DPMI::DPMI(cpu::CPU &cpu, memory::MemoryBus &memory)
    : m_cpu(cpu), m_memory(memory) {
  m_ldt.resize(MAX_LDT);
  m_ldtUsed.resize(MAX_LDT, false);
  m_ldtUsed[0] = true; // Index 0 = null descriptor (never allocate)
}

// ── INT 2Fh AX=1687h — DPMI detection ───────────────────────────────────────
void DPMI::handleDetect() {
  m_cpu.setReg16(cpu::AX, 0x0000); // DPMI is available
  m_cpu.setReg16(cpu::BX, 0x0001); // Flags: 32-bit programs supported
  m_cpu.setReg8(cpu::CL, 0x03);    // Processor type: 386
  m_cpu.setReg8(cpu::DH, 0x00);    // DPMI major version
  m_cpu.setReg8(cpu::DL, 0x09);    // DPMI minor version (0.9)
  m_cpu.setReg16(cpu::SI, 0x0000); // Paragraphs needed for host data (0)
  m_cpu.setSegReg(cpu::ES, DPMI_ENTRY_SEG);
  m_cpu.setReg16(cpu::DI, DPMI_ENTRY_OFF);
  LOG_DEBUG("DPMI: Detection → available, entry at F000:0050");
}

// ── DPMI entry — real mode → protected mode ─────────────────────────────────
// Called when the client does CALL FAR [F000:0050].
// The 0F FF E1 HLE trap at that address routes here.
// Stack has only the CALL FAR return CS:IP (no INT frame).
bool DPMI::handleEntry() {
  if (m_active) {
    LOG_WARN("DPMI: Entry called but already active");
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    return false;
  }

  // --- Read caller's return address from the CALL FAR frame ---
  uint16_t callerSS = m_cpu.getSegReg(cpu::SS);
  uint32_t ssBase = static_cast<uint32_t>(callerSS) << 4;
  uint16_t sp = m_cpu.getReg16(cpu::SP);

  uint16_t callerIP = m_memory.read16(ssBase + sp);
  uint16_t callerCS_RM = m_memory.read16(ssBase + sp + 2);
  sp += 4; // Pop CALL frame
  m_cpu.setReg16(cpu::SP, sp);

  uint16_t callerDS = m_cpu.getSegReg(cpu::DS);
  uint16_t callerES = m_cpu.getSegReg(cpu::ES);

  uint16_t flags = m_cpu.getReg16(cpu::AX); // bit 0 = 32-bit if set
  bool is32 = (flags & 1) != 0;

  LOG_INFO("DPMI: Entry from RM CS=0x", std::hex, callerCS_RM, " DS=0x",
           callerDS, " SS=0x", callerSS, " IP=0x", callerIP,
           " 32bit=", is32);

  // --- Allocate LDT entries for the client's initial selectors ---
  auto makeDataDesc = [&](uint16_t seg) -> RawDescriptor {
    return buildDescriptor(static_cast<uint32_t>(seg) << 4, 0xFFFF,
                           0xF2, // Present, DPL=3, Data, R/W
                           0x00); // Byte gran, 16-bit
  };
  auto makeCodeDesc = [&](uint16_t seg) -> RawDescriptor {
    return buildDescriptor(static_cast<uint32_t>(seg) << 4, 0xFFFF,
                           0xFA, // Present, DPL=3, Code, R/X
                           is32 ? 0x04 : 0x00); // D bit for 32-bit
  };

  m_clientCS = allocateDescriptors(1);
  m_clientDS = allocateDescriptors(1);
  m_clientSS = allocateDescriptors(1);
  m_clientES = allocateDescriptors(1);
  m_clientPSPSel = allocateDescriptors(1);

  if (!m_clientCS || !m_clientDS || !m_clientSS || !m_clientES) {
    LOG_ERROR("DPMI: Failed to allocate initial descriptors");
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    return false;
  }

  m_ldt[selectorToIndex(m_clientCS)] = makeCodeDesc(callerCS_RM);
  m_ldt[selectorToIndex(m_clientDS)] = makeDataDesc(callerDS);
  m_ldt[selectorToIndex(m_clientSS)] = makeDataDesc(callerSS);
  m_ldt[selectorToIndex(m_clientES)] = makeDataDesc(callerES);

  // PSP selector
  uint16_t pspSeg = m_dos ? m_dos->getPSPSegment() : 0x0801;
  m_ldt[selectorToIndex(m_clientPSPSel)] = makeDataDesc(pspSeg);

  // --- Write LDT to physical memory ---
  for (int i = 0; i < MAX_LDT; ++i) {
    uint32_t addr = LDT_PHYS + i * 8;
    m_memory.write32(addr, m_ldt[i].low);
    m_memory.write32(addr + 4, m_ldt[i].high);
  }

  // --- Build GDT ---
  // Entry 0 (sel 0x00): Null
  m_memory.write32(GDT_PHYS + 0x00, 0);
  m_memory.write32(GDT_PHYS + 0x04, 0);

  // Entry 1 (sel 0x08): Ring-0 code, flat 4GB, 32-bit
  {
    auto d = buildDescriptor(0, 0xFFFFF, 0x9A, 0x0C); // G=1, D=1
    m_memory.write32(GDT_PHYS + 0x08, d.low);
    m_memory.write32(GDT_PHYS + 0x0C, d.high);
  }
  // Entry 2 (sel 0x10): Ring-0 data, flat 4GB, 32-bit
  {
    auto d = buildDescriptor(0, 0xFFFFF, 0x92, 0x0C); // G=1, D=1
    m_memory.write32(GDT_PHYS + 0x10, d.low);
    m_memory.write32(GDT_PHYS + 0x14, d.high);
  }
  // Entry 3 (sel 0x18): LDT descriptor (system)
  {
    uint32_t ldtLimit = MAX_LDT * 8 - 1;
    auto d = buildDescriptor(LDT_PHYS, ldtLimit, 0x82, 0x00); // LDT type
    m_memory.write32(GDT_PHYS + 0x18, d.low);
    m_memory.write32(GDT_PHYS + 0x1C, d.high);
  }
  // Entry 4 (sel 0x20): TSS (32-bit, available)
  {
    auto d = buildDescriptor(TSS_PHYS, 0x67, 0x89, 0x00);
    m_memory.write32(GDT_PHYS + 0x20, d.low);
    m_memory.write32(GDT_PHYS + 0x24, d.high);
    // Write minimal TSS
    for (int i = 0; i < 0x68; ++i)
      m_memory.write8(TSS_PHYS + i, 0);
  }

  // --- Build PM IDT ---
  // Each entry points to the HLE stub at physical F0000h + HLE_STUB_BASE + v*4
  // using selector 0x08 (flat Ring-0 code with base=0).
  for (int v = 0; v < 256; ++v) {
    uint32_t stubPhysAddr = 0xF0000 + 0x0100 + v * 4;
    // 32-bit interrupt gate: selector=0x08, P=1, DPL=3, type=0xEE
    // DPL=3 so Ring-3 code can use INT instructions
    uint32_t low = (0x0008 << 16) | (stubPhysAddr & 0xFFFF);
    uint32_t high = (stubPhysAddr & 0xFFFF0000) | 0x0000EE00;
    m_memory.write32(IDT_PM_PHYS + v * 8, low);
    m_memory.write32(IDT_PM_PHYS + v * 8 + 4, high);
  }

  // --- Load system registers ---
  m_cpu.setGDTR({0x27, GDT_PHYS}); // 5 entries, limit = 39
  m_cpu.setIDTR({0x7FF, IDT_PM_PHYS}); // 256 entries
  m_cpu.setLDTRSelector(0x18);
  m_cpu.setLDTR({static_cast<uint16_t>(MAX_LDT * 8 - 1), LDT_PHYS});
  m_cpu.setTRSelector(0x20);
  m_cpu.setTR({0x67, TSS_PHYS});

  // --- Enter protected mode ---
  m_cpu.setCR(0, m_cpu.getCR(0) | 1);
  m_memory.setA20(true);

  // --- Set client selectors ---
  // The 0F FF handler will call loadSegment() to sync InstructionDecoder caches
  m_cpu.setSegReg(cpu::CS, m_clientCS);
  m_cpu.setSegReg(cpu::DS, m_clientDS);
  m_cpu.setSegReg(cpu::SS, m_clientSS);
  m_cpu.setSegReg(cpu::ES, m_clientES);
  m_cpu.setSegReg(cpu::FS, 0);
  m_cpu.setSegReg(cpu::GS, 0);

  // Set return address
  m_cpu.setEIP(callerIP);

  // Clear carry → success
  m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);

  // Set AX = PSP selector (DPMI spec)
  m_cpu.setReg16(cpu::AX, m_clientPSPSel);

  m_active = true;

  LOG_INFO("DPMI: Entered PM. CS=0x", std::hex, m_clientCS, " DS=0x",
           m_clientDS, " SS=0x", m_clientSS, " ES=0x", m_clientES,
           " EIP=0x", callerIP);
  return true;
}

// ── INT 31h dispatcher ──────────────────────────────────────────────────────
bool DPMI::handleInt31() {
  if (!m_active)
    return false;

  uint16_t ax = m_cpu.getReg16(cpu::AX);
  uint8_t ah = ax >> 8;

  switch (ah) {
  case 0x00:
    handleDescriptorMgmt();
    return true;
  case 0x01:
    handleDOSMemory();
    return true;
  case 0x02:
    handleInterruptVectors();
    return true;
  case 0x03:
    handleTranslation();
    return true;
  case 0x04:
    handleVersion();
    return true;
  case 0x05:
    handleMemoryInfo();
    return true;
  case 0x06:
    handlePageSize();
    return true;
  case 0x09:
    handleVirtualInterrupt();
    return true;
  default:
    LOG_WARN("DPMI: Unhandled INT 31h AX=0x", std::hex, ax);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    m_cpu.setReg16(cpu::AX, 0x8001); // Unsupported function
    return true;
  }
}

// ── Descriptor management (AX=0000h..000Ch) ─────────────────────────────────
void DPMI::handleDescriptorMgmt() {
  uint16_t func = m_cpu.getReg16(cpu::AX);
  switch (func) {
  case 0x0000: { // Allocate LDT Descriptors
    uint16_t count = m_cpu.getReg16(cpu::CX);
    uint16_t sel = allocateDescriptors(count);
    if (sel) {
      m_cpu.setReg16(cpu::AX, sel);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DEBUG("DPMI 0000h: Allocated ", count, " descriptors, base sel=0x",
                std::hex, sel);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      m_cpu.setReg16(cpu::AX, 0x8011);
    }
    break;
  }
  case 0x0001: { // Free LDT Descriptor
    uint16_t sel = m_cpu.getReg16(cpu::BX);
    if (freeDescriptor(sel)) {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      m_cpu.setReg16(cpu::AX, 0x8022);
    }
    break;
  }
  case 0x0002: { // Segment to Descriptor
    uint16_t seg = m_cpu.getReg16(cpu::BX);
    uint16_t sel = allocateDescriptors(1);
    if (sel) {
      m_ldt[selectorToIndex(sel)] =
          buildDescriptor(static_cast<uint32_t>(seg) << 4, 0xFFFF, 0xF2, 0x00);
      flushLDTEntry(selectorToIndex(sel));
      m_cpu.setReg16(cpu::AX, sel);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      m_cpu.setReg16(cpu::AX, 0x8011);
    }
    break;
  }
  case 0x0003: { // Get Selector Increment Value
    m_cpu.setReg16(cpu::AX, SELECTOR_INCREMENT);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    break;
  }
  case 0x0006: { // Get Segment Base Address
    uint16_t sel = m_cpu.getReg16(cpu::BX);
    int idx = selectorToIndex(sel);
    if (idx > 0 && idx < MAX_LDT && m_ldtUsed[idx]) {
      uint32_t base = extractBase(m_ldt[idx]);
      m_cpu.setReg16(cpu::CX, static_cast<uint16_t>(base >> 16));
      m_cpu.setReg16(cpu::DX, static_cast<uint16_t>(base & 0xFFFF));
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      m_cpu.setReg16(cpu::AX, 0x8022);
    }
    break;
  }
  case 0x0007: { // Set Segment Base Address
    uint16_t sel = m_cpu.getReg16(cpu::BX);
    uint32_t base = (static_cast<uint32_t>(m_cpu.getReg16(cpu::CX)) << 16) |
                    m_cpu.getReg16(cpu::DX);
    int idx = selectorToIndex(sel);
    if (idx > 0 && idx < MAX_LDT && m_ldtUsed[idx]) {
      setDescBase(m_ldt[idx], base);
      flushLDTEntry(idx);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      LOG_DEBUG("DPMI 0007h: Set sel 0x", std::hex, sel, " base=0x", base);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      m_cpu.setReg16(cpu::AX, 0x8022);
    }
    break;
  }
  case 0x0008: { // Set Segment Limit
    uint16_t sel = m_cpu.getReg16(cpu::BX);
    uint32_t limit = (static_cast<uint32_t>(m_cpu.getReg16(cpu::CX)) << 16) |
                     m_cpu.getReg16(cpu::DX);
    int idx = selectorToIndex(sel);
    if (idx > 0 && idx < MAX_LDT && m_ldtUsed[idx]) {
      setDescLimit(m_ldt[idx], limit);
      flushLDTEntry(idx);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      m_cpu.setReg16(cpu::AX, 0x8022);
    }
    break;
  }
  case 0x0009: { // Set Descriptor Access Rights
    uint16_t sel = m_cpu.getReg16(cpu::BX);
    uint16_t rights = m_cpu.getReg16(cpu::CX);
    int idx = selectorToIndex(sel);
    if (idx > 0 && idx < MAX_LDT && m_ldtUsed[idx]) {
      auto &d = m_ldt[idx];
      // CL = access byte (bits 8-15 of high dword)
      // CH bits 4-7 = G/D/L/AVL (bits 20-23 of high dword)
      uint8_t accessByte = rights & 0xFF;
      uint8_t extNibble = (rights >> 12) & 0x0F;
      d.high = (d.high & 0xFF0F00FF) | // Keep base[23:16], limit[19:16],
                                        // base[31:24]
               (static_cast<uint32_t>(accessByte) << 8) |
               (static_cast<uint32_t>(extNibble) << 20);
      flushLDTEntry(idx);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      m_cpu.setReg16(cpu::AX, 0x8022);
    }
    break;
  }
  case 0x000A: { // Create Code Segment Alias (data alias of code seg)
    uint16_t sel = m_cpu.getReg16(cpu::BX);
    int idx = selectorToIndex(sel);
    if (idx > 0 && idx < MAX_LDT && m_ldtUsed[idx]) {
      uint16_t newSel = allocateDescriptors(1);
      if (newSel) {
        m_ldt[selectorToIndex(newSel)] = m_ldt[idx];
        auto &d = m_ldt[selectorToIndex(newSel)];
        // Change type from Code to Data R/W: type field [11:8] in high dword
        uint8_t access = (d.high >> 8) & 0xFF;
        access = (access & 0xF0) | 0x02; // Data, R/W
        d.high = (d.high & ~0x0000FF00U) | (static_cast<uint32_t>(access) << 8);
        flushLDTEntry(selectorToIndex(newSel));
        m_cpu.setReg16(cpu::AX, newSel);
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
      } else {
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
        m_cpu.setReg16(cpu::AX, 0x8011);
      }
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      m_cpu.setReg16(cpu::AX, 0x8022);
    }
    break;
  }
  case 0x000B: { // Get Descriptor
    uint16_t sel = m_cpu.getReg16(cpu::BX);
    int idx = selectorToIndex(sel);
    if (idx > 0 && idx < MAX_LDT && m_ldtUsed[idx]) {
      uint32_t addr = getLinearAddr(cpu::ES, cpu::DI);
      m_memory.write32(addr, m_ldt[idx].low);
      m_memory.write32(addr + 4, m_ldt[idx].high);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      m_cpu.setReg16(cpu::AX, 0x8022);
    }
    break;
  }
  case 0x000C: { // Set Descriptor
    uint16_t sel = m_cpu.getReg16(cpu::BX);
    int idx = selectorToIndex(sel);
    if (idx > 0 && idx < MAX_LDT && m_ldtUsed[idx]) {
      uint32_t addr = getLinearAddr(cpu::ES, cpu::DI);
      m_ldt[idx].low = m_memory.read32(addr);
      m_ldt[idx].high = m_memory.read32(addr + 4);
      flushLDTEntry(idx);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      m_cpu.setReg16(cpu::AX, 0x8022);
    }
    break;
  }
  default:
    LOG_WARN("DPMI: Unhandled descriptor function 0x", std::hex, func);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    m_cpu.setReg16(cpu::AX, 0x8001);
    break;
  }
}

// ── DOS memory (AX=0100h..0102h) ────────────────────────────────────────────
void DPMI::handleDOSMemory() {
  uint16_t func = m_cpu.getReg16(cpu::AX);
  switch (func) {
  case 0x0100: { // Allocate DOS Memory Block
    uint16_t paras = m_cpu.getReg16(cpu::BX);
    m_cpu.setReg8(cpu::AH, 0x48);
    m_cpu.setReg16(cpu::BX, paras);
    if (m_dos)
      m_dos->handleInterrupt(0x21);
    if (!(m_cpu.getEFLAGS() & cpu::FLAG_CARRY)) {
      uint16_t seg = m_cpu.getReg16(cpu::AX);
      uint16_t sel = allocateDescriptors(1);
      if (sel) {
        m_ldt[selectorToIndex(sel)] = buildDescriptor(
            static_cast<uint32_t>(seg) << 4,
            static_cast<uint32_t>(paras) * 16 - 1, 0xF2, 0x00);
        flushLDTEntry(selectorToIndex(sel));
        m_cpu.setReg16(cpu::DX, sel);
        m_cpu.setReg16(cpu::AX, seg);
      }
    }
    break;
  }
  case 0x0101: { // Free DOS Memory Block
    uint16_t sel = m_cpu.getReg16(cpu::DX);
    int idx = selectorToIndex(sel);
    if (idx > 0 && idx < MAX_LDT && m_ldtUsed[idx]) {
      uint32_t base = extractBase(m_ldt[idx]);
      uint16_t seg = static_cast<uint16_t>(base >> 4);
      m_cpu.setSegReg(cpu::ES, seg);
      m_cpu.setReg8(cpu::AH, 0x49);
      if (m_dos)
        m_dos->handleInterrupt(0x21);
      freeDescriptor(sel);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    }
    break;
  }
  case 0x0102: { // Resize DOS Memory Block
    uint16_t sel = m_cpu.getReg16(cpu::DX);
    uint16_t newParas = m_cpu.getReg16(cpu::BX);
    int idx = selectorToIndex(sel);
    if (idx > 0 && idx < MAX_LDT && m_ldtUsed[idx]) {
      uint32_t base = extractBase(m_ldt[idx]);
      uint16_t seg = static_cast<uint16_t>(base >> 4);
      m_cpu.setSegReg(cpu::ES, seg);
      m_cpu.setReg8(cpu::AH, 0x4A);
      m_cpu.setReg16(cpu::BX, newParas);
      if (m_dos)
        m_dos->handleInterrupt(0x21);
      if (!(m_cpu.getEFLAGS() & cpu::FLAG_CARRY)) {
        setDescLimit(m_ldt[idx],
                     static_cast<uint32_t>(newParas) * 16 - 1);
        flushLDTEntry(idx);
      }
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    }
    break;
  }
  default:
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    m_cpu.setReg16(cpu::AX, 0x8001);
    break;
  }
}

// ── Interrupt vectors (AX=0200h..0205h) ─────────────────────────────────────
void DPMI::handleInterruptVectors() {
  uint16_t func = m_cpu.getReg16(cpu::AX);
  uint8_t vec = m_cpu.getReg8(cpu::BL);
  switch (func) {
  case 0x0200: { // Get Real Mode Interrupt Vector
    uint16_t ip = m_memory.read16(vec * 4);
    uint16_t cs = m_memory.read16(vec * 4 + 2);
    m_cpu.setReg16(cpu::CX, cs);
    m_cpu.setReg16(cpu::DX, ip);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    break;
  }
  case 0x0201: { // Set Real Mode Interrupt Vector
    uint16_t cs = m_cpu.getReg16(cpu::CX);
    uint16_t ip = m_cpu.getReg16(cpu::DX);
    m_memory.write16(vec * 4, ip);
    m_memory.write16(vec * 4 + 2, cs);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    break;
  }
  case 0x0204: { // Get Protected Mode Interrupt Vector
    m_cpu.setReg16(cpu::CX, m_pmVectors[vec].selector);
    if (m_cpu.is32BitCode())
      m_cpu.setReg32(cpu::EDX, m_pmVectors[vec].offset);
    else
      m_cpu.setReg16(cpu::DX,
                     static_cast<uint16_t>(m_pmVectors[vec].offset));
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    break;
  }
  case 0x0205: { // Set Protected Mode Interrupt Vector
    m_pmVectors[vec].selector = m_cpu.getReg16(cpu::CX);
    if (m_cpu.is32BitCode())
      m_pmVectors[vec].offset = m_cpu.getReg32(cpu::EDX);
    else
      m_pmVectors[vec].offset = m_cpu.getReg16(cpu::DX);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    LOG_DEBUG("DPMI 0205h: Set PM INT 0x", std::hex, (int)vec, " -> sel=0x",
              m_pmVectors[vec].selector, " off=0x", m_pmVectors[vec].offset);
    break;
  }
  default:
    LOG_WARN("DPMI: Unhandled interrupt vector function 0x", std::hex, func);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    m_cpu.setReg16(cpu::AX, 0x8001);
    break;
  }
}

// ── Translation services (AX=0300h..0304h) ──────────────────────────────────
void DPMI::handleTranslation() {
  uint16_t func = m_cpu.getReg16(cpu::AX);
  switch (func) {
  case 0x0300: // Simulate Real Mode Interrupt
  case 0x0301: // Call Real Mode Procedure With Far Return Frame
  {
    uint8_t intNo = m_cpu.getReg8(cpu::BL);
    uint32_t structAddr = getLinearAddr(cpu::ES, cpu::EDI);

    // Save current PM state
    uint32_t savedRegs[8];
    for (int i = 0; i < 8; i++)
      savedRegs[i] = m_cpu.getReg32(i);
    uint16_t savedSegs[6];
    for (int i = 0; i < 6; i++)
      savedSegs[i] = m_cpu.getSegReg(i);
    uint32_t savedEIP = m_cpu.getEIP();
    uint32_t savedEFLAGS = m_cpu.getEFLAGS();
    bool savedIs32Code = m_cpu.is32BitCode();
    bool savedIs32Stack = m_cpu.is32BitStack();

    // Load registers from the RM call structure (50 bytes):
    // +00: EDI, +04: ESI, +08: EBP, +0C: reserved
    // +10: EBX, +14: EDX, +18: ECX, +1C: EAX
    // +20: FLAGS, +22: ES, +24: DS, +26: FS, +28: GS
    // +2A: IP, +2C: CS, +2E: SP, +30: SS
    m_cpu.setReg32(cpu::EDI, m_memory.read32(structAddr + 0x00));
    m_cpu.setReg32(cpu::ESI, m_memory.read32(structAddr + 0x04));
    m_cpu.setReg32(cpu::EBP, m_memory.read32(structAddr + 0x08));
    m_cpu.setReg32(cpu::EBX, m_memory.read32(structAddr + 0x10));
    m_cpu.setReg32(cpu::EDX, m_memory.read32(structAddr + 0x14));
    m_cpu.setReg32(cpu::ECX, m_memory.read32(structAddr + 0x18));
    m_cpu.setReg32(cpu::EAX, m_memory.read32(structAddr + 0x1C));

    uint16_t rmFlags = m_memory.read16(structAddr + 0x20);
    uint16_t rmES = m_memory.read16(structAddr + 0x22);
    uint16_t rmDS = m_memory.read16(structAddr + 0x24);

    // Temporarily set RM state for the call
    m_cpu.setEFLAGS((m_cpu.getEFLAGS() & 0xFFFF0000) | rmFlags);
    m_cpu.setIs32BitCode(false);
    m_cpu.setIs32BitStack(false);

    // Set RM segments
    m_cpu.setSegReg(cpu::ES, rmES);
    m_cpu.setSegBase(cpu::ES, static_cast<uint32_t>(rmES) << 4);
    m_cpu.setSegReg(cpu::DS, rmDS);
    m_cpu.setSegBase(cpu::DS, static_cast<uint32_t>(rmDS) << 4);

    if (func == 0x0300) {
      LOG_DEBUG("DPMI 0300h: Simulate RM INT 0x", std::hex, (int)intNo);
      // Use HLE to handle the interrupt directly
      bool handled = false;
      if (m_dos && m_dos->handleInterrupt(intNo))
        handled = true;
      if (!handled && m_bios && m_bios->handleInterrupt(intNo))
        handled = true;
      if (!handled)
        LOG_WARN("DPMI 0300h: INT 0x", std::hex, (int)intNo, " not handled");
    }
    // func == 0x0301: call procedure — not commonly used, stub for now

    // Write back modified registers to the call structure
    m_memory.write32(structAddr + 0x00, m_cpu.getReg32(cpu::EDI));
    m_memory.write32(structAddr + 0x04, m_cpu.getReg32(cpu::ESI));
    m_memory.write32(structAddr + 0x08, m_cpu.getReg32(cpu::EBP));
    m_memory.write32(structAddr + 0x10, m_cpu.getReg32(cpu::EBX));
    m_memory.write32(structAddr + 0x14, m_cpu.getReg32(cpu::EDX));
    m_memory.write32(structAddr + 0x18, m_cpu.getReg32(cpu::ECX));
    m_memory.write32(structAddr + 0x1C, m_cpu.getReg32(cpu::EAX));
    m_memory.write16(structAddr + 0x20,
                     static_cast<uint16_t>(m_cpu.getEFLAGS() & 0xFFFF));
    m_memory.write16(structAddr + 0x22, m_cpu.getSegReg(cpu::ES));
    m_memory.write16(structAddr + 0x24, m_cpu.getSegReg(cpu::DS));

    // Restore PM state
    for (int i = 0; i < 8; i++)
      m_cpu.setReg32(i, savedRegs[i]);
    for (int i = 0; i < 6; i++) {
      m_cpu.setSegReg(i, savedSegs[i]);
      // Restore segment bases from our LDT
      uint16_t sel = savedSegs[i];
      if (sel & LDT_TI_BIT) {
        int idx = selectorToIndex(sel);
        if (idx > 0 && idx < MAX_LDT && m_ldtUsed[idx])
          m_cpu.setSegBase(i, extractBase(m_ldt[idx]));
      } else if (sel == 0) {
        m_cpu.setSegBase(i, 0);
      }
    }
    m_cpu.setEIP(savedEIP);
    m_cpu.setEFLAGS(savedEFLAGS & ~cpu::FLAG_CARRY);
    m_cpu.setIs32BitCode(savedIs32Code);
    m_cpu.setIs32BitStack(savedIs32Stack);
    break;
  }
  case 0x0303: { // Allocate Real Mode Callback Address
    m_cpu.setReg16(cpu::CX, 0xF000);
    m_cpu.setReg16(cpu::DX, 0x0060);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    break;
  }
  case 0x0304: { // Free Real Mode Callback Address
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    break;
  }
  default:
    LOG_WARN("DPMI: Unhandled translation function 0x", std::hex, func);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    m_cpu.setReg16(cpu::AX, 0x8001);
    break;
  }
}

// ── DPMI version (AX=0400h) ─────────────────────────────────────────────────
void DPMI::handleVersion() {
  m_cpu.setReg8(cpu::AH, 0x00); // Major
  m_cpu.setReg8(cpu::AL, 0x09); // Minor (0.9)
  m_cpu.setReg16(cpu::BX, 0x0005); // Flags: 32-bit, no reflection
  m_cpu.setReg8(cpu::CL, 0x04);    // CPU type (486)
  m_cpu.setReg8(cpu::DH, 0x08);    // PIC master base (IRQ0 = INT 08h)
  m_cpu.setReg8(cpu::DL, 0x70);    // PIC slave base (IRQ8 = INT 70h)
  m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
}

// ── Memory info (AX=0500h..0503h) ───────────────────────────────────────────
void DPMI::handleMemoryInfo() {
  uint16_t func = m_cpu.getReg16(cpu::AX);
  switch (func) {
  case 0x0500: { // Get Free Memory Information
    uint32_t addr = getLinearAddr(cpu::ES, cpu::DI);
    uint32_t freeBytes = 0;
    if (m_himem) {
      uint16_t largestKB = 0;
      uint16_t totalKB = m_himem->queryFreeKB(largestKB);
      freeBytes = static_cast<uint32_t>(totalKB) * 1024;
    }
    if (freeBytes == 0)
      freeBytes = 16 * 1024 * 1024; // Default 16MB
    uint32_t freePages = freeBytes / 4096;
    m_memory.write32(addr + 0x00, freeBytes);     // Largest free block
    m_memory.write32(addr + 0x04, freePages);     // Max unlocked pages
    m_memory.write32(addr + 0x08, freePages);     // Max locked pages
    m_memory.write32(addr + 0x0C, 0xFFFFFFFF);    // Linear addr space pages
    m_memory.write32(addr + 0x10, freePages);     // Total unlocked pages
    m_memory.write32(addr + 0x14, freePages);     // Free pages
    m_memory.write32(addr + 0x18, freePages);     // Total physical pages
    m_memory.write32(addr + 0x1C, freePages);     // Free linear addr space
    m_memory.write32(addr + 0x20, 0xFFFFFFFF);    // Swap file size
    for (int i = 0x24; i < 0x30; i += 4)
      m_memory.write32(addr + i, 0);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    break;
  }
  case 0x0501: { // Allocate Memory Block
    uint32_t size = (static_cast<uint32_t>(m_cpu.getReg16(cpu::BX)) << 16) |
                    m_cpu.getReg16(cpu::CX);
    if (size == 0)
      size = 1;

    uint16_t sizeKB = static_cast<uint16_t>((size + 1023) / 1024);
    uint32_t linearAddr = 0;

    if (m_himem) {
      uint16_t handle = m_himem->allocateEMB(sizeKB);
      if (handle && m_himem->lockEMB(handle, linearAddr)) {
        m_cpu.setReg16(cpu::BX,
                       static_cast<uint16_t>(linearAddr >> 16));
        m_cpu.setReg16(cpu::CX,
                       static_cast<uint16_t>(linearAddr & 0xFFFF));
        uint32_t blockHandle = m_nextBlockHandle++;
        m_cpu.setReg16(cpu::SI,
                       static_cast<uint16_t>(blockHandle >> 16));
        m_cpu.setReg16(cpu::DI,
                       static_cast<uint16_t>(blockHandle & 0xFFFF));
        m_memBlocks.push_back({linearAddr, size, handle});
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
        LOG_DEBUG("DPMI 0501h: Alloc ", size, " bytes -> linear=0x", std::hex,
                  linearAddr, " handle=0x", blockHandle);
      } else {
        if (handle)
          m_himem->freeEMB(handle);
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
        m_cpu.setReg16(cpu::AX, 0x8012);
      }
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      m_cpu.setReg16(cpu::AX, 0x8012);
    }
    break;
  }
  case 0x0502: { // Free Memory Block
    uint32_t blockHandle =
        (static_cast<uint32_t>(m_cpu.getReg16(cpu::SI)) << 16) |
        m_cpu.getReg16(cpu::DI);
    bool found = false;
    for (auto it = m_memBlocks.begin(); it != m_memBlocks.end(); ++it) {
      uint32_t thisHandle =
          static_cast<uint32_t>(it - m_memBlocks.begin()) + 1;
      if (thisHandle == blockHandle) {
        if (m_himem && it->xmsHandle) {
          m_himem->unlockEMB(it->xmsHandle);
          m_himem->freeEMB(it->xmsHandle);
        }
        m_memBlocks.erase(it);
        found = true;
        break;
      }
    }
    if (found) {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      m_cpu.setReg16(cpu::AX, 0x8023);
    }
    break;
  }
  case 0x0503: { // Resize Memory Block — stub error
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    m_cpu.setReg16(cpu::AX, 0x8012);
    break;
  }
  default:
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    m_cpu.setReg16(cpu::AX, 0x8001);
    break;
  }
}

// ── Page size (AX=0604h) ────────────────────────────────────────────────────
void DPMI::handlePageSize() {
  m_cpu.setReg16(cpu::BX, 0);
  m_cpu.setReg16(cpu::CX, 4096);
  m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
}

// ── Virtual interrupt state (AX=0900h..0902h) ───────────────────────────────
void DPMI::handleVirtualInterrupt() {
  uint16_t func = m_cpu.getReg16(cpu::AX);
  switch (func) {
  case 0x0900:
    m_cpu.setReg8(cpu::AL, m_virtualIF ? 1 : 0);
    m_virtualIF = false;
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    break;
  case 0x0901:
    m_cpu.setReg8(cpu::AL, m_virtualIF ? 1 : 0);
    m_virtualIF = true;
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    break;
  case 0x0902:
    m_cpu.setReg8(cpu::AL, m_virtualIF ? 1 : 0);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    break;
  default:
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    break;
  }
}

// ── Helpers ─────────────────────────────────────────────────────────────────

uint16_t DPMI::allocateDescriptors(uint16_t count) {
  if (count == 0)
    return 0;
  for (int i = 1; i <= MAX_LDT - count; ++i) {
    bool ok = true;
    for (int j = 0; j < count; ++j) {
      if (m_ldtUsed[i + j]) {
        ok = false;
        break;
      }
    }
    if (ok) {
      for (int j = 0; j < count; ++j)
        m_ldtUsed[i + j] = true;
      return makeSelector(i);
    }
  }
  return 0;
}

bool DPMI::freeDescriptor(uint16_t selector) {
  int idx = selectorToIndex(selector);
  if (idx <= 0 || idx >= MAX_LDT || !m_ldtUsed[idx])
    return false;
  m_ldtUsed[idx] = false;
  m_ldt[idx] = {0, 0};
  flushLDTEntry(idx);
  return true;
}

DPMI::RawDescriptor DPMI::buildDescriptor(uint32_t base, uint32_t limit,
                                          uint8_t access,
                                          uint8_t flags) const {
  // flags is a 4-bit nibble for G:D/B:L:AVL (bits 3:2:1:0)
  // positioned into descriptor high dword bits 20-23
  uint32_t limitVal = limit & 0xFFFFF;

  RawDescriptor d;
  d.low = (limitVal & 0xFFFF) | ((base & 0xFFFF) << 16);
  d.high = ((base >> 16) & 0xFF) |
           (static_cast<uint32_t>(access) << 8) |
           (limitVal & 0x000F0000) |
           (static_cast<uint32_t>(flags & 0x0F) << 20) |
           (base & 0xFF000000);
  return d;
}

uint32_t DPMI::extractBase(const RawDescriptor &d) const {
  return (d.low >> 16) |
         ((d.high & 0xFF) << 16) |
         (d.high & 0xFF000000);
}

void DPMI::setDescBase(RawDescriptor &d, uint32_t base) {
  d.low = (d.low & 0x0000FFFF) | ((base & 0xFFFF) << 16);
  d.high = (d.high & 0x00FFFF00) |
           ((base >> 16) & 0xFF) |
           (base & 0xFF000000);
}

void DPMI::setDescLimit(RawDescriptor &d, uint32_t limit) {
  bool granularity = (limit > 0xFFFFF);
  uint32_t limitVal = granularity ? (limit >> 12) : limit;
  d.low = (d.low & 0xFFFF0000) | (limitVal & 0xFFFF);
  d.high = (d.high & 0xFFF0FFFF) | (limitVal & 0x000F0000);
  if (granularity)
    d.high |= 0x00800000; // G bit
  else
    d.high &= ~0x00800000U;
}

void DPMI::flushLDTEntry(int index) {
  uint32_t addr = LDT_PHYS + index * 8;
  m_memory.write32(addr, m_ldt[index].low);
  m_memory.write32(addr + 4, m_ldt[index].high);
}

uint32_t DPMI::getLinearAddr(uint8_t segReg, uint8_t reg) const {
  uint32_t base = m_cpu.getSegBase(segReg);
  uint32_t offset = m_cpu.is32BitCode() ? m_cpu.getReg32(reg)
                                        : m_cpu.getReg16(reg);
  return base + offset;
}

} // namespace fador::hw
