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
static constexpr uint32_t LDT_PHYS = 0x1F00000; // 31MB — safe from conventional/UMA

// ── Constructor ──────────────────────────────────────────────────────────────
DPMI::DPMI(cpu::CPU &cpu, memory::MemoryBus &memory)
    : m_cpu(cpu), m_memory(memory) {
  m_ldt.resize(MAX_LDT);
  m_ldtUsed.resize(MAX_LDT, false);
  m_ldtBatchAlloc.resize(MAX_LDT, false);
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
  m_is32BitClient = is32;

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
    // Initial CS is always 16-bit (D=0) regardless of client's 32-bit flag.
    // The caller's code at the return address is 16-bit real-mode code;
    // the client will allocate its own 32-bit CS and switch to it.
    return buildDescriptor(static_cast<uint32_t>(seg) << 4, 0xFFFF,
                           0xFA, // Present, DPL=3, Code, R/X
                           0x00); // D=0: 16-bit code segment
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

  // Environment selector — the environment block is at pspSeg - 0x20.
  // Create a PM selector for it and patch PSP:0x2C so PM clients can
  // access the environment through the PSP selector chain.
  {
    uint16_t envSeg = pspSeg - 0x20;
    uint16_t envSel = allocateDescriptors(1);
    if (envSel) {
      m_ldt[selectorToIndex(envSel)] = makeDataDesc(envSeg);
      // Overwrite PSP:0x2C with the PM environment selector
      uint32_t pspPhys = static_cast<uint32_t>(pspSeg) << 4;
      m_memory.write16(pspPhys + 0x2C, envSel);
    }
  }

  // Enable A20 before writing system tables to high memory (LDT_PHYS > 1MB)
  m_memory.setA20(true);

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

    // Initialize PM vector table to match the IDT HLE stubs.
    // When clients read vectors via 0x0204, they get valid "old" handler
    // addresses they can chain to.
    m_pmVectors[v].selector = 0x0008;
    m_pmVectors[v].offset = stubPhysAddr;
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
    handlePageLocking();
    return true;
  case 0x07:
    // Page demand paging candidates / discard — no-op success
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    return true;
  case 0x09:
    handleVirtualInterrupt();
    return true;
  case 0x0A:
    // Vendor specific API — not supported
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    m_cpu.setReg16(cpu::AX, 0x8001);
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
    LOG_WARN("DPMI 0002h: Seg-to-Desc seg=0x", std::hex, seg,
             " base=0x", static_cast<uint32_t>(seg) << 4);
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
      reloadAffectedSegments(sel);
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
      reloadAffectedSegments(sel);
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
      uint8_t accessByte = rights & 0xFF;
      // DPMI spec: Present bit (bit 7) must be set and DPL must match CPL.
      // If invalid, reject the call without modifying the descriptor.
      uint8_t present = (accessByte >> 7) & 1;
      uint8_t dpl = (accessByte >> 5) & 3;
      if (!present || dpl != 3) {
        LOG_DEBUG("DPMI 0009h: Rejected sel=0x", std::hex, sel,
                  " CX=0x", rights, " (P=", (int)present, " DPL=", (int)dpl, ")");
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
        m_cpu.setReg16(cpu::AX, 0x8021);
        break;
      }
      // CL = access byte (bits 8-15 of high dword)
      // CH bits 4-7 = G/D/L/AVL (bits 20-23 of high dword)
      uint8_t extNibble = (rights >> 12) & 0x0F;
      // Always update the access byte. For the flags nibble (G/D/L/AVL),
      // only update when the caller provides non-zero extended type.
      // 16-bit LAR (used by DOS/4GW's 16-bit stub code) can't capture
      // the flags nibble; passing CH=0 should not clear D/G on
      // descriptors that already have them set.
      d.high = (d.high & ~0x0000FF00U) |
               (static_cast<uint32_t>(accessByte) << 8);
      if (extNibble != 0) {
        d.high = (d.high & ~0x00F00000U) |
                 (static_cast<uint32_t>(extNibble) << 20);
      }
      flushLDTEntry(idx);
      reloadAffectedSegments(sel);
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
    uint8_t ptrReg = m_is32BitClient ? cpu::EDI : cpu::DI;
    int idx = selectorToIndex(sel);
    if (idx == 0) {
      // Null selector (GDT[0]): return 8 zero bytes (the null descriptor).
      // DOS/4GW queries selector 0 during dispatcher init and expects
      // CF=0 with an all-zero descriptor (access byte 0 = "not present").
      uint32_t addr = getLinearAddr(cpu::ES, ptrReg);
      m_memory.write32(addr, 0);
      m_memory.write32(addr + 4, 0);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else if (idx < MAX_LDT && m_ldtUsed[idx]) {
      uint32_t addr = getLinearAddr(cpu::ES, ptrReg);
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
    uint8_t ptrReg = m_is32BitClient ? cpu::EDI : cpu::DI;
    int idx = selectorToIndex(sel);
    if (idx > 0 && idx < MAX_LDT && m_ldtUsed[idx]) {
      uint32_t addr = getLinearAddr(cpu::ES, ptrReg);
      uint32_t oldBase = extractBase(m_ldt[idx]);
      uint32_t newLow = m_memory.read32(addr);
      uint32_t newHigh = m_memory.read32(addr + 4);
      // Read the CURRENT descriptor from physical LDT memory, as the
      // program may have written directly to it via the flat data segment.
      uint32_t ldtAddr = LDT_PHYS + idx * 8;
      uint32_t physLow = m_memory.read32(ldtAddr);
      uint32_t physHigh = m_memory.read32(ldtAddr + 4);
      // Sync m_ldt from physical memory in case it was modified externally
      m_ldt[idx].low = physLow;
      m_ldt[idx].high = physHigh;
      // If incoming descriptor has access byte = 0 (Not Present, no type),
      // preserve existing access byte and flags from current descriptor.
      // DOS/4GW sets base+limit via 0x000C with zero access, expecting
      // existing access rights to persist.
      uint8_t incomingAccess = (newHigh >> 8) & 0xFF;
      if (incomingAccess == 0) {
        // Preserve both access byte and flags nibble when access is 0.
        // DOS/4GW sets base+limit via 0x000C with zero access, expecting
        // existing access rights to persist.
        newHigh = (newHigh & ~0x00F0FF00U) |
                  (m_ldt[idx].high & 0x00F0FF00U);
      }
      m_ldt[idx].low = newLow;
      m_ldt[idx].high = newHigh;
      uint32_t newBase = extractBase(m_ldt[idx]);
      LOG_INFO("DPMI 000Ch: Set Descriptor sel=0x", std::hex, sel,
               " base 0x", oldBase, "->0x", newBase,
               " CS=0x", m_cpu.getSegReg(cpu::CS),
               " SS=0x", m_cpu.getSegReg(cpu::SS),
               " DS=0x", m_cpu.getSegReg(cpu::DS),
               " ES=0x", m_cpu.getSegReg(cpu::ES));
      flushLDTEntry(idx);
      reloadAffectedSegments(sel);
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
  case 0x0202: { // Get Processor Exception Handler Vector
    if (vec < 32) {
      m_cpu.setReg16(cpu::CX, m_excVectors[vec].selector);
      m_cpu.setReg32(cpu::EDX, m_excVectors[vec].offset);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      m_cpu.setReg16(cpu::AX, 0x8021);
    }
    break;
  }
  case 0x0203: { // Set Processor Exception Handler Vector
    if (vec < 32) {
      m_excVectors[vec].selector = m_cpu.getReg16(cpu::CX);
      m_excVectors[vec].offset = m_cpu.getReg32(cpu::EDX);
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    } else {
      m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      m_cpu.setReg16(cpu::AX, 0x8021);
    }
    break;
  }
  case 0x0204: { // Get Protected Mode Interrupt Vector
    m_cpu.setReg16(cpu::CX, m_pmVectors[vec].selector);
    if (m_is32BitClient)
      m_cpu.setReg32(cpu::EDX, m_pmVectors[vec].offset);
    else
      m_cpu.setReg16(cpu::DX,
                     static_cast<uint16_t>(m_pmVectors[vec].offset));
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    break;
  }
  case 0x0205: { // Set Protected Mode Interrupt Vector
    uint16_t newSel = m_cpu.getReg16(cpu::CX);
    uint32_t newOff = m_is32BitClient ? m_cpu.getReg32(cpu::EDX)
                                      : m_cpu.getReg16(cpu::DX);
      // Skip updates (both m_pmVectors and IDT) when the caller is restoring our
      // own HLE stub for this vector. That is a save/restore operation by a
      // sub-host (e.g. DOS/4GW cleanup) and must not disturb the chain-wrappers
      // that are currently installed.
      {
        uint32_t hleStub = 0xF0000u + 0x0100u + static_cast<uint32_t>(vec) * 4;
        bool isRestoringHleStub = (newSel == 0x0008 && newOff == hleStub);
        if (!isRestoringHleStub) {
          m_pmVectors[vec].selector = newSel;
          m_pmVectors[vec].offset = newOff;
        uint32_t idtAddr = IDT_PM_PHYS + vec * 8;
        uint32_t low =
            (static_cast<uint32_t>(newSel) << 16) | (newOff & 0xFFFF);
        uint32_t high = (newOff & 0xFFFF0000) | 0x0000EE00;
        {
          static uint32_t s_0205log = 0;
          if (s_0205log < 512) {
            ++s_0205log;
            uint32_t oldLow = m_memory.read32(idtAddr);
            uint32_t oldHigh = m_memory.read32(idtAddr + 4);
            uint16_t oldSel = (oldLow >> 16) & 0xFFFF;
            uint32_t oldOff = (oldLow & 0xFFFF) | (oldHigh & 0xFFFF0000);
            bool host31 = (m_pmVectors[0x31].selector == 0x08);
            LOG_INFO("[0205 #", std::dec, s_0205log,
                     "] vec=0x", std::hex, static_cast<uint32_t>(vec),
                     " new=", newSel, ":", newOff,
                     " old=", oldSel, ":", oldOff,
                     " host31=", host31 ? 1 : 0);
          }
        }
        m_memory.write32(idtAddr, low);
        m_memory.write32(idtAddr + 4, high);
      }
    }

    // Phase 6: compare authoritative m_pmVectors against physical IDT after
    // each 0205 update for this vector and key chain-critical vectors.
    {
      auto logVecState = [&](uint8_t v, const char *tag) {
        uint32_t idtAddr = IDT_PM_PHYS + static_cast<uint32_t>(v) * 8;
        uint32_t idtLow = m_memory.read32(idtAddr);
        uint32_t idtHigh = m_memory.read32(idtAddr + 4);
        uint16_t idtSel = static_cast<uint16_t>((idtLow >> 16) & 0xFFFF);
        uint32_t idtOff = (idtLow & 0xFFFF) | (idtHigh & 0xFFFF0000);
        uint16_t pmSel = m_pmVectors[v].selector;
        uint32_t pmOff = m_pmVectors[v].offset;
        bool mismatch = (idtSel != pmSel) || (idtOff != pmOff);
        if (mismatch) {
          static uint32_t s_phase6Mismatch = 0;
          if (s_phase6Mismatch < 128) {
            ++s_phase6Mismatch;
            LOG_INFO("[phase6-0205-mismatch #", std::dec, s_phase6Mismatch,
                     "] tag=", tag,
                     " vec=0x", std::hex, static_cast<uint32_t>(v),
                     " idt=", idtSel, ":", idtOff,
                     " pm=", pmSel, ":", pmOff,
                     " ax=0x", m_cpu.getReg16(cpu::AX),
                     " bx=0x", m_cpu.getReg16(cpu::BX),
                     " cs=0x", m_cpu.getSegReg(cpu::CS),
                     " eip=0x", m_cpu.getEIP());
          }
        }
      };

      logVecState(vec, "updated");
      if (vec != 0x21)
        logVecState(0x21, "watch");
      if (vec != 0x31)
        logVecState(0x31, "watch");
      if (vec != 0x3D)
        logVecState(0x3D, "watch");
    }

    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
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
  case 0x0302: // Call Real Mode Procedure With IRET Frame
  {
    uint8_t intNo = m_cpu.getReg8(cpu::BL);
    uint32_t structAddr = getLinearAddr(cpu::ES, cpu::EDI);
    uint32_t reqRmEax = m_is32BitClient ? m_memory.read32(structAddr + 0x1C)
                                        : m_memory.read16(structAddr + 0x1C);
    uint32_t reqRmEbx = m_is32BitClient ? m_memory.read32(structAddr + 0x10)
                                        : m_memory.read16(structAddr + 0x10);
    uint32_t reqRmEdx = m_is32BitClient ? m_memory.read32(structAddr + 0x14)
                                        : m_memory.read16(structAddr + 0x14);

    // Phase 6: runtime consistency snapshot near DOS call flow that leads to
    // interrupt-chain validation.
    if (func == 0x0302) {
      uint16_t ax16 = static_cast<uint16_t>(reqRmEax & 0xFFFF);
      bool watchCall = (ax16 == 0x4400) || (ax16 == 0x3D00) ||
                       (ax16 == 0x3F82) || (ax16 == 0x4200);
      static uint32_t s_phase6Snap = 0;
      if (watchCall && s_phase6Snap < 96) {
        auto logSnap = [&](uint8_t v) {
          uint32_t idtAddr = IDT_PM_PHYS + static_cast<uint32_t>(v) * 8;
          uint32_t idtLow = m_memory.read32(idtAddr);
          uint32_t idtHigh = m_memory.read32(idtAddr + 4);
          uint16_t idtSel = static_cast<uint16_t>((idtLow >> 16) & 0xFFFF);
          uint32_t idtOff = (idtLow & 0xFFFF) | (idtHigh & 0xFFFF0000);
          uint16_t pmSel = m_pmVectors[v].selector;
          uint32_t pmOff = m_pmVectors[v].offset;
          bool mismatch = (idtSel != pmSel) || (idtOff != pmOff);
          LOG_INFO("[phase6-0302-snap #", std::dec, s_phase6Snap,
                   "] vec=0x", std::hex, static_cast<uint32_t>(v),
                   " idt=", idtSel, ":", idtOff,
                   " pm=", pmSel, ":", pmOff,
                   " mm=", mismatch ? 1 : 0,
                   " reqAX=0x", ax16,
                   " reqBX=0x", (reqRmEbx & 0xFFFF),
                   " reqDX=0x", (reqRmEdx & 0xFFFF));
        };
        ++s_phase6Snap;
        logSnap(0x21);
        logSnap(0x31);
        logSnap(0x3D);
      }
    }

    // Save current PM state
    uint32_t savedRegs[8];
    for (int i = 0; i < 8; i++)
      savedRegs[i] = m_cpu.getReg32(i);
    uint16_t savedSegs[6];
    uint32_t savedSegBases[6];
    for (int i = 0; i < 6; i++) {
      savedSegs[i] = m_cpu.getSegReg(i);
      savedSegBases[i] = m_cpu.getSegBase(i);
    }
    uint32_t savedEIP = m_cpu.getEIP();
    uint32_t savedEFLAGS = m_cpu.getEFLAGS();
    bool savedIs32Code = m_cpu.is32BitCode();
    bool savedIs32Stack = m_cpu.is32BitStack();

    if (func == 0x0302) {
      static uint32_t s_trace0302Enter = 0;
      bool interesting = ((reqRmEax & 0xFFFF) == 0x4400) ||
                         (savedRegs[cpu::ESP] < 0x1000) ||
                         (savedSegs[cpu::SS] == 0x0177) ||
                         (savedSegs[cpu::SS] == 0x00AF);
      if ((interesting || s_trace0302Enter < 8) && s_trace0302Enter < 96) {
        ++s_trace0302Enter;
        LOG_INFO("[0302-enter #", std::dec, s_trace0302Enter,
                 "] int=0x", std::hex, static_cast<uint32_t>(intNo),
                 " reqAX=0x", (reqRmEax & 0xFFFF),
                 " reqBX=0x", (reqRmEbx & 0xFFFF),
                 " reqDX=0x", (reqRmEdx & 0xFFFF),
                 " pmSS=0x", savedSegs[cpu::SS],
                 " pmESP=0x", savedRegs[cpu::ESP],
                 " pmSP=0x", (savedRegs[cpu::ESP] & 0xFFFF),
                 " pmStack32=", savedIs32Stack ? 1 : 0,
                 " pmCode32=", savedIs32Code ? 1 : 0,
                 " cs=0x", savedSegs[cpu::CS],
                 " eip=0x", savedEIP,
                 " struct=0x", structAddr);
      }
    }

    // Load registers from the RM call structure (50 bytes):
    // +00: EDI, +04: ESI, +08: EBP, +0C: reserved
    // +10: EBX, +14: EDX, +18: ECX, +1C: EAX
    // +20: FLAGS, +22: ES, +24: DS, +26: FS, +28: GS
    // +2A: IP, +2C: CS, +2E: SP, +30: SS
    if (m_is32BitClient) {
      m_cpu.setReg32(cpu::EDI, m_memory.read32(structAddr + 0x00));
      m_cpu.setReg32(cpu::ESI, m_memory.read32(structAddr + 0x04));
      m_cpu.setReg32(cpu::EBP, m_memory.read32(structAddr + 0x08));
      m_cpu.setReg32(cpu::EBX, m_memory.read32(structAddr + 0x10));
      m_cpu.setReg32(cpu::EDX, m_memory.read32(structAddr + 0x14));
      m_cpu.setReg32(cpu::ECX, m_memory.read32(structAddr + 0x18));
      m_cpu.setReg32(cpu::EAX, m_memory.read32(structAddr + 0x1C));
    } else {
      // 16-bit clients use only low halves in this structure.
      // Zero-extend to avoid propagating stale high-word garbage.
      m_cpu.setReg32(cpu::EDI, m_memory.read16(structAddr + 0x00));
      m_cpu.setReg32(cpu::ESI, m_memory.read16(structAddr + 0x04));
      m_cpu.setReg32(cpu::EBP, m_memory.read16(structAddr + 0x08));
      m_cpu.setReg32(cpu::EBX, m_memory.read16(structAddr + 0x10));
      m_cpu.setReg32(cpu::EDX, m_memory.read16(structAddr + 0x14));
      m_cpu.setReg32(cpu::ECX, m_memory.read16(structAddr + 0x18));
      m_cpu.setReg32(cpu::EAX, m_memory.read16(structAddr + 0x1C));
    }

    uint16_t rmFlags = m_memory.read16(structAddr + 0x20);
    uint16_t rmES = m_memory.read16(structAddr + 0x22);
    uint16_t rmDS = m_memory.read16(structAddr + 0x24);
    uint16_t rmFS = m_memory.read16(structAddr + 0x26);
    uint16_t rmGS = m_memory.read16(structAddr + 0x28);
    uint16_t rmSP = m_memory.read16(structAddr + 0x2E);
    uint16_t rmSS = m_memory.read16(structAddr + 0x30);

    // Temporarily set RM state for the call
    m_cpu.setEFLAGS((m_cpu.getEFLAGS() & 0xFFFF0000) | rmFlags);
    m_cpu.setIs32BitCode(false);
    m_cpu.setIs32BitStack(false);

    // Set RM segments
    m_cpu.setSegReg(cpu::ES, rmES);
    m_cpu.setSegBase(cpu::ES, static_cast<uint32_t>(rmES) << 4);
    m_cpu.setSegReg(cpu::DS, rmDS);
    m_cpu.setSegBase(cpu::DS, static_cast<uint32_t>(rmDS) << 4);
    m_cpu.setSegReg(cpu::FS, rmFS);
    m_cpu.setSegBase(cpu::FS, static_cast<uint32_t>(rmFS) << 4);
    m_cpu.setSegReg(cpu::GS, rmGS);
    m_cpu.setSegBase(cpu::GS, static_cast<uint32_t>(rmGS) << 4);
    m_cpu.setSegReg(cpu::SS, rmSS);
    m_cpu.setSegBase(cpu::SS, static_cast<uint32_t>(rmSS) << 4);
    m_cpu.setReg16(cpu::SP, rmSP);

    if (func == 0x0302) {
      static uint32_t s_trace0302Armed = 0;
      bool interesting = ((reqRmEax & 0xFFFF) == 0x4400) ||
                         (rmSS == 0x0177) || (rmSS == 0x00AF) ||
                         (rmSP < 0x1000);
      if ((interesting || s_trace0302Armed < 8) && s_trace0302Armed < 96) {
        ++s_trace0302Armed;
        LOG_INFO("[0302-rm-armed #", std::dec, s_trace0302Armed,
                 "] int=0x", std::hex, static_cast<uint32_t>(intNo),
                 " rmSS=0x", m_cpu.getSegReg(cpu::SS),
                 " rmSP=0x", m_cpu.getReg16(cpu::SP),
                 " rmESP=0x", m_cpu.getReg32(cpu::ESP),
                 " stack32=", m_cpu.is32BitStack() ? 1 : 0,
                 " code32=", m_cpu.is32BitCode() ? 1 : 0,
                 " cs=0x", m_cpu.getSegReg(cpu::CS),
                 " eip=0x", m_cpu.getEIP());
      }
    }

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
    } else {
      // func == 0x0301/0x0302: Call RM procedure at CS:IP from the structure
      uint16_t rmIP = m_memory.read16(structAddr + 0x2A);
      uint16_t rmCS = m_memory.read16(structAddr + 0x2C);
      m_cpu.setSegReg(cpu::CS, rmCS);
      m_cpu.setSegBase(cpu::CS, static_cast<uint32_t>(rmCS) << 4);
      m_cpu.setEIP(rmIP);
      {
        uint32_t rmEAX = m_memory.read32(structAddr + 0x1C);
        uint32_t rmEBX = m_memory.read32(structAddr + 0x10);
        uint32_t rmEDX = m_memory.read32(structAddr + 0x14);
        LOG_INFO("DPMI 030", (int)(func & 0xF),
                 "h: Call RM proc at ", std::hex, rmCS, ":", rmIP,
                 " AX=", rmEAX & 0xFFFF,
                 " BX=", rmEBX & 0xFFFF,
                 " DX=", rmEDX & 0xFFFF,
                 " structAddr=", structAddr);
      }
      // Try HLE at the target address
      bool handled = false;
      // Check if the target is one of our HLE stubs: F000:(0x100 + v*4).
      // We match against the known stub layout directly rather than the
      // runtime IVT, which may have been hooked by DOS extenders/TSRs.
      if (m_bios && rmCS == 0xF000 && rmIP >= 0x0100 &&
          rmIP < 0x0100 + 256 * 4 && ((rmIP - 0x0100) & 3) == 0) {
        int v = (rmIP - 0x0100) / 4;
        if (m_dos && m_dos->handleInterrupt(v))
          handled = true;
        if (!handled && m_bios->handleInterrupt(v))
          handled = true;
        if (!handled) {
          // The target is our HLE stub but neither DOS nor BIOS handles
          // this vector. On real hardware, executing the stub would just
          // IRET (it's a dummy). Treat as successful no-op.
          LOG_DEBUG("DPMI 030", (int)(func & 0xF),
                    "h: stub for INT 0x", std::hex, v, " — no-op");
          handled = true;
        }
      }
      // Also check the runtime IVT as fallback for non-standard stubs
      if (!handled && m_bios) {
        for (int v = 0; v < 256; v++) {
          uint16_t ivtIP = m_memory.read16(v * 4);
          uint16_t ivtCS = m_memory.read16(v * 4 + 2);
          if (ivtCS == rmCS && ivtIP == rmIP) {
            if (m_dos && m_dos->handleInterrupt(v)) {
              handled = true;
              break;
            }
            if (m_bios->handleInterrupt(v)) {
              handled = true;
              break;
            }
          }
        }
      }
      if (!handled) {
        LOG_WARN("DPMI 030", (int)(func & 0xF),
                 "h: RM proc at ", std::hex, rmCS, ":", rmIP, " not handled");
        // Signal failure to the caller — set CF in the returned flags
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
      }
    }

    // Write back modified registers to the call structure
    {
      uint16_t retFlags = static_cast<uint16_t>(m_cpu.getEFLAGS() & 0xFFFF);
      bool retCF = !!(retFlags & cpu::FLAG_CARRY);
      /*
      LOG_INFO("DPMI 030xh result: AX=", std::hex,
                m_cpu.getReg16(cpu::AX),
                " BX=", m_cpu.getReg16(cpu::BX),
                " CF=", retCF ? 1 : 0,
                " ES=", m_cpu.getSegReg(cpu::ES));
                */
    }
    if (m_is32BitClient) {
      m_memory.write32(structAddr + 0x00, m_cpu.getReg32(cpu::EDI));
      m_memory.write32(structAddr + 0x04, m_cpu.getReg32(cpu::ESI));
      m_memory.write32(structAddr + 0x08, m_cpu.getReg32(cpu::EBP));
      m_memory.write32(structAddr + 0x10, m_cpu.getReg32(cpu::EBX));
      m_memory.write32(structAddr + 0x14, m_cpu.getReg32(cpu::EDX));
      m_memory.write32(structAddr + 0x18, m_cpu.getReg32(cpu::ECX));
      m_memory.write32(structAddr + 0x1C, m_cpu.getReg32(cpu::EAX));
    } else {
      // 16-bit client return: write low halves and clear high halves.
      m_memory.write32(structAddr + 0x00,
                       static_cast<uint32_t>(m_cpu.getReg16(cpu::DI)));
      m_memory.write32(structAddr + 0x04,
                       static_cast<uint32_t>(m_cpu.getReg16(cpu::SI)));
      m_memory.write32(structAddr + 0x08,
                       static_cast<uint32_t>(m_cpu.getReg16(cpu::BP)));
      m_memory.write32(structAddr + 0x10,
                       static_cast<uint32_t>(m_cpu.getReg16(cpu::BX)));
      m_memory.write32(structAddr + 0x14,
                       static_cast<uint32_t>(m_cpu.getReg16(cpu::DX)));
      m_memory.write32(structAddr + 0x18,
                       static_cast<uint32_t>(m_cpu.getReg16(cpu::CX)));
      m_memory.write32(structAddr + 0x1C,
                       static_cast<uint32_t>(m_cpu.getReg16(cpu::AX)));
    }
    m_memory.write16(structAddr + 0x20,
                     static_cast<uint16_t>(m_cpu.getEFLAGS() & 0xFFFF));
    m_memory.write16(structAddr + 0x22, m_cpu.getSegReg(cpu::ES));
    m_memory.write16(structAddr + 0x24, m_cpu.getSegReg(cpu::DS));
    m_memory.write16(structAddr + 0x26, m_cpu.getSegReg(cpu::FS));
    m_memory.write16(structAddr + 0x28, m_cpu.getSegReg(cpu::GS));
    m_memory.write16(structAddr + 0x2A,
             static_cast<uint16_t>(m_cpu.getEIP() & 0xFFFF));
    m_memory.write16(structAddr + 0x2C, m_cpu.getSegReg(cpu::CS));
    m_memory.write16(structAddr + 0x2E, m_cpu.getReg16(cpu::SP));
    m_memory.write16(structAddr + 0x30, m_cpu.getSegReg(cpu::SS));

    if (func == 0x0302) {
      static uint32_t s_trace0302PreRestore = 0;
      bool interesting = ((reqRmEax & 0xFFFF) == 0x4400) ||
                         (m_cpu.getReg16(cpu::SP) < 0x1000) ||
                         (m_cpu.getSegReg(cpu::SS) == 0x0177) ||
                         (m_cpu.getSegReg(cpu::SS) == 0x00AF);
      if ((interesting || s_trace0302PreRestore < 8) &&
          s_trace0302PreRestore < 96) {
        ++s_trace0302PreRestore;
        LOG_INFO("[0302-pre-restore #", std::dec, s_trace0302PreRestore,
                 "] int=0x", std::hex, static_cast<uint32_t>(intNo),
                 " reqAX=0x", (reqRmEax & 0xFFFF),
                 " curSS=0x", m_cpu.getSegReg(cpu::SS),
                 " curSP=0x", m_cpu.getReg16(cpu::SP),
                 " curESP=0x", m_cpu.getReg32(cpu::ESP),
                 " stack32=", m_cpu.is32BitStack() ? 1 : 0,
                 " code32=", m_cpu.is32BitCode() ? 1 : 0,
                 " retAX=0x", m_cpu.getReg16(cpu::AX),
                 " retCF=", (m_cpu.getEFLAGS() & cpu::FLAG_CARRY) ? 1 : 0);
      }
    }

    // Restore PM state
    for (int i = 0; i < 8; i++)
      m_cpu.setReg32(i, savedRegs[i]);
    for (int i = 0; i < 6; i++) {
      m_cpu.setSegReg(i, savedSegs[i]);
      m_cpu.setSegBase(i, savedSegBases[i]);
    }
    m_cpu.setEIP(savedEIP);
    m_cpu.setEFLAGS(savedEFLAGS & ~cpu::FLAG_CARRY);
    m_cpu.setIs32BitCode(savedIs32Code);
    m_cpu.setIs32BitStack(savedIs32Stack);

    if (func == 0x0302) {
      static uint32_t s_trace0302Restored = 0;
      bool mismatch = (m_cpu.getSegReg(cpu::SS) != savedSegs[cpu::SS]) ||
                      (m_cpu.getReg32(cpu::ESP) != savedRegs[cpu::ESP]) ||
                      (m_cpu.is32BitStack() != savedIs32Stack);
      bool interesting = ((reqRmEax & 0xFFFF) == 0x4400) || mismatch ||
                         (savedRegs[cpu::ESP] < 0x1000) ||
                         (savedSegs[cpu::SS] == 0x0177) ||
                         (savedSegs[cpu::SS] == 0x00AF);
      if ((interesting || s_trace0302Restored < 8) &&
          s_trace0302Restored < 96) {
        ++s_trace0302Restored;
        LOG_INFO("[0302-restored #", std::dec, s_trace0302Restored,
                 "] int=0x", std::hex, static_cast<uint32_t>(intNo),
                 " reqAX=0x", (reqRmEax & 0xFFFF),
                 " ss=0x", m_cpu.getSegReg(cpu::SS),
                 " esp=0x", m_cpu.getReg32(cpu::ESP),
                 " sp=0x", m_cpu.getReg16(cpu::SP),
                 " stack32=", m_cpu.is32BitStack() ? 1 : 0,
                 " code32=", m_cpu.is32BitCode() ? 1 : 0,
                 " savedSS=0x", savedSegs[cpu::SS],
                 " savedESP=0x", savedRegs[cpu::ESP],
                 " mismatch=", mismatch ? 1 : 0);
      }
    }
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
  case 0x0305: { // Get State Save/Restore Addresses
    // Return size=0 (no state to save) and dummy addresses
    m_cpu.setReg16(cpu::AX, 0); // Buffer size = 0
    m_cpu.setReg16(cpu::BX, 0xF000);
    m_cpu.setReg16(cpu::CX, 0x0063); // RM procedure at F000:0063
    m_cpu.setReg16(cpu::SI, 0x0008); // PM selector (flat code)
    m_cpu.setReg32(cpu::EDI, 0xF0063); // PM offset (same phys addr)
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    break;
  }
  case 0x0306: { // Get Raw Mode Switch Addresses
    // PM→RM switch: PM JMP FAR to selector 0x08 : 0x0500
    // Stub at physical 0x500 (low address reachable with 16-bit offset)
    m_cpu.setReg16(cpu::SI, 0x0008); // PM selector (flat code)
    m_cpu.setReg32(cpu::EDI, 0x0500); // PM offset (fits in 16 bits)
    // RM→PM switch: RM CALL FAR to F000:005D
    m_cpu.setReg16(cpu::BX, 0xF000);
    m_cpu.setReg16(cpu::CX, 0x005D);
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
    uint8_t ptrReg = m_is32BitClient ? cpu::EDI : cpu::DI;
    uint32_t addr = getLinearAddr(cpu::ES, ptrReg);
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
    LOG_INFO("DPMI 0500h: Free memory = ", freeBytes, " bytes (", freeBytes / 1024, " KB)");
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
        m_memBlocks.push_back({blockHandle, linearAddr, size, handle});
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
        LOG_INFO("DPMI 0501h: Alloc ", size, " bytes -> linear=0x", std::hex,
                  linearAddr, " handle=0x", blockHandle);
      } else {
        if (handle)
          m_himem->freeEMB(handle);
        LOG_WARN("DPMI 0501h: FAILED alloc ", size, " bytes (handle=", handle, ")");
        m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
        m_cpu.setReg16(cpu::AX, 0x8012);
      }
    } else {
      LOG_WARN("DPMI 0501h: FAILED alloc ", size, " bytes (no HIMEM)");
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
      if (it->handle == blockHandle) {
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
void DPMI::handlePageLocking() {
  uint16_t func = m_cpu.getReg16(cpu::AX);
  switch (func) {
  case 0x0600: // Lock Linear Region — no-op success
  case 0x0601: // Unlock Linear Region — no-op success
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    break;
  case 0x0604: // Get Page Size
    m_cpu.setReg16(cpu::BX, 0);
    m_cpu.setReg16(cpu::CX, 4096);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    break;
  default:
    LOG_WARN("DPMI: Unhandled page function 0x", std::hex, func);
    m_cpu.setEFLAGS(m_cpu.getEFLAGS() | cpu::FLAG_CARRY);
    m_cpu.setReg16(cpu::AX, 0x8001);
    break;
  }
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
      for (int j = 0; j < count; ++j) {
        m_ldtUsed[i + j] = true;
        m_ldtBatchAlloc[i + j] = (count > 1);
        // DPMI spec: newly allocated descriptors have Present bit set,
        // type = data R/W, DPL = 3, base = 0, limit = 0.
        m_ldt[i + j] = buildDescriptor(0, 0, 0xF2, 0x00);
        flushLDTEntry(i + j);
      }
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

void DPMI::reloadAffectedSegments(uint16_t sel) {
  if (!m_segReloadFn)
    return;
  // Check each segment register; if it holds the modified selector, reload it.
  static constexpr uint8_t segs[] = {cpu::ES, cpu::CS, cpu::SS,
                                     cpu::DS, cpu::FS, cpu::GS};
  static constexpr const char *segNames[] = {"ES", "CS", "SS", "DS", "FS", "GS"};
  for (uint8_t s : segs) {
    if (m_cpu.getSegReg(s) == sel) {
      uint32_t oldBase = m_cpu.getSegBase(s);
      m_segReloadFn(s, sel);
      uint32_t newBase = m_cpu.getSegBase(s);
      if (oldBase != newBase) {
        LOG_INFO("DPMI: Reloaded ", segNames[s], " sel=0x", std::hex, sel,
                 " base 0x", oldBase, " -> 0x", newBase);
      }
    }
  }
}

uint32_t DPMI::getLinearAddr(uint8_t segReg, uint8_t reg) const {
  uint32_t base = m_cpu.getSegBase(segReg);
  uint32_t offset = m_is32BitClient ? m_cpu.getReg32(reg)
                                    : m_cpu.getReg16(reg);
  return base + offset;
}

// ── Raw mode switch: PM → RM (vector 0xE2) ──────────────────────────────────
// Called via JMP FAR to 0x08:0xF005A (flat PM address).
// Registers: AX=new DS seg, CX=new CS seg, DX=new SS seg,
//            BX=new SP, SI=new IP, EDI preserved.
void DPMI::handleRawSwitchPMtoRM() {
  uint16_t newDS = m_cpu.getReg16(cpu::AX);
  uint16_t newCS = m_cpu.getReg16(cpu::CX);
  uint16_t newSS = m_cpu.getReg16(cpu::DX);
  uint16_t newSP = m_cpu.getReg16(cpu::BX);
  uint16_t newIP = m_cpu.getReg16(cpu::SI);

  LOG_DEBUG("DPMI: Raw switch PM→RM CS=", std::hex, newCS, " IP=", newIP,
            " DS=", newDS, " SS:SP=", newSS, ":", newSP);

  // Switch to real mode
  m_cpu.setCR(0, m_cpu.getCR(0) & ~1u);

  // Set real-mode segments
  m_cpu.setSegReg(cpu::CS, newCS);
  m_cpu.setSegReg(cpu::DS, newDS);
  m_cpu.setSegReg(cpu::SS, newSS);
  m_cpu.setSegReg(cpu::ES, newDS); // ES defaults to DS per convention
  m_cpu.setSegReg(cpu::FS, 0);
  m_cpu.setSegReg(cpu::GS, 0);
  m_cpu.setSegBase(cpu::CS, static_cast<uint32_t>(newCS) << 4);
  m_cpu.setSegBase(cpu::DS, static_cast<uint32_t>(newDS) << 4);
  m_cpu.setSegBase(cpu::SS, static_cast<uint32_t>(newSS) << 4);
  m_cpu.setSegBase(cpu::ES, static_cast<uint32_t>(newDS) << 4);
  m_cpu.setSegBase(cpu::FS, 0);
  m_cpu.setSegBase(cpu::GS, 0);

  m_cpu.setReg16(cpu::SP, newSP);
  m_cpu.setEIP(newIP);

  m_cpu.setIs32BitCode(false);
  m_cpu.setIs32BitStack(false);
}

// ── Raw mode switch: RM → PM (vector 0xE3) ──────────────────────────────────
// Called via JMP FAR to F000:005D (real-mode address).
// Registers: AX=new DS sel, CX=new CS sel, DX=new SS sel,
//            (E)BX=new ESP, SI=new EIP, EDI preserved.
void DPMI::handleRawSwitchRMtoPM() {
  uint16_t newDS = m_cpu.getReg16(cpu::AX);
  uint16_t newCS = m_cpu.getReg16(cpu::CX);
  uint16_t newSS = m_cpu.getReg16(cpu::DX);
  uint32_t newESP = m_cpu.getReg32(cpu::EBX);
  uint32_t newEIP = m_cpu.getReg32(cpu::ESI);

  LOG_DEBUG("DPMI: Raw switch RM→PM CS=", std::hex, newCS, " EIP=", newEIP,
            " DS=", newDS, " SS:ESP=", newSS, ":", newESP);

  // Switch to protected mode
  m_cpu.setCR(0, m_cpu.getCR(0) | 1);

  // Set PM selectors
  m_cpu.setSegReg(cpu::CS, newCS);
  m_cpu.setSegReg(cpu::DS, newDS);
  m_cpu.setSegReg(cpu::SS, newSS);
  m_cpu.setSegReg(cpu::ES, newDS);
  m_cpu.setSegReg(cpu::FS, 0);
  m_cpu.setSegReg(cpu::GS, 0);

  auto decodePMSelector = [&](uint16_t sel, uint32_t &base, bool &is32,
                              bool &valid) {
    base = 0;
    is32 = false;
    valid = false;
    if (sel == 0)
      return;

    uint32_t low = 0;
    uint32_t high = 0;
    if (sel & LDT_TI_BIT) {
      int idx = selectorToIndex(sel);
      if (idx <= 0 || idx >= MAX_LDT || !m_ldtUsed[idx])
        return;
      low = m_ldt[idx].low;
      high = m_ldt[idx].high;
    } else {
      auto gdtr = m_cpu.getGDTR();
      uint32_t entry = gdtr.base + (sel & ~7);
      if (entry + 7 > gdtr.base + gdtr.limit)
        return;
      low = m_memory.read32(entry);
      high = m_memory.read32(entry + 4);
    }

    base = ((low >> 16) & 0xFFFF) | ((high & 0xFF) << 16) |
           (high & 0xFF000000);
    is32 = (high & 0x00400000u) != 0;
    valid = true;
  };

  uint32_t csBase = 0, dsBase = 0, ssBase = 0;
  bool cs32 = false, ds32 = false, ss32 = false;
  bool csValid = false, dsValid = false, ssValid = false;
  decodePMSelector(newCS, csBase, cs32, csValid);
  decodePMSelector(newDS, dsBase, ds32, dsValid);
  decodePMSelector(newSS, ssBase, ss32, ssValid);
  m_cpu.setSegBase(cpu::CS, csBase);
  m_cpu.setSegBase(cpu::DS, dsBase);
  m_cpu.setSegBase(cpu::SS, ssBase);
  m_cpu.setSegBase(cpu::ES, dsBase);
  m_cpu.setSegBase(cpu::FS, 0);
  m_cpu.setSegBase(cpu::GS, 0);

  m_cpu.setReg32(cpu::ESP, newESP);
  m_cpu.setEIP(newEIP);

  // Determine 32-bit mode from the CS descriptor's D bit
  if (csValid)
    m_cpu.setIs32BitCode(cs32);
  if (ssValid)
    m_cpu.setIs32BitStack(ss32);
}

} // namespace fador::hw
