#include "VCPI.hpp"
#include "PIC8259.hpp"
#include "../cpu/CPU.hpp"
#include "../memory/MemoryBus.hpp"
#include "../memory/himem/HIMEM.hpp"
#include "../utils/Logger.hpp"

#include <algorithm>

namespace fador::hw {

namespace {
constexpr uint32_t kIF = 0x00000200, kTF = 0x00000100, kNT = 0x00004000;
constexpr uint32_t kRF = 0x00010000, kVM = 0x00020000, kVIF = 0x00080000,
                   kVIP = 0x00100000;
constexpr uint32_t kPagingAndPE = 0x80000001u;
constexpr uint32_t kChunkKB = 1024; // XMS taken per refill of the page pool

// Status codes (AH).
constexpr uint8_t kOk = 0x00;
constexpr uint8_t kNoPages = 0x88;
constexpr uint8_t kBadPage = 0x8A;
constexpr uint8_t kBadPhysicalPage = 0x8B;
constexpr uint8_t kBadFunction = 0x8F;

void writeDescriptor(memory::MemoryBus &mem, uint32_t at, uint32_t base,
                     uint32_t limit, uint8_t access, uint8_t flags) {
  mem.write32(at, (limit & 0xFFFF) | ((base & 0xFFFF) << 16));
  mem.write32(at + 4, ((base >> 16) & 0xFF) | (uint32_t(access) << 8) |
                          (limit & 0xF0000) | (uint32_t(flags) << 20) |
                          (base & 0xFF000000u));
}
} // namespace

VCPI::VCPI(cpu::CPU &cpu, memory::MemoryBus &memory)
    : m_cpu(cpu), m_memory(memory) {}

void VCPI::installEntryStub() {
  // 0F FF E4: HLE trap to handleProtectedEntry(); then RETF (32-bit code
  // segment) back to the client for every function except DE0Ch.
  const uint32_t at = kServerBase + kEntryOffset;
  m_memory.write8(at, 0x0F);
  m_memory.write8(at + 1, 0xFF);
  m_memory.write8(at + 2, kEntryTrap);
  m_memory.write8(at + 3, 0xCB);
}

void VCPI::reset() {
  if (m_himem) {
    for (uint16_t handle : m_chunks) {
      m_himem->unlockEMB(handle);
      m_himem->freeEMB(handle);
    }
  }
  m_chunks.clear();
  m_freePages.clear();
  m_allocated.clear();
}

bool VCPI::handleInt67() {
  if (!m_enabled)
    return false;
  dispatch(false);
  return true;
}

void VCPI::handleProtectedEntry() { dispatch(true); }

uint32_t VCPI::freePageCount() const {
  uint32_t pages = static_cast<uint32_t>(m_freePages.size());
  if (m_himem) {
    uint32_t largest = 0;
    pages += m_himem->queryFreeKB32(largest) / 4;
  }
  return pages;
}

bool VCPI::allocatePage(uint32_t &physical) {
  if (m_freePages.empty()) {
    if (!m_himem)
      return false;
    uint32_t largestKB = 0;
    m_himem->queryFreeKB32(largestKB);
    const uint32_t chunkKB = std::min(kChunkKB + 4, largestKB);
    if (chunkKB < 8)
      return false;
    const uint16_t handle = m_himem->allocateEMB(chunkKB);
    uint32_t linear = 0;
    if (handle == 0 || !m_himem->lockEMB(handle, linear)) {
      if (handle != 0)
        m_himem->freeEMB(handle);
      return false;
    }
    m_chunks.push_back(handle);
    const uint32_t end = linear + chunkKB * 1024;
    // Highest address first so pages are handed out in ascending order.
    for (uint32_t page = (end & ~0xFFFu) - 0x1000; page >= ((linear + 0xFFF) & ~0xFFFu);
         page -= 0x1000)
      m_freePages.push_back(page);
    if (m_freePages.empty())
      return false;
  }
  physical = m_freePages.back();
  m_freePages.pop_back();
  m_allocated.push_back(physical);
  return true;
}

bool VCPI::freePage(uint32_t physical) {
  auto it = std::find(m_allocated.begin(), m_allocated.end(),
                      physical & ~0xFFFu);
  if (it == m_allocated.end())
    return false;
  m_allocated.erase(it);
  m_freePages.push_back(physical & ~0xFFFu);
  return true;
}

void VCPI::dispatch(bool fromProtected) {
  using namespace cpu;
  const uint8_t function = m_cpu.getReg8(AL);
  auto status = [this](uint8_t s) { m_cpu.setReg8(AH, s); };
  // Real-mode callers address buffers with 16-bit offsets.
  auto bufferAt = [&](uint8_t seg, uint8_t reg) {
    return m_cpu.getSegBase(seg) +
           (fromProtected ? m_cpu.getReg32(reg) : m_cpu.getReg16(reg));
  };
  LOG_DEBUG("VCPI: AX=DE", std::hex, static_cast<int>(function),
            fromProtected ? " (protected mode)" : "");
  switch (function) {
  case 0x00: // installation check
    m_cpu.setReg16(BX, 0x0100);
    status(kOk);
    break;
  case 0x01: { // get protected-mode interface
    // Page table 0: identity map of the first megabyte.
    const uint32_t table = bufferAt(ES, EDI);
    for (uint32_t i = 0; i < 256; ++i)
      m_memory.lwrite32(table + i * 4, (i << 12) | 0x67);
    const uint32_t gdt = bufferAt(DS, ESI);
    // Descriptors for the client's GDT: server code (32-bit) and data.
    writeDescriptor(m_memory, gdt, kServerBase, 0xFFFF, 0x9A, 0x4);
    writeDescriptor(m_memory, gdt + 8, kServerBase, 0xFFFF, 0x92, 0x4);
    writeDescriptor(m_memory, gdt + 16, 0, 0xFFFFF, 0x92, 0xC);
    if (fromProtected)
      m_cpu.setReg32(EDI, m_cpu.getReg32(EDI) + 256 * 4);
    else
      m_cpu.setReg16(DI, static_cast<uint16_t>(m_cpu.getReg16(DI) + 256 * 4));
    m_cpu.setReg32(EBX, kEntryOffset);
    status(kOk);
    break;
  }
  case 0x02: // maximum physical address
    m_cpu.setReg32(EDX, memory::MemoryBus::XMS_LIMIT - 0x1000);
    status(kOk);
    break;
  case 0x03: // free 4 KB pages
    m_cpu.setReg32(EDX, freePageCount());
    status(kOk);
    break;
  case 0x04: { // allocate a 4 KB page
    uint32_t page = 0;
    if (allocatePage(page)) {
      m_cpu.setReg32(EDX, page);
      status(kOk);
    } else {
      status(kNoPages);
    }
    break;
  }
  case 0x05: // free a 4 KB page
    status(freePage(m_cpu.getReg32(EDX)) ? kOk : kBadPage);
    break;
  case 0x06: { // physical address of a page in the first megabyte
    const uint16_t page = m_cpu.getReg16(CX);
    if (page >= 0x100) {
      status(kBadPhysicalPage);
      break;
    }
    m_cpu.setReg32(EDX, static_cast<uint32_t>(page) << 12);
    status(kOk);
    break;
  }
  case 0x07: // read CR0
    m_cpu.setReg32(EBX, m_cpu.getCR(0));
    status(kOk);
    break;
  case 0x08: { // read debug registers
    const uint32_t buf = bufferAt(ES, EDI);
    for (uint8_t i = 0; i < 8; ++i)
      m_memory.lwrite32(buf + i * 4u, m_cpu.getDR(i));
    status(kOk);
    break;
  }
  case 0x09: { // load debug registers (DR4/DR5 are skipped)
    const uint32_t buf = bufferAt(ES, EDI);
    for (uint8_t i : {0, 1, 2, 3, 6, 7})
      m_cpu.setDR(i, m_memory.lread32(buf + i * 4u));
    status(kOk);
    break;
  }
  case 0x0A: // 8259 interrupt vector mappings
    m_cpu.setReg16(BX, m_master ? m_master->baseVector() : 0x08);
    m_cpu.setReg16(CX, m_slave ? m_slave->baseVector() : 0x70);
    status(kOk);
    break;
  case 0x0B: // the client reprogrammed the PICs (they hold the new bases)
    status(kOk);
    break;
  case 0x0C:
    if (fromProtected)
      switchToReal();
    else
      switchToProtected();
    break;
  default:
    status(kBadFunction);
    break;
  }
}

void VCPI::switchToProtected() {
  using namespace cpu;
  // ESI: linear address (first megabyte) of
  //   +00 CR3, +04 -> GDTR image, +08 -> IDTR image, +0C LDTR, +0E TR,
  //   +10 EIP, +14 CS of the protected-mode entry.
  const uint32_t s = m_cpu.getReg32(ESI);
  const uint32_t cr3 = m_memory.read32(s);
  const uint32_t gdtr = m_memory.read32(s + 4);
  const uint32_t idtr = m_memory.read32(s + 8);
  const uint16_t ldt = m_memory.read16(s + 0x0C);
  const uint16_t tr = m_memory.read16(s + 0x0E);
  const uint32_t eip = m_memory.read32(s + 0x10);
  const uint16_t cs = m_memory.read16(s + 0x14);
  m_cpu.setGDTR({m_memory.read16(gdtr), m_memory.read32(gdtr + 2)});
  m_cpu.setIDTR({m_memory.read16(idtr), m_memory.read32(idtr + 2)});

  m_savedA20 = m_memory.isA20Enabled();
  m_memory.setA20(true); // V86 servers run with the A20 gate open
  m_cpu.setEFLAGS(m_cpu.getEFLAGS() & ~(kIF | kTF | kNT | kRF | kVM));
  m_cpu.setCR(3, cr3);
  m_cpu.setCR(0, m_cpu.getCR(0) | kPagingAndPE);
  m_cpu.setNativeProtection(true);

  // Descriptor tables are read through the client's page tables now.
  const DescriptorRegister g = m_cpu.getGDTR();
  auto readDesc = [&](uint16_t sel, uint32_t &low, uint32_t &high) {
    if ((sel & 0xFFFC) == 0 || (sel & 0xFFF8u) + 7 > g.limit)
      return false;
    low = m_memory.lread32(g.base + (sel & 0xFFF8u));
    high = m_memory.lread32(g.base + (sel & 0xFFF8u) + 4);
    return true;
  };
  auto baseOf = [](uint32_t low, uint32_t high) {
    return (low >> 16) | ((high & 0xFF) << 16) | (high & 0xFF000000u);
  };
  auto limitOf = [](uint32_t low, uint32_t high) {
    uint32_t limit = (low & 0xFFFF) | (high & 0xF0000);
    return (high & 0x00800000) ? (limit << 12) | 0xFFF : limit;
  };
  uint32_t low = 0, high = 0;
  m_cpu.setLDTRSelector(ldt);
  if (readDesc(ldt, low, high))
    m_cpu.setLDTR({static_cast<uint16_t>(limitOf(low, high)), baseOf(low, high)});
  else
    m_cpu.setLDTR({0, 0});
  m_cpu.setTRSelector(tr);
  if (readDesc(tr, low, high)) {
    // LTR marks the task busy.
    m_memory.lwrite32(g.base + (tr & 0xFFF8u) + 4, high | 0x0200);
    m_cpu.setTR({static_cast<uint16_t>(limitOf(low, high)), baseOf(low, high)});
  }
  m_cpu.loadSegment(CS, cs);
  m_cpu.setEIP(eip);
  for (auto seg : {DS, ES, FS, GS})
    m_cpu.loadSegment(seg, 0);
  LOG_DEBUG("VCPI: switch to protected mode at ", std::hex, cs, ":", eip,
            " CR3=", cr3);
}

void VCPI::switchToReal() {
  using namespace cpu;
  // The far CALL's return address, then EIP CS EFLAGS ESP SS ES DS FS GS
  // of the V86 (here: real-mode) context, one dword each.
  const uint32_t sp = m_cpu.is32BitStack() ? m_cpu.getReg32(ESP)
                                           : m_cpu.getReg16(SP);
  const uint32_t f = m_cpu.getSegBase(SS) + sp + 8;
  const uint32_t eip = m_memory.lread32(f);
  const uint16_t cs = static_cast<uint16_t>(m_memory.lread32(f + 0x04));
  const uint32_t esp = m_memory.lread32(f + 0x0C);
  const uint16_t ss = static_cast<uint16_t>(m_memory.lread32(f + 0x10));
  const uint16_t es = static_cast<uint16_t>(m_memory.lread32(f + 0x14));
  const uint16_t ds = static_cast<uint16_t>(m_memory.lread32(f + 0x18));
  const uint16_t fs = static_cast<uint16_t>(m_memory.lread32(f + 0x1C));
  const uint16_t gs = static_cast<uint16_t>(m_memory.lread32(f + 0x20));

  m_cpu.setEFLAGS(m_cpu.getEFLAGS() &
                  ~(kIF | kTF | kNT | kRF | kVM | kVIF | kVIP));
  m_cpu.setCR(0, m_cpu.getCR(0) & ~kPagingAndPE); // ends native protection
  m_cpu.setIDTR({0x03FF, 0});                     // the real-mode IVT
  m_memory.setA20(m_savedA20);
  m_cpu.loadSegment(SS, ss);
  m_cpu.setReg32(ESP, esp);
  m_cpu.loadSegment(ES, es);
  m_cpu.loadSegment(DS, ds);
  m_cpu.loadSegment(FS, fs);
  m_cpu.loadSegment(GS, gs);
  m_cpu.loadSegment(CS, cs);
  m_cpu.setEIP(eip & 0xFFFF);
  LOG_DEBUG("VCPI: switch to real mode at ", std::hex, cs, ":", eip);
}

} // namespace fador::hw
