#pragma once
// VCPI 1.0 server (INT 67h AH=DEh) on top of the EMS driver.
//
// A real VCPI server (EMM386, QEMM) runs DOS in virtual-8086 mode and
// switches its clients between V86 and protected mode.  This emulator runs
// DOS in real mode, so the server switches between *real* mode and the
// client's protected mode instead.  Clients cannot tell the difference
// except through SMSW/MOV CR0 (PE reads 0 while DOS runs):
//  - DE0Ch from real mode loads the client's CR3, GDTR, IDTR, LDTR and TR,
//    turns on PE and PG, and jumps to the client's CS:EIP with interrupts
//    disabled (EAX, ESI, DS, ES, FS, GS are undefined, as in V86).
//  - DE0Ch through the protected-mode entry point (a far CALL with the
//    VCPI stack frame) turns PE and PG off, reloads the real-mode IVT
//    (IDTR = 0:3FFh) and resumes the given CS:EIP, SS:ESP, DS, ES, FS, GS
//    with interrupts disabled, as V86 mode would be entered.
// The first megabyte is identity-mapped (EMS pages are copied in and out
// of the page frame, so its physical pages never move); 4 KB pages for
// the client come from XMS in 1 MB chunks.
#include <cstdint>
#include <cstddef>
#include <vector>

namespace fador::cpu {
class CPU;
}
namespace fador::memory {
class MemoryBus;
class HIMEM;
} // namespace fador::memory

namespace fador::hw {

class PIC8259;

class VCPI {
public:
  // Protected-mode entry: code segment base (descriptor 0 of DE01h) and
  // the offset of the "0F FF E4; RETF" stub in it.
  static constexpr uint32_t kServerBase = 0xF0000;
  static constexpr uint32_t kEntryOffset = 0x1400;
  static constexpr uint8_t kEntryTrap = 0xE4;

  VCPI(cpu::CPU &cpu, memory::MemoryBus &memory);

  void setEnabled(bool on) { m_enabled = on; }
  bool enabled() const { return m_enabled; }
  void setHIMEM(memory::HIMEM *himem) { m_himem = himem; }
  void setPICs(const PIC8259 *master, const PIC8259 *slave) {
    m_master = master;
    m_slave = slave;
  }
  // Writes the protected-mode entry stub (BIOS initialisation).
  void installEntryStub();
  void reset();

  // INT 67h AH=DEh from real mode.  Returns false when VCPI is disabled.
  bool handleInt67();
  // The protected-mode entry point (far CALL from the client).
  void handleProtectedEntry();

  size_t allocatedPages() const { return m_allocated.size(); }

private:
  void dispatch(bool fromProtected);
  void switchToProtected();
  void switchToReal();
  bool allocatePage(uint32_t &physical);
  bool freePage(uint32_t physical);
  uint32_t freePageCount() const;

  cpu::CPU &m_cpu;
  memory::MemoryBus &m_memory;
  memory::HIMEM *m_himem = nullptr;
  const PIC8259 *m_master = nullptr;
  const PIC8259 *m_slave = nullptr;
  bool m_enabled = true;
  std::vector<uint32_t> m_freePages;  // physical addresses
  std::vector<uint32_t> m_allocated;  // physical addresses
  std::vector<uint16_t> m_chunks;     // XMS handles backing the pages
  bool m_savedA20 = false;            // A20 gate while DOS runs
};

} // namespace fador::hw
