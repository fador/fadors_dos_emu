#pragma once
#include "../cpu/CPU.hpp"
#include "../memory/MemoryBus.hpp"
#include "../memory/himem/HIMEM.hpp"
#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace fador::hw {

class BIOS; // forward-declare for RM interrupt simulation
class DOS;  // forward-declare for DOS memory allocation

// Minimal DPMI 0.9 host for DOS extenders (DOS/4GW, PMODE/W, etc.)
class DPMI {
public:
  DPMI(cpu::CPU &cpu, memory::MemoryBus &memory);

  void setDOS(DOS *dos) { m_dos = dos; }
  void setBIOS(BIOS *bios) { m_bios = bios; }
  void setHIMEM(memory::HIMEM *himem) { m_himem = himem; }

  // Callback to reload segment register caches when a descriptor is modified.
  // Signature: void(uint8_t segIndex, uint16_t selector)
  using SegReloadFn = std::function<void(uint8_t, uint16_t)>;
  void setSegReloadCallback(SegReloadFn fn) { m_segReloadFn = std::move(fn); }

  bool isActive() const { return m_active; }

  // Called from INT 2Fh AX=1687h — fill registers for DPMI detection
  void handleDetect();

  // Called when the client calls the DPMI entry point (F000:0050)
  // Switches from real mode to protected mode.
  bool handleEntry();

  // Handle INT 31h DPMI function calls. Returns true if handled.
  bool handleInt31();

  // Raw mode switch: PM → RM (vector 0xE2)
  void handleRawSwitchPMtoRM();
  // Raw mode switch: RM → PM (vector 0xE3)
  void handleRawSwitchRMtoPM();

  // DPMI entry point location in BIOS ROM area
  static constexpr uint16_t DPMI_ENTRY_SEG = 0xF000;
  static constexpr uint16_t DPMI_ENTRY_OFF = 0x0050;

private:
  // GDT descriptor raw bytes (8 bytes each)
  struct RawDescriptor {
    uint32_t low = 0;
    uint32_t high = 0;
  };

  // LDT management — up to 8192 descriptors (selectors 0x0004..0xFFFC)
  static constexpr int MAX_LDT = 8192;
  static constexpr uint16_t SELECTOR_INCREMENT = 8;
  static constexpr uint16_t LDT_TI_BIT = 0x04; // TI bit set → LDT

  // Build a selector from LDT index: (index * 8) | TI | RPL
  uint16_t makeSelector(int index, uint8_t rpl = 3) const {
    return static_cast<uint16_t>((index << 3) | LDT_TI_BIT | rpl);
  }
  int selectorToIndex(uint16_t sel) const { return (sel >> 3); }

  // Allocate N contiguous LDT entries; returns base selector, 0 on failure
  uint16_t allocateDescriptors(uint16_t count);
  bool freeDescriptor(uint16_t selector);

  RawDescriptor buildDescriptor(uint32_t base, uint32_t limit, uint8_t access,
                                uint8_t flags) const;
  uint32_t extractBase(const RawDescriptor &d) const;
  void setDescBase(RawDescriptor &d, uint32_t base);
  void setDescLimit(RawDescriptor &d, uint32_t limit);
  void flushLDTEntry(int index);
  uint32_t getLinearAddr(uint8_t segReg, uint8_t reg) const;

  // DPMI memory block allocations (INT 31h AX=0501h)
  struct MemBlock {
    uint32_t linearAddr;
    uint32_t size;
    uint16_t xmsHandle; // 0 = conventional
  };
  std::vector<MemBlock> m_memBlocks;
  uint32_t m_nextBlockHandle = 1;

  // PM interrupt vectors (INT 31h AX=0204h/0205h)
  struct PMVector {
    uint16_t selector = 0;
    uint32_t offset = 0;
  };
  std::array<PMVector, 256> m_pmVectors{};

  // Processor exception handler vectors (INT 31h AX=0202h/0203h)
  std::array<PMVector, 32> m_excVectors{};

  // Virtual interrupt flag
  bool m_virtualIF = true;

  // State
  bool m_active = false;
  bool m_is32BitClient = false; // Set from AX bit 0 at DPMI entry
  std::vector<RawDescriptor> m_ldt;
  std::vector<bool> m_ldtUsed;

  // Initial selectors given to client at entry
  uint16_t m_clientCS = 0;
  uint16_t m_clientDS = 0;
  uint16_t m_clientSS = 0;
  uint16_t m_clientES = 0;
  uint16_t m_clientPSPSel = 0;

  cpu::CPU &m_cpu;
  memory::MemoryBus &m_memory;
  DOS *m_dos = nullptr;
  BIOS *m_bios = nullptr;
  memory::HIMEM *m_himem = nullptr;
  SegReloadFn m_segReloadFn;

  // Reload segment register caches if the modified selector matches any
  // currently-loaded segment register.
  void reloadAffectedSegments(uint16_t sel);

  // INT 31h sub-handlers
  void handleDescriptorMgmt();   // AX=0000h..000Ch
  void handleDOSMemory();        // AX=0100h..0102h
  void handleInterruptVectors(); // AX=0200h..0205h
  void handleTranslation();      // AX=0300h..0304h
  void handleVersion();          // AX=0400h
  void handleMemoryInfo();       // AX=0500h..0503h
  void handlePageLocking();       // AX=0600h-0604h
  void handleVirtualInterrupt(); // AX=0900h..0902h
};

} // namespace fador::hw
