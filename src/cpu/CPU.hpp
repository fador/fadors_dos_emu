#pragma once
#include <array>
#include <cstdint>

namespace fador::memory {
class MemoryBus;
}

namespace fador::cpu {

enum Reg32Index {
  EAX = 0,
  ECX = 1,
  EDX = 2,
  EBX = 3,
  ESP = 4,
  EBP = 5,
  ESI = 6,
  EDI = 7
};
enum Reg16Index {
  AX = 0,
  CX = 1,
  DX = 2,
  BX = 3,
  SP = 4,
  BP = 5,
  SI = 6,
  DI = 7
};
enum Reg8Index {
  AL = 0,
  CL = 1,
  DL = 2,
  BL = 3,
  AH = 4,
  CH = 5,
  DH = 6,
  BH = 7
};
enum SegRegIndex { ES = 0, CS = 1, SS = 2, DS = 3, FS = 4, GS = 5 };

// EFLAGS bit masks
constexpr uint32_t FLAG_CARRY = 1 << 0;
constexpr uint32_t FLAG_PARITY = 1 << 2;
constexpr uint32_t FLAG_AUX = 1 << 4;
constexpr uint32_t FLAG_ZERO = 1 << 6;
constexpr uint32_t FLAG_SIGN = 1 << 7;
constexpr uint32_t FLAG_TRAP = 1 << 8;
constexpr uint32_t FLAG_INTERRUPT = 1 << 9;
constexpr uint32_t FLAG_DIRECTION = 1 << 10;
constexpr uint32_t FLAG_OVERFLOW = 1 << 11;

struct DescriptorRegister {
  uint16_t limit;
  uint32_t base;
};

class CPU {
public:
  CPU();
  ~CPU() = default;

  void reset();

  // Register access
  uint32_t getReg32(uint8_t index) const;
  void setReg32(uint8_t index, uint32_t value);

  uint16_t getReg16(uint8_t index) const;
  void setReg16(uint8_t index, uint16_t value);

  uint8_t getReg8(uint8_t index) const;
  void setReg8(uint8_t index, uint8_t value);

  uint16_t getSegReg(uint8_t index) const {
    return m_segRegs[index % m_segRegs.size()];
  }
  void setSegReg(uint8_t index, uint16_t value) {
    m_segRegs[index % m_segRegs.size()] = value;
  }

  uint32_t getEIP() const { return m_eip; }
  void setEIP(uint32_t value) { m_eip = value; }

  uint32_t getEFLAGS() const { return m_eflags; }
  void setEFLAGS(uint32_t value) { m_eflags = value; }

  DescriptorRegister getGDTR() const { return m_gdtr; }
  void setGDTR(const DescriptorRegister &gdtr) { m_gdtr = gdtr; }

  DescriptorRegister getIDTR() const { return m_idtr; }
  void setIDTR(const DescriptorRegister &idtr) { m_idtr = idtr; }

  uint32_t getCR(uint8_t index) const { return m_cr[index % 8]; }
  void setCR(uint8_t index, uint32_t value) { m_cr[index % 8] = value; }

  uint32_t getDR(uint8_t index) const { return m_dr[index % 8]; }
  void setDR(uint8_t index, uint32_t value) { m_dr[index % 8] = value; }

  // MemoryBus reference for stack ops
  void setMemoryBus(memory::MemoryBus *memory) { m_memory = memory; }

  uint32_t getSegBase(uint8_t index) const { return m_segBase[index % 6]; }
  void setSegBase(uint8_t index, uint32_t base) { m_segBase[index % 6] = base; }

  bool is32BitCode() const { return m_is32BitCode; }
  void setIs32BitCode(bool val) { m_is32BitCode = val; }

  bool is32BitStack() const { return m_is32BitStack; }
  void setIs32BitStack(bool val) { m_is32BitStack = val; }

  // Stack operations
  void push16(uint16_t value);
  void push32(uint32_t value);
  uint16_t pop16();
  uint32_t pop32();

  uint64_t getCycles() const { return m_cycles; }
  void addCycles(uint32_t c) { m_cycles += c; }

private:
  uint64_t m_cycles{0};
  memory::MemoryBus *m_memory{nullptr};
  std::array<uint32_t, 8> m_regs{};    // EAX, ECX, EDX, EBX, ESP, EBP, ESI, EDI
  std::array<uint16_t, 6> m_segRegs{}; // ES, CS, SS, DS, FS, GS
  std::array<uint32_t, 6> m_segBase{}; // Cached bases for PM/RM
  bool m_is32BitCode{false};           // CS D/B bit status
  bool m_is32BitStack{false};          // SS B bit status
  uint32_t m_eip{0};
  uint32_t m_eflags{0x00000002}; // bit 1 is always reserved as 1

  std::array<uint32_t, 8> m_cr{};
  std::array<uint32_t, 8> m_dr{};

  DescriptorRegister m_gdtr{0xFFFF, 0};
  DescriptorRegister m_idtr{0xFFFF, 0};
};

} // namespace fador::cpu
