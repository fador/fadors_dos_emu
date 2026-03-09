#pragma once

#include "../memory/MemoryBus.hpp"
#include "CPU.hpp"
#include <cstdint>
#include <tuple>

namespace fador::hw {
class IOBus;
class BIOS;
class DOS;
} // namespace fador::hw

namespace fador::cpu {

struct ModRM {
  uint8_t mod;
  uint8_t reg;
  uint8_t rm;
};

struct SIB {
  uint8_t scale;
  uint8_t index;
  uint8_t base;
};

// Provides helper functions for fetching CPU instructions and processing
// prefixes
class InstructionDecoder {
public:
  InstructionDecoder(CPU &cpu, memory::MemoryBus &memory, hw::IOBus &iobus,
                     hw::BIOS &bios, hw::DOS &dos);

  // Main execution loop single step
  void step();

  // Inject a hardware interrupt (always pushes FLAGS/CS/IP, unlike
  // triggerInterrupt which may take the HLE fast-path).
  void injectHardwareInterrupt(uint8_t vector);

  // Calculates effective address based on ModRM and SIB (public for testing)
  uint32_t getEffectiveAddress16(const ModRM &modrm);
  uint32_t getEffectiveAddress32(const ModRM &modrm);

private:
  CPU &m_cpu;
  memory::MemoryBus &m_memory;
  hw::IOBus &m_iobus;
  hw::BIOS &m_bios;
  hw::DOS &m_dos;
  uint64_t m_stepCount;

  // Prefixes state for current instruction
  uint32_t m_instrStartEIP;
  bool m_hasPrefix66; // Operand size override
  bool m_hasPrefix67; // Address size override
  bool m_hasRepnz;
  bool m_hasRepz;
  uint8_t m_segmentOverride; // SegRegIndex or 0xFF for none

  // Caching for EA to avoid double-fetching displacement in RMW instructions
  uint32_t m_currentEA;
  uint32_t m_currentOffset; // Holds isolated offset (critical for 0x8D LEA)
  bool m_eaResolved;

  // Fetches next byte from memory through CS:EIP
  uint8_t fetch8();
  uint16_t fetch16();
  uint32_t fetch32();

  ModRM decodeModRM(uint8_t byte);
  SIB decodeSIB(uint8_t byte);

  // Helpers to resolve operand values
  uint32_t readModRM32(const ModRM &modrm);
  uint16_t readModRM16(const ModRM &modrm);
  uint8_t readModRM8(const ModRM &modrm);

  void writeModRM32(const ModRM &modrm, uint32_t value);
  void writeModRM16(const ModRM &modrm, uint16_t value);
  void writeModRM8(const ModRM &modrm, uint8_t value);

  // General ALU operation evaluator covering Math and Logic EFLAGS
  uint32_t aluOp(uint8_t op, uint32_t dest, uint32_t src, int size);

  // Checks branch/set conditions
  bool checkCondition(uint8_t cond);

  // Instruction Handlers
  void executeOpcode(uint8_t opcode);
  void executeOpcode0F(uint8_t opcode); // Two-byte opcodes

  // Interrupt and Exception handling
  void triggerInterrupt(uint8_t vector);
};

} // namespace fador::cpu