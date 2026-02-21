#pragma once
#include <cstdint>
#include <array>

namespace fador::memory { class MemoryBus; }

namespace fador::cpu {

enum Reg32Index { EAX = 0, ECX = 1, EDX = 2, EBX = 3, ESP = 4, EBP = 5, ESI = 6, EDI = 7 };
enum Reg16Index { AX = 0, CX = 1, DX = 2, BX = 3, SP = 4, BP = 5, SI = 6, DI = 7 };
enum Reg8Index  { AL = 0, CL = 1, DL = 2, BL = 3, AH = 4, CH = 5, DH = 6, BH = 7 };
enum SegRegIndex { ES = 0, CS = 1, SS = 2, DS = 3, FS = 4, GS = 5 };

// EFLAGS bit masks
constexpr uint32_t FLAG_CARRY     = 1 << 0;
constexpr uint32_t FLAG_PARITY    = 1 << 2;
constexpr uint32_t FLAG_AUX       = 1 << 4;
constexpr uint32_t FLAG_ZERO      = 1 << 6;
constexpr uint32_t FLAG_SIGN      = 1 << 7;
constexpr uint32_t FLAG_TRAP      = 1 << 8;
constexpr uint32_t FLAG_INTERRUPT = 1 << 9;
constexpr uint32_t FLAG_DIRECTION = 1 << 10;
constexpr uint32_t FLAG_OVERFLOW  = 1 << 11;

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

    uint16_t getSegReg(uint8_t index) const { return m_segRegs[index % m_segRegs.size()]; }
    void setSegReg(uint8_t index, uint16_t value) { m_segRegs[index % m_segRegs.size()] = value; }

    uint32_t getEIP() const { return m_eip; }
    void setEIP(uint32_t value) { m_eip = value; }

    uint32_t getEFLAGS() const { return m_eflags; }
    void setEFLAGS(uint32_t value) { m_eflags = value; }

    // MemoryBus reference for stack ops
    void setMemoryBus(memory::MemoryBus* memory) { m_memory = memory; }

    // Stack operations
    void push16(uint16_t value);
    void push32(uint32_t value);
    uint16_t pop16();
    uint32_t pop32();

private:
    memory::MemoryBus* m_memory{nullptr};
    std::array<uint32_t, 8> m_regs{}; // EAX, ECX, EDX, EBX, ESP, EBP, ESI, EDI
    std::array<uint16_t, 6> m_segRegs{}; // ES, CS, SS, DS, FS, GS
    uint32_t m_eip{0};
    uint32_t m_eflags{0x00000002}; // bit 1 is always reserved as 1

    std::array<uint32_t, 8> m_cr{};
    std::array<uint32_t, 8> m_dr{};
};

} // namespace fador::cpu
