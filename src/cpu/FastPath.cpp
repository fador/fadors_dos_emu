// Predecoded fast path of the interpreter.
//
// The hottest integer instructions (MOV, ALU, INC/DEC, PUSH/POP, Jcc,
// CALL/RET, LEA, shifts, MUL/IMUL, MOVZX/MOVSX, SETcc, CMOVcc, ...) are
// decoded once into a DecodedInsn (prefixes, ModRM/SIB, displacement and
// immediates resolved) and executed by a small handler on every later
// visit.  InstructionDecoder::tryFastPath() looks the instruction up by
// linear address, checks that the bytes in memory are still the decoded
// ones and calls the handler; anything else (string instructions, segment
// loads, far transfers, interrupts, I/O, system instructions, x87, page- or
// IP-wrapping instructions, code outside plain RAM) runs through the legacy
// decoder in InstructionDecoder.cpp.
//
// Every handler must have exactly the architectural effect of the legacy
// code for the same opcode: the same register, flag and memory results,
// the same memory accesses in the same order (they matter for the VGA
// window and for #PF restart), and the same guest cycle charge
// (DecodedInsn::cycles = the opcode's cycle-table entry, plus the memory
// operand cost when the legacy code computes an effective address, plus
// the MUL surcharge for F6/F7 /4 and /5).  tools/cpu_fuzz and the
// "fast path" tests in tests/test_interpreter_fast_paths.cpp run random
// instruction streams through both paths and compare the full state.
#include "InstructionDecoder.hpp"
#include "../hw/DOS.hpp"

#include <cstring>

namespace fador::cpu {

void InstructionDecoder::flushDecodedCache() {
  for (uint32_t i = 0; i < kDecodedSets * 2; ++i)
    m_decoded[i] = DecodedInsn{};
}

struct FastOps {
  using ID = InstructionDecoder;
  using D = DecodedInsn;

  // ── operand helpers ───────────────────────────────────────────────
  static uint32_t offset(ID &c, const D &d) {
    const uint32_t b = c.m_cpu.getReg32(d.base) & d.baseMask;
    const uint32_t i = (c.m_cpu.getReg32(d.index) & d.indexMask) << d.scale;
    return (d.disp + b + i) & d.addrMask;
  }
  static uint32_t ea(ID &c, const D &d) { return c.m_segBase[d.seg] + offset(c, d); }

  template <int S> static uint32_t getR(ID &c, uint8_t r) {
    if constexpr (S == 8)
      return c.m_cpu.getReg8(r);
    else if constexpr (S == 16)
      return c.m_cpu.getReg16(r);
    else
      return c.m_cpu.getReg32(r);
  }
  template <int S> static void setR(ID &c, uint8_t r, uint32_t v) {
    if constexpr (S == 8)
      c.m_cpu.setReg8(r, static_cast<uint8_t>(v));
    else if constexpr (S == 16)
      c.m_cpu.setReg16(r, static_cast<uint16_t>(v));
    else
      c.m_cpu.setReg32(r, v);
  }
  template <int S> static uint32_t rd(ID &c, uint32_t a) {
    if constexpr (S == 8)
      return c.m_memory.lread8(a);
    else if constexpr (S == 16)
      return c.m_memory.lread16(a);
    else
      return c.m_memory.lread32(a);
  }
  template <int S> static void wr(ID &c, uint32_t a, uint32_t v) {
    if constexpr (S == 8)
      c.m_memory.lwrite8(a, static_cast<uint8_t>(v));
    else if constexpr (S == 16)
      c.m_memory.lwrite16(a, static_cast<uint16_t>(v));
    else
      c.m_memory.lwrite32(a, v);
  }
  template <int S> static void push(ID &c, uint32_t v) {
    if constexpr (S == 32)
      c.m_cpu.push32(v);
    else
      c.m_cpu.push16(static_cast<uint16_t>(v));
  }
  template <int S> static uint32_t pop(ID &c) {
    if constexpr (S == 32)
      return c.m_cpu.pop32();
    else
      return c.m_cpu.pop16();
  }

  // ── ALU ───────────────────────────────────────────────────────────
  // op r/m, reg (RegDest = false) or op reg, r/m (RegDest = true)
  template <int Op, int S, bool Mem, bool RegDest>
  static void aluModRM(ID &c, const D &d) {
    if constexpr (RegDest) {
      const uint32_t src = Mem ? rd<S>(c, ea(c, d)) : getR<S>(c, d.rm);
      const uint32_t r = c.aluOp(Op, getR<S>(c, d.reg), src, S);
      if constexpr (Op != 7)
        setR<S>(c, d.reg, r);
    } else if constexpr (Mem) {
      const uint32_t a = ea(c, d);
      const uint32_t r = c.aluOp(Op, rd<S>(c, a), getR<S>(c, d.reg), S);
      if constexpr (Op != 7)
        wr<S>(c, a, r);
    } else {
      const uint32_t r = c.aluOp(Op, getR<S>(c, d.rm), getR<S>(c, d.reg), S);
      if constexpr (Op != 7)
        setR<S>(c, d.rm, r);
    }
  }
  // op AL/AX/EAX, imm
  template <int Op, int S> static void aluAcc(ID &c, const D &d) {
    const uint32_t r = c.aluOp(Op, getR<S>(c, 0), d.imm, S);
    if constexpr (Op != 7)
      setR<S>(c, 0, r);
  }
  // group 1: op r/m, imm (80-83); Op is ModRM.reg
  template <int Op, int S, bool Mem> static void aluImm(ID &c, const D &d) {
    if constexpr (Mem) {
      const uint32_t a = ea(c, d);
      const uint32_t r = c.aluOp(Op, rd<S>(c, a), d.imm, S);
      if constexpr (Op != 7)
        wr<S>(c, a, r);
    } else {
      const uint32_t r = c.aluOp(Op, getR<S>(c, d.rm), d.imm, S);
      if constexpr (Op != 7)
        setR<S>(c, d.rm, r);
    }
  }
  template <int S> static void testAcc(ID &c, const D &d) { // A8/A9
    c.aluOp(4, getR<S>(c, 0), d.imm, S);
  }
  template <int S, bool Mem> static void testRM(ID &c, const D &d) {
    const uint32_t v = Mem ? rd<S>(c, ea(c, d)) : getR<S>(c, d.rm);
    c.aluOp(4, v, getR<S>(c, d.reg), S);
  }
  template <bool Inc, int S> static void incDecReg(ID &c, const D &d) {
    const uint32_t v = getR<S>(c, d.reg);
    setR<S>(c, d.reg, Inc ? c.incOp(v, S) : c.decOp(v, S));
  }
  // FE/FF /0 /1
  template <bool Inc, int S, bool Mem> static void incDecRM(ID &c, const D &d) {
    if constexpr (Mem) {
      const uint32_t a = ea(c, d);
      const uint32_t v = rd<S>(c, a);
      wr<S>(c, a, Inc ? c.incOp(v, S) : c.decOp(v, S));
    } else {
      const uint32_t v = getR<S>(c, d.rm);
      setR<S>(c, d.rm, Inc ? c.incOp(v, S) : c.decOp(v, S));
    }
  }

  // ── data movement ─────────────────────────────────────────────────
  template <int S, bool Mem> static void movRMReg(ID &c, const D &d) { // 88/89
    const uint32_t v = getR<S>(c, d.reg);
    if constexpr (Mem)
      wr<S>(c, ea(c, d), v);
    else
      setR<S>(c, d.rm, v);
  }
  template <int S, bool Mem> static void movRegRM(ID &c, const D &d) { // 8A/8B
    setR<S>(c, d.reg, Mem ? rd<S>(c, ea(c, d)) : getR<S>(c, d.rm));
  }
  template <int S, bool Mem> static void movRMImm(ID &c, const D &d) { // C6/C7
    if constexpr (Mem)
      wr<S>(c, ea(c, d), d.imm);
    else
      setR<S>(c, d.rm, d.imm);
  }
  template <int S> static void movRegImm(ID &c, const D &d) { // B0-BF
    setR<S>(c, d.reg, d.imm);
  }
  // A0-A3: d.aux bit 1 = store
  template <int S, bool Store> static void movMoffs(ID &c, const D &d) {
    const uint32_t a = c.m_segBase[d.seg] + d.disp;
    if constexpr (Store)
      wr<S>(c, a, getR<S>(c, 0));
    else
      setR<S>(c, 0, rd<S>(c, a));
  }
  template <int S> static void lea(ID &c, const D &d) {
    setR<S>(c, d.reg, offset(c, d));
  }
  template <int S, bool Mem> static void xchgRM(ID &c, const D &d) { // 86/87
    if constexpr (Mem) {
      const uint32_t a = ea(c, d);
      const uint32_t v = rd<S>(c, a);
      wr<S>(c, a, getR<S>(c, d.reg));
      setR<S>(c, d.reg, v);
    } else {
      const uint32_t v = getR<S>(c, d.rm);
      setR<S>(c, d.rm, getR<S>(c, d.reg));
      setR<S>(c, d.reg, v);
    }
  }
  template <int S> static void xchgAcc(ID &c, const D &d) { // 91-97
    const uint32_t v = getR<S>(c, 0);
    setR<S>(c, 0, getR<S>(c, d.reg));
    setR<S>(c, d.reg, v);
  }
  static void nop(ID &, const D &) {}
  template <int S> static void cbw(ID &c, const D &) {
    if constexpr (S == 32)
      c.m_cpu.setReg32(EAX, static_cast<uint32_t>(
                                static_cast<int32_t>(static_cast<int16_t>(c.m_cpu.getReg16(AX)))));
    else
      c.m_cpu.setReg16(AX, static_cast<uint16_t>(
                               static_cast<int16_t>(static_cast<int8_t>(c.m_cpu.getReg8(AL)))));
  }
  template <int S> static void cwd(ID &c, const D &) {
    if constexpr (S == 32)
      c.m_cpu.setReg32(EDX, static_cast<int32_t>(c.m_cpu.getReg32(EAX)) < 0 ? 0xFFFFFFFFu : 0u);
    else
      c.m_cpu.setReg16(DX, static_cast<int16_t>(c.m_cpu.getReg16(AX)) < 0 ? 0xFFFF : 0);
  }
  template <int S, bool Mem, bool Sign, int SrcS>
  static void movExt(ID &c, const D &d) { // 0F B6/B7/BE/BF
    uint32_t v = Mem ? rd<SrcS>(c, ea(c, d)) : getR<SrcS>(c, d.rm);
    if constexpr (Sign) {
      if constexpr (SrcS == 8)
        v = static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(v)));
      else
        v = static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(v)));
    }
    setR<S>(c, d.reg, v);
  }

  // ── segment registers (never CS or SS: those end a burst or start an
  //    interrupt shadow and stay on the legacy path) ─────────────────
  template <bool Mem> static void movSregRM(ID &c, const D &d) { // 8E
    const uint16_t v = static_cast<uint16_t>(Mem ? rd<16>(c, ea(c, d)) : getR<16>(c, d.rm));
    c.loadSegment(static_cast<SegRegIndex>(d.reg), v);
  }
  template <bool Mem> static void movRMSreg(ID &c, const D &d) { // 8C
    const uint16_t v = c.m_cpu.getSegReg(d.reg);
    if constexpr (Mem)
      wr<16>(c, ea(c, d), v);
    else
      setR<16>(c, d.rm, v);
  }
  template <int S> static void pushSreg(ID &c, const D &d) {
    push<S>(c, c.m_cpu.getSegReg(d.reg));
  }
  template <int S> static void popSreg(ID &c, const D &d) {
    const uint16_t v = static_cast<uint16_t>(pop<S>(c) & 0xFFFF);
    c.loadSegment(static_cast<SegRegIndex>(d.reg), v);
  }
  // LES/LDS (C4/C5): d.aux = segment register
  template <int S> static void loadFarPointer(ID &c, const D &d) {
    const uint32_t a = ea(c, d);
    setR<S>(c, d.reg, rd<S>(c, a));
    c.loadSegment(static_cast<SegRegIndex>(d.aux),
                  static_cast<uint16_t>(rd<16>(c, a + S / 8)));
  }

  // ── stack ─────────────────────────────────────────────────────────
  template <int S> static void pushReg(ID &c, const D &d) { push<S>(c, getR<S>(c, d.reg)); }
  template <int S> static void popReg(ID &c, const D &d) { setR<S>(c, d.reg, pop<S>(c)); }
  template <int S> static void pushImm(ID &c, const D &d) { push<S>(c, d.imm); }
  template <int S, bool Mem> static void pushRM(ID &c, const D &d) { // FF /6
    push<S>(c, Mem ? rd<S>(c, ea(c, d)) : getR<S>(c, d.rm));
  }
  template <int S> static void leave(ID &c, const D &) {
    if constexpr (S == 32) {
      c.m_cpu.setReg32(ESP, c.m_cpu.getReg32(EBP));
      c.m_cpu.setReg32(EBP, c.m_cpu.pop32());
    } else {
      c.m_cpu.setReg16(SP, c.m_cpu.getReg16(BP));
      c.m_cpu.setReg16(BP, c.m_cpu.pop16());
    }
  }

  // ── control transfer ──────────────────────────────────────────────
  static void jccShort(ID &c, const D &d) { // 70-7F: d.disp sign-extended
    if (c.m_cpu.testCondition(d.aux))
      c.m_cpu.setEIP(c.m_cpu.getEIP() + d.disp);
  }
  template <int S> static void jccNear(ID &c, const D &d) { // 0F 80-8F
    if (c.m_cpu.testCondition(d.aux)) {
      if constexpr (S == 32)
        c.m_cpu.setEIP(c.m_cpu.getEIP() + d.disp);
      else
        c.m_cpu.setEIP(static_cast<uint16_t>(c.m_cpu.getEIP() + d.disp));
    }
  }
  static void jmpShort(ID &c, const D &d) { c.m_cpu.setEIP(c.m_cpu.getEIP() + d.disp); }
  template <int S> static void jmpNear(ID &c, const D &d) { // E9
    if constexpr (S == 32)
      c.m_cpu.setEIP(c.m_cpu.getEIP() + d.disp);
    else
      c.m_cpu.setEIP(static_cast<uint16_t>(c.m_cpu.getEIP() + d.disp));
  }
  template <int S> static void callNear(ID &c, const D &d) { // E8
    if constexpr (S == 32) {
      const uint32_t ret = c.m_cpu.getEIP();
      c.m_cpu.push32(ret);
      c.m_cpu.setEIP(c.m_cpu.getEIP() + d.disp);
    } else {
      const uint16_t ret = static_cast<uint16_t>(c.m_cpu.getEIP());
      const uint16_t target = static_cast<uint16_t>(ret + d.disp);
      c.m_cpu.push16(ret);
      c.m_cpu.setEIP(target);
    }
  }
  template <int S> static void retNear(ID &c, const D &) { // C3
    c.m_cpu.setEIP(pop<S>(c));
  }
  template <int S> static void retNearImm(ID &c, const D &d) { // C2
    const uint32_t eip = pop<S>(c);
    if (c.m_cpu.is32BitStack())
      c.m_cpu.setReg32(ESP, c.m_cpu.getReg32(ESP) + d.imm);
    else
      c.m_cpu.setReg16(SP, static_cast<uint16_t>(c.m_cpu.getReg16(SP) + d.imm));
    c.m_cpu.setEIP(eip);
  }
  template <int S, bool Mem> static void callRM(ID &c, const D &d) { // FF /2
    const uint32_t target = Mem ? rd<S>(c, ea(c, d)) : getR<S>(c, d.rm);
    push<S>(c, c.m_cpu.getEIP());
    c.m_cpu.setEIP(target);
  }
  template <int S, bool Mem> static void jmpRM(ID &c, const D &d) { // FF /4
    c.m_cpu.setEIP(Mem ? rd<S>(c, ea(c, d)) : getR<S>(c, d.rm));
  }
  template <bool Mem> static void setcc(ID &c, const D &d) { // 0F 90-9F
    const uint32_t v = c.m_cpu.testCondition(d.aux) ? 1 : 0;
    if constexpr (Mem)
      wr<8>(c, ea(c, d), v);
    else
      setR<8>(c, d.rm, v);
  }
  template <int S, bool Mem> static void cmovcc(ID &c, const D &d) { // 0F 40-4F
    const bool take = c.m_cpu.testCondition(d.aux);
    const uint32_t v = Mem ? rd<S>(c, ea(c, d)) : getR<S>(c, d.rm);
    if (take)
      setR<S>(c, d.reg, v);
  }

  // ── x87 (D8-DF) ───────────────────────────────────────────────────
  // d.op = opcode, d.aux = ModRM byte, d.imm = the memory-operand cycles
  // the legacy decoder charges (none for its inline [reg+disp32] form).
  // Entries carry DecodedInsn::kX87: they only run with no x87 error
  // pending, so the waiting check (#MF / FERR#) never fires here.
  template <bool Op32, bool Mem> static void x87(ID &c, const D &d) {
    c.m_hasPrefix66 = Op32; // operand size of FSTENV/FLDENV/FSAVE/FRSTOR
    uint32_t addr = 0;
    if constexpr (Mem) {
      const uint32_t off = offset(c, d);
      c.m_currentOffset = off;
      c.m_currentEASegment = d.seg;
      c.m_currentEA = addr = c.m_segBase[d.seg] + off;
      c.m_eaResolved = true;
      c.m_cpu.addGuestCycles(d.imm);
    }
    c.executeFPUOperands(d.op, d.aux, Mem, addr);
  }

  // ── flags ─────────────────────────────────────────────────────────
  static void cmc(ID &c, const D &) {
    c.m_cpu.setArithFlags(FLAG_CARRY, c.m_cpu.carryFlag() ? 0u : FLAG_CARRY);
  }
  static void clc(ID &c, const D &) { c.m_cpu.setArithFlags(FLAG_CARRY, 0); }
  static void stc(ID &c, const D &) { c.m_cpu.setArithFlags(FLAG_CARRY, FLAG_CARRY); }
  static void cld(ID &c, const D &) { c.m_cpu.setEFLAGS(c.m_cpu.getEFLAGS() & ~0x0400u); }
  static void std_(ID &c, const D &) { c.m_cpu.setEFLAGS(c.m_cpu.getEFLAGS() | 0x0400u); }

  // ── shifts, group 3, multiply ─────────────────────────────────────
  // Count: 0 = imm8 (d.imm), 1 = one, 2 = CL
  template <int S, bool Mem, int Count> static void shift(ID &c, const D &d) {
    const uint8_t count = Count == 0 ? static_cast<uint8_t>(d.imm)
                          : Count == 1 ? 1
                                       : c.m_cpu.getReg8(CL);
    if constexpr (Mem) {
      const uint32_t a = ea(c, d);
      wr<S>(c, a, c.shiftRotate(d.reg, rd<S>(c, a), count, S));
    } else {
      setR<S>(c, d.rm, c.shiftRotate(d.reg, getR<S>(c, d.rm), count, S));
    }
  }
  // F6/F7 /0../5 (DIV and IDIV stay on the legacy path)
  template <int S, bool Mem> static void group3(ID &c, const D &d) {
    const uint32_t a = Mem ? ea(c, d) : 0;
    const uint32_t v = Mem ? rd<S>(c, a) : getR<S>(c, d.rm);
    auto put = [&](uint32_t r) {
      if constexpr (Mem)
        wr<S>(c, a, r);
      else
        setR<S>(c, d.rm, r);
    };
    switch (d.reg) {
    case 0:
    case 1:
      c.aluOp(4, v, d.imm, S);
      break;
    case 2:
      put(~v);
      break;
    case 3:
      put(c.aluOp(5, 0, v, S));
      break;
    case 4:
      if constexpr (S == 8) {
        const uint16_t res = static_cast<uint16_t>(c.m_cpu.getReg8(AL) * v);
        c.m_cpu.setReg16(AX, res);
        c.setMulOverflow((res >> 8) != 0);
      } else if constexpr (S == 16) {
        const uint32_t res = static_cast<uint32_t>(c.m_cpu.getReg16(AX)) * v;
        c.m_cpu.setReg16(AX, static_cast<uint16_t>(res));
        c.m_cpu.setReg16(DX, static_cast<uint16_t>(res >> 16));
        c.setMulOverflow((res >> 16) != 0);
      } else {
        const uint64_t res = static_cast<uint64_t>(c.m_cpu.getReg32(EAX)) * v;
        c.m_cpu.setReg32(EAX, static_cast<uint32_t>(res));
        c.m_cpu.setReg32(EDX, static_cast<uint32_t>(res >> 32));
        c.setMulOverflow((res >> 32) != 0);
      }
      break;
    default: // 5: IMUL
      if constexpr (S == 8) {
        const int16_t res = static_cast<int16_t>(
            static_cast<int16_t>(static_cast<int8_t>(c.m_cpu.getReg8(AL))) *
            static_cast<int8_t>(v));
        c.m_cpu.setReg16(AX, static_cast<uint16_t>(res));
        c.setMulOverflow(res != static_cast<int8_t>(res));
      } else if constexpr (S == 16) {
        const int32_t res =
            static_cast<int32_t>(static_cast<int16_t>(c.m_cpu.getReg16(AX))) *
            static_cast<int16_t>(v);
        c.m_cpu.setReg16(AX, static_cast<uint16_t>(res));
        c.m_cpu.setReg16(DX, static_cast<uint16_t>(res >> 16));
        c.setMulOverflow(res != static_cast<int16_t>(res));
      } else {
        const int64_t res =
            static_cast<int64_t>(static_cast<int32_t>(c.m_cpu.getReg32(EAX))) *
            static_cast<int32_t>(v);
        c.m_cpu.setReg32(EAX, static_cast<uint32_t>(res));
        c.m_cpu.setReg32(EDX, static_cast<uint32_t>(res >> 32));
        c.setMulOverflow(res != static_cast<int32_t>(res));
      }
      break;
    }
  }
  // IMUL reg, r/m, imm (69/6B; d.imm already sign-extended) and
  // IMUL reg, r/m (0F AF; Imm = false)
  template <int S, bool Mem, bool Imm> static void imulReg(ID &c, const D &d) {
    const uint32_t v = Mem ? rd<S>(c, ea(c, d)) : getR<S>(c, d.rm);
    const uint32_t other = Imm ? d.imm : getR<S>(c, d.reg);
    if constexpr (S == 32) {
      const int64_t res = static_cast<int64_t>(static_cast<int32_t>(v)) *
                          static_cast<int32_t>(other);
      c.m_cpu.setReg32(d.reg, static_cast<uint32_t>(res));
      c.setMulOverflow(res != static_cast<int32_t>(res));
    } else {
      const int32_t res = static_cast<int32_t>(static_cast<int16_t>(v)) *
                          static_cast<int16_t>(other);
      c.m_cpu.setReg16(d.reg, static_cast<uint16_t>(res));
      c.setMulOverflow(res != static_cast<int16_t>(res));
    }
  }
};

namespace {

using Handler = DecodedInsn::Handler;
using F = FastOps;

// Handler selection: H::h<S, M> is the handler for operand size S (8, 16,
// 32) and a memory (M = true) or register r/m operand.
template <class H> Handler pickSM(int size, bool mem) {
  switch (size) {
  case 8:
    return mem ? H::template h<8, true> : H::template h<8, false>;
  case 16:
    return mem ? H::template h<16, true> : H::template h<16, false>;
  default:
    return mem ? H::template h<32, true> : H::template h<32, false>;
  }
}
template <template <int> class H> Handler pickOpSM(int op, int size, bool mem) {
  switch (op & 7) {
  case 0: return pickSM<H<0>>(size, mem);
  case 1: return pickSM<H<1>>(size, mem);
  case 2: return pickSM<H<2>>(size, mem);
  case 3: return pickSM<H<3>>(size, mem);
  case 4: return pickSM<H<4>>(size, mem);
  case 5: return pickSM<H<5>>(size, mem);
  case 6: return pickSM<H<6>>(size, mem);
  default: return pickSM<H<7>>(size, mem);
  }
}
// Size-only families: H::h<S, false> is used.
template <class H> Handler pickS(int size) { return pickSM<H>(size, false); }

template <int Op, bool RegDest> struct HAlu {
  template <int S, bool M> static constexpr Handler h = &F::aluModRM<Op, S, M, RegDest>;
};
template <int Op> using HAluRM = HAlu<Op, false>;
template <int Op> using HAluReg = HAlu<Op, true>;
template <int Op> struct HAluImm {
  template <int S, bool M> static constexpr Handler h = &F::aluImm<Op, S, M>;
};
template <int Op> struct HAluAcc {
  template <int S, bool M> static constexpr Handler h = &F::aluAcc<Op, S>;
};
#define FADOR_FAMILY(Name, expr)                                               \
  struct Name {                                                                \
    template <int S, bool M> static constexpr Handler h = expr;                \
  };
FADOR_FAMILY(HTest, (&F::testRM<S, M>))
FADOR_FAMILY(HTestAcc, (&F::testAcc<S>))
FADOR_FAMILY(HMovRMReg, (&F::movRMReg<S, M>))
FADOR_FAMILY(HMovRegRM, (&F::movRegRM<S, M>))
FADOR_FAMILY(HMovRMImm, (&F::movRMImm<S, M>))
FADOR_FAMILY(HMovRegImm, (&F::movRegImm<S>))
FADOR_FAMILY(HXchg, (&F::xchgRM<S, M>))
FADOR_FAMILY(HXchgAcc, (&F::xchgAcc<S>))
FADOR_FAMILY(HInc, (&F::incDecRM<true, S, M>))
FADOR_FAMILY(HDec, (&F::incDecRM<false, S, M>))
FADOR_FAMILY(HIncReg, (&F::incDecReg<true, S>))
FADOR_FAMILY(HDecReg, (&F::incDecReg<false, S>))
FADOR_FAMILY(HShiftImm, (&F::shift<S, M, 0>))
FADOR_FAMILY(HShiftOne, (&F::shift<S, M, 1>))
FADOR_FAMILY(HShiftCL, (&F::shift<S, M, 2>))
FADOR_FAMILY(HGroup3, (&F::group3<S, M>))
FADOR_FAMILY(HPushReg, (&F::pushReg<S>))
FADOR_FAMILY(HPopReg, (&F::popReg<S>))
FADOR_FAMILY(HPushImm, (&F::pushImm<S>))
FADOR_FAMILY(HPushRM, (&F::pushRM<S, M>))
FADOR_FAMILY(HCallRM, (&F::callRM<S, M>))
FADOR_FAMILY(HJmpRM, (&F::jmpRM<S, M>))
FADOR_FAMILY(HCmov, (&F::cmovcc<S, M>))
FADOR_FAMILY(HImulImm, (&F::imulReg<S, M, true>))
FADOR_FAMILY(HImul, (&F::imulReg<S, M, false>))
FADOR_FAMILY(HMovzx8, (&F::movExt<S, M, false, 8>))
FADOR_FAMILY(HMovzx16, (&F::movExt<S, M, false, 16>))
FADOR_FAMILY(HMovsx8, (&F::movExt<S, M, true, 8>))
FADOR_FAMILY(HMovsx16, (&F::movExt<S, M, true, 16>))
FADOR_FAMILY(HSetcc, (&F::setcc<M>))
FADOR_FAMILY(HLea, (&F::lea<S>))
FADOR_FAMILY(HCbw, (&F::cbw<S>))
FADOR_FAMILY(HCwd, (&F::cwd<S>))
FADOR_FAMILY(HLeave, (&F::leave<S>))
FADOR_FAMILY(HJccNear, (&F::jccNear<S>))
FADOR_FAMILY(HJmpNear, (&F::jmpNear<S>))
FADOR_FAMILY(HCallNear, (&F::callNear<S>))
FADOR_FAMILY(HRet, (&F::retNear<S>))
FADOR_FAMILY(HRetImm, (&F::retNearImm<S>))
FADOR_FAMILY(HMoffsLoad, (&F::movMoffs<S, false>))
FADOR_FAMILY(HMoffsStore, (&F::movMoffs<S, true>))
FADOR_FAMILY(HMovSregRM, (&F::movSregRM<M>))
FADOR_FAMILY(HMovRMSreg, (&F::movRMSreg<M>))
FADOR_FAMILY(HPushSreg, (&F::pushSreg<S>))
FADOR_FAMILY(HPopSreg, (&F::popSreg<S>))
FADOR_FAMILY(HLoadFar, (&F::loadFarPointer<S>))
// x87: S = operand size (16/32) of the environment/state images
FADOR_FAMILY(HX87, (S == 32 ? &F::x87<true, M> : &F::x87<false, M>))
#undef FADOR_FAMILY

uint32_t le(const uint8_t *p, int n) {
  uint32_t v = 0;
  for (int i = 0; i < n; ++i)
    v |= static_cast<uint32_t>(p[i]) << (8 * i);
  return v;
}

} // namespace

void InstructionDecoder::decodeFast(DecodedInsn &d, uint32_t linear,
                                    const uint8_t *code, bool code32) {
  d = DecodedInsn{};
  d.linear = linear;
  d.code32 = code32 ? 1 : 0;
  // d.handler stays null (legacy decoder) unless a fast form is found.

  constexpr unsigned kWindow = 16;
  bool op32 = code32, addr32 = code32;
  uint8_t segOverride = 0xFF;
  unsigned pos = 0;
  for (;; ++pos) {
    if (pos >= kWindow)
      return;
    const uint8_t b = code[pos];
    if (b == 0x66)
      op32 = !op32; // the legacy decoder toggles: two prefixes cancel
    else if (b == 0x67)
      addr32 = !addr32;
    else if (b == 0xF0)
      continue; // LOCK
    else if (b == 0xF2 || b == 0xF3)
      return; // REP forms: legacy
    else if (b == 0x26)
      segOverride = ES;
    else if (b == 0x2E)
      segOverride = CS;
    else if (b == 0x36)
      segOverride = SS;
    else if (b == 0x3E)
      segOverride = DS;
    else if (b == 0x64)
      segOverride = FS;
    else if (b == 0x65)
      segOverride = GS;
    else
      break;
  }
  uint8_t op = code[pos++];
  bool twoByte = false;
  if (op == 0x0F) {
    if (pos >= kWindow)
      return;
    op = code[pos++];
    twoByte = true;
  }
  const int osize = op32 ? 32 : 16;
  uint32_t cycles = m_cycleTable[twoByte ? 256u + op : op];

  // ModRM / SIB / displacement.
  bool mem = false;
  auto modrm = [&]() -> bool {
    if (pos >= kWindow)
      return false;
    const uint8_t m = code[pos++];
    const uint8_t mod = m >> 6;
    d.reg = (m >> 3) & 7;
    d.rm = m & 7;
    if (mod == 3)
      return true;
    mem = true;
    cycles += m_cpu.memOperandCycles();
    uint8_t defSeg = DS;
    if (!addr32) {
      static constexpr uint8_t kBase[8] = {BX, BX, BP, BP, SI, DI, BP, BX};
      static constexpr uint8_t kIndex[8] = {SI, DI, SI, DI, 0, 0, 0, 0};
      d.addrMask = 0xFFFF;
      if (mod == 0 && d.rm == 6) {
        if (pos + 2 > kWindow)
          return false;
        d.disp = le(code + pos, 2);
        pos += 2;
      } else {
        d.base = kBase[d.rm];
        d.baseMask = 0xFFFFFFFFu;
        d.index = kIndex[d.rm];
        d.indexMask = d.rm < 4 ? 0xFFFFFFFFu : 0u;
        if (d.rm == 2 || d.rm == 3 || d.rm == 6)
          defSeg = SS;
        if (mod == 1) {
          if (pos + 1 > kWindow)
            return false;
          d.disp = static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(code[pos++])));
        } else if (mod == 2) {
          if (pos + 2 > kWindow)
            return false;
          d.disp = le(code + pos, 2);
          pos += 2;
        }
      }
    } else {
      d.addrMask = 0xFFFFFFFFu;
      bool hasBase = true;
      if (d.rm == 4) {
        if (pos >= kWindow)
          return false;
        const uint8_t sib = code[pos++];
        d.scale = sib >> 6;
        d.index = (sib >> 3) & 7;
        d.indexMask = d.index == 4 ? 0u : 0xFFFFFFFFu;
        d.base = sib & 7;
        if (d.base == 5 && mod == 0) {
          hasBase = false;
          if (pos + 4 > kWindow)
            return false;
          d.disp = le(code + pos, 4);
          pos += 4;
        } else if (d.base == ESP || d.base == EBP) {
          defSeg = SS;
        }
      } else if (mod == 0 && d.rm == 5) {
        hasBase = false;
        if (pos + 4 > kWindow)
          return false;
        d.disp = le(code + pos, 4);
        pos += 4;
      } else {
        d.base = d.rm;
        if (d.rm == EBP)
          defSeg = SS;
      }
      d.baseMask = hasBase ? 0xFFFFFFFFu : 0u;
      if (mod == 1) {
        if (pos + 1 > kWindow)
          return false;
        d.disp += static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(code[pos++])));
      } else if (mod == 2) {
        if (pos + 4 > kWindow)
          return false;
        d.disp += le(code + pos, 4);
        pos += 4;
      }
    }
    d.seg = segOverride != 0xFF ? segOverride : defSeg;
    return true;
  };
  auto imm = [&](int bytes, bool signExtend) -> bool {
    if (pos + static_cast<unsigned>(bytes) > kWindow)
      return false;
    uint32_t v = le(code + pos, bytes);
    pos += static_cast<unsigned>(bytes);
    if (signExtend && bytes == 1)
      v = static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(v)));
    else if (signExtend && bytes == 2)
      v = static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(v)));
    d.imm = v;
    return true;
  };
  const int immBytes = op32 ? 4 : 2;

  Handler h = nullptr;
  if (!twoByte) {
    if (op < 0x40 && (op & 7) < 6) {
      const int aop = op >> 3;
      const int size = (op & 1) ? osize : 8;
      if ((op & 7) < 4) { // ALU r/m forms
        if (!modrm())
          return;
        h = (op & 2) ? pickOpSM<HAluReg>(aop, size, mem)
                     : pickOpSM<HAluRM>(aop, size, mem);
      } else { // ALU accumulator, imm
        if (!imm(size == 8 ? 1 : immBytes, false))
          return;
        h = pickOpSM<HAluAcc>(aop, size, false);
      }
    } else if (op < 0x20 && (op & 7) >= 6 && op != 0x0F && op != 0x17) {
      // PUSH ES/CS/SS/DS (06 0E 16 1E), POP ES/DS (07 1F); POP SS (17)
      // starts an interrupt shadow: legacy.
      d.reg = op >> 3;
      h = (op & 1) ? pickS<HPopSreg>(osize) : pickS<HPushSreg>(osize);
    } else if (op >= 0x40 && op <= 0x4F) {
      d.reg = op & 7;
      h = op < 0x48 ? pickS<HIncReg>(osize) : pickS<HDecReg>(osize);
    } else if (op >= 0x50 && op <= 0x57) {
      d.reg = op & 7;
      h = pickS<HPushReg>(osize);
    } else if (op >= 0x58 && op <= 0x5F) {
      d.reg = op & 7;
      h = pickS<HPopReg>(osize);
    } else if (op == 0x68 || op == 0x6A) {
      if (!imm(op == 0x6A ? 1 : immBytes, op == 0x6A))
        return;
      h = pickS<HPushImm>(osize);
    } else if (op == 0x69 || op == 0x6B) {
      if (!modrm() || !imm(op == 0x6B ? 1 : immBytes, true))
        return;
      h = pickSM<HImulImm>(osize, mem);
    } else if (op >= 0x70 && op <= 0x7F) {
      if (!imm(1, true))
        return;
      d.disp = d.imm;
      d.aux = op & 0x0F;
      h = &F::jccShort;
    } else if (op >= 0x80 && op <= 0x83) {
      const int size = (op == 0x80 || op == 0x82) ? 8 : osize;
      if (!modrm())
        return;
      if (op == 0x81) {
        if (!imm(immBytes, false))
          return;
      } else if (!imm(1, op == 0x83)) {
        return;
      }
      h = pickOpSM<HAluImm>(d.reg, size, mem);
    } else if (op == 0x84 || op == 0x85) {
      if (!modrm())
        return;
      h = pickSM<HTest>(op == 0x84 ? 8 : osize, mem);
    } else if (op == 0x86 || op == 0x87) {
      if (!modrm())
        return;
      h = pickSM<HXchg>(op == 0x86 ? 8 : osize, mem);
    } else if (op >= 0x88 && op <= 0x8B) {
      if (!modrm())
        return;
      const int size = (op & 1) ? osize : 8;
      h = (op & 2) ? pickSM<HMovRegRM>(size, mem) : pickSM<HMovRMReg>(size, mem);
    } else if (op == 0x8C || op == 0x8E) {
      if (!modrm() || d.reg > 5)
        return; // #UD
      if (op == 0x8E && (d.reg == CS || d.reg == SS))
        return; // CS changes the code segment, SS starts a shadow
      h = op == 0x8C ? pickSM<HMovRMSreg>(16, mem) : pickSM<HMovSregRM>(16, mem);
    } else if (op == 0xC4 || op == 0xC5) {
      if (!modrm() || !mem)
        return;
      d.aux = op == 0xC4 ? ES : DS;
      h = pickS<HLoadFar>(osize);
    } else if (op == 0x8D) {
      if (!modrm() || !mem)
        return;
      h = pickS<HLea>(osize);
    } else if (op == 0x90) {
      h = &F::nop;
    } else if (op >= 0x91 && op <= 0x97) {
      d.reg = op & 7;
      h = pickS<HXchgAcc>(osize);
    } else if (op == 0x98) {
      h = pickS<HCbw>(osize);
    } else if (op == 0x99) {
      h = pickS<HCwd>(osize);
    } else if (op >= 0xA0 && op <= 0xA3) {
      if (!imm(addr32 ? 4 : 2, false))
        return;
      d.disp = d.imm;
      d.seg = segOverride != 0xFF ? segOverride : DS;
      const int size = (op & 1) ? osize : 8;
      h = (op & 2) ? pickS<HMoffsStore>(size) : pickS<HMoffsLoad>(size);
    } else if (op == 0xA8 || op == 0xA9) {
      const int size = op == 0xA8 ? 8 : osize;
      if (!imm(size == 8 ? 1 : immBytes, false))
        return;
      h = pickS<HTestAcc>(size);
    } else if (op >= 0xB0 && op <= 0xBF) {
      d.reg = op & 7;
      const int size = op < 0xB8 ? 8 : osize;
      if (!imm(size == 8 ? 1 : immBytes, false))
        return;
      h = pickS<HMovRegImm>(size);
    } else if (op == 0xC0 || op == 0xC1 || (op >= 0xD0 && op <= 0xD3)) {
      if (!modrm())
        return;
      const int size = (op & 1) ? osize : 8;
      if (op <= 0xC1) {
        if (!imm(1, false))
          return;
        h = pickSM<HShiftImm>(size, mem);
      } else {
        h = op <= 0xD1 ? pickSM<HShiftOne>(size, mem) : pickSM<HShiftCL>(size, mem);
      }
    } else if (op == 0xC2) {
      if (!imm(2, false))
        return;
      h = pickS<HRetImm>(osize);
    } else if (op == 0xC3) {
      h = pickS<HRet>(osize);
    } else if (op == 0xC6 || op == 0xC7) {
      if (!modrm())
        return;
      const int size = op == 0xC6 ? 8 : osize;
      if (!imm(size == 8 ? 1 : immBytes, false))
        return;
      h = pickSM<HMovRMImm>(size, mem);
    } else if (op == 0xC9) {
      h = pickS<HLeave>(osize);
    } else if (op == 0xE8 || op == 0xE9) {
      if (!imm(immBytes, false))
        return;
      d.disp = op32 ? d.imm
                    : static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(d.imm)));
      h = op == 0xE8 ? pickS<HCallNear>(osize) : pickS<HJmpNear>(osize);
    } else if (op == 0xEB) {
      if (!imm(1, true))
        return;
      d.disp = d.imm;
      h = &F::jmpShort;
    } else if (op == 0xF5) {
      h = &F::cmc;
    } else if (op == 0xF8) {
      h = &F::clc;
    } else if (op == 0xF9) {
      h = &F::stc;
    } else if (op == 0xFC) {
      h = &F::cld;
    } else if (op == 0xFD) {
      h = &F::std_;
    } else if (op == 0xF6 || op == 0xF7) {
      if (!modrm())
        return;
      if (d.reg >= 6)
        return; // DIV/IDIV: legacy (#DE delivery)
      const int size = op == 0xF6 ? 8 : osize;
      if (d.reg < 2 && !imm(size == 8 ? 1 : immBytes, false))
        return;
      if (d.reg >= 4)
        cycles += m_cpu.mulExtraCycles();
      h = pickSM<HGroup3>(size, mem);
    } else if (op == 0x9B) {
      // FWAIT: with no x87 error pending (kX87) it does nothing.
      d.flags = DecodedInsn::kX87;
      h = &F::nop;
    } else if (op >= 0xD8 && op <= 0xDF) {
      if (pos >= kWindow)
        return;
      const uint8_t m = code[pos];
      // FCMOVcc and F(U)COMI(P) raise #UD before the P6: legacy decoder.
      if ((op == 0xDA || op == 0xDB || op == 0xDF) &&
          !m_cpu.hasFeature(feature::kCmov)) {
        const bool fcmov = op != 0xDF && m >= 0xC0 && m <= 0xDF;
        const bool fcomi = op != 0xDA && m >= 0xE8 && m <= 0xF7;
        if (fcmov || fcomi)
          return;
      }
      const uint32_t before = cycles;
      if (!modrm())
        return;
      // The legacy decoder resolves a 32-bit [reg+disp] operand inline
      // without the memory-operand charge, other forms through
      // getEffectiveAddress*() with it; it is added after the x87 error
      // check, so it is not part of d.cycles.
      d.imm = (mem && !(addr32 && d.rm != 4)) ? m_cpu.memOperandCycles() : 0;
      cycles = before;
      d.aux = m;
      d.flags = DecodedInsn::kX87;
      h = pickSM<HX87>(osize, mem);
    } else if (op == 0xFE || op == 0xFF) {
      if (!modrm())
        return;
      const int size = op == 0xFE ? 8 : osize;
      switch (d.reg) {
      case 0:
        h = pickSM<HInc>(size, mem);
        break;
      case 1:
        h = pickSM<HDec>(size, mem);
        break;
      case 2:
        if (op == 0xFF)
          h = pickSM<HCallRM>(osize, mem);
        break;
      case 4:
        if (op == 0xFF)
          h = pickSM<HJmpRM>(osize, mem);
        break;
      case 6:
        if (op == 0xFF)
          h = pickSM<HPushRM>(osize, mem);
        break;
      default:
        break;
      }
    }
  } else {
    if (op >= 0x80 && op <= 0x8F) {
      if (!imm(immBytes, false))
        return;
      d.disp = d.imm;
      d.aux = op & 0x0F;
      h = pickS<HJccNear>(osize);
    } else if (op >= 0x90 && op <= 0x9F) {
      if (!modrm())
        return;
      d.aux = op & 0x0F;
      h = pickSM<HSetcc>(8, mem);
    } else if (op >= 0x40 && op <= 0x4F) {
      if (!m_cpu.hasFeature(feature::kCmov) || !modrm())
        return; // #UD on models without CMOV: legacy
      d.aux = op & 0x0F;
      h = pickSM<HCmov>(osize, mem);
    } else if (op == 0xA0 || op == 0xA8) { // PUSH FS / GS
      d.reg = op == 0xA0 ? FS : GS;
      h = pickS<HPushSreg>(osize);
    } else if (op == 0xA1 || op == 0xA9) { // POP FS / GS
      d.reg = op == 0xA1 ? FS : GS;
      h = pickS<HPopSreg>(osize);
    } else if (op == 0xAF) {
      if (!modrm())
        return;
      h = pickSM<HImul>(osize, mem);
    } else if (op == 0xB6 || op == 0xB7 || op == 0xBE || op == 0xBF) {
      if (!modrm())
        return;
      if (op == 0xB6)
        h = pickSM<HMovzx8>(osize, mem);
      else if (op == 0xBE)
        h = pickSM<HMovsx8>(osize, mem);
      else if (op == 0xB7)
        h = op32 ? pickSM<HMovzx16>(32, mem) : pickSM<HMovRegRM>(16, mem);
      else
        h = op32 ? pickSM<HMovsx16>(32, mem) : pickSM<HMovRegRM>(16, mem);
    }
  }
  if (h == nullptr || pos > kWindow)
    return;

  d.op = op;
  d.len = static_cast<uint8_t>(pos);
  d.cycles = static_cast<uint16_t>(cycles);
  std::memcpy(d.bytes, code, kWindow);
  for (unsigned i = pos; i < kWindow; ++i) // zero-pad past the instruction
    d.bytes[i / 8] &= ~(0xFFull << (8 * (i % 8)));
  d.handler = h;
}

} // namespace fador::cpu
