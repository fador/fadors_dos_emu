// Differential CPU tester: executes randomly generated integer instructions
// both on the host x86-64 CPU and in the emulator's interpreter (32-bit
// flat protected mode) and compares registers and the architecturally
// defined flags.
//
//   cpu_fuzz [iterations] [seed] [filter]
//
// Only register/immediate forms are generated so the same bytes are valid
// in 64-bit mode (no REX prefixes, no stack or memory operands, no
// instructions that are invalid in long mode).  Exit status is non-zero if
// any mismatch is found.
#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "hw/BIOS.hpp"
#include "hw/DOS.hpp"
#include "hw/IOBus.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIC8259.hpp"
#include "hw/PIT8254.hpp"
#include "memory/MemoryBus.hpp"
#include "utils/Logger.hpp"

#include <cinttypes>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <random>
#include <string>
#include <sys/mman.h>
#include <vector>

using namespace fador;

namespace {

struct State {
  uint32_t r[8]; // EAX ECX EDX EBX ESP EBP ESI EDI (ESP unused)
  uint64_t flags;
};

// Host trampoline: loads registers from State (RDI), sets RFLAGS, calls the
// generated code (RSI), stores registers and RFLAGS back.
extern "C" void fuzz_trampoline(State *st, const void *code);
asm(R"(
    .text
    .globl fuzz_trampoline
fuzz_trampoline:
    push %rbx
    push %rbp
    push %r12
    push %r13
    push %r14
    push %r15
    mov %rdi, %r15
    mov %rsi, %r14
    mov 0(%r15), %eax
    mov 4(%r15), %ecx
    mov 8(%r15), %edx
    mov 12(%r15), %ebx
    mov 20(%r15), %ebp
    mov 24(%r15), %esi
    mov 28(%r15), %edi
    pushq 32(%r15)
    popfq
    call *%r14
    pushfq
    popq 32(%r15)
    mov %eax, 0(%r15)
    mov %ecx, 4(%r15)
    mov %edx, 8(%r15)
    mov %ebx, 12(%r15)
    mov %ebp, 20(%r15)
    mov %esi, 24(%r15)
    mov %edi, 28(%r15)
    cld
    pop %r15
    pop %r14
    pop %r13
    pop %r12
    pop %rbp
    pop %rbx
    ret
)");

constexpr uint32_t CF = 0x001, PF = 0x004, AF = 0x010, ZF = 0x040,
                   SF = 0x080, DF = 0x400, OF = 0x800;
constexpr uint32_t ALL = CF | PF | AF | ZF | SF | OF;

struct Insn {
  std::string name;
  std::vector<uint8_t> bytes;
  uint32_t flagMask = ALL; // flags to compare
  bool compareRegs = true; // false when the result is architecturally undefined
};

std::mt19937_64 rng;
const char *g_current = "";
std::string g_currentBytes;
uint32_t g_regs[8];
uint32_t rnd32() {
  // Bias towards interesting values
  switch (rng() % 8) {
  case 0: return 0;
  case 1: return 0xFFFFFFFFu;
  case 2: return 0x80000000u;
  case 3: return 0x7FFFFFFFu;
  case 4: return static_cast<uint32_t>(rng() % 256);
  case 5: return static_cast<uint32_t>(rng() % 65536) | (rng() % 2 ? 0xFFFF0000u : 0);
  default: return static_cast<uint32_t>(rng());
  }
}
uint8_t gpr() { // any GPR except ESP
  static const uint8_t regs[] = {0, 1, 2, 3, 5, 6, 7};
  return regs[rng() % 7];
}
uint8_t modrmReg(uint8_t reg, uint8_t rm) {
  return static_cast<uint8_t>(0xC0 | (reg << 3) | rm);
}
void imm(std::vector<uint8_t> &b, uint32_t v, int bytes) {
  for (int i = 0; i < bytes; ++i)
    b.push_back(static_cast<uint8_t>(v >> (8 * i)));
}

const char *kAlu[] = {"add", "or", "adc", "sbb", "and", "sub", "xor", "cmp"};
const char *kBt[] = {"bt", "bts", "btr", "btc"};
const char *kShift[] = {"rol", "ror", "rcl", "rcr", "shl", "shr", "sal", "sar"};

// Generate one random instruction; `size` is 8, 16 or 32.
Insn generate(State &st) {
  Insn in;
  const int size = (rng() % 3 == 0) ? 8 : ((rng() % 2) ? 16 : 32);
  const bool p66 = size == 16;
  auto pre = [&]() {
    if (p66)
      in.bytes.push_back(0x66);
  };
  const uint8_t w = size == 8 ? 0 : 1;
  const int immBytes = size == 8 ? 1 : (size == 16 ? 2 : 4);
  const char *sz = size == 8 ? "8" : (size == 16 ? "16" : "32");
  auto r8 = [&]() { return static_cast<uint8_t>(rng() % 8); };
  auto reg = [&]() { return size == 8 ? r8() : gpr(); };

  switch (rng() % 22) {
  case 0: { // ALU r/m, r
    uint8_t op = rng() % 8;
    pre();
    in.bytes.push_back(static_cast<uint8_t>(op * 8 + w + ((rng() % 2) ? 2 : 0)));
    in.bytes.push_back(modrmReg(reg(), reg()));
    in.name = std::string(kAlu[op]) + sz + " r,r";
    if (op == 1 || op == 4 || op == 6)
      in.flagMask = ALL & ~AF;
    break;
  }
  case 1: { // ALU r/m, imm (80/81/83)
    uint8_t op = rng() % 8;
    pre();
    if (size != 8 && rng() % 2) {
      in.bytes.push_back(0x83);
      in.bytes.push_back(modrmReg(op, reg()));
      imm(in.bytes, rnd32(), 1);
      in.name = std::string(kAlu[op]) + sz + " r,simm8";
    } else {
      in.bytes.push_back(size == 8 ? 0x80 : 0x81);
      in.bytes.push_back(modrmReg(op, reg()));
      imm(in.bytes, rnd32(), immBytes);
      in.name = std::string(kAlu[op]) + sz + " r,imm";
    }
    if (op == 1 || op == 4 || op == 6)
      in.flagMask = ALL & ~AF;
    break;
  }
  case 2: { // ALU acc, imm
    uint8_t op = rng() % 8;
    pre();
    in.bytes.push_back(static_cast<uint8_t>(op * 8 + 4 + w));
    imm(in.bytes, rnd32(), immBytes);
    in.name = std::string(kAlu[op]) + sz + " acc,imm";
    if (op == 1 || op == 4 || op == 6)
      in.flagMask = ALL & ~AF;
    break;
  }
  case 3: { // shifts/rotates by imm / 1 / CL
    uint8_t op = rng() % 8;
    if (op == 6)
      op = 4;
    int form = rng() % 3;
    uint32_t count;
    pre();
    if (form == 0) {
      in.bytes.push_back(size == 8 ? 0xC0 : 0xC1);
      in.bytes.push_back(modrmReg(op, reg()));
      count = rng() % 40;
      in.bytes.push_back(static_cast<uint8_t>(count));
    } else if (form == 1) {
      in.bytes.push_back(size == 8 ? 0xD0 : 0xD1);
      in.bytes.push_back(modrmReg(op, reg()));
      count = 1;
    } else {
      in.bytes.push_back(size == 8 ? 0xD2 : 0xD3);
      uint8_t r = reg();
      in.bytes.push_back(modrmReg(op, r));
      count = rng() % 40;
      st.r[1] = (st.r[1] & ~0xFFu) | count;
      count = st.r[1] & 0xFF; // CL may be clobbered if r==CL; recomputed below
    }
    in.name = std::string(kShift[op]) + sz + (form == 2 ? " r,cl" : form == 1 ? " r,1" : " r,imm");
    count &= 31;
    if (op <= 3) { // rotates: only CF/OF defined; others preserved
      in.flagMask = CF | OF | PF | ZF | SF | AF;
      in.flagMask &= ~(PF | ZF | SF | AF) | (PF | ZF | SF | AF); // preserved: compare
      if (count != 1)
        in.flagMask &= ~OF;
    } else {
      in.flagMask = ALL & ~AF;
      if (count != 1)
        in.flagMask &= ~OF;
      if (size < 32 && count >= static_cast<uint32_t>(size))
        in.flagMask &= ~CF; // CF undefined when count >= operand size
    }
    break;
  }
  case 4: { // INC/DEC (FE/FF forms; 40-4F are REX prefixes in 64-bit mode)
    uint8_t op = rng() % 2;
    pre();
    in.bytes.push_back(size == 8 ? 0xFE : 0xFF);
    in.bytes.push_back(modrmReg(op, reg()));
    in.name = std::string(op ? "dec" : "inc") + sz;
    break;
  }
  case 5: { // NOT / NEG / TEST
    uint8_t op = (rng() % 3) + 1; // 1=TEST(alias)? use 0 for TEST
    if (op == 1)
      op = 0;
    pre();
    in.bytes.push_back(size == 8 ? 0xF6 : 0xF7);
    in.bytes.push_back(modrmReg(op, reg()));
    if (op == 0) {
      imm(in.bytes, rnd32(), immBytes);
      in.name = std::string("test") + sz + " r,imm";
      in.flagMask = ALL & ~AF;
    } else {
      in.name = std::string(op == 2 ? "not" : "neg") + sz;
    }
    break;
  }
  case 6: { // MUL / IMUL (one operand)
    uint8_t op = 4 + rng() % 2;
    pre();
    in.bytes.push_back(size == 8 ? 0xF6 : 0xF7);
    in.bytes.push_back(modrmReg(op, reg()));
    in.name = std::string(op == 4 ? "mul" : "imul") + sz;
    in.flagMask = CF | OF;
    break;
  }
  case 7: { // DIV / IDIV with safe operands
    uint8_t op = 6 + rng() % 2;
    uint8_t r = gpr();
    if (r == 0 || r == 2)
      r = 3; // divisor must not be EAX/EDX
    uint32_t divisor = rnd32();
    if (size == 8)
      r = 3; // BL
    uint32_t mask = size == 32 ? 0xFFFFFFFFu : (size == 16 ? 0xFFFFu : 0xFFu);
    divisor &= mask;
    if (divisor == 0)
      divisor = 7;
    st.r[r] = (st.r[r] & ~mask) | divisor;
    if (op == 6) { // DIV: high part < divisor
      if (size == 8)
        st.r[0] = (st.r[0] & ~0xFF00u) | (((st.r[0] >> 8) % divisor) << 8);
      else
        st.r[2] = (st.r[2] & ~mask) | ((st.r[2] & mask) % divisor);
    } else { // IDIV: sign-extend dividend, avoid MIN/-1
      if (size == 8) {
        if ((divisor & 0xFF) == 0xFF && (st.r[0] & 0xFF) == 0x80)
          st.r[0] = (st.r[0] & ~0xFFu) | 0x10;
        int8_t lo = static_cast<int8_t>(st.r[0] & 0xFF);
        st.r[0] = (st.r[0] & ~0xFF00u) | ((lo < 0 ? 0xFFu : 0u) << 8);
      } else {
        uint32_t lo = st.r[0] & mask;
        if (divisor == mask && lo == (1u << (size - 1))) {
          st.r[0] = (st.r[0] & ~mask) | 0x10;
          lo = 0x10;
        }
        bool neg = (lo >> (size - 1)) & 1;
        st.r[2] = (st.r[2] & ~mask) | (neg ? mask : 0);
      }
    }
    pre();
    in.bytes.push_back(size == 8 ? 0xF6 : 0xF7);
    in.bytes.push_back(modrmReg(op, r));
    in.name = std::string(op == 6 ? "div" : "idiv") + sz;
    in.flagMask = 0;
    break;
  }
  case 8: { // IMUL r, r/m [, imm]
    if (size == 8)
      return generate(st);
    int form = rng() % 3;
    pre();
    if (form == 0) {
      in.bytes.push_back(0x0F);
      in.bytes.push_back(0xAF);
      in.bytes.push_back(modrmReg(gpr(), gpr()));
      in.name = std::string("imul") + sz + " r,r";
    } else if (form == 1) {
      in.bytes.push_back(0x6B);
      in.bytes.push_back(modrmReg(gpr(), gpr()));
      imm(in.bytes, rnd32(), 1);
      in.name = std::string("imul") + sz + " r,r,imm8";
    } else {
      in.bytes.push_back(0x69);
      in.bytes.push_back(modrmReg(gpr(), gpr()));
      imm(in.bytes, rnd32(), immBytes);
      in.name = std::string("imul") + sz + " r,r,imm";
    }
    in.flagMask = CF | OF;
    break;
  }
  case 9: { // SHLD / SHRD
    if (size == 8)
      return generate(st);
    bool right = rng() % 2;
    bool byCl = rng() % 2;
    uint32_t count = rng() % 40;
    pre();
    in.bytes.push_back(0x0F);
    in.bytes.push_back(static_cast<uint8_t>((right ? 0xAC : 0xA4) + (byCl ? 1 : 0)));
    in.bytes.push_back(modrmReg(gpr(), gpr()));
    if (byCl)
      st.r[1] = (st.r[1] & ~0xFFu) | count;
    else
      in.bytes.push_back(static_cast<uint8_t>(count));
    in.name = std::string(right ? "shrd" : "shld") + sz + (byCl ? " cl" : " imm");
    count &= 31;
    in.flagMask = ALL & ~AF;
    if (count != 1)
      in.flagMask &= ~OF;
    if (size == 16 && count > 16) {
      in.flagMask = 0; // result undefined
      in.compareRegs = false;
      in.name += " (undef)";
    }
    break;
  }
  case 10: { // BT/BTS/BTR/BTC r,r and r,imm8
    if (size == 8)
      return generate(st);
    uint8_t op = rng() % 4;
    pre();
    if (rng() % 2) {
      in.bytes.push_back(0x0F);
      in.bytes.push_back(static_cast<uint8_t>(0xA3 + op * 8));
      in.bytes.push_back(modrmReg(gpr(), gpr()));
      in.name = std::string(kBt[op]) + sz + " r,r";
    } else {
      in.bytes.push_back(0x0F);
      in.bytes.push_back(0xBA);
      in.bytes.push_back(modrmReg(4 + op, gpr()));
      in.bytes.push_back(static_cast<uint8_t>(rng()));
      in.name = std::string(kBt[op]) + sz + " r,imm";
    }
    in.flagMask = CF;
    break;
  }
  case 11: { // BSF / BSR
    if (size == 8)
      return generate(st);
    pre();
    in.bytes.push_back(0x0F);
    bool rev = rng() % 2;
    in.bytes.push_back(rev ? 0xBD : 0xBC);
    in.bytes.push_back(modrmReg(gpr(), gpr()));
    in.name = std::string(rev ? "bsr" : "bsf") + sz;
    in.flagMask = ZF;
    break;
  }
  case 12: { // MOVZX / MOVSX
    if (size == 8)
      return generate(st);
    bool sx = rng() % 2;
    bool word = rng() % 2;
    pre();
    in.bytes.push_back(0x0F);
    in.bytes.push_back(static_cast<uint8_t>((sx ? 0xBE : 0xB6) + (word ? 1 : 0)));
    in.bytes.push_back(modrmReg(gpr(), word ? gpr() : r8()));
    in.name = std::string(sx ? "movsx" : "movzx") + sz + (word ? " r,r16" : " r,r8");
    break;
  }
  case 13: { // BSWAP
    in.bytes.push_back(0x0F);
    in.bytes.push_back(static_cast<uint8_t>(0xC8 + gpr()));
    in.name = "bswap32";
    break;
  }
  case 14: { // XCHG / XADD / CMPXCHG
    int form = rng() % 3;
    pre();
    if (form == 0) {
      in.bytes.push_back(static_cast<uint8_t>(0x86 + w));
      in.bytes.push_back(modrmReg(reg(), reg()));
      in.name = std::string("xchg") + sz;
    } else if (form == 1) {
      in.bytes.push_back(0x0F);
      in.bytes.push_back(static_cast<uint8_t>(0xC0 + w));
      in.bytes.push_back(modrmReg(reg(), reg()));
      in.name = std::string("xadd") + sz;
    } else {
      in.bytes.push_back(0x0F);
      in.bytes.push_back(static_cast<uint8_t>(0xB0 + w));
      in.bytes.push_back(modrmReg(reg(), reg()));
      in.name = std::string("cmpxchg") + sz;
    }
    break;
  }
  case 15: { // SETcc
    uint8_t cc = rng() % 16;
    in.bytes.push_back(0x0F);
    in.bytes.push_back(static_cast<uint8_t>(0x90 + cc));
    in.bytes.push_back(modrmReg(0, r8()));
    in.name = "setcc";
    break;
  }
  case 16: { // CMOVcc
    if (size == 8)
      return generate(st);
    uint8_t cc = rng() % 16;
    pre();
    in.bytes.push_back(0x0F);
    in.bytes.push_back(static_cast<uint8_t>(0x40 + cc));
    in.bytes.push_back(modrmReg(gpr(), gpr()));
    in.name = std::string("cmovcc") + sz;
    break;
  }
  case 17: { // CBW/CWDE/CWD/CDQ/LAHF/SAHF/CLC/STC/CMC
    static const uint8_t ops[] = {0x98, 0x99, 0x9E, 0x9F, 0xF8, 0xF9, 0xF5};
    static const char *names[] = {"cbw/cwde", "cwd/cdq", "sahf", "lahf", "clc", "stc", "cmc"};
    int i = rng() % 7;
    if (i <= 1)
      pre();
    in.bytes.push_back(ops[i]);
    in.name = std::string(names[i]) + (i <= 1 ? sz : "");
    break;
  }
  case 18: { // LEA with register addressing (no memory access)
    if (size == 8)
      return generate(st);
    pre();
    in.bytes.push_back(0x8D);
    uint8_t mod = rng() % 3;
    uint8_t dst = gpr();
    if (rng() % 2) { // SIB
      in.bytes.push_back(static_cast<uint8_t>((mod << 6) | (dst << 3) | 4));
      uint8_t base = gpr();
      if (mod == 0 && base == 5)
        base = 3;
      uint8_t index = rng() % 8; // 4 = no index
      in.bytes.push_back(static_cast<uint8_t>(((rng() % 4) << 6) | (index << 3) | base));
    } else {
      uint8_t base = gpr();
      if (mod == 0 && base == 5)
        base = 3;
      in.bytes.push_back(static_cast<uint8_t>((mod << 6) | (dst << 3) | base));
    }
    if (mod == 1)
      imm(in.bytes, rnd32(), 1);
    else if (mod == 2)
      imm(in.bytes, rnd32(), 4);
    in.name = std::string("lea") + sz;
    break;
  }
  case 19: { // MOV r, imm / r,r / movsx-like via 8-bit regs
    pre();
    if (rng() % 2) {
      in.bytes.push_back(static_cast<uint8_t>((size == 8 ? 0xB0 : 0xB8) + (size == 8 ? r8() : gpr())));
      imm(in.bytes, rnd32(), immBytes);
      in.name = std::string("mov") + sz + " r,imm";
    } else {
      in.bytes.push_back(static_cast<uint8_t>(0x88 + w + ((rng() % 2) ? 2 : 0)));
      in.bytes.push_back(modrmReg(reg(), reg()));
      in.name = std::string("mov") + sz + " r,r";
    }
    break;
  }
  case 20: { // ADC/SBB chains are covered by ALU; use RCL/RCR by 1 with CF
    uint8_t op = 2 + rng() % 2;
    pre();
    in.bytes.push_back(size == 8 ? 0xD0 : 0xD1);
    in.bytes.push_back(modrmReg(op, reg()));
    in.name = std::string(kShift[op]) + sz + " r,1";
    in.flagMask = CF | OF | PF | ZF | SF;
    break;
  }
  default: { // XCHG eAX, r (91-97) and NOP-free forms
    if (size == 8)
      return generate(st);
    pre();
    uint8_t r = gpr();
    if (r == 0)
      r = 1;
    in.bytes.push_back(static_cast<uint8_t>(0x90 + r));
    in.name = std::string("xchg") + sz + " eax,r";
    break;
  }
  }
  return in;
}

struct Emulator {
  cpu::CPU cpu;
  memory::MemoryBus mem;
  hw::IOBus iobus;
  hw::KeyboardController kbd;
  hw::PIT8254 pit;
  hw::PIC8259 pic{true};
  hw::BIOS bios{cpu, mem, kbd, pit, pic};
  hw::DOS dos{cpu, mem};
  cpu::InstructionDecoder decoder{cpu, mem, iobus, bios, dos};
  static constexpr uint32_t GDT = 0x1000;
  static constexpr uint32_t CODE = 0x20000;

  void writeDesc(uint32_t addr, bool code) {
    uint32_t low = 0x0000FFFF;
    uint32_t high = 0x00CF9200 | (code ? 0x0A00 : 0);
    mem.write32(addr, low);
    mem.write32(addr + 4, high);
  }
  Emulator() {
    mem.setA20(true);
    writeDesc(GDT + 8, true);
    writeDesc(GDT + 16, false);
    cpu.setGDTR({0x17, GDT});
    cpu.setCR(0, cpu.getCR(0) | 1);
    decoder.loadSegment(cpu::CS, 0x08);
    decoder.loadSegment(cpu::DS, 0x10);
    decoder.loadSegment(cpu::ES, 0x10);
    decoder.loadSegment(cpu::SS, 0x10);
  }
  // Returns false if the instruction did not complete normally (e.g. it
  // raised #UD in the interpreter).
  bool run(const Insn &in, State &st) {
    cpu.setGDTR({0x17, GDT});
    cpu.setCR(0, cpu.getCR(0) | 1);
    cpu.setEFLAGS(0x202);
    decoder.loadSegment(cpu::CS, 0x08);
    decoder.loadSegment(cpu::DS, 0x10);
    decoder.loadSegment(cpu::ES, 0x10);
    decoder.loadSegment(cpu::SS, 0x10);
    for (size_t i = 0; i < in.bytes.size(); ++i)
      mem.write8(CODE + static_cast<uint32_t>(i), in.bytes[i]);
    mem.write8(CODE + static_cast<uint32_t>(in.bytes.size()), 0x90);
    for (int i = 0; i < 8; ++i)
      if (i != 4)
        cpu.setReg32(static_cast<uint8_t>(i), st.r[i]);
    cpu.setReg32(cpu::ESP, 0x80000);
    cpu.setEFLAGS(0x00000202u | (static_cast<uint32_t>(st.flags) & (ALL | DF)));
    cpu.setEIP(CODE);
    decoder.step();
    for (int i = 0; i < 8; ++i)
      if (i != 4)
        st.r[i] = cpu.getReg32(static_cast<uint8_t>(i));
    st.flags = cpu.getEFLAGS();
    return cpu.getSegReg(cpu::CS) == 0x08 &&
           cpu.getEIP() == CODE + in.bytes.size();
  }
};

std::string hexBytes(const std::vector<uint8_t> &b) {
  std::string s;
  char buf[4];
  for (uint8_t v : b) {
    std::snprintf(buf, sizeof(buf), "%02X ", v);
    s += buf;
  }
  return s;
}

} // namespace

int main(int argc, char **argv) {
  uint64_t iterations = argc > 1 ? std::strtoull(argv[1], nullptr, 0) : 200000;
  uint64_t seed = argc > 2 ? std::strtoull(argv[2], nullptr, 0) : 12345;
  std::string filter = argc > 3 ? argv[3] : "";
  rng.seed(seed);
  // Silence the interpreter's diagnostics (#UD dumps etc.)
  utils::currentLevel = static_cast<utils::LogLevel>(
      static_cast<int>(utils::LogLevel::Error) + 1);

  uint8_t *page = static_cast<uint8_t *>(mmap(nullptr, 4096,
                                              PROT_READ | PROT_WRITE | PROT_EXEC,
                                              MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));
  if (page == MAP_FAILED) {
    std::perror("mmap");
    return 2;
  }

  std::signal(SIGFPE, [](int) {
    std::fprintf(stderr, "host fault in generated test '%s' (%s) EAX=%08X ECX=%08X EDX=%08X EBX=%08X\n", g_current,
                 g_currentBytes.c_str(), g_regs[0], g_regs[1], g_regs[2], g_regs[3]);
    std::_Exit(3);
  });
  Emulator emu;
  std::map<std::string, std::pair<uint64_t, uint64_t>> stats; // name -> (runs, fails)
  std::map<std::string, std::string> firstFailure;
  uint64_t failures = 0, executed = 0;

  for (uint64_t it = 0; it < iterations; ++it) {
    State init{};
    for (int i = 0; i < 8; ++i)
      init.r[i] = rnd32();
    init.r[4] = 0;
    init.flags = 0x202 | (rng() & ALL);
    Insn in = generate(init);
    if (!filter.empty() && in.name.find(filter) == std::string::npos)
      continue;
    ++executed;

    State host = init, guest = init;
    g_current = in.name.c_str();
    g_currentBytes = hexBytes(in.bytes);
    std::memcpy(g_regs, init.r, sizeof(g_regs));
    std::memcpy(page, in.bytes.data(), in.bytes.size());
    page[in.bytes.size()] = 0xC3; // ret
    fuzz_trampoline(&host, page);
    bool completed = emu.run(in, guest);

    bool ok = completed;
    for (int i = 0; i < 8; ++i)
      if (in.compareRegs && i != 4 && host.r[i] != guest.r[i])
        ok = false;
    uint32_t hf = static_cast<uint32_t>(host.flags) & in.flagMask;
    uint32_t gf = static_cast<uint32_t>(guest.flags) & in.flagMask;
    if (hf != gf)
      ok = false;

    auto &s = stats[in.name];
    ++s.first;
    if (!ok) {
      ++s.second;
      ++failures;
      if (!firstFailure.count(in.name)) {
        char buf[1024];
        int n = std::snprintf(buf, sizeof(buf), "  bytes: %s\n  in : ",
                              hexBytes(in.bytes).c_str());
        std::string msg(buf, n);
        auto dump = [&](const State &x) {
          std::string out;
          static const char *names[] = {"EAX", "ECX", "EDX", "EBX", "ESP", "EBP", "ESI", "EDI"};
          for (int i = 0; i < 8; ++i) {
            if (i == 4)
              continue;
            std::snprintf(buf, sizeof(buf), "%s=%08X ", names[i], x.r[i]);
            out += buf;
          }
          std::snprintf(buf, sizeof(buf), "FL=%03X",
                        static_cast<uint32_t>(x.flags) & (ALL | DF));
          return out + buf;
        };
        msg += dump(init) + "\n  cpu: " + dump(host) + "\n  emu: " + dump(guest);
        if (!completed)
          msg += "\n  emu: instruction faulted / not implemented";
        std::snprintf(buf, sizeof(buf), "\n  compared flags mask=%03X\n", in.flagMask);
        msg += buf;
        firstFailure[in.name] = msg;
      }
    }
  }

  std::printf("cpu_fuzz: %" PRIu64 " instructions executed, %" PRIu64
              " mismatches (seed %" PRIu64 ")\n",
              executed, failures, seed);
  for (const auto &[name, s] : stats) {
    if (s.second)
      std::printf("FAIL %-22s %6" PRIu64 "/%" PRIu64 "\n%s", name.c_str(),
                  s.second, s.first, firstFailure[name].c_str());
  }
  return failures ? 1 : 0;
}
