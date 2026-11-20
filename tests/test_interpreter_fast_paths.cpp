// Interpreter fast paths: lazily computed arithmetic flags, the cached
// instruction-fetch window and the batch runner.  Every test compares the
// fast machinery against an eager reference or against the architectural
// result that must not change.
#include "test_framework.hpp"
#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "hw/BIOS.hpp"
#include "hw/DOS.hpp"
#include "hw/IOBus.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIC8259.hpp"
#include "hw/PIT8254.hpp"
#include "memory/MemoryBus.hpp"

#include <bit>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <initializer_list>
#include <random>
#include <vector>

using namespace fador;

namespace {

constexpr uint32_t kArith = 0x8D5; // OF SF ZF AF PF CF

// The eager flag computation the interpreter used before flags became lazy
// (InstructionDecoder::aluOp up to 8120f53): the reference for every op.
uint32_t eagerAlu(uint8_t op, uint32_t dest, uint32_t src, int size,
                  uint32_t &flags) {
  const uint64_t mask = (size == 8) ? 0xFFull : (size == 16) ? 0xFFFFull
                                                             : 0xFFFFFFFFull;
  const uint64_t msb = 1ull << (size - 1);
  const uint64_t d = dest & mask, s = src & mask;
  const uint64_t cIn = flags & 1u;
  uint32_t f = flags & ~kArith;
  uint64_t r = 0;
  switch (op) {
  case 0:
  case 2:
    r = d + s + (op == 2 ? cIn : 0);
    if (r & (mask + 1))
      f |= 0x001;
    if ((d ^ r) & (s ^ r) & msb)
      f |= 0x800;
    if ((d ^ s ^ r) & 0x10)
      f |= 0x010;
    break;
  case 3:
  case 5:
  case 7: {
    const uint64_t borrow = (op == 3) ? cIn : 0;
    r = d - s - borrow;
    if (d < s + borrow)
      f |= 0x001;
    if ((d ^ s) & (d ^ r) & msb)
      f |= 0x800;
    if ((d ^ s ^ r) & 0x10)
      f |= 0x010;
    break;
  }
  case 1:
    r = d | s;
    break;
  case 4:
    r = d & s;
    break;
  case 6:
    r = d ^ s;
    break;
  }
  r &= mask;
  if (r == 0)
    f |= 0x040;
  if (r & msb)
    f |= 0x080;
  if (!(std::popcount(static_cast<uint8_t>(r)) & 1))
    f |= 0x004;
  flags = f;
  return static_cast<uint32_t>(r);
}

bool conditionFromFlags(uint32_t f, uint8_t cc) {
  const bool cf = f & 0x001, pf = f & 0x004, zf = f & 0x040,
             sf = f & 0x080, of = f & 0x800;
  bool r = false;
  switch (cc >> 1) {
  case 0: r = of; break;
  case 1: r = cf; break;
  case 2: r = zf; break;
  case 3: r = cf || zf; break;
  case 4: r = sf; break;
  case 5: r = pf; break;
  case 6: r = sf != of; break;
  case 7: r = zf || (sf != of); break;
  }
  return (cc & 1) ? !r : r;
}

// Real-mode machine running code at 1000:0000.
struct Machine {
  cpu::CPU cpu;
  memory::MemoryBus mem;
  hw::IOBus iobus;
  hw::KeyboardController kbd;
  hw::PIT8254 pit;
  hw::PIC8259 pic{true};
  hw::DOS dos{cpu, mem};
  hw::BIOS bios{cpu, mem, kbd, pit, pic};
  cpu::InstructionDecoder decoder{cpu, mem, iobus, bios, dos};
  static constexpr uint32_t kCode = 0x10000;

  Machine() {
    cpu.setSegReg(cpu::CS, kCode >> 4);
    cpu.setSegReg(cpu::DS, 0x2000);
    cpu.setSegReg(cpu::ES, 0x3000);
    cpu.setSegReg(cpu::SS, 0x4000);
    cpu.setReg32(cpu::ESP, 0xFFF0);
    cpu.setEFLAGS(0x2);
    cpu.setEIP(0);
    decoder.syncSegments();
  }
  void load(const std::vector<uint8_t> &code, uint32_t at = 0) {
    for (size_t i = 0; i < code.size(); ++i)
      mem.write8(kCode + at + static_cast<uint32_t>(i), code[i]);
  }
  // Loads `code` at CS:0 and executes `steps` instructions from there.
  void run(const std::vector<uint8_t> &code, int steps) {
    load(code);
    cpu.setEIP(0);
    for (int i = 0; i < steps; ++i)
      decoder.step();
  }
};

// "op reg, reg" in real mode: 8-bit (op*8+0), 16-bit (op*8+1), 32-bit with
// an operand-size prefix.  ModRM C8 = op AX/AL, CX/CL.
std::vector<uint8_t> aluRegReg(uint8_t op, int size) {
  std::vector<uint8_t> b;
  if (size == 32)
    b.push_back(0x66);
  b.push_back(static_cast<uint8_t>(op * 8 + (size == 8 ? 0 : 1)));
  b.push_back(0xC8);
  return b;
}

uint32_t pick(std::mt19937 &rng) {
  switch (rng() % 6) {
  case 0: return 0;
  case 1: return 0xFFFFFFFFu;
  case 2: return 0x80000000u >> (rng() % 32);
  case 3: return 0x7FFFFFFFu >> (rng() % 32);
  default: return static_cast<uint32_t>(rng());
  }
}

} // namespace

TEST_CASE("Lazy flags: every ALU op, size and carry-in matches the eager reference",
          "[CPU][LazyFlags]") {
  Machine m;
  std::mt19937 rng(1234);
  for (int iter = 0; iter < 4000; ++iter) {
    const uint8_t op = static_cast<uint8_t>(iter % 8);
    const int size = (iter / 8) % 3 == 0 ? 8 : ((iter / 8) % 3 == 1 ? 16 : 32);
    const uint32_t a = pick(rng), c = pick(rng);
    const uint32_t f0 = 0x2 | (rng() & kArith);
    m.cpu.setReg32(cpu::EAX, a);
    m.cpu.setReg32(cpu::ECX, c);
    m.cpu.setEFLAGS(f0);
    m.run(aluRegReg(op, size), 1);

    uint32_t ref = f0;
    const uint32_t res = eagerAlu(op, a, c, size, ref);
    // Conditions straight from the lazy record, before anything folds it.
    for (uint8_t cc = 0; cc < 16; ++cc)
      REQUIRE(m.cpu.testCondition(cc) == conditionFromFlags(ref, cc));
    REQUIRE(m.cpu.carryFlag() == ((ref & 1) != 0));
    REQUIRE(m.cpu.zeroFlag() == ((ref & 0x40) != 0));
    REQUIRE(m.cpu.getEFLAGS() == ref);
    const uint32_t mask = size == 8 ? 0xFF : size == 16 ? 0xFFFF : 0xFFFFFFFF;
    if (op != 7)
      REQUIRE((m.cpu.getReg32(cpu::EAX) & mask) == res);
  }
}

TEST_CASE("Lazy flags: INC/DEC keep the carry of the previous lazy op",
          "[CPU][LazyFlags]") {
  Machine m;
  SECTION("ADD sets CF, INC keeps it") {
    m.cpu.setReg32(cpu::EAX, 0xFFFF);
    m.cpu.setReg32(cpu::ECX, 0x0001);
    m.cpu.setReg32(cpu::EBX, 0x7FFF);
    // add ax,cx (CF=1, ZF=1); inc bx (OF=1, SF=1, ZF=0, AF=1)
    m.run({0x01, 0xC8, 0x43}, 2);
    REQUIRE(m.cpu.carryFlag());
    REQUIRE(m.cpu.testCondition(0x2));  // JB
    REQUIRE(m.cpu.testCondition(0x0));  // JO
    REQUIRE(!m.cpu.testCondition(0x4)); // JZ
    const uint32_t f = m.cpu.getEFLAGS();
    REQUIRE((f & kArith) == (0x001 | 0x800 | 0x080 | 0x010 | 0x004));
  }
  SECTION("SUB clears CF, DEC to zero keeps it clear") {
    m.cpu.setReg32(cpu::EAX, 5);
    m.cpu.setReg32(cpu::ECX, 3);
    m.cpu.setReg32(cpu::EBX, 1);
    m.run({0x29, 0xC8, 0x4B}, 2); // sub ax,cx; dec bx
    const uint32_t f = m.cpu.getEFLAGS();
    REQUIRE((f & 0x001) == 0);
    REQUIRE((f & 0x040) != 0);
  }
  SECTION("STC then INC then ADC adds the preserved carry") {
    m.cpu.setReg32(cpu::EAX, 1);
    m.cpu.setReg32(cpu::ECX, 1);
    m.cpu.setReg32(cpu::EBX, 0);
    m.run({0xF9, 0x43, 0x43, 0x11, 0xC8}, 4); // stc; inc bx; inc bx; adc ax,cx
    REQUIRE(m.cpu.getReg16(cpu::AX) == 3);
    REQUIRE(m.cpu.getReg16(cpu::BX) == 2);
    REQUIRE((m.cpu.getEFLAGS() & 1) == 0);
  }
  SECTION("Every INC/DEC after every lazy op matches the reference") {
    std::mt19937 rng(99);
    for (int iter = 0; iter < 2000; ++iter) {
      const uint8_t op = static_cast<uint8_t>(rng() % 8);
      const int size = (rng() % 2) ? 16 : 32;
      const bool dec = rng() & 1;
      const uint32_t a = pick(rng), c = pick(rng), b = pick(rng);
      const uint32_t f0 = 0x2 | (rng() & kArith);
      m.cpu.setReg32(cpu::EAX, a);
      m.cpu.setReg32(cpu::ECX, c);
      m.cpu.setReg32(cpu::EBX, b);
      m.cpu.setEFLAGS(f0);
      std::vector<uint8_t> code = aluRegReg(op, size);
      if (size == 32)
        code.push_back(0x66);
      code.push_back(dec ? 0x4B : 0x43);
      m.run(code, 2);
      uint32_t ref = f0;
      eagerAlu(op, a, c, size, ref);
      const uint32_t cf = ref & 1;
      eagerAlu(dec ? 5 : 0, b, 1, size, ref);
      ref = (ref & ~1u) | cf;
      for (uint8_t cc = 0; cc < 16; ++cc)
        REQUIRE(m.cpu.testCondition(cc) == conditionFromFlags(ref, cc));
      REQUIRE(m.cpu.getEFLAGS() == ref);
    }
  }
}

TEST_CASE("Lazy flags: shifts by zero and masked counts leave them pending",
          "[CPU][LazyFlags]") {
  Machine m;
  // cmp ax,cx (AX<CX: CF=1, SF=1); shl bx,cl with CL=0 / CL=32; jb +2
  for (uint8_t count : {0, 32}) {
    m.cpu.setReg32(cpu::EAX, 1);
    m.cpu.setReg32(cpu::ECX, 0x00000200u | count);
    m.cpu.setReg32(cpu::EBX, 0x1234);
    m.cpu.setEFLAGS(0x2);
    m.run({0x38, 0xC8, // cmp al,cl  (1 - count: CF=1 unless count==0)
           0xD3, 0xE3},  // shl bx,cl
          2);
    uint32_t ref = 0x2;
    eagerAlu(7, 1, count, 8, ref);
    REQUIRE(m.cpu.getReg16(cpu::BX) == 0x1234);
    REQUIRE(m.cpu.getEFLAGS() == ref);
  }
  // A real shift replaces the arithmetic flags it defines.
  m.cpu.setReg32(cpu::EAX, 0);
  m.cpu.setReg32(cpu::ECX, 1);
  m.cpu.setReg32(cpu::EBX, 0x8000);
  m.cpu.setEFLAGS(0x2);
  m.run({0x38, 0xC8, 0xD3, 0xE3}, 2); // cmp al,cl (CF=1); shl bx,1 (CF=1, ZF=1)
  REQUIRE(m.cpu.getReg16(cpu::BX) == 0);
  REQUIRE((m.cpu.getEFLAGS() & 0x041) == 0x041);
}

TEST_CASE("Lazy flags: shifts and rotates replace only the flags they write",
          "[CPU][LazyFlags]") {
  Machine m;
  SECTION("ROL by 3 after ADD: CF from the rotate, the rest from the ADD") {
    m.cpu.setReg32(cpu::EAX, 0x8F);
    m.cpu.setReg32(cpu::ECX, 0x71);
    m.cpu.setReg32(cpu::EBX, 0x2001);
    m.cpu.setEFLAGS(0x2 | 0x800);
    m.run({0x00, 0xC8, 0xC1, 0xC3, 0x03}, 2); // add al,cl ; rol bx,3
    uint32_t ref = 0x2 | 0x800;
    eagerAlu(0, 0x8F, 0x71, 8, ref); // 0x100: CF ZF AF PF, OF clear
    ref = (ref & ~1u) | 1u;          // ROL 0x2001 by 3 = 0x0009: CF = 1
    REQUIRE(m.cpu.getReg16(cpu::BX) == 0x0009);
    REQUIRE(m.cpu.testCondition(0x4)); // JZ from the ADD
    REQUIRE(m.cpu.getEFLAGS() == ref);
  }
  SECTION("SHL by 2 after SUB: AF and OF stay from the SUB") {
    m.cpu.setReg32(cpu::EAX, 0x10);
    m.cpu.setReg32(cpu::ECX, 0x81);
    m.cpu.setReg32(cpu::EBX, 0x4001);
    m.cpu.setEFLAGS(0x2);
    m.run({0x28, 0xC8, 0xC1, 0xE3, 0x02}, 2); // sub al,cl ; shl bx,2
    uint32_t ref = 0x2;
    eagerAlu(5, 0x10, 0x81, 8, ref); // 0x8F: CF AF OF SF
    // SHL 0x4001 by 2 = 0x0004: CF = 1, ZF = 0, SF = 0, PF = 0
    ref = (ref & ~(0x001u | 0x040u | 0x080u | 0x004u)) | 0x001u;
    REQUIRE(m.cpu.getReg16(cpu::BX) == 0x0004);
    REQUIRE(m.cpu.getEFLAGS() == ref);
  }
  SECTION("RCL takes its carry in from a pending SUB") {
    m.cpu.setReg32(cpu::EAX, 1);
    m.cpu.setReg32(cpu::ECX, 2);
    m.cpu.setReg32(cpu::EBX, 0);
    m.run({0x28, 0xC8, 0xD1, 0xD3}, 2); // sub al,cl (CF=1) ; rcl bx,1
    REQUIRE(m.cpu.getReg16(cpu::BX) == 1);
    REQUIRE(!m.cpu.carryFlag());
  }
  SECTION("CLC/STC/CMC after CMP keep ZF and SF") {
    m.cpu.setReg32(cpu::EAX, 5);
    m.cpu.setReg32(cpu::ECX, 5);
    m.run({0x38, 0xC8, 0xF9, 0xF5, 0xF5}, 4); // cmp al,cl ; stc ; cmc ; cmc
    REQUIRE(m.cpu.carryFlag());
    REQUIRE(m.cpu.testCondition(0x4));
    uint32_t ref = 0x2;
    eagerAlu(7, 5, 5, 8, ref);
    REQUIRE(m.cpu.getEFLAGS() == (ref | 1u));
  }
}

TEST_CASE("Lazy flags: MUL, IMUL and DIV see the pending flags", "[CPU][LazyFlags]") {
  Machine m;
  SECTION("MUL keeps SF/ZF/AF/PF of the preceding SUB") {
    m.cpu.setReg32(cpu::EAX, 0x10);
    m.cpu.setReg32(cpu::ECX, 0x20);
    m.cpu.setReg32(cpu::EBX, 3);
    // sub ax,cx (negative, CF=1); mov ax,0x100 ; mul bx (DX:AX=0x300: CF=OF=0)
    m.run({0x29, 0xC8, 0xB8, 0x00, 0x01, 0xF7, 0xE3}, 3);
    uint32_t ref = 0x2;
    eagerAlu(5, 0x10, 0x20, 16, ref);
    ref &= ~0x801u;
    REQUIRE(m.cpu.getReg16(cpu::AX) == 0x300);
    REQUIRE(m.cpu.getEFLAGS() == ref);
  }
  SECTION("IMUL overflow after a logic op") {
    m.cpu.setReg32(cpu::EAX, 0x4000);
    m.cpu.setReg32(cpu::ECX, 0);
    m.cpu.setReg32(cpu::EBX, 4);
    // and cx,cx (ZF=1); imul bx (0x10000: CF=OF=1)
    m.run({0x21, 0xC9, 0xF7, 0xEB}, 2);
    REQUIRE(m.cpu.testCondition(0x2)); // JB: CF
    REQUIRE(m.cpu.testCondition(0x0)); // JO
    REQUIRE(m.cpu.testCondition(0x4)); // JZ: ZF of the AND
    REQUIRE((m.cpu.getEFLAGS() & kArith) == (0x001 | 0x800 | 0x040 | 0x004));
  }
  SECTION("DIV after CMP") {
    m.cpu.setReg32(cpu::EAX, 100);
    m.cpu.setReg32(cpu::EDX, 0);
    m.cpu.setReg32(cpu::ECX, 7);
    m.cpu.setReg32(cpu::EBX, 9);
    m.cpu.setEFLAGS(0x2);
    m.run({0x39, 0xD9, 0xF7, 0xF3}, 2); // cmp cx,bx; div bx
    uint32_t ref = 0x2;
    eagerAlu(7, 7, 9, 16, ref);
    REQUIRE(m.cpu.getReg16(cpu::AX) == 11);
    REQUIRE(m.cpu.getReg16(cpu::DX) == 1);
    REQUIRE((m.cpu.getEFLAGS() & kArith) == (ref & kArith));
  }
}

TEST_CASE("Lazy flags: PUSHF, LAHF, SETcc, Jcc and SAHF observe them",
          "[CPU][LazyFlags]") {
  Machine m;
  m.cpu.setReg32(cpu::EAX, 0x7F);
  m.cpu.setReg32(cpu::ECX, 0x01);
  m.cpu.setEFLAGS(0x2);
  // add al,cl (0x80: OF SF AF); pushf; lahf; seto bl; jo +1; hlt...
  m.run({0x00, 0xC8, 0x9C, 0x9F, 0x0F, 0x90, 0xC3, 0x70, 0x01}, 5);
  uint32_t ref = 0x2;
  eagerAlu(0, 0x7F, 0x01, 8, ref);
  const uint16_t pushed = m.mem.read16(0x40000 + 0xFFEE);
  REQUIRE(pushed == ref);
  REQUIRE(m.cpu.getReg8(cpu::AH) == (ref & 0xFF));
  REQUIRE(m.cpu.getReg8(cpu::BL) == 1);
  REQUIRE(m.cpu.getEIP() == 10); // JO taken
  // SAHF replaces SF ZF AF PF CF and drops the lazy record.
  m.cpu.setReg8(cpu::AH, 0x41); // ZF CF
  m.run({0x00, 0xC8, 0x9E}, 2);
  REQUIRE((m.cpu.getEFLAGS() & 0xD5) == 0x41);
  REQUIRE((m.cpu.getEFLAGS() & 0x800) == 0); // OF from the ADD (0x80+1)
}

// Flat 32-bit protected mode owned by the guest (native protection), with
// paging identity-mapping 0..4 MiB and an IDT gate for #PF.
struct PagedMachine : Machine {
  static constexpr uint32_t kGdt = 0x1000, kIdt = 0x2000;
  static constexpr uint32_t kDir = 0x100000, kTable = 0x101000;
  static constexpr uint32_t kPfHandler = 0x7000;
  static constexpr uint32_t kHole = 0x50000; // not-present page
  PagedMachine() {
    mem.setA20(true);
    mem.write32(kGdt + 8, 0x0000FFFF);
    mem.write32(kGdt + 12, 0x00CF9A00);
    mem.write32(kGdt + 16, 0x0000FFFF);
    mem.write32(kGdt + 20, 0x00CF9200);
    mem.write32(kIdt + 14 * 8, (0x08u << 16) | (kPfHandler & 0xFFFF));
    mem.write32(kIdt + 14 * 8 + 4, (kPfHandler & 0xFFFF0000u) | 0x8E00);
    cpu.setGDTR({0x17, kGdt});
    cpu.setIDTR({0x7FF, kIdt});
    for (uint32_t i = 0; i < 1024; ++i)
      mem.write32(kTable + i * 4,
                  (i << 12) | ((i << 12) == kHole ? 0x06 : 0x07));
    mem.write32(kDir, kTable | 0x07);
    cpu.setCR(3, kDir);
    cpu.setCR(0, cpu.getCR(0) | 0x80000001u);
    cpu.setNativeProtection(true);
    decoder.loadSegment(cpu::CS, 0x08);
    decoder.loadSegment(cpu::DS, 0x10);
    decoder.loadSegment(cpu::ES, 0x10);
    decoder.loadSegment(cpu::SS, 0x10);
    cpu.setReg32(cpu::ESP, 0x9000);
  }
  void runAt(uint32_t addr, const std::vector<uint8_t> &code, int steps) {
    for (size_t i = 0; i < code.size(); ++i)
      mem.write8(addr + static_cast<uint32_t>(i), code[i]);
    cpu.setEIP(addr);
    for (int i = 0; i < steps; ++i)
      decoder.step();
  }
};

TEST_CASE("Lazy flags: #PF restart restores the pending record", "[CPU][LazyFlags][Paging]") {
  PagedMachine m;
  m.cpu.setReg32(cpu::EAX, 5);
  m.cpu.setReg32(cpu::ECX, 9);
  m.cpu.setEFLAGS(0x2);
  // sub eax,ecx (lazy: CF SF); add eax,[0x50010] faults
  m.runAt(0x6000, {0x29, 0xC8, 0x03, 0x05, 0x10, 0x00, 0x05, 0x00}, 2);
  REQUIRE(m.cpu.getEIP() == PagedMachine::kPfHandler);
  REQUIRE(m.cpu.getCR(2) == 0x50010);
  REQUIRE(m.cpu.getReg32(cpu::EAX) == 5u - 9u);
  uint32_t ref = 0x2;
  eagerAlu(5, 5, 9, 32, ref);
  // Frame: error code, EIP, CS, EFLAGS.
  const uint32_t esp = m.cpu.getReg32(cpu::ESP);
  REQUIRE(m.mem.read32(esp + 4) == 0x6002);
  REQUIRE((m.mem.read32(esp + 12) & kArith) == (ref & kArith));
}

// ═══════════════════════════════════════════════════════════════════════
// Predecoded fast path vs. legacy decoder
// ═══════════════════════════════════════════════════════════════════════

namespace {

// A machine for differential runs: code at linear 0x10000, data segment
// at 0x20000, stack at 0x30000; real mode (16-bit code) or flat-limit
// protected mode with those bases (32-bit code), optionally with paging
// identity-mapping the first 4 MiB.
struct DiffMachine {
  cpu::CPU cpu{cpu::CpuModelId::PentiumPro};
  memory::MemoryBus mem;
  hw::IOBus iobus;
  hw::KeyboardController kbd;
  hw::PIT8254 pit;
  hw::PIC8259 pic{true};
  hw::DOS dos{cpu, mem};
  hw::BIOS bios{cpu, mem, kbd, pit, pic};
  cpu::InstructionDecoder decoder{cpu, mem, iobus, bios, dos};
  static constexpr uint32_t kCode = 0x10000, kData = 0x20000, kStack = 0x30000;
  static constexpr uint32_t kGdt = 0x1000, kDir = 0x100000, kTable = 0x101000;

  void desc(uint16_t sel, uint32_t base, uint32_t type) {
    mem.write32(kGdt + sel, 0xFFFFu | ((base & 0xFFFF) << 16));
    mem.write32(kGdt + sel + 4, ((base >> 16) & 0xFF) | (base & 0xFF000000u) |
                                    0x00CF0000u | type);
  }
  DiffMachine(bool fast, bool code32, bool paging) {
    decoder.setFastPathEnabled(fast);
    mem.setA20(true);
    if (!code32) {
      cpu.setSegReg(cpu::CS, kCode >> 4);
      cpu.setSegReg(cpu::DS, kData >> 4);
      cpu.setSegReg(cpu::ES, kData >> 4);
      cpu.setSegReg(cpu::FS, kData >> 4);
      cpu.setSegReg(cpu::GS, kData >> 4);
      cpu.setSegReg(cpu::SS, kStack >> 4);
      decoder.syncSegments();
      return;
    }
    desc(0x08, kCode, 0x9A00);
    desc(0x10, kData, 0x9200);
    desc(0x18, kStack, 0x9200);
    cpu.setGDTR({0x1F, kGdt});
    cpu.setCR(0, cpu.getCR(0) | 1);
    if (paging) {
      for (uint32_t i = 0; i < 1024; ++i)
        mem.write32(kTable + i * 4, (i << 12) | 0x07);
      mem.write32(kDir, kTable | 0x07);
      cpu.setCR(3, kDir);
      cpu.setCR(0, cpu.getCR(0) | 0x80000000u);
    }
    decoder.loadSegment(cpu::CS, 0x08);
    for (auto s : {cpu::DS, cpu::ES, cpu::FS, cpu::GS})
      decoder.loadSegment(s, 0x10);
    decoder.loadSegment(cpu::SS, 0x18);
  }
};

// Random instruction streams over the fast-path opcodes, some legacy ones
// and stores through CS: (self-modifying code).
struct StreamGen {
  std::mt19937 &rng;
  bool code32;
  std::vector<uint8_t> out;
  uint32_t r(uint32_t n) { return static_cast<uint32_t>(rng() % n); }
  void b(uint32_t v) { out.push_back(static_cast<uint8_t>(v)); }
  void imm(int n) {
    for (int i = 0; i < n; ++i)
      b(r(4) == 0 ? 0xFF : r(256));
  }
  // ModRM with a memory or register operand; memory operands stay in the
  // data/stack/code segments (registers hold small values, see below).
  void modrm(bool addr32, int reg = -1) {
    const uint32_t mod = r(4);
    const uint32_t rg = reg >= 0 ? static_cast<uint32_t>(reg) : r(8);
    uint32_t rm = r(8);
    b((mod << 6) | (rg << 3) | rm);
    if (mod == 3)
      return;
    if (addr32) {
      if (rm == 4)
        b((r(4) << 6) | (r(8) << 3) | r(8)); // SIB
      if (mod == 1)
        b(r(256));
      else if (mod == 2 || (mod == 0 && rm == 5))
        imm(2), b(0), b(0);
      else if (mod == 0 && rm == 4 && (out.back() & 7) == 5)
        imm(2), b(0), b(0);
    } else {
      if (mod == 1)
        b(r(256));
      else if (mod == 2 || (mod == 0 && rm == 6))
        imm(2);
    }
  }
  void one() {
    const uint32_t kind = r(43);
    if (kind == 37) { // CALL to an inline subroutine ending in RET / RET imm
      const int rel = code32 ? 4 : 2;
      std::vector<uint8_t> body;
      std::swap(out, body);
      for (uint32_t i = 0, n = r(3); i < n; ++i)
        b(0x40 + r(16));
      if (r(2)) {
        b(0xC3);
      } else {
        b(0xC2);
        b(r(3) * 2);
        b(0);
      }
      std::swap(out, body);
      b(0xEB); // jump over the subroutine
      b(static_cast<uint32_t>(body.size()));
      const size_t sub = out.size();
      out.insert(out.end(), body.begin(), body.end());
      const int32_t disp =
          static_cast<int32_t>(sub) - static_cast<int32_t>(out.size() + 1 + rel);
      b(0xE8);
      for (int i = 0; i < rel; ++i)
        b(static_cast<uint32_t>(disp) >> (8 * i));
      return;
    }
    bool op32 = code32, addr32 = code32;
    switch (r(16)) {
    case 0: b(0x66); op32 = !op32; break;
    case 1: b(0x67); addr32 = !addr32; break;
    case 2: b(0x2E); break; // CS: stores modify the code
    case 3: b(0x36); break;
    case 4: b(0x26); break;
    case 5: b(0x66); b(0x66); break; // the legacy decoder toggles
    default: break;
    }
    const int iw = op32 ? 4 : 2;
    switch (kind) {
    case 0: case 1: case 2: b(r(8) * 8 + r(4)); modrm(addr32); break; // ALU r/m
    case 3: b(r(8) * 8 + 4); imm(1); break;
    case 4: b(r(8) * 8 + 5); imm(iw); break;
    case 5: b(0x40 + r(16)); break;           // INC/DEC
    case 6: b(0x50 + r(8)); break;            // PUSH
    case 7: b(0x58 + r(8)); break;            // POP
    case 8: { const bool i8 = r(2); b(i8 ? 0x6A : 0x68); imm(i8 ? 1 : iw); break; }
    case 9: { const bool i8 = r(2); b(i8 ? 0x6B : 0x69); modrm(addr32); imm(i8 ? 1 : iw); break; }
    case 10: b(0x70 + r(16)); b(r(6)); break; // Jcc short forward
    case 11: { const uint32_t op = 0x80 + r(4); b(op); modrm(addr32); imm(op == 0x81 ? iw : 1); break; }
    case 12: b(0x84 + r(4)); modrm(addr32); break; // TEST / XCHG
    case 13: case 14: case 15: b(0x88 + r(4)); modrm(addr32); break; // MOV
    case 16: b(0x8D); modrm(addr32); break;   // LEA (mod 3: legacy)
    case 17: b(0x90 + r(10)); break;          // NOP, XCHG, CBW, CWD
    case 18: b(0xA0 + r(4)); imm(2); if (addr32) { b(0); b(0); } break;
    case 19: b(0xA8); imm(1); break;
    case 20: b(0xA9); imm(iw); break;
    case 21: b(0xB0 + r(8)); imm(1); break;
    case 22: b(0xB8 + r(8)); imm(iw); break;
    case 23: b(0xC0 + r(2)); modrm(addr32); imm(1); break; // shifts
    case 24: b(0xD0 + r(4)); modrm(addr32); break;
    case 25: { const bool w = r(2); b(0xC6 + w); modrm(addr32, 0); imm(w ? iw : 1); break; }
    case 26: b(0xF5); break;                  // CMC
    case 27: b(0xF8 + r(2)); break;           // CLC/STC
    case 28: { // TEST NOT NEG MUL IMUL (DIV: legacy, may fault)
      const bool w = r(2);
      const int reg = static_cast<int>(r(6));
      b(0xF6 + w);
      modrm(addr32, reg);
      if (reg < 2)
        imm(w ? iw : 1);
      break;
    }
    case 29: b(0xFE + r(2)); modrm(addr32, static_cast<int>(r(2))); break;
    case 30: b(0xFF); modrm(addr32, 6); break; // PUSH r/m
    case 31: b(0x0F); b(0x80 + r(16)); b(r(6)); b(0); if (op32) { b(0); b(0); } break;
    case 32: b(0x0F); b(0x90 + r(16)); modrm(addr32); break;
    case 33: b(0x0F); b(0x40 + r(16)); modrm(addr32); break;
    case 34: b(0x0F); b(0xAF); modrm(addr32); break;
    case 35: b(0x0F); b(0xB6 + r(2) + 8 * r(2)); modrm(addr32); break;
    case 36: // legacy instructions in between
      switch (r(6)) {
      case 0: b(0x9C); break;              // PUSHF
      case 1: b(0x9F); break;              // LAHF
      case 2: b(0x0F); b(0xA3); modrm(addr32); break; // BT
      case 3: b(0xAA + r(6)); break;       // string ops, no REP
      case 4: b(0x27 + 8 * r(4)); break;   // DAA/DAS/AAA/AAS
      default: b(0x0F); b(0xBC + r(2)); modrm(addr32); break; // BSF/BSR
      }
      break;
    case 38: b(0xE9); b(r(5)); b(0); if (op32) { b(0); b(0); } break;
    case 40: case 41: b(0xD8 + r(8)); modrm(addr32); break; // x87, any ModRM
    case 42: b(0x9B); break;                                // FWAIT
    default: b(0xEB); b(r(5)); break;
    }
  }
};

// Registers that keep memory operands near the segments' bases.
uint32_t smallValue(std::mt19937 &rng) {
  switch (rng() % 4) {
  case 0: return rng() % 0x100;
  case 1: return rng() % 0x10000;
  case 2: return 0xFFFF0000u | (rng() % 0x10000);
  default: return rng() % 0x8000;
  }
}

std::string hexString(const std::vector<uint8_t> &v) {
  std::string s;
  char buf[4];
  for (uint8_t x : v) {
    std::snprintf(buf, sizeof buf, "%02X ", x);
    s += buf;
  }
  return s;
}

void runDifferential(bool code32, bool paging, int sequences, uint32_t seed) {
  std::mt19937 rng(seed);
  DiffMachine fast(true, code32, paging), legacy(false, code32, paging);
  for (int seq = 0; seq < sequences; ++seq) {
    StreamGen gen{rng, code32, {}};
    while (gen.out.size() < 160)
      gen.one();
    // Same initial state in both machines.
    std::vector<uint8_t> data(0x400);
    for (auto &x : data)
      x = static_cast<uint8_t>(rng());
    uint32_t regs[8];
    for (auto &x : regs)
      x = smallValue(rng);
    regs[cpu::ESP] = 0x8000 + (rng() % 0x100) * 4;
    const uint32_t flags = 0x2 | (rng() & 0x8D5);
    for (DiffMachine *m : {&fast, &legacy}) {
      for (uint32_t i = 0; i < 0x2000; ++i)
        m->mem.write8(DiffMachine::kCode + i, 0x90);
      for (size_t i = 0; i < gen.out.size(); ++i)
        m->mem.write8(DiffMachine::kCode + static_cast<uint32_t>(i), gen.out[i]);
      for (uint32_t i = 0; i < 0x10000; i += 0x400)
        for (size_t j = 0; j < data.size(); ++j) {
          m->mem.write8(DiffMachine::kData + i + static_cast<uint32_t>(j), data[j]);
          m->mem.write8(DiffMachine::kStack + i + static_cast<uint32_t>(j), data[j]);
        }
      for (int i = 0; i < 8; ++i)
        m->cpu.setReg32(static_cast<uint8_t>(i), regs[i]);
      m->cpu.setEFLAGS(flags);
      m->cpu.setEIP(0);
    }
    for (int step = 0; step < 120;) {
      const uint32_t eip = fast.cpu.getEIP();
      if (eip >= 0x1F00)
        break; // ran off the generated code
      // The fast machine alternates single steps and batches (bursts of
      // fast-path instructions); the legacy one always single-steps.
      const uint64_t k = (seq & 1) ? 1 + rng() % 8 : 1;
      if (k == 1) {
        fast.decoder.step();
        fast.cpu.addCycles(1);
      } else {
        REQUIRE(fast.decoder.run(k) == k);
      }
      for (uint64_t i = 0; i < k; ++i) {
        legacy.decoder.step();
        legacy.cpu.addCycles(1);
      }
      step += static_cast<int>(k);
      bool same = fast.cpu.getEIP() == legacy.cpu.getEIP() &&
                  fast.cpu.getEFLAGS() == legacy.cpu.getEFLAGS() &&
                  fast.cpu.cycles() == legacy.cpu.cycles() &&
                  fast.cpu.getCycles() == legacy.cpu.getCycles() &&
                  fast.cpu.getInstructionStartEIP() ==
                      legacy.cpu.getInstructionStartEIP() &&
                  fast.cpu.getInstructionStartCS() ==
                      legacy.cpu.getInstructionStartCS() &&
                  fast.cpu.getPrevInstructionCS() ==
                      legacy.cpu.getPrevInstructionCS();
      for (int i = 0; i < 8; ++i)
        same = same && fast.cpu.getReg32(static_cast<uint8_t>(i)) ==
                           legacy.cpu.getReg32(static_cast<uint8_t>(i));
      for (int i = 0; i < 6; ++i)
        same = same && fast.cpu.getSegReg(static_cast<uint8_t>(i)) ==
                           legacy.cpu.getSegReg(static_cast<uint8_t>(i));
      // x87 state, including the last instruction/operand pointers.
      const auto &ff = fast.cpu.fpu(), &lf = legacy.cpu.fpu();
      same = same && ff.statusWord() == lf.statusWord() &&
             ff.controlWord() == lf.controlWord() && ff.tagWord() == lf.tagWord() &&
             ff.fip == lf.fip && ff.fdp == lf.fdp && ff.fop == lf.fop &&
             ff.fcs == lf.fcs && ff.fds == lf.fds;
      for (uint8_t i = 0; i < 8; ++i) {
        const auto a = fast.cpu.getFPURegister80(i), c = legacy.cpu.getFPURegister80(i);
        same = same && a.mant == c.mant && a.signExp == c.signExp;
      }
      if (same && (step % 16 < static_cast<int>(k) || step >= 119)) {
        for (uint32_t a = DiffMachine::kCode; a < DiffMachine::kStack + 0x10000; a += 4)
          if (fast.mem.read32(a) != legacy.mem.read32(a)) {
            same = false;
            break;
          }
      }
      if (!same) {
        std::vector<uint8_t> at;
        for (uint32_t i = 0; i < 12; ++i)
          at.push_back(legacy.mem.read8(DiffMachine::kCode + eip + i));
        std::fprintf(stderr,
                     "fast/legacy mismatch: %s code, paging %d, seed %u, sequence %d, step %d\n"
                     "  at EIP %04X: %s\n  fast   EIP=%08X FL=%08X EAX=%08X ECX=%08X EDX=%08X EBX=%08X ESP=%08X cyc=%llu\n"
                     "  legacy EIP=%08X FL=%08X EAX=%08X ECX=%08X EDX=%08X EBX=%08X ESP=%08X cyc=%llu\n",
                     code32 ? "32-bit" : "16-bit", paging ? 1 : 0, seed, seq, step,
                     eip, hexString(at).c_str(), fast.cpu.getEIP(), fast.cpu.getEFLAGS(),
                     fast.cpu.getReg32(0), fast.cpu.getReg32(1), fast.cpu.getReg32(2),
                     fast.cpu.getReg32(3), fast.cpu.getReg32(4),
                     static_cast<unsigned long long>(fast.cpu.cycles()),
                     legacy.cpu.getEIP(), legacy.cpu.getEFLAGS(), legacy.cpu.getReg32(0),
                     legacy.cpu.getReg32(1), legacy.cpu.getReg32(2), legacy.cpu.getReg32(3),
                     legacy.cpu.getReg32(4),
                     static_cast<unsigned long long>(legacy.cpu.cycles()));
        REQUIRE(same);
      }
    }
  }
}

int fuzzSequences(int dflt) {
  const char *env = std::getenv("FASTPATH_FUZZ_SEQUENCES");
  return env ? std::atoi(env) : dflt;
}

} // namespace

TEST_CASE("Fast path: random 16-bit streams match the legacy decoder",
          "[CPU][FastPath]") {
  runDifferential(false, false, fuzzSequences(300), 1);
}

TEST_CASE("Fast path: random 32-bit streams match the legacy decoder",
          "[CPU][FastPath]") {
  runDifferential(true, false, fuzzSequences(300), 2);
}

TEST_CASE("Fast path: random 32-bit streams with paging match the legacy decoder",
          "[CPU][FastPath][Paging]") {
  runDifferential(true, true, fuzzSequences(150), 3);
}

TEST_CASE("Fast path: self-modifying code is re-decoded", "[CPU][FastPath][SMC]") {
  for (bool fast : {true, false}) {
    Machine m;
    m.decoder.setFastPathEnabled(fast);
    SECTION("an instruction changed after it was cached") {
      // mov ax,0x1111 ; executed (and cached), then patched to mov ax,0x2222
      m.run({0xB8, 0x11, 0x11}, 1);
      REQUIRE(m.cpu.getReg16(cpu::AX) == 0x1111);
      m.mem.write8(Machine::kCode + 1, 0x22);
      m.mem.write8(Machine::kCode + 2, 0x22);
      m.cpu.setEIP(0);
      m.decoder.step();
      REQUIRE(m.cpu.getReg16(cpu::AX) == 0x2222);
      // A different instruction of a different length at the same place.
      m.load({0x40}); // inc ax
      m.cpu.setEIP(0);
      m.decoder.step();
      REQUIRE(m.cpu.getReg16(cpu::AX) == 0x2223);
      REQUIRE(m.cpu.getEIP() == 1);
    }
    SECTION("an instruction patches the next one") {
      // loop twice over: mov byte cs:[next+1],0x05 ; next: mov al,0x01
      for (int pass = 0; pass < 2; ++pass) {
        m.run({0x2E, 0xC6, 0x06, 0x07, 0x00, 0x05 + pass, // mov byte cs:[7],imm
               0xB0, 0x01},                              // mov al,1
              2);
        REQUIRE(m.cpu.getReg8(cpu::AL) == 0x05 + pass);
      }
    }
    SECTION("REP MOVSB copies new code over cached code") {
      m.run({0xB0, 0x01, 0x90, 0x90}, 1); // mov al,1 (cached)
      REQUIRE(m.cpu.getReg8(cpu::AL) == 1);
      // Source: "mov al,7" at DS:0x100; copy it over CS:0 with ES = CS.
      m.mem.write8(0x20100, 0xB0);
      m.mem.write8(0x20101, 0x07);
      m.cpu.setSegReg(cpu::ES, Machine::kCode >> 4);
      m.cpu.setReg32(cpu::ESI, 0x100);
      m.cpu.setReg32(cpu::EDI, 0x0);
      m.cpu.setReg32(cpu::ECX, 2);
      m.decoder.syncSegments();
      m.load({0xF3, 0xA4}, 0x40);
      m.cpu.setEIP(0x40);
      m.decoder.step();
      REQUIRE(m.cpu.getReg16(cpu::CX) == 0);
      m.cpu.setEIP(0);
      m.decoder.step();
      REQUIRE(m.cpu.getReg8(cpu::AL) == 7);
    }
  }
}

TEST_CASE("Fast path: IP wrap-around and the 16-byte window", "[CPU][FastPath]") {
  Machine fast, legacy;
  legacy.decoder.setFastPathEnabled(false);
  for (Machine *m : {&fast, &legacy}) {
    // mov ax,imm16 at IP FFFE: IP wraps to 0001 after it.  (Which byte the
    // immediate's high half comes from is the legacy decoder's business;
    // the fast path must not handle IP-wrapping instructions at all.)
    m->mem.write8(Machine::kCode + 0xFFFE, 0xB8);
    m->mem.write8(Machine::kCode + 0xFFFF, 0x34);
    m->mem.write8(Machine::kCode + 0x0000, 0x12);
    m->mem.write8(Machine::kCode + 0x10000, 0x56);
    for (int pass = 0; pass < 2; ++pass) {
      m->cpu.setEIP(0xFFFE);
      m->decoder.step();
      REQUIRE(m->cpu.getEIP() == 1);
    }
    // Instructions in the last 15 bytes of a page use the legacy fetch.
    m->load({0x66, 0xB8, 0x78, 0x56, 0x34, 0x12}, 0x0FFC);
    m->cpu.setEIP(0x0FFC);
    m->decoder.step();
    REQUIRE(m->cpu.getReg32(cpu::EAX) == 0x12345678u);
    REQUIRE(m->cpu.getEIP() == 0x1002);
  }
  fast.mem.write32(Machine::kCode + 0x0FFC, 0);
  REQUIRE(fast.cpu.cycles() == legacy.cpu.cycles());
  fast.cpu.setEIP(0xFFFE);
  legacy.cpu.setEIP(0xFFFE);
  fast.decoder.step();
  legacy.decoder.step();
  REQUIRE(fast.cpu.getReg32(cpu::EAX) == legacy.cpu.getReg32(cpu::EAX));
}

TEST_CASE("Fast path: code pages follow the TLB (INVLPG, CR3, page crossing)",
          "[CPU][FastPath][Paging]") {
  for (bool fast : {true, false}) {
    PagedMachine m;
    m.decoder.setFastPathEnabled(fast);
    constexpr uint32_t kPte = PagedMachine::kTable + (0x60000 >> 12) * 4;
    // Linear 0x60000 -> physical 0x60000 ("mov eax,1") or 0x70000 ("mov eax,2").
    const std::vector<uint8_t> one = {0xB8, 1, 0, 0, 0}, two = {0xB8, 2, 0, 0, 0};
    for (size_t i = 0; i < one.size(); ++i) {
      m.mem.write8(0x60000 + static_cast<uint32_t>(i), one[i]);
      m.mem.write8(0x70000 + static_cast<uint32_t>(i), two[i]);
    }
    auto exec = [&]() {
      m.cpu.setEIP(0x60000);
      m.decoder.step();
      return m.cpu.getReg32(cpu::EAX);
    };
    REQUIRE(exec() == 1);
    m.mem.write32(kPte, 0x70000 | 0x07); // remap behind the TLB's back
    REQUIRE(exec() == 1);                // stale TLB entry, like the CPU
    m.mem.invalidatePage(0x60000);       // INVLPG
    REQUIRE(exec() == 2);
    m.mem.write32(kPte, 0x60000 | 0x07);
    m.cpu.setCR(3, PagedMachine::kDir); // CR3 reload flushes the TLB
    REQUIRE(exec() == 1);

    // An instruction straddling two pages whose frames are not adjacent:
    // linear 0x61FFE.. -> 0x61000 page maps to 0x80000, 0x62000 to 0x90000.
    m.mem.write32(PagedMachine::kTable + (0x61000 >> 12) * 4, 0x80000 | 0x07);
    m.mem.write32(PagedMachine::kTable + (0x62000 >> 12) * 4, 0x90000 | 0x07);
    m.cpu.setCR(3, PagedMachine::kDir);
    m.mem.write8(0x80FFE, 0xB8);
    m.mem.write8(0x80FFF, 0x44);
    m.mem.write8(0x90000, 0x33);
    m.mem.write8(0x90001, 0x22);
    m.mem.write8(0x90002, 0x11);
    m.cpu.setEIP(0x61FFE);
    m.decoder.step();
    REQUIRE(m.cpu.getReg32(cpu::EAX) == 0x11223344u);
    REQUIRE(m.cpu.getEIP() == 0x62003);
  }
}

TEST_CASE("Fast path: the batch runner counts like step() + addCycles(1)",
          "[CPU][FastPath]") {
  Machine a, b;
  a.decoder.setFastPathEnabled(true);
  b.decoder.setFastPathEnabled(false);
  // inc ax ; jmp short -3 (an endless loop)
  for (Machine *m : {&a, &b})
    m->load({0x40, 0xEB, 0xFD});
  REQUIRE(a.decoder.run(1000) == 1000);
  for (int i = 0; i < 1000; ++i) {
    b.decoder.step();
    b.cpu.addCycles(1);
  }
  REQUIRE(a.cpu.getReg16(cpu::AX) == 500);
  REQUIRE(a.cpu.getReg16(cpu::AX) == b.cpu.getReg16(cpu::AX));
  REQUIRE(a.cpu.getCycles() == b.cpu.getCycles());
  REQUIRE(a.cpu.cycles() == b.cpu.cycles());
  REQUIRE(a.cpu.getEIP() == b.cpu.getEIP());
}
