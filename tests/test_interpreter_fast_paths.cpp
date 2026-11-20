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
