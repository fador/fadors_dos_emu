// x87 FPU model and back-end tests with known bit patterns (values from the
// Intel SDM and checked against a real x87).  Back-end level checks run on
// both the portable soft back end and the active one (the host x87 fast
// path on GCC/Clang x86 builds), so a soft-only build tests the same
// expectations.
#include "test_framework.hpp"
#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "cpu/x87/FPU.hpp"
#include "cpu/x87/U128.hpp"
#include "cpu/x87/X87Math.hpp"
#include "hw/BIOS.hpp"
#include "hw/DOS.hpp"
#include "hw/IOBus.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIC8259.hpp"
#include "hw/PIT8254.hpp"
#include "memory/MemoryBus.hpp"

#include <algorithm>
#include <random>

using namespace fador;
using namespace fador::cpu::x87;

namespace {

constexpr Float80 F(uint16_t se, uint64_t m) { return Float80(m, se); }

constexpr uint16_t CW(int pc, int rc, uint16_t masks = 0x3F) {
  return static_cast<uint16_t>(0x0040 | (pc << 8) | (rc << 10) | masks);
}
constexpr int RN = 0, RD = 1, RU = 2, RZ = 3;
constexpr int PC24 = 0, PC53 = 2, PC64 = 3;

const Float80 kThree = F(0x4000, 0xC000000000000000ull);
const Float80 kTen = F(0x4002, 0xA000000000000000ull);
const Float80 kTwo = F(0x4000, 0x8000000000000000ull);
const Float80 kHalf = F(0x3FFE, 0x8000000000000000ull);

struct Result {
  Float80 v;
  uint16_t sw;
};

// Runs `fn` with both back ends and requires identical results.
template <typename Fn> Result both(Fn fn) {
  uint16_t s1 = 0, s2 = 0;
  const Float80 a = fn(true, s1);
  const Float80 b = fn(false, s2);
  REQUIRE(a == b);
  REQUIRE((s1 & (SW_EXC | SW_C1)) == (s2 & (SW_EXC | SW_C1)));
  return {a, s1};
}

Result arith(Arith op, Float80 a, Float80 b, uint16_t cw) {
  return both([&](bool soft, uint16_t &sw) {
    return soft ? soft::arith(op, a, b, cw, sw) : active::arith(op, a, b, cw, sw);
  });
}

Result stack1(uint8_t modrm, Float80 a, uint16_t cw, Float80 b = kPosZero) {
  return both([&](bool soft, uint16_t &sw) {
    Float80 in[2] = {a, b}, out[3];
    const int n = stackOpInputs(modrm);
    if (soft)
      soft::stackOp(modrm, n, in, out, cw, sw);
    else
      active::stackOp(modrm, n, in, out, cw, sw);
    return out[0];
  });
}

} // namespace

TEST_CASE("x87: rounding modes and precision control (1/3)", "[x87]") {
  const uint16_t pe = SW_PE;
  SECTION("64-bit precision, all rounding modes") {
    auto r = arith(Arith::Div, kOne, kThree, CW(PC64, RN));
    REQUIRE(r.v == F(0x3FFD, 0xAAAAAAAAAAAAAAABull));
    REQUIRE((r.sw & pe) && (r.sw & SW_C1)); // rounded up
    REQUIRE(arith(Arith::Div, kOne, kThree, CW(PC64, RU)).v == F(0x3FFD, 0xAAAAAAAAAAAAAAABull));
    r = arith(Arith::Div, kOne, kThree, CW(PC64, RD));
    REQUIRE(r.v == F(0x3FFD, 0xAAAAAAAAAAAAAAAAull));
    REQUIRE(!(r.sw & SW_C1));
    REQUIRE(arith(Arith::Div, kOne, kThree, CW(PC64, RZ)).v == F(0x3FFD, 0xAAAAAAAAAAAAAAAAull));
    // Negative operand: RD now rounds away from zero
    const Float80 m1 = F(0xBFFF, 0x8000000000000000ull);
    REQUIRE(arith(Arith::Div, m1, kThree, CW(PC64, RD)).v == F(0xBFFD, 0xAAAAAAAAAAAAAAABull));
    REQUIRE(arith(Arith::Div, m1, kThree, CW(PC64, RU)).v == F(0xBFFD, 0xAAAAAAAAAAAAAAAAull));
  }
  SECTION("53- and 24-bit precision control") {
    REQUIRE(arith(Arith::Div, kOne, kThree, CW(PC53, RN)).v == F(0x3FFD, 0xAAAAAAAAAAAAA800ull));
    REQUIRE(arith(Arith::Div, kOne, kThree, CW(PC53, RU)).v == F(0x3FFD, 0xAAAAAAAAAAAAB000ull));
    REQUIRE(arith(Arith::Div, kOne, kThree, CW(PC24, RN)).v == F(0x3FFD, 0xAAAAAB0000000000ull));
    REQUIRE(arith(Arith::Div, kOne, kThree, CW(PC24, RZ)).v == F(0x3FFD, 0xAAAAAA0000000000ull));
  }
  SECTION("exact results raise nothing") {
    auto r = arith(Arith::Add, kOne, kOne, CW(PC64, RN));
    REQUIRE(r.v == kTwo);
    REQUIRE((r.sw & (SW_EXC | SW_C1)) == 0);
    // x - x is +0, or -0 when rounding down
    REQUIRE(arith(Arith::Sub, kThree, kThree, CW(PC64, RN)).v == kPosZero);
    REQUIRE(arith(Arith::Sub, kThree, kThree, CW(PC64, RD)).v == kNegZero);
  }
  SECTION("square root of 2") {
    REQUIRE(stack1(0xFA, kTwo, CW(PC64, RN)).v == F(0x3FFF, 0xB504F333F9DE6484ull));
    REQUIRE(stack1(0xFA, kTwo, CW(PC64, RU)).v == F(0x3FFF, 0xB504F333F9DE6485ull));
  }
}

TEST_CASE("x87: special values, NaN propagation and invalid operations", "[x87]") {
  const uint16_t cw = CW(PC64, RN);
  SECTION("invalid operations give the real indefinite") {
    auto r = arith(Arith::Div, kPosZero, kPosZero, cw);
    REQUIRE(r.v == kIndefinite);
    REQUIRE(r.sw & SW_IE);
    r = arith(Arith::Sub, kPosInf, kPosInf, cw);
    REQUIRE(r.v == kIndefinite);
    r = stack1(0xFA, F(0xBFFF, 0x8000000000000000ull), cw); // sqrt(-1)
    REQUIRE(r.v == kIndefinite);
    REQUIRE(r.sw & SW_IE);
    // Unsupported format (unnormal): IE
    r = arith(Arith::Add, F(0x3FFF, 0x4000000000000000ull), kOne, cw);
    REQUIRE(r.v == kIndefinite);
    REQUIRE(r.sw & SW_IE);
  }
  SECTION("division by zero") {
    auto r = arith(Arith::Div, kOne, kNegZero, cw);
    REQUIRE(r.v == kNegInf);
    REQUIRE((r.sw & SW_EXC) == SW_ZE);
  }
  SECTION("SNaN operand is quieted with IE, two QNaNs keep the larger significand") {
    auto r = arith(Arith::Add, F(0x7FFF, 0xA000000000000000ull), kOne, cw);
    REQUIRE(r.v == F(0x7FFF, 0xE000000000000000ull));
    REQUIRE(r.sw & SW_IE);
    r = arith(Arith::Mul, F(0x7FFF, 0xC000000000000001ull), F(0xFFFF, 0xD000000000000000ull), cw);
    REQUIRE(r.v == F(0xFFFF, 0xD000000000000000ull));
    REQUIRE(!(r.sw & SW_IE));
    // QNaN wins over SNaN
    r = arith(Arith::Add, F(0x7FFF, 0xB000000000000000ull), F(0x7FFF, 0xC000000000000001ull), cw);
    REQUIRE(r.v == F(0x7FFF, 0xC000000000000001ull));
    REQUIRE(r.sw & SW_IE);
  }
  SECTION("denormals: operand DE, masked underflow only when inexact") {
    // pseudo-denormal (exp 0, J=1) + 0 is normalised, DE
    auto r = arith(Arith::Add, F(0x0000, 0x8000000000000000ull), kPosZero, cw);
    REQUIRE(r.v == F(0x0001, 0x8000000000000000ull));
    REQUIRE(r.sw & SW_DE);
    // exact denormal result: no UE
    r = arith(Arith::Mul, F(0x0001, 0x8000000000000000ull), kHalf, cw);
    REQUIRE(r.v == F(0x0000, 0x4000000000000000ull));
    REQUIRE((r.sw & (SW_UE | SW_PE)) == 0);
    // inexact denormal result: UE + PE (tie rounds to even)
    r = arith(Arith::Mul, F(0x0001, 0x8000000000000001ull), kHalf, cw);
    REQUIRE(r.v == F(0x0000, 0x4000000000000000ull));
    REQUIRE((r.sw & (SW_UE | SW_PE)) == (SW_UE | SW_PE));
  }
  SECTION("overflow: masked to infinity, unmasked rebiased by 24576") {
    const Float80 maxF = F(0x7FFE, 0xFFFFFFFFFFFFFFFFull);
    auto r = arith(Arith::Mul, maxF, kTwo, cw);
    REQUIRE(r.v == kPosInf);
    REQUIRE((r.sw & (SW_OE | SW_PE | SW_C1)) == (SW_OE | SW_PE | SW_C1));
    r = arith(Arith::Mul, maxF, kTwo, CW(PC64, RZ)); // chop: largest finite
    REQUIRE(r.v == maxF);
    r = arith(Arith::Mul, maxF, kTwo, CW(PC64, RN, 0x37)); // OE unmasked
    REQUIRE(r.v == F(0x7FFF - 24576, 0xFFFFFFFFFFFFFFFFull));
    REQUIRE(r.sw & SW_OE);
  }
}

TEST_CASE("x87: conversions to integers, binary32/64 and BCD", "[x87]") {
  auto fist = [](Float80 v, int bits, uint16_t cw, uint16_t &sw) {
    uint16_t s2 = 0;
    const int64_t a = soft::storeInt(v, bits, cw, sw);
    const int64_t b = active::storeInt(v, bits, cw, s2);
    REQUIRE(a == b);
    REQUIRE((sw & (SW_EXC | SW_C1)) == (s2 & (SW_EXC | SW_C1)));
    return a;
  };
  const Float80 two5 = F(0x4000, 0xA000000000000000ull);  // 2.5
  const Float80 three5 = F(0x4000, 0xE000000000000000ull); // 3.5
  const Float80 m2_5 = F(0xC000, 0xA000000000000000ull);  // -2.5
  uint16_t sw = 0;
  REQUIRE(fist(two5, 16, CW(PC64, RN), sw) == 2);
  REQUIRE((sw & SW_PE) && !(sw & SW_C1));
  sw = 0;
  REQUIRE(fist(three5, 32, CW(PC64, RN), sw) == 4);
  REQUIRE(sw & SW_C1);
  sw = 0;
  REQUIRE(fist(m2_5, 64, CW(PC64, RD), sw) == -3);
  sw = 0;
  REQUIRE(fist(m2_5, 64, CW(PC64, RZ), sw) == -2);
  sw = 0;
  REQUIRE(fist(F(0x400E, 0x9C40000000000000ull), 16, CW(PC64, RN), sw) == -32768); // 40000
  REQUIRE(sw & SW_IE);

  // FST m32 / m64 of 1/3 (rounded to nearest)
  const Float80 third = F(0x3FFD, 0xAAAAAAAAAAAAAAABull);
  uint16_t s1 = 0, s2 = 0;
  REQUIRE(soft::storeF32(third, CW(PC64, RN), s1) == 0x3EAAAAABu);
  REQUIRE(active::storeF32(third, CW(PC64, RN), s2) == 0x3EAAAAABu);
  REQUIRE(soft::storeF64(third, CW(PC64, RN), s1) == 0x3FD5555555555555ull);
  REQUIRE(active::storeF64(third, CW(PC64, RN), s2) == 0x3FD5555555555555ull);
  // round trip through binary64 is exact
  REQUIRE(fromDouble(0.1) == F(0x3FFB, 0xCCCCCCCCCCCCD000ull));
  REQUIRE(toDouble(F(0x3FFB, 0xCCCCCCCCCCCCD000ull)) == 0.1);

  // Packed BCD: 123456789012 and its indefinite
  const uint8_t bcd[10] = {0x12, 0x90, 0x78, 0x56, 0x34, 0x12, 0, 0, 0, 0x80};
  REQUIRE(soft::loadBCD(bcd) == F(0xC023, 0xE5F4C8D0A0000000ull));
  REQUIRE(active::loadBCD(bcd) == F(0xC023, 0xE5F4C8D0A0000000ull));
  uint8_t out[10];
  sw = 0;
  soft::storeBCD(F(0xC023, 0xE5F4C8D0A0000000ull), out, CW(PC64, RN), sw);
  REQUIRE(std::equal(out, out + 10, bcd));
  sw = 0;
  soft::storeBCD(kPosInf, out, CW(PC64, RN), sw);
  const uint8_t indef[10] = {0, 0, 0, 0, 0, 0, 0, 0xC0, 0xFF, 0xFF};
  REQUIRE(std::equal(out, out + 10, indef));
  REQUIRE(sw & SW_IE);
}

TEST_CASE("x87: FPREM/FPREM1, FRNDINT, FXTRACT, FSCALE, constants", "[x87]") {
  const uint16_t cw = CW(PC64, RN);
  SECTION("FPREM quotient bits and FPREM1 rounding") {
    auto r = stack1(0xF8, kTen, cw, kThree); // 10 mod 3 = 1, q = 3
    REQUIRE(r.v == kOne);
    REQUIRE((r.sw & SW_CC) == (SW_C3 | SW_C1));
    const Float80 eleven = F(0x4002, 0xB000000000000000ull);
    r = stack1(0xF8, eleven, cw, kThree); // 11 mod 3 = 2, q = 3
    REQUIRE(r.v == kTwo);
    r = stack1(0xF5, eleven, cw, kThree); // IEEE: 11 - 4*3 = -1, q = 4
    REQUIRE(r.v == F(0xBFFF, 0x8000000000000000ull));
    REQUIRE((r.sw & SW_CC) == SW_C0);
    r = stack1(0xF8, F(0x4063, 0x8000000000000000ull), cw, kThree); // 2^100: partial
    REQUIRE(r.sw & SW_C2);
  }
  SECTION("FRNDINT honours RC") {
    const Float80 two5 = F(0x4000, 0xA000000000000000ull);
    REQUIRE(stack1(0xFC, two5, CW(PC64, RN)).v == kTwo);
    REQUIRE(stack1(0xFC, two5, CW(PC64, RU)).v == kThree);
    REQUIRE(stack1(0xFC, two5, CW(PC64, RZ)).v == kTwo);
    REQUIRE(stack1(0xFC, F(0xC000, 0xA000000000000000ull), CW(PC64, RD)).v ==
            F(0xC000, 0xC000000000000000ull));
    // precision control does not apply to FRNDINT
    REQUIRE(stack1(0xFC, F(0x403E, 0xFFFFFFFFFFFFFFFFull), CW(PC24, RN)).v ==
            F(0x403E, 0xFFFFFFFFFFFFFFFFull));
  }
  SECTION("FXTRACT and FSCALE") {
    Float80 in[2] = {F(0x4002, 0x8000000000000000ull), kPosZero}, out[3];
    uint16_t sw = 0;
    REQUIRE(active::stackOp(0xF4, 1, in, out, cw, sw) == 2);
    REQUIRE(out[0] == kOne);                               // significand
    REQUIRE(out[1] == F(0x4000, 0xC000000000000000ull));   // exponent 3
    REQUIRE(stack1(0xFD, F(0x3FFF, 0xC000000000000000ull), cw, kThree).v ==
            F(0x4002, 0xC000000000000000ull)); // 1.5 * 2^3 = 12
  }
  SECTION("FLDPI and friends round the 66-bit internal constants") {
    REQUIRE(soft::loadConst(Const::Pi, CW(PC64, RN)) == F(0x4000, 0xC90FDAA22168C235ull));
    REQUIRE(soft::loadConst(Const::Pi, CW(PC64, RZ)) == F(0x4000, 0xC90FDAA22168C234ull));
    REQUIRE(soft::loadConst(Const::L2T, CW(PC64, RN)) == F(0x4000, 0xD49A784BCD1B8AFEull));
    REQUIRE(soft::loadConst(Const::L2T, CW(PC64, RU)) == F(0x4000, 0xD49A784BCD1B8AFFull));
    REQUIRE(soft::loadConst(Const::LN2, CW(PC64, RN)) == F(0x3FFE, 0xB17217F7D1CF79ACull));
  }
  SECTION("trig range rule: |x| >= 2^63 sets C2 and leaves the operand") {
    Float80 in[2] = {F(0x403F, 0x8000000000000000ull), kPosZero}, out[3];
    uint16_t sw = 0;
    REQUIRE(soft::stackOp(0xFE, 1, in, out, cw, sw) == 1);
    REQUIRE(out[0] == in[0]);
    REQUIRE(sw & SW_C2);
    sw = 0;
    REQUIRE(active::stackOp(0xF2, 1, in, out, cw, sw) == 1);
    REQUIRE(sw & SW_C2);
  }
}

TEST_CASE("x87: FPU stack faults, tags and unmasked responses", "[x87]") {
  SECTION("tag word is computed from the register contents") {
    FPU fpu;
    fpu.fldValue(kPosInf);  // special
    fpu.fldValue(kPosZero); // zero
    fpu.fldValue(kOne);     // valid
    // TOP = 5: phys 5 = 1.0 (00), phys 6 = 0 (01), phys 7 = inf (10),
    // phys 0..4 empty (11)
    REQUIRE(fpu.tagWord() == 0x93FF);
    // Only empty / non-empty is kept; phys 0..4 hold +0 -> zero (01)
    fpu.setTagWord(0x0000);
    REQUIRE(fpu.tagWord() == 0x9155);
  }
  SECTION("masked stack overflow pushes the indefinite with C1=1") {
    FPU fpu;
    for (int k = 0; k < 8; ++k)
      fpu.fldValue(kOne);
    fpu.fldConst(Const::Pi);
    REQUIRE(fpu.st(0) == kIndefinite);
    REQUIRE((fpu.statusWord() & (SW_IE | SW_SF | SW_C1)) == (SW_IE | SW_SF | SW_C1));
    REQUIRE(fpu.top() == 7);
  }
  SECTION("unmasked stack underflow changes nothing but the flags") {
    FPU fpu;
    fpu.setControlWord(0x037E); // IE unmasked
    fpu.fchs();
    REQUIRE((fpu.statusWord() & (SW_IE | SW_SF | SW_ES | SW_B)) ==
            (SW_IE | SW_SF | SW_ES | SW_B));
    REQUIRE(fpu.isEmpty(0));
    REQUIRE(fpu.errorPending());
    fpu.fnclex();
    REQUIRE(!fpu.errorPending());
  }
  SECTION("unmasked divide by zero leaves the destination") {
    FPU fpu;
    fpu.setControlWord(0x037B); // ZE unmasked
    fpu.fldValue(kPosZero);
    fpu.fldValue(kThree);
    fpu.arithReg(Arith::Div, 0, 1, false);
    REQUIRE(fpu.st(0) == kThree);
    REQUIRE(fpu.statusWord() & SW_ZE);
    REQUIRE(fpu.errorPending());
  }
  SECTION("FLDCW unmasking a sticky flag makes the error pending") {
    FPU fpu;
    fpu.fldValue(kOne);
    fpu.fldValue(kThree);
    fpu.arithReg(Arith::Div, 0, 1, false); // inexact? 3/1 is exact
    fpu.arithReg(Arith::Div, 1, 0, false); // 1/3: PE
    REQUIRE(!fpu.errorPending());
    fpu.setControlWord(0x035F); // PE unmasked
    REQUIRE(fpu.errorPending());
  }
  SECTION("FXAM classes") {
    FPU fpu;
    auto cls = [&](Float80 v) {
      fpu.reset();
      fpu.fldValue(v);
      fpu.fxam();
      return static_cast<uint16_t>(fpu.statusWord() & SW_CC);
    };
    REQUIRE(cls(kOne) == SW_C2);
    REQUIRE(cls(kNegZero) == (SW_C3 | SW_C1));
    REQUIRE(cls(kPosInf) == (SW_C2 | SW_C0));
    REQUIRE(cls(kIndefinite) == (SW_C0 | SW_C1));
    REQUIRE(cls(F(0x0000, 0x8000000000000000ull)) == (SW_C3 | SW_C2)); // pseudo-denormal
    REQUIRE(cls(F(0x3FFF, 0x4000000000000000ull)) == 0);               // unnormal
    fpu.reset();
    fpu.fxam();
    REQUIRE((fpu.statusWord() & (SW_C3 | SW_C2 | SW_C0)) == (SW_C3 | SW_C0)); // empty
  }
}

TEST_CASE("x87: FNSAVE/FRSTOR keep all 64 significand bits", "[x87]") {
  cpu::CPU cpu;
  memory::MemoryBus mem;
  hw::IOBus iobus;
  hw::KeyboardController kbd;
  hw::PIT8254 pit;
  hw::DOS dos(cpu, mem);
  hw::PIC8259 pic(true);
  hw::BIOS bios(cpu, mem, kbd, pit, pic);
  bios.initialize();
  dos.initialize();
  cpu::InstructionDecoder decoder(cpu, mem, iobus, bios, dos);
  cpu.setSegReg(cpu::SegRegIndex::CS, 0);
  cpu.setSegReg(cpu::SegRegIndex::DS, 0);
  cpu.setEIP(0x100);

  const Float80 third = F(0x3FFD, 0xAAAAAAAAAAAAAAABull);
  const Float80 odd = F(0xC123, 0x8000000000000001ull);
  cpu.fpu().fldValue(odd);
  cpu.fpu().fldValue(third);
  mem.write8(0x100, 0xDD); mem.write8(0x101, 0x36); mem.write16(0x102, 0x3000); // FNSAVE [3000]
  mem.write8(0x104, 0xDD); mem.write8(0x105, 0x26); mem.write16(0x106, 0x3000); // FRSTOR [3000]
  mem.write8(0x108, 0xDB); mem.write8(0x109, 0x3E); mem.write16(0x10A, 0x3100); // FSTP m80 [3100]
  decoder.step();
  REQUIRE(cpu.isFPURegisterEmpty(0));
  // register area at +14: ST0 = 1/3 exactly
  REQUIRE(mem.read32(0x300E) == 0xAAAAAAABu);
  REQUIRE(mem.read16(0x3016) == 0x3FFD);
  decoder.step();
  REQUIRE(cpu.getFPURegister80(0) == third);
  REQUIRE(cpu.getFPURegister80(1) == odd);
  decoder.step();
  REQUIRE(mem.read32(0x3100) == 0xAAAAAAABu);
  REQUIRE(mem.read32(0x3104) == 0xAAAAAAAAu);
  REQUIRE(mem.read16(0x3108) == 0x3FFD);
  REQUIRE(cpu.getFPURegister80(0) == odd);
}

TEST_CASE("x87: portable 128-bit helpers match the native ones", "[x87]") {
  std::mt19937_64 rng(7);
  for (int k = 0; k < 20000; ++k) {
    const uint64_t a = rng(), b = rng() | 1;
    const U128 p = mul64x64Portable(a, b);
    const U128 q = mul64x64(a, b);
    REQUIRE(p == q);
    const uint64_t hi = rng() % b;
    const uint64_t lo = rng();
    uint64_t r1 = 0, r2 = 0;
    REQUIRE(div128by64Portable(hi, lo, b, r1) == div128by64(hi, lo, b, r2));
    REQUIRE(r1 == r2);
  }
}
