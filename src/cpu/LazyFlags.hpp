#pragma once
#include <bit>
#include <cstdint>

namespace fador::cpu {

// Deferred arithmetic flags of the last ALU instruction.
//
// ADD/ADC/SUB/SBB/CMP/AND/OR/XOR/TEST/INC/DEC/NEG record their operands and
// result here instead of computing OF SF ZF AF PF CF; CPU::getEFLAGS() folds
// them into EFLAGS on demand, and CPU::testCondition() evaluates Jcc/SETcc
// conditions straight from the record.  The functions below define the
// flags exactly as the former eager InstructionDecoder::aluOp() did,
// including the choice for architecturally undefined bits (AF = 0 after the
// logical operations).
//
// All values are masked to the operand size (msb = 0x80, 0x8000 or
// 0x80000000).  ADC with CF=0 is recorded as Add and SBB with CF=0 as Sub
// (identical flags); with a carry in they use AddC / SubB.
struct LazyFlags {
  enum Kind : uint8_t {
    None = 0, // EFLAGS holds the architectural value
    Add,      // res = dst + src
    AddC,     // res = dst + src + 1
    Sub,      // res = dst - src (also CMP, NEG as 0 - src)
    SubB,     // res = dst - src - 1
    Logic,    // AND/OR/XOR/TEST: CF = OF = AF = 0
    Inc,      // res = dst + 1, CF unchanged (kept in EFLAGS)
    Dec,      // res = dst - 1, CF unchanged (kept in EFLAGS)
  };
  static constexpr uint32_t kCF = 0x001, kPF = 0x004, kAF = 0x010,
                            kZF = 0x040, kSF = 0x080, kOF = 0x800;
  static constexpr uint32_t kArithMask = kCF | kPF | kAF | kZF | kSF | kOF;

  Kind kind = None;
  uint32_t msb = 0;
  uint32_t dst = 0;
  uint32_t src = 0;
  uint32_t res = 0;

  uint32_t mask() const { return (msb << 1) - 1u; }

  // The flags this record defines (Inc/Dec leave CF alone).
  uint32_t definedMask() const {
    return (kind == Inc || kind == Dec) ? kArithMask & ~kCF : kArithMask;
  }

  bool carry() const {
    switch (kind) {
    case Add:
      return res < dst;
    case AddC:
      return res <= dst;
    case Sub:
      return dst < src;
    case SubB:
      return dst <= src;
    default:
      return false;
    }
  }
  bool overflow() const {
    switch (kind) {
    case Add:
    case AddC:
    case Inc:
      return ((dst ^ res) & (src ^ res) & msb) != 0;
    case Sub:
    case SubB:
    case Dec:
      return ((dst ^ src) & (dst ^ res) & msb) != 0;
    default:
      return false;
    }
  }
  bool sign() const { return (res & msb) != 0; }
  bool zero() const { return res == 0; }
  bool parity() const {
    return (std::popcount(static_cast<uint8_t>(res)) & 1) == 0;
  }
  bool aux() const { return kind != Logic && ((dst ^ src ^ res) & 0x10) != 0; }

  // Value of the bits in definedMask().
  uint32_t flags(uint32_t /*eflags*/) const {
    uint32_t f = 0;
    if (kind != Inc && kind != Dec && carry())
      f |= kCF;
    if (overflow())
      f |= kOF;
    if (aux())
      f |= kAF;
    if (zero())
      f |= kZF;
    if (sign())
      f |= kSF;
    if (parity())
      f |= kPF;
    return f;
  }
};

} // namespace fador::cpu
