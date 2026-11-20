// Differential x87 FPU tester.
//
//   fpu_fuzz [--backend] [iterations] [seed] [filter]
//
// Default mode: random sequences of x87 instructions (register and memory
// forms, 16/32/64/80-bit loads/stores, BCD, compares, control instructions,
// FSTENV/FSAVE images) are executed natively on the host x87 (64-bit mode)
// and in the emulator (32-bit flat protected mode); after every instruction
// the 80-bit register stack, tag word, status word and the data area are
// compared.  Every instruction is followed by FNSTSW/FNCLEX on both sides so
// control words with unmasked exceptions can be tested without the host
// trapping.
//
// --backend: value-level comparison of the portable soft back end with the
// host back end (operation by operation, operands biased to special values).
//
// Transcendental instructions (F2XM1 FYL2X FYL2XP1 FPTAN FPATAN FSIN FCOS
// FSINCOS) are bit-exact in the default (host) build.  In a soft build
// (FADOR_SOFT_X87=ON) they are approximations, so their results are compared
// with a tolerance and their PE/UE/C1 bits are not compared.
//
// Exit status is non-zero if any mismatch is found.
#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "cpu/x87/X87Math.hpp"
#include "fpu_fuzz_backend.hpp"
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
#include <string>
#include <sys/mman.h>
#include <vector>

using namespace fador;
using namespace fpu_fuzz;
namespace x87 = fador::cpu::x87;

namespace {

bool isTranscendental(uint8_t modrm) {
  switch (modrm) {
  case 0xF0:
  case 0xF1:
  case 0xF2:
  case 0xF3:
  case 0xF9:
  case 0xFB:
  case 0xFE:
  case 0xFF:
    return true;
  default:
    return false;
  }
}

// Soft back end: transcendental results must be within this many units in
// the last place (of the 64-bit significand) of the host x87's.
constexpr uint64_t kTranscendentalUlps = 16;

std::string swStr(uint16_t sw) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%04X", sw);
  return buf;
}

int runBackend(uint64_t iterations, uint64_t seed, const std::string &filter) {
#if !FADOR_X87_HOST
  (void)iterations;
  (void)seed;
  (void)filter;
  std::printf("fpu_fuzz --backend: no host x87 back end in this build\n");
  return 0;
#else
  ValueGen g(seed);
  BackendStats st;
  const uint16_t kNoResult = x87::SW_IE | x87::SW_DE | x87::SW_ZE;
  static const char *arithNames[8] = {"fadd", "fmul", "", "", "fsub", "fsubr", "fdiv", "fdivr"};
  static const char *fmtNames[4] = {"m32fp", "m32int", "m64fp", "m16int"};
  static const uint8_t arithOps[6] = {0, 1, 4, 5, 6, 7};
  static const uint8_t stackOps[] = {0xE0, 0xE1, 0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5,
                                     0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF};
  static const char *stackNames[] = {"fchs", "fabs", "f2xm1", "fyl2x", "fptan", "fpatan",
                                     "fxtract", "fprem1", "fprem", "fyl2xp1", "fsqrt",
                                     "fsincos", "frndint", "fscale", "fsin", "fcos"};

  for (uint64_t it = 0; it < iterations; ++it) {
    const uint16_t cw = g.controlWord(true);
    const uint16_t unmaskedOf = static_cast<uint16_t>(~cw & x87::SW_EXC);
    const Float80 a = g.value();
    const Float80 b = g.below(3) == 0 ? g.near(a) : g.value();
    const int kind = static_cast<int>(g.below(9));
    std::string name;
    bool ok = true;
    std::string detail;
    auto describe = [&](const char *what, uint16_t sws, uint16_t swh,
                        const std::string &rs, const std::string &rh) {
      char buf[512];
      std::snprintf(buf, sizeof(buf),
                    "  %s cw=%04X a=%s b=%s\n  soft: sw=%s %s\n  host: sw=%s %s\n",
                    what, cw, hex80(a).c_str(), hex80(b).c_str(), swStr(sws).c_str(),
                    rs.c_str(), swStr(swh).c_str(), rh.c_str());
      detail = buf;
    };
    switch (kind) {
    case 0: { // register arithmetic
      const auto op = static_cast<Arith>(arithOps[g.below(6)]);
      name = arithNames[static_cast<int>(op)];
      uint16_t ss = 0, hs = 0;
      const Float80 rs = x87::soft::arith(op, a, b, cw, ss);
      const Float80 rh = x87::host::arith(op, a, b, cw, hs);
      const uint16_t mask = x87::SW_EXC | x87::SW_C1;
      ok = (ss & mask) == (hs & mask) && ((hs & unmaskedOf & kNoResult) || rs == rh);
      if (!ok)
        describe(name.c_str(), ss & mask, hs & mask, hex80(rs), hex80(rh));
      break;
    }
    case 1: { // memory arithmetic
      const auto op = static_cast<Arith>(arithOps[g.below(6)]);
      const auto fmt = static_cast<MemFmt>(g.below(4));
      const uint64_t raw = g.memRaw(fmt);
      name = std::string(arithNames[static_cast<int>(op)]) + " " + fmtNames[static_cast<int>(fmt)];
      uint16_t ss = 0, hs = 0;
      const Float80 rs = x87::soft::arithMem(op, a, fmt, raw, cw, ss);
      const Float80 rh = x87::host::arithMem(op, a, fmt, raw, cw, hs);
      const uint16_t mask = x87::SW_EXC | x87::SW_C1;
      ok = (ss & mask) == (hs & mask) && ((hs & unmaskedOf & kNoResult) || rs == rh);
      if (!ok) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "raw=%016" PRIX64, raw);
        describe(name.c_str(), ss & mask, hs & mask, hex80(rs) + " " + buf, hex80(rh));
      }
      break;
    }
    case 2: { // compares
      const bool unord = g.below(2) != 0;
      name = unord ? "fucom" : "fcom";
      uint16_t ss = 0, hs = 0;
      x87::soft::compare(a, b, unord, ss);
      x87::host::compare(a, b, unord, hs);
      const uint16_t mask = x87::SW_EXC | x87::SW_CC;
      ok = (ss & mask) == (hs & mask);
      if (!ok)
        describe(name.c_str(), ss & mask, hs & mask, "", "");
      break;
    }
    case 3: { // memory compares
      const auto fmt = static_cast<MemFmt>(g.below(4));
      const uint64_t raw = g.memRaw(fmt);
      name = std::string("fcom ") + fmtNames[static_cast<int>(fmt)];
      uint16_t ss = 0, hs = 0;
      x87::soft::compareMem(a, fmt, raw, ss);
      x87::host::compareMem(a, fmt, raw, hs);
      const uint16_t mask = x87::SW_EXC | x87::SW_CC;
      ok = (ss & mask) == (hs & mask);
      if (!ok) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "raw=%016" PRIX64, raw);
        describe(name.c_str(), ss & mask, hs & mask, buf, "");
      }
      break;
    }
    case 4: { // loads
      const bool f64 = g.below(2) != 0;
      const uint64_t raw = g.memRaw(f64 ? MemFmt::F64 : MemFmt::F32);
      name = f64 ? "fld m64" : "fld m32";
      uint16_t ss = 0, hs = 0;
      const Float80 rs = f64 ? x87::soft::loadF64(raw, cw, ss)
                             : x87::soft::loadF32(static_cast<uint32_t>(raw), cw, ss);
      const Float80 rh = f64 ? x87::host::loadF64(raw, cw, hs)
                             : x87::host::loadF32(static_cast<uint32_t>(raw), cw, hs);
      const uint16_t mask = x87::SW_EXC | x87::SW_C1;
      ok = (ss & mask) == (hs & mask) && ((hs & unmaskedOf & kNoResult) || rs == rh);
      if (!ok) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "raw=%016" PRIX64, raw);
        describe(name.c_str(), ss & mask, hs & mask, hex80(rs) + " " + buf, hex80(rh));
      }
      break;
    }
    case 5: { // float stores
      const bool f64 = g.below(2) != 0;
      name = f64 ? "fst m64" : "fst m32";
      uint16_t ss = 0, hs = 0;
      const uint64_t rs = f64 ? x87::soft::storeF64(a, cw, ss) : x87::soft::storeF32(a, cw, ss);
      const uint64_t rh = f64 ? x87::host::storeF64(a, cw, hs) : x87::host::storeF32(a, cw, hs);
      const uint16_t mask = x87::SW_EXC | x87::SW_C1;
      const uint16_t noStore = x87::SW_IE | x87::SW_DE | x87::SW_OE | x87::SW_UE;
      ok = (ss & mask) == (hs & mask) && ((hs & unmaskedOf & noStore) || rs == rh);
      if (!ok) {
        char b1[32], b2[32];
        std::snprintf(b1, sizeof(b1), "%016" PRIX64, rs);
        std::snprintf(b2, sizeof(b2), "%016" PRIX64, rh);
        describe(name.c_str(), ss & mask, hs & mask, b1, b2);
      }
      break;
    }
    case 6: { // integer / BCD stores
      const int which = static_cast<int>(g.below(4));
      uint16_t ss = 0, hs = 0;
      const uint16_t mask = x87::SW_EXC | x87::SW_C1;
      std::string rsS, rhS;
      if (which < 3) {
        const int bits = which == 0 ? 16 : which == 1 ? 32 : 64;
        name = "fist m" + std::to_string(bits);
        const int64_t rs = x87::soft::storeInt(a, bits, cw, ss);
        const int64_t rh = x87::host::storeInt(a, bits, cw, hs);
        ok = rs == rh || (hs & unmaskedOf & x87::SW_IE);
        char b1[32], b2[32];
        std::snprintf(b1, sizeof(b1), "%" PRId64, rs);
        std::snprintf(b2, sizeof(b2), "%" PRId64, rh);
        rsS = b1;
        rhS = b2;
      } else {
        name = "fbstp";
        uint8_t bs[10], bh[10];
        x87::soft::storeBCD(a, bs, cw, ss);
        x87::host::storeBCD(a, bh, cw, hs);
        ok = std::memcmp(bs, bh, 10) == 0 || (hs & unmaskedOf & x87::SW_IE);
        for (int i = 9; i >= 0; --i) {
          char b[4];
          std::snprintf(b, sizeof(b), "%02X", bs[i]);
          rsS += b;
          std::snprintf(b, sizeof(b), "%02X", bh[i]);
          rhS += b;
        }
      }
      ok = ok && (ss & mask) == (hs & mask);
      if (!ok)
        describe(name.c_str(), ss & mask, hs & mask, rsS, rhS);
      break;
    }
    default: { // D9 E0..FF stack operations
      const int idx = static_cast<int>(g.below(sizeof(stackOps)));
      const uint8_t modrm = stackOps[idx];
      name = stackNames[idx];
      const bool trans = isTranscendental(modrm);
      Float80 in[2] = {a, b};
      if (trans && g.below(2)) {
        // keep transcendental arguments in their useful domain
        in[0] = Float80::make(a.sign(), static_cast<uint16_t>(16383 - 1 - g.below(30)),
                              g.mantissa());
      }
      const int nIn = x87::stackOpInputs(modrm);
      Float80 os[3], oh[3];
      uint16_t ss = 0, hs = 0;
      const int nh = x87::host::stackOp(modrm, nIn, in, oh, cw, hs);
      x87::host::restoreHostControlWord(); // soft maths uses long double
      const int ns = x87::soft::stackOp(modrm, nIn, in, os, cw, ss);
      uint16_t mask = x87::SW_EXC | x87::stackOpDefinedCC(modrm, isNaN(oh[0]));
      if (trans)
        mask &= static_cast<uint16_t>(~(x87::SW_PE | x87::SW_UE | x87::SW_C1 | x87::SW_DE));
      const bool noResult = (hs & unmaskedOf & kNoResult) != 0;
      if (noResult)
        mask &= x87::SW_EXC; // condition codes are left alone
      // Soft transcendentals: values far outside the long double range after
      // an unmasked over/underflow, and trig arguments where the x87's
      // 66-bit pi reduction dominates, are not comparable.
      const bool skipValues =
          trans && ((hs & (x87::SW_OE | x87::SW_UE)) ||
                    ((modrm == 0xF2 || modrm == 0xFB || modrm == 0xFE || modrm == 0xFF) &&
                     in[0].exp() >= 16383 + 63));
      bool resultsOk = noResult || skipValues;
      if (!resultsOk) {
        resultsOk = ns == nh;
        for (int i = 0; resultsOk && i < nh && i < 3; ++i) {
          if (trans)
            resultsOk = closeEnough(os[i], oh[i], kTranscendentalUlps);
          else
            resultsOk = os[i] == oh[i];
        }
      }
      ok = resultsOk && (ss & mask) == (hs & mask);
      if (!ok) {
        std::string rs = std::to_string(ns) + ":", rh = std::to_string(nh) + ":";
        for (int i = 0; i < ns && i < 3; ++i)
          rs += " " + hex80(os[i]);
        for (int i = 0; i < nh && i < 3; ++i)
          rh += " " + hex80(oh[i]);
        in[1] = b;
        describe(name.c_str(), ss & mask, hs & mask, rs, rh);
        detail += "  in0=" + hex80(in[0]) + "\n";
      }
      break;
    }
    }
    if (!filter.empty() && name.find(filter) == std::string::npos)
      continue;
    st.record(name, ok, detail);
  }
  std::printf("fpu_fuzz --backend: %" PRIu64 " operations, %" PRIu64
              " mismatches (seed %" PRIu64 ")\n",
              st.total, st.failures, seed);
  for (const auto &[name, s] : st.perOp) {
    if (s.second)
      std::printf("FAIL %-16s %6" PRIu64 "/%" PRIu64 "\n%s", name.c_str(), s.second,
                  s.first, st.firstFailure[name].c_str());
  }
  return st.failures ? 1 : 0;
#endif
}


// ---------------------------------------------------------------------------
// Instruction-level differential test

struct HostState {
  uint64_t eax;
  uint64_t flags;
};

// Runs generated code natively: RBX = data area, EAX/RFLAGS from HostState.
extern "C" void fpu_fuzz_trampoline(HostState *st, const void *code, void *data);
asm(R"(
    .text
    .globl fpu_fuzz_trampoline
fpu_fuzz_trampoline:
    push %rbx
    push %rbp
    push %r12
    mov %rdi, %r12
    mov %rdx, %rbx
    mov 0(%r12), %eax
    pushq 8(%r12)
    popfq
    call *%rsi
    pushfq
    popq 8(%r12)
    mov %eax, 0(%r12)
    cld
    pop %r12
    pop %rbp
    pop %rbx
    ret
)");

constexpr uint32_t kDataSize = 1024;
constexpr uint32_t kImageOff = 0;     // initial FRSTOR image (108 bytes)
constexpr uint32_t kSlotOff = 128;    // 16 operand slots x 16 bytes
constexpr uint32_t kSlotCount = 16;
constexpr uint32_t kSwLogOff = 400;   // FNSTSW log, 2 bytes per instruction
constexpr uint32_t kEnvOff = 512;     // FNSTENV/FNSAVE target (108 bytes)
constexpr uint32_t kFinalOff = 640;   // final FNSAVE (108 bytes)
constexpr uint32_t kCwSlotOff = 768;  // FLDCW operands

struct Sequence {
  std::vector<uint8_t> code;
  std::vector<std::string> names;
  std::vector<uint32_t> offsets; // code offset of each instruction
  bool usesTranscendental = false;
  int transcendentalIndex = -1; // index in names of the first one
  bool hasEnvImage = false;
};

class InsnGen {
public:
  explicit InsnGen(uint64_t seed) : g(seed) {}

  ValueGen g;

  void mem(Sequence &s, uint8_t opcode, uint8_t reg, uint32_t off) {
    s.code.push_back(opcode);
    if (off < 128 && g.below(2)) {
      s.code.push_back(static_cast<uint8_t>(0x40 | (reg << 3) | 3)); // [ebx+disp8]
      s.code.push_back(static_cast<uint8_t>(off));
    } else {
      s.code.push_back(static_cast<uint8_t>(0x80 | (reg << 3) | 3)); // [ebx+disp32]
      for (int i = 0; i < 4; ++i)
        s.code.push_back(static_cast<uint8_t>(off >> (8 * i)));
    }
  }
  void reg(Sequence &s, uint8_t opcode, uint8_t modrm) {
    s.code.push_back(opcode);
    s.code.push_back(modrm);
  }

  // Fills a 16-byte slot with a value of the given kind.
  void fillSlot(uint8_t *data, uint32_t off, int kind) {
    uint8_t *p = data + off;
    std::memset(p, 0, 16);
    switch (kind) {
    case 0: { // m32fp
      const uint32_t v = static_cast<uint32_t>(g.memRaw(MemFmt::F32));
      std::memcpy(p, &v, 4);
      break;
    }
    case 1: { // m64fp
      const uint64_t v = g.memRaw(MemFmt::F64);
      std::memcpy(p, &v, 8);
      break;
    }
    case 2: { // m16int
      const uint16_t v = static_cast<uint16_t>(g.memRaw(MemFmt::I16));
      std::memcpy(p, &v, 2);
      break;
    }
    case 3: { // m32int
      const uint32_t v = static_cast<uint32_t>(g.memRaw(MemFmt::I32));
      std::memcpy(p, &v, 4);
      break;
    }
    case 4: { // m64int
      const uint64_t v = g.below(2) ? g.r64() : static_cast<uint64_t>(static_cast<int64_t>(g.below(2000)) - 1000);
      std::memcpy(p, &v, 8);
      break;
    }
    case 5: { // m80fp
      x87::storeFloat80(p, g.value());
      break;
    }
    case 6: { // packed BCD
      const int digits = static_cast<int>(g.below(19));
      for (int i = 0; i < 9; ++i) {
        uint8_t lo = static_cast<uint8_t>(2 * i < digits ? g.below(10) : 0);
        uint8_t hi = static_cast<uint8_t>(2 * i + 1 < digits ? g.below(10) : 0);
        p[i] = static_cast<uint8_t>(lo | (hi << 4));
      }
      p[9] = g.below(2) ? 0x80 : 0x00;
      break;
    }
    default:
      break;
    }
  }

  uint32_t slot(uint8_t *data, int kind) {
    const uint32_t off = kSlotOff + 16 * g.below(kSlotCount);
    fillSlot(data, off, kind);
    return off;
  }

  // Appends one random instruction (or a store/restore pair).
  void instruction(Sequence &s, uint8_t *data) {
    auto mark = [&](const std::string &name) {
      s.names.push_back(name);
    };
    const uint32_t r = g.below(100);
    const uint8_t i = static_cast<uint8_t>(g.below(8));
    if (r < 22) { // arithmetic / compare with memory operand
      static const uint8_t ops[4] = {0xD8, 0xDA, 0xDC, 0xDE};
      static const int kinds[4] = {0, 3, 1, 2};
      static const char *names[4] = {"m32fp", "m32int", "m64fp", "m16int"};
      const int f = static_cast<int>(g.below(4));
      const uint8_t op = static_cast<uint8_t>(g.below(8));
      static const char *mn[8] = {"fadd", "fmul", "fcom", "fcomp", "fsub", "fsubr", "fdiv", "fdivr"};
      s.offsets.push_back(static_cast<uint32_t>(s.code.size()));
      mem(s, ops[f], op, slot(data, kinds[f]));
      mark(std::string(mn[op]) + " " + names[f]);
    } else if (r < 44) { // register arithmetic / compare
      static const uint8_t ops[3] = {0xD8, 0xDC, 0xDE};
      const uint8_t opc = ops[g.below(3)];
      const uint8_t op = static_cast<uint8_t>(g.below(8));
      uint8_t modrm = static_cast<uint8_t>(0xC0 | (op << 3) | i);
      if (opc == 0xDE && op == 3)
        modrm = 0xD9; // FCOMPP
      s.offsets.push_back(static_cast<uint32_t>(s.code.size()));
      reg(s, opc, modrm);
      static const char *mn[8] = {"add", "mul", "com", "comp", "sub", "subr", "div", "divr"};
      mark(std::string(opc == 0xD8 ? "d8 " : opc == 0xDC ? "dc " : "de ") + mn[op]);
    } else if (r < 58) { // loads
      s.offsets.push_back(static_cast<uint32_t>(s.code.size()));
      switch (g.below(9)) {
      case 0: mem(s, 0xD9, 0, slot(data, 0)); mark("fld m32"); break;
      case 1: mem(s, 0xDD, 0, slot(data, 1)); mark("fld m64"); break;
      case 2: mem(s, 0xDB, 5, slot(data, 5)); mark("fld m80"); break;
      case 3: mem(s, 0xDF, 0, slot(data, 2)); mark("fild m16"); break;
      case 4: mem(s, 0xDB, 0, slot(data, 3)); mark("fild m32"); break;
      case 5: mem(s, 0xDF, 5, slot(data, 4)); mark("fild m64"); break;
      case 6: mem(s, 0xDF, 4, slot(data, 6)); mark("fbld"); break;
      case 7: reg(s, 0xD9, static_cast<uint8_t>(0xC0 | i)); mark("fld st(i)"); break;
      default: {
        const uint8_t c = static_cast<uint8_t>(0xE8 + g.below(7));
        reg(s, 0xD9, c);
        mark("fld const");
        break;
      }
      }
    } else if (r < 72) { // stores
      s.offsets.push_back(static_cast<uint32_t>(s.code.size()));
      const uint32_t off = kSlotOff + 16 * g.below(kSlotCount);
      switch (g.below(14)) {
      case 0: mem(s, 0xD9, 2, off); mark("fst m32"); break;
      case 1: mem(s, 0xD9, 3, off); mark("fstp m32"); break;
      case 2: mem(s, 0xDD, 2, off); mark("fst m64"); break;
      case 3: mem(s, 0xDD, 3, off); mark("fstp m64"); break;
      case 4: mem(s, 0xDB, 7, off); mark("fstp m80"); break;
      case 5: mem(s, 0xDF, 2, off); mark("fist m16"); break;
      case 6: mem(s, 0xDF, 3, off); mark("fistp m16"); break;
      case 7: mem(s, 0xDB, 2, off); mark("fist m32"); break;
      case 8: mem(s, 0xDB, 3, off); mark("fistp m32"); break;
      case 9: mem(s, 0xDF, 7, off); mark("fistp m64"); break;
      case 10: mem(s, 0xDF, 6, off); mark("fbstp"); break;
      case 11: reg(s, 0xDD, static_cast<uint8_t>(0xD0 | i)); mark("fst st(i)"); break;
      case 12: reg(s, 0xDD, static_cast<uint8_t>(0xD8 | i)); mark("fstp st(i)"); break;
      default: {
        static const uint8_t alias[3][2] = {{0xD9, 0xD8}, {0xDF, 0xD0}, {0xDF, 0xD8}};
        const int a = static_cast<int>(g.below(3));
        reg(s, alias[a][0], static_cast<uint8_t>(alias[a][1] | i));
        mark("fstp alias");
        break;
      }
      }
    } else if (r < 84) { // D9 E0..FF group
      static const uint8_t ops[] = {0xE0, 0xE1, 0xE4, 0xE5, 0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5,
                                    0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF};
      const uint8_t m = ops[g.below(sizeof(ops))];
      if (isTranscendental(m)) {
        s.usesTranscendental = true;
        s.transcendentalIndex = static_cast<int>(s.names.size());
      }
      s.offsets.push_back(static_cast<uint32_t>(s.code.size()));
      reg(s, 0xD9, m);
      char buf[16];
      std::snprintf(buf, sizeof(buf), "d9 %02x", m);
      mark(buf);
    } else if (r < 92) { // exchange, free, cmov, comi, misc
      s.offsets.push_back(static_cast<uint32_t>(s.code.size()));
      switch (g.below(12)) {
      case 0: reg(s, 0xD9, static_cast<uint8_t>(0xC8 | i)); mark("fxch"); break;
      case 1: reg(s, 0xDD, static_cast<uint8_t>(0xC8 | i)); mark("fxch4"); break;
      case 2: reg(s, 0xDF, static_cast<uint8_t>(0xC8 | i)); mark("fxch7"); break;
      case 3: reg(s, 0xDD, static_cast<uint8_t>(0xC0 | i)); mark("ffree"); break;
      case 4: reg(s, 0xDF, static_cast<uint8_t>(0xC0 | i)); mark("ffreep"); break;
      case 5: reg(s, 0xDA, static_cast<uint8_t>(0xC0 | (g.below(4) << 3) | i)); mark("fcmov"); break;
      case 6: reg(s, 0xDB, static_cast<uint8_t>(0xC0 | (g.below(4) << 3) | i)); mark("fcmovn"); break;
      case 7: {
        const uint8_t opc = g.below(2) ? 0xDB : 0xDF;
        reg(s, opc, static_cast<uint8_t>((g.below(2) ? 0xE8 : 0xF0) | i));
        mark(opc == 0xDB ? "fcomi" : "fcomip");
        break;
      }
      case 8: reg(s, 0xDD, static_cast<uint8_t>((g.below(2) ? 0xE0 : 0xE8) | i)); mark("fucom(p)"); break;
      case 9: reg(s, 0xDA, 0xE9); mark("fucompp"); break;
      case 10: reg(s, 0xDE, static_cast<uint8_t>(0xD0 | i)); mark("fcomp5"); break;
      default: reg(s, 0xD9, 0xD0); mark("fnop"); break;
      }
    } else { // control
      s.offsets.push_back(static_cast<uint32_t>(s.code.size()));
      switch (g.below(8)) {
      case 0: {
        const uint32_t off = kCwSlotOff + 2 * g.below(8);
        uint16_t cw = g.controlWord(true);
        if (g.below(4) == 0)
          cw = static_cast<uint16_t>(cw | 0x1000); // infinity control (ignored)
        std::memcpy(data + off, &cw, 2);
        mem(s, 0xD9, 5, off);
        mark("fldcw");
        break;
      }
      case 1: mem(s, 0xD9, 7, kSlotOff + 16 * g.below(kSlotCount)); mark("fnstcw"); break;
      case 2: mem(s, 0xDD, 7, kSlotOff + 16 * g.below(kSlotCount)); mark("fnstsw m16"); break;
      case 3: reg(s, 0xDF, 0xE0); mark("fnstsw ax"); break;
      case 4: reg(s, 0xDB, 0xE3); mark("fninit"); break;
      case 5: reg(s, 0xDB, static_cast<uint8_t>(g.below(2) ? 0xE0 : 0xE4)); mark("feni/fsetpm"); break;
      case 6:
        mem(s, 0xD9, 6, kEnvOff);
        mark("fnstenv");
        s.hasEnvImage = true;
        if (g.below(2)) {
          logStatus(s);
          s.offsets.push_back(static_cast<uint32_t>(s.code.size()));
          mem(s, 0xD9, 4, kEnvOff);
          mark("fldenv");
        }
        break;
      default:
        mem(s, 0xDD, 6, kEnvOff);
        mark("fnsave");
        s.hasEnvImage = true;
        if (g.below(2)) {
          logStatus(s);
          s.offsets.push_back(static_cast<uint32_t>(s.code.size()));
          mem(s, 0xDD, 4, kEnvOff);
          mark("frstor");
        }
        break;
      }
    }
    logStatus(s);
  }

  // FNSTSW [log + 2k]; FNCLEX
  void logStatus(Sequence &s) {
    const uint32_t k = static_cast<uint32_t>(s.names.size() - 1);
    mem(s, 0xDD, 7, kSwLogOff + 2 * k);
    s.code.push_back(0xDB);
    s.code.push_back(0xE2);
  }

  // Random initial FRSTOR image (32-bit protected-mode layout).
  void initialImage(uint8_t *data) {
    uint8_t *p = data + kImageOff;
    std::memset(p, 0, 108);
    const uint16_t cw = g.controlWord(true);
    uint16_t sw = static_cast<uint16_t>((g.below(8) << 11) | (g.r32() & (x87::SW_CC)));
    // Sticky flags only where masked, so nothing is pending.
    sw = static_cast<uint16_t>(sw | (g.r32() & cw & 0x3F & (g.below(4) == 0 ? 0x3F : 0)));
    uint16_t tw = 0;
    const int depth = static_cast<int>(g.below(9));
    const int top = (sw >> 11) & 7;
    for (int phys = 0; phys < 8; ++phys) {
      const int logical = (phys - top) & 7;
      const bool full = logical < depth;
      tw = static_cast<uint16_t>(tw | ((full ? 0 : 3) << (2 * phys)));
    }
    const uint32_t w[7] = {0xFFFF0000u | cw, 0xFFFF0000u | sw, 0xFFFF0000u | tw, 0, 0, 0, 0xFFFF0000u};
    std::memcpy(p, w, sizeof(w));
    for (int k = 0; k < 8; ++k)
      x87::storeFloat80(p + 28 + 10 * k, g.value());
  }

  Sequence sequence(uint8_t *data) {
    Sequence s;
    initialImage(data);
    for (uint32_t k = 0; k < kSlotCount; ++k)
      fillSlot(data, kSlotOff + 16 * k, static_cast<int>(g.below(7)));
    mem(s, 0xDD, 4, kImageOff); // FRSTOR initial image
    const int n = 1 + static_cast<int>(g.below(8));
    for (int k = 0; k < n; ++k) {
      instruction(s, data);
      // Soft back end: transcendental results are approximations, so one
      // ends the sequence (nothing downstream depends on its last bits).
      if (FADOR_X87_FORCE_SOFT && s.usesTranscendental)
        break;
    }
    mem(s, 0xDD, 6, kFinalOff); // FNSAVE final state
    return s;
  }
};

struct Emu {
  cpu::CPU cpu{cpu::CpuModelId::PentiumPro}; // FCMOVcc and FCOMI are P6 instructions
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
  static constexpr uint32_t DATA = 0x40000;

  void writeDesc(uint32_t addr, bool code) {
    mem.write32(addr, 0x0000FFFF);
    mem.write32(addr + 4, 0x00CF9200 | (code ? 0x0A00 : 0));
  }
  Emu() {
    mem.setA20(true);
    writeDesc(GDT + 8, true);
    writeDesc(GDT + 16, false);
  }
  // Returns false if execution did not reach the end of the code.
  bool run(const std::vector<uint8_t> &code, uint8_t *data, HostState &st) {
    cpu.setGDTR({0x17, GDT});
    cpu.setCR(0, (cpu.getCR(0) | 1) & ~0x20u);
    cpu.setEFLAGS(0x202);
    decoder.loadSegment(cpu::CS, 0x08);
    decoder.loadSegment(cpu::DS, 0x10);
    decoder.loadSegment(cpu::ES, 0x10);
    decoder.loadSegment(cpu::SS, 0x10);
    for (size_t k = 0; k < code.size(); ++k)
      mem.write8(CODE + static_cast<uint32_t>(k), code[k]);
    for (uint32_t k = 0; k < kDataSize; ++k)
      mem.write8(DATA + k, data[k]);
    cpu.setReg32(cpu::EBX, DATA);
    cpu.setReg32(cpu::EAX, static_cast<uint32_t>(st.eax));
    cpu.setReg32(cpu::ESP, 0x80000);
    cpu.setEFLAGS(0x00000202u | (static_cast<uint32_t>(st.flags) & 0x8D5u));
    cpu.setEIP(CODE);
    const uint32_t end = CODE + static_cast<uint32_t>(code.size());
    for (int steps = 0; steps < 400 && cpu.getEIP() != end; ++steps)
      decoder.step();
    for (uint32_t k = 0; k < kDataSize; ++k)
      data[k] = mem.read8(DATA + k);
    st.eax = cpu.getReg32(cpu::EAX);
    st.flags = cpu.getEFLAGS();
    return cpu.getEIP() == end && cpu.getSegReg(cpu::CS) == 0x08;
  }
};

std::string hexRange(const uint8_t *p, uint32_t n) {
  std::string out;
  char b[4];
  for (uint32_t k = 0; k < n; ++k) {
    std::snprintf(b, sizeof(b), "%02X", p[k]);
    out += b;
    if (k % 2 == 1)
      out += ' ';
  }
  return out;
}

// Bytes that legitimately differ: FIP/FCS/FOP/FDP/FDS in saved images.
bool ignoredByte(uint32_t off, bool envImage) {
  if (off >= kFinalOff + 12 && off < kFinalOff + 28)
    return true;
  if (envImage && off >= kEnvOff + 12 && off < kEnvOff + 28)
    return true;
  if (off >= kImageOff && off < kImageOff + 108)
    return true; // input only
  return false;
}

// Soft builds: a sequence ending in a transcendental instruction is compared
// exactly except for that instruction's PE/UE/C1 (and the ES/B summary) and
// the register values, which must be within kTranscendentalUlps.
bool transcendentalTolerant(const uint8_t *host, const uint8_t *emu, const Sequence &s) {
  const uint16_t loose = x87::SW_PE | x87::SW_UE | x87::SW_C1 | x87::SW_ES | x87::SW_B;
  auto sw16 = [](const uint8_t *p) {
    uint16_t v;
    std::memcpy(&v, p, 2);
    return v;
  };
  const uint32_t lastLog = kSwLogOff + 2 * static_cast<uint32_t>(s.names.size() - 1);
  for (uint32_t k = 0; k < kDataSize; ++k) {
    if (ignoredByte(k, s.hasEnvImage))
      continue;
    if (k == lastLog || k == lastLog + 1 || (k >= kFinalOff && k < kFinalOff + 108))
      continue;
    if (host[k] != emu[k])
      return false;
  }
  if ((sw16(host + lastLog) ^ sw16(emu + lastLog)) & ~loose)
    return false;
  const uint8_t *hf = host + kFinalOff, *ef = emu + kFinalOff;
  if (std::memcmp(hf, ef, 4) != 0 || std::memcmp(hf + 8, ef + 8, 4) != 0)
    return false; // control word, tag word
  if ((sw16(hf + 4) ^ sw16(ef + 4)) & ~loose)
    return false;
  for (int r = 0; r < 8; ++r) {
    const Float80 a = x87::loadFloat80(hf + 28 + 10 * r);
    const Float80 b = x87::loadFloat80(ef + 28 + 10 * r);
    if (!closeEnough(a, b, kTranscendentalUlps))
      return false;
  }
  return true;
}

int runInstructions(uint64_t iterations, uint64_t seed, const std::string &filter,
                    bool verbose) {
  uint8_t *page = static_cast<uint8_t *>(mmap(nullptr, 8192, PROT_READ | PROT_WRITE | PROT_EXEC,
                                              MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));
  if (page == MAP_FAILED) {
    std::perror("mmap");
    return 2;
  }
  std::signal(SIGFPE, [](int) {
    std::fprintf(stderr, "fpu_fuzz: host SIGFPE (unmasked exception escaped)\n");
    std::_Exit(3);
  });
  InsnGen gen(seed);
  Emu emu;
  std::map<std::string, std::pair<uint64_t, uint64_t>> stats;
  std::map<std::string, std::string> firstFailure;
  uint64_t failures = 0, executed = 0, skippedTrans = 0;
  alignas(16) static uint8_t hostData[kDataSize];
  static uint8_t emuData[kDataSize];
  static uint8_t init[kDataSize];

  for (uint64_t it = 0; it < iterations; ++it) {
    std::memset(init, 0, sizeof(init));
    Sequence s = gen.sequence(init);
    std::string key = s.names.empty() ? "?" : s.names.back();
    if (!filter.empty()) {
      bool any = false;
      for (const auto &n : s.names)
        any = any || n.find(filter) != std::string::npos;
      if (!any)
        continue;
    }
    ++executed;
    std::memcpy(hostData, init, kDataSize);
    std::memcpy(emuData, init, kDataSize);
    HostState hs{static_cast<uint32_t>(gen.g.r32()), 0x202u | (gen.g.r32() & 0x8D5u)};
    HostState es = hs;

    std::memcpy(page, s.code.data(), s.code.size());
    page[s.code.size()] = 0xC3;
    fpu_fuzz_trampoline(&hs, page, hostData);
#if FADOR_X87_HOST
    x87::host::restoreHostControlWord();
#endif
    const bool completed = emu.run(s.code, emuData, es);

    bool ok = completed;
    uint32_t firstDiff = kDataSize;
    for (uint32_t k = 0; k < kDataSize && ok; ++k) {
      if (ignoredByte(k, s.hasEnvImage))
        continue;
      if (hostData[k] != emuData[k]) {
        ok = false;
        firstDiff = k;
      }
    }
    if ((hs.eax & 0xFFFF) != (es.eax & 0xFFFF) || ((hs.flags ^ es.flags) & 0x45))
      ok = false;
    if (!ok && FADOR_X87_FORCE_SOFT && s.usesTranscendental && completed &&
        (hs.eax & 0xFFFF) == (es.eax & 0xFFFF) && !((hs.flags ^ es.flags) & 0x45) &&
        transcendentalTolerant(hostData, emuData, s)) {
      ++skippedTrans; // within tolerance
      ok = true;
    }
    // Attribute the result to every instruction in the sequence.
    std::string seqDesc;
    for (const auto &n : s.names)
      seqDesc += n + "; ";
    for (const auto &n : s.names) {
      auto &st = stats[n];
      ++st.first;
    }
    if (!ok) {
      ++failures;
      // Blame the first instruction whose status log differs, else the last.
      std::string blame = s.names.back();
      for (size_t k = 0; k < s.names.size(); ++k) {
        const uint32_t off = kSwLogOff + 2 * static_cast<uint32_t>(k);
        if (std::memcmp(hostData + off, emuData + off, 2) != 0) {
          blame = s.names[k];
          break;
        }
      }
      ++stats[blame].second;
      if (!firstFailure.count(blame) || verbose) {
        char buf[256];
        std::string msg = "  seq: " + seqDesc + "\n  code: " + hexRange(s.code.data(), static_cast<uint32_t>(s.code.size())) + "\n";
        std::snprintf(buf, sizeof(buf), "  first diff at data+%u%s  AX host=%04X emu=%04X  FL host=%03X emu=%03X\n",
                      firstDiff, completed ? "" : " (emulator did not complete)",
                      static_cast<unsigned>(hs.eax & 0xFFFF), static_cast<unsigned>(es.eax & 0xFFFF),
                      static_cast<unsigned>(hs.flags & 0x8D5), static_cast<unsigned>(es.flags & 0x8D5));
        msg += buf;
        msg += "  init image: " + hexRange(init + kImageOff, 28) + "\n  init regs:  " + hexRange(init + kImageOff + 28, 80) + "\n";
        msg += "  swlog host: " + hexRange(hostData + kSwLogOff, 2 * static_cast<uint32_t>(s.names.size())) + "\n";
        msg += "  swlog emu:  " + hexRange(emuData + kSwLogOff, 2 * static_cast<uint32_t>(s.names.size())) + "\n";
        msg += "  final host: " + hexRange(hostData + kFinalOff, 12) + "| " + hexRange(hostData + kFinalOff + 28, 80) + "\n";
        msg += "  final emu:  " + hexRange(emuData + kFinalOff, 12) + "| " + hexRange(emuData + kFinalOff + 28, 80) + "\n";
        if (firstDiff >= kSlotOff && firstDiff < kSlotOff + 16 * kSlotCount) {
          const uint32_t so = kSlotOff + (firstDiff - kSlotOff) / 16 * 16;
          msg += "  slot host:  " + hexRange(hostData + so, 16) + "\n  slot emu:   " + hexRange(emuData + so, 16) +
                 "\n  slot init:  " + hexRange(init + so, 16) + "\n";
        }
        if (verbose)
          std::printf("FAIL [%s]\n%s", blame.c_str(), msg.c_str());
        firstFailure[blame] = msg;
      }
    }
  }
  munmap(page, 8192);
  std::printf("fpu_fuzz: %" PRIu64 " sequences, %" PRIu64 " mismatches (seed %" PRIu64 ", %s back end)\n",
              executed, failures, seed, FADOR_X87_HOST ? "host x87" : "soft");
  if (skippedTrans)
    std::printf("  %" PRIu64 " sequences ending in a transcendental matched within %" PRIu64
                " ulps, not bit-exactly (soft back end)\n",
                skippedTrans, kTranscendentalUlps);
  for (const auto &[name, st] : stats) {
    if (st.second)
      std::printf("FAIL %-16s %6" PRIu64 "/%" PRIu64 "\n%s", name.c_str(), st.second, st.first,
                  firstFailure[name].c_str());
  }
  return failures ? 1 : 0;
}

} // namespace

int main(int argc, char **argv) {
  bool backend = false;
  bool verbose = false;
  std::vector<std::string> args;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--backend") == 0)
      backend = true;
    else if (std::strcmp(argv[i], "-v") == 0)
      verbose = true;
    else
      args.emplace_back(argv[i]);
  }
  const uint64_t iterations = args.size() > 0 ? std::strtoull(args[0].c_str(), nullptr, 0) : 100000;
  const uint64_t seed = args.size() > 1 ? std::strtoull(args[1].c_str(), nullptr, 0) : 1;
  const std::string filter = args.size() > 2 ? args[2] : "";
  utils::currentLevel = static_cast<utils::LogLevel>(static_cast<int>(utils::LogLevel::Error) + 1);
  if (backend)
    return runBackend(iterations, seed, filter);
  return runInstructions(iterations, seed, filter, verbose);
}
