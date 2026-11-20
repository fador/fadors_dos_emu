#include "CpuModel.hpp"

namespace fador::cpu {

namespace {

constexpr uint32_t kCr0PE = 0x00000001, kCr0MP = 0x00000002,
                   kCr0EM = 0x00000004, kCr0TS = 0x00000008,
                   kCr0ET = 0x00000010, kCr0NE = 0x00000020,
                   kCr0WP = 0x00010000, kCr0AM = 0x00040000,
                   kCr0NW = 0x20000000, kCr0CD = 0x40000000,
                   kCr0PG = 0x80000000;
constexpr uint32_t kCr0Mask386 = kCr0PE | kCr0MP | kCr0EM | kCr0TS | kCr0ET | kCr0PG;
constexpr uint32_t kCr0Mask486 = kCr0PE | kCr0MP | kCr0EM | kCr0TS | kCr0NE |
                                 kCr0WP | kCr0AM | kCr0NW | kCr0CD | kCr0PG;
// CR4: TSD, DE, PSE, MCE.  VME/PVI are not emulated and stay clear.
constexpr uint32_t kCr4MaskP5 = 0x0000005C;

constexpr uint32_t kFlags386 = eflags::kBase | eflags::kRF | eflags::kVM;
constexpr uint32_t kFlags486 = kFlags386 | eflags::kAC;
constexpr uint32_t kFlags486Cpuid = kFlags486 | eflags::kID;
constexpr uint32_t kFlagsP5 = kFlags486Cpuid | eflags::kVIF | eflags::kVIP;

constexpr uint32_t kFeat486 = feature::k486;
constexpr uint32_t kFeat486Cpuid = kFeat486 | feature::kCpuid;
constexpr uint32_t kFeatP5 = kFeat486Cpuid | feature::kTsc | feature::kMsr |
                             feature::kCx8 | feature::kCr4;
constexpr uint32_t kFeatP6 = kFeatP5 | feature::kCmov;

constexpr uint32_t kEdxP5 = cpuid_edx::kFpu | cpuid_edx::kPse |
                            cpuid_edx::kTsc | cpuid_edx::kMsr | cpuid_edx::kCx8;

using Costs = std::array<uint8_t, kCycleClassCount>;

// Approximate clocks per class, from the Intel data books (register forms,
// real-mode timings, averages over operand sizes / taken and not taken).
// Order: Alu Shift Movx Xchg Push Pop PushPopAll SegLoad Branch Loop Call Ret
// FarCall Interrupt Iret Flags Mul Div Bit Scan String Io EnterLeave Bcd
// System Fpu Hlt
constexpr Costs costs386(uint8_t io) {
  return {2, 3, 3, 3, 2, 4, 21, 4, 6, 11, 8, 11, 18, 37, 22, 3,
          18, 30, 6, 20, 7, io, 8, 4, 15, 35, 5};
}
constexpr Costs costs486(uint8_t io) {
  return {1, 2, 3, 3, 1, 1, 10, 3, 2, 7, 3, 5, 18, 26, 15, 3,
          18, 30, 5, 20, 6, io, 8, 3, 12, 12, 4};
}
constexpr Costs costsP5(uint8_t io) {
  return {1, 1, 3, 2, 1, 1, 5, 3, 1, 6, 1, 2, 4, 16, 8, 3,
          10, 30, 5, 20, 4, io, 6, 3, 10, 3, 12};
}
constexpr Costs costsP6(uint8_t io) {
  return {1, 1, 1, 2, 1, 1, 8, 3, 1, 4, 2, 2, 10, 25, 20, 2,
          4, 25, 1, 1, 3, io, 4, 2, 15, 3, 10};
}

// An ISA bus I/O cycle takes about 0.8 us whatever the core clock.
constexpr uint8_t ioCycles(uint32_t clockHz) {
  return static_cast<uint8_t>(clockHz / 1250000u);
}

constexpr std::array<CpuModel, 6> kModels{{
    {CpuModelId::I386DX, "386", "Intel 386DX-33 + 387", 33000000, 0x0308,
     0, 0, 0, kFlags386, kCr0Mask386, 0, 0, costs386(ioCycles(33000000)), 3,
     {4, 5, 5, 9, 8}},
    {CpuModelId::I486DX, "486", "Intel 486DX-33", 33000000, 0x0404,
     kFeat486, 0, 0, kFlags486, kCr0Mask486, kCr0ET, 0,
     costs486(ioCycles(33000000)), 1, {3, 4, 5, 7, 5}},
    {CpuModelId::I486DX2, "486dx2", "Intel 486DX2-66", 66000000, 0x0435,
     kFeat486Cpuid, 1, cpuid_edx::kFpu, kFlags486Cpuid, kCr0Mask486, kCr0ET,
     0, costs486(ioCycles(66000000)), 1, {3, 4, 5, 7, 5}},
    {CpuModelId::I486DX4, "486dx4", "IntelDX4-100", 100000000, 0x0480,
     kFeat486Cpuid, 1, cpuid_edx::kFpu, kFlags486Cpuid, kCr0Mask486, kCr0ET,
     0, costs486(ioCycles(100000000)), 1, {3, 4, 5, 7, 5}},
    {CpuModelId::Pentium, "pentium", "Intel Pentium 100 (P54C)", 100000000,
     0x0525, kFeatP5, 1, kEdxP5, kFlagsP5, kCr0Mask486, kCr0ET, kCr4MaskP5,
     costsP5(ioCycles(100000000)), 1, {1, 1, 2, 4, 4}},
    {CpuModelId::PentiumPro, "pentiumpro", "Intel Pentium Pro 200",
     200000000, 0x0619, kFeatP6, 2, kEdxP5 | cpuid_edx::kCmov, kFlagsP5,
     kCr0Mask486, kCr0ET, kCr4MaskP5, costsP6(ioCycles(200000000)), 1,
     {1, 1, 2, 2, 2}},
}};

} // namespace

const CpuModel &cpuModel(CpuModelId id) {
  return kModels[static_cast<size_t>(id)];
}

std::optional<CpuModelId> parseCpuModel(std::string_view name) {
  for (const auto &m : kModels)
    if (name == m.cliName)
      return m.id;
  if (name == "386dx")
    return CpuModelId::I386DX;
  if (name == "486dx")
    return CpuModelId::I486DX;
  if (name == "586" || name == "p5")
    return CpuModelId::Pentium;
  if (name == "686" || name == "p6" || name == "ppro")
    return CpuModelId::PentiumPro;
  return std::nullopt;
}

std::string_view cpuModelNames() {
  return "386, 486, 486dx2, 486dx4, pentium, pentiumpro";
}

CycleClass cycleClassOf(uint8_t op, bool twoByte) {
  using C = CycleClass;
  if (twoByte) {
    if (op >= 0x80 && op <= 0x8F)
      return C::Branch;
    if (op >= 0x90 && op <= 0x9F)
      return C::Movx;
    if (op >= 0x40 && op <= 0x4F)
      return C::Alu; // CMOVcc
    if (op >= 0xC8)
      return C::Alu; // BSWAP
    switch (op) {
    case 0xA0: case 0xA8:
      return C::Push;
    case 0xA1: case 0xA9: case 0xB2: case 0xB4: case 0xB5:
      return C::SegLoad;
    case 0xA3: case 0xAB: case 0xB3: case 0xBB: case 0xBA:
    case 0xB0: case 0xB1:
      return C::Bit;
    case 0xBC: case 0xBD:
      return C::Scan;
    case 0xAF:
      return C::Mul;
    case 0xA4: case 0xA5: case 0xAC: case 0xAD:
    case 0xB6: case 0xB7: case 0xBE: case 0xBF:
    case 0xC0: case 0xC1:
      return C::Movx;
    case 0xFF:
      return C::Alu; // emulator HLE trap
    default:
      return C::System;
    }
  }
  if (op <= 0x3F) {
    switch (op & 7) {
    case 6:
      if (op == 0x26 || op == 0x2E || op == 0x36 || op == 0x3E)
        return C::Alu; // prefixes, never looked up
      return C::Push;  // PUSH ES/CS/SS/DS
    case 7:
      if (op == 0x27 || op == 0x2F || op == 0x37 || op == 0x3F)
        return C::Bcd;
      return op == 0x0F ? C::Alu : C::SegLoad; // POP ES/SS/DS
    default:
      return C::Alu;
    }
  }
  if (op <= 0x4F)
    return C::Alu; // INC/DEC
  if (op <= 0x57)
    return C::Push;
  if (op <= 0x5F)
    return C::Pop;
  if (op >= 0x70 && op <= 0x7F)
    return C::Branch;
  if (op >= 0xD8 && op <= 0xDF)
    return C::Fpu;
  switch (op) {
  case 0x60: case 0x61:
    return C::PushPopAll;
  case 0x62: case 0xC8: case 0xC9:
    return C::EnterLeave;
  case 0x63:
    return C::System; // ARPL
  case 0x68: case 0x6A:
    return C::Push;
  case 0x69: case 0x6B: case 0xD4: case 0xD5:
    return C::Mul;
  case 0x6C: case 0x6D: case 0x6E: case 0x6F:
  case 0xE4: case 0xE5: case 0xE6: case 0xE7:
  case 0xEC: case 0xED: case 0xEE: case 0xEF:
    return C::Io;
  case 0x86: case 0x87:
  case 0x91: case 0x92: case 0x93: case 0x94:
  case 0x95: case 0x96: case 0x97:
    return C::Xchg;
  case 0x8E: case 0xC4: case 0xC5:
    return C::SegLoad;
  case 0x8F: case 0xA0: case 0xA1: case 0xA2: case 0xA3: case 0xD7:
    return C::Pop;
  case 0x98: case 0x99:
  case 0xC0: case 0xC1: case 0xD0: case 0xD1: case 0xD2: case 0xD3:
    return C::Shift;
  case 0x9A: case 0xCA: case 0xCB: case 0xEA:
    return C::FarCall;
  case 0x9C: case 0x9D: case 0x9E: case 0x9F: case 0xFA: case 0xFB:
    return C::Flags;
  case 0xA4: case 0xA5: case 0xA6: case 0xA7:
  case 0xAA: case 0xAB: case 0xAC: case 0xAD: case 0xAE: case 0xAF:
    return C::String;
  case 0xC2: case 0xC3:
    return C::Ret;
  case 0xCC: case 0xCD: case 0xCE: case 0xF1:
    return C::Interrupt;
  case 0xCF:
    return C::Iret;
  case 0xE0: case 0xE1: case 0xE2: case 0xE3:
    return C::Loop;
  case 0xE8:
    return C::Call;
  case 0xE9: case 0xEB:
    return C::Branch;
  case 0xF4:
    return C::Hlt;
  default:
    return C::Alu;
  }
}

void buildCycleTable(const CpuModel &model, CycleTable &table) {
  for (unsigned op = 0; op < 256; ++op) {
    const auto b = static_cast<uint8_t>(op);
    table[op] = model.cycles[static_cast<size_t>(cycleClassOf(b, false))];
    table[256 + op] = model.cycles[static_cast<size_t>(cycleClassOf(b, true))];
    table[512 + op] = 0;
  }
  // LEA pays the memory-operand cost through its address calculation.
  const uint8_t alu = model.cycles[static_cast<size_t>(CycleClass::Alu)];
  table[0x8D] = alu > model.memOperandCycles
                    ? static_cast<uint8_t>(alu - model.memOperandCycles)
                    : 0;
  const uint8_t io = model.cycles[static_cast<size_t>(CycleClass::Io)];
  auto rep = [&](uint8_t op, uint8_t c) { table[512 + op] = c; };
  rep(0xA4, model.rep.movs);
  rep(0xA5, model.rep.movs);
  rep(0xA6, model.rep.cmps);
  rep(0xA7, model.rep.cmps);
  rep(0xAA, model.rep.stos);
  rep(0xAB, model.rep.stos);
  rep(0xAC, model.rep.lods);
  rep(0xAD, model.rep.lods);
  rep(0xAE, model.rep.scas);
  rep(0xAF, model.rep.scas);
  rep(0x6C, io);
  rep(0x6D, io);
  rep(0x6E, io);
  rep(0x6F, io);
}

} // namespace fador::cpu
