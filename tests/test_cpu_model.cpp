// CPU model selection: identification (reset signature, CPUID), EFLAGS
// write rules (POPF/IRET masking, IOPL/IF privilege, V86), model-gated
// instructions, control-register bits and the guest cycle counter.
#include "test_framework.hpp"
#include "cpu/Assembler.hpp"
#include "cpu/CPU.hpp"
#include "cpu/CpuModel.hpp"
#include "cpu/Disassembler.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "hw/BIOS.hpp"
#include "hw/DOS.hpp"
#include "hw/IOBus.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIC8259.hpp"
#include "hw/PIT8254.hpp"
#include "memory/MemoryBus.hpp"

#include <initializer_list>
#include <vector>

using namespace fador;
using cpu::CpuModelId;

namespace {

constexpr uint16_t kCodeSeg = 0x0500; // real-mode code at 0x5000
constexpr uint16_t kUdHandlerSeg = 0x0600;
constexpr uint16_t kGpHandlerSeg = 0x0700;
constexpr uint32_t kNoFault = 0xFFFFFFFF;

struct ModelEnv {
  cpu::CPU cpu;
  memory::MemoryBus mem;
  hw::IOBus iobus;
  hw::KeyboardController kbd;
  hw::PIT8254 pit;
  hw::PIC8259 pic{true};
  hw::DOS dos{cpu, mem};
  hw::BIOS bios{cpu, mem, kbd, pit, pic};
  cpu::InstructionDecoder decoder{cpu, mem, iobus, bios, dos};

  explicit ModelEnv(CpuModelId model) : cpu(model) {
    bios.initialize();
    dos.initialize();
    // #UD and #GP land on recognisable real-mode handlers.
    mem.write16(6 * 4, 0x0000);
    mem.write16(6 * 4 + 2, kUdHandlerSeg);
    mem.write16(13 * 4, 0x0000);
    mem.write16(13 * 4 + 2, kGpHandlerSeg);
    cpu.setSegReg(cpu::CS, kCodeSeg);
    cpu.setSegReg(cpu::DS, 0x0000);
    cpu.setSegReg(cpu::SS, 0x0000);
    cpu.setReg32(cpu::ESP, 0x8000);
    cpu.setEFLAGS(0x00000002);
    cpu.setEIP(0);
    decoder.syncSegments();
  }

  // Writes `code` at CS:0 and runs `steps` instructions.
  void run(std::initializer_list<uint8_t> code, int steps = 1) {
    uint32_t addr = cpu.getSegBase(cpu::CS);
    for (uint8_t b : code)
      mem.write8(addr++, b);
    cpu.setEIP(0);
    for (int i = 0; i < steps; ++i)
      decoder.step();
  }
  bool raisedUD() const { return cpu.getSegReg(cpu::CS) == kUdHandlerSeg; }
  bool raisedGP() const { return cpu.getSegReg(cpu::CS) == kGpHandlerSeg; }
};

// Flat protected mode owned by the guest (native protection) with ring 0
// and ring 3 segments, a TSS and IDT gates for #UD and #GP.
struct ProtectedEnv : ModelEnv {
  static constexpr uint32_t kGdt = 0x1000, kIdt = 0x2000, kTss = 0x3000;
  static constexpr uint32_t kUdHandler = 0x6000, kGpHandler = 0x7000;
  static constexpr uint32_t kRing0Stack = 0x9000;
  static constexpr uint32_t kCode = 0x5000;

  void descriptor(uint16_t sel, uint32_t high) {
    mem.write32(kGdt + sel, 0x0000FFFF);
    mem.write32(kGdt + sel + 4, high);
  }
  void gate(uint8_t vector, uint32_t offset) {
    mem.write32(kIdt + vector * 8u, (0x08u << 16) | (offset & 0xFFFF));
    mem.write32(kIdt + vector * 8u + 4, (offset & 0xFFFF0000u) | 0x8E00);
  }

  explicit ProtectedEnv(CpuModelId model) : ModelEnv(model) {
    mem.setA20(true);
    descriptor(0x08, 0x00CF9A00); // ring 0 code
    descriptor(0x10, 0x00CF9200); // ring 0 data
    descriptor(0x18, 0x00CFFA00); // ring 3 code
    descriptor(0x20, 0x00CFF200); // ring 3 data
    mem.write32(kGdt + 0x28, 0x67 | ((kTss & 0xFFFF) << 16));
    mem.write32(kGdt + 0x2C, ((kTss >> 16) & 0xFF) | 0x8900);
    mem.write32(kTss + 4, kRing0Stack);
    mem.write32(kTss + 8, 0x10);
    gate(6, kUdHandler);
    gate(13, kGpHandler);
    cpu.setGDTR({0x2F, kGdt});
    cpu.setIDTR({0x7FF, kIdt});
    cpu.setTRSelector(0x28);
    cpu.setTR({0x67, kTss});
    cpu.setCR(0, cpu.getCR(0) | 1);
    cpu.setNativeProtection(true);
  }

  void enterRing(uint8_t ring, uint32_t flags) {
    cpu.setEFLAGS(flags);
    const uint16_t cs = ring ? 0x1B : 0x08;
    const uint16_t ds = ring ? 0x23 : 0x10;
    decoder.loadSegment(cpu::CS, cs);
    decoder.loadSegment(cpu::DS, ds);
    decoder.loadSegment(cpu::ES, ds);
    decoder.loadSegment(cpu::SS, ds);
    cpu.setReg32(cpu::ESP, 0x8000);
  }

  void enterV86(uint32_t iopl) {
    cpu.setEFLAGS(0x00020202u | (iopl << 12));
    decoder.loadSegment(cpu::CS, kCode >> 4);
    decoder.loadSegment(cpu::DS, 0);
    decoder.loadSegment(cpu::ES, 0);
    decoder.loadSegment(cpu::SS, 0);
    cpu.setReg32(cpu::ESP, 0x8000);
  }

  void runAt(std::initializer_list<uint8_t> code, int steps = 1) {
    uint32_t addr = kCode;
    for (uint8_t b : code)
      mem.write8(addr++, b);
    cpu.setEIP((cpu.getEFLAGS() & 0x00020000u) ? 0 : kCode);
    for (int i = 0; i < steps; ++i)
      decoder.step();
  }

  // Returns the error code if #GP was delivered, kNoFault otherwise.
  uint32_t gpError() const {
    if (cpu.getSegReg(cpu::CS) != 0x08 || cpu.getEIP() != kGpHandler)
      return kNoFault;
    return mem.read32(cpu.getReg32(cpu::ESP));
  }
  bool raisedUD() const {
    return cpu.getSegReg(cpu::CS) == 0x08 && cpu.getEIP() == kUdHandler;
  }
};

// PUSH imm16 / POPF / PUSHF / POP AX: returns the flags image after a
// 16-bit POPF of `value` in real mode.
uint16_t realModePopfImage(ModelEnv &env, uint16_t value) {
  env.run({0x68, static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8),
           0x9D, 0x9C, 0x58},
          4);
  return env.cpu.getReg16(cpu::AX);
}

// Toggles `bit` in EFLAGS with PUSHFD/POPFD and reports whether it stuck
// (the classic AC / ID detection sequence).
bool eflagsBitToggles(ModelEnv &env, uint32_t bit) {
  env.cpu.setReg32(cpu::ECX, bit);
  env.run({0x66, 0x9C,       // PUSHFD
           0x66, 0x58,       // POP EAX
           0x66, 0x89, 0xC3, // MOV EBX, EAX
           0x66, 0x31, 0xC8, // XOR EAX, ECX
           0x66, 0x50,       // PUSH EAX
           0x66, 0x9D,       // POPFD
           0x66, 0x9C,       // PUSHFD
           0x66, 0x58},      // POP EAX
          8);
  return ((env.cpu.getReg32(cpu::EAX) ^ env.cpu.getReg32(cpu::EBX)) & bit) != 0;
}

} // namespace

TEST_CASE("CPU model: names, reset signature and CR0", "[CPU][Model]") {
  REQUIRE(cpu::parseCpuModel("386") == CpuModelId::I386DX);
  REQUIRE(cpu::parseCpuModel("486") == CpuModelId::I486DX);
  REQUIRE(cpu::parseCpuModel("486dx2") == CpuModelId::I486DX2);
  REQUIRE(cpu::parseCpuModel("486dx4") == CpuModelId::I486DX4);
  REQUIRE(cpu::parseCpuModel("pentium") == CpuModelId::Pentium);
  REQUIRE(cpu::parseCpuModel("pentiumpro") == CpuModelId::PentiumPro);
  REQUIRE(!cpu::parseCpuModel("z80").has_value());

  struct Expect {
    CpuModelId id;
    uint32_t edx;
  };
  for (const Expect e : {Expect{CpuModelId::I386DX, 0x0308},
                         Expect{CpuModelId::I486DX, 0x0404},
                         Expect{CpuModelId::I486DX2, 0x0435},
                         Expect{CpuModelId::I486DX4, 0x0480},
                         Expect{CpuModelId::Pentium, 0x0525},
                         Expect{CpuModelId::PentiumPro, 0x0619}}) {
    cpu::CPU c(e.id);
    REQUIRE(c.getReg32(cpu::EDX) == e.edx);
    REQUIRE(c.getCR(0) == 0x10); // ET: an FPU is present
    REQUIRE(c.getEFLAGS() == 0x2);
  }
}

TEST_CASE("CPU model: CPUID per model", "[CPU][Model][CPUID]") {
  SECTION("386 and early 486 have no CPUID (#UD)") {
    for (auto id : {CpuModelId::I386DX, CpuModelId::I486DX}) {
      ModelEnv env(id);
      env.cpu.setReg32(cpu::EAX, 0);
      env.run({0x0F, 0xA2});
      REQUIRE(env.raisedUD());
      // The #UD frame points at the CPUID instruction itself.
      REQUIRE(env.mem.read16(env.cpu.getSegBase(cpu::SS) +
                             env.cpu.getReg16(cpu::SP)) == 0);
    }
  }

  SECTION("486DX2 / DX4: GenuineIntel, family 4, FPU only") {
    for (auto [id, sig] : {std::pair{CpuModelId::I486DX2, 0x0435u},
                           std::pair{CpuModelId::I486DX4, 0x0480u}}) {
      ModelEnv env(id);
      env.cpu.setReg32(cpu::EAX, 0);
      env.run({0x0F, 0xA2});
      REQUIRE(env.cpu.getReg32(cpu::EAX) == 1);
      REQUIRE(env.cpu.getReg32(cpu::EBX) == 0x756E6547);
      REQUIRE(env.cpu.getReg32(cpu::EDX) == 0x49656E69);
      REQUIRE(env.cpu.getReg32(cpu::ECX) == 0x6C65746E);
      env.cpu.setReg32(cpu::EAX, 1);
      env.run({0x0F, 0xA2});
      REQUIRE(env.cpu.getReg32(cpu::EAX) == sig);
      REQUIRE(env.cpu.getReg32(cpu::EDX) == 0x00000001);
      REQUIRE(env.cpu.getReg32(cpu::ECX) == 0);
    }
  }

  SECTION("Pentium: family 5 model 2, FPU PSE TSC MSR CX8") {
    ModelEnv env(CpuModelId::Pentium);
    env.cpu.setReg32(cpu::EAX, 1);
    env.run({0x0F, 0xA2});
    REQUIRE(env.cpu.getReg32(cpu::EAX) == 0x0525);
    REQUIRE(env.cpu.getReg32(cpu::EDX) == 0x00000139);
    REQUIRE((env.cpu.getReg32(cpu::EDX) & 0x00800000) == 0); // no MMX
    // Leaves above the maximum return the highest basic leaf (Intel).
    for (uint32_t leaf : {2u, 7u, 0x80000000u}) {
      env.cpu.setReg32(cpu::EAX, leaf);
      env.run({0x0F, 0xA2});
      REQUIRE(env.cpu.getReg32(cpu::EAX) == 0x0525);
      REQUIRE(env.cpu.getReg32(cpu::EDX) == 0x00000139);
    }
  }

  SECTION("Pentium Pro: CMOV, leaf 2") {
    ModelEnv env(CpuModelId::PentiumPro);
    env.cpu.setReg32(cpu::EAX, 0);
    env.run({0x0F, 0xA2});
    REQUIRE(env.cpu.getReg32(cpu::EAX) == 2);
    env.cpu.setReg32(cpu::EAX, 1);
    env.run({0x0F, 0xA2});
    REQUIRE(env.cpu.getReg32(cpu::EAX) == 0x0619);
    REQUIRE((env.cpu.getReg32(cpu::EDX) & (1u << 15)) != 0);
    env.cpu.setReg32(cpu::EAX, 2);
    env.run({0x0F, 0xA2});
    REQUIRE((env.cpu.getReg32(cpu::EAX) & 0xFF) == 1);
  }
}

TEST_CASE("CPU model: detection idioms", "[CPU][Model][Flags]") {
  for (auto id : {CpuModelId::I386DX, CpuModelId::I486DX, CpuModelId::I486DX2,
                  CpuModelId::Pentium}) {
    ModelEnv env(id);
    // 8086: bits 12-15 always set; 286 real mode: always clear; 386+:
    // IOPL/NT follow the value written and bit 15 stays clear.
    REQUIRE((realModePopfImage(env, 0xF000) & 0xF000) == 0x7000);
    REQUIRE((realModePopfImage(env, 0x0000) & 0xF000) == 0x0000);
    // Reserved bits 3 and 5 read as 0, bit 1 as 1.
    REQUIRE((realModePopfImage(env, 0x00FF) & 0x00FF) == 0x00D7);

    const bool is386 = id == CpuModelId::I386DX;
    const bool hasCpuid = id == CpuModelId::I486DX2 || id == CpuModelId::Pentium;
    REQUIRE(eflagsBitToggles(env, 0x00040000) == !is386); // AC: 486+
    REQUIRE(eflagsBitToggles(env, 0x00200000) == hasCpuid); // ID: CPUID
  }
}

TEST_CASE("CPU model: POPF masking", "[CPU][Model][Flags]") {
  SECTION("Real mode POPFD loads only existing bits") {
    ModelEnv p5(CpuModelId::Pentium);
    p5.cpu.push32(0xFFFFFFFF);
    p5.run({0x66, 0x9D});
    // No VM, RF, VIF, VIP or reserved bits; IOPL/NT/AC/ID loaded.
    REQUIRE(p5.cpu.getEFLAGS() == 0x00247FD7);

    ModelEnv i386(CpuModelId::I386DX);
    i386.cpu.push32(0xFFFFFFFF);
    i386.run({0x66, 0x9D});
    REQUIRE(i386.cpu.getEFLAGS() == 0x00007FD7);
  }

  SECTION("CPL 3, IOPL 0: IOPL and IF are unchanged") {
    ProtectedEnv env(CpuModelId::Pentium);
    env.enterRing(3, 0x00000202);
    env.cpu.push32(0x00003000 | 0x0001); // IOPL 3, IF 0, CF 1
    env.runAt({0x9D});                   // POPFD (32-bit code segment)
    REQUIRE(env.gpError() == kNoFault);
    REQUIRE((env.cpu.getEFLAGS() & 0x3201) == 0x0201);
  }

  SECTION("CPL 3, IOPL 3: IF changes, IOPL does not") {
    ProtectedEnv env(CpuModelId::Pentium);
    env.enterRing(3, 0x00003202);
    env.cpu.push32(0x00000000);
    env.runAt({0x9D});
    REQUIRE((env.cpu.getEFLAGS() & 0x3200) == 0x3000);
  }

  SECTION("CPL 0: IOPL and IF change, VM does not") {
    ProtectedEnv env(CpuModelId::Pentium);
    env.enterRing(0, 0x00000202);
    env.cpu.push32(0x00023000);
    env.runAt({0x9D});
    REQUIRE(env.cpu.getEFLAGS() == 0x00003002);
  }

  SECTION("CLI / STI at CPL 3 with IOPL 0 raise #GP(0)") {
    for (uint8_t op : {0xFA, 0xFB}) {
      ProtectedEnv env(CpuModelId::I486DX2);
      env.enterRing(3, 0x00000202);
      env.runAt({op});
      REQUIRE(env.gpError() == 0);
      // The fault frame points at the instruction (ring 3 stack switch).
      REQUIRE(env.mem.read32(env.cpu.getReg32(cpu::ESP) + 4) ==
              ProtectedEnv::kCode);
    }
    ProtectedEnv env(CpuModelId::I486DX2);
    env.enterRing(3, 0x00003202);
    env.runAt({0xFA});
    REQUIRE(env.gpError() == kNoFault);
    REQUIRE((env.cpu.getEFLAGS() & 0x200) == 0);
  }

  SECTION("V86 with IOPL < 3: PUSHF, POPF, CLI, STI, INT n, IRET fault") {
    const std::initializer_list<std::initializer_list<uint8_t>> sensitive = {
        {0x9C}, {0x9D}, {0xFA}, {0xFB}, {0xCD, 0x21}, {0xCF}};
    for (auto code : sensitive) {
      ProtectedEnv env(CpuModelId::Pentium);
      env.enterV86(0);
      env.runAt(code);
      REQUIRE(env.gpError() == 0);
    }
  }

  SECTION("V86 with IOPL 3: POPF changes IF but not IOPL") {
    ProtectedEnv env(CpuModelId::Pentium);
    env.enterV86(3);
    env.cpu.push16(0x0000);
    env.runAt({0x9D});
    REQUIRE(env.gpError() == kNoFault);
    REQUIRE((env.cpu.getEFLAGS() & 0x00023200) == 0x00023000);
  }

  SECTION("IRET at CPL 3 keeps IOPL; reserved bits are not loaded") {
    ProtectedEnv env(CpuModelId::Pentium);
    env.enterRing(3, 0x00000202);
    env.cpu.push32(0xFFFFFFFF & ~0x00020000u); // EFLAGS image
    env.cpu.push32(0x1B);
    env.cpu.push32(ProtectedEnv::kCode + 0x10);
    env.runAt({0xCF}); // IRETD
    REQUIRE(env.cpu.getEIP() == ProtectedEnv::kCode + 0x10);
    // IOPL, VIF, VIP keep their values at CPL 3, and IF since CPL > IOPL.
    REQUIRE(env.cpu.getEFLAGS() == 0x00254FD7);
  }
}

TEST_CASE("CPU model: instruction availability", "[CPU][Model][UD]") {
  using Code = std::initializer_list<uint8_t>;
  const Code bswap{0x66, 0x0F, 0xC8};
  const Code cmpxchg{0x0F, 0xB1, 0xC8};
  const Code xadd{0x0F, 0xC1, 0xC8};
  const Code invd{0x0F, 0x08};
  const Code wbinvd{0x0F, 0x09};
  const Code invlpg{0x0F, 0x01, 0x3E, 0x00, 0x10};
  const Code cpuid{0x0F, 0xA2};
  const Code rdtsc{0x0F, 0x31};
  const Code rdmsr{0x0F, 0x32};
  const Code cmpxchg8b{0x0F, 0xC7, 0x0E, 0x00, 0x10};
  const Code cmov{0x0F, 0x44, 0xC8};
  const Code fcmov{0xDA, 0xC1};
  const Code fcomi{0xDB, 0xF1};
  const Code movups{0x0F, 0x10, 0xC0};

  auto ud = [](CpuModelId id, Code code) {
    ModelEnv env(id);
    env.cpu.setReg32(cpu::ECX, 0x10);
    env.run(code);
    return env.raisedUD();
  };

  for (const Code &c : {bswap, cmpxchg, xadd, invd, wbinvd, invlpg, cpuid})
    REQUIRE(ud(CpuModelId::I386DX, c));
  for (const Code &c : {bswap, cmpxchg, xadd, invd, wbinvd, invlpg})
    REQUIRE(!ud(CpuModelId::I486DX, c));
  REQUIRE(ud(CpuModelId::I486DX, cpuid));
  REQUIRE(!ud(CpuModelId::I486DX2, cpuid));
  for (const Code &c : {rdtsc, rdmsr, cmpxchg8b, cmov, fcmov, fcomi})
    REQUIRE(ud(CpuModelId::I486DX4, c));
  for (const Code &c : {rdtsc, rdmsr, cmpxchg8b})
    REQUIRE(!ud(CpuModelId::Pentium, c));
  for (const Code &c : {cmov, fcmov, fcomi})
    REQUIRE(ud(CpuModelId::Pentium, c));
  for (const Code &c : {cmov, fcmov, fcomi})
    REQUIRE(!ud(CpuModelId::PentiumPro, c));
  // No model has SSE.
  REQUIRE(ud(CpuModelId::PentiumPro, movups));
}

TEST_CASE("CPU model: control registers", "[CPU][Model][CR]") {
  SECTION("486: ET is hardwired, CR4 does not exist") {
    ModelEnv env(CpuModelId::I486DX2);
    env.cpu.setReg32(cpu::EAX, 0x60000000); // CD | NW, ET clear
    env.run({0x0F, 0x22, 0xC0});              // MOV CR0, EAX
    REQUIRE(env.cpu.getCR(0) == 0x60000010);
    env.run({0x0F, 0x20, 0xE0}); // MOV EAX, CR4
    REQUIRE(env.raisedUD());
  }

  SECTION("386: ET writable, 486 bits ignored") {
    ModelEnv env(CpuModelId::I386DX);
    env.cpu.setReg32(cpu::EAX, 0x60010020); // CD NW WP NE
    env.run({0x0F, 0x22, 0xC0});
    REQUIRE(env.cpu.getCR(0) == 0);
  }

  SECTION("Invalid CR0 combinations raise #GP") {
    ModelEnv env(CpuModelId::I486DX2);
    env.cpu.setReg32(cpu::EAX, 0x20000000); // NW without CD
    env.run({0x0F, 0x22, 0xC0});
    REQUIRE(env.raisedGP());
  }

  SECTION("Pentium CR4: TSD/DE/PSE/MCE writable, VME refused") {
    ModelEnv env(CpuModelId::Pentium);
    env.cpu.setReg32(cpu::EAX, 0x10);
    env.run({0x0F, 0x22, 0xE0}); // MOV CR4, EAX
    REQUIRE(env.cpu.getCR(4) == 0x10);
    env.cpu.setReg32(cpu::EAX, 0x01);
    env.run({0x0F, 0x22, 0xE0});
    REQUIRE(env.raisedGP());
  }

  SECTION("LMSW loads only PE/MP/EM/TS") {
    ModelEnv env(CpuModelId::I486DX2);
    env.cpu.setReg32(cpu::EAX, 0xFFFE);
    env.run({0x0F, 0x01, 0xF0}); // LMSW AX
    REQUIRE(env.cpu.getCR(0) == 0x1E);
  }
}

TEST_CASE("CPU model: guest cycles, RDTSC and MSRs", "[CPU][Model][TSC]") {
  SECTION("RDTSC is monotonic and follows the cycle counter") {
    ModelEnv env(CpuModelId::Pentium);
    env.run({0x0F, 0x31, 0x66, 0x89, 0xC3, 0x66, 0x89, 0xD1, // RDTSC; EBX/ECX
             0x90, 0x90, 0x90, 0x0F, 0x31},
            7);
    const uint64_t first =
        (static_cast<uint64_t>(env.cpu.getReg32(cpu::ECX)) << 32) |
        env.cpu.getReg32(cpu::EBX);
    const uint64_t second =
        (static_cast<uint64_t>(env.cpu.getReg32(cpu::EDX)) << 32) |
        env.cpu.getReg32(cpu::EAX);
    REQUIRE(second > first);
    REQUIRE(second <= env.cpu.cycles());
  }

  SECTION("WRMSR/RDMSR 10h set and read the TSC; unknown MSRs #GP") {
    ModelEnv env(CpuModelId::Pentium);
    env.cpu.setReg32(cpu::ECX, 0x10);
    env.cpu.setReg32(cpu::EDX, 0x00000001);
    env.cpu.setReg32(cpu::EAX, 0x00000000);
    env.run({0x0F, 0x30, 0x0F, 0x32}, 2); // WRMSR; RDMSR
    REQUIRE(env.cpu.getReg32(cpu::EDX) == 1);
    REQUIRE(env.cpu.getReg32(cpu::EAX) < 100);

    env.cpu.setReg32(cpu::ECX, 0x1234);
    env.run({0x0F, 0x32});
    REQUIRE(env.raisedGP());
  }

  SECTION("Older models need more cycles for the same code") {
    auto cyclesFor = [](CpuModelId id) {
      ModelEnv env(id);
      const uint64_t start = env.cpu.cycles();
      env.cpu.setReg16(cpu::CX, 10);
      // 10 x (ADD AX,[BX]; IMUL AX,BX; LOOP)
      env.run({0x03, 0x07, 0x0F, 0xAF, 0xC3, 0xE2, 0xF9}, 30);
      return env.cpu.cycles() - start;
    };
    const uint64_t c386 = cyclesFor(CpuModelId::I386DX);
    const uint64_t c486 = cyclesFor(CpuModelId::I486DX);
    const uint64_t cP5 = cyclesFor(CpuModelId::Pentium);
    REQUIRE(c386 > c486);
    REQUIRE(c486 > cP5);
    REQUIRE(cP5 > 0);
  }

  SECTION("REP string instructions charge per iteration") {
    ModelEnv env(CpuModelId::I486DX2);
    env.cpu.setSegReg(cpu::ES, 0x0000);
    env.cpu.setReg32(cpu::ESI, 0x1000);
    env.cpu.setReg32(cpu::EDI, 0x2000);
    env.cpu.setReg32(cpu::ECX, 100);
    const uint64_t start = env.cpu.cycles();
    env.run({0xF3, 0xA4}); // REP MOVSB
    REQUIRE(env.cpu.getReg32(cpu::ECX) == 0);
    REQUIRE(env.cpu.cycles() - start >= 300); // 3 clocks per byte on a 486
  }
}

TEST_CASE("CPU model: assembler and disassembler know the system opcodes",
          "[CPU][Model][Assembler][Disassembler]") {
  cpu::Assembler as;
  memory::MemoryBus mem;
  cpu::Disassembler dis(mem);
  struct Op {
    const char *text;
    std::vector<uint8_t> bytes;
  };
  for (const Op &op : {Op{"CPUID", {0x0F, 0xA2}}, Op{"RDTSC", {0x0F, 0x31}},
                       Op{"RDMSR", {0x0F, 0x32}}, Op{"WRMSR", {0x0F, 0x30}},
                       Op{"WBINVD", {0x0F, 0x09}}}) {
    const auto r = as.assembleLine(op.text, 0x100);
    REQUIRE(r.error.empty());
    REQUIRE(r.bytes == op.bytes);
    for (size_t i = 0; i < op.bytes.size(); ++i)
      mem.write8(0x100 + static_cast<uint32_t>(i), op.bytes[i]);
    const auto d = dis.disassembleAt(0x100);
    REQUIRE(d.mnemonic == op.text);
    REQUIRE(d.length == op.bytes.size());
  }
  // CMPXCHG8B [1000h]
  const uint8_t cx8[] = {0x0F, 0xC7, 0x0E, 0x00, 0x10};
  for (size_t i = 0; i < sizeof(cx8); ++i)
    mem.write8(0x200 + static_cast<uint32_t>(i), cx8[i]);
  const auto d = dis.disassembleAt(0x200);
  REQUIRE(d.length == 5);
  REQUIRE(d.mnemonic.rfind("CMPXCHG8B ", 0) == 0);
}
