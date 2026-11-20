// 386/486/Pentium protection model under guest-owned protected mode:
// x87 availability (#NM), privileged instructions, the I/O permission
// bitmap and INS/OUTS, debug registers and single-stepping, hardware task
// switches, virtual-8086 mode (incl. VME/PVI).
#include "test_framework.hpp"
#include "cpu/CPU.hpp"
#include "cpu/CpuModel.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "hw/BIOS.hpp"
#include "hw/DOS.hpp"
#include "hw/IODevice.hpp"
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

constexpr uint32_t kNone = 0xFFFFFFFFu;

// Records port accesses (one entry per byte) and answers reads with
// 0x40 + low port bits.
struct RecorderDevice : hw::IODevice {
  std::vector<std::pair<uint16_t, uint8_t>> writes;
  int reads = 0;
  uint8_t read8(uint16_t port) override {
    ++reads;
    return static_cast<uint8_t>(0x40 + (port & 0x0F));
  }
  void write8(uint16_t port, uint8_t value) override {
    writes.emplace_back(port, value);
  }
};

struct Env {
  // Real-mode layout: code at 0500:0000, exception handlers at
  // (0x0800 + vector):0000.
  static constexpr uint16_t kRealCode = 0x0500;
  // Protected-mode layout (flat segments).
  static constexpr uint32_t kGdt = 0x1000, kIdt = 0x2000, kTss = 0x3000;
  static constexpr uint32_t kTss2 = 0x6000, kTss16 = 0x6800;
  static constexpr uint32_t kHandlers = 0x8000; // vector v at + v * 16
  static constexpr uint32_t kCode = 0x5000;
  static constexpr uint32_t kRing0Stack = 0x9F00;
  // I/O permission bitmap offset; the V86 interrupt redirection bitmap
  // occupies the 32 bytes below it (68h-87h).
  static constexpr uint32_t kIopb = 0x88;
  static constexpr uint16_t kTssSel = 0x28, kTss2Sel = 0x30, kTss16Sel = 0x38;
  static constexpr uint16_t kTaskGateSel = 0x40;

  cpu::CPU cpu;
  memory::MemoryBus mem;
  hw::IOBus iobus;
  hw::KeyboardController kbd;
  hw::PIT8254 pit;
  hw::PIC8259 pic{true};
  hw::DOS dos{cpu, mem};
  hw::BIOS bios{cpu, mem, kbd, pit, pic};
  cpu::InstructionDecoder decoder{cpu, mem, iobus, bios, dos};
  RecorderDevice device;

  explicit Env(CpuModelId model = CpuModelId::Pentium) : cpu(model) {
    bios.initialize();
    dos.initialize();
    iobus.registerDevice(0x00E0, 0x00EF, &device);
    iobus.registerDevice(0x03F0, 0x03FF, &device);
    for (uint32_t v = 0; v < 32; ++v) {
      mem.write16(v * 4, 0);
      mem.write16(v * 4 + 2, static_cast<uint16_t>(0x0800 + v));
    }
    cpu.setSegReg(cpu::CS, kRealCode);
    cpu.setSegReg(cpu::DS, 0);
    cpu.setSegReg(cpu::ES, 0);
    cpu.setSegReg(cpu::SS, 0);
    cpu.setReg32(cpu::ESP, 0x7000);
    cpu.setEFLAGS(0x2);
    cpu.setEIP(0);
    decoder.syncSegments();
  }

  // ── real mode ──
  void runReal(std::initializer_list<uint8_t> code, int steps = 1) {
    uint32_t a = cpu.getSegBase(cpu::CS);
    for (uint8_t b : code)
      mem.write8(a++, b);
    cpu.setEIP(0);
    for (int i = 0; i < steps; ++i)
      decoder.step();
  }
  // Vector whose real-mode handler is running, or kNone.
  uint32_t realVector() const {
    const uint16_t cs = cpu.getSegReg(cpu::CS);
    return (cs >= 0x0800 && cs < 0x0820 && cpu.getEIP() == 0) ? cs - 0x0800u
                                                              : kNone;
  }

  // ── protected mode ──
  void descriptor(uint16_t sel, uint32_t base, uint32_t limit, uint32_t access,
                  uint32_t flags = 0) {
    const uint32_t low = (limit & 0xFFFF) | ((base & 0xFFFF) << 16);
    const uint32_t high = ((base >> 16) & 0xFF) | (access << 8) |
                          (limit & 0xF0000) | (flags << 20) |
                          (base & 0xFF000000u);
    mem.write32(kGdt + (sel & 0xFFF8u), low);
    mem.write32(kGdt + (sel & 0xFFF8u) + 4, high);
  }
  void gate(uint8_t vector, uint32_t offset, uint32_t access = 0xEE,
            uint16_t sel = 0x08) {
    mem.write32(kIdt + vector * 8u, (uint32_t(sel) << 16) | (offset & 0xFFFF));
    mem.write32(kIdt + vector * 8u + 4, (offset & 0xFFFF0000u) | (access << 8));
  }
  uint32_t gdtHigh(uint16_t sel) const { return mem.read32(kGdt + (sel & 0xFFF8u) + 4); }
  bool busy(uint16_t sel) const { return (gdtHigh(sel) & 0x0200) != 0; }

  void enterProtected() {
    mem.setA20(true);
    descriptor(0x08, 0, 0xFFFFF, 0x9A, 0xC); // ring 0 code
    descriptor(0x10, 0, 0xFFFFF, 0x92, 0xC); // ring 0 data
    descriptor(0x18, 0, 0xFFFFF, 0xFA, 0xC); // ring 3 code
    descriptor(0x20, 0, 0xFFFFF, 0xF2, 0xC); // ring 3 data
    // 32-bit TSS with a full I/O permission bitmap (all ports allowed)
    // and the terminating FFh byte.
    descriptor(kTssSel, kTss, kIopb + 0x2000, 0x89);
    for (uint32_t i = 0; i < kIopb; ++i)
      mem.write8(kTss + i, 0);
    mem.write32(kTss + 4, kRing0Stack);
    mem.write32(kTss + 8, 0x10);
    mem.write16(kTss + 0x66, kIopb);
    for (uint32_t i = 0; i < 0x2000; ++i)
      mem.write8(kTss + kIopb + i, 0);
    mem.write8(kTss + kIopb + 0x2000, 0xFF);
    for (uint8_t v = 0; v < 32; ++v)
      gate(v, kHandlers + v * 16u);
    cpu.setGDTR({0x7F, kGdt});
    cpu.setIDTR({0x7FF, kIdt});
    cpu.setCR(0, cpu.getCR(0) | 1);
    cpu.setNativeProtection(true);
    mem.write32(kGdt + kTssSel + 4, gdtHigh(kTssSel) | 0x0200); // busy
    cpu.setTRSelector(kTssSel);
    cpu.setTR({static_cast<uint16_t>(kIopb + 0x2000), kTss});
  }
  void enterRing(uint8_t ring, uint32_t flags = 0x202) {
    cpu.setEFLAGS(flags);
    const uint16_t cs = ring ? 0x1B : 0x08, ds = ring ? 0x23 : 0x10;
    decoder.loadSegment(cpu::CS, cs);
    decoder.loadSegment(cpu::DS, ds);
    decoder.loadSegment(cpu::ES, ds);
    decoder.loadSegment(cpu::SS, ds);
    decoder.loadSegment(cpu::FS, ds);
    decoder.loadSegment(cpu::GS, ds);
    cpu.setReg32(cpu::ESP, 0x7000);
    cpu.setEIP(kCode);
  }
  void enterV86(uint32_t iopl, uint32_t extraFlags = 0) {
    cpu.setEFLAGS(0x00020202u | (iopl << 12) | extraFlags);
    for (auto s : {cpu::CS, cpu::DS, cpu::ES, cpu::SS, cpu::FS, cpu::GS})
      decoder.loadSegment(s, 0);
    decoder.loadSegment(cpu::CS, kCode >> 4);
    cpu.setReg32(cpu::ESP, 0x7000);
    cpu.setEIP(0);
  }
  // Writes `code` at kCode and runs `steps` instructions from the current
  // EIP (kCode in protected mode, 0 in V86 mode).
  void run(std::initializer_list<uint8_t> code, int steps = 1) {
    uint32_t a = kCode;
    for (uint8_t b : code)
      mem.write8(a++, b);
    for (int i = 0; i < steps; ++i)
      decoder.step();
  }
  // Vector of the protected-mode handler that is about to run, or kNone.
  uint32_t pmVector() const {
    const uint32_t eip = cpu.getEIP();
    if (cpu.getSegReg(cpu::CS) != 0x08 || eip < kHandlers ||
        eip >= kHandlers + 32 * 16 || (eip & 15))
      return kNone;
    return (eip - kHandlers) / 16;
  }
  uint32_t stack(unsigned index) const {
    return mem.read32(cpu.getSegBase(cpu::SS) + cpu.getReg32(cpu::ESP) +
                      4 * index);
  }
  void denyPort(uint16_t port) {
    const uint32_t a = kTss + kIopb + port / 8u;
    mem.write8(a, static_cast<uint8_t>(mem.read8(a) | (1u << (port & 7))));
  }
};

} // namespace

// ── x87 availability ────────────────────────────────────────────────────

TEST_CASE("Protection: #NM from CR0.EM, CR0.TS and MP+WAIT", "[CPU][Protection][NM]") {
  SECTION("EM=1: every x87 instruction faults, WAIT does not") {
    Env env;
    env.cpu.setCR(0, env.cpu.getCR(0) | 0x4);
    env.runReal({0xD9, 0xE8}); // FLD1
    REQUIRE(env.realVector() == 7);
    REQUIRE(env.mem.read16(env.cpu.getReg16(cpu::SP)) == 0); // faulting IP
    REQUIRE(env.cpu.fpu().top() == 0);                      // nothing pushed

    Env w;
    w.cpu.setCR(0, w.cpu.getCR(0) | 0x4);
    w.runReal({0x9B, 0x90}); // FWAIT
    REQUIRE(w.realVector() == kNone);
    REQUIRE(w.cpu.getEIP() == 1);
  }

  SECTION("TS=1 faults, CLTS clears TS and the instruction runs") {
    Env env;
    env.cpu.setCR(0, env.cpu.getCR(0) | 0x8);
    env.runReal({0x66, 0xDB, 0xE3}); // FNINIT with a prefix
    REQUIRE(env.realVector() == 7);
    REQUIRE(env.mem.read16(env.cpu.getReg16(cpu::SP)) == 0); // prefix included

    Env ok;
    ok.cpu.setCR(0, ok.cpu.getCR(0) | 0x8);
    ok.runReal({0x0F, 0x06, 0xD9, 0xE8}, 2); // CLTS; FLD1
    REQUIRE(ok.realVector() == kNone);
    REQUIRE((ok.cpu.getCR(0) & 0x8) == 0);
    REQUIRE(ok.cpu.getFPURegister(0) == 1.0);
  }

  SECTION("WAIT faults only with TS and MP") {
    for (uint32_t bits : {0x8u, 0x2u, 0xAu}) {
      Env env;
      env.cpu.setCR(0, env.cpu.getCR(0) | bits);
      env.runReal({0x9B});
      REQUIRE((env.realVector() == 7) == (bits == 0xA));
    }
  }

  SECTION("Predecoded x87 instructions are gated too") {
    Env env;
    // Decode FLD1 on the fast path first, then set TS.
    env.runReal({0xD9, 0xE8, 0x9B});
    env.cpu.setEIP(0);
    env.cpu.setCR(0, env.cpu.getCR(0) | 0xA);
    env.decoder.run(1);
    REQUIRE(env.realVector() == 7);
    Env w;
    w.runReal({0x9B, 0x9B});
    w.cpu.setEIP(0);
    w.cpu.setCR(0, w.cpu.getCR(0) | 0xA);
    w.decoder.run(1);
    REQUIRE(w.realVector() == 7);
  }

  SECTION("Protected mode: #NM through the IDT, no error code") {
    Env env;
    env.enterProtected();
    env.enterRing(3);
    env.cpu.setCR(0, env.cpu.getCR(0) | 0x4);
    env.run({0xDC, 0xC1}); // FADD ST(1), ST
    REQUIRE(env.pmVector() == 7);
    REQUIRE(env.stack(0) == Env::kCode);
    REQUIRE(env.stack(1) == 0x1B);
  }

  SECTION("No coprocessor: x87 instructions do nothing") {
    Env env(CpuModelId::I386DX);
    env.cpu.setFpuPresent(false);
    REQUIRE((env.cpu.getCR(0) & 0x10) == 0); // CR0.ET clear
    env.mem.write16(0x600, 0x5A5A);
    // FNINIT; FNSTSW [0600h]; FNSTCW [0602h]
    env.runReal({0xDB, 0xE3, 0xDD, 0x3E, 0x00, 0x06, 0xD9, 0x3E, 0x02, 0x06},
                3);
    REQUIRE(env.realVector() == kNone);
    REQUIRE(env.cpu.getEIP() == 10);
    REQUIRE(env.mem.read16(0x600) == 0x5A5A);
  }
}

// ── privileged instructions ─────────────────────────────────────────────

TEST_CASE("Protection: privileged instructions at CPL 3 raise #GP(0)",
          "[CPU][Protection][GP]") {
  const std::initializer_list<std::initializer_list<uint8_t>> privileged = {
      {0xF4},                         // HLT
      {0x0F, 0x01, 0x15, 0, 0x60, 0, 0}, // LGDT [6000h]
      {0x0F, 0x01, 0x1D, 0, 0x60, 0, 0}, // LIDT [6000h]
      {0x0F, 0x00, 0xD0},             // LLDT AX
      {0x0F, 0x00, 0xD8},             // LTR AX
      {0x0F, 0x01, 0xF0},             // LMSW AX
      {0x0F, 0x06},                   // CLTS
      {0x0F, 0x20, 0xC0},             // MOV EAX, CR0
      {0x0F, 0x22, 0xD8},             // MOV CR3, EAX
      {0x0F, 0x21, 0xC0},             // MOV EAX, DR0
      {0x0F, 0x23, 0xF8},             // MOV DR7, EAX
      {0x0F, 0x08},                   // INVD
      {0x0F, 0x09},                   // WBINVD
      {0x0F, 0x01, 0x3D, 0, 0x60, 0, 0}, // INVLPG [6000h]
      {0x0F, 0x30},                   // WRMSR
      {0x0F, 0x32},                   // RDMSR
  };
  for (auto code : privileged) {
    Env env;
    env.enterProtected();
    env.enterRing(3);
    env.run(code);
    REQUIRE(env.pmVector() == 13);
    REQUIRE(env.stack(0) == 0); // error code
    REQUIRE(env.stack(1) == Env::kCode);
  }
  SECTION("RDTSC faults at CPL 3 only with CR4.TSD") {
    Env env;
    env.enterProtected();
    env.enterRing(3);
    env.run({0x0F, 0x31});
    REQUIRE(env.pmVector() == kNone);
    env.cpu.setCR(4, 0x4);
    env.cpu.setEIP(Env::kCode);
    env.run({0x0F, 0x31});
    REQUIRE(env.pmVector() == 13);
  }
  SECTION("V86 mode is CPL 3: HLT and CLTS fault, LTR is #UD") {
    for (auto code : {std::initializer_list<uint8_t>{0xF4},
                      std::initializer_list<uint8_t>{0x0F, 0x06}}) {
      Env env;
      env.enterProtected();
      env.enterV86(3);
      env.run(code);
      REQUIRE(env.pmVector() == 13);
    }
    Env env;
    env.enterProtected();
    env.enterV86(3);
    env.run({0x0F, 0x00, 0xD8});
    REQUIRE(env.pmVector() == 6);
  }
  SECTION("CPL 0 runs them") {
    Env env;
    env.enterProtected();
    env.enterRing(0);
    env.cpu.setCR(0, env.cpu.getCR(0) | 0x8);
    env.run({0x0F, 0x06, 0x0F, 0x20, 0xC0}, 2); // CLTS; MOV EAX, CR0
    REQUIRE(env.pmVector() == kNone);
    REQUIRE((env.cpu.getReg32(cpu::EAX) & 0x9) == 0x1);
  }
  SECTION("LTR checks the descriptor and marks the TSS busy") {
    Env env;
    env.enterProtected();
    env.enterRing(0);
    env.descriptor(Env::kTss2Sel, Env::kTss2, 0x67, 0x89);
    env.cpu.setReg32(cpu::EAX, Env::kTss2Sel);
    env.run({0x0F, 0x00, 0xD8}); // LTR AX
    REQUIRE(env.pmVector() == kNone);
    REQUIRE(env.cpu.getTRSelector() == Env::kTss2Sel);
    REQUIRE(env.busy(Env::kTss2Sel));
    // Loading a busy TSS (or a data segment) is #GP(selector).
    env.cpu.setEIP(Env::kCode);
    env.run({0x0F, 0x00, 0xD8});
    REQUIRE(env.pmVector() == 13);
    REQUIRE(env.stack(0) == Env::kTss2Sel);
  }
}

// ── I/O permission bitmap ───────────────────────────────────────────────

TEST_CASE("Protection: I/O permission bitmap", "[CPU][Protection][IOPB]") {
  SECTION("CPL 3 > IOPL: allowed and denied ports, multi-byte spans") {
    struct Case {
      std::initializer_list<uint8_t> code;
      uint16_t dx;
      bool allowed;
    };
    const Case cases[] = {
        {{0xEE}, 0x3F9, true},             // OUT DX, AL
        {{0xEE}, 0x3F8, false},            // denied port
        {{0xE6, 0xE5}, 0, true},           // OUT E5h, AL
        {{0xE6, 0xE8}, 0, false},          // OUT E8h, AL (denied)
        {{0x66, 0xEF}, 0x3F7, false},      // OUT DX, AX: 3F7-3F8
        {{0x66, 0xEF}, 0x3F9, true},       // 3F9-3FA
        {{0xEF}, 0x3F5, false},            // OUT DX, EAX: 3F5-3F8
        {{0xEF}, 0x3FA, true},             // 3FA-3FD
        {{0xED}, 0x3F6, false},            // IN EAX, DX: 3F6-3F9
        {{0xEC}, 0x3F0, true},             // IN AL, DX
        {{0xEF}, 0x00E6, false},           // E6-E9 crosses into E8
    };
    for (const Case &c : cases) {
      Env env;
      env.enterProtected();
      env.denyPort(0x3F8);
      env.denyPort(0xE8);
      env.enterRing(3);
      env.cpu.setReg32(cpu::EDX, c.dx);
      env.cpu.setReg32(cpu::EAX, 0x11223344);
      env.run(c.code);
      REQUIRE((env.pmVector() == kNone) == c.allowed);
      if (!c.allowed) {
        REQUIRE(env.stack(0) == 0);
        REQUIRE(env.device.writes.empty());
        REQUIRE(env.device.reads == 0);
      }
    }
  }

  SECTION("Bitmap beyond the TSS limit denies the port") {
    Env env;
    env.enterProtected();
    env.cpu.setTR({static_cast<uint16_t>(Env::kIopb + 0x40), Env::kTss});
    env.enterRing(3);
    env.cpu.setReg32(cpu::EDX, 0x1FF); // byte 3Fh: inside
    env.run({0xEC});
    REQUIRE(env.pmVector() == kNone);
    env.cpu.setReg32(cpu::EDX, 0x3F0); // byte 7Eh: outside
    env.cpu.setEIP(Env::kCode);
    env.run({0xEC});
    REQUIRE(env.pmVector() == 13);
  }

  SECTION("CPL <= IOPL skips the bitmap; V86 always uses it") {
    Env env;
    env.enterProtected();
    env.denyPort(0x3F8);
    env.enterRing(3, 0x3202); // IOPL 3
    env.cpu.setReg32(cpu::EDX, 0x3F8);
    env.run({0xEE});
    REQUIRE(env.pmVector() == kNone);
    REQUIRE(env.device.writes.size() == 1);

    Env v86;
    v86.enterProtected();
    v86.denyPort(0x3F8);
    v86.enterV86(3);
    v86.cpu.setReg32(cpu::EDX, 0x3F8);
    v86.run({0xEE});
    REQUIRE(v86.pmVector() == 13);
    REQUIRE(v86.stack(0) == 0);
    v86.cpu.setReg32(cpu::EDX, 0x3F9);
    v86.enterV86(3);
    v86.run({0xEE});
    REQUIRE(v86.pmVector() == kNone);
  }

  SECTION("A 16-bit TSS has no bitmap") {
    Env env;
    env.enterProtected();
    env.descriptor(Env::kTss16Sel, Env::kTss16, 0x2B, 0x83);
    env.cpu.setTRSelector(Env::kTss16Sel);
    env.cpu.setTR({0x2B, Env::kTss16});
    env.mem.write16(Env::kTss16 + 2, 0x9000); // SP0
    env.mem.write16(Env::kTss16 + 4, 0x10);   // SS0
    env.enterRing(3);
    env.cpu.setReg32(cpu::EDX, 0x3F9);
    env.run({0xEC});
    REQUIRE(env.pmVector() == 13);
  }

  SECTION("REP OUTSB / REP INSW; a denied REP OUTS keeps ECX") {
    Env env;
    env.enterProtected();
    env.enterRing(3);
    for (uint32_t i = 0; i < 4; ++i)
      env.mem.write8(0x6000 + i, static_cast<uint8_t>(0xA0 + i));
    env.cpu.setReg32(cpu::ESI, 0x6000);
    env.cpu.setReg32(cpu::EDI, 0x6100);
    env.cpu.setReg32(cpu::ECX, 4);
    env.cpu.setReg32(cpu::EDX, 0x3F2);
    env.run({0xF3, 0x6E}); // REP OUTSB
    REQUIRE(env.cpu.getReg32(cpu::ECX) == 0);
    REQUIRE(env.cpu.getReg32(cpu::ESI) == 0x6004);
    REQUIRE(env.device.writes.size() == 4);
    REQUIRE(env.device.writes[3].second == 0xA3);
    env.cpu.setReg32(cpu::ECX, 2);
    env.run({0xF3, 0x66, 0x6D}, 0); // REP INSW
    env.cpu.setEIP(Env::kCode);
    env.decoder.step();
    REQUIRE(env.cpu.getReg32(cpu::EDI) == 0x6104);
    REQUIRE(env.mem.read16(0x6100) == 0x4342);
    REQUIRE(env.mem.read16(0x6102) == 0x4342);

    Env denied;
    denied.enterProtected();
    denied.denyPort(0x3F2);
    denied.enterRing(3);
    denied.cpu.setReg32(cpu::ESI, 0x6000);
    denied.cpu.setReg32(cpu::ECX, 4);
    denied.cpu.setReg32(cpu::EDX, 0x3F2);
    denied.run({0xF3, 0x6E});
    REQUIRE(denied.pmVector() == 13);
    REQUIRE(denied.cpu.getReg32(cpu::ECX) == 4);
    REQUIRE(denied.cpu.getReg32(cpu::ESI) == 0x6000);
    REQUIRE(denied.device.writes.empty());
  }

  SECTION("Real mode OUTSW with DF=1") {
    Env env;
    env.mem.write16(0x600, 0xBEEF);
    env.cpu.setReg32(cpu::ESI, 0x600);
    env.cpu.setReg32(cpu::EDX, 0x3F0);
    env.cpu.setEFLAGS(0x402);
    env.runReal({0x6F});
    REQUIRE(env.cpu.getReg16(cpu::SI) == 0x5FE);
    REQUIRE(env.device.writes.size() == 2);
    REQUIRE(env.device.writes[0].second == 0xEF);
    REQUIRE(env.device.writes[1].second == 0xBE);
  }
}

// ── debug registers ─────────────────────────────────────────────────────

TEST_CASE("Protection: debug breakpoints and single-step", "[CPU][Protection][Debug]") {
  SECTION("Instruction breakpoint: fault before the instruction, B0 in DR6") {
    Env env;
    env.cpu.setDR(0, (Env::kRealCode << 4) + 1);
    env.cpu.setDR(7, 0x00000001); // L0, execute
    env.runReal({0x90, 0x40, 0x40}, 2);
    REQUIRE(env.realVector() == 1);
    REQUIRE(env.mem.read16(env.cpu.getReg16(cpu::SP)) == 1); // faulting IP
    REQUIRE(env.cpu.getReg16(cpu::AX) == 0);                 // INC not run
    REQUIRE((env.cpu.getDR(6) & 0xF) == 0x1);
  }

  SECTION("RF from the handler's IRETD lets the instruction run once") {
    Env env;
    env.enterProtected();
    env.enterRing(0);
    env.cpu.setDR(1, Env::kCode);
    env.cpu.setDR(7, 0x00000008); // G1, execute
    // #DB handler: OR dword [ESP+8], 10000h (RF); IRETD
    const uint32_t h = Env::kHandlers + 16;
    const uint8_t handler[] = {0x81, 0x4C, 0x24, 0x08, 0x00, 0x00, 0x01, 0x00, 0xCF};
    for (uint32_t i = 0; i < sizeof(handler); ++i)
      env.mem.write8(h + i, handler[i]);
    env.run({0x40, 0x40}); // INC EAX; INC EAX
    REQUIRE(env.pmVector() == 1);
    env.decoder.step(); // OR
    env.decoder.step(); // IRETD (loads RF)
    REQUIRE(env.cpu.getEIP() == Env::kCode);
    REQUIRE((env.cpu.getEFLAGS() & 0x10000) != 0);
    env.decoder.step(); // INC EAX runs, RF is cleared
    REQUIRE(env.cpu.getReg32(cpu::EAX) == 1);
    REQUIRE((env.cpu.getEFLAGS() & 0x10000) == 0);
    REQUIRE((env.cpu.getDR(6) & 0xF) == 0x2);
  }

  SECTION("Data breakpoints: write-only and read/write, length and trap EIP") {
    struct Case {
      std::initializer_list<uint8_t> code;
      uint32_t dr7;
      bool hit;
    };
    // DR2 = 6002h; R/W2 at bits 24-25, LEN2 at bits 26-27, L2 = bit 4.
    const Case cases[] = {
        {{0xA2, 0x00, 0x60, 0, 0}, 0x0D000010, true},   // MOV [6000h], AL: LEN 4 W
        {{0xA2, 0x01, 0x60, 0, 0}, 0x01000010, false},  // [6001h], LEN 1 W at 6002h
        {{0xA2, 0x02, 0x60, 0, 0}, 0x01000010, true},   // [6002h]
        {{0xA0, 0x02, 0x60, 0, 0}, 0x01000010, false},  // read, write-only BP
        {{0xA0, 0x02, 0x60, 0, 0}, 0x03000010, true},   // read, R/W BP
        {{0x66, 0xA3, 0xFF, 0x5F, 0, 0}, 0x03000010, false}, // [5FFFh] word
        {{0xA3, 0x00, 0x60, 0, 0}, 0x01000010, true},   // dword at 6000h
    };
    for (const Case &c : cases) {
      Env env;
      env.enterProtected();
      env.enterRing(0);
      env.cpu.setDR(2, 0x6002);
      env.cpu.setDR(7, c.dr7);
      env.run(c.code);
      REQUIRE((env.pmVector() == 1) == c.hit);
      if (c.hit) {
        REQUIRE(env.stack(0) == Env::kCode + c.code.size()); // trap
        REQUIRE((env.cpu.getDR(6) & 0xF) == 0x4);
      }
    }
  }

  SECTION("Instruction fetches do not hit data breakpoints") {
    Env env;
    env.enterProtected();
    env.enterRing(0);
    env.cpu.setDR(0, Env::kCode);
    env.cpu.setDR(7, 0x000F0001); // L0, R/W 11, LEN 4
    env.run({0x90});
    REQUIRE(env.pmVector() == kNone);
  }

  SECTION("Data breakpoints are seen by fast-path bursts and REP STOS") {
    Env env;
    env.enterProtected();
    env.enterRing(0);
    // 20 x MOV [6000h+i*4], EAX on the predecoded path, then REP STOSD.
    std::vector<uint8_t> code;
    for (int i = 0; i < 20; ++i)
      code.insert(code.end(), {0xA3, static_cast<uint8_t>(i * 4), 0x60, 0, 0});
    for (size_t i = 0; i < code.size(); ++i)
      env.mem.write8(Env::kCode + static_cast<uint32_t>(i), code[i]);
    env.decoder.run(25); // decode everything without the breakpoint
    env.cpu.setEIP(Env::kCode);
    env.cpu.setDR(3, 0x6028);
    env.cpu.setDR(7, 0xD0000040); // L3, write, LEN 4
    env.decoder.run(11);
    REQUIRE(env.pmVector() == 1);
    REQUIRE(env.stack(0) == Env::kCode + 11 * 5);
    REQUIRE((env.cpu.getDR(6) & 0xF) == 0x8);

    Env rep;
    rep.enterProtected();
    rep.enterRing(0);
    rep.cpu.setReg32(cpu::EDI, 0x6000);
    rep.cpu.setReg32(cpu::ECX, 64);
    rep.cpu.setDR(3, 0x6080);
    rep.cpu.setDR(7, 0x50000040); // L3, write, LEN 2
    rep.run({0xF3, 0xAB});
    REQUIRE(rep.pmVector() == 1);
  }

  SECTION("TF single-step traps after each instruction with BS") {
    Env env;
    env.cpu.setEFLAGS(0x102);
    env.runReal({0x40, 0x40});
    REQUIRE(env.realVector() == 1);
    REQUIRE(env.mem.read16(env.cpu.getReg16(cpu::SP)) == 1); // next IP
    REQUIRE((env.cpu.getDR(6) & 0x4000) != 0);
    REQUIRE((env.cpu.getEFLAGS() & 0x100) == 0); // handler runs without TF
    // The pushed FLAGS keep TF.
    REQUIRE((env.mem.read16(env.cpu.getReg16(cpu::SP) + 4) & 0x100) != 0);
  }

  SECTION("POPF setting TF traps after the next instruction") {
    Env env;
    env.cpu.push16(0x0102);
    env.runReal({0x9D, 0x40, 0x40}, 1);
    REQUIRE(env.realVector() == kNone);
    env.decoder.step();
    REQUIRE(env.realVector() == 1);
    REQUIRE(env.cpu.getReg16(cpu::AX) == 1);
  }

  SECTION("General detect, DR4/DR5 aliases, CR4.DE") {
    Env env;
    env.enterProtected();
    env.enterRing(0);
    env.cpu.setDR(7, 0x2000);
    env.run({0x0F, 0x21, 0xC0}); // MOV EAX, DR0
    REQUIRE(env.pmVector() == 1);
    REQUIRE(env.stack(0) == Env::kCode); // fault
    REQUIRE((env.cpu.getDR(6) & 0x2000) != 0);
    REQUIRE((env.cpu.getDR(7) & 0x2000) == 0);

    Env alias;
    alias.enterProtected();
    alias.enterRing(0);
    alias.cpu.setDR(6, 0x4001);
    alias.run({0x0F, 0x21, 0xE0}); // MOV EAX, DR4 (= DR6)
    REQUIRE(alias.cpu.getReg32(cpu::EAX) == 0xFFFF4FF1u);
    alias.cpu.setCR(4, 0x8); // DE
    alias.cpu.setEIP(Env::kCode);
    alias.run({0x0F, 0x21, 0xE0});
    REQUIRE(alias.pmVector() == 6);
  }

  SECTION("ICEBP (F1) is a #DB trap") {
    Env env;
    env.runReal({0xF1});
    REQUIRE(env.realVector() == 1);
    REQUIRE(env.mem.read16(env.cpu.getReg16(cpu::SP)) == 1);
  }
}

// ── hardware task switches ──────────────────────────────────────────────

namespace {

constexpr uint32_t kTaskBCode = 0x5800;

// Task B: a 32-bit TSS (available) whose code starts at kTaskBCode.
void setupTaskB(Env &env, uint32_t flags = 0x2) {
  const uint32_t b = Env::kTss2;
  for (uint32_t i = 0; i < 0x68; ++i)
    env.mem.write8(b + i, 0);
  env.mem.write32(b + 0x04, Env::kRing0Stack - 0x400); // ESP0
  env.mem.write32(b + 0x08, 0x10);                     // SS0
  env.mem.write32(b + 0x20, kTaskBCode);               // EIP
  env.mem.write32(b + 0x24, flags);                    // EFLAGS
  env.mem.write32(b + 0x28, 0xB0B0B0B0);               // EAX
  env.mem.write32(b + 0x38, 0x7800);                   // ESP
  env.mem.write32(b + 0x48, 0x10);                     // ES
  env.mem.write32(b + 0x4C, 0x08);                     // CS
  env.mem.write32(b + 0x50, 0x10);                     // SS
  env.mem.write32(b + 0x54, 0x10);                     // DS
  env.mem.write32(b + 0x58, 0x10);                     // FS
  env.mem.write32(b + 0x5C, 0x10);                     // GS
  env.mem.write16(b + 0x66, 0x68);
  env.descriptor(Env::kTss2Sel, b, 0x67, 0x89);
  env.descriptor(Env::kTaskGateSel, 0, 0, 0xE5); // task gate, DPL 3
  env.mem.write16(Env::kGdt + Env::kTaskGateSel + 2, Env::kTss2Sel);
}

void writeCode(Env &env, uint32_t at, std::initializer_list<uint8_t> code) {
  for (uint8_t b : code)
    env.mem.write8(at++, b);
}

} // namespace

TEST_CASE("Protection: hardware task switches", "[CPU][Protection][Task]") {
  SECTION("CALL to a TSS: state saved, back link, NT, busy bits; IRET returns") {
    Env env;
    env.enterProtected();
    env.enterRing(0, 0x202);
    setupTaskB(env);
    env.cpu.setReg32(cpu::EAX, 0xA0A0A0A0);
    writeCode(env, kTaskBCode, {0x40, 0xCF}); // INC EAX; IRETD
    env.run({0x9A, 0, 0, 0, 0, Env::kTss2Sel, 0}); // CALL FAR 30h:0
    REQUIRE(env.cpu.getTRSelector() == Env::kTss2Sel);
    REQUIRE(env.cpu.getEIP() == kTaskBCode);
    REQUIRE(env.cpu.getReg32(cpu::EAX) == 0xB0B0B0B0);
    REQUIRE(env.cpu.getReg32(cpu::ESP) == 0x7800);
    REQUIRE((env.cpu.getEFLAGS() & 0x4000) != 0); // NT
    REQUIRE((env.cpu.getCR(0) & 0x8) != 0);       // TS
    REQUIRE(env.busy(Env::kTssSel));
    REQUIRE(env.busy(Env::kTss2Sel));
    REQUIRE(env.mem.read16(Env::kTss2) == Env::kTssSel); // back link
    REQUIRE(env.mem.read32(Env::kTss + 0x20) == Env::kCode + 7);
    REQUIRE(env.mem.read32(Env::kTss + 0x28) == 0xA0A0A0A0);
    REQUIRE(env.mem.read32(Env::kTss + 0x24) == 0x202);
    env.decoder.step(); // INC EAX
    env.decoder.step(); // IRETD with NT: back to task A
    REQUIRE(env.cpu.getTRSelector() == Env::kTssSel);
    REQUIRE(env.cpu.getEIP() == Env::kCode + 7);
    REQUIRE(env.cpu.getReg32(cpu::EAX) == 0xA0A0A0A0);
    REQUIRE(env.cpu.getEFLAGS() == 0x202);
    REQUIRE(env.busy(Env::kTssSel));
    REQUIRE(!env.busy(Env::kTss2Sel));
    // Task B's state was saved with NT clear.
    REQUIRE(env.mem.read32(Env::kTss2 + 0x20) == kTaskBCode + 2);
    REQUIRE(env.mem.read32(Env::kTss2 + 0x28) == 0xB0B0B0B1);
    REQUIRE((env.mem.read32(Env::kTss2 + 0x24) & 0x4000) == 0);
  }

  SECTION("JMP to a TSS clears the old busy bit and sets no back link") {
    Env env;
    env.enterProtected();
    env.enterRing(0);
    setupTaskB(env);
    env.mem.write16(Env::kTss2, 0x1234);
    env.run({0xEA, 0, 0, 0, 0, Env::kTss2Sel, 0}); // JMP FAR 30h:0
    REQUIRE(env.cpu.getTRSelector() == Env::kTss2Sel);
    REQUIRE(!env.busy(Env::kTssSel));
    REQUIRE(env.busy(Env::kTss2Sel));
    REQUIRE(env.mem.read16(Env::kTss2) == 0x1234);
    REQUIRE((env.cpu.getEFLAGS() & 0x4000) == 0);
  }

  SECTION("CALL to a busy TSS is #GP(selector); a task gate in the GDT works") {
    Env env;
    env.enterProtected();
    env.enterRing(0);
    setupTaskB(env);
    env.run({0x9A, 0, 0, 0, 0, Env::kTssSel, 0}); // CALL FAR 28h:0 (busy)
    REQUIRE(env.pmVector() == 13);
    REQUIRE(env.stack(0) == Env::kTssSel);

    Env gate;
    gate.enterProtected();
    gate.enterRing(3);
    setupTaskB(gate);
    gate.run({0xEA, 0, 0, 0, 0, Env::kTaskGateSel | 3, 0}); // JMP FAR 43h:0
    REQUIRE(gate.cpu.getTRSelector() == Env::kTss2Sel);
    REQUIRE(gate.cpu.getSegReg(cpu::CS) == 0x08);
    REQUIRE(gate.cpu.getEIP() == kTaskBCode);
  }

  SECTION("INT n and exceptions through IDT task gates") {
    Env env;
    env.enterProtected();
    env.enterRing(3);
    setupTaskB(env);
    // INT 21h through a DPL 3 task gate.
    env.gate(0x21, 0, 0xE5, Env::kTss2Sel);
    env.run({0xCD, 0x21});
    REQUIRE(env.cpu.getTRSelector() == Env::kTss2Sel);
    REQUIRE(env.mem.read16(Env::kTss2) == Env::kTssSel);
    REQUIRE((env.cpu.getEFLAGS() & 0x4000) != 0);
    REQUIRE(env.mem.read32(Env::kTss + 0x20) == Env::kCode + 2);
    REQUIRE(env.cpu.getReg32(cpu::ESP) == 0x7800); // no error code

    // #GP through a task gate pushes the error code on the new stack.
    Env fault;
    fault.enterProtected();
    fault.enterRing(3);
    setupTaskB(fault);
    fault.gate(13, 0, 0x85, Env::kTss2Sel);
    fault.run({0xF4}); // HLT at CPL 3
    REQUIRE(fault.cpu.getTRSelector() == Env::kTss2Sel);
    REQUIRE(fault.cpu.getReg32(cpu::ESP) == 0x7800 - 4);
    REQUIRE(fault.stack(0) == 0);
    // The faulting task resumes at the HLT.
    REQUIRE(fault.mem.read32(Env::kTss + 0x20) == Env::kCode);

    // INT n to a DPL 0 task gate from CPL 3 is #GP(vector * 8 + 2).
    Env denied;
    denied.enterProtected();
    denied.enterRing(3);
    setupTaskB(denied);
    denied.gate(0x21, 0, 0x85, Env::kTss2Sel);
    denied.run({0xCD, 0x21});
    REQUIRE(denied.pmVector() == 13);
    REQUIRE(denied.stack(0) == 0x21 * 8 + 2);
  }

  SECTION("16-bit (286) TSS") {
    Env env;
    env.enterProtected();
    env.enterRing(0, 0x202);
    const uint32_t t = Env::kTss16;
    for (uint32_t i = 0; i < 0x2C; ++i)
      env.mem.write8(t + i, 0);
    env.mem.write16(t + 0x0E, 0x5900);          // IP
    env.mem.write16(t + 0x10, 0x0046);          // FLAGS (ZF PF)
    env.mem.write16(t + 0x12, 0x1616);          // AX
    env.mem.write16(t + 0x1A, 0x7A00);          // SP
    env.mem.write16(t + 0x22, 0x10);            // ES
    env.mem.write16(t + 0x24, 0x08);            // CS
    env.mem.write16(t + 0x26, 0x10);            // SS
    env.mem.write16(t + 0x28, 0x10);            // DS
    env.descriptor(Env::kTss16Sel, t, 0x2B, 0x81);
    env.cpu.setReg32(cpu::EAX, 0x12345678);
    writeCode(env, 0x5900, {0x66, 0xCF}); // IRET (16-bit operand)
    env.run({0x9A, 0, 0, 0, 0, Env::kTss16Sel, 0});
    REQUIRE(env.cpu.getTRSelector() == Env::kTss16Sel);
    REQUIRE(env.cpu.getEIP() == 0x5900);
    REQUIRE(env.cpu.getReg32(cpu::EAX) == 0x12341616);
    REQUIRE((env.cpu.getEFLAGS() & 0x40C6) == 0x4046);
    REQUIRE(env.mem.read16(t) == Env::kTssSel);
    REQUIRE((env.gdtHigh(Env::kTss16Sel) & 0x0F00) == 0x0300); // busy 286 TSS
    env.decoder.step(); // IRET: back to the 32-bit task
    REQUIRE(env.cpu.getTRSelector() == Env::kTssSel);
    REQUIRE(env.cpu.getEIP() == Env::kCode + 7);
    REQUIRE(env.mem.read16(t + 0x0E) == 0x5902);
    REQUIRE(env.mem.read16(t + 0x12) == 0x1616);
  }

  SECTION("CR3 and the LDT are loaded from the new TSS; T bit traps") {
    Env env;
    env.enterProtected();
    env.enterRing(0);
    // Identity-map 4 MB through two page directories sharing one table.
    for (uint32_t i = 0; i < 1024; ++i)
      env.mem.write32(0x11000 + i * 4, (i << 12) | 3);
    env.mem.write32(0x10000, 0x11003);
    env.mem.write32(0x12000, 0x11003);
    env.cpu.setCR(3, 0x10000);
    env.cpu.setCR(0, env.cpu.getCR(0) | 0x80000000u);
    setupTaskB(env);
    env.mem.write32(Env::kTss2 + 0x1C, 0x12000);
    // LDT with one data segment at base 0x20000 (selector 0x04|3? no: 0x04).
    env.descriptor(0x48, 0x14000, 0x0F, 0x82);
    env.mem.write32(0x14000, 0x0000FFFF);
    env.mem.write32(0x14004, 0x00CF9202 | 0); // base 0x020000
    env.mem.write32(Env::kTss2 + 0x60, 0x48);
    env.mem.write32(Env::kTss2 + 0x54, 0x04); // DS = LDT selector 0
    env.mem.write16(Env::kTss2 + 0x64, 1);    // T bit
    env.run({0xEA, 0, 0, 0, 0, Env::kTss2Sel, 0});
    REQUIRE(env.cpu.getCR(3) == 0x12000);
    REQUIRE(env.cpu.getLDTRSelector() == 0x48);
    REQUIRE(env.cpu.getSegBase(cpu::DS) == 0x20000);
    REQUIRE(env.pmVector() == 1); // T bit: #DB before the first instruction
    REQUIRE(env.stack(0) == kTaskBCode);
    REQUIRE((env.cpu.getDR(6) & 0x8000) != 0);
  }

  SECTION("A task with VM set starts in virtual-8086 mode") {
    Env env;
    env.enterProtected();
    env.enterRing(0);
    setupTaskB(env, 0x00020202);
    env.mem.write32(Env::kTss2 + 0x4C, 0x0580); // CS paragraph
    env.mem.write32(Env::kTss2 + 0x20, 0x0010);
    env.run({0xEA, 0, 0, 0, 0, Env::kTss2Sel, 0});
    REQUIRE((env.cpu.getEFLAGS() & 0x00020000) != 0);
    REQUIRE(env.cpu.getSegBase(cpu::CS) == 0x5800);
    REQUIRE(env.cpu.getEIP() == 0x10);
  }
}

// ── virtual-8086 mode ───────────────────────────────────────────────────

TEST_CASE("Protection: V86 INT n / IRETD round trip through the monitor",
          "[CPU][Protection][V86]") {
  Env env;
  env.enterProtected();
  constexpr uint32_t kMonitor = 0x8400;
  env.gate(0x21, kMonitor); // DPL 3 interrupt gate
  writeCode(env, kMonitor, {0xCF}); // IRETD
  env.enterV86(3);
  for (auto s : {cpu::DS, cpu::ES, cpu::FS, cpu::GS})
    env.decoder.loadSegment(s, static_cast<uint16_t>(0x1000 + s));
  env.cpu.setReg32(cpu::ESP, 0x6FF0);
  env.run({0xCD, 0x21, 0x40});
  REQUIRE(env.cpu.getSegReg(cpu::CS) == 0x08);
  REQUIRE(env.cpu.getEIP() == kMonitor);
  REQUIRE((env.cpu.getEFLAGS() & 0x00020000) == 0);
  // The V86 frame: EIP CS EFLAGS ESP SS ES DS FS GS; data segments nulled.
  REQUIRE(env.stack(0) == 2);
  REQUIRE(env.stack(1) == (Env::kCode >> 4));
  REQUIRE((env.stack(2) & 0x00023000) == 0x00023000);
  REQUIRE(env.stack(3) == 0x6FF0);
  REQUIRE(env.stack(4) == 0);
  REQUIRE(env.stack(5) == 0x1000 + cpu::ES);
  REQUIRE(env.stack(6) == 0x1000 + cpu::DS);
  REQUIRE(env.stack(7) == 0x1000 + cpu::FS);
  REQUIRE(env.stack(8) == 0x1000 + cpu::GS);
  REQUIRE(env.cpu.getSegReg(cpu::DS) == 0);
  REQUIRE(env.cpu.getReg32(cpu::ESP) == Env::kRing0Stack - 36);
  env.decoder.step(); // IRETD back to V86
  REQUIRE((env.cpu.getEFLAGS() & 0x00020000) != 0);
  REQUIRE(env.cpu.getSegReg(cpu::CS) == (Env::kCode >> 4));
  REQUIRE(env.cpu.getEIP() == 2);
  REQUIRE(env.cpu.getReg32(cpu::ESP) == 0x6FF0);
  REQUIRE(env.cpu.getSegReg(cpu::DS) == 0x1000 + cpu::DS);
  REQUIRE(env.cpu.getSegBase(cpu::GS) == (0x1000u + cpu::GS) << 4);
  env.decoder.step(); // INC AX in V86
  REQUIRE(env.cpu.getReg16(cpu::AX) == 1);

  SECTION("INT3 is not IOPL-sensitive; INT n to a DPL 0 gate is #GP") {
    Env e;
    e.enterProtected();
    e.enterV86(0);
    e.run({0xCC});
    REQUIRE(e.pmVector() == 3);
    Env g;
    g.enterProtected();
    g.gate(0x21, kMonitor, 0x8E);
    g.enterV86(3);
    g.run({0xCD, 0x21});
    REQUIRE(g.pmVector() == 13);
    REQUIRE(g.stack(0) == 0x21 * 8 + 2);
  }
  SECTION("V86 segment loads are paragraphs; LAR and ARPL are #UD") {
    Env e;
    e.enterProtected();
    e.enterV86(3);
    e.cpu.setReg32(cpu::EAX, 0x1234);
    e.run({0x8E, 0xD8}); // MOV DS, AX
    REQUIRE(e.cpu.getSegBase(cpu::DS) == 0x12340);
    Env u;
    u.enterProtected();
    u.enterV86(3);
    u.run({0x0F, 0x02, 0xC0}); // LAR AX, AX
    REQUIRE(u.pmVector() == 6);
  }
}

TEST_CASE("Protection: VME and PVI virtual interrupt flag", "[CPU][Protection][VME]") {
  constexpr uint32_t kVIF = 0x00080000, kVIP = 0x00100000, kIF = 0x200;
  auto vme = [](Env &env, uint32_t extraFlags = 0) {
    env.enterProtected();
    env.cpu.setCR(4, 0x1);
    env.enterV86(0, extraFlags);
  };

  SECTION("CLI/STI change VIF, not IF; STI with VIP faults") {
    Env env;
    vme(env, kVIF);
    env.run({0xFA});
    REQUIRE(env.pmVector() == kNone);
    REQUIRE((env.cpu.getEFLAGS() & (kVIF | kIF)) == kIF);
    env.run({0xFA, 0xFB}, 1);
    REQUIRE((env.cpu.getEFLAGS() & kVIF) == kVIF);

    Env pending;
    vme(pending, kVIP);
    pending.run({0xFB});
    REQUIRE(pending.pmVector() == 13);
  }

  SECTION("PUSHF shows VIF as IF and IOPL 3; POPF loads VIF; PUSHFD faults") {
    Env env;
    vme(env, kVIF);
    env.cpu.setEFLAGS(env.cpu.getEFLAGS() & ~kIF);
    env.run({0x9C});
    const uint16_t image = env.mem.read16(env.cpu.getReg32(cpu::ESP));
    REQUIRE((image & 0x3200) == 0x3200);

    Env pop;
    vme(pop);
    pop.cpu.push16(0x0201); // IF, CF
    pop.run({0x9D});
    REQUIRE(pop.pmVector() == kNone);
    REQUIRE((pop.cpu.getEFLAGS() & (kVIF | 0x3001)) == (kVIF | 0x0001));

    Env tf;
    vme(tf);
    tf.cpu.push16(0x0100);
    tf.run({0x9D});
    REQUIRE(tf.pmVector() == 13);

    Env d;
    vme(d);
    d.run({0x66, 0x9C});
    REQUIRE(d.pmVector() == 13);
  }

  SECTION("INT n: redirection bitmap clear -> V86 IVT, set -> monitor") {
    Env env;
    vme(env, kVIF);
    env.mem.write16(0x21 * 4, 0x0040);
    env.mem.write16(0x21 * 4 + 2, 0x0600);
    const uint16_t sp = static_cast<uint16_t>(env.cpu.getReg32(cpu::ESP));
    env.run({0xCD, 0x21});
    REQUIRE((env.cpu.getEFLAGS() & 0x00020000) != 0);
    REQUIRE(env.cpu.getSegReg(cpu::CS) == 0x0600);
    REQUIRE(env.cpu.getEIP() == 0x0040);
    REQUIRE((env.cpu.getEFLAGS() & kVIF) == 0);
    REQUIRE((env.cpu.getEFLAGS() & kIF) == kIF); // the real IF is untouched
    REQUIRE(env.mem.read16(sp - 2) == (0x0200 | 0x3000 | 0x2)); // VIF as IF
    REQUIRE(env.mem.read16(sp - 4) == (Env::kCode >> 4));
    REQUIRE(env.mem.read16(sp - 6) == 2);
    // IRET (16-bit) returns and restores VIF from the popped IF.
    writeCode(env, 0x6040, {0xCF});
    env.decoder.step();
    REQUIRE(env.cpu.getSegReg(cpu::CS) == (Env::kCode >> 4));
    REQUIRE(env.cpu.getEIP() == 2);
    REQUIRE((env.cpu.getEFLAGS() & kVIF) == kVIF);

    Env set;
    vme(set);
    set.mem.write8(Env::kTss + Env::kIopb - 32 + 0x21 / 8, 1u << (0x21 & 7));
    set.run({0xCD, 0x21});
    REQUIRE(set.pmVector() == 13); // IOPL 0: the monitor emulates it
  }

  SECTION("PVI: CLI/STI at CPL 3 change VIF") {
    Env env;
    env.enterProtected();
    env.cpu.setCR(4, 0x2);
    env.enterRing(3, 0x202);
    env.run({0xFA});
    REQUIRE(env.pmVector() == kNone);
    REQUIRE((env.cpu.getEFLAGS() & (kVIF | kIF)) == kIF);
    Env pending;
    pending.enterProtected();
    pending.cpu.setCR(4, 0x2);
    pending.enterRing(3, 0x202 | kVIP);
    pending.run({0xFB});
    REQUIRE(pending.pmVector() == 13);
  }
}
