#include "test_framework.hpp"
#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "memory/MemoryBus.hpp"
#include "hw/IOBus.hpp"
#include "hw/PIC8259.hpp"
#include "hw/BIOS.hpp"
#include "hw/DOS.hpp"
#include "hw/DPMI.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIT8254.hpp"
#include <chrono>

using namespace fador;

TEST_CASE("CPU Interrupt Pipeline", "[Interrupts]") {
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

    // Setup base execution environment
    cpu.setSegReg(cpu::SegRegIndex::CS, 0x1000);
    cpu.setSegReg(cpu::SegRegIndex::DS, 0x0000);
    cpu.setSegReg(cpu::SegRegIndex::SS, 0x0000);
    cpu.setReg16(cpu::Reg16Index::SP, 0x0100);
    cpu.setEIP(0x100);

    // Setup IVT vector 0x77
    mem.write16(0x77 * 4, 0x1234);     // IP = 0x1234
    mem.write16((0x77 * 4) + 2, 0xF000); // CS = 0xF000

    SECTION("Software Interrupt INT imm8 (0xCD 0x77)") {
        mem.write8((0x1000 << 4) + 0x100, 0xCD); // INT
        mem.write8((0x1000 << 4) + 0x101, 0x77); // 0x77

        uint32_t prevEflags = cpu.getEFLAGS();

        decoder.step();

        // Ensure Jumped
        REQUIRE(cpu.getSegReg(cpu::SegRegIndex::CS) == 0xF000);
        REQUIRE(cpu.getEIP() == 0x1234);

        // Check Stack
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SP) == 0x00FA); // 0x100 - 6 bytes pushed
        
        // Stack contains IP, CS, FLAGS
        REQUIRE(mem.read16((0x0000 << 4) + 0x00FA) == 0x0102); // Return IP
        REQUIRE(mem.read16((0x0000 << 4) + 0x00FC) == 0x1000); // Return CS
        REQUIRE(mem.read16((0x0000 << 4) + 0x00FE) == (prevEflags & 0xFFFF)); // Pushed FLAGS
    }

    SECTION("Interrupt Return IRET (0xCF)") {
        // Mock a stack frame: IP=0xABCD, CS=0x1111, FLAGS=0x0002
        cpu.setReg16(cpu::Reg16Index::SP, 0x00FA);
        mem.write16((0x0000 << 4) + 0x00FA, 0xABCD); // return IP
        mem.write16((0x0000 << 4) + 0x00FC, 0x1111); // return CS
        mem.write16((0x0000 << 4) + 0x00FE, 0x0002); // return FLAGS

        mem.write8((0x1000 << 4) + 0x100, 0xCF); // IRET
        decoder.step();

        REQUIRE(cpu.getSegReg(cpu::SegRegIndex::CS) == 0x1111);
        REQUIRE(cpu.getEIP() == 0xABCD);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SP) == 0x0100);
    }
}

TEST_CASE("IRET in HLE stub infers 16-bit return frame width", "[Interrupts][IRET]") {
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

    auto writeDesc = [&](uint32_t addr, uint32_t base, uint32_t limit,
                         uint8_t access, uint8_t flagsNibble) {
        uint32_t low = (limit & 0xFFFFu) | ((base & 0xFFFFu) << 16);
        uint32_t high = ((base >> 16) & 0xFFu) |
                        (static_cast<uint32_t>(access) << 8) |
                        (((limit >> 16) & 0x0Fu) << 16) |
                        ((static_cast<uint32_t>(flagsNibble) & 0x0Fu) << 20) |
                        (base & 0xFF000000u);
        mem.write32(addr, low);
        mem.write32(addr + 4, high);
    };

    // Build a minimal GDT with:
    // 0x08: 32-bit code segment (stub CS)
    // 0x10: 16-bit code segment (return CS)
    // 0x18: 16-bit data/stack segment (SS)
    constexpr uint32_t gdtBase = 0x3000;
    mem.write32(gdtBase + 0, 0);
    mem.write32(gdtBase + 4, 0);
    writeDesc(gdtBase + 0x08, 0x00004000u, 0x000FFFFFu, 0x9Au, 0xCu);
    writeDesc(gdtBase + 0x10, 0x00005000u, 0x000FFFFFu, 0x9Au, 0x8u);
    writeDesc(gdtBase + 0x18, 0x00006000u, 0x0000FFFFu, 0x92u, 0x0u);

    cpu.setGDTR({0x0030, gdtBase});
    cpu.setCR(0, cpu.getCR(0) | 1u); // PE=1

    // Current execution context: HLE stub-like CS=0x08 (USE32), SS=0x18 (16-bit).
    cpu.setSegReg(cpu::CS, 0x0008);
    cpu.setSegBase(cpu::CS, 0x00004000u);
    cpu.setIs32BitCode(true);

    cpu.setSegReg(cpu::SS, 0x0018);
    cpu.setSegBase(cpu::SS, 0x00006000u);
    cpu.setIs32BitStack(false);

    cpu.setEIP(0x0000);
    cpu.setReg16(cpu::SP, 0x0100);

    // 16-bit IRET frame at SS:SP -> IP=0x1234, CS=0x0010, FLAGS=0x0202.
    mem.write16(0x00006000u + 0x0100u, 0x1234);
    mem.write16(0x00006000u + 0x0102u, 0x0010);
    mem.write16(0x00006000u + 0x0104u, 0x0202);

    // Execute IRET opcode in the 32-bit stub context.
    mem.write8(0x00004000u, 0xCF);
    decoder.step();

    // Must infer 16-bit frame width from return CS descriptor (0x10 has D=0).
    REQUIRE(cpu.getSegReg(cpu::CS) == 0x0010);
    REQUIRE(cpu.getEIP() == 0x1234);
    REQUIRE(cpu.getReg16(cpu::SP) == 0x0106);
}

TEST_CASE("0F FF handled interrupt without tracked frame uses chain-return frame", "[Interrupts][Chain]") {
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

    // Execute from a simple RM code segment.
    cpu.setCR(0, 0);
    cpu.setSegReg(cpu::CS, 0x2000);
    cpu.setSegBase(cpu::CS, 0x20000);
    cpu.setSegReg(cpu::SS, 0x0000);
    cpu.setSegBase(cpu::SS, 0x0000);
    cpu.setIs32BitCode(false);
    cpu.setIs32BitStack(false);
    cpu.setEIP(0x0100);
    cpu.setReg16(cpu::SP, 0x0200);

    // Chain frame at SS:SP: IP, CS, FLAGS.
    mem.write16(0x0200, 0x3456);
    mem.write16(0x0202, 0x1234);
    mem.write16(0x0204, 0x0202);

    // 0F FF 15h -> BIOS INT 15h handler (AH=88h succeeds).
    cpu.setReg8(cpu::AH, 0x88);
    mem.write8(0x20000 + 0x0100, 0x0F);
    mem.write8(0x20000 + 0x0101, 0xFF);
    mem.write8(0x20000 + 0x0102, 0x15);

    REQUIRE(cpu.hleStackSize() == 0);
    decoder.step();

    REQUIRE(cpu.getSegReg(cpu::CS) == 0x1234);
    REQUIRE(cpu.getEIP() == 0x3456);
    REQUIRE(cpu.getReg16(cpu::SP) == 0x0206);
    REQUIRE(cpu.hleStackSize() == 0);
}

TEST_CASE("0F FF INT 31h chain path merges only CF into caller flags", "[Interrupts][Chain][DPMI]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::DOS dos(cpu, mem);
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DPMI dpmi(cpu, mem);
    hw::IOBus iobus;

    bios.initialize();
    dos.initialize();
    dpmi.setDOS(&dos);
    dpmi.setBIOS(&bios);
    dos.setDPMI(&dpmi);

    // Enter DPMI host so INT 31h is handled.
    cpu.setSegReg(cpu::CS, 0x1000);
    cpu.setSegReg(cpu::DS, 0x2000);
    cpu.setSegReg(cpu::ES, 0x2000);
    cpu.setSegReg(cpu::SS, 0x0000);
    cpu.setReg16(cpu::SP, 0xFFFC);
    uint16_t sp = cpu.getReg16(cpu::SP);
    sp -= 2; mem.write16(sp, 0x1000);
    sp -= 2; mem.write16(sp, 0x0100);
    cpu.setReg16(cpu::SP, sp);
    cpu.setReg16(cpu::AX, 0x0001);
    REQUIRE(dpmi.handleEntry());

    cpu::InstructionDecoder decoder(cpu, mem, iobus, bios, dos);

    uint32_t ssBase = cpu.getSegBase(cpu::SS);
    uint16_t spBefore = cpu.getReg16(cpu::SP);

    // Caller frame flags have ZF set, CF clear.
    uint16_t frameFlags = 0x0040;
    mem.write16(ssBase + spBefore + 0, 0x2222);
    mem.write16(ssBase + spBefore + 2, cpu.getSegReg(cpu::CS));
    mem.write16(ssBase + spBefore + 4, frameFlags);

    // INT 31h call that should return CF=1 (unsupported function).
    cpu.setReg16(cpu::AX, 0xFFFF);
    uint32_t pre = cpu.getEFLAGS();
    cpu.setEFLAGS(pre | cpu::FLAG_ZERO | cpu::FLAG_SIGN | cpu::FLAG_PARITY);

    uint32_t codePhys = cpu.getSegBase(cpu::CS) + cpu.getEIP();
    mem.write8(codePhys, 0x0F);
    mem.write8(codePhys + 1, 0xFF);
    mem.write8(codePhys + 2, 0x31);

    REQUIRE(cpu.hleStackSize() == 0);
    decoder.step();

    uint16_t outFlags = static_cast<uint16_t>(cpu.getEFLAGS() & 0xFFFF);
    REQUIRE((outFlags & cpu::FLAG_CARRY) != 0);
    REQUIRE((outFlags & cpu::FLAG_ZERO) != 0); // preserved from frame
    REQUIRE((outFlags & cpu::FLAG_SIGN) == 0); // not merged from handler state
    REQUIRE((outFlags & cpu::FLAG_PARITY) == 0); // not merged from handler state
}

TEST_CASE("0F FF chain width falls back to CS at ESP+4", "[Interrupts][Chain][IRET]") {
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

    auto writeDesc = [&](uint32_t addr, uint32_t base, uint32_t limit,
                         uint8_t access, uint8_t flagsNibble) {
        uint32_t low = (limit & 0xFFFFu) | ((base & 0xFFFFu) << 16);
        uint32_t high = ((base >> 16) & 0xFFu) |
                        (static_cast<uint32_t>(access) << 8) |
                        (((limit >> 16) & 0x0Fu) << 16) |
                        ((static_cast<uint32_t>(flagsNibble) & 0x0Fu) << 20) |
                        (base & 0xFF000000u);
        mem.write32(addr, low);
        mem.write32(addr + 4, high);
    };

    // Minimal PM descriptor setup.
    constexpr uint32_t gdtBase = 0x4000;
    mem.write32(gdtBase + 0, 0);
    mem.write32(gdtBase + 4, 0);
    writeDesc(gdtBase + 0x08, 0x00003000u, 0x000FFFFFu, 0x9Au, 0xCu); // 32-bit code
    writeDesc(gdtBase + 0x10, 0x00004000u, 0x000FFFFFu, 0x9Au, 0xCu); // return CS (32-bit)
    writeDesc(gdtBase + 0x18, 0x00005000u, 0x0000FFFFu, 0x92u, 0x0u); // 16-bit stack

    cpu.setGDTR({0x0030, gdtBase});
    cpu.setCR(0, cpu.getCR(0) | 1u);
    cpu.setSegReg(cpu::CS, 0x0008);
    cpu.setSegBase(cpu::CS, 0x00003000u);
    cpu.setSegReg(cpu::SS, 0x0018);
    cpu.setSegBase(cpu::SS, 0x00005000u);
    cpu.setIs32BitCode(true);
    cpu.setIs32BitStack(false);
    cpu.setEIP(0x0000);
    cpu.setReg16(cpu::SP, 0x0100);

    // Chain frame intentionally makes CS at SP+2 invalid (0), but provides a
    // valid code selector at ESP+4 for fallback probing.
    mem.write32(0x00005000u + 0x0100u, 0x00001234u); // EIP
    mem.write32(0x00005000u + 0x0104u, 0x00000010u); // CS in low 16 bits
    mem.write32(0x00005000u + 0x0108u, 0x00000202u); // EFLAGS

    // Seed a tracked frame at another address so this is classified as chain call.
    cpu.pushHLEFrame(true, 0x15);
    auto &hf = cpu.lastHLEFrameMut();
    hf.framePhysAddr = 0x0000DEADu;
    hf.frameSP = 0x0000DEADu;
    hf.stackIs32 = false;

    cpu.setReg8(cpu::AH, 0x88); // BIOS INT 15h handled path
    mem.write8(0x00003000u + 0x0000u, 0x0F);
    mem.write8(0x00003000u + 0x0001u, 0xFF);
    mem.write8(0x00003000u + 0x0002u, 0x15);

    decoder.step();

    // Fallback via ESP+4 should choose 32-bit chain IRET and advance by 12.
    REQUIRE(cpu.getSegReg(cpu::CS) == 0x0010);
    REQUIRE(cpu.getEIP() == 0x1234);
    REQUIRE(cpu.getReg16(cpu::SP) == 0x010C);
}

TEST_CASE("IRET infers 16-bit synthetic frame width with FLAGS=0x0000", "[Interrupts][IRET]") {
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

    auto writeDesc = [&](uint32_t addr, uint32_t base, uint32_t limit,
                         uint8_t access, uint8_t flagsNibble) {
        uint32_t low = (limit & 0xFFFFu) | ((base & 0xFFFFu) << 16);
        uint32_t high = ((base >> 16) & 0xFFu) |
                        (static_cast<uint32_t>(access) << 8) |
                        (((limit >> 16) & 0x0Fu) << 16) |
                        ((static_cast<uint32_t>(flagsNibble) & 0x0Fu) << 20) |
                        (base & 0xFF000000u);
        mem.write32(addr, low);
        mem.write32(addr + 4, high);
    };

    constexpr uint32_t gdtBase = 0x3000;
    mem.write32(gdtBase + 0, 0);
    mem.write32(gdtBase + 4, 0);
    writeDesc(gdtBase + 0x08, 0x00004000u, 0x000FFFFFu, 0x9Au, 0xCu);
    writeDesc(gdtBase + 0x10, 0x00005000u, 0x000FFFFFu, 0x9Au, 0x8u);
    writeDesc(gdtBase + 0x18, 0x00006000u, 0x0000FFFFu, 0x92u, 0x0u);

    cpu.setGDTR({0x0030, gdtBase});
    cpu.setCR(0, cpu.getCR(0) | 1u); // PE=1

    // Current execution context: HLE stub-like CS=0x08 (USE32), SS=0x18 (16-bit).
    cpu.setSegReg(cpu::CS, 0x0008);
    cpu.setSegBase(cpu::CS, 0x00004000u);
    cpu.setIs32BitCode(true);

    cpu.setSegReg(cpu::SS, 0x0018);
    cpu.setSegBase(cpu::SS, 0x00006000u);
    cpu.setIs32BitStack(true); // <--- Set to 32-bit stack!

    cpu.setEIP(0x0000);
    cpu.setReg32(cpu::ESP, 0x0100); // <--- Use ESP

    // 16-bit synthetic IRET frame at SS:SP -> IP=0x1234, CS=0x0010, FLAGS=0x0000.
    mem.write16(0x00006000u + 0x0100u, 0x1234);
    mem.write16(0x00006000u + 0x0102u, 0x0010);
    mem.write16(0x00006000u + 0x0104u, 0x0000); // NO 0x02 BIT!

    // Execute IRET opcode in the 32-bit stub context.
    mem.write8(0x00004000u, 0xCF);
    decoder.step();

    // Must infer 16-bit frame width.
    REQUIRE(cpu.getSegReg(cpu::CS) == 0x0010);
    REQUIRE(cpu.getEIP() == 0x1234);
    REQUIRE(cpu.getReg32(cpu::ESP) == 0x0106); // <--- Use getReg32
}

TEST_CASE("IRETD infers 32-bit synthetic frame width with EFLAGS=0x00000000", "[Interrupts][IRET]") {
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

    auto writeDesc = [&](uint32_t addr, uint32_t base, uint32_t limit,
                         uint8_t access, uint8_t flagsNibble) {
        uint32_t low = (limit & 0xFFFFu) | ((base & 0xFFFFu) << 16);
        uint32_t high = ((base >> 16) & 0xFFu) |
                        (static_cast<uint32_t>(access) << 8) |
                        (((limit >> 16) & 0x0Fu) << 16) |
                        ((static_cast<uint32_t>(flagsNibble) & 0x0Fu) << 20) |
                        (base & 0xFF000000u);
        mem.write32(addr, low);
        mem.write32(addr + 4, high);
    };

    constexpr uint32_t gdtBase = 0x3000;
    mem.write32(gdtBase + 0, 0);
    mem.write32(gdtBase + 4, 0);
    writeDesc(gdtBase + 0x08, 0x00004000u, 0x000FFFFFu, 0x9Au, 0xCu);
    writeDesc(gdtBase + 0x10, 0x00005000u, 0x000FFFFFu, 0x9Au, 0xCu);
    writeDesc(gdtBase + 0x18, 0x00006000u, 0x0000FFFFu, 0x92u, 0x0u);

    cpu.setGDTR({0x0030, gdtBase});
    cpu.setCR(0, cpu.getCR(0) | 1u); // PE=1

    // Current execution context: HLE stub-like CS=0x08 (USE32).
    cpu.setSegReg(cpu::CS, 0x0008);
    cpu.setSegBase(cpu::CS, 0x00004000u);
    cpu.setIs32BitCode(true);

    cpu.setSegReg(cpu::SS, 0x0018);
    cpu.setSegBase(cpu::SS, 0x00006000u);
    cpu.setIs32BitStack(false); // Called from 16-bit stack

    cpu.setEIP(0x0000);
    cpu.setReg16(cpu::SP, 0x0100);

    // 32-bit synthetic IRETD frame at SS:SP -> EIP=0x12345678, CS=0x0010, EFLAGS=0x00000000.
    mem.write32(0x00006000u + 0x0100u, 0x12345678u);
    mem.write32(0x00006000u + 0x0104u, 0x00000010u);
    mem.write32(0x00006000u + 0x0108u, 0x00000000u); // NO 0x02 BIT!

    // Execute IRET opcode in the 32-bit stub context.
    mem.write8(0x00004000u, 0xCF);
    decoder.step();

    // Must infer 32-bit frame width.
    REQUIRE(cpu.getSegReg(cpu::CS) == 0x0010);
    REQUIRE(cpu.getEIP() == 0x12345678u);
    REQUIRE(cpu.getReg16(cpu::SP) == 0x010C);
}

TEST_CASE("0F FF chain infers 16-bit synthetic frame with FLAGS=0x0000", "[Interrupts][Chain]") {
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

    auto writeDesc = [&](uint32_t addr, uint32_t base, uint32_t limit,
                         uint8_t access, uint8_t flagsNibble) {
        uint32_t low = (limit & 0xFFFFu) | ((base & 0xFFFFu) << 16);
        uint32_t high = ((base >> 16) & 0xFFu) |
                        (static_cast<uint32_t>(access) << 8) |
                        (((limit >> 16) & 0x0Fu) << 16) |
                        ((static_cast<uint32_t>(flagsNibble) & 0x0Fu) << 20) |
                        (base & 0xFF000000u);
        mem.write32(addr, low);
        mem.write32(addr + 4, high);
    };

    constexpr uint32_t gdtBase = 0x4000;
    mem.write32(gdtBase + 0, 0);
    mem.write32(gdtBase + 4, 0);
    writeDesc(gdtBase + 0x08, 0x00003000u, 0x000FFFFFu, 0x9Au, 0xCu); // 32-bit code
    writeDesc(gdtBase + 0x10, 0x00004000u, 0x000FFFFFu, 0x9Au, 0x8u); // return CS (16-bit)
    writeDesc(gdtBase + 0x18, 0x00005000u, 0x0000FFFFu, 0x92u, 0x0u); // 16-bit stack

    cpu.setGDTR({0x0030, gdtBase});
    cpu.setCR(0, cpu.getCR(0) | 1u);
    cpu.setSegReg(cpu::CS, 0x0008);
    cpu.setSegBase(cpu::CS, 0x00003000u);
    cpu.setSegReg(cpu::SS, 0x0018);
    cpu.setSegBase(cpu::SS, 0x00005000u);
    cpu.setIs32BitCode(true);
    cpu.setIs32BitStack(true); // <--- Set to 32-bit stack!
    cpu.setEIP(0x0000);
    cpu.setReg32(cpu::ESP, 0x0100); // <--- Use ESP

    // 16-bit chain frame at SP: IP=0x1234, CS=0x0010, FLAGS=0x0000
    mem.write16(0x00005000u + 0x0100u, 0x1234);
    mem.write16(0x00005000u + 0x0102u, 0x0010);
    mem.write16(0x00005000u + 0x0104u, 0x0000);

    // Seed a tracked frame at another address so this is classified as chain call.
    cpu.pushHLEFrame(true, 0x15);
    auto &hf = cpu.lastHLEFrameMut();
    hf.framePhysAddr = 0x0000DEADu;
    hf.frameSP = 0x0000DEADu;
    hf.stackIs32 = false;

    cpu.setReg8(cpu::AH, 0x88);
    mem.write8(0x00003000u + 0x0000u, 0x0F);
    mem.write8(0x00003000u + 0x0001u, 0xFF);
    mem.write8(0x00003000u + 0x0002u, 0x15);

    decoder.step();

    REQUIRE(cpu.getSegReg(cpu::CS) == 0x0010);
    REQUIRE(cpu.getEIP() == 0x1234);
    REQUIRE(cpu.getReg32(cpu::ESP) == 0x0106); // <--- Use getReg32
}

TEST_CASE("Interrupt Chaining: HLE stub returns to chain caller", "[Interrupts][Chain]") {
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

    // RM setup
    decoder.loadSegment(cpu::CS, 0x1000);
    decoder.loadSegment(cpu::SS, 0x0000);
    cpu.setReg16(cpu::SP, 0x1000);
    cpu.setEIP(0x0100);

    // 1. Manually push a "tracked" frame to simulate a pending interrupt
    // Vector 0xAA, CS:IP=0x1234:0x5678, Flags=0x0000
    uint32_t origSP = 0x1000;
    cpu.pushHLEFrame(false, 0xAA);
    auto& hf = cpu.lastHLEFrameMut();
    hf.framePhysAddr = 0x00000000u + (origSP - 6);
    hf.frameSP = origSP - 6;
    hf.stackIs32 = false;

    // Simulate the INT AAh pushing onto stack
    mem.write16(origSP - 6, 0x5678); // IP
    mem.write16(origSP - 4, 0x1234); // CS
    mem.write16(origSP - 2, 0x0000); // Flags
    cpu.setReg16(cpu::SP, origSP - 6);

    // 2. Simulate a chain call: Hooked handler does PUSHF + CALL FAR to HLE stub
    // Current SP = origSP - 6
    cpu.push16(0x0202); // PUSHF
    cpu.push16(0x1000); // CS (return to handler)
    cpu.push16(0x0105); // IP (return to handler)
    // New SP = origSP - 12

    // 3. Execute HLE trap 0F FF AA
    mem.write8(0x10000 + 0x0100, 0x0F);
    mem.write8(0x10000 + 0x0101, 0xFF);
    mem.write8(0x10000 + 0x0102, 0xAA);
    cpu.setEIP(0x0100);

    decoder.step();

    // Should have detected chain call (SP=origSP-12 != trackedSP=origSP-6)
    // and simulated IRET back to 1000:0105
    REQUIRE(cpu.getSegReg(cpu::CS) == 0x1000);
    REQUIRE(cpu.getEIP() == 0x0105);
    REQUIRE(cpu.getReg16(cpu::SP) == (origSP - 6));
    
    // The tracked HLE frame should STILL BE THERE because we didn't "handle" it
    // and we returned to a chain caller who will likely perform their own IRET.
    REQUIRE(cpu.hleStackSize() == 1);

    // 4. Now execute the IRET in the chain caller (at 1000:0105)
    mem.write8(0x10000 + 0x0105, 0xCF); // IRET
    decoder.step();

    // Now it should return to original caller 1234:5678 and pop the frame
    REQUIRE(cpu.getSegReg(cpu::CS) == 0x1234);
    REQUIRE(cpu.getEIP() == 0x5678);
    REQUIRE(cpu.getReg16(cpu::SP) == origSP);
    REQUIRE(cpu.hleStackSize() == 0);
}

TEST_CASE("Blocking INT 16h AH=0 rewinds properly for 0F FF", "[Interrupts][Keyboard]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::IOBus iobus;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::DOS dos(cpu, mem);
    dos.setKeyboard(kbd);
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    bios.initialize();
    dos.initialize();
    cpu::InstructionDecoder decoder(cpu, mem, iobus, bios, dos);

    cpu.setSegReg(cpu::CS, 0x1000);
    cpu.setSegBase(cpu::CS, 0x10000);
    cpu.setEIP(0x0100);

    // Write 0F FF 16 (HLE trap for INT 16h)
    mem.write8(0x10100, 0x0F);
    mem.write8(0x10101, 0xFF);
    mem.write8(0x10102, 0x16);

    // AH=00h (blocking read)
    cpu.setReg8(cpu::AH, 0x00);

    // No key in keyboard buffer
    REQUIRE(kbd.hasKey() == false);

    // Execute instruction
    decoder.step();

    // EIP should be rewound to 0x0100 to repeat the instruction
    REQUIRE(cpu.getEIP() == 0x0100);

    // Now push a key
    kbd.pushKey('A', 0x1E);

    decoder.step();

    // EIP should now advance past the 3-byte instruction

}

TEST_CASE("Blocking INT 21h AH=01 rewinds properly for 0F FF", "[Interrupts][DOS]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::IOBus iobus;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::DOS dos(cpu, mem);
    dos.setKeyboard(kbd);
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    bios.initialize();
    dos.initialize();
    cpu::InstructionDecoder decoder(cpu, mem, iobus, bios, dos);

    cpu.setSegReg(cpu::CS, 0x1000);
    cpu.setSegBase(cpu::CS, 0x10000);
    cpu.setEIP(0x0100);

    // Write 0F FF 21 (HLE trap for INT 21h)
    mem.write8(0x10100, 0x0F);
    mem.write8(0x10101, 0xFF);
    mem.write8(0x10102, 0x21);

    // AH=01h (blocking read with echo)
    cpu.setReg8(cpu::AH, 0x01);

    // No key in keyboard buffer
    REQUIRE(kbd.hasKey() == false);

    // Execute instruction
    decoder.step();

    // EIP should be rewound to 0x0100 to repeat the instruction
    REQUIRE(cpu.getEIP() == 0x0100);

    // Now push a key
    kbd.pushKey('B', 0x30);

    decoder.step();

    // EIP should now advance past the 3-byte instruction

}

TEST_CASE("Timer IRQ pipeline delivers every elapsed realtime pulse", "[Interrupts][Timer]") {
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

    cpu.setCR(0, 0);
    cpu.setSegReg(cpu::CS, 0x1000);
    cpu.setSegBase(cpu::CS, 0x10000);
    cpu.setSegReg(cpu::DS, 0x0000);
    cpu.setSegBase(cpu::DS, 0x0000);
    cpu.setSegReg(cpu::SS, 0x0000);
    cpu.setSegBase(cpu::SS, 0x0000);
    cpu.setReg16(cpu::SP, 0x0200);
    cpu.setEIP(0x0100);
    cpu.setEFLAGS(cpu.getEFLAGS() | cpu::FLAG_INTERRUPT);

    pit.write8(0x43, 0x36);
    pit.write8(0x40, 0x50);
    pit.write8(0x40, 0xC3); // Reload = 50000 -> about 41.9 ms per IRQ0
    pit.advanceTime(std::chrono::milliseconds(126));

    while (pit.checkPendingIRQ0()) {
        pic.raiseIRQ(0);
    }

    int delivered = 0;
    while (true) {
        int pending = pic.getPendingInterrupt();
        if (pending == -1) {
            break;
        }

        REQUIRE(pending == 0x08);
        pic.acknowledgeInterrupt();
        decoder.injectHardwareInterrupt(static_cast<uint8_t>(pending));
        ++delivered;
    }

    REQUIRE(delivered == 3);
    REQUIRE(mem.read32(0x46C) == 3);
}

TEST_CASE("Hardware IRQ is deferred until after the instruction following STI", "[Interrupts][Timer]") {
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

    cpu.setCR(0, 0);
    cpu.setSegReg(cpu::CS, 0x1000);
    cpu.setSegBase(cpu::CS, 0x10000);
    cpu.setSegReg(cpu::DS, 0x0000);
    cpu.setSegBase(cpu::DS, 0x0000);
    cpu.setSegReg(cpu::SS, 0x0000);
    cpu.setSegBase(cpu::SS, 0x0000);
    cpu.setReg16(cpu::SP, 0x0200);
    cpu.setEIP(0x0100);
    cpu.setEFLAGS(cpu.getEFLAGS() & ~cpu::FLAG_INTERRUPT);

    mem.write16(0x08 * 4, 0x4321);
    mem.write16((0x08 * 4) + 2, 0x1234);

    mem.write8(0x10100, 0xFB); // STI
    mem.write8(0x10101, 0x90); // NOP

    auto dispatchPendingHardwareInterrupt = [&]() {
        if (cpu.hardwareInterruptsEnabled()) {
            int pending = pic.getPendingInterrupt();
            if (pending != -1) {
                pic.acknowledgeInterrupt();
                decoder.injectHardwareInterrupt(static_cast<uint8_t>(pending));
            }
        }
        cpu.advanceInterruptShadow();
    };

    pic.raiseIRQ(0);

    decoder.step();
    REQUIRE(cpu.getEFLAGS() & cpu::FLAG_INTERRUPT);
    dispatchPendingHardwareInterrupt();

    REQUIRE(cpu.getSegReg(cpu::CS) == 0x1000);
    REQUIRE(cpu.getEIP() == 0x0101);

    decoder.step();
    dispatchPendingHardwareInterrupt();

    REQUIRE(cpu.getSegReg(cpu::CS) == 0x1234);
    REQUIRE(cpu.getEIP() == 0x4321);
}

TEST_CASE("Real-mode timer IRQ dispatch respects hooked INT 8 vector", "[Interrupts][Timer]") {
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

    cpu.setCR(0, 0);
    cpu.setSegReg(cpu::CS, 0x1000);
    cpu.setSegBase(cpu::CS, 0x10000);
    cpu.setSegReg(cpu::DS, 0x0000);
    cpu.setSegBase(cpu::DS, 0x0000);
    cpu.setSegReg(cpu::SS, 0x0000);
    cpu.setSegBase(cpu::SS, 0x0000);
    cpu.setReg16(cpu::SP, 0x0200);
    cpu.setEIP(0x0100);

    mem.write16(0x08 * 4, 0x0300);
    mem.write16(0x08 * 4 + 2, 0x2000);

    uint32_t handlerPhys = (0x2000u << 4) + 0x0300u;
    const uint8_t handler[] = {
        0xB0, 0x01,             // MOV AL,01h
        0xA2, 0x00, 0x05,       // MOV [0500h],AL
        0xB0, 0x20,             // MOV AL,20h
        0xE6, 0x20,             // OUT 20h,AL
        0xCF                    // IRET
    };
    for (size_t i = 0; i < sizeof(handler); ++i) {
        mem.write8(handlerPhys + static_cast<uint32_t>(i), handler[i]);
    }

    uint16_t timerBefore = mem.read16(0x46C);

    decoder.injectHardwareInterrupt(0x08);

    REQUIRE(cpu.getSegReg(cpu::CS) == 0x2000);
    REQUIRE(cpu.getEIP() == 0x0300);
    REQUIRE(mem.read16(0x46C) == timerBefore);

    for (int step = 0; step < 5; ++step) {
        decoder.step();
    }

    REQUIRE(mem.read8(0x0500) == 1);
    REQUIRE(cpu.getSegReg(cpu::CS) == 0x1000);
    REQUIRE(cpu.getEIP() == 0x0100);
}

TEST_CASE("Real-mode timer IRQ HLE still dispatches guest INT 1Ch hook", "[Interrupts][Timer]") {
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

    cpu.setCR(0, 0);
    cpu.setSegReg(cpu::CS, 0x1000);
    cpu.setSegBase(cpu::CS, 0x10000);
    cpu.setSegReg(cpu::DS, 0x0000);
    cpu.setSegBase(cpu::DS, 0x0000);
    cpu.setSegReg(cpu::SS, 0x0000);
    cpu.setSegBase(cpu::SS, 0x0000);
    cpu.setReg16(cpu::SP, 0x0200);
    cpu.setEIP(0x0100);

    mem.write16(0x1C * 4, 0x0300);
    mem.write16(0x1C * 4 + 2, 0x2000);

    uint32_t handlerPhys = (0x2000u << 4) + 0x0300u;
    const uint8_t handler[] = {
        0xB0, 0x01,             // MOV AL,01h
        0xA2, 0x00, 0x05,       // MOV [0500h],AL
        0xCF                    // IRET
    };
    for (size_t i = 0; i < sizeof(handler); ++i) {
        mem.write8(handlerPhys + static_cast<uint32_t>(i), handler[i]);
    }

    uint16_t timerBefore = mem.read16(0x46C);

    decoder.injectHardwareInterrupt(0x08);

    REQUIRE(cpu.getSegReg(cpu::CS) == 0x2000);
    REQUIRE(cpu.getEIP() == 0x0300);
    REQUIRE(mem.read16(0x46C) == timerBefore + 1);

    for (int step = 0; step < 3; ++step) {
        decoder.step();
    }

    REQUIRE(mem.read8(0x0500) == 1);
    REQUIRE(cpu.getSegReg(cpu::CS) == 0x1000);
    REQUIRE(cpu.getEIP() == 0x0100);
}

TEST_CASE("BIOS timer tick count grows over one second in live loop", "[Interrupts][Timer][Integration]") {
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

    cpu.setCR(0, 0);
    cpu.setSegReg(cpu::CS, 0x1000);
    cpu.setSegBase(cpu::CS, 0x10000);
    cpu.setSegReg(cpu::DS, 0x0000);
    cpu.setSegBase(cpu::DS, 0x0000);
    cpu.setSegReg(cpu::SS, 0x0000);
    cpu.setSegBase(cpu::SS, 0x0000);
    cpu.setReg16(cpu::SP, 0x0200);
    cpu.setEIP(0x0100);
    cpu.setEFLAGS(cpu.getEFLAGS() | cpu::FLAG_INTERRUPT);

    // Tiny program loop: NOP; JMP short -3
    mem.write8(0x10100, 0x90);
    mem.write8(0x10101, 0xEB);
    mem.write8(0x10102, 0xFD);

    const uint32_t tickStart = mem.read32(0x46C);
    constexpr auto slice = std::chrono::milliseconds(10);
    constexpr int sliceCount = 100;
    constexpr int instructionsPerSlice = 0x400;
    constexpr uint32_t expectedTicks = 18;

    for (int sliceIndex = 0; sliceIndex < sliceCount; ++sliceIndex) {
        for (int instruction = 0; instruction < instructionsPerSlice; ++instruction) {
            decoder.step();
            cpu.addCycles(4);
        }

        pit.advanceTime(slice);

        while (pit.checkPendingIRQ0()) {
            pic.raiseIRQ(0);
        }
        if (kbd.checkPendingIRQ()) {
            pic.raiseIRQ(1);
        }

        if (cpu.getEFLAGS() & cpu::FLAG_INTERRUPT) {
            while (true) {
                int pending = pic.getPendingInterrupt();
                if (pending == -1) {
                    break;
                }

                pic.acknowledgeInterrupt();
                decoder.injectHardwareInterrupt(static_cast<uint8_t>(pending));
            }
        }
    }

    REQUIRE(mem.read32(0x46C) - tickStart == expectedTicks);
    REQUIRE(pic.getPendingInterrupt() == -1);
}
