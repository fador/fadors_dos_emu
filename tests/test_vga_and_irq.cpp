// test_vga_and_irq.cpp – Regression tests for VGA Mode-X plane memory,
// D-bit guard for hardware interrupts, keyboard IRQ EOI fallback, and
// timer ticcount HLE increment.

#include "test_framework.hpp"
#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "memory/MemoryBus.hpp"
#include "memory/himem/HIMEM.hpp"
#include "hw/IOBus.hpp"
#include "hw/PIC8259.hpp"
#include "hw/BIOS.hpp"
#include "hw/DOS.hpp"
#include "hw/DPMI.hpp"
#include "hw/VGAController.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIT8254.hpp"

using namespace fador;

// ═══════════════════════════════════════════════════════════════════════
// VGA Mode-X Plane Memory Tests
// ═══════════════════════════════════════════════════════════════════════

TEST_CASE("VGA: Chain-4 write distributes to correct plane", "[VGA]") {
    memory::MemoryBus mem;
    hw::VGAController vga(mem);

    // Default is Chain-4 mode
    REQUIRE(vga.isChain4() == true);

    SECTION("Chain-4 address bits [1:0] select plane") {
        // In Chain-4, byte at offset N goes to plane (N & 3), planeOffset (N >> 2)
        vga.planeWrite8(0, 0xAA);   // plane 0, offset 0
        vga.planeWrite8(1, 0xBB);   // plane 1, offset 0
        vga.planeWrite8(2, 0xCC);   // plane 2, offset 0
        vga.planeWrite8(3, 0xDD);   // plane 3, offset 0
        vga.planeWrite8(4, 0x11);   // plane 0, offset 1

        REQUIRE(vga.readPlane(0, 0) == 0xAA);
        REQUIRE(vga.readPlane(1, 0) == 0xBB);
        REQUIRE(vga.readPlane(2, 0) == 0xCC);
        REQUIRE(vga.readPlane(3, 0) == 0xDD);
        REQUIRE(vga.readPlane(0, 1) == 0x11);
    }

    SECTION("Chain-4 read returns correct plane byte") {
        vga.planeWrite8(0, 0x10);
        vga.planeWrite8(1, 0x20);
        vga.planeWrite8(2, 0x30);
        vga.planeWrite8(3, 0x40);

        REQUIRE(vga.planeRead8(0) == 0x10);
        REQUIRE(vga.planeRead8(1) == 0x20);
        REQUIRE(vga.planeRead8(2) == 0x30);
        REQUIRE(vga.planeRead8(3) == 0x40);
    }
}

TEST_CASE("VGA: Mode-X (unchained) writes respect map mask", "[VGA]") {
    memory::MemoryBus mem;
    hw::VGAController vga(mem);

    // Switch to unchained mode: Sequencer reg 4, clear bit 3
    vga.write8(0x3C4, 4);    // select seq register 4
    vga.write8(0x3C5, 0x00); // clear Chain-4 bit
    REQUIRE(vga.isChain4() == false);

    SECTION("Map mask selects which planes are written") {
        // Set map mask to plane 0 only
        vga.write8(0x3C4, 2);    // select seq register 2 (map mask)
        vga.write8(0x3C5, 0x01); // only plane 0
        vga.planeWrite8(0, 0xAA);

        REQUIRE(vga.readPlane(0, 0) == 0xAA);
        REQUIRE(vga.readPlane(1, 0) == 0x00); // untouched
        REQUIRE(vga.readPlane(2, 0) == 0x00);
        REQUIRE(vga.readPlane(3, 0) == 0x00);
    }

    SECTION("Map mask 0x0F writes all planes") {
        vga.write8(0x3C4, 2);
        vga.write8(0x3C5, 0x0F); // all planes
        vga.planeWrite8(100, 0x55);

        REQUIRE(vga.readPlane(0, 100) == 0x55);
        REQUIRE(vga.readPlane(1, 100) == 0x55);
        REQUIRE(vga.readPlane(2, 100) == 0x55);
        REQUIRE(vga.readPlane(3, 100) == 0x55);
    }

    SECTION("Map mask selects specific planes") {
        vga.write8(0x3C4, 2);
        vga.write8(0x3C5, 0x05); // planes 0 and 2
        vga.planeWrite8(50, 0x77);

        REQUIRE(vga.readPlane(0, 50) == 0x77);
        REQUIRE(vga.readPlane(1, 50) == 0x00);
        REQUIRE(vga.readPlane(2, 50) == 0x77);
        REQUIRE(vga.readPlane(3, 50) == 0x00);
    }
}

TEST_CASE("VGA: Mode-X read uses read map select", "[VGA]") {
    memory::MemoryBus mem;
    hw::VGAController vga(mem);

    // Switch to unchained mode
    vga.write8(0x3C4, 4);
    vga.write8(0x3C5, 0x00);

    // Write different values to each plane at same offset
    for (int p = 0; p < 4; ++p) {
        vga.write8(0x3C4, 2);
        vga.write8(0x3C5, 1 << p); // single plane
        vga.planeWrite8(200, 0x10 * (p + 1)); // 0x10, 0x20, 0x30, 0x40
    }

    SECTION("Read map select chooses which plane is read") {
        for (int p = 0; p < 4; ++p) {
            vga.write8(0x3CE, 4);        // GC register 4 = read map select
            vga.write8(0x3CF, p);
            REQUIRE(vga.planeRead8(200) == static_cast<uint8_t>(0x10 * (p + 1)));
        }
    }
}

TEST_CASE("VGA: Sequencer Chain-4 bit toggle", "[VGA]") {
    memory::MemoryBus mem;
    hw::VGAController vga(mem);

    REQUIRE(vga.isChain4() == true);

    // Disable Chain-4
    vga.write8(0x3C4, 4);
    vga.write8(0x3C5, 0x00);
    REQUIRE(vga.isChain4() == false);

    // Re-enable Chain-4
    vga.write8(0x3C4, 4);
    vga.write8(0x3C5, 0x08);
    REQUIRE(vga.isChain4() == true);

    // Read back seq register 4
    vga.write8(0x3C4, 4);
    REQUIRE(vga.read8(0x3C5) == 0x08);

    vga.write8(0x3C5, 0x00);
    vga.write8(0x3C4, 4);
    REQUIRE(vga.read8(0x3C5) == 0x00);
}

TEST_CASE("VGA: CRTC display start for page flipping", "[VGA]") {
    memory::MemoryBus mem;
    hw::VGAController vga(mem);

    REQUIRE(vga.getDisplayStart() == 0);

    SECTION("Set display start high byte") {
        vga.write8(0x3D4, 0x0C); // CRTC start address high
        vga.write8(0x3D5, 0x80); // 0x8000
        REQUIRE(vga.getDisplayStart() == 0x8000);
    }

    SECTION("Set display start low byte") {
        vga.write8(0x3D4, 0x0D); // CRTC start address low
        vga.write8(0x3D5, 0x40);
        REQUIRE(vga.getDisplayStart() == 0x0040);
    }

    SECTION("Set display start both bytes") {
        vga.write8(0x3D4, 0x0C);
        vga.write8(0x3D5, 0x40); // high = 0x40
        vga.write8(0x3D4, 0x0D);
        vga.write8(0x3D5, 0x00); // low = 0x00
        REQUIRE(vga.getDisplayStart() == 0x4000);
    }
}

TEST_CASE("VGA: MemoryBus intercepts A0000 writes to plane memory", "[VGA]") {
    memory::MemoryBus mem;
    hw::VGAController vga(mem);
    mem.setVGA(&vga);

    SECTION("Chain-4 write through MemoryBus") {
        mem.write8(0xA0000, 0xAA); // plane 0, offset 0
        mem.write8(0xA0001, 0xBB); // plane 1, offset 0
        mem.write8(0xA0002, 0xCC); // plane 2, offset 0
        mem.write8(0xA0003, 0xDD); // plane 3, offset 0

        REQUIRE(vga.readPlane(0, 0) == 0xAA);
        REQUIRE(vga.readPlane(1, 0) == 0xBB);
        REQUIRE(vga.readPlane(2, 0) == 0xCC);
        REQUIRE(vga.readPlane(3, 0) == 0xDD);
    }

    SECTION("Chain-4 read through MemoryBus") {
        mem.write8(0xA0000, 0x11);
        mem.write8(0xA0001, 0x22);

        REQUIRE(mem.read8(0xA0000) == 0x11);
        REQUIRE(mem.read8(0xA0001) == 0x22);
    }

    SECTION("Mode-X write through MemoryBus with map mask") {
        // Switch to unchained mode
        vga.write8(0x3C4, 4);
        vga.write8(0x3C5, 0x00);

        // Set map mask to all planes
        vga.write8(0x3C4, 2);
        vga.write8(0x3C5, 0x0F);

        mem.write8(0xA0000, 0x42);

        REQUIRE(vga.readPlane(0, 0) == 0x42);
        REQUIRE(vga.readPlane(1, 0) == 0x42);
        REQUIRE(vga.readPlane(2, 0) == 0x42);
        REQUIRE(vga.readPlane(3, 0) == 0x42);
    }

    SECTION("Mode-X read through MemoryBus uses read map select") {
        // Switch to unchained mode
        vga.write8(0x3C4, 4);
        vga.write8(0x3C5, 0x00);

        // Write different values to each plane
        for (int p = 0; p < 4; ++p) {
            vga.write8(0x3C4, 2);
            vga.write8(0x3C5, 1 << p);
            mem.write8(0xA0064, 0x10 * (p + 1));
        }

        // Read back with different read map selects
        for (int p = 0; p < 4; ++p) {
            vga.write8(0x3CE, 4);
            vga.write8(0x3CF, p);
            REQUIRE(mem.read8(0xA0064) == static_cast<uint8_t>(0x10 * (p + 1)));
        }
    }

    SECTION("Writes outside VGA window are not intercepted") {
        // Write to 0xB0000 (outside the A0000-AFFFF VGA window)
        mem.write8(0xB0000, 0xFF);
        // Should go to flat memory, not VGA planes
        REQUIRE(mem.read8(0xB0000) == 0xFF);
    }
}

TEST_CASE("VGA: write16/write32 propagate to planes", "[VGA]") {
    memory::MemoryBus mem;
    hw::VGAController vga(mem);
    mem.setVGA(&vga);

    SECTION("write16 in Chain-4 mode") {
        mem.write16(0xA0000, 0xBBAA); // bytes: AA at offset 0, BB at offset 1
        REQUIRE(vga.readPlane(0, 0) == 0xAA); // plane 0
        REQUIRE(vga.readPlane(1, 0) == 0xBB); // plane 1
    }

    SECTION("write32 in Chain-4 mode") {
        mem.write32(0xA0000, 0xDDCCBBAA);
        REQUIRE(vga.readPlane(0, 0) == 0xAA);
        REQUIRE(vga.readPlane(1, 0) == 0xBB);
        REQUIRE(vga.readPlane(2, 0) == 0xCC);
        REQUIRE(vga.readPlane(3, 0) == 0xDD);
    }
}

TEST_CASE("VGA: DAC palette read/write cycle", "[VGA]") {
    memory::MemoryBus mem;
    hw::VGAController vga(mem);

    SECTION("Write and read back palette entry 0") {
        vga.write8(0x3C8, 0); // DAC write index = 0
        vga.write8(0x3C9, 0x3F); // R = 63
        vga.write8(0x3C9, 0x00); // G = 0
        vga.write8(0x3C9, 0x1F); // B = 31

        // Read back
        vga.write8(0x3C7, 0); // DAC read index = 0
        REQUIRE(vga.read8(0x3C9) == 0x3F); // R
        REQUIRE(vga.read8(0x3C9) == 0x00); // G
        REQUIRE(vga.read8(0x3C9) == 0x1F); // B
    }

    SECTION("Palette auto-increment across entries") {
        vga.write8(0x3C8, 5); // Start at entry 5
        vga.write8(0x3C9, 10); // R
        vga.write8(0x3C9, 20); // G
        vga.write8(0x3C9, 30); // B
        // Should auto-advance to entry 6
        vga.write8(0x3C9, 40); // R of entry 6
        vga.write8(0x3C9, 50); // G
        vga.write8(0x3C9, 60); // B

        vga.write8(0x3C7, 5);
        REQUIRE(vga.read8(0x3C9) == 10);
        REQUIRE(vga.read8(0x3C9) == 20);
        REQUIRE(vga.read8(0x3C9) == 30);
        REQUIRE(vga.read8(0x3C9) == 40);
        REQUIRE(vga.read8(0x3C9) == 50);
        REQUIRE(vga.read8(0x3C9) == 60);
    }
}

// ═══════════════════════════════════════════════════════════════════════
// D-bit Guard & IRQ Handling Tests
// ═══════════════════════════════════════════════════════════════════════

// Helper: sets up a minimal PM environment with GDT, IDT, and DPMI
// so that injectHardwareInterrupt can check the D-bit of the IDT target.
struct PMIRQTestEnv {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::IOBus iobus;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic{true};
    hw::DOS dos;
    hw::BIOS bios;
    memory::HIMEM himem;
    hw::DPMI dpmi;
    cpu::InstructionDecoder decoder;

    // GDT layout (in low memory at 0x1000):
    //   [0] null
    //   [1] sel=0x08: 32-bit code (HLE stubs CS)
    //   [2] sel=0x10: 32-bit data
    //   [3] sel=0x18: 16-bit code (thunk CS, D=0)
    static constexpr uint32_t GDT_ADDR = 0x1000;
    static constexpr uint32_t IDT_ADDR = 0x2000;
    // A flat 32-bit code segment for application use
    static constexpr uint16_t CODE32_SEL = 0x08;
    // A 16-bit code segment (simulates a DOS extender thunk)
    static constexpr uint16_t CODE16_SEL = 0x18;
    // 32-bit data segment
    static constexpr uint16_t DATA32_SEL = 0x10;

    PMIRQTestEnv()
        : dos(cpu, mem), bios(cpu, mem, kbd, pit, pic),
          dpmi(cpu, mem),
          decoder(cpu, mem, iobus, bios, dos)
    {
        bios.initialize();
        dos.initialize();
        dpmi.setDOS(&dos);
        dos.setDPMI(&dpmi);
        dpmi.setBIOS(&bios);
        dpmi.setHIMEM(&himem);
        himem.setMemoryBus(&mem);

        // Register PIC on IOBus so EOI writes work
        iobus.registerDevice(0x20, 0x21, &pic);

        // Initialize PIC: vector base 0x08 for master
        pic.write8(0x20, 0x11); // ICW1
        pic.write8(0x21, 0x08); // ICW2: vector base 0x08
        pic.write8(0x21, 0x04); // ICW3
        pic.write8(0x21, 0x01); // ICW4
        pic.write8(0x21, 0x00); // unmask all

        setupGDT();
        setupIDT_16bit(); // default: IDT points to 16-bit thunk selector
        enterPM();
    }

    void setupGDT() {
        // Null descriptor
        mem.write32(GDT_ADDR + 0, 0);
        mem.write32(GDT_ADDR + 4, 0);

        // sel 0x08: 32-bit code, base=0, limit=FFFFF, G=1, D=1
        writeDescriptor(GDT_ADDR + 0x08, 0, 0xFFFFF, true, true, true);

        // sel 0x10: 32-bit data, base=0, limit=FFFFF, G=1
        writeDescriptor(GDT_ADDR + 0x10, 0, 0xFFFFF, false, true, true);

        // sel 0x18: 16-bit code, base=0, limit=FFFF, G=0, D=0
        writeDescriptor(GDT_ADDR + 0x18, 0, 0xFFFF, true, false, false);

        cpu.setGDTR({0x1F, GDT_ADDR}); // 4 entries, limit=31, base=0x1000
    }

    // Write a GDT descriptor
    // isCode: executable segment? isGranular: 4K pages? is32Bit: D bit?
    void writeDescriptor(uint32_t addr, uint32_t base, uint32_t limit,
                         bool isCode, bool is32Bit, bool isGranular) {
        uint8_t access = 0x90; // present, ring 0
        if (isCode)
            access |= 0x0A; // code, readable
        else
            access |= 0x02; // data, writable

        uint8_t flags = 0;
        if (isGranular) flags |= 0x80; // G bit
        if (is32Bit)    flags |= 0x40; // D/B bit
        flags |= ((limit >> 16) & 0x0F);

        uint32_t low = (limit & 0xFFFF) | ((base & 0xFFFF) << 16);
        uint32_t high = ((base >> 16) & 0xFF) | (access << 8) |
                        (flags << 16) | (base & 0xFF000000);
        mem.write32(addr, low);
        mem.write32(addr + 4, high);
    }

    // Set up IDT entry for vector to point to a 16-bit code segment (D=0)
    void setupIDT_16bit() {
        // IDT entry for vector 0x08: selector=CODE16_SEL, offset=0x0020
        writeIDTEntry(0x08, CODE16_SEL, 0x0020);
        // IDT entry for vector 0x09: selector=CODE16_SEL, offset=0x0020
        writeIDTEntry(0x09, CODE16_SEL, 0x0020);

        cpu.setIDTR({0x7FF, IDT_ADDR}); // 256 entries, limit=2047, base=0x2000
    }

    // Set up IDT entry for vector to point to a 32-bit HLE stub
    void setupIDT_32bit_HLE(uint8_t vector) {
        uint32_t offset = 0xF0100 + vector * 4;
        writeIDTEntry(vector, CODE32_SEL, offset);
    }

    void writeIDTEntry(uint8_t vector, uint16_t selector, uint32_t offset) {
        uint32_t addr = IDT_ADDR + vector * 8;
        uint32_t low = (offset & 0xFFFF) | (uint32_t(selector) << 16);
        uint32_t high = (offset & 0xFFFF0000) | 0x8E00; // present, 32-bit int gate
        mem.write32(addr, low);
        mem.write32(addr + 4, high);
    }

    void enterPM() {
        // Set CR0 PE bit
        cpu.setCR(0, cpu.getCR(0) | 1);
        // Load flat 32-bit segments
        decoder.loadSegment(cpu::CS, CODE32_SEL);
        decoder.loadSegment(cpu::DS, DATA32_SEL);
        decoder.loadSegment(cpu::ES, DATA32_SEL);
        decoder.loadSegment(cpu::SS, DATA32_SEL);
        cpu.setReg32(cpu::ESP, 0x80000);
        cpu.setEIP(0x100000);
        // Enable interrupts
        cpu.setEFLAGS(cpu.getEFLAGS() | cpu::FLAG_INTERRUPT);
    }
};

TEST_CASE("IRQ: D-bit guard forces HLE for 16-bit thunk IDT target", "[IRQ]") {
    PMIRQTestEnv env;

    SECTION("Timer IRQ (0x08) with 16-bit IDT target uses HLE") {
        // The IDT vector 0x08 points to CODE16_SEL (D=0), so the D-bit
        // guard should force HLE.  The BIOS HLE handler increments 0x46C.
        uint16_t counterBefore = env.mem.read16(0x46C);

        // Raise IRQ 0 and inject
        env.pic.raiseIRQ(0);
        int pending = env.pic.getPendingInterrupt();
        REQUIRE(pending == 0x08);
        env.pic.acknowledgeInterrupt();
        env.decoder.injectHardwareInterrupt(0x08);

        // If HLE worked, 0x46C should be incremented
        uint16_t counterAfter = env.mem.read16(0x46C);
        REQUIRE(counterAfter == counterBefore + 1);

        // CPU should NOT have jumped into the thunk — EIP should be unchanged
        REQUIRE(env.cpu.getEIP() == 0x100000);
        // CS should still be the 32-bit code selector
        REQUIRE(env.cpu.getSegReg(cpu::CS) == PMIRQTestEnv::CODE32_SEL);
    }

    SECTION("Timer IRQ with 32-bit HLE IDT target also uses HLE") {
        // If IDT points to the HLE stub (sel=0x08, offset=F0100+vec*4),
        // HLE should also trigger.
        env.setupIDT_32bit_HLE(0x08);

        uint16_t counterBefore = env.mem.read16(0x46C);
        env.decoder.injectHardwareInterrupt(0x08);
        uint16_t counterAfter = env.mem.read16(0x46C);
        REQUIRE(counterAfter == counterBefore + 1);
        REQUIRE(env.cpu.getEIP() == 0x100000);
    }
}

TEST_CASE("IRQ: Unhandled HLE vector sends EOI and returns", "[IRQ]") {
    PMIRQTestEnv env;

    // Vector 0x09 (keyboard IRQ) has no BIOS HLE handler, but IDT target
    // is 16-bit, so D-bit guard sets useHLE=true.  The sendEOI fallback
    // should fire.

    SECTION("Keyboard IRQ does not crash when no HLE handler exists") {
        uint32_t eipBefore = env.cpu.getEIP();
        uint16_t csBefore = env.cpu.getSegReg(cpu::CS);
        uint32_t espBefore = env.cpu.getReg32(cpu::ESP);

        env.decoder.injectHardwareInterrupt(0x09);

        // CPU state should be unchanged — the interrupt was silently dropped
        REQUIRE(env.cpu.getEIP() == eipBefore);
        REQUIRE(env.cpu.getSegReg(cpu::CS) == csBefore);
        REQUIRE(env.cpu.getReg32(cpu::ESP) == espBefore);
    }
}

TEST_CASE("IRQ: Timer HLE increments app ticcount via Watcom ISR table", "[IRQ]") {
    PMIRQTestEnv env;

    // The ticcount HLE reads a handler address from 0x26D3E0 (Watcom RTL
    // ISR table slot 0), looks for the pattern PUSH EDX; MOV EDX,[addr],
    // and increments [addr].

    SECTION("Pattern: PUSH EDX; MOV EDX,[addr]; ... detected and incremented") {
        // Create a fake I_TimerISR at address 0x200000:
        //   52              PUSH EDX
        //   8B 15 XX XX XX XX  MOV EDX, [0x300000]
        //   42              INC EDX
        //   89 15 XX XX XX XX  MOV [0x300000], EDX
        //   5A              POP EDX
        //   C3              RET
        uint32_t handlerAddr = 0x200000;
        uint32_t counterAddr = 0x300000;

        env.mem.write8(handlerAddr + 0, 0x52);       // PUSH EDX
        env.mem.write8(handlerAddr + 1, 0x8B);       // MOV EDX, [imm32]
        env.mem.write8(handlerAddr + 2, 0x15);
        env.mem.write32(handlerAddr + 3, counterAddr);
        env.mem.write8(handlerAddr + 7, 0x42);       // INC EDX
        env.mem.write8(handlerAddr + 8, 0x89);       // MOV [imm32], EDX
        env.mem.write8(handlerAddr + 9, 0x15);
        env.mem.write32(handlerAddr + 10, counterAddr);
        env.mem.write8(handlerAddr + 14, 0x5A);      // POP EDX
        env.mem.write8(handlerAddr + 15, 0xC3);      // RET

        // Write the handler address to Watcom ISR table slot 0
        env.mem.write32(0x26D3E0, handlerAddr);

        // Set the counter to a known value
        env.mem.write32(counterAddr, 100);

        // Inject timer interrupt
        env.decoder.injectHardwareInterrupt(0x08);

        // The counter should have been incremented by 1
        REQUIRE(env.mem.read32(counterAddr) == 101);

        // And the BIOS counter at 0x46C should also have incremented
        uint16_t biosCounter = env.mem.read16(0x46C);
        REQUIRE(biosCounter > 0);
    }

    SECTION("Pattern: MOV EDX,[addr] without PUSH EDX prefix") {
        uint32_t handlerAddr = 0x200100;
        uint32_t counterAddr = 0x300100;

        // No PUSH EDX prefix — handler starts directly with MOV EDX,[addr]
        env.mem.write8(handlerAddr + 0, 0x8B);
        env.mem.write8(handlerAddr + 1, 0x15);
        env.mem.write32(handlerAddr + 2, counterAddr);
        env.mem.write8(handlerAddr + 6, 0xC3);

        env.mem.write32(0x26D3E0, handlerAddr);
        env.mem.write32(counterAddr, 42);

        env.decoder.injectHardwareInterrupt(0x08);
        REQUIRE(env.mem.read32(counterAddr) == 43);
    }

    SECTION("Empty Watcom ISR table does not increment anything") {
        env.mem.write32(0x26D3E0, 0); // no handler
        uint32_t testAddr = 0x300200;
        env.mem.write32(testAddr, 999);

        env.decoder.injectHardwareInterrupt(0x08);
        REQUIRE(env.mem.read32(testAddr) == 999); // unchanged
    }

    SECTION("Handler with non-matching pattern does not increment") {
        uint32_t handlerAddr = 0x200200;
        uint32_t counterAddr = 0x300300;

        // Handler with a different opcode pattern (NOT 8B 15)
        env.mem.write8(handlerAddr + 0, 0x52); // PUSH EDX
        env.mem.write8(handlerAddr + 1, 0x90); // NOP (not MOV EDX,[addr])
        env.mem.write8(handlerAddr + 2, 0xC3); // RET

        env.mem.write32(0x26D3E0, handlerAddr);
        env.mem.write32(counterAddr, 500);

        env.decoder.injectHardwareInterrupt(0x08);
        REQUIRE(env.mem.read32(counterAddr) == 500); // unchanged
    }
}

TEST_CASE("IRQ: BIOS sendEOI clears PIC ISR for master and slave", "[IRQ]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic(true);
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    bios.initialize();

    // Initialize PIC
    pic.write8(0x20, 0x11);
    pic.write8(0x21, 0x08);
    pic.write8(0x21, 0x04);
    pic.write8(0x21, 0x01);
    pic.write8(0x21, 0x00);

    SECTION("sendEOI for master IRQ (vector 0x08)") {
        pic.raiseIRQ(0);
        int pending = pic.getPendingInterrupt();
        REQUIRE(pending == 0x08);
        pic.acknowledgeInterrupt();

        // After acknowledge, the PIC ISR bit is set, blocking same-priority IRQs
        // sendEOI should clear it
        bios.sendEOI(0x08);

        // After EOI, a new IRQ 0 should be deliverable
        pic.raiseIRQ(0);
        REQUIRE(pic.getPendingInterrupt() == 0x08);
    }

    SECTION("sendEOI for keyboard IRQ (vector 0x09)") {
        pic.raiseIRQ(1);
        int pending = pic.getPendingInterrupt();
        REQUIRE(pending == 0x09);
        pic.acknowledgeInterrupt();

        bios.sendEOI(0x09);

        pic.raiseIRQ(1);
        REQUIRE(pic.getPendingInterrupt() == 0x09);
    }
}
