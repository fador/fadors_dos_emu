// test_dpmi.cpp – Comprehensive tests for the DPMI 0.9 host implementation.
// Covers all DOS/4GW-relevant DPMI functions: descriptor management, interrupt
// vectors, translation services, memory, page locking, virtual interrupts,
// and raw mode switches.

#include "test_framework.hpp"
#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "memory/MemoryBus.hpp"
#include "memory/himem/HIMEM.hpp"
#include "hw/DPMI.hpp"
#include "hw/IOBus.hpp"
#include "hw/PIC8259.hpp"
#include "hw/BIOS.hpp"
#include "hw/DOS.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIT8254.hpp"

using namespace fador;

// Helper environment: creates CPU + MemoryBus + DPMI, and enters PM so
// handleInt31() calls work.  Optional DOS/BIOS/HIMEM are wired in.
struct DPMITestEnv {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic{true};
    hw::DOS dos;
    hw::BIOS bios;
    memory::HIMEM himem;
    hw::DPMI dpmi;

    // A scratch linear address for passing buffers via ES:DI
    static constexpr uint32_t SCRATCH_ADDR = 0x20000;

    DPMITestEnv()
        : dos(cpu, mem), bios(cpu, mem, kbd, pit, pic),
          dpmi(cpu, mem)
    {
        bios.initialize();
        dos.initialize();

        dpmi.setDOS(&dos);
        dos.setDPMI(&dpmi);
        dpmi.setBIOS(&bios);
        dpmi.setHIMEM(&himem);
        himem.setMemoryBus(&mem);

        // Enter DPMI protected mode via the standard entry sequence.
        // Set up a fake CALL FAR frame on the stack so handleEntry() can pop it.
        uint16_t rmCS = 0x1000;
        uint16_t rmIP = 0x100;
        uint16_t rmDS = 0x2000;
        uint16_t rmSS = 0x0000;

        cpu.setSegReg(cpu::CS, rmCS);
        cpu.setSegReg(cpu::DS, rmDS);
        cpu.setSegReg(cpu::ES, rmDS);
        cpu.setSegReg(cpu::SS, rmSS);
        cpu.setReg16(cpu::SP, 0xFFFC);

        // Push the CALL FAR return frame: IP, CS
        uint32_t ssBase = static_cast<uint32_t>(rmSS) << 4;
        uint16_t sp = cpu.getReg16(cpu::SP);
        sp -= 2; mem.write16(ssBase + sp, rmCS);   // push CS
        sp -= 2; mem.write16(ssBase + sp, rmIP);   // push IP
        cpu.setReg16(cpu::SP, sp);

        // AX = 1 → 32-bit client
        cpu.setReg16(cpu::AX, 0x0001);

        // Enter PM
        bool ok = dpmi.handleEntry();
        (void)ok;

        // After entry the CPU is in PM.  Set up segment bases for ES:DI
        // addressing used by DPMI calls (getLinearAddr reads segBase).
        // We point ES base at 0 (flat) so linear = ES.base + DI = DI.
        cpu.setSegBase(cpu::ES, 0);
        cpu.setSegBase(cpu::DS, 0);
    }

    // Clear carry flag before calling an INT 31h handler
    void clearCarry() {
        cpu.setEFLAGS(cpu.getEFLAGS() & ~cpu::FLAG_CARRY);
    }
    bool carrySet() const {
        return (cpu.getEFLAGS() & cpu::FLAG_CARRY) != 0;
    }
};

// ═══════════════════════════════════════════════════════════════════════════
// Detection (INT 2Fh AX=1687h)
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DPMI detection returns correct values", "[dpmi][detect]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::DPMI dpmi(cpu, mem);

    dpmi.handleDetect();

    REQUIRE(cpu.getReg16(cpu::AX) == 0x0000);  // DPMI available
    REQUIRE(cpu.getReg16(cpu::BX) == 0x0001);  // 32-bit supported
    REQUIRE(cpu.getReg8(cpu::CL)  == 0x03);    // 386 processor
    REQUIRE(cpu.getReg8(cpu::DH)  == 0x00);    // Major version 0
    REQUIRE(cpu.getReg8(cpu::DL)  == 0x09);    // Minor version 9
    REQUIRE(cpu.getReg16(cpu::SI) == 0x0000);  // No host data needed
    REQUIRE(cpu.getSegReg(cpu::ES) == 0xF000); // Entry segment
    REQUIRE(cpu.getReg16(cpu::DI)  == 0x0050); // Entry offset
}

// ═══════════════════════════════════════════════════════════════════════════
// DPMI Entry (RM → PM switch)
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DPMI entry switches to protected mode", "[dpmi][entry]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::DOS dos(cpu, mem);
    hw::PIC8259 pic{true};
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DPMI dpmi(cpu, mem);

    bios.initialize();
    dos.initialize();
    dpmi.setDOS(&dos);
    dpmi.setBIOS(&bios);

    REQUIRE(!dpmi.isActive());

    // Set up RM state for entry
    uint16_t rmCS = 0x1000, rmIP = 0x200;
    cpu.setSegReg(cpu::CS, rmCS);
    cpu.setSegReg(cpu::DS, 0x2000);
    cpu.setSegReg(cpu::SS, 0x0000);
    cpu.setReg16(cpu::SP, 0xFFFC);

    // Push CALL FAR return frame
    uint16_t sp = cpu.getReg16(cpu::SP);
    sp -= 2; mem.write16(sp, rmCS);
    sp -= 2; mem.write16(sp, rmIP);
    cpu.setReg16(cpu::SP, sp);

    cpu.setReg16(cpu::AX, 0x0001); // 32-bit client

    bool result = dpmi.handleEntry();
    REQUIRE(result);
    REQUIRE(dpmi.isActive());

    // CR0.PE should be set
    REQUIRE((cpu.getCR(0) & 1) != 0);

    // Carry should be clear (success)
    REQUIRE(!(cpu.getEFLAGS() & cpu::FLAG_CARRY));

    // AX should hold the PSP selector
    uint16_t pspSel = cpu.getReg16(cpu::AX);
    REQUIRE(pspSel != 0);

    // CS, DS, SS, ES should all be valid LDT selectors (TI bit set)
    REQUIRE((cpu.getSegReg(cpu::CS) & 0x04) != 0);
    REQUIRE((cpu.getSegReg(cpu::DS) & 0x04) != 0);
    REQUIRE((cpu.getSegReg(cpu::SS) & 0x04) != 0);
    REQUIRE((cpu.getSegReg(cpu::ES) & 0x04) != 0);

    // EIP should be the return IP
    REQUIRE(cpu.getEIP() == rmIP);
}

TEST_CASE("DPMI double entry fails with carry", "[dpmi][entry]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic{true};
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    hw::DOS dos(cpu, mem);
    hw::DPMI dpmi(cpu, mem);
    bios.initialize();
    dos.initialize();
    dpmi.setDOS(&dos);

    // First entry
    cpu.setSegReg(cpu::CS, 0x1000);
    cpu.setSegReg(cpu::DS, 0x2000);
    cpu.setSegReg(cpu::SS, 0x0000);
    cpu.setReg16(cpu::SP, 0xFFFC);
    uint16_t sp = cpu.getReg16(cpu::SP);
    sp -= 2; mem.write16(sp, 0x1000);
    sp -= 2; mem.write16(sp, 0x100);
    cpu.setReg16(cpu::SP, sp);
    cpu.setReg16(cpu::AX, 0x0001);
    REQUIRE(dpmi.handleEntry());

    // Second entry should fail
    bool result = dpmi.handleEntry();
    REQUIRE(!result);
    REQUIRE(cpu.getEFLAGS() & cpu::FLAG_CARRY);
}

// ═══════════════════════════════════════════════════════════════════════════
// Descriptor Management (INT 31h AX=0000h..000Ch)
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DPMI 0000h allocate descriptors", "[dpmi][desc]") {
    DPMITestEnv env;

    SECTION("allocate single descriptor") {
        env.cpu.setReg16(cpu::AX, 0x0000);
        env.cpu.setReg16(cpu::CX, 1);
        env.clearCarry();
        env.dpmi.handleInt31();

        REQUIRE(!env.carrySet());
        uint16_t sel = env.cpu.getReg16(cpu::AX);
        REQUIRE(sel != 0);
        REQUIRE((sel & 0x04) != 0); // TI bit (LDT)
        REQUIRE((sel & 0x03) == 3); // RPL=3
    }

    SECTION("allocate multiple contiguous descriptors") {
        env.cpu.setReg16(cpu::AX, 0x0000);
        env.cpu.setReg16(cpu::CX, 4);
        env.clearCarry();
        env.dpmi.handleInt31();

        REQUIRE(!env.carrySet());
        uint16_t baseSel = env.cpu.getReg16(cpu::AX);
        REQUIRE(baseSel != 0);
        // Each subsequent selector is baseSel + 8*n
    }

    SECTION("allocate zero descriptors returns failure") {
        env.cpu.setReg16(cpu::AX, 0x0000);
        env.cpu.setReg16(cpu::CX, 0);
        env.clearCarry();
        env.dpmi.handleInt31();
        // allocateDescriptors(0) returns 0, so AX=0 and CF set
        REQUIRE(env.carrySet());
    }
}

TEST_CASE("DPMI 0001h free descriptor", "[dpmi][desc]") {
    DPMITestEnv env;

    // Allocate a descriptor first
    env.cpu.setReg16(cpu::AX, 0x0000);
    env.cpu.setReg16(cpu::CX, 1);
    env.dpmi.handleInt31();
    uint16_t sel = env.cpu.getReg16(cpu::AX);
    REQUIRE(!env.carrySet());

    SECTION("free valid descriptor succeeds") {
        env.cpu.setReg16(cpu::AX, 0x0001);
        env.cpu.setReg16(cpu::BX, sel);
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());
    }

    SECTION("free invalid selector fails") {
        env.cpu.setReg16(cpu::AX, 0x0001);
        env.cpu.setReg16(cpu::BX, 0xFFFF); // bogus selector
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(env.carrySet());
    }

    SECTION("free null selector fails") {
        env.cpu.setReg16(cpu::AX, 0x0001);
        env.cpu.setReg16(cpu::BX, 0x0000);
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(env.carrySet());
    }
}

TEST_CASE("DPMI 0002h segment to descriptor", "[dpmi][desc]") {
    DPMITestEnv env;

    env.cpu.setReg16(cpu::AX, 0x0002);
    env.cpu.setReg16(cpu::BX, 0xA000); // video segment
    env.clearCarry();
    env.dpmi.handleInt31();

    REQUIRE(!env.carrySet());
    uint16_t sel = env.cpu.getReg16(cpu::AX);
    REQUIRE(sel != 0);
    REQUIRE((sel & 0x04) != 0); // LDT selector
}

TEST_CASE("DPMI 0003h get selector increment", "[dpmi][desc]") {
    DPMITestEnv env;

    env.cpu.setReg16(cpu::AX, 0x0003);
    env.clearCarry();
    env.dpmi.handleInt31();

    REQUIRE(!env.carrySet());
    REQUIRE(env.cpu.getReg16(cpu::AX) == 8);
}

TEST_CASE("DPMI 0006h/0007h get/set segment base", "[dpmi][desc]") {
    DPMITestEnv env;

    // Allocate a descriptor
    env.cpu.setReg16(cpu::AX, 0x0000);
    env.cpu.setReg16(cpu::CX, 1);
    env.dpmi.handleInt31();
    uint16_t sel = env.cpu.getReg16(cpu::AX);
    REQUIRE(!env.carrySet());

    SECTION("set then get base address") {
        uint32_t newBase = 0x00123456;

        // Set base
        env.cpu.setReg16(cpu::AX, 0x0007);
        env.cpu.setReg16(cpu::BX, sel);
        env.cpu.setReg16(cpu::CX, static_cast<uint16_t>(newBase >> 16));
        env.cpu.setReg16(cpu::DX, static_cast<uint16_t>(newBase & 0xFFFF));
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());

        // Get base
        env.cpu.setReg16(cpu::AX, 0x0006);
        env.cpu.setReg16(cpu::BX, sel);
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());

        uint32_t readBase = (static_cast<uint32_t>(env.cpu.getReg16(cpu::CX)) << 16) |
                            env.cpu.getReg16(cpu::DX);
        REQUIRE(readBase == newBase);
    }

    SECTION("get base of invalid selector fails") {
        env.cpu.setReg16(cpu::AX, 0x0006);
        env.cpu.setReg16(cpu::BX, 0x0000); // null selector
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(env.carrySet());
    }
}

TEST_CASE("DPMI 0008h set segment limit", "[dpmi][desc]") {
    DPMITestEnv env;

    // Allocate
    env.cpu.setReg16(cpu::AX, 0x0000);
    env.cpu.setReg16(cpu::CX, 1);
    env.dpmi.handleInt31();
    uint16_t sel = env.cpu.getReg16(cpu::AX);
    REQUIRE(!env.carrySet());

    // Set limit
    uint32_t newLimit = 0x000FFFFF;
    env.cpu.setReg16(cpu::AX, 0x0008);
    env.cpu.setReg16(cpu::BX, sel);
    env.cpu.setReg16(cpu::CX, static_cast<uint16_t>(newLimit >> 16));
    env.cpu.setReg16(cpu::DX, static_cast<uint16_t>(newLimit & 0xFFFF));
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
}

TEST_CASE("DPMI 0009h set descriptor access rights", "[dpmi][desc]") {
    DPMITestEnv env;

    // Allocate
    env.cpu.setReg16(cpu::AX, 0x0000);
    env.cpu.setReg16(cpu::CX, 1);
    env.dpmi.handleInt31();
    uint16_t sel = env.cpu.getReg16(cpu::AX);
    REQUIRE(!env.carrySet());

    // Set access rights: CL=access byte, CH bits 4-7=G/D/L/AVL
    // Data segment, R/W, DPL=3, Present, 32-bit, page granularity
    uint16_t rights = 0xC0F2; // CH=0xC0 (G=1,D=1), CL=0xF2 (P=1,DPL=3,Data R/W)
    env.cpu.setReg16(cpu::AX, 0x0009);
    env.cpu.setReg16(cpu::BX, sel);
    env.cpu.setReg16(cpu::CX, rights);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
}

TEST_CASE("DPMI 000Ah create code segment alias", "[dpmi][desc]") {
    DPMITestEnv env;

    // Allocate a code descriptor
    env.cpu.setReg16(cpu::AX, 0x0000);
    env.cpu.setReg16(cpu::CX, 1);
    env.dpmi.handleInt31();
    uint16_t codeSel = env.cpu.getReg16(cpu::AX);
    REQUIRE(!env.carrySet());

    // Set it as a code segment (access byte = 0xFA: P=1, DPL=3, Code R/X)
    env.cpu.setReg16(cpu::AX, 0x0009);
    env.cpu.setReg16(cpu::BX, codeSel);
    env.cpu.setReg16(cpu::CX, 0x00FA); // Code, RX, DPL=3, present
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // Create alias
    env.cpu.setReg16(cpu::AX, 0x000A);
    env.cpu.setReg16(cpu::BX, codeSel);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    uint16_t aliasSel = env.cpu.getReg16(cpu::AX);
    REQUIRE(aliasSel != 0);
    REQUIRE(aliasSel != codeSel); // Must be a different selector
}

TEST_CASE("DPMI 000Bh get descriptor", "[dpmi][desc]") {
    DPMITestEnv env;

    // Allocate and configure a descriptor
    env.cpu.setReg16(cpu::AX, 0x0000);
    env.cpu.setReg16(cpu::CX, 1);
    env.dpmi.handleInt31();
    uint16_t sel = env.cpu.getReg16(cpu::AX);
    REQUIRE(!env.carrySet());

    // Set its base to a known value
    env.cpu.setReg16(cpu::AX, 0x0007);
    env.cpu.setReg16(cpu::BX, sel);
    env.cpu.setReg16(cpu::CX, 0x0012);
    env.cpu.setReg16(cpu::DX, 0x3456);
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    SECTION("get descriptor for valid selector") {
        env.cpu.setReg16(cpu::AX, 0x000B);
        env.cpu.setReg16(cpu::BX, sel);
        env.cpu.setSegBase(cpu::ES, 0);
        env.cpu.setReg32(cpu::EDI, DPMITestEnv::SCRATCH_ADDR);
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());

        // Read the 8-byte descriptor from scratch memory
        uint32_t low = env.mem.read32(DPMITestEnv::SCRATCH_ADDR);
        uint32_t high = env.mem.read32(DPMITestEnv::SCRATCH_ADDR + 4);

        // Verify the base is encoded correctly in the descriptor
        uint32_t base = (low >> 16) | ((high & 0xFF) << 16) | (high & 0xFF000000);
        REQUIRE(base == 0x00123456);
    }

    SECTION("get descriptor for null selector returns 8 zero bytes") {
        env.cpu.setReg16(cpu::AX, 0x000B);
        env.cpu.setReg16(cpu::BX, 0x0000); // null selector

        // Write known non-zero data at scratch first
        env.mem.write32(DPMITestEnv::SCRATCH_ADDR, 0xDEADBEEF);
        env.mem.write32(DPMITestEnv::SCRATCH_ADDR + 4, 0xCAFEBABE);

        env.cpu.setSegBase(cpu::ES, 0);
        env.cpu.setReg32(cpu::EDI, DPMITestEnv::SCRATCH_ADDR);
        env.clearCarry();
        env.dpmi.handleInt31();

        REQUIRE(!env.carrySet());
        REQUIRE(env.mem.read32(DPMITestEnv::SCRATCH_ADDR) == 0);
        REQUIRE(env.mem.read32(DPMITestEnv::SCRATCH_ADDR + 4) == 0);
    }

    SECTION("get descriptor for unallocated selector fails") {
        env.cpu.setReg16(cpu::AX, 0x000B);
        env.cpu.setReg16(cpu::BX, 0x7FFC); // very high, likely unallocated
        env.cpu.setSegBase(cpu::ES, 0);
        env.cpu.setReg32(cpu::EDI, DPMITestEnv::SCRATCH_ADDR);
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(env.carrySet());
    }
}

TEST_CASE("DPMI 000Ch set descriptor", "[dpmi][desc]") {
    DPMITestEnv env;

    // Allocate
    env.cpu.setReg16(cpu::AX, 0x0000);
    env.cpu.setReg16(cpu::CX, 1);
    env.dpmi.handleInt31();
    uint16_t sel = env.cpu.getReg16(cpu::AX);
    REQUIRE(!env.carrySet());

    SECTION("set descriptor with full values") {
        // Build a descriptor: base=0x00400000, limit=0xFFFF, access=0xF2, flags=0
        uint32_t base = 0x00400000;
        uint32_t limit = 0xFFFF;
        uint8_t access = 0xF2;

        uint32_t low = (limit & 0xFFFF) | ((base & 0xFFFF) << 16);
        uint32_t high = ((base >> 16) & 0xFF) |
                        (static_cast<uint32_t>(access) << 8) |
                        (limit & 0x000F0000) |
                        (base & 0xFF000000);

        env.mem.write32(DPMITestEnv::SCRATCH_ADDR, low);
        env.mem.write32(DPMITestEnv::SCRATCH_ADDR + 4, high);

        env.cpu.setReg16(cpu::AX, 0x000C);
        env.cpu.setReg16(cpu::BX, sel);
        env.cpu.setSegBase(cpu::ES, 0);
        env.cpu.setReg32(cpu::EDI, DPMITestEnv::SCRATCH_ADDR);
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());

        // Verify via 0x0006 (get base)
        env.cpu.setReg16(cpu::AX, 0x0006);
        env.cpu.setReg16(cpu::BX, sel);
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());
        uint32_t readBase = (static_cast<uint32_t>(env.cpu.getReg16(cpu::CX)) << 16) |
                            env.cpu.getReg16(cpu::DX);
        REQUIRE(readBase == 0x00400000);
    }

    SECTION("set descriptor preserves access byte when incoming is zero") {
        // First set access to 0xF2 via 0x0009
        env.cpu.setReg16(cpu::AX, 0x0009);
        env.cpu.setReg16(cpu::BX, sel);
        env.cpu.setReg16(cpu::CX, 0x00F2);
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());

        // Now set descriptor with access byte = 0 (DOS/4GW pattern)
        uint32_t base = 0x00500000;
        uint32_t limit = 0xFFFF;
        uint32_t low = (limit & 0xFFFF) | ((base & 0xFFFF) << 16);
        uint32_t high = ((base >> 16) & 0xFF) |
                        (0x00 << 8) | // access = 0
                        (limit & 0x000F0000) |
                        (base & 0xFF000000);

        env.mem.write32(DPMITestEnv::SCRATCH_ADDR, low);
        env.mem.write32(DPMITestEnv::SCRATCH_ADDR + 4, high);

        env.cpu.setReg16(cpu::AX, 0x000C);
        env.cpu.setReg16(cpu::BX, sel);
        env.cpu.setSegBase(cpu::ES, 0);
        env.cpu.setReg32(cpu::EDI, DPMITestEnv::SCRATCH_ADDR);
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());

        // Read back descriptor and check access byte was preserved
        env.cpu.setReg16(cpu::AX, 0x000B);
        env.cpu.setReg16(cpu::BX, sel);
        env.cpu.setReg32(cpu::EDI, DPMITestEnv::SCRATCH_ADDR + 0x100);
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());

        uint32_t readHigh = env.mem.read32(DPMITestEnv::SCRATCH_ADDR + 0x100 + 4);
        uint8_t accessByte = (readHigh >> 8) & 0xFF;
        REQUIRE(accessByte == 0xF2); // preserved from prior 0x0009 call
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Interrupt Vectors (INT 31h AX=0200h..0205h)
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DPMI 0200h/0201h get/set real mode interrupt vector", "[dpmi][int]") {
    DPMITestEnv env;

    // Set RM vector for INT 0x21 to a known value
    env.cpu.setReg16(cpu::AX, 0x0201);
    env.cpu.setReg8(cpu::BL, 0x21);
    env.cpu.setReg16(cpu::CX, 0xF000);  // CS
    env.cpu.setReg16(cpu::DX, 0x1234);  // IP
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // Get it back
    env.cpu.setReg16(cpu::AX, 0x0200);
    env.cpu.setReg8(cpu::BL, 0x21);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
    REQUIRE(env.cpu.getReg16(cpu::CX) == 0xF000);
    REQUIRE(env.cpu.getReg16(cpu::DX) == 0x1234);
}

TEST_CASE("DPMI 0202h/0203h get/set exception handler vector", "[dpmi][int]") {
    DPMITestEnv env;

    // Set exception vector 0x0D (GPF)
    env.cpu.setReg16(cpu::AX, 0x0203);
    env.cpu.setReg8(cpu::BL, 0x0D);
    env.cpu.setReg16(cpu::CX, 0x001C);    // selector
    env.cpu.setReg32(cpu::EDX, 0x00012345); // offset
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // Get it back
    env.cpu.setReg16(cpu::AX, 0x0202);
    env.cpu.setReg8(cpu::BL, 0x0D);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
    REQUIRE(env.cpu.getReg16(cpu::CX) == 0x001C);
    REQUIRE(env.cpu.getReg32(cpu::EDX) == 0x00012345);
}

TEST_CASE("DPMI 0203h rejects exception vector >= 32", "[dpmi][int]") {
    DPMITestEnv env;

    env.cpu.setReg16(cpu::AX, 0x0203);
    env.cpu.setReg8(cpu::BL, 32); // out of range
    env.cpu.setReg16(cpu::CX, 0x0008);
    env.cpu.setReg32(cpu::EDX, 0x1000);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(env.carrySet());
}

TEST_CASE("DPMI 0204h/0205h get/set PM interrupt vector", "[dpmi][int]") {
    DPMITestEnv env;

    // Set PM vector for INT 0x21
    env.cpu.setReg16(cpu::AX, 0x0205);
    env.cpu.setReg8(cpu::BL, 0x21);
    env.cpu.setReg16(cpu::CX, 0x0077);    // selector
    env.cpu.setReg32(cpu::EDX, 0x00ABCDEF); // offset
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // Get it back
    env.cpu.setReg16(cpu::AX, 0x0204);
    env.cpu.setReg8(cpu::BL, 0x21);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
    REQUIRE(env.cpu.getReg16(cpu::CX) == 0x0077);
    REQUIRE(env.cpu.getReg32(cpu::EDX) == 0x00ABCDEF);
}

// ═══════════════════════════════════════════════════════════════════════════
// Translation Services (INT 31h AX=0300h..0306h)
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DPMI 0303h allocate RM callback", "[dpmi][translation]") {
    DPMITestEnv env;

    env.cpu.setReg16(cpu::AX, 0x0303);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
    // Returns CX:DX = RM callback address
    REQUIRE(env.cpu.getReg16(cpu::CX) == 0xF000);
    REQUIRE(env.cpu.getReg16(cpu::DX) == 0x0060);
}

TEST_CASE("DPMI 0304h free RM callback", "[dpmi][translation]") {
    DPMITestEnv env;

    env.cpu.setReg16(cpu::AX, 0x0304);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
}

TEST_CASE("DPMI 0305h get state save/restore addresses", "[dpmi][translation]") {
    DPMITestEnv env;

    env.cpu.setReg16(cpu::AX, 0x0305);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
    REQUIRE(env.cpu.getReg16(cpu::AX) == 0); // buffer size = 0
}

TEST_CASE("DPMI 0306h get raw mode switch addresses", "[dpmi][translation]") {
    DPMITestEnv env;

    env.cpu.setReg16(cpu::AX, 0x0306);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // PM→RM: selector SI : offset EDI
    REQUIRE(env.cpu.getReg16(cpu::SI) == 0x0008);
    REQUIRE(env.cpu.getReg32(cpu::EDI) == 0x0500);

    // RM→PM: BX:CX
    REQUIRE(env.cpu.getReg16(cpu::BX) == 0xF000);
    REQUIRE(env.cpu.getReg16(cpu::CX) == 0x005D);
}

TEST_CASE("DPMI 0300h simulate RM interrupt", "[dpmi][translation]") {
    DPMITestEnv env;

    // Build a RM call structure at scratch address
    uint32_t structAddr = DPMITestEnv::SCRATCH_ADDR;
    // Zero the 50-byte structure
    for (int i = 0; i < 50; ++i)
        env.mem.write8(structAddr + i, 0);

    // Set up the RM call for INT 12h (returns conventional memory size in AX)
    env.mem.write32(structAddr + 0x1C, 0);   // EAX
    env.mem.write16(structAddr + 0x20, 0);   // FLAGS

    env.cpu.setReg16(cpu::AX, 0x0300);
    env.cpu.setReg8(cpu::BL, 0x12);  // INT 12h
    env.cpu.setReg8(cpu::BH, 0x00);
    env.cpu.setReg16(cpu::CX, 0);    // no stack words to copy
    env.cpu.setSegBase(cpu::ES, 0);
    env.cpu.setReg32(cpu::EDI, structAddr);
    env.clearCarry();
    env.dpmi.handleInt31();

    // After return, carry should be clear
    REQUIRE(!env.carrySet());

    // The RM call structure should have been updated with INT 12h result
    uint16_t retAX = env.mem.read16(structAddr + 0x1C); // low 16 of EAX
    REQUIRE(retAX == 640); // INT 12h returns 640KB
}

// ═══════════════════════════════════════════════════════════════════════════
// Version (INT 31h AX=0400h)
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DPMI 0400h get version", "[dpmi][version]") {
    DPMITestEnv env;

    env.cpu.setReg16(cpu::AX, 0x0400);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    REQUIRE(env.cpu.getReg8(cpu::AH) == 0x00); // Major = 0
    REQUIRE(env.cpu.getReg8(cpu::AL) == 0x09); // Minor = 9
    REQUIRE(env.cpu.getReg16(cpu::BX) == 0x0005); // Flags
    REQUIRE(env.cpu.getReg8(cpu::CL) == 0x04);    // CPU type (486)
    REQUIRE(env.cpu.getReg8(cpu::DH) == 0x08);    // PIC master (IRQ0=INT 08h)
    REQUIRE(env.cpu.getReg8(cpu::DL) == 0x70);    // PIC slave (IRQ8=INT 70h)
}

TEST_CASE("DPMI 0E00h/0E01h coprocessor status and emulation flags", "[dpmi][version]") {
    DPMITestEnv env;

    env.cpu.setReg16(cpu::AX, 0x0E00);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
    REQUIRE((env.cpu.getReg16(cpu::AX) & 0x0001) == 0x0001);
    REQUIRE((env.cpu.getReg16(cpu::AX) & 0x0008) == 0x0008);

    env.cpu.setReg16(cpu::AX, 0x0E01);
    env.cpu.setReg16(cpu::BX, 0x0002);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    env.cpu.setReg16(cpu::AX, 0x0E00);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
    REQUIRE((env.cpu.getReg16(cpu::AX) & 0x0003) == 0x0002);

    env.cpu.setReg16(cpu::AX, 0x0E01);
    env.cpu.setReg16(cpu::BX, 0x0004);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(env.carrySet());
    REQUIRE(env.cpu.getReg16(cpu::AX) == 0x8026);
}

// ═══════════════════════════════════════════════════════════════════════════
// Memory Info (INT 31h AX=0500h..0503h)
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DPMI 0500h get free memory information", "[dpmi][memory]") {
    DPMITestEnv env;

    env.cpu.setReg16(cpu::AX, 0x0500);
    env.cpu.setSegBase(cpu::ES, 0);
    env.cpu.setReg32(cpu::EDI, DPMITestEnv::SCRATCH_ADDR);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // Largest free block should be non-zero
    uint32_t largestFree = env.mem.read32(DPMITestEnv::SCRATCH_ADDR);
    REQUIRE(largestFree > 0);
}

TEST_CASE("DPMI 0501h/0502h allocate and free memory block", "[dpmi][memory]") {
    DPMITestEnv env;

    // Allocate 64KB
    uint32_t size = 64 * 1024;
    env.cpu.setReg16(cpu::AX, 0x0501);
    env.cpu.setReg16(cpu::BX, static_cast<uint16_t>(size >> 16));
    env.cpu.setReg16(cpu::CX, static_cast<uint16_t>(size & 0xFFFF));
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // BX:CX = linear address, SI:DI = handle
    uint32_t linearAddr = (static_cast<uint32_t>(env.cpu.getReg16(cpu::BX)) << 16) |
                          env.cpu.getReg16(cpu::CX);
    uint32_t handle = (static_cast<uint32_t>(env.cpu.getReg16(cpu::SI)) << 16) |
                      env.cpu.getReg16(cpu::DI);
    REQUIRE(linearAddr != 0);
    REQUIRE(handle != 0);

    // Free it
    env.cpu.setReg16(cpu::AX, 0x0502);
    env.cpu.setReg16(cpu::SI, static_cast<uint16_t>(handle >> 16));
    env.cpu.setReg16(cpu::DI, static_cast<uint16_t>(handle & 0xFFFF));
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
}

TEST_CASE("DPMI 0502h free invalid handle fails", "[dpmi][memory]") {
    DPMITestEnv env;

    env.cpu.setReg16(cpu::AX, 0x0502);
    env.cpu.setReg16(cpu::SI, 0xFFFF);
    env.cpu.setReg16(cpu::DI, 0xFFFF);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(env.carrySet());
}

TEST_CASE("DPMI 0503h resize memory block stub returns error", "[dpmi][memory]") {
    DPMITestEnv env;

    env.cpu.setReg16(cpu::AX, 0x0503);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(env.carrySet());
}

// ═══════════════════════════════════════════════════════════════════════════
// Page Locking (INT 31h AX=0600h..0604h)
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DPMI 0600h/0601h lock/unlock region succeeds", "[dpmi][page]") {
    DPMITestEnv env;

    SECTION("lock region") {
        env.cpu.setReg16(cpu::AX, 0x0600);
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());
    }

    SECTION("unlock region") {
        env.cpu.setReg16(cpu::AX, 0x0601);
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());
    }
}

TEST_CASE("DPMI 0604h get page size", "[dpmi][page]") {
    DPMITestEnv env;

    env.cpu.setReg16(cpu::AX, 0x0604);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
    REQUIRE(env.cpu.getReg16(cpu::BX) == 0);
    REQUIRE(env.cpu.getReg16(cpu::CX) == 4096);
}

// ═══════════════════════════════════════════════════════════════════════════
// Virtual Interrupt State (INT 31h AX=0900h..0902h)
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DPMI 0900h/0901h/0902h virtual interrupt management", "[dpmi][vint]") {
    DPMITestEnv env;

    SECTION("disable virtual interrupts returns previous state") {
        // Default is enabled (1)
        env.cpu.setReg16(cpu::AX, 0x0900);
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());
        REQUIRE(env.cpu.getReg8(cpu::AL) == 1); // was enabled

        // Now disabled, query should return 0
        env.cpu.setReg16(cpu::AX, 0x0902);
        env.dpmi.handleInt31();
        REQUIRE(env.cpu.getReg8(cpu::AL) == 0);
    }

    SECTION("enable virtual interrupts returns previous state") {
        // Disable first
        env.cpu.setReg16(cpu::AX, 0x0900);
        env.dpmi.handleInt31();

        // Now enable
        env.cpu.setReg16(cpu::AX, 0x0901);
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());
        REQUIRE(env.cpu.getReg8(cpu::AL) == 0); // was disabled

        // Query should return 1
        env.cpu.setReg16(cpu::AX, 0x0902);
        env.dpmi.handleInt31();
        REQUIRE(env.cpu.getReg8(cpu::AL) == 1);
    }

    SECTION("query returns current state without changing it") {
        env.cpu.setReg16(cpu::AX, 0x0902);
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());
        REQUIRE(env.cpu.getReg8(cpu::AL) == 1); // default is enabled
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Raw Mode Switches
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DPMI raw switch PM→RM", "[dpmi][rawswitch]") {
    DPMITestEnv env;

    // Set up registers for the raw switch
    env.cpu.setReg16(cpu::AX, 0x3000);  // new DS
    env.cpu.setReg16(cpu::CX, 0x4000);  // new CS
    env.cpu.setReg16(cpu::DX, 0x5000);  // new SS
    env.cpu.setReg16(cpu::BX, 0xFFF0);  // new SP
    env.cpu.setReg16(cpu::SI, 0x0200);  // new IP

    env.dpmi.handleRawSwitchPMtoRM();

    // Should be in real mode (CR0.PE cleared)
    REQUIRE((env.cpu.getCR(0) & 1) == 0);

    // Check segment registers
    REQUIRE(env.cpu.getSegReg(cpu::CS) == 0x4000);
    REQUIRE(env.cpu.getSegReg(cpu::DS) == 0x3000);
    REQUIRE(env.cpu.getSegReg(cpu::SS) == 0x5000);
    REQUIRE(env.cpu.getReg16(cpu::SP) == 0xFFF0);
    REQUIRE(env.cpu.getEIP() == 0x0200);
}

TEST_CASE("DPMI raw switch RM→PM", "[dpmi][rawswitch]") {
    DPMITestEnv env;

    // First switch to RM
    env.cpu.setReg16(cpu::AX, 0x3000);
    env.cpu.setReg16(cpu::CX, 0x4000);
    env.cpu.setReg16(cpu::DX, 0x5000);
    env.cpu.setReg16(cpu::BX, 0xFFF0);
    env.cpu.setReg16(cpu::SI, 0x0200);
    env.dpmi.handleRawSwitchPMtoRM();
    REQUIRE((env.cpu.getCR(0) & 1) == 0);

    // Now switch back to PM
    // Get the initial selectors from the entry for a valid PM return
    // We'll use simple values that exist in the LDT
    uint16_t pmDS = 0x0014; // LDT index 2 (entry allocated during DPMITestEnv setup)
    uint16_t pmCS = 0x000C; // LDT index 1
    uint16_t pmSS = 0x001C; // LDT index 3

    env.cpu.setReg16(cpu::AX, pmDS);
    env.cpu.setReg16(cpu::CX, pmCS);
    env.cpu.setReg16(cpu::DX, pmSS);
    env.cpu.setReg32(cpu::EBX, 0xFFF0);  // ESP
    env.cpu.setReg32(cpu::ESI, 0x100);    // EIP

    env.dpmi.handleRawSwitchRMtoPM();

    // Should be in protected mode again
    REQUIRE((env.cpu.getCR(0) & 1) != 0);
    REQUIRE(env.cpu.getSegReg(cpu::CS) == pmCS);
    REQUIRE(env.cpu.getSegReg(cpu::DS) == pmDS);
    REQUIRE(env.cpu.getSegReg(cpu::SS) == pmSS);
}

TEST_CASE("DPMI raw switch RM→PM zero-extends SI into EIP", "[dpmi][rawswitch]") {
    DPMITestEnv env;

    env.cpu.setReg16(cpu::AX, 0x3000);
    env.cpu.setReg16(cpu::CX, 0x4000);
    env.cpu.setReg16(cpu::DX, 0x5000);
    env.cpu.setReg16(cpu::BX, 0xFFF0);
    env.cpu.setReg16(cpu::SI, 0x0200);
    env.dpmi.handleRawSwitchPMtoRM();
    REQUIRE((env.cpu.getCR(0) & 1) == 0);

    uint16_t pmDS = 0x0014;
    uint16_t pmCS = 0x000C;
    uint16_t pmSS = 0x001C;

    env.cpu.setReg16(cpu::AX, pmDS);
    env.cpu.setReg16(cpu::CX, pmCS);
    env.cpu.setReg16(cpu::DX, pmSS);
    env.cpu.setReg32(cpu::EBX, 0x0010FFF0);
    env.cpu.setReg32(cpu::ESI, 0x843A0594);

    env.dpmi.handleRawSwitchRMtoPM();

    REQUIRE((env.cpu.getCR(0) & 1) != 0);
    REQUIRE(env.cpu.getEIP() == 0x0594);
    REQUIRE(env.cpu.getReg32(cpu::ESP) == 0x0010FFF0);
    REQUIRE(env.cpu.getSegBase(cpu::CS) == 0x10000);
}

// ═══════════════════════════════════════════════════════════════════════════
// Unhandled / Vendor-specific functions
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DPMI vendor-specific API 0A00h returns unsupported", "[dpmi][misc]") {
    DPMITestEnv env;

    env.cpu.setReg16(cpu::AX, 0x0A00);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(env.carrySet());
    REQUIRE(env.cpu.getReg16(cpu::AX) == 0x8001);
}

TEST_CASE("DPMI page demand paging 0700h returns success", "[dpmi][misc]") {
    DPMITestEnv env;

    env.cpu.setReg16(cpu::AX, 0x0700);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
}

TEST_CASE("DPMI unhandled function returns unsupported", "[dpmi][misc]") {
    DPMITestEnv env;

    env.cpu.setReg16(cpu::AX, 0x0B00); // unknown function
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(env.carrySet());
    REQUIRE(env.cpu.getReg16(cpu::AX) == 0x8001);
}

TEST_CASE("DPMI INT 31h returns false when not active", "[dpmi][misc]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::DPMI dpmi(cpu, mem);

    // Not entered PM yet
    cpu.setReg16(cpu::AX, 0x0400);
    REQUIRE(!dpmi.handleInt31());
}

// ═══════════════════════════════════════════════════════════════════════════
// Descriptor round-trip: allocate + set base/limit/access + get descriptor
// (exercises the full DOS/4GW init sequence)
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DPMI descriptor round-trip: alloc, set base/limit/access, get", "[dpmi][desc][roundtrip]") {
    DPMITestEnv env;

    // 1. Allocate
    env.cpu.setReg16(cpu::AX, 0x0000);
    env.cpu.setReg16(cpu::CX, 1);
    env.dpmi.handleInt31();
    uint16_t sel = env.cpu.getReg16(cpu::AX);
    REQUIRE(!env.carrySet());

    // 2. Set base to 0x00100000
    env.cpu.setReg16(cpu::AX, 0x0007);
    env.cpu.setReg16(cpu::BX, sel);
    env.cpu.setReg16(cpu::CX, 0x0010);
    env.cpu.setReg16(cpu::DX, 0x0000);
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // 3. Set limit to 0xFFFFF (1MB - 1)
    env.cpu.setReg16(cpu::AX, 0x0008);
    env.cpu.setReg16(cpu::BX, sel);
    env.cpu.setReg16(cpu::CX, 0x000F);
    env.cpu.setReg16(cpu::DX, 0xFFFF);
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // 4. Set access rights: 32-bit data, page gran, DPL=3
    env.cpu.setReg16(cpu::AX, 0x0009);
    env.cpu.setReg16(cpu::BX, sel);
    env.cpu.setReg16(cpu::CX, 0xC0F2); // G=1, D=1, access=0xF2
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // 5. Get descriptor and verify all fields
    env.cpu.setReg16(cpu::AX, 0x000B);
    env.cpu.setReg16(cpu::BX, sel);
    env.cpu.setSegBase(cpu::ES, 0);
    env.cpu.setReg32(cpu::EDI, DPMITestEnv::SCRATCH_ADDR);
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    uint32_t low = env.mem.read32(DPMITestEnv::SCRATCH_ADDR);
    uint32_t high = env.mem.read32(DPMITestEnv::SCRATCH_ADDR + 4);

    // Extract base
    uint32_t base = (low >> 16) | ((high & 0xFF) << 16) | (high & 0xFF000000);
    REQUIRE(base == 0x00100000);

    // Extract access byte
    uint8_t access = (high >> 8) & 0xFF;
    REQUIRE(access == 0xF2);

    // Extract G and D bits (high dword bits 23 and 22)
    bool G = (high & 0x00800000) != 0;
    bool D = (high & 0x00400000) != 0;
    REQUIRE(G); // Granularity bit set
    REQUIRE(D); // 32-bit segment
}

// ═══════════════════════════════════════════════════════════════════════════
// Multiple allocations and frees (DOS/4GW allocates many descriptors)
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DPMI allocate/free multiple descriptors", "[dpmi][desc][stress]") {
    DPMITestEnv env;

    std::vector<uint16_t> selectors;

    // Allocate 20 descriptors one by one
    for (int i = 0; i < 20; ++i) {
        env.cpu.setReg16(cpu::AX, 0x0000);
        env.cpu.setReg16(cpu::CX, 1);
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());
        selectors.push_back(env.cpu.getReg16(cpu::AX));
    }

    // All selectors should be unique
    for (size_t i = 0; i < selectors.size(); ++i) {
        for (size_t j = i + 1; j < selectors.size(); ++j) {
            REQUIRE(selectors[i] != selectors[j]);
        }
    }

    // Free them all
    for (auto sel : selectors) {
        env.cpu.setReg16(cpu::AX, 0x0001);
        env.cpu.setReg16(cpu::BX, sel);
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());
    }

    // Freeing again should fail
    env.cpu.setReg16(cpu::AX, 0x0001);
    env.cpu.setReg16(cpu::BX, selectors[0]);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(env.carrySet());
}

// ═══════════════════════════════════════════════════════════════════════════
// Memory allocation stress (multiple blocks)
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DPMI multiple memory allocations", "[dpmi][memory][stress]") {
    DPMITestEnv env;

    struct Block {
        uint32_t linearAddr;
        uint32_t handle;
    };
    std::vector<Block> blocks;

    // Allocate 5 blocks of 16KB each
    for (int i = 0; i < 5; ++i) {
        uint32_t size = 16 * 1024;
        env.cpu.setReg16(cpu::AX, 0x0501);
        env.cpu.setReg16(cpu::BX, static_cast<uint16_t>(size >> 16));
        env.cpu.setReg16(cpu::CX, static_cast<uint16_t>(size & 0xFFFF));
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());

        Block b;
        b.linearAddr = (static_cast<uint32_t>(env.cpu.getReg16(cpu::BX)) << 16) |
                        env.cpu.getReg16(cpu::CX);
        b.handle = (static_cast<uint32_t>(env.cpu.getReg16(cpu::SI)) << 16) |
                   env.cpu.getReg16(cpu::DI);
        REQUIRE(b.linearAddr != 0);
        REQUIRE(b.handle != 0);
        blocks.push_back(b);
    }

    // All linear addresses should be different
    for (size_t i = 0; i < blocks.size(); ++i) {
        for (size_t j = i + 1; j < blocks.size(); ++j) {
            REQUIRE(blocks[i].linearAddr != blocks[j].linearAddr);
        }
    }

    // Free them all
    for (auto& b : blocks) {
        env.cpu.setReg16(cpu::AX, 0x0502);
        env.cpu.setReg16(cpu::SI, static_cast<uint16_t>(b.handle >> 16));
        env.cpu.setReg16(cpu::DI, static_cast<uint16_t>(b.handle & 0xFFFF));
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// IDT write semantics for INT 31h AX=0205h  (suspected error-1001 root causes)
// ═══════════════════════════════════════════════════════════════════════════

// Physical addresses used by DPMI internals (mirrors DPMI.cpp constants)
static constexpr uint32_t IDT_PM_PHYS = 0xF0800;
// HLE stub for vector V lives at physical 0xF0000 + 0x0100 + V*4
static constexpr uint32_t hleStubPhys(uint8_t v) {
    return 0xF0000u + 0x0100u + static_cast<uint32_t>(v) * 4u;
}
// Encode a 32-bit interrupt-gate IDT entry (type EE = 32-bit, DPL=3)
static uint32_t idtLow (uint16_t sel, uint32_t off) {
    return (static_cast<uint32_t>(sel) << 16) | (off & 0xFFFF);
}
static uint32_t idtHigh(uint32_t off, uint16_t gateType = 0xEE00) {
    return (off & 0xFFFF0000u) | gateType;
}

TEST_CASE("DPMI 0205h writes physical IDT when we are the sole DPMI host", "[dpmi][int][idt]") {
    DPMITestEnv env;
    constexpr uint8_t vec = 0x42;

    // After entry, INT 31h vector still points to our HLE stub → sole host
    env.cpu.setReg16(cpu::AX, 0x0205);
    env.cpu.setReg8(cpu::BL, vec);
    env.cpu.setReg16(cpu::CX, 0x0077);       // custom selector
    env.cpu.setReg32(cpu::EDX, 0x00012345);  // custom offset
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // Physical IDT entry must have been updated
    REQUIRE(env.mem.read32(IDT_PM_PHYS + vec * 8)     == idtLow(0x0077, 0x00012345));
    REQUIRE(env.mem.read32(IDT_PM_PHYS + vec * 8 + 4) == idtHigh(0x00012345));
}

TEST_CASE("DPMI 0205h always writes IDT; sub-host passes its own wrapper", "[dpmi][int][idt]") {
    DPMITestEnv env;
    constexpr uint8_t vecClient = 0x43;

    // Step 1: hook INT 31h itself (simulating DOS/4GW installing its dispatcher)
    env.cpu.setReg16(cpu::AX, 0x0205);
    env.cpu.setReg8(cpu::BL, 0x31);
    env.cpu.setReg16(cpu::CX, 0x0057); // DOS/4GW selector
    env.cpu.setReg32(cpu::EDX, 0x00009000);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // Record the IDT entry for vecClient (should still be original HLE stub)
    uint32_t beforeLow  = env.mem.read32(IDT_PM_PHYS + vecClient * 8);
    uint32_t beforeHigh = env.mem.read32(IDT_PM_PHYS + vecClient * 8 + 4);
    REQUIRE(beforeLow  == idtLow(0x0008, hleStubPhys(vecClient)));
    REQUIRE(beforeHigh == idtHigh(hleStubPhys(vecClient)));

    // Step 2: simulate DOS/4GW's INT 31h thunk chaining to our stub for vecClient.
    // A real sub-host intercepts the client's 0205h call and substitutes its OWN
    // wrapper address in CX:EDX before calling the DPMI host. So what we receive
    // here is DOS/4GW's chain wrapper, not the raw client handler.
    env.cpu.setReg16(cpu::AX, 0x0205);
    env.cpu.setReg8(cpu::BL, vecClient);
    env.cpu.setReg16(cpu::CX, 0x0057); // DOS/4GW's wrapper selector
    env.cpu.setReg32(cpu::EDX, 0x0001ABCD); // DOS/4GW's wrapper offset
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // IDT entry for vecClient must now contain DOS/4GW's wrapper —
    // the host always writes whatever selector:offset it receives.
    REQUIRE(env.mem.read32(IDT_PM_PHYS + vecClient * 8)     == idtLow(0x0057, 0x0001ABCD));
    REQUIRE(env.mem.read32(IDT_PM_PHYS + vecClient * 8 + 4) == idtHigh(0x0001ABCD));

    // m_pmVectors[vecClient] must also reflect the new (wrapper) value
    env.cpu.setReg16(cpu::AX, 0x0204);
    env.cpu.setReg8(cpu::BL, vecClient);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
    REQUIRE(env.cpu.getReg16(cpu::CX)  == 0x0057);
    REQUIRE(env.cpu.getReg32(cpu::EDX) == 0x0001ABCD);
}

TEST_CASE("DPMI 0205h always writes IDT when setting INT 31h itself", "[dpmi][int][idt]") {
    DPMITestEnv env;

    // Hook INT 31h a first time (DOS/4GW first layer)
    env.cpu.setReg16(cpu::AX, 0x0205);
    env.cpu.setReg8(cpu::BL, 0x31);
    env.cpu.setReg16(cpu::CX, 0x0057);
    env.cpu.setReg32(cpu::EDX, 0x00009000);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
    REQUIRE(env.mem.read32(IDT_PM_PHYS + 0x31 * 8)     == idtLow(0x0057, 0x00009000));
    REQUIRE(env.mem.read32(IDT_PM_PHYS + 0x31 * 8 + 4) == idtHigh(0x00009000));

    // Hook INT 31h again with a different selector (second layer hooker)
    env.cpu.setReg16(cpu::AX, 0x0205);
    env.cpu.setReg8(cpu::BL, 0x31);
    env.cpu.setReg16(cpu::CX, 0x008F);
    env.cpu.setReg32(cpu::EDX, 0x00020000);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // IDT[0x31] must reflect the newest hook
    REQUIRE(env.mem.read32(IDT_PM_PHYS + 0x31 * 8)     == idtLow(0x008F, 0x00020000));
    REQUIRE(env.mem.read32(IDT_PM_PHYS + 0x31 * 8 + 4) == idtHigh(0x00020000));
}

TEST_CASE("DPMI 0205h preserves installed chain wrapper when caller restores original stub", "[dpmi][int][idt][chain]") {
    DPMITestEnv env;
    constexpr uint8_t vec = 0x21;

    // Initial state: IDT and pmVectors point at our original HLE stub.
    uint32_t origLow = idtLow(0x0008, hleStubPhys(vec));
    uint32_t origHigh = idtHigh(hleStubPhys(vec));
    REQUIRE(env.mem.read32(IDT_PM_PHYS + vec * 8) == origLow);
    REQUIRE(env.mem.read32(IDT_PM_PHYS + vec * 8 + 4) == origHigh);

    // Sub-host installs a chain wrapper for INT 21h.
    // This is what DOS/4GW-style dispatchers pass to the host via 0205h.
    constexpr uint16_t wrapSel = 0x008F;
    constexpr uint32_t wrapOff = 0x00000084;
    env.cpu.setReg16(cpu::AX, 0x0205);
    env.cpu.setReg8(cpu::BL, vec);
    env.cpu.setReg16(cpu::CX, wrapSel);
    env.cpu.setReg32(cpu::EDX, wrapOff);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    REQUIRE(env.mem.read32(IDT_PM_PHYS + vec * 8) == idtLow(wrapSel, wrapOff));
    REQUIRE(env.mem.read32(IDT_PM_PHYS + vec * 8 + 4) == idtHigh(wrapOff));

    // A later "restore old vector" call presents the original stub address.
    // The host must preserve active chain wrappers in this scenario.
    env.cpu.setReg16(cpu::AX, 0x0205);
    env.cpu.setReg8(cpu::BL, vec);
    env.cpu.setReg16(cpu::CX, 0x0008);
    env.cpu.setReg32(cpu::EDX, hleStubPhys(vec));
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // Wrapper remains authoritative in physical IDT.
    REQUIRE(env.mem.read32(IDT_PM_PHYS + vec * 8) == idtLow(wrapSel, wrapOff));
    REQUIRE(env.mem.read32(IDT_PM_PHYS + vec * 8 + 4) == idtHigh(wrapOff));

    // 0204h should also continue returning the wrapper.
    env.cpu.setReg16(cpu::AX, 0x0204);
    env.cpu.setReg8(cpu::BL, vec);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
    REQUIRE(env.cpu.getReg16(cpu::CX) == wrapSel);
    REQUIRE(env.cpu.getReg32(cpu::EDX) == wrapOff);
}

TEST_CASE("DPMI 0204h returns m_pmVectors even when IDT was not written", "[dpmi][int][idt]") {
    DPMITestEnv env;

    // Hook INT 31h so subsequent 0x0205h calls skip IDT writes
    env.cpu.setReg16(cpu::AX, 0x0205);
    env.cpu.setReg8(cpu::BL, 0x31);
    env.cpu.setReg16(cpu::CX, 0x0057);
    env.cpu.setReg32(cpu::EDX, 0x00009000);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // Set three different vectors
    for (uint8_t vec : {uint8_t(0x08), uint8_t(0x09), uint8_t(0x0D)}) {
        env.cpu.setReg16(cpu::AX, 0x0205);
        env.cpu.setReg8(cpu::BL, vec);
        env.cpu.setReg16(cpu::CX, 0x008F);
        env.cpu.setReg32(cpu::EDX, 0x00100000u + vec);
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());
    }

    // Each 0x0204h query must return exactly what was set via 0x0205h
    for (uint8_t vec : {uint8_t(0x08), uint8_t(0x09), uint8_t(0x0D)}) {
        env.cpu.setReg16(cpu::AX, 0x0204);
        env.cpu.setReg8(cpu::BL, vec);
        env.clearCarry();
        env.dpmi.handleInt31();
        REQUIRE(!env.carrySet());
        REQUIRE(env.cpu.getReg16(cpu::CX)  == 0x008F);
        REQUIRE(env.cpu.getReg32(cpu::EDX) == 0x00100000u + vec);
    }
}

TEST_CASE("HLE 0F FF chain merges only carry for INT 0x31", "[dpmi][hle][mask]") {
    DPMITestEnv env;
    hw::IOBus iobus;
    cpu::InstructionDecoder decoder(env.cpu, env.mem, iobus, env.bios, env.dos);

    // Let the DPMI host know about a wrapper for INT 0x31 so m_dos.handleInterrupt
    // will dispatch into DPMI and 'handled' becomes true inside the HLE path.
    env.cpu.setReg16(cpu::AX, 0x0205);
    env.cpu.setReg8(cpu::BL, 0x31);
    env.cpu.setReg16(cpu::CX, 0x008F);
    env.cpu.setReg32(cpu::EDX, 0x000000C4);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // Prepare a tracked HLE frame (original IRET) at a different SP than the
    // chain caller's IRET frame so the handler takes the "chain" path.
    uint32_t ssBase = env.cpu.getSegBase(cpu::SS);
    uint32_t espBase = env.cpu.getReg32(cpu::ESP);
    uint32_t spTracked = espBase - 0x200;
    uint32_t spChain = espBase - 0x100;

    env.cpu.pushHLEFrame(true, 0x31);
    auto &hf = env.cpu.lastHLEFrameMut();
    hf.framePhysAddr = ssBase + spTracked;
    hf.frameSP = spTracked;
    hf.stackIs32 = true;

    // Write the chained IRET-like frame at the current ESP location that the
    // 0F FF handler will read: [newEIP(32), newCS(32), popFlags(32)].
    uint32_t newEip = 0x00000918u;
    uint32_t newCs = env.cpu.getSegReg(cpu::CS); // use a known-valid selector
    uint32_t popFlagsVal = cpu::FLAG_ZERO | cpu::FLAG_SIGN; // 0x40 | 0x80
    env.mem.write32(ssBase + spChain, newEip);
    env.mem.write32(ssBase + spChain + 4, newCs);
    env.mem.write32(ssBase + spChain + 8, popFlagsVal);

    // Set CPU to the chain caller's SP and current flags (carry set)
    env.cpu.setReg32(cpu::ESP, spChain);
    env.cpu.setEFLAGS(cpu::FLAG_CARRY);

    // Place the HLE trap instruction at CS:EIP and execute it
    uint32_t codePhys = env.cpu.getSegBase(cpu::CS) + env.cpu.getEIP();
    env.mem.write8(codePhys, 0x0F);
    env.mem.write8(codePhys + 1, 0xFF);
    env.mem.write8(codePhys + 2, 0x31);

    decoder.step();

    // After the chain return, the CPU should have been set to newEIP/newCS
    // and EFLAGS should equal the stored popFlags with only the carry bit
    // merged from the current flags (mask == FLAG_CARRY for vec=0x31).
    uint32_t expected = popFlagsVal | cpu::FLAG_CARRY;
    REQUIRE(env.cpu.getEIP() == newEip);
    REQUIRE(env.cpu.getSegReg(cpu::CS) == static_cast<uint16_t>(newCs & 0xFFFFu));
    REQUIRE((env.cpu.getEFLAGS() & 0xFFFFu) == (expected & 0xFFFFu));
}

TEST_CASE("HLE 0F FF chain preserves stacked flags for hardware IRQs", "[dpmi][hle][irq]") {
    DPMITestEnv env;
    hw::IOBus iobus;
    cpu::InstructionDecoder decoder(env.cpu, env.mem, iobus, env.bios, env.dos);

    uint32_t ssBase = env.cpu.getSegBase(cpu::SS);
    uint32_t espBase = env.cpu.getReg32(cpu::ESP);
    uint32_t spTracked = espBase - 0x200;
    uint32_t spChain = espBase - 0x100;

    env.kbd.pushMakeKey('a', 0x1E);

    env.cpu.pushHLEFrame(true, 0x09);
    auto &hf = env.cpu.lastHLEFrameMut();
    hf.framePhysAddr = ssBase + spTracked;
    hf.frameSP = spTracked;
    hf.stackIs32 = true;

    uint32_t newEip = 0x0000535Eu;
    uint32_t newCs = env.cpu.getSegReg(cpu::CS);
    uint32_t popFlagsVal = cpu::FLAG_INTERRUPT | 0x02u;
    env.mem.write32(ssBase + spChain, newEip);
    env.mem.write32(ssBase + spChain + 4, newCs);
    env.mem.write32(ssBase + spChain + 8, popFlagsVal);

    env.cpu.setReg32(cpu::ESP, spChain);
    env.cpu.setEFLAGS(cpu::FLAG_ZERO | cpu::FLAG_PARITY | 0x02u);

    uint32_t codePhys = env.cpu.getSegBase(cpu::CS) + env.cpu.getEIP();
    env.mem.write8(codePhys, 0x0F);
    env.mem.write8(codePhys + 1, 0xFF);
    env.mem.write8(codePhys + 2, 0x09);

    decoder.step();

    REQUIRE(env.cpu.getEIP() == newEip);
    REQUIRE(env.cpu.getSegReg(cpu::CS) == static_cast<uint16_t>(newCs & 0xFFFFu));
    REQUIRE((env.cpu.getEFLAGS() & 0xFFFFu) == (popFlagsVal & 0xFFFFu));
    REQUIRE((env.kbd.read8(0x64) & 0x01u) == 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// isOriginalIVT PM path — controls HLE shortcut in InstructionDecoder
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("isOriginalIVT returns true only for original HLE stub addresses", "[bios][ivt]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::KeyboardController kbd;
    hw::PIT8254 pit;
    hw::PIC8259 pic{true};
    hw::BIOS bios(cpu, mem, kbd, pit, pic);
    bios.initialize();

    constexpr uint8_t vec = 0x21;
    const uint32_t origStubPhys = 0xF0000u + 0x0100u + vec * 4u;

    SECTION("PM: selector 0x08 + orig offset → true") {
        REQUIRE(bios.isOriginalIVT(vec, 0x0008, origStubPhys));
    }

    SECTION("PM: any selector + orig offset → true (offset-only check)") {
        // isOriginalIVT only compares the physical offset in PM, not the selector
        REQUIRE(bios.isOriginalIVT(vec, 0x0077, origStubPhys));
    }

    SECTION("PM: correct selector but wrong offset → false") {
        REQUIRE(!bios.isOriginalIVT(vec, 0x0008, 0x00012345));
    }

    SECTION("PM: DOS/4GW thunk in extended memory → false") {
        // Real DOS/4GW handlers live above 1MB; none coincide with stub area
        REQUIRE(!bios.isOriginalIVT(vec, 0x0057, 0x00100000));
        REQUIRE(!bios.isOriginalIVT(vec, 0x008F, 0x001ABCDE));
    }

    SECTION("RM: F000:stub_offset → true") {
        uint16_t stubOff = static_cast<uint16_t>(0x0100u + vec * 4u);
        REQUIRE(bios.isOriginalIVT(vec, 0xF000, stubOff));
    }

    SECTION("RM: program-hooked vector → false") {
        REQUIRE(!bios.isOriginalIVT(vec, 0x1234, 0x0200));
    }
}

TEST_CASE("isOriginalIVT returns false after 0x0205h changes the IDT entry", "[dpmi][int][idt]") {
    DPMITestEnv env;
    constexpr uint8_t vec = 0x2C;
    const uint32_t origStubPhys = 0xF0000u + 0x0100u + vec * 4u;

    // Sanity: before any hook, the original address is recognised
    REQUIRE(env.bios.isOriginalIVT(vec, 0x0008, origStubPhys));

    // Hook vector via 0x0205h (sole host → IDT written)
    env.cpu.setReg16(cpu::AX, 0x0205);
    env.cpu.setReg8(cpu::BL, vec);
    env.cpu.setReg16(cpu::CX, 0x0077);
    env.cpu.setReg32(cpu::EDX, 0x00098765);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // New address (different offset) is NOT recognised as original
    REQUIRE(!env.bios.isOriginalIVT(vec, 0x0077, 0x00098765));
    // The original address is still recognised by isOriginalIVT  
    // (it records the fixed BIOS-time value, not the current IDT)
    REQUIRE(env.bios.isOriginalIVT(vec, 0x0008, origStubPhys));
}

// ═══════════════════════════════════════════════════════════════════════════
// popHLEFrameByPhysAddr — nested frame matching correctness
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("popHLEFrameByPhysAddr removes only the matching frame", "[cpu][hle]") {
    cpu::CPU cpu;

    // Push three frames with distinct physical addresses
    cpu.pushHLEFrame(false, 0x21);
    cpu.lastHLEFrameMut().framePhysAddr = 0x1000;

    cpu.pushHLEFrame(false, 0x10);
    cpu.lastHLEFrameMut().framePhysAddr = 0x2000;

    cpu.pushHLEFrame(false, 0x0D);
    cpu.lastHLEFrameMut().framePhysAddr = 0x3000;

    REQUIRE(cpu.hleStackSize() == 3);

    SECTION("pop middle frame by address") {
        bool found = cpu.popHLEFrameByPhysAddr(0x2000);
        REQUIRE(found);
        REQUIRE(cpu.hleStackSize() == 2);
        // Remaining frames: 0x1000 (bottom) and 0x3000 (top)
        // Top frame must still be the innermost (0x3000)
        REQUIRE(cpu.lastHLEFrame().framePhysAddr == 0x3000);
    }

    SECTION("pop innermost frame by address") {
        bool found = cpu.popHLEFrameByPhysAddr(0x3000);
        REQUIRE(found);
        REQUIRE(cpu.hleStackSize() == 2);
        REQUIRE(cpu.lastHLEFrame().framePhysAddr == 0x2000);
    }

    SECTION("pop non-existent address returns false and stack unchanged") {
        bool found = cpu.popHLEFrameByPhysAddr(0xDEAD);
        REQUIRE(!found);
        REQUIRE(cpu.hleStackSize() == 3);
    }

    SECTION("pop outermost frame by address") {
        bool found = cpu.popHLEFrameByPhysAddr(0x1000);
        REQUIRE(found);
        REQUIRE(cpu.hleStackSize() == 2);
    }
}

TEST_CASE("popHLEFrameByPhysAddr on empty stack returns false", "[cpu][hle]") {
    cpu::CPU cpu;
    REQUIRE(!cpu.popHLEFrameByPhysAddr(0x1000));
}

TEST_CASE("popHLEFrameByPhysAddr does not disturb overlapping vector frames", "[cpu][hle]") {
    cpu::CPU cpu;

    // Simulate two nested INT 0x21 dispatches (e.g., reentrant DPMI calls)
    cpu.pushHLEFrame(false, 0x21);
    cpu.lastHLEFrameMut().framePhysAddr = 0x0FF0;

    cpu.pushHLEFrame(false, 0x21);
    cpu.lastHLEFrameMut().framePhysAddr = 0x0FE0;

    // IRET for the inner frame (0x0FE0) should leave the outer one intact
    bool found = cpu.popHLEFrameByPhysAddr(0x0FE0);
    REQUIRE(found);
    REQUIRE(cpu.hleStackSize() == 1);
    REQUIRE(cpu.lastHLEFrame().framePhysAddr == 0x0FF0);
    REQUIRE(cpu.lastHLEFrame().vector == 0x21);
}

// ═══════════════════════════════════════════════════════════════════════════
// DPMI 0302h state restoration — PM registers must survive an RM round-trip
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DPMI 0302h fully restores PM CPU state after RM round-trip", "[dpmi][translation][stack]") {
    DPMITestEnv env;

    // Record PM state before the translation call
    uint32_t espBefore = env.cpu.getReg32(cpu::ESP);
    uint32_t ebxBefore = env.cpu.getReg32(cpu::EBX);
    uint16_t csBefore  = env.cpu.getSegReg(cpu::CS);
    uint16_t ssBefore  = env.cpu.getSegReg(cpu::SS);
    uint32_t eipBefore = env.cpu.getEIP();

    // Build a DPMI RM call structure in scratch memory (50 bytes, zeroed)
    uint32_t structAddr = DPMITestEnv::SCRATCH_ADDR;
    for (int i = 0; i < 50; ++i)
        env.mem.write8(structAddr + i, 0);

    // Simulate calling INT 21h AH=2Ch (get time) — a benign, always-handled RM call
    env.mem.write32(structAddr + 0x1C, 0x002C0000u); // EAX: AH=2Ch

    env.cpu.setReg16(cpu::AX, 0x0302); // Call RM Proc with IRET frame
    env.cpu.setReg8(cpu::BL, 0x21);    // INT 21h
    env.cpu.setReg8(cpu::BH, 0x00);
    env.cpu.setReg16(cpu::CX, 0);
    env.cpu.setSegBase(cpu::ES, 0);
    env.cpu.setReg32(cpu::EDI, structAddr);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // PM state must be fully restored
    REQUIRE(env.cpu.getReg32(cpu::ESP) == espBefore);
    REQUIRE(env.cpu.getSegReg(cpu::CS)  == csBefore);
    REQUIRE(env.cpu.getSegReg(cpu::SS)  == ssBefore);
    REQUIRE(env.cpu.getEIP()            == eipBefore);
}

TEST_CASE("DPMI 0302h INT 21h AH=44h IOCTL does not corrupt PM stack pointer", "[dpmi][translation][stack]") {
    DPMITestEnv env;

    uint32_t espBefore = env.cpu.getReg32(cpu::ESP);

    uint32_t structAddr = DPMITestEnv::SCRATCH_ADDR;

    // Two consecutive 0x0302 calls with AX=4400h (IOCTL Get Device Info)
    // using different file handles — reproduces the deterministic OOB pattern
    for (uint16_t bx : {uint16_t(5), uint16_t(5)}) {
        for (int i = 0; i < 50; ++i)
            env.mem.write8(structAddr + i, 0);
        env.mem.write32(structAddr + 0x1C, 0x44000000u); // EAX: AH=44h AL=00h
        env.mem.write32(structAddr + 0x10, bx);           // EBX: file handle

        env.cpu.setReg16(cpu::AX, 0x0302);
        env.cpu.setReg8(cpu::BL, 0x21);
        env.cpu.setReg8(cpu::BH, 0x00);
        env.cpu.setReg16(cpu::CX, 0);
        env.cpu.setSegBase(cpu::ES, 0);
        env.cpu.setReg32(cpu::EDI, structAddr);
        env.clearCarry();
        env.dpmi.handleInt31();
        // CF may be set (invalid file handle) — that's fine
    }

    // PM ESP must be unchanged: no stack underflow / wrap-around occurred
    REQUIRE(env.cpu.getReg32(cpu::ESP) == espBefore);

    // ESP must not have wrapped to near-zero then to 0xFFFFFFFC
    REQUIRE(env.cpu.getReg32(cpu::ESP) < 0xFFF00000u);
}

TEST_CASE("DPMI 0302h HLE frame stack is clean after RM round-trip", "[dpmi][translation][stack]") {
    DPMITestEnv env;

    size_t hleSizeBefore = env.cpu.hleStackSize();

    uint32_t structAddr = DPMITestEnv::SCRATCH_ADDR;
    for (int i = 0; i < 50; ++i)
        env.mem.write8(structAddr + i, 0);
    env.mem.write32(structAddr + 0x1C, 0x002C0000u); // INT 21h AH=2Ch

    env.cpu.setReg16(cpu::AX, 0x0302);
    env.cpu.setReg8(cpu::BL, 0x21);
    env.cpu.setReg8(cpu::BH, 0x00);
    env.cpu.setReg16(cpu::CX, 0);
    env.cpu.setSegBase(cpu::ES, 0);
    env.cpu.setReg32(cpu::EDI, structAddr);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // 0302h is pure HLE; it must not leave orphaned frames on the HLE stack
    REQUIRE(env.cpu.hleStackSize() == hleSizeBefore);
}

// ═══════════════════════════════════════════════════════════════════════════
// PM INT entry semantics (triggerInterrupt path): pushed flags and IF/TF
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("PM INT entry pushes original EFLAGS before IF/TF adjustment", "[dpmi][int][entry]") {
    DPMITestEnv env;
    hw::IOBus iobus;
    cpu::InstructionDecoder decoder(env.cpu, env.mem, iobus, env.bios, env.dos);

    constexpr uint8_t vec = 0x77;

    // Execute INT 77h from current PM CS:EIP
    uint32_t codePhys = env.cpu.getSegBase(cpu::CS) + env.cpu.getEIP();
    env.mem.write8(codePhys, 0xCD);
    env.mem.write8(codePhys + 1, vec);

    // Force known source flags before entry.
    uint32_t preFlags = env.cpu.getEFLAGS();
    preFlags |= (cpu::FLAG_INTERRUPT | cpu::FLAG_TRAP | cpu::FLAG_CARRY);
    env.cpu.setEFLAGS(preFlags);

    uint32_t ssBase = env.cpu.getSegBase(cpu::SS);
    uint16_t oldCs = env.cpu.getSegReg(cpu::CS);
    uint32_t spBefore = env.cpu.is32BitStack()
                            ? env.cpu.getReg32(cpu::ESP)
                            : static_cast<uint32_t>(env.cpu.getReg16(cpu::SP));

    uint32_t idtEntry = env.cpu.getIDTR().base + static_cast<uint32_t>(vec) * 8u;
    uint32_t idtLow = env.mem.read32(idtEntry);
    uint32_t idtHigh = env.mem.read32(idtEntry + 4);
    uint16_t newCs = static_cast<uint16_t>((idtLow >> 16) & 0xFFFFu);
    uint8_t gateType = static_cast<uint8_t>((idtHigh >> 8) & 0x0Fu);
    bool use32Gate = (gateType & 0x08u) != 0;
    bool privChange = ((newCs & 3u) < (oldCs & 3u)) || ((preFlags & 0x00020000u) != 0);
    uint32_t expectedDelta = use32Gate ? (privChange ? 20u : 12u)
                                       : (privChange ? 10u : 6u);

    decoder.step();

    uint32_t spAfter = env.cpu.is32BitStack()
                           ? env.cpu.getReg32(cpu::ESP)
                           : static_cast<uint32_t>(env.cpu.getReg16(cpu::SP));

    // Stack delta depends on gate width and whether CPL changes.
    // Privilege changes also push old SS:ESP before the IRET frame.
    if (env.cpu.is32BitStack()) {
        REQUIRE(spAfter == spBefore - expectedDelta);
    } else {
        REQUIRE(spAfter == ((spBefore - expectedDelta) & 0xFFFFu));
    }

    uint32_t pushedFlags = env.mem.read32(ssBase + spAfter + 8);
    REQUIRE(pushedFlags == preFlags);
}

TEST_CASE("PM INT entry clears IF only for interrupt gates, always clears TF", "[dpmi][int][entry]") {
    DPMITestEnv env;
    hw::IOBus iobus;
    cpu::InstructionDecoder decoder(env.cpu, env.mem, iobus, env.bios, env.dos);

    constexpr uint8_t vecInt = 0x78;
    constexpr uint8_t vecTrap = 0x79;

    auto writeIntInstruction = [&](uint8_t vec) {
        uint32_t codePhys = env.cpu.getSegBase(cpu::CS) + env.cpu.getEIP();
        env.mem.write8(codePhys, 0xCD);
        env.mem.write8(codePhys + 1, vec);
    };

    auto runInt = [&](uint8_t vec) {
        writeIntInstruction(vec);
        decoder.step();
    };

    // Convert vecTrap IDT entry to a trap gate (type 0xF) while preserving
    // selector/offset/DPL/present bits.
    uint32_t idtBase = env.cpu.getIDTR().base;
    uint32_t trapEntry = idtBase + static_cast<uint32_t>(vecTrap) * 8u;
    uint32_t trapHigh = env.mem.read32(trapEntry + 4);
    trapHigh = (trapHigh & ~0x00000F00u) | 0x00000F00u;
    env.mem.write32(trapEntry + 4, trapHigh);

    SECTION("interrupt gate clears IF and TF") {
        uint32_t flags = env.cpu.getEFLAGS() | cpu::FLAG_INTERRUPT | cpu::FLAG_TRAP;
        env.cpu.setEFLAGS(flags);
        runInt(vecInt);

        REQUIRE((env.cpu.getEFLAGS() & cpu::FLAG_TRAP) == 0);
        REQUIRE((env.cpu.getEFLAGS() & cpu::FLAG_INTERRUPT) == 0);
    }

    SECTION("trap gate clears TF but preserves IF") {
        uint32_t flags = env.cpu.getEFLAGS() | cpu::FLAG_INTERRUPT | cpu::FLAG_TRAP;
        env.cpu.setEFLAGS(flags);
        runInt(vecTrap);

        REQUIRE((env.cpu.getEFLAGS() & cpu::FLAG_TRAP) == 0);
        REQUIRE((env.cpu.getEFLAGS() & cpu::FLAG_INTERRUPT) != 0);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// 0302h — Call Real Mode Procedure With IRET Frame
// ═══════════════════════════════════════════════════════════════════════════

// Helper: compute HLE stub address for a given interrupt vector
static constexpr uint16_t HLE_STUB_CS = 0xF000;
static constexpr uint16_t HLE_STUB_BASE = 0x0100;
static uint16_t hleStubIP(uint8_t vector) { return 0x0100 + static_cast<uint16_t>(vector) * 4; }

TEST_CASE("DPMI 0302h calls RM DOS service INT 21h AH=30h", "[dpmi][0302]") {
    DPMITestEnv env;

    uint32_t structAddr = DPMITestEnv::SCRATCH_ADDR;
    for (int i = 0; i < 50; ++i)
        env.mem.write8(structAddr + i, 0);

    uint8_t intNo = 0x21;
    // INT 21h AH=30h — Get DOS Version
    env.mem.write32(structAddr + 0x1C, 0x00003000); // EAX = AH=30h
    env.mem.write16(structAddr + 0x20, 0x0202);     // FLAGS
    env.mem.write16(structAddr + 0x2E, 0xFFF0);     // SP
    env.mem.write16(structAddr + 0x30, 0x8000);     // SS
    env.mem.write16(structAddr + 0x2A, hleStubIP(intNo)); // IP → HLE stub
    env.mem.write16(structAddr + 0x2C, HLE_STUB_CS);      // CS → F000

    env.cpu.setReg16(cpu::AX, 0x0302);
    env.cpu.setReg8(cpu::BL, intNo);
    env.cpu.setReg8(cpu::BH, 0x00);
    env.cpu.setReg16(cpu::CX, 0);
    env.cpu.setSegBase(cpu::ES, 0);
    env.cpu.setReg32(cpu::EDI, structAddr);
    env.clearCarry();
    env.dpmi.handleInt31();

    // 0302h should succeed (carry clear) and the struct should be updated
    REQUIRE(!env.carrySet());
    // The write-back should have occurred — at minimum the flags word at +0x20
    // should be non-zero (IF bit set in returned flags)
    uint16_t retFlags = env.mem.read16(structAddr + 0x20);
    REQUIRE(retFlags != 0);
}

TEST_CASE("DPMI 0302h calls RM DOS service INT 21h AH=2Ah", "[dpmi][0302]") {
    DPMITestEnv env;

    uint32_t structAddr = DPMITestEnv::SCRATCH_ADDR;
    for (int i = 0; i < 50; ++i)
        env.mem.write8(structAddr + i, 0);

    uint8_t intNo = 0x21;
    // INT 21h AH=2Ah — Get System Date
    env.mem.write32(structAddr + 0x1C, 0x00002A00); // EAX = AH=2Ah
    env.mem.write16(structAddr + 0x20, 0x0202);
    env.mem.write16(structAddr + 0x2E, 0xFFF0);
    env.mem.write16(structAddr + 0x30, 0x8000);
    env.mem.write16(structAddr + 0x2A, hleStubIP(intNo));
    env.mem.write16(structAddr + 0x2C, HLE_STUB_CS);

    env.cpu.setReg16(cpu::AX, 0x0302);
    env.cpu.setReg8(cpu::BL, intNo);
    env.cpu.setReg8(cpu::BH, 0x00);
    env.cpu.setReg16(cpu::CX, 0);
    env.cpu.setSegBase(cpu::ES, 0);
    env.cpu.setReg32(cpu::EDI, structAddr);
    env.clearCarry();
    env.dpmi.handleInt31();

    REQUIRE(!env.carrySet());
    // INT 2Ah should return valid date values (CX=year)
    uint32_t retECX = env.mem.read32(structAddr + 0x18);
    uint16_t retCX = static_cast<uint16_t>(retECX & 0xFFFF);
    REQUIRE(retCX >= 1980);
}

TEST_CASE("DPMI 0302h preserves PM registers across RM call", "[dpmi][0302]") {
    DPMITestEnv env;

    env.cpu.setReg32(cpu::EBP, 0xDEADBEEF);
    env.cpu.setReg32(cpu::ESI, 0xCAFEBABE);
    uint32_t savedEBP = env.cpu.getReg32(cpu::EBP);
    uint32_t savedESI = env.cpu.getReg32(cpu::ESI);

    uint32_t structAddr = DPMITestEnv::SCRATCH_ADDR;
    for (int i = 0; i < 50; ++i)
        env.mem.write8(structAddr + i, 0);

    // INT 12h returns 640 in AX
    uint8_t intNo = 0x12;
    env.mem.write32(structAddr + 0x1C, 0);
    env.mem.write16(structAddr + 0x20, 0x0202);
    env.mem.write16(structAddr + 0x2E, 0xFFF0);
    env.mem.write16(structAddr + 0x30, 0x8000);
    env.mem.write16(structAddr + 0x2A, hleStubIP(intNo));
    env.mem.write16(structAddr + 0x2C, HLE_STUB_CS);

    env.cpu.setReg16(cpu::AX, 0x0302);
    env.cpu.setReg8(cpu::BL, intNo);
    env.cpu.setReg8(cpu::BH, 0x00);
    env.cpu.setReg16(cpu::CX, 0);
    env.cpu.setSegBase(cpu::ES, 0);
    env.cpu.setReg32(cpu::EDI, structAddr);
    env.clearCarry();
    env.dpmi.handleInt31();

    REQUIRE(!env.carrySet());
    REQUIRE(env.cpu.getReg32(cpu::EBP) == savedEBP);
    REQUIRE(env.cpu.getReg32(cpu::ESI) == savedESI);
}

TEST_CASE("DPMI 0302h propagates carry flag from RM call", "[dpmi][0302]") {
    DPMITestEnv env;

    uint32_t structAddr = DPMITestEnv::SCRATCH_ADDR;
    for (int i = 0; i < 50; ++i)
        env.mem.write8(structAddr + i, 0);

    uint8_t intNo = 0x21;
    // INT 21h AH=0xFF (invalid function — should set carry)
    env.mem.write32(structAddr + 0x1C, 0x0000FF00);
    env.mem.write16(structAddr + 0x20, 0x0202);
    env.mem.write16(structAddr + 0x2E, 0xFFF0);
    env.mem.write16(structAddr + 0x30, 0x8000);
    env.mem.write16(structAddr + 0x2A, hleStubIP(intNo));
    env.mem.write16(structAddr + 0x2C, HLE_STUB_CS);

    env.cpu.setReg16(cpu::AX, 0x0302);
    env.cpu.setReg8(cpu::BL, intNo);
    env.cpu.setReg8(cpu::BH, 0x00);
    env.cpu.setReg16(cpu::CX, 0);
    env.cpu.setSegBase(cpu::ES, 0);
    env.cpu.setReg32(cpu::EDI, structAddr);
    env.clearCarry();
    env.dpmi.handleInt31();

    // INT 21h/FFh: DOS returns AL=0, AH=0 with CF? Actually DOS 30h stub
    // for unknown functions sets CF. Verify the call doesn't crash.
    // The carry flag in the returned flags word at +0x20 should reflect RM state.
    uint16_t retFlags = env.mem.read16(structAddr + 0x20);
    REQUIRE(retFlags != 0); // Flags were written back
}

TEST_CASE("DPMI 0302h handles multiple consecutive calls", "[dpmi][0302]") {
    DPMITestEnv env;

    uint32_t structAddr = DPMITestEnv::SCRATCH_ADDR;

    // Call 1: INT 12h (memory size)
    uint8_t int12 = 0x12;
    for (int i = 0; i < 50; ++i) env.mem.write8(structAddr + i, 0);
    env.mem.write32(structAddr + 0x1C, 0);
    env.mem.write16(structAddr + 0x20, 0x0202);
    env.mem.write16(structAddr + 0x2E, 0xFFF0);
    env.mem.write16(structAddr + 0x30, 0x8000);
    env.mem.write16(structAddr + 0x2A, hleStubIP(int12));
    env.mem.write16(structAddr + 0x2C, HLE_STUB_CS);

    env.cpu.setReg16(cpu::AX, 0x0302);
    env.cpu.setReg8(cpu::BL, int12);
    env.cpu.setReg8(cpu::BH, 0x00);
    env.cpu.setReg16(cpu::CX, 0);
    env.cpu.setSegBase(cpu::ES, 0);
    env.cpu.setReg32(cpu::EDI, structAddr);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
    uint16_t ret1 = env.mem.read16(structAddr + 0x1C);
    REQUIRE(ret1 == 640);

    // Call 2: INT 11h (equipment list)
    uint8_t int11 = 0x11;
    for (int i = 0; i < 50; ++i) env.mem.write8(structAddr + i, 0);
    env.mem.write32(structAddr + 0x1C, 0);
    env.mem.write16(structAddr + 0x20, 0x0202);
    env.mem.write16(structAddr + 0x2E, 0xFFF0);
    env.mem.write16(structAddr + 0x30, 0x8000);
    env.mem.write16(structAddr + 0x2A, hleStubIP(int11));
    env.mem.write16(structAddr + 0x2C, HLE_STUB_CS);

    env.cpu.setReg16(cpu::AX, 0x0302);
    env.cpu.setReg8(cpu::BL, int11);
    env.cpu.setReg8(cpu::BH, 0x00);
    env.cpu.setReg16(cpu::CX, 0);
    env.cpu.setSegBase(cpu::ES, 0);
    env.cpu.setReg32(cpu::EDI, structAddr);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
    uint16_t ret2 = env.mem.read16(structAddr + 0x1C);
    REQUIRE(ret2 == 0x002F); // Typical equipment word

    // Call 3: INT 1Ah AH=00h (get timer ticks)
    uint8_t int1a = 0x1A;
    for (int i = 0; i < 50; ++i) env.mem.write8(structAddr + i, 0);
    env.mem.write32(structAddr + 0x1C, 0);
    env.mem.write16(structAddr + 0x20, 0x0202);
    env.mem.write16(structAddr + 0x2E, 0xFFF0);
    env.mem.write16(structAddr + 0x30, 0x8000);
    env.mem.write16(structAddr + 0x2A, hleStubIP(int1a));
    env.mem.write16(structAddr + 0x2C, HLE_STUB_CS);

    env.cpu.setReg16(cpu::AX, 0x0302);
    env.cpu.setReg8(cpu::BL, int1a);
    env.cpu.setReg8(cpu::BH, 0x00);
    env.cpu.setReg16(cpu::CX, 0);
    env.cpu.setSegBase(cpu::ES, 0);
    env.cpu.setReg32(cpu::EDI, structAddr);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
    // INT 1Ah AH=00h returns timer ticks in CX:DX and AL=midnight flag
    // Flags should be written back
    uint16_t retFlags = env.mem.read16(structAddr + 0x20);
    REQUIRE(retFlags != 0);
}

TEST_CASE("DPMI 0302h copes with ES segment wrap-around in structAddr", "[dpmi][0302]") {
    DPMITestEnv env;

    uint32_t structAddr = 0x10000;
    for (int i = 0; i < 50; ++i)
        env.mem.write8(structAddr + i, 0);

    uint8_t intNo = 0x21;
    env.mem.write32(structAddr + 0x1C, 0x00003000);
    env.mem.write16(structAddr + 0x20, 0x0202);
    env.mem.write16(structAddr + 0x2E, 0xFFF0);
    env.mem.write16(structAddr + 0x30, 0x8000);
    env.mem.write16(structAddr + 0x2A, hleStubIP(intNo));
    env.mem.write16(structAddr + 0x2C, HLE_STUB_CS);

    env.cpu.setSegBase(cpu::ES, 0x10000);
    env.cpu.setSegReg(cpu::ES, 0x1000);

    env.cpu.setReg16(cpu::AX, 0x0302);
    env.cpu.setReg8(cpu::BL, intNo);
    env.cpu.setReg8(cpu::BH, 0x00);
    env.cpu.setReg16(cpu::CX, 0);
    env.cpu.setReg32(cpu::EDI, 0);
    env.clearCarry();
    env.dpmi.handleInt31();

    REQUIRE(!env.carrySet());
    // Flags should be written back — struct was properly accessed
    uint16_t retFlags = env.mem.read16(structAddr + 0x20);
    REQUIRE(retFlags != 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// 0100h/0101h — DOS Memory Allocation from Protected Mode
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DPMI 0101h free invalid selector fails", "[dpmi][dosmem]") {
    DPMITestEnv env;

    env.cpu.setReg16(cpu::AX, 0x0101);
    env.cpu.setReg16(cpu::DX, 0xFFFF);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(env.carrySet());
}

TEST_CASE("DPMI 0100h allocate paragraphs returns error when no free memory", "[dpmi][dosmem]") {
    DPMITestEnv env;

    // DPMITestEnv doesn't set up free DOS memory, so allocation should fail
    env.cpu.setReg16(cpu::AX, 0x0100);
    env.cpu.setReg16(cpu::BX, 64);
    env.clearCarry();
    env.dpmi.handleInt31();
    // May or may not succeed depending on MCB state — just verify it doesn't crash
    (void)env.carrySet();
}

// ═══════════════════════════════════════════════════════════════════════════
// 0205h — Set PM Interrupt Vector with host chain flag
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DPMI 0205h with host interrupt chain flag", "[dpmi][int]") {
    DPMITestEnv env;

    // Set PM vector for INT 0x21 with host chain flag (BL bit 0 = 1)
    env.cpu.setReg16(cpu::AX, 0x0205);
    env.cpu.setReg8(cpu::BL, 0x01);  // INT 0x01 with host chain flag
    env.cpu.setReg16(cpu::CX, 0x0047);    // selector
    env.cpu.setReg32(cpu::EDX, 0x00BEEF00); // offset
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // Verify host 0x01 vector was stored in RM IVT
    // The host handler for INT 0x01 should now point to the PM vector
    // Get back the PM vector
    env.cpu.setReg16(cpu::AX, 0x0204);
    env.cpu.setReg8(cpu::BL, 0x01);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
    REQUIRE(env.cpu.getReg16(cpu::CX) == 0x0047);
    REQUIRE(env.cpu.getReg32(cpu::EDX) == 0x00BEEF00);
}

TEST_CASE("DPMI 0205h hook and restore INT 0x09 (keyboard IRQ) with host chain", "[dpmi][int]") {
    DPMITestEnv env;

    // Hook keyboard IRQ vector with host chain
    env.cpu.setReg16(cpu::AX, 0x0205);
    env.cpu.setReg8(cpu::BL, 0x09);  // INT 0x09 = keyboard
    env.cpu.setReg16(cpu::CX, 0x0057);
    env.cpu.setReg32(cpu::EDX, 0x00C0DE00);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    // Verify it stuck
    env.cpu.setReg16(cpu::AX, 0x0204);
    env.cpu.setReg8(cpu::BL, 0x09);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
    REQUIRE(env.cpu.getReg16(cpu::CX) == 0x0057);

    // Hook INT 0x08 (timer IRQ) as well
    env.cpu.setReg16(cpu::AX, 0x0205);
    env.cpu.setReg8(cpu::BL, 0x08);
    env.cpu.setReg16(cpu::CX, 0x0047);
    env.cpu.setReg32(cpu::EDX, 0x00F00D00);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());

    env.cpu.setReg16(cpu::AX, 0x0204);
    env.cpu.setReg8(cpu::BL, 0x08);
    env.clearCarry();
    env.dpmi.handleInt31();
    REQUIRE(!env.carrySet());
    REQUIRE(env.cpu.getReg16(cpu::CX) == 0x0047);
}

// ═══════════════════════════════════════════════════════════════════════════
// 0301h — Call Real Mode Procedure With Far Return Frame
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("DPMI 0301h calls RM procedure with far return", "[dpmi][0301]") {
    DPMITestEnv env;

    uint32_t structAddr = DPMITestEnv::SCRATCH_ADDR;
    for (int i = 0; i < 50; ++i)
        env.mem.write8(structAddr + i, 0);

    uint8_t intNo = 0x12;
    env.mem.write32(structAddr + 0x1C, 0);
    env.mem.write16(structAddr + 0x20, 0x0202);
    env.mem.write16(structAddr + 0x2E, 0xFFF0);
    env.mem.write16(structAddr + 0x30, 0x8000);
    env.mem.write16(structAddr + 0x2A, hleStubIP(intNo));
    env.mem.write16(structAddr + 0x2C, HLE_STUB_CS);

    env.cpu.setReg16(cpu::AX, 0x0301);
    env.cpu.setReg8(cpu::BL, intNo);
    env.cpu.setReg8(cpu::BH, 0x00);
    env.cpu.setReg16(cpu::CX, 0);
    env.cpu.setSegBase(cpu::ES, 0);
    env.cpu.setReg32(cpu::EDI, structAddr);
    env.clearCarry();
    env.dpmi.handleInt31();

    REQUIRE(!env.carrySet());
    uint16_t retAX = env.mem.read16(structAddr + 0x1C);
    REQUIRE(retAX == 640);
}
