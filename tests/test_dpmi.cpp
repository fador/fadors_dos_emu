// test_dpmi.cpp – Comprehensive tests for the DPMI 0.9 host implementation.
// Covers all DOS/4GW-relevant DPMI functions: descriptor management, interrupt
// vectors, translation services, memory, page locking, virtual interrupts,
// and raw mode switches.

#include "test_framework.hpp"
#include "cpu/CPU.hpp"
#include "memory/MemoryBus.hpp"
#include "memory/himem/HIMEM.hpp"
#include "hw/DPMI.hpp"
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
    hw::DOS dos;
    hw::BIOS bios;
    memory::HIMEM himem;
    hw::DPMI dpmi;

    // A scratch linear address for passing buffers via ES:DI
    static constexpr uint32_t SCRATCH_ADDR = 0x20000;

    DPMITestEnv()
        : dos(cpu, mem), bios(cpu, mem, kbd, pit),
          dpmi(cpu, mem)
    {
        bios.initialize();
        dos.initialize();

        dpmi.setDOS(&dos);
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
    hw::BIOS bios(cpu, mem, kbd, pit);
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
    hw::BIOS bios(cpu, mem, kbd, pit);
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
