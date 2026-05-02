#include "test_framework.hpp"
#include "cpu/CPU.hpp"
#include "memory/MemoryBus.hpp"

using namespace fador;

TEST_CASE("CPU Reset Logic", "[CPU]") {
    cpu::CPU cpu;

    SECTION("Initial reset state") {
        REQUIRE(cpu.getSegReg(cpu::SegRegIndex::CS) == 0xF000);
        REQUIRE(cpu.getSegReg(cpu::SegRegIndex::DS) == 0);
        REQUIRE(cpu.getSegReg(cpu::SegRegIndex::ES) == 0);
        REQUIRE(cpu.getSegReg(cpu::SegRegIndex::SS) == 0);
        REQUIRE(cpu.getSegReg(cpu::SegRegIndex::FS) == 0);
        REQUIRE(cpu.getSegReg(cpu::SegRegIndex::GS) == 0);

        REQUIRE(cpu.getEIP() == 0xFFF0);
        REQUIRE(cpu.getEFLAGS() == 0x00000002);

        REQUIRE(cpu.getReg32(cpu::Reg32Index::EAX) == 0);
        REQUIRE(cpu.getReg32(cpu::Reg32Index::ECX) == 0);
        REQUIRE(cpu.getReg32(cpu::Reg32Index::EDX) == 0);
        REQUIRE(cpu.getReg32(cpu::Reg32Index::EBX) == 0);
        REQUIRE(cpu.getReg32(cpu::Reg32Index::ESP) == 0);
        REQUIRE(cpu.getReg32(cpu::Reg32Index::EBP) == 0);
        REQUIRE(cpu.getReg32(cpu::Reg32Index::ESI) == 0);
        REQUIRE(cpu.getReg32(cpu::Reg32Index::EDI) == 0);

        REQUIRE(cpu.is32BitCode() == false);
        REQUIRE(cpu.is32BitStack() == false);

        REQUIRE(cpu.getCR(0) == 0);
        REQUIRE(cpu.getDR(0) == 0);
    }
}

TEST_CASE("CPU Extended Stack Operations", "[CPU]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    cpu.setMemoryBus(&mem);

    SECTION("16-bit push and pop") {
        cpu.setReg16(cpu::Reg16Index::SP, 0x1000);
        cpu.setSegBase(cpu::SegRegIndex::SS, 0x0000);
        cpu.setIs32BitStack(false);

        cpu.push16(0x1234);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SP) == 0x0FFE);
        REQUIRE(mem.read16(0x0FFE) == 0x1234);

        uint16_t val = cpu.pop16();
        REQUIRE(val == 0x1234);
        REQUIRE(cpu.getReg16(cpu::Reg16Index::SP) == 0x1000);
    }

    SECTION("32-bit push and pop") {
        cpu.setReg32(cpu::Reg32Index::ESP, 0x2000);
        cpu.setSegBase(cpu::SegRegIndex::SS, 0x0000);
        cpu.setIs32BitStack(true);

        cpu.push32(0xDEADBEEF);
        REQUIRE(cpu.getReg32(cpu::Reg32Index::ESP) == 0x1FFC);
        REQUIRE(mem.read32(0x1FFC) == 0xDEADBEEF);

        uint32_t val = cpu.pop32();
        REQUIRE(val == 0xDEADBEEF);
        REQUIRE(cpu.getReg32(cpu::Reg32Index::ESP) == 0x2000);
    }
}

TEST_CASE("CPU Registers Read and Write", "[CPU]") {
    cpu::CPU cpu;

    SECTION("Control and Debug Registers") {
        cpu.setCR(0, 0x12345678);
        REQUIRE(cpu.getCR(0) == 0x12345678);

        cpu.setCR(4, 0x00000010);
        REQUIRE(cpu.getCR(4) == 0x00000010);

        cpu.setDR(0, 0xDEADBEEF);
        REQUIRE(cpu.getDR(0) == 0xDEADBEEF);

        cpu.setDR(7, 0x00000400);
        REQUIRE(cpu.getDR(7) == 0x00000400);
    }

    SECTION("Descriptor Registers") {
        cpu::DescriptorRegister gdtr = {0x0123, 0x10203040};
        cpu.setGDTR(gdtr);
        REQUIRE(cpu.getGDTR().limit == 0x0123);
        REQUIRE(cpu.getGDTR().base == 0x10203040);

        cpu::DescriptorRegister idtr = {0x07FF, 0x50607080};
        cpu.setIDTR(idtr);
        REQUIRE(cpu.getIDTR().limit == 0x07FF);
        REQUIRE(cpu.getIDTR().base == 0x50607080);

        cpu::DescriptorRegister ldtr = {0x00FF, 0x90A0B0C0};
        cpu.setLDTR(ldtr);
        REQUIRE(cpu.getLDTR().limit == 0x00FF);
        REQUIRE(cpu.getLDTR().base == 0x90A0B0C0);

        cpu::DescriptorRegister tr = {0x001F, 0xD0E0F000};
        cpu.setTR(tr);
        REQUIRE(cpu.getTR().limit == 0x001F);
        REQUIRE(cpu.getTR().base == 0xD0E0F000);

        cpu.setLDTRSelector(0x1234);
        REQUIRE(cpu.getLDTRSelector() == 0x1234);

        cpu.setTRSelector(0x5678);
        REQUIRE(cpu.getTRSelector() == 0x5678);
    }
}

TEST_CASE("CPU HLE Stack Frames", "[CPU]") {
    cpu::CPU cpu;

    SECTION("Basic Frame Operations") {
        REQUIRE(cpu.hleStackSize() == 0);
        REQUIRE(cpu.isLastInt32() == false);

        cpu.pushHLEFrame(true, 0x21);
        REQUIRE(cpu.hleStackSize() == 1);
        REQUIRE(cpu.isLastInt32() == true);
        REQUIRE(cpu.lastHLEFrame().vector == 0x21);

        cpu.pushHLEFrame(false, 0x10);
        REQUIRE(cpu.hleStackSize() == 2);
        REQUIRE(cpu.isLastInt32() == false);
        REQUIRE(cpu.lastHLEFrame().vector == 0x10);

        cpu.popHLEFrame();
        REQUIRE(cpu.hleStackSize() == 1);
        REQUIRE(cpu.isLastInt32() == true);
        REQUIRE(cpu.lastHLEFrame().vector == 0x21);
    }

    SECTION("Address-based Lookups") {
        cpu.pushHLEFrame(true, 0x21);
        cpu.lastHLEFrameMut().framePhysAddr = 0x1000;

        cpu.pushHLEFrame(false, 0x10);
        cpu.lastHLEFrameMut().framePhysAddr = 0x2000;

        auto frame = cpu.popAndGetHLEFrameByPhysAddr(0x1000);
        REQUIRE(frame.vector == 0x21);
        REQUIRE(frame.framePhysAddr == 0x1000);
        REQUIRE(cpu.hleStackSize() == 1);
        REQUIRE(cpu.lastHLEFrame().vector == 0x10);

        bool popped = cpu.popHLEFrameByPhysAddr(0x2000);
        REQUIRE(popped == true);
        REQUIRE(cpu.hleStackSize() == 0);
    }

    SECTION("Vector-based Lookups") {
        cpu.pushHLEFrame(true, 0x21);
        cpu.pushHLEFrame(false, 0x10);
        cpu.pushHLEFrame(false, 0x13);

        auto peeked = cpu.peekHLEFrameForVector(0x10);
        REQUIRE(peeked.vector == 0x10);
        REQUIRE(cpu.hleStackSize() == 3);

        auto popped = cpu.popHLEFrameForVector(0x10);
        REQUIRE(popped.vector == 0x10);
        REQUIRE(cpu.hleStackSize() == 1);
        REQUIRE(cpu.lastHLEFrame().vector == 0x21);
    }
}
