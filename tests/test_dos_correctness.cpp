#include "test_framework.hpp"
#include <algorithm>
#include <fstream>
#include <filesystem>
#include <vector>
#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "memory/MemoryBus.hpp"
#include "hw/IOBus.hpp"
#include "hw/PIC8259.hpp"
#include "hw/BIOS.hpp"
#include "hw/DOS.hpp"
#include "hw/ProgramLoader.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIT8254.hpp"

using namespace fador;

namespace {

void setSegment(cpu::CPU &cpu, cpu::SegRegIndex reg, uint16_t seg) {
    cpu.setSegReg(reg, seg);
    cpu.setSegBase(reg, static_cast<uint32_t>(seg) << 4);
}

void writeCString(memory::MemoryBus &mem, uint32_t physAddr, const std::string &text) {
    for (size_t i = 0; i < text.size(); ++i)
        mem.write8(physAddr + static_cast<uint32_t>(i), static_cast<uint8_t>(text[i]));
    mem.write8(physAddr + static_cast<uint32_t>(text.size()), 0);
}

std::string readFCBName(memory::MemoryBus &mem, uint32_t fcbAddr) {
    std::string name;
    for (int i = 0; i < 8; ++i) {
        name.push_back(static_cast<char>(mem.read8(fcbAddr + 1 + i)));
    }
    return name;
}

std::string readFCBExt(memory::MemoryBus &mem, uint32_t fcbAddr) {
    std::string ext;
    for (int i = 0; i < 3; ++i) {
        ext.push_back(static_cast<char>(mem.read8(fcbAddr + 9 + i)));
    }
    return ext;
}

} // namespace

TEST_CASE("DOS Correctness: PSP FCB Parsing for command line arguments", "[dos][psp][fcb]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::DOS dos(cpu, mem);
    dos.initialize();
    hw::ProgramLoader loader(cpu, mem);

    // Create a dummy small COM file
    const std::string comPath = "test_cmd.com";
    {
        std::ofstream f(comPath, std::ios::binary);
        uint8_t retOp = 0xC3; // RET
        f.write(reinterpret_cast<const char*>(&retOp), 1);
    }

    constexpr uint16_t seg = 0x1000;
    const uint32_t pspPhys = static_cast<uint32_t>(seg) << 4;

    // 1. Empty arguments
    REQUIRE(loader.loadCOM(comPath, seg, dos, ""));
    REQUIRE(mem.read8(pspPhys + 0x5C) == 0x00); // Default drive
    REQUIRE(readFCBName(mem, pspPhys + 0x5C) == "        ");
    REQUIRE(readFCBExt(mem, pspPhys + 0x5C) == "   ");

    // 2. Single filename with drive prefix
    REQUIRE(loader.loadCOM(comPath, seg, dos, "C:INPUT.DAT"));
    REQUIRE(mem.read8(pspPhys + 0x5C) == 0x03); // C: = drive 3
    REQUIRE(readFCBName(mem, pspPhys + 0x5C) == "INPUT   ");
    REQUIRE(readFCBExt(mem, pspPhys + 0x5C) == "DAT");
    REQUIRE(mem.read8(pspPhys + 0x6C) == 0x00); // Second FCB empty
    REQUIRE(readFCBName(mem, pspPhys + 0x6C) == "        ");

    // 3. Two filenames with different drives
    REQUIRE(loader.loadCOM(comPath, seg, dos, "A:FILE1.TXT B:FILE2.BIN"));
    REQUIRE(mem.read8(pspPhys + 0x5C) == 0x01); // A: = drive 1
    REQUIRE(readFCBName(mem, pspPhys + 0x5C) == "FILE1   ");
    REQUIRE(readFCBExt(mem, pspPhys + 0x5C) == "TXT");
    REQUIRE(mem.read8(pspPhys + 0x6C) == 0x02); // B: = drive 2
    REQUIRE(readFCBName(mem, pspPhys + 0x6C) == "FILE2   ");
    REQUIRE(readFCBExt(mem, pspPhys + 0x6C) == "BIN");

    // 4. Wildcards
    REQUIRE(loader.loadCOM(comPath, seg, dos, "*.EXE"));
    REQUIRE(readFCBName(mem, pspPhys + 0x5C) == "????????");
    REQUIRE(readFCBExt(mem, pspPhys + 0x5C) == "EXE");

    std::filesystem::remove(comPath);
}

TEST_CASE("DOS Correctness: Startup Register Initialization for COM and EXE", "[dos][loader][registers]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::DOS dos(cpu, mem);
    dos.initialize();
    hw::ProgramLoader loader(cpu, mem);

    // 1. COM program startup registers
    const std::string comPath = "test_regs.com";
    {
        std::ofstream f(comPath, std::ios::binary);
        uint8_t retOp = 0xC3;
        f.write(reinterpret_cast<const char*>(&retOp), 1);
    }

    constexpr uint16_t comSeg = 0x1200;
    REQUIRE(loader.loadCOM(comPath, comSeg, dos, "FILE.TXT"));
    REQUIRE(cpu.getReg16(cpu::AX) == 0x0000); // Both drives valid
    REQUIRE(cpu.getReg16(cpu::BX) == 0x0000);
    REQUIRE(cpu.getReg16(cpu::CX) == 0x00FF);
    REQUIRE(cpu.getReg16(cpu::DX) == comSeg);
    REQUIRE(cpu.getReg16(cpu::SI) == 0x0100);
    REQUIRE(cpu.getReg16(cpu::DI) == 0xFFFE);
    REQUIRE(cpu.getReg16(cpu::BP) == 0x091C);
    REQUIRE(cpu.getReg16(cpu::SP) == 0xFFFE);
    REQUIRE((cpu.getEFLAGS() & cpu::FLAG_INTERRUPT) != 0); // IF = 1

    std::filesystem::remove(comPath);

    // 2. EXE program startup registers
    const std::string exePath = "test_regs.exe";
    {
        std::ofstream f(exePath, std::ios::binary);
        // Minimal MZ Header
        uint16_t mzHeader[14] = {
            0x5A4D, // 'MZ'
            0x0020, // last page size (32 bytes)
            0x0001, // 1 page
            0x0000, // 0 relocations
            0x0002, // header paragraphs (32 bytes)
            0x0010, // minAlloc
            0xFFFF, // maxAlloc
            0x0000, // initial SS
            0x0100, // initial SP
            0x0000, // checksum
            0x0000, // initial IP
            0x0000, // initial CS
            0x001C, // reloc offset
            0x0000  // overlay
        };
        f.write(reinterpret_cast<const char*>(mzHeader), sizeof(mzHeader));
        uint8_t dummyCode[4] = {0x90, 0x90, 0xCD, 0x20};
        f.write(reinterpret_cast<const char*>(dummyCode), sizeof(dummyCode));
    }

    constexpr uint16_t exeSeg = 0x1400;
    REQUIRE(loader.loadEXE(exePath, exeSeg, dos, "A:FILE.DAT"));
    REQUIRE(cpu.getReg16(cpu::AX) == 0x0000);
    REQUIRE(cpu.getReg16(cpu::BX) == 0x0000);
    REQUIRE(cpu.getReg16(cpu::CX) == 0x0000);
    REQUIRE(cpu.getReg16(cpu::DX) == 0x0000);
    REQUIRE(cpu.getReg16(cpu::SI) == 0x0000); // IP
    REQUIRE(cpu.getReg16(cpu::DI) == 0x0100); // SP
    REQUIRE(cpu.getReg16(cpu::BP) == 0x091C);
    REQUIRE(cpu.getSegReg(cpu::DS) == exeSeg);
    REQUIRE(cpu.getSegReg(cpu::ES) == exeSeg);
    REQUIRE((cpu.getEFLAGS() & cpu::FLAG_INTERRUPT) != 0); // IF = 1

    std::filesystem::remove(exePath);
}

TEST_CASE("DOS Correctness: Extended Error Reporting (AH=59h)", "[dos][error]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::DOS dos(cpu, mem);
    dos.initialize();

    // Try to open a nonexistent file
    writeCString(mem, 0x10000, "nonexistent_file_xyz.txt");
    setSegment(cpu, cpu::DS, 0x1000);
    cpu.setReg16(cpu::DX, 0x0000);
    cpu.setReg8(cpu::AH, 0x3D); // Open File
    cpu.setReg8(cpu::AL, 0x00); // Read-only
    dos.handleInterrupt(0x21);

    REQUIRE((cpu.getEFLAGS() & cpu::FLAG_CARRY) != 0);
    REQUIRE(cpu.getReg16(cpu::AX) == 0x0002); // File not found

    // Now query Extended Error Information via AH=59h
    cpu.setReg8(cpu::AH, 0x59);
    cpu.setReg16(cpu::BX, 0x0000);
    dos.handleInterrupt(0x21);

    REQUIRE((cpu.getEFLAGS() & cpu::FLAG_CARRY) == 0);
    REQUIRE(cpu.getReg16(cpu::AX) == 0x0002); // Error code 2 (File not found)
    REQUIRE(cpu.getReg8(cpu::BH) == 0x08);   // Error class 8 (Not found)
    REQUIRE(cpu.getReg8(cpu::BL) == 0x03);   // Recommended action 3 (Prompt user to re-enter)
    REQUIRE(cpu.getReg8(cpu::CH) == 0x02);   // Locus 2 (Block device / disk)
}

TEST_CASE("DOS Correctness: Allocation Strategy First Fit vs Best Fit vs Last Fit", "[dos][memory][strategy]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::DOS dos(cpu, mem);
    dos.initialize();

    // Create three free blocks of varying sizes
    // Initial free block in dos.initialize is at 0x811
    uint16_t b1 = dos.allocateMemory(0x100); // 256 paras at ~0x812
    uint16_t b2 = dos.allocateMemory(0x300); // 768 paras
    uint16_t b3 = dos.allocateMemory(0x200); // 512 paras

    REQUIRE(b1 != 0);
    REQUIRE(b2 != 0);
    REQUIRE(b3 != 0);

    // Free b1 and b3 (b1 is 0x100 paras, b3 is 0x200 paras), keeping b2 allocated in between
    REQUIRE(dos.freeMemory(b1));
    REQUIRE(dos.freeMemory(b3));

    // 1. Test First Fit (Strategy 0x0000): should pick the lowest block that fits (b1-1 has 0x100 paras >= 0x80)
    cpu.setReg8(cpu::AH, 0x58);
    cpu.setReg8(cpu::AL, 0x01); // Set strategy
    cpu.setReg16(cpu::BX, 0x0000); // First Fit
    dos.handleInterrupt(0x21);

    uint16_t allocFF = dos.allocateMemory(0x80);
    REQUIRE(allocFF == b1);

    REQUIRE(dos.freeMemory(allocFF));

    // 2. Test Best Fit (Strategy 0x0001): should pick the smaller matching block (b1 is 0x100 paras, b3 is 0x200)
    cpu.setReg8(cpu::AH, 0x58);
    cpu.setReg8(cpu::AL, 0x01);
    cpu.setReg16(cpu::BX, 0x0001); // Best Fit
    dos.handleInterrupt(0x21);

    uint16_t allocBF = dos.allocateMemory(0x80);
    REQUIRE(allocBF == b1); // b1 (0x100 paras) is a tighter fit than b3 (0x200 paras)

    REQUIRE(dos.freeMemory(allocBF));

    // 3. Test Last Fit (Strategy 0x0002): should pick the highest matching block (b3)
    cpu.setReg8(cpu::AH, 0x58);
    cpu.setReg8(cpu::AL, 0x01);
    cpu.setReg16(cpu::BX, 0x0002); // Last Fit
    dos.handleInterrupt(0x21);

    uint16_t allocLF = dos.allocateMemory(0x80);
    REQUIRE(allocLF == b3);

    REQUIRE(dos.freeMemory(allocLF));
    REQUIRE(dos.freeMemory(b2));
}

TEST_CASE("DOS Correctness: PSP Job File Table (JFT) Synchronization", "[dos][jft]") {
    cpu::CPU cpu;
    memory::MemoryBus mem;
    hw::DOS dos(cpu, mem);
    dos.initialize();

    const std::string fname = "test_jft.tmp";
    writeCString(mem, 0x20000, fname);
    setSegment(cpu, cpu::DS, 0x2000);
    cpu.setReg16(cpu::DX, 0x0000);

    const uint32_t pspPhys = static_cast<uint32_t>(dos.getPSPSegment()) << 4;

    // Handles 0-4 are standard stdin/stdout/stderr
    for (int h = 0; h < 5; ++h) {
        REQUIRE(mem.read8(pspPhys + 0x18 + h) == static_cast<uint8_t>(h));
    }
    // Handle 5 should initially be unused (0xFF)
    REQUIRE(mem.read8(pspPhys + 0x18 + 5) == 0xFF);

    // Create file (AH=3Ch) -> should assign handle 5
    cpu.setReg8(cpu::AH, 0x3C);
    cpu.setReg16(cpu::CX, 0x0000);
    dos.handleInterrupt(0x21);

    REQUIRE((cpu.getEFLAGS() & cpu::FLAG_CARRY) == 0);
    uint16_t handle = cpu.getReg16(cpu::AX);
    REQUIRE(handle == 5);

    // Check that PSP:0018 + 5 is now active (non-0xFF)
    REQUIRE(mem.read8(pspPhys + 0x18 + 5) == 5);

    // Duplicate handle 5 (AH=45h) -> should assign handle 6
    cpu.setReg8(cpu::AH, 0x45);
    cpu.setReg16(cpu::BX, handle);
    dos.handleInterrupt(0x21);

    REQUIRE((cpu.getEFLAGS() & cpu::FLAG_CARRY) == 0);
    uint16_t dupHandle = cpu.getReg16(cpu::AX);
    REQUIRE(dupHandle == 6);
    REQUIRE(mem.read8(pspPhys + 0x18 + 6) == 5);

    // Close handle 5 (AH=3Eh)
    cpu.setReg8(cpu::AH, 0x3E);
    cpu.setReg16(cpu::BX, handle);
    dos.handleInterrupt(0x21);

    REQUIRE((cpu.getEFLAGS() & cpu::FLAG_CARRY) == 0);
    REQUIRE(mem.read8(pspPhys + 0x18 + 5) == 0xFF); // Slot 5 closed
    REQUIRE(mem.read8(pspPhys + 0x18 + 6) == 5);    // Slot 6 remains active

    // Close handle 6
    cpu.setReg8(cpu::AH, 0x3E);
    cpu.setReg16(cpu::BX, dupHandle);
    dos.handleInterrupt(0x21);

    REQUIRE((cpu.getEFLAGS() & cpu::FLAG_CARRY) == 0);
    REQUIRE(mem.read8(pspPhys + 0x18 + 6) == 0xFF);

    std::filesystem::remove(fname);
}
