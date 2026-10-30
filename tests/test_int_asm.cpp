// test_int_asm.cpp – assembler-driven integration tests for interrupt handlers.
// Each test assembles real x86 instructions, writes them into memory, and runs
// them through the full InstructionDecoder pipeline.

#include "test_framework.hpp"
#include <cstdio>
#include <filesystem>
#include <fstream>
#include "cpu/CPU.hpp"
#include "cpu/InstructionDecoder.hpp"
#include "cpu/Assembler.hpp"
#include "utils/Logger.hpp"
#include "memory/MemoryBus.hpp"
#include "hw/IOBus.hpp"
#include "hw/PIC8259.hpp"
#include "hw/BIOS.hpp"
#include "hw/DOS.hpp"
#include "hw/KeyboardController.hpp"
#include "hw/PIT8254.hpp"
#include "hw/DPMI.hpp"
#include "memory/himem/HIMEM.hpp"

using namespace fador;

// ── Helper: creates a full emulator stack and assembles+executes code ──
struct IntTestEnv {
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
    cpu::Assembler assembler;

    static constexpr uint16_t CODE_SEG = 0x1000;
    static constexpr uint16_t CODE_OFF = 0x100;
    static constexpr uint32_t CODE_LINEAR = (CODE_SEG << 4) + CODE_OFF;

    IntTestEnv()
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
        // Set PSP to match test CODE_SEG
        dos.setPSPSegment(CODE_SEG);
        // Use decoder.loadSegment() so the decoder's cached m_segBase[]
        // stays in sync with the CPU segment registers.
        decoder.loadSegment(cpu::CS, CODE_SEG);
        decoder.loadSegment(cpu::DS, CODE_SEG);
        decoder.loadSegment(cpu::ES, CODE_SEG);
        decoder.loadSegment(cpu::SS, 0x0000);
        cpu.setReg16(cpu::SP, 0xFFFE);
        cpu.setEIP(CODE_OFF);
        // Shrink PSP block to free memory for allocation tests.
        // The MCB at FIRST_MCB_SEGMENT (0x0800) owns all conventional memory;
        // resize it to just cover CODE_SEG area and create a free block after.
        shrinkPSPBlock();
    }

    // Assemble block, write to CS:IP, return number of bytes
    uint32_t assemble(const std::string& code) {
        auto r = assembler.assembleBlock(code, CODE_LINEAR);
        if (!r.error.empty())
            throw std::runtime_error("Assembly error: " + r.error);
        for (size_t i = 0; i < r.bytes.size(); ++i)
            mem.write8(CODE_LINEAR + static_cast<uint32_t>(i), r.bytes[i]);
        // Debug: print assembled bytes and location when DEBUG_LOGS is enabled
        LOG_INFO("Assembled ", r.bytes.size(), " bytes at 0x", std::hex, CODE_LINEAR);
        for (size_t i = 0; i < r.bytes.size(); ++i)
            LOG_DEBUG("ASM[0x", std::hex, (CODE_LINEAR + static_cast<uint32_t>(i)), "] = 0x", std::hex, (int)r.bytes[i]);
        return static_cast<uint32_t>(r.bytes.size());
    }

    // Step N instructions
    void run(int n) {
        for (int i = 0; i < n; ++i)
            decoder.step();
    }

    // Write a null-terminated string at DS:offset
    void writeString(uint16_t offset, const std::string& s) {
        uint32_t base = (cpu.getSegReg(cpu::DS) << 4) + offset;
        for (size_t i = 0; i < s.size(); ++i)
            mem.write8(base + static_cast<uint32_t>(i), static_cast<uint8_t>(s[i]));
        mem.write8(base + static_cast<uint32_t>(s.size()), 0);
    }

    // Write a '$'-terminated string at DS:offset
    void writeDollarString(uint16_t offset, const std::string& s) {
        uint32_t base = (cpu.getSegReg(cpu::DS) << 4) + offset;
        for (size_t i = 0; i < s.size(); ++i)
            mem.write8(base + static_cast<uint32_t>(i), static_cast<uint8_t>(s[i]));
        mem.write8(base + static_cast<uint32_t>(s.size()), '$');
    }

private:
    // Shrink the PSP MCB so there's free memory for allocation tests.
    // MCB at FIRST_MCB_SEGMENT(0x0800): owner=CODE_SEG, size=PSP_SIZE, type='M'
    // MCB at (0x0800 + 1 + PSP_SIZE): owner=0, size=remaining, type='Z' (free)
    void shrinkPSPBlock() {
        static constexpr uint16_t FIRST_MCB = 0x0800;
        static constexpr uint16_t LAST_PARA = 0x9FFF;
        // Give the PSP block enough room for CODE_SEG + 0x1000 paragraphs
        uint16_t pspSize = (CODE_SEG + 0x1000) - (FIRST_MCB + 1);
        uint16_t freeSeg = FIRST_MCB + 1 + pspSize;
        uint16_t freeSize = LAST_PARA - freeSeg;

        auto writeMCB = [&](uint16_t seg, uint8_t type, uint16_t owner, uint16_t size) {
            uint32_t addr = static_cast<uint32_t>(seg) << 4;
            mem.write8(addr, type);
            mem.write16(addr + 1, owner);
            mem.write16(addr + 3, size);
        };
        writeMCB(FIRST_MCB, 'M', CODE_SEG, pspSize);
        writeMCB(freeSeg, 'Z', 0x0000, freeSize);
    }
};

// ════════════════════════════════════════════════════════════════════════════
//  INT 10h – Video BIOS
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 10h AH=00h Set Video Mode via asm", "[int][asm][video]") {
    IntTestEnv e;
    e.assemble("MOV AX, 03h\nINT 10h");
    e.run(2); // MOV + INT
    REQUIRE(e.mem.read8(0x449) == 0x03); // BDA current mode
    REQUIRE(e.mem.read8(0x462) == 0x00); // Active page reset to 0
}

TEST_CASE("INT 10h AH=01h Set Cursor Type via asm", "[int][asm][video]") {
    IntTestEnv e;
    e.assemble("MOV AH, 01h\nMOV CX, 0607h\nINT 10h");
    e.run(3);
    REQUIRE(e.mem.read16(0x460) == 0x0607);
}

TEST_CASE("INT 10h AH=02h Set Cursor Position via asm", "[int][asm][video]") {
    IntTestEnv e;
    // Set cursor to row 5, col 10, page 0
    e.assemble("MOV AH, 02h\nMOV BH, 0\nMOV DH, 05h\nMOV DL, 0Ah\nINT 10h");
    e.run(5);
    REQUIRE(e.mem.read8(0x450) == 10); // Column
    REQUIRE(e.mem.read8(0x451) == 5);  // Row
}

TEST_CASE("INT 10h AH=03h Get Cursor Position via asm", "[int][asm][video]") {
    IntTestEnv e;
    // Pre-set cursor at row 3, col 7
    e.mem.write8(0x450, 7);  // Col
    e.mem.write8(0x451, 3);  // Row
    e.mem.write16(0x460, 0x0C0D); // Cursor type
    e.assemble("MOV AH, 03h\nMOV BH, 0\nINT 10h");
    e.run(3);
    REQUIRE(e.cpu.getReg8(cpu::DL) == 7); // Col
    REQUIRE(e.cpu.getReg8(cpu::DH) == 3); // Row
    REQUIRE(e.cpu.getReg16(cpu::CX) == 0x0C0D); // Cursor type
}

TEST_CASE("INT 10h AH=05h Set Active Display Page via asm", "[int][asm][video]") {
    IntTestEnv e;
    e.assemble("MOV AH, 05h\nMOV AL, 02h\nINT 10h");
    e.run(3);
    REQUIRE(e.mem.read8(0x462) == 2);
}

TEST_CASE("INT 10h AH=0Eh Teletype Output via asm", "[int][asm][video]") {
    IntTestEnv e;
    // Reset cursor to 0,0
    e.mem.write8(0x450, 0);
    e.mem.write8(0x451, 0);
    // Write 'H' then 'i'
    e.assemble("MOV AH, 0Eh\nMOV AL, 48h\nINT 10h\nMOV AL, 69h\nINT 10h");
    e.run(5);
    REQUIRE(e.mem.read8(0xB8000) == 'H');
    REQUIRE(e.mem.read8(0xB8002) == 'i');
    REQUIRE(e.mem.read8(0x450) == 2); // Cursor advanced by 2
}

TEST_CASE("INT 10h AH=08h Read Char at Cursor via asm", "[int][asm][video]") {
    IntTestEnv e;
    // Put 'Z' with attribute 0x1F at page 0, row 0, col 0
    e.mem.write8(0xB8000, 'Z');
    e.mem.write8(0xB8001, 0x1F);
    e.mem.write8(0x450, 0); e.mem.write8(0x451, 0); // Cursor at 0,0
    e.assemble("MOV AH, 08h\nMOV BH, 0\nINT 10h");
    e.run(3);
    REQUIRE(e.cpu.getReg8(cpu::AL) == 'Z');
    REQUIRE(e.cpu.getReg8(cpu::AH) == 0x1F);
}

TEST_CASE("INT 10h AH=09h Write Char and Attr via asm", "[int][asm][video]") {
    IntTestEnv e;
    e.mem.write8(0x450, 0); e.mem.write8(0x451, 0);
    // Write 'A' with attr 0x4E, 3 times
    e.assemble("MOV AH, 09h\nMOV AL, 41h\nMOV BL, 4Eh\nMOV CX, 3\nINT 10h");
    e.run(5);
    for (int i = 0; i < 3; ++i) {
        REQUIRE(e.mem.read8(0xB8000 + i * 2) == 'A');
        REQUIRE(e.mem.read8(0xB8000 + i * 2 + 1) == 0x4E);
    }
    // Cursor should NOT advance (INT 10h/09h doesn't move it)
    REQUIRE(e.mem.read8(0x450) == 0);
}

TEST_CASE("INT 10h AH=0Ah Write Char Only via asm", "[int][asm][video]") {
    IntTestEnv e;
    e.mem.write8(0x450, 0); e.mem.write8(0x451, 0);
    // Pre-fill attr so we can verify it's preserved
    e.mem.write8(0xB8001, 0x07);
    e.assemble("MOV AH, 0Ah\nMOV AL, 42h\nMOV CX, 1\nINT 10h");
    e.run(4);
    REQUIRE(e.mem.read8(0xB8000) == 'B');
    REQUIRE(e.mem.read8(0xB8001) == 0x07); // Attribute preserved
}

TEST_CASE("INT 10h AH=0Fh Get Video Mode via asm", "[int][asm][video]") {
    IntTestEnv e;
    e.mem.write8(0x449, 0x03); // Mode 3
    e.mem.write8(0x44A, 80);   // 80 columns
    e.mem.write8(0x462, 0);    // Page 0
    e.assemble("MOV AH, 0Fh\nINT 10h");
    e.run(2);
    REQUIRE(e.cpu.getReg8(cpu::AL) == 0x03);
    REQUIRE(e.cpu.getReg8(cpu::AH) == 80);
    REQUIRE(e.cpu.getReg8(cpu::BH) == 0);
}

TEST_CASE("INT 10h AH=06h Scroll Up via asm", "[int][asm][video]") {
    IntTestEnv e;
    // Fill line 0 with 'X'
    for (int i = 0; i < 80; ++i) {
        e.mem.write8(0xB8000 + i * 2, 'X');
        e.mem.write8(0xB8000 + i * 2 + 1, 0x07);
    }
    // Fill line 1 with 'Y'
    for (int i = 0; i < 80; ++i) {
        e.mem.write8(0xB8000 + 160 + i * 2, 'Y');
        e.mem.write8(0xB8000 + 160 + i * 2 + 1, 0x07);
    }
    // Scroll up 1 line, full screen, fill with attr 0x07
    e.assemble("MOV AH, 06h\nMOV AL, 01h\nMOV BH, 07h\nMOV CX, 0\nMOV DX, 184Fh\nINT 10h");
    e.run(6);
    // Line 0 should now contain 'Y' (scrolled up from line 1)
    REQUIRE(e.mem.read8(0xB8000) == 'Y');
    // Last line should be blank (space with attr 0x07)
    uint32_t lastLine = 0xB8000 + 24 * 160;
    REQUIRE(e.mem.read8(lastLine) == ' ');
    REQUIRE(e.mem.read8(lastLine + 1) == 0x07);
}

TEST_CASE("INT 10h AH=07h Scroll Down via asm", "[int][asm][video]") {
    IntTestEnv e;
    // Fill line 0 with 'A'
    for (int i = 0; i < 80; ++i) {
        e.mem.write8(0xB8000 + i * 2, 'A');
        e.mem.write8(0xB8000 + i * 2 + 1, 0x07);
    }
    // Scroll down 1 line, full screen
    e.assemble("MOV AH, 07h\nMOV AL, 01h\nMOV BH, 07h\nMOV CX, 0\nMOV DX, 184Fh\nINT 10h");
    e.run(6);
    // Line 0 should now be blank
    REQUIRE(e.mem.read8(0xB8000) == ' ');
    REQUIRE(e.mem.read8(0xB8001) == 0x07);
    // Line 1 should have 'A' (scrolled down)
    REQUIRE(e.mem.read8(0xB8000 + 160) == 'A');
}

TEST_CASE("INT 10h AH=13h Write String via asm", "[int][asm][video]") {
    IntTestEnv e;
    // Put string "Hi" at ES:BP
    e.cpu.setSegReg(cpu::ES, 0x2000);
    e.mem.write8(0x20000, 'H');
    e.mem.write8(0x20001, 'i');
    // Write string: AL=01 (move cursor), BL=0x07, CX=2, DH=0, DL=0, page 0
    e.assemble(
        "MOV AH, 13h\n"
        "MOV AL, 01h\n"
        "MOV BH, 0\n"
        "MOV BL, 07h\n"
        "MOV CX, 2\n"
        "MOV DH, 0\n"
        "MOV DL, 0\n"
        "INT 10h"
    );
    // Set BP to point to string
    e.cpu.setReg16(cpu::BP, 0x0000);
    e.run(8);
    REQUIRE(e.mem.read8(0xB8000) == 'H');
    REQUIRE(e.mem.read8(0xB8002) == 'i');
    // Cursor should have advanced (mode 01)
    REQUIRE(e.mem.read8(0x450) == 2);
}

// ════════════════════════════════════════════════════════════════════════════
//  INT 16h – Keyboard BIOS
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 16h AH=01h Peek keyboard (no key) via asm", "[int][asm][kbd]") {
    IntTestEnv e;
    e.assemble("MOV AH, 01h\nINT 16h");
    e.run(2);
    REQUIRE(e.cpu.getEFLAGS() & cpu::FLAG_ZERO); // ZF=1: no key
}

TEST_CASE("INT 16h AH=01h Peek keyboard (key present) via asm", "[int][asm][kbd]") {
    IntTestEnv e;
    e.kbd.pushKey('a', 0x1E); // 'A' key (ascii + scancode)
    e.assemble("MOV AH, 01h\nINT 16h");
    e.run(2);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_ZERO)); // ZF=0: key present
    REQUIRE(e.cpu.getReg8(cpu::AL) == 'a'); // ASCII
    REQUIRE(e.cpu.getReg8(cpu::AH) == 0x1E); // Scan code
}

TEST_CASE("INT 16h AH=02h Get Shift Flags via asm", "[int][asm][kbd]") {
    IntTestEnv e;
    e.mem.write8(0x417, 0x03); // Left Shift + Right Shift
    e.assemble("MOV AH, 02h\nINT 16h");
    e.run(2);
    REQUIRE(e.cpu.getReg8(cpu::AL) == 0x03);
}

TEST_CASE("INT 16h AH=05h Store Key in buffer via asm", "[int][asm][kbd]") {
    IntTestEnv e;
    // Store key 'Z' (scancode 0x2C) into buffer, then peek it
    e.assemble(
        "MOV AH, 05h\n"
        "MOV CL, 5Ah\n"  // ASCII 'Z'
        "MOV CH, 2Ch\n"  // Scancode
        "INT 16h\n"
        "MOV AH, 01h\n"
        "INT 16h"
    );
    e.run(6);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_ZERO)); // Key present
}

// ════════════════════════════════════════════════════════════════════════════
//  INT 1Ah – Timer
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 1Ah AH=00h Read Timer Ticks via asm", "[int][asm][timer]") {
    IntTestEnv e;
    // Set tick count in BDA at 0x46C (dword)
    e.mem.write16(0x46C, 0x1234);
    e.mem.write16(0x46E, 0x0056);
    e.assemble("MOV AH, 0\nINT 1Ah");
    e.run(2);
    REQUIRE(e.cpu.getReg16(cpu::DX) == 0x1234); // Low word
    REQUIRE(e.cpu.getReg16(cpu::CX) == 0x0056); // High word
    REQUIRE(e.cpu.getReg8(cpu::AL) == 0); // No midnight
}

// ════════════════════════════════════════════════════════════════════════════
//  INT 11h / INT 12h – Equipment / Memory Size
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 11h Equipment List via asm", "[int][asm]") {
    IntTestEnv e;
    e.assemble("INT 11h");
    e.run(1);
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0x002F);
}

TEST_CASE("INT 12h Memory Size via asm", "[int][asm]") {
    IntTestEnv e;
    e.assemble("INT 12h");
    e.run(1);
    REQUIRE(e.cpu.getReg16(cpu::AX) == 640);
}

// ════════════════════════════════════════════════════════════════════════════
//  INT 13h – Disk BIOS
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 13h AH=00h Reset Disk via asm", "[int][asm][disk]") {
    IntTestEnv e;
    e.assemble("MOV AH, 0\nMOV DL, 0\nINT 13h");
    e.run(3);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg8(cpu::AH) == 0);
}

TEST_CASE("INT 13h AH=08h Get Drive Params via asm", "[int][asm][disk]") {
    IntTestEnv e;
    e.assemble("MOV AH, 08h\nMOV DL, 0\nINT 13h");
    e.run(3);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg8(cpu::BL) == 0x04); // 1.44MB
    REQUIRE(e.cpu.getReg8(cpu::CL) == 18);   // 18 sectors
}

TEST_CASE("INT 13h AH=15h Get Disk Type via asm", "[int][asm][disk]") {
    IntTestEnv e;
    e.assemble("MOV AH, 15h\nMOV DL, 0\nINT 13h");
    e.run(3);
    REQUIRE(e.cpu.getReg8(cpu::AH) == 0x01); // Floppy, no change line
}

// ════════════════════════════════════════════════════════════════════════════
//  INT 14h / INT 15h / INT 17h – Stubs
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 14h Serial Stub via asm", "[int][asm]") {
    IntTestEnv e;
    e.assemble("MOV AH, 0FFh\nINT 14h");
    e.run(2);
    REQUIRE(e.cpu.getReg8(cpu::AH) == 0);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
}

TEST_CASE("INT 15h System Services Stub via asm", "[int][asm]") {
    IntTestEnv e;
    e.assemble("MOV AH, 0\nINT 15h");
    e.run(2);
    REQUIRE(e.cpu.getReg8(cpu::AH) == 0x86);
    REQUIRE(e.cpu.getEFLAGS() & cpu::FLAG_CARRY);
}

TEST_CASE("INT 17h Printer Stub via asm", "[int][asm]") {
    IntTestEnv e;
    e.assemble("MOV AH, 0FFh\nINT 17h");
    e.run(2);
    REQUIRE(e.cpu.getReg8(cpu::AH) == 0);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
}

// ════════════════════════════════════════════════════════════════════════════
//  INT 33h – Mouse
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 33h AX=0000h Mouse Reset via asm", "[int][asm][mouse]") {
    IntTestEnv e;
    e.assemble("MOV AX, 0\nINT 33h");
    e.run(2);
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0xFFFF); // Installed
    REQUIRE(e.cpu.getReg16(cpu::BX) == 0x0002); // 2 buttons
}

TEST_CASE("INT 33h AX=0001h/0002h Show/Hide cursor via asm", "[int][asm][mouse]") {
    IntTestEnv e;
    // Reset first
    e.bios.mouseState().installed = true;
    e.assemble("MOV AX, 1\nINT 33h");
    e.run(2);
    REQUIRE(e.bios.mouseState().visible == true);
    // Now hide
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble("MOV AX, 2\nINT 33h");
    e.run(2);
    REQUIRE(e.bios.mouseState().visible == false);
}

TEST_CASE("INT 33h AX=0003h Get Mouse Position via asm", "[int][asm][mouse]") {
    IntTestEnv e;
    e.bios.mouseState().installed = true;
    e.bios.mouseState().x = 100;
    e.bios.mouseState().y = 50;
    e.bios.mouseState().buttons = 1; // Left button
    e.assemble("MOV AX, 3\nINT 33h");
    e.run(2);
    REQUIRE(e.cpu.getReg16(cpu::BX) == 1);   // Buttons
    REQUIRE(e.cpu.getReg16(cpu::CX) == 100); // X
    REQUIRE(e.cpu.getReg16(cpu::DX) == 50);  // Y
}

TEST_CASE("INT 33h AX=0004h Set Mouse Position via asm", "[int][asm][mouse]") {
    IntTestEnv e;
    e.bios.mouseState().installed = true;
    e.assemble("MOV AX, 4\nMOV CX, 200\nMOV DX, 150\nINT 33h");
    e.run(4);
    REQUIRE(e.bios.mouseState().x == 200);
    REQUIRE(e.bios.mouseState().y == 150);
}

TEST_CASE("INT 33h AX=0021h Software Reset via asm", "[int][asm][mouse]") {
    IntTestEnv e;
    e.assemble("MOV AX, 21h\nINT 33h");
    e.run(2);
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0xFFFF);
    REQUIRE(e.cpu.getReg16(cpu::BX) == 0x0002);
}

// ════════════════════════════════════════════════════════════════════════════
//  INT 2Fh – Multiplex
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 2Fh Multiplex not-installed via asm", "[int][asm]") {
    IntTestEnv e;
    e.assemble("MOV AX, 1234h\nINT 2Fh");
    e.run(2);
    REQUIRE(e.cpu.getReg8(cpu::AL) == 0);
}

// ════════════════════════════════════════════════════════════════════════════
//  INT 20h – Terminate
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 20h Terminate via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.assemble("INT 20h");
    e.run(1);
    REQUIRE(e.dos.isTerminated());
    REQUIRE(e.dos.getExitCode() == 0);
}

TEST_CASE("0F FF INT 21h handled by DOS uses chain-return semantics", "[int][asm][chain][doom]") {
    IntTestEnv e;

    // Prepare real-mode stack and code segment
    e.cpu.setSegReg(cpu::CS, IntTestEnv::CODE_SEG);
    e.cpu.setSegBase(cpu::CS, IntTestEnv::CODE_SEG << 4);
    e.cpu.setSegReg(cpu::SS, 0x0000);
    e.cpu.setSegBase(cpu::SS, 0x0000);
    e.cpu.setIs32BitCode(false);
    e.cpu.setIs32BitStack(false);

    // Place a chain frame at SS:SP (IP, CS, FLAGS)
    e.cpu.setReg16(cpu::SP, 0x0200);
    uint32_t ssBase = (e.cpu.getSegReg(cpu::SS) << 4);
    e.mem.write16(ssBase + 0x0200, 0x3456); // IP
    e.mem.write16(ssBase + 0x0202, 0x1234); // CS
    e.mem.write16(ssBase + 0x0204, 0x0202); // FLAGS

    // Arrange DOS to handle INT 21h AH=02h (print char)
    e.cpu.setReg8(cpu::AH, 0x02);
    e.cpu.setReg8(cpu::DL, 'X');

    // Assemble 0F FF 21 at CS:IP
    e.assemble("0F FF 21");

    REQUIRE(e.cpu.hleStackSize() == 0);
    e.run(1); // execute the HLE trap
    // After handling, should have returned to chained caller IP/CS and adjusted SP
    LOG_INFO("TEST DBG: CS=0x", std::hex, e.cpu.getSegReg(cpu::CS), " EIP=0x", e.cpu.getEIP(), " SP=0x", e.cpu.getReg16(cpu::SP), " hleSize=", e.cpu.hleStackSize());
    REQUIRE(e.cpu.getSegReg(cpu::CS) == 0x1234);
    REQUIRE(e.cpu.getEIP() == 0x3456);
    REQUIRE(e.cpu.getReg16(cpu::SP) == 0x0206);
    REQUIRE(e.cpu.hleStackSize() == 0);

    // DOS should have written the character to VRAM via INT 21h/02h
    REQUIRE(e.mem.read8(0xB8000) == 'X');
}

TEST_CASE("0F FF INT 21h pops tracked HLE frame when framePhysAddr matches", "[int][asm][chain]") {
    IntTestEnv e;

    // Prepare real-mode stack and code segment
    e.cpu.setSegReg(cpu::CS, IntTestEnv::CODE_SEG);
    e.cpu.setSegBase(cpu::CS, IntTestEnv::CODE_SEG << 4);
    e.cpu.setSegReg(cpu::SS, 0x0000);
    e.cpu.setSegBase(cpu::SS, 0x0000);
    e.cpu.setIs32BitCode(false);
    e.cpu.setIs32BitStack(false);

    // Place an IRET-like frame at SS:SP (IP, CS, FLAGS)
    e.cpu.setReg16(cpu::SP, 0x0300);
    uint32_t framePhys = (e.cpu.getSegReg(cpu::SS) << 4) + 0x0300;
    e.mem.write16(framePhys + 0x0000, 0x1111); // IP
    e.mem.write16(framePhys + 0x0002, 0x2222); // CS
    e.mem.write16(framePhys + 0x0004, 0x0202); // FLAGS

    // Simulate an HLE frame being tracked for INT 0x21
    e.cpu.pushHLEFrame(false, 0x21);
    e.cpu.lastHLEFrameMut().framePhysAddr = framePhys;
    e.cpu.lastHLEFrameMut().frameSP = 0x0300;

    // Arrange DOS to handle INT 21h AH=02h (print char)
    e.cpu.setReg8(cpu::AH, 0x02);
    e.cpu.setReg8(cpu::DL, 'Z');

    // Assemble 0F FF 21 at CS:IP
    e.assemble("0F FF 21");

    REQUIRE(e.cpu.hleStackSize() == 1);
    e.run(1); // execute the HLE trap
    // The tracked HLE frame should have been popped and execution
    // should return to the frame's CS:IP and DOS should print the char.
    LOG_INFO("TEST DBG (pop): CS=0x", std::hex, e.cpu.getSegReg(cpu::CS), " EIP=0x", e.cpu.getEIP(), " SP=0x", e.cpu.getReg16(cpu::SP), " hleSize=", e.cpu.hleStackSize());
    REQUIRE(e.cpu.hleStackSize() == 0);
    REQUIRE(e.cpu.getSegReg(cpu::CS) == 0x2222);
    REQUIRE(e.cpu.getEIP() == 0x1111);
    REQUIRE(e.mem.read8(0xB8000) == 'Z');
}

// ════════════════════════════════════════════════════════════════════════════
//  INT 21h – DOS API
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 21h AH=02h Print Character via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.mem.write8(0x450, 0); e.mem.write8(0x451, 0); // Cursor at 0,0
    e.assemble("MOV AH, 02h\nMOV DL, 41h\nINT 21h"); // Print 'A'
    e.run(3);
    REQUIRE(e.mem.read8(0xB8000) == 'A');
}

TEST_CASE("INT 21h AH=09h Print Dollar String via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.mem.write8(0x450, 0); e.mem.write8(0x451, 0);
    // Put "Hi$" at DS:0200h
    e.writeDollarString(0x200, "Hi");
    e.assemble("MOV AH, 09h\nMOV DX, 200h\nINT 21h");
    e.run(3);
    REQUIRE(e.mem.read8(0xB8000) == 'H');
    REQUIRE(e.mem.read8(0xB8002) == 'i');
}

TEST_CASE("INT 21h AH=25h/35h Set and Get Interrupt Vector via asm", "[int][asm][dos]") {
    IntTestEnv e;
    // Set vector 0x77 to DS:1234h
    e.assemble(
        "MOV AH, 25h\n"
        "MOV AL, 77h\n"
        "MOV DX, 1234h\n"
        "INT 21h\n"
        "MOV AH, 35h\n"
        "MOV AL, 77h\n"
        "INT 21h"
    );
    e.run(7);
    // After Get Vector: ES:BX should be DS:1234h
    REQUIRE(e.cpu.getReg16(cpu::BX) == 0x1234);
    REQUIRE(e.cpu.getSegReg(cpu::ES) == e.cpu.getSegReg(cpu::DS));
}

TEST_CASE("INT 21h AH=2Ah Get Date via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.assemble("MOV AH, 2Ah\nINT 21h");
    e.run(2);
    // Year should be reasonable (2020-2030 range)
    uint16_t year = e.cpu.getReg16(cpu::CX);
    REQUIRE(year >= 2020);
    REQUIRE(year <= 2030);
    // Month 1-12
    uint8_t month = e.cpu.getReg8(cpu::DH);
    REQUIRE(month >= 1);
    REQUIRE(month <= 12);
    // Day 1-31
    uint8_t day = e.cpu.getReg8(cpu::DL);
    REQUIRE(day >= 1);
    REQUIRE(day <= 31);
}

TEST_CASE("INT 21h AH=2Ch Get Time via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.assemble("MOV AH, 2Ch\nINT 21h");
    e.run(2);
    uint8_t hour = e.cpu.getReg8(cpu::CH);
    uint8_t minute = e.cpu.getReg8(cpu::CL);
    REQUIRE(hour <= 23);
    REQUIRE(minute <= 59);
}

TEST_CASE("INT 21h AH=30h Get DOS Version via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.assemble("MOV AH, 30h\nINT 21h");
    e.run(2);
    // Major version in AL (should be 3 or 5 depending on implementation)
    uint8_t major = e.cpu.getReg8(cpu::AL);
    REQUIRE((major == 3 || major == 5 || major == 6));
}

TEST_CASE("INT 21h AH=36h Get Free Disk Space via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.assemble("MOV AH, 36h\nMOV DL, 0\nINT 21h");
    e.run(3);
    REQUIRE(e.cpu.getReg16(cpu::AX) == 32);    // Sectors per cluster
    REQUIRE(e.cpu.getReg16(cpu::CX) == 512);   // Bytes per sector
    // BX = free clusters (should be > 0)
    REQUIRE(e.cpu.getReg16(cpu::BX) > 0);
}

TEST_CASE("INT 21h AH=4Ch Terminate with exit code via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.assemble("MOV AH, 4Ch\nMOV AL, 42h\nINT 21h");
    e.run(3);
    REQUIRE(e.dos.isTerminated());
    REQUIRE(e.dos.getExitCode() == 0x42);
}

TEST_CASE("INT 21h AH=51h Get PSP via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.assemble("MOV AH, 51h\nINT 21h");
    e.run(2);
    REQUIRE(e.cpu.getReg16(cpu::BX) == 0x1000);
}

TEST_CASE("INT 21h AH=62h Get PSP (documented) via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.assemble("MOV AH, 62h\nINT 21h");
    e.run(2);
    REQUIRE(e.cpu.getReg16(cpu::BX) == 0x1000);
}

TEST_CASE("INT 21h AH=44h IOCTL Get Device Info via asm", "[int][asm][dos]") {
    IntTestEnv e;
    // Stdin handle = 0
    e.assemble("MOV AH, 44h\nMOV AL, 0\nMOV BX, 0\nINT 21h");
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    // Bit 7 should be set (character device)
    REQUIRE(e.cpu.getReg16(cpu::DX) & 0x80);
}

TEST_CASE("INT 21h AH=2Fh Get DTA Address via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.assemble("MOV AH, 2Fh\nINT 21h");
    e.run(2);
    // Default DTA is at PSP:0080h
    REQUIRE(e.cpu.getReg16(cpu::BX) == 0x0080);
}

TEST_CASE("INT 21h AH=1Ah Set DTA via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.assemble("MOV AH, 1Ah\nMOV DX, 300h\nINT 21h\nMOV AH, 2Fh\nINT 21h");
    e.run(5);
    // After setting DTA to DS:0300h, Get DTA should return it
    REQUIRE(e.cpu.getReg16(cpu::BX) == 0x0300);
}

TEST_CASE("INT 21h AH=67h Set Handle Count via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.assemble("MOV AH, 67h\nMOV BX, 40h\nINT 21h");
    e.run(3);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.mem.read16((IntTestEnv::CODE_SEG << 4) + 0x32) == 0x0040);
}

TEST_CASE("INT 21h AH=37h Get Switch Character via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.assemble("MOV AH, 37h\nMOV AL, 0\nINT 21h");
    e.run(3);
    REQUIRE(e.cpu.getReg8(cpu::DL) == '/');
}

TEST_CASE("INT 21h AH=33h Get Ctrl-Break / Boot Drive via asm", "[int][asm][dos]") {
    IntTestEnv e;
    // AL=05h: Get boot drive
    e.assemble("MOV AH, 33h\nMOV AL, 05h\nINT 21h");
    e.run(3);
    REQUIRE(e.cpu.getReg8(cpu::DL) == 3); // C: drive
}

// ════════════════════════════════════════════════════════════════════════════
//  INT 21h – File I/O (Create, Write, Read, Seek, Close)
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 21h File I/O roundtrip via asm", "[int][asm][dos][file]") {
    IntTestEnv e;
    const char* fname = "asm_test_file.txt";

    // Write filename at DS:0200
    e.writeString(0x200, fname);

    // AH=3Ch: Create file
    e.assemble("MOV AH, 3Ch\nMOV CX, 0\nMOV DX, 200h\nINT 21h");
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    uint16_t handle = e.cpu.getReg16(cpu::AX);
    REQUIRE(handle >= 5);

    // AH=40h: Write "Hello" (5 bytes at DS:0300)
    e.writeDollarString(0x300, "Hello"); // '$' after, but we only write 5 bytes
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble("MOV AH, 40h\nMOV CX, 5\nMOV DX, 300h\nINT 21h");
    e.cpu.setReg16(cpu::BX, handle);
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg16(cpu::AX) == 5);

    // AH=42h: Seek to beginning (AL=0, CX:DX=0)
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble("MOV AH, 42h\nMOV AL, 0\nMOV CX, 0\nMOV DX, 0\nINT 21h");
    e.cpu.setReg16(cpu::BX, handle);
    e.run(5);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));

    // AH=3Fh: Read back 5 bytes into DS:0400
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble("MOV AH, 3Fh\nMOV CX, 5\nMOV DX, 400h\nINT 21h");
    e.cpu.setReg16(cpu::BX, handle);
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg16(cpu::AX) == 5);
    uint32_t readBase = (e.cpu.getSegReg(cpu::DS) << 4) + 0x400;
    REQUIRE(e.mem.read8(readBase + 0) == 'H');
    REQUIRE(e.mem.read8(readBase + 1) == 'e');
    REQUIRE(e.mem.read8(readBase + 2) == 'l');
    REQUIRE(e.mem.read8(readBase + 3) == 'l');
    REQUIRE(e.mem.read8(readBase + 4) == 'o');

    // AH=3Eh: Close file
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble("MOV AH, 3Eh\nINT 21h");
    e.cpu.setReg16(cpu::BX, handle);
    e.run(2);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));

    std::remove(fname);
}

TEST_CASE("INT 21h AH=43h Get File Attributes via asm", "[int][asm][dos][file]") {
    IntTestEnv e;
    const char* fname = "asm_attr_test.txt";
    { std::ofstream ofs(fname); ofs << "test"; }

    e.writeString(0x200, fname);

    // AL=00h: Get attributes
    e.assemble("MOV AH, 43h\nMOV AL, 0\nMOV DX, 200h\nINT 21h");
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    // CX should contain archive bit (0x20) at minimum
    uint16_t attr = e.cpu.getReg16(cpu::CX);
    REQUIRE((attr & 0x20) == 0x20); // Archive bit

    std::remove(fname);
}

TEST_CASE("INT 21h AH=41h Delete File via asm", "[int][asm][dos][file]") {
    IntTestEnv e;
    const char* fname = "asm_delete_test.txt";
    { std::ofstream ofs(fname); ofs << "delete me"; }

    REQUIRE(std::filesystem::exists(fname));

    e.writeString(0x200, fname);
    e.assemble("MOV AH, 41h\nMOV DX, 200h\nINT 21h");
    e.run(3);

    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    const bool existsAfterDelete = std::filesystem::exists(fname);
    std::remove(fname);
    REQUIRE(!existsAfterDelete);
}

TEST_CASE("INT 21h AH=41h Delete missing file reports DOS error", "[int][asm][dos][file]") {
    IntTestEnv e;
    const char* fname = "asm_delete_missing.txt";
    std::remove(fname);

    e.writeString(0x200, fname);
    e.assemble("MOV AH, 41h\nMOV DX, 200h\nINT 21h");
    e.run(3);

    REQUIRE(e.cpu.getEFLAGS() & cpu::FLAG_CARRY);
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0x02);
}

// ════════════════════════════════════════════════════════════════════════════
//  INT 21h – Memory Management
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 21h AH=48h/49h Alloc and Free Memory via asm", "[int][asm][dos][mem]") {
    IntTestEnv e;
    // Allocate 16 paragraphs (256 bytes)
    e.assemble("MOV AH, 48h\nMOV BX, 10h\nINT 21h");
    e.run(3);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    uint16_t allocSeg = e.cpu.getReg16(cpu::AX);
    REQUIRE(allocSeg > 0);

    // Free it: ES = allocated segment
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.decoder.loadSegment(cpu::ES, allocSeg);
    e.assemble("MOV AH, 49h\nINT 21h");
    e.run(2);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
}

TEST_CASE("INT 21h AH=4Ah Resize Memory via asm", "[int][asm][dos][mem]") {
    IntTestEnv e;
    // Allocate 16 paragraphs
    e.assemble("MOV AH, 48h\nMOV BX, 10h\nINT 21h");
    e.run(3);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    uint16_t allocSeg = e.cpu.getReg16(cpu::AX);

    // Resize to 32 paragraphs
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.decoder.loadSegment(cpu::ES, allocSeg);
    e.assemble("MOV AH, 4Ah\nMOV BX, 20h\nINT 21h");
    e.run(3);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));

    // Free
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble("MOV AH, 49h\nINT 21h");
    e.run(2);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
}

TEST_CASE("INT 21h AH=4Ah Resize Memory returns max block on failure via asm", "[int][asm][dos][mem]") {
    IntTestEnv e;

    e.assemble("MOV AH, 48h\nMOV BX, 10h\nINT 21h");
    e.run(3);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));

    const uint16_t allocSeg = e.cpu.getReg16(cpu::AX);
    const uint16_t mcbSeg = static_cast<uint16_t>(allocSeg - 1);
    const uint32_t mcbAddr = static_cast<uint32_t>(mcbSeg) << 4;
    const uint16_t initialSize = e.mem.read16(mcbAddr + 3);
    const uint16_t nextSeg = static_cast<uint16_t>(mcbSeg + initialSize + 1);
    const uint32_t nextMcbAddr = static_cast<uint32_t>(nextSeg) << 4;
    const uint16_t maxPossible = static_cast<uint16_t>(
        initialSize + 1 + e.mem.read16(nextMcbAddr + 3));

    REQUIRE(e.mem.read16(nextMcbAddr + 1) == 0);

    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.decoder.loadSegment(cpu::ES, allocSeg);
    e.assemble("MOV AH, 4Ah\nMOV BX, 0FFFFh\nINT 21h");
    e.run(3);

    REQUIRE(e.cpu.getEFLAGS() & cpu::FLAG_CARRY);
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0x0008);
    REQUIRE(e.cpu.getReg16(cpu::BX) == maxPossible);
    REQUIRE(e.mem.read16(mcbAddr + 3) == maxPossible);
    REQUIRE(e.mem.read8(mcbAddr) == 'Z');
}

// ════════════════════════════════════════════════════════════════════════════
//  INT 21h – Directory Operations
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 21h AH=47h Get Current Directory via asm", "[int][asm][dos][dir]") {
    IntTestEnv e;
    // Buffer at DS:SI for result
    e.assemble("MOV AH, 47h\nMOV DL, 0\nMOV SI, 400h\nINT 21h");
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
}

// ════════════════════════════════════════════════════════════════════════════
//  INT 33h — Mouse Driver
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 33h AX=0000h Reset mouse via asm", "[int][asm][mouse]") {
    IntTestEnv e;
    e.assemble("MOV AX, 0000h\nINT 33h");
    e.run(2);
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0xFFFF); // Driver installed
    REQUIRE(e.cpu.getReg16(cpu::BX) == 2);      // 2 buttons
}

TEST_CASE("INT 33h AX=0001h Show / 0002h Hide cursor via asm", "[int][asm][mouse]") {
    IntTestEnv e;
    e.assemble(
        "MOV AX, 0000h\n"  // Reset
        "INT 33h\n"
        "MOV AX, 0001h\n"  // Show cursor
        "INT 33h\n"
        "MOV AX, 0002h\n"  // Hide cursor
        "INT 33h"
    );
    e.run(6);
    // Reset returns AX=0xFFFF (installed), BX=2 (2 buttons)
    // Show/Hide don't change AX/BX — so carry flag check is sufficient
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
}

TEST_CASE("INT 33h AX=0003h Get position via asm", "[int][asm][mouse]") {
    IntTestEnv e;
    // Reset first, then set position, then get
    e.assemble(
        "MOV AX, 0000h\n"  // Reset
        "INT 33h\n"
        "MOV AX, 0004h\n"  // Set position to (320, 100) = (0x140, 0x64)
        "MOV CX, 140h\n"
        "MOV DX, 64h\n"
        "INT 33h\n"
        "MOV AX, 0003h\n"  // Get position
        "INT 33h"
    );
    e.run(7);
    // After set/get, CX=0x140 (320), DX=0x64 (100), BX=button state (0 unless pressed)
    REQUIRE(e.cpu.getReg16(cpu::CX) == 0x140);
    REQUIRE(e.cpu.getReg16(cpu::DX) == 0x64);
}

TEST_CASE("INT 33h AX=0004h Set position clamps to default bounds via asm", "[int][asm][mouse]") {
    IntTestEnv e;
    // Reset (sets bounds to 0-639, 0-199)
    e.assemble(
        "MOV AX, 0000h\n"
        "INT 33h\n"
        "MOV AX, 0004h\n"  // Set to (640, 200) = (0x280, 0xC8) — within bounds
        "MOV CX, 280h\n"
        "MOV DX, 0C8h\n"
        "INT 33h\n"
        "MOV AX, 0003h\n"  // Get position
        "INT 33h"
    );
    e.run(7);
    REQUIRE(e.cpu.getReg16(cpu::CX) == 0x280);
    REQUIRE(e.cpu.getReg16(cpu::DX) == 0xC8);
}

TEST_CASE("INT 33h AX=0007h/0008h Set bounds via asm", "[int][asm][mouse]") {
    IntTestEnv e;
    e.assemble(
        "MOV AX, 0000h\n"  // Reset
        "INT 33h\n"
        "MOV AX, 0007h\n"  // Set horizontal bounds: 100–500 (0x64–0x1F4)
        "MOV CX, 64h\n"
        "MOV DX, 1F4h\n"
        "INT 33h\n"
        "MOV AX, 0008h\n"  // Set vertical bounds: 50–150 (0x32–0x96)
        "MOV CX, 32h\n"
        "MOV DX, 96h\n"
        "INT 33h\n"
        "MOV AX, 0004h\n"  // Set position to midpoint
        "MOV CX, 0C8h\n"   // 0xC8 = 200
        "MOV DX, 64h\n"    // 0x64 = 100
        "INT 33h\n"
        "MOV AX, 0003h\n"  // Get position
        "INT 33h"
    );
    e.run(14);
    REQUIRE(e.cpu.getReg16(cpu::CX) == 0xC8); // 200 (within bounds)
    REQUIRE(e.cpu.getReg16(cpu::DX) == 0x64); // 100 (within bounds)
}

TEST_CASE("IRQ1 INT 9 dispatch via asm-installed RM handler", "[int][asm][kbd][irq]") {
    IntTestEnv e;

    e.iobus.registerDevice(0x20, 0x21, &e.pic);
    e.iobus.registerDevice(0x60, 0x60, &e.kbd);

    e.pic.write8(0x20, 0x11);
    e.pic.write8(0x21, 0x08);
    e.pic.write8(0x21, 0x04);
    e.pic.write8(0x21, 0x01);
    e.pic.write8(0x21, 0x00);

    constexpr uint16_t callerCs = 0x1234;
    constexpr uint16_t callerIp = 0x0100;

    e.cpu.setCR(0, 0);
    e.decoder.loadSegment(cpu::CS, callerCs);
    e.decoder.loadSegment(cpu::DS, 0x0000);
    e.decoder.loadSegment(cpu::SS, 0x0000);
    e.cpu.setReg16(cpu::SP, 0x0200);
    e.cpu.setEIP(callerIp);

    e.mem.write16(0x09 * 4, IntTestEnv::CODE_OFF);
    e.mem.write16(0x09 * 4 + 2, IntTestEnv::CODE_SEG);

    e.assemble(
        "PUSH AX\n"
        "PUSH BX\n"
        "IN AL, 60h\n"
        "MOV BX, 0500h\n"
        "MOV [BX], AL\n"
        "MOV AL, 20h\n"
        "OUT 20h, AL\n"
        "POP BX\n"
        "POP AX\n"
        "IRET"
    );

    e.kbd.pushMakeKey('a', 0x1E);
    REQUIRE(e.kbd.checkPendingIRQ());
    e.pic.raiseIRQ(1);
    REQUIRE(e.pic.getPendingInterrupt() == 0x09);
    e.pic.acknowledgeInterrupt();

    e.decoder.injectHardwareInterrupt(0x09);

    REQUIRE(e.cpu.getSegReg(cpu::CS) == IntTestEnv::CODE_SEG);
    REQUIRE(e.cpu.getEIP() == IntTestEnv::CODE_OFF);
    REQUIRE(e.cpu.getReg16(cpu::SP) == 0x01FA);

    e.run(10);

    REQUIRE(e.mem.read8(0x0500) == 0x1E);
    REQUIRE(e.cpu.getSegReg(cpu::CS) == callerCs);
    REQUIRE(e.cpu.getEIP() == callerIp);
    REQUIRE(e.cpu.getReg16(cpu::SP) == 0x0200);

    e.pic.raiseIRQ(1);
    REQUIRE(e.pic.getPendingInterrupt() == 0x09);
}

// ════════════════════════════════════════════════════════════════════════════
//  INT 16h — Extended Keyboard BIOS
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 16h AH=00h blocking read key via asm", "[int][asm][kbd]") {
    IntTestEnv e;
    // Push a key into buffer, then do blocking read
    e.kbd.pushKey('X', 0x2D);
    e.assemble("MOV AH, 00h\nINT 16h");
    e.run(2);
    REQUIRE(e.cpu.getReg8(cpu::AL) == 'X');   // ASCII
    REQUIRE(e.cpu.getReg8(cpu::AH) == 0x2D);  // Scancode
}

TEST_CASE("INT 16h AH=10h extended keyboard read via asm", "[int][asm][kbd]") {
    IntTestEnv e;
    // Push an extended key (arrow key: scancode 0x48 with E0 prefix emulated as normal)
    e.kbd.pushKey(0x00, 0x48); // Up arrow: no ASCII, scancode 0x48
    e.assemble("MOV AH, 10h\nINT 16h");
    e.run(2);
    REQUIRE(e.cpu.getReg8(cpu::AH) == 0x48);  // Scancode
    REQUIRE(e.cpu.getReg8(cpu::AL) == 0x00);  // No ASCII for extended keys
}

TEST_CASE("INT 16h AH=11h check extended keystroke via asm", "[int][asm][kbd]") {
    IntTestEnv e;
    // No key initially
    e.assemble("MOV AH, 11h\nINT 16h");
    e.run(2);
    REQUIRE(e.cpu.getEFLAGS() & cpu::FLAG_ZERO); // ZF=1: no key

    // Now push a key and check again
    e.kbd.pushKey('!', 0x02);
    e.cpu.setEIP(IntTestEnv::CODE_OFF); // Reset IP
    e.assemble("MOV AH, 11h\nINT 16h");
    e.run(2);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_ZERO)); // ZF=0: key present
    REQUIRE(e.cpu.getReg8(cpu::AL) == '!');
    REQUIRE(e.cpu.getReg8(cpu::AH) == 0x02);
}

// ════════════════════════════════════════════════════════════════════════════
//  INT 15h AH=4Fh — Keyboard Intercept
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 15h AH=4Fh keyboard intercept via asm", "[int][asm][kbd]") {
    IntTestEnv e;
    e.assemble("MOV AH, 4Fh\nINT 15h");
    e.run(2);
    // Should return with carry set (key should be processed normally)
    REQUIRE(e.cpu.getEFLAGS() & cpu::FLAG_CARRY);
}

// ════════════════════════════════════════════════════════════════════════════
//  INT 16h — Keyboard buffer edge cases
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 16h AH=01h peek does not remove key via asm", "[int][asm][kbd]") {
    IntTestEnv e;
    e.kbd.pushKey('Q', 0x10);
    // Peek twice — should see the same key both times
    e.assemble(
        "MOV AH, 01h\n"
        "INT 16h\n"
        "MOV AH, 01h\n"
        "INT 16h"
    );
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_ZERO));
    REQUIRE(e.cpu.getReg8(cpu::AL) == 'Q');
    REQUIRE(e.cpu.getReg8(cpu::AH) == 0x10);
}

TEST_CASE("INT 16h AH=00h empties buffer after read via asm", "[int][asm][kbd]") {
    IntTestEnv e;
    e.kbd.pushKey('A', 0x1E);
    // Read with AH=00h — should get the key
    e.assemble("MOV AH, 00h\nINT 16h");
    e.run(2);
    REQUIRE(e.cpu.getReg8(cpu::AL) == 'A');
    REQUIRE(e.cpu.getReg8(cpu::AH) == 0x1E);
}

TEST_CASE("INT 16h AH=05h stores key with carry check via asm", "[int][asm][kbd]") {
    IntTestEnv e;
    // Store key, then read it back
    // CH=scancode(0x19), CL=ascii('P'=0x50) → CX=0x1950
    e.assemble(
        "MOV AH, 05h\n"
        "MOV CX, 1950h\n"  // CH=0x19, CL=0x50 ('P')
        "INT 16h\n"
        "MOV AH, 00h\n"
        "INT 16h"
    );
    e.run(5);
    REQUIRE(e.cpu.getReg8(cpu::AL) == 'P');
    REQUIRE(e.cpu.getReg8(cpu::AH) == 0x19);
}

// ════════════════════════════════════════════════════════════════════════════
//  Multi-instruction program: full int pipeline test
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("Full program: set mode, print string, terminate via asm", "[int][asm][integration]") {
    IntTestEnv e;
    // Put "OK$" at DS:0200
    e.writeDollarString(0x200, "OK");

    e.assemble(
        "MOV AX, 03h\n"      // INT 10h: Set mode 3
        "INT 10h\n"
        "MOV AH, 09h\n"      // INT 21h: Print "$"-string
        "MOV DX, 200h\n"
        "INT 21h\n"
        "MOV AH, 4Ch\n"      // INT 21h: Terminate
        "MOV AL, 0\n"
        "INT 21h"
    );
    e.run(8);

    // Mode should be 3
    REQUIRE(e.mem.read8(0x449) == 0x03);
    // "OK" should be in VRAM
    REQUIRE(e.mem.read8(0xB8000) == 'O');
    REQUIRE(e.mem.read8(0xB8002) == 'K');
    // Program terminated
    REQUIRE(e.dos.isTerminated());
    REQUIRE(e.dos.getExitCode() == 0);
}


// ════════════════════════════════════════════════════════════════════════════
//  DOS Interrupts: INT 25h, 26h, 27h, 28h, 29h, 2Ah, 2Eh, 2Fh (Extended)
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 25h Absolute Disk Read via asm", "[int][asm][dos][disk]") {
    IntTestEnv e;
    uint32_t bufAddr = (e.cpu.getSegReg(cpu::DS) << 4) + 0x300;
    for (int i = 0; i < 512; ++i) e.mem.write8(bufAddr + i, 0xAA);

    e.assemble(
        "MOV AL, 02h\n"
        "MOV CX, 01h\n"
        "MOV DX, 00h\n"
        "MOV BX, 300h\n"
        "INT 25h\n"
        "POPF"
    );
    e.run(6);

    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0);
    REQUIRE(e.mem.read8(bufAddr + 0) == 0);
    REQUIRE(e.mem.read8(bufAddr + 511) == 0);

    // Invalid drive
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AL, 50\n"
        "MOV CX, 01h\n"
        "INT 25h\n"
        "POP DX"
    );
    e.run(4);
    REQUIRE((e.cpu.getEFLAGS() & cpu::FLAG_CARRY) != 0);
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0x0201);
}

TEST_CASE("INT 26h Absolute Disk Write via asm", "[int][asm][dos][disk]") {
    IntTestEnv e;
    e.assemble(
        "MOV AL, 02h\n"
        "MOV CX, 01h\n"
        "MOV DX, 00h\n"
        "MOV BX, 300h\n"
        "INT 26h\n"
        "POPF"
    );
    e.run(6);

    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0);

    // Invalid drive
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AL, 50\n"
        "MOV CX, 01h\n"
        "INT 26h\n"
        "POP DX"
    );
    e.run(4);
    REQUIRE((e.cpu.getEFLAGS() & cpu::FLAG_CARRY) != 0);
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0x0201);
}

TEST_CASE("INT 27h TSR via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.assemble(
        "MOV DX, 100h\n"
        "INT 27h"
    );
    e.run(2);
    REQUIRE(e.dos.isTerminated());
    REQUIRE(e.dos.getExitCode() == 0);
}

TEST_CASE("INT 28h DOS Idle via asm", "[int][asm][dos]") {
    IntTestEnv e;
    bool idleInvoked = false;
    e.dos.setIdleCallback([&]() { idleInvoked = true; });

    e.assemble("INT 28h");
    e.run(1);
    REQUIRE(idleInvoked);
}

TEST_CASE("INT 29h Fast Console Output via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.mem.write8(0x450, 0); // Cursor col 0
    e.mem.write8(0x451, 0); // Cursor row 0
    e.mem.write16(0x44A, 80); // 80 cols

    e.assemble(
        "MOV AL, 57h\n" // 'W'
        "INT 29h"
    );
    e.run(2);

    REQUIRE(e.mem.read8(0xB8000) == 'W');
    REQUIRE(e.mem.read8(0x450) == 1);
}

TEST_CASE("INT 2Ah Network / Critical Section via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.assemble(
        "MOV AH, 00h\n"
        "INT 2Ah\n"
        "MOV AH, 80h\n"
        "INT 2Ah\n"
        "MOV AH, 81h\n"
        "INT 2Ah"
    );
    e.run(6);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg8(cpu::AL) == 0);
}

TEST_CASE("INT 2Eh Alternate Shell Exec via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.assemble("INT 2Eh");
    e.run(1);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
}

TEST_CASE("INT 2Fh Multiplex extended services via asm", "[int][asm]") {
    IntTestEnv e;
    // AX=1200h: DOS internal installation check
    e.assemble(
        "MOV AX, 1200h\n"
        "INT 2Fh"
    );
    e.run(2);
    REQUIRE(e.cpu.getReg8(cpu::AL) == 0xFF);

    // AX=1600h: Windows check
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AX, 1600h\n"
        "INT 2Fh"
    );
    e.run(2);
    REQUIRE(e.cpu.getReg8(cpu::AL) == 0x00);

    // AX=4A01h: HMA query
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AX, 4A01h\n"
        "INT 2Fh"
    );
    e.run(2);
    REQUIRE((e.cpu.getEFLAGS() & cpu::FLAG_CARRY) != 0);
    REQUIRE(e.cpu.getReg16(cpu::BX) == 0);
}

// ════════════════════════════════════════════════════════════════════════════
//  INT 21h Subfunctions: STDAUX/STDPRN, DPB, Verify, True Version, IOCTL, File
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 21h AH=03h/04h/05h STDAUX and STDPRN via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.assemble(
        "MOV AH, 03h\n"
        "INT 21h\n"
        "MOV AH, 04h\n"
        "MOV DL, 41h\n"
        "INT 21h\n"
        "MOV AH, 05h\n"
        "MOV DL, 42h\n"
        "INT 21h"
    );
    e.run(7);
    REQUIRE(e.cpu.getReg8(cpu::AL) == 0);
}

TEST_CASE("INT 21h AH=1Bh/1Ch Get Allocation Info via asm", "[int][asm][dos]") {
    IntTestEnv e;
    // Default drive
    e.assemble(
        "MOV AH, 1Bh\n"
        "INT 21h"
    );
    e.run(2);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg8(cpu::AL) == 32);     // Sectors per cluster
    REQUIRE(e.cpu.getReg16(cpu::CX) == 512);   // Bytes per sector
    REQUIRE(e.cpu.getReg16(cpu::DX) == 65535); // Total clusters
    uint16_t ds = e.cpu.getSegReg(cpu::DS);
    uint16_t bx = e.cpu.getReg16(cpu::BX);
    uint32_t mediaPhys = (static_cast<uint32_t>(ds) << 4) + bx;
    REQUIRE(e.mem.read8(mediaPhys) == 0xF8);   // Media descriptor byte

    // Specific drive DL=3 (C:)
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 1Ch\n"
        "MOV DL, 03h\n"
        "INT 21h"
    );
    e.run(3);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg8(cpu::AL) == 32);

    // Invalid drive DL=50
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 1Ch\n"
        "MOV DL, 50\n"
        "INT 21h"
    );
    e.run(3);
    REQUIRE((e.cpu.getEFLAGS() & cpu::FLAG_CARRY) != 0);
    REQUIRE(e.cpu.getReg8(cpu::AL) == 0xFF);
}

TEST_CASE("INT 21h AH=1Fh/32h Get Drive Parameter Block via asm", "[int][asm][dos]") {
    IntTestEnv e;
    // Default drive DPB
    e.assemble(
        "MOV AH, 1Fh\n"
        "INT 21h"
    );
    e.run(2);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg8(cpu::AL) == 0x00);
    uint16_t ds = e.cpu.getSegReg(cpu::DS);
    uint16_t bx = e.cpu.getReg16(cpu::BX);
    uint32_t dpbPhys = (static_cast<uint32_t>(ds) << 4) + bx;
    REQUIRE(e.mem.read16(dpbPhys + 2) == 512); // Bytes per sector

    // Specific drive DL=3 (C:)
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 32h\n"
        "MOV DL, 03h\n"
        "INT 21h"
    );
    e.run(3);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg8(cpu::AL) == 0x00);
    ds = e.cpu.getSegReg(cpu::DS);
    bx = e.cpu.getReg16(cpu::BX);
    dpbPhys = (static_cast<uint32_t>(ds) << 4) + bx;
    REQUIRE(e.mem.read8(dpbPhys + 0) == 2); // Drive C: = 2

    // Invalid drive DL=99
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 32h\n"
        "MOV DL, 99\n"
        "INT 21h"
    );
    e.run(3);
    REQUIRE((e.cpu.getEFLAGS() & cpu::FLAG_CARRY) != 0);
    REQUIRE(e.cpu.getReg8(cpu::AL) == 0xFF);
}

TEST_CASE("INT 21h AH=2Eh/54h Set and Get Verify Flag via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.assemble(
        "MOV AH, 54h\n"
        "INT 21h"
    );
    e.run(2);
    REQUIRE(e.cpu.getReg8(cpu::AL) == 0x00);

    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 2Eh\n"
        "MOV AL, 01h\n"
        "INT 21h\n"
        "MOV AH, 54h\n"
        "INT 21h"
    );
    e.run(5);
    REQUIRE(e.cpu.getReg8(cpu::AL) == 0x01);
    REQUIRE(e.dos.getVerifyFlag() == true);

    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 2Eh\n"
        "MOV AL, 00h\n"
        "INT 21h\n"
        "MOV AH, 54h\n"
        "INT 21h"
    );
    e.run(5);
    REQUIRE(e.cpu.getReg8(cpu::AL) == 0x00);
    REQUIRE(e.dos.getVerifyFlag() == false);
}

TEST_CASE("INT 21h AX=3306h Get True DOS Version via asm", "[int][asm][dos]") {
    IntTestEnv e;
    e.assemble(
        "MOV AX, 3306h\n"
        "INT 21h"
    );
    e.run(2);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg8(cpu::BL) == 6);
    REQUIRE(e.cpu.getReg8(cpu::BH) == 22);
    REQUIRE(e.cpu.getReg8(cpu::DL) == 0);
    REQUIRE(e.cpu.getReg8(cpu::DH) == 0x10);
}

TEST_CASE("INT 21h AH=44h IOCTL Input/Output Status and Block Info via asm", "[int][asm][dos]") {
    IntTestEnv e;
    // AL=00h: Check STDAUX (BX=3) and STDPRN (BX=4)
    e.assemble(
        "MOV AH, 44h\n"
        "MOV AL, 00h\n"
        "MOV BX, 03h\n"
        "INT 21h"
    );
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE((e.cpu.getReg16(cpu::DX) & 0x80) != 0);

    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 44h\n"
        "MOV AL, 00h\n"
        "MOV BX, 04h\n"
        "INT 21h"
    );
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE((e.cpu.getReg16(cpu::DX) & 0x80) != 0);

    // AL=06h: Get input status for STDOUT (handle 1)
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 44h\n"
        "MOV AL, 06h\n"
        "MOV BX, 01h\n"
        "INT 21h"
    );
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));

    // AL=07h: Get output status for STDOUT (handle 1) -> ready (0xFF)
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 44h\n"
        "MOV AL, 07h\n"
        "MOV BX, 01h\n"
        "INT 21h"
    );
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg8(cpu::AL) == 0xFF);

    // AL=08h: Check block device removable
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 44h\n"
        "MOV AL, 08h\n"
        "MOV BL, 01h\n"
        "INT 21h"
    );
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0x0000); // Removable

    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 44h\n"
        "MOV AL, 08h\n"
        "MOV BL, 03h\n"
        "INT 21h"
    );
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0x0001); // Fixed

    // AL=09h: Check block device remote -> DX=0802h (local)
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 44h\n"
        "MOV AL, 09h\n"
        "MOV BL, 03h\n"
        "INT 21h"
    );
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg16(cpu::DX) == 0x0802);
}

TEST_CASE("INT 21h AH=56h Rename File via asm", "[int][asm][dos][file]") {
    IntTestEnv e;
    const char* oldName = "test_old_name.tmp";
    const char* newName = "test_new_name.tmp";
    std::filesystem::remove(oldName);
    std::filesystem::remove(newName);

    // Create old file
    e.writeString(0x200, oldName);
    e.assemble("MOV AH, 3Ch\nMOV CX, 0\nMOV DX, 200h\nINT 21h\nMOV BX, AX\nMOV AH, 3Eh\nINT 21h");
    e.run(7);
    REQUIRE(std::filesystem::exists(oldName));

    // Rename oldName -> newName via AH=56h (DS:DX=old, ES:DI=new)
    e.writeString(0x300, newName);
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 56h\n"
        "MOV DX, 200h\n"
        "MOV DI, 300h\n"
        "INT 21h"
    );
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(!std::filesystem::exists(oldName));
    REQUIRE(std::filesystem::exists(newName));

    // Rename again -> fails with CF=1, AX=0x02
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 56h\n"
        "MOV DX, 200h\n"
        "MOV DI, 300h\n"
        "INT 21h"
    );
    e.run(4);
    REQUIRE((e.cpu.getEFLAGS() & cpu::FLAG_CARRY) != 0);
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0x02);

    std::filesystem::remove(newName);
}

TEST_CASE("INT 21h AH=57h Get and Set File Date and Time via asm", "[int][asm][dos][file]") {
    IntTestEnv e;
    const char* fname = "test_datetime.tmp";
    std::filesystem::remove(fname);

    e.writeString(0x200, fname);
    e.assemble("MOV AH, 3Ch\nMOV CX, 0\nMOV DX, 200h\nINT 21h");
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    uint16_t handle = e.cpu.getReg16(cpu::AX);

    // Set custom time=0x1234, date=0x5678 (AL=01h)
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 57h\n"
        "MOV AL, 01h\n"
        "MOV CX, 1234h\n"
        "MOV DX, 5678h\n"
        "INT 21h"
    );
    e.cpu.setReg16(cpu::BX, handle);
    e.run(5);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));

    // Get time/date back (AL=00h)
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 57h\n"
        "MOV AL, 00h\n"
        "INT 21h"
    );
    e.cpu.setReg16(cpu::BX, handle);
    e.run(3);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg16(cpu::CX) == 0x1234);
    REQUIRE(e.cpu.getReg16(cpu::DX) == 0x5678);

    // Close file
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble("MOV AH, 3Eh\nINT 21h");
    e.cpu.setReg16(cpu::BX, handle);
    e.run(2);

    std::filesystem::remove(fname);
}

TEST_CASE("INT 21h AH=5Ah Create Temporary File via asm", "[int][asm][dos][file]") {
    IntTestEnv e;
    e.writeString(0x200, ".\\");
    e.assemble(
        "MOV AH, 5Ah\n"
        "MOV CX, 00h\n"
        "MOV DX, 200h\n"
        "INT 21h"
    );
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    uint16_t handle = e.cpu.getReg16(cpu::AX);
    REQUIRE(handle >= 5);

    uint32_t base = (e.cpu.getSegReg(cpu::DS) << 4) + 0x200;
    std::string genPath;
    for (int i = 0; i < 64; ++i) {
        char ch = static_cast<char>(e.mem.read8(base + i));
        if (ch == 0) break;
        genPath += ch;
    }
    REQUIRE(genPath.size() > 2);

    // Close handle
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble("MOV AH, 3Eh\nINT 21h");
    e.cpu.setReg16(cpu::BX, handle);
    e.run(2);

    std::filesystem::remove(genPath);
}

TEST_CASE("INT 21h AH=5Bh Create New File via asm", "[int][asm][dos][file]") {
    IntTestEnv e;
    const char* fname = "test_newfile.tmp";
    std::filesystem::remove(fname);

    e.writeString(0x200, fname);
    e.assemble(
        "MOV AH, 5Bh\n"
        "MOV CX, 00h\n"
        "MOV DX, 200h\n"
        "INT 21h"
    );
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    uint16_t handle = e.cpu.getReg16(cpu::AX);
    REQUIRE(handle >= 5);

    // Attempting create on existing file fails
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 5Bh\n"
        "MOV CX, 00h\n"
        "MOV DX, 200h\n"
        "INT 21h"
    );
    e.run(4);
    REQUIRE((e.cpu.getEFLAGS() & cpu::FLAG_CARRY) != 0);
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0x50);

    // Close handle
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble("MOV AH, 3Eh\nINT 21h");
    e.cpu.setReg16(cpu::BX, handle);
    e.run(2);

    std::filesystem::remove(fname);
}

TEST_CASE("INT 21h AH=68h Commit File via asm", "[int][asm][dos][file]") {
    IntTestEnv e;
    const char* fname = "test_commit.tmp";
    std::filesystem::remove(fname);

    e.writeString(0x200, fname);
    e.assemble(
        "MOV AH, 3Ch\n"
        "MOV CX, 00h\n"
        "MOV DX, 200h\n"
        "INT 21h"
    );
    e.run(4);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    uint16_t handle = e.cpu.getReg16(cpu::AX);

    // Commit file
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 68h\n"
        "INT 21h"
    );
    e.cpu.setReg16(cpu::BX, handle);
    e.run(2);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0);

    // Commit invalid handle
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble(
        "MOV AH, 68h\n"
        "MOV BX, 99\n"
        "INT 21h"
    );
    e.run(3);
    REQUIRE((e.cpu.getEFLAGS() & cpu::FLAG_CARRY) != 0);
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0x06);

    // Close handle
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    e.assemble("MOV AH, 3Eh\nINT 21h");
    e.cpu.setReg16(cpu::BX, handle);
    e.run(2);

    std::filesystem::remove(fname);
}

// ════════════════════════════════════════════════════════════════════════════
//  DPMI Services via Assembler & InstructionDecoder
// ════════════════════════════════════════════════════════════════════════════

TEST_CASE("INT 2Fh AX=1687h DPMI Installation Query via asm", "[int][asm][dpmi]") {
    IntTestEnv e;
    e.assemble(
        "MOV AX, 1687h\n"
        "INT 2Fh"
    );
    e.run(2);
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0x0000);  // DPMI available
    REQUIRE((e.cpu.getReg16(cpu::BX) & 1) != 0); // 32-bit clients supported
    REQUIRE(e.cpu.getReg8(cpu::CL) == 0x03);     // 386 processor
    REQUIRE(e.cpu.getReg8(cpu::DH) == 0x00);     // Major version 0
    REQUIRE(e.cpu.getReg8(cpu::DL) == 0x09);     // Minor version 9
    REQUIRE(e.cpu.getReg16(cpu::SI) == 0x0000);  // 0 paras private data
    REQUIRE(e.cpu.getSegReg(cpu::ES) == 0xF000); // Entry point segment
    REQUIRE(e.cpu.getReg16(cpu::DI) == 0x0050);  // Entry point offset
}

TEST_CASE("INT 2Fh AX=1686h Get CPU Mode via asm", "[int][asm][dpmi]") {
    IntTestEnv e;
    // In real mode
    e.assemble(
        "MOV AX, 1686h\n"
        "INT 2Fh"
    );
    e.run(2);
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0x0000); // 0 = Real Mode
}

TEST_CASE("DPMI entry and protected mode execution via asm", "[int][asm][dpmi]") {
    IntTestEnv e;
    REQUIRE(!e.dpmi.isActive());
    REQUIRE((e.cpu.getCR(0) & 1) == 0); // Real mode

    // 1. Enter PM via CALL 0F000h:0050h
    // 2. Execute MOV DX, 55AAh in PM
    // 3. Query CPU mode via INT 2Fh AX=1686h in PM
    e.assemble(
        "MOV AX, 0001h\n"
        "CALL 0F000h:0050h\n"
        "MOV DX, 55AAh\n"
        "MOV AX, 1686h\n"
        "INT 2Fh"
    );
    // 1: MOV AX, 1
    // 2: CALL 0F000h:0050h (jumps to F000:0050)
    // 3: 0F FF E1 at F000:0050 (HLE trap switches to PM and returns)
    // 4: MOV DX, 55AAh
    // 5: MOV AX, 1686h
    // 6: INT 2Fh
    e.run(6);

    REQUIRE(e.dpmi.isActive());
    REQUIRE((e.cpu.getCR(0) & 1) != 0); // Protected mode active!
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg16(cpu::DX) == 0x55AA);
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0x0001); // 1 = Protected Mode!
}

TEST_CASE("INT 31h AX=0003h Get Selector Increment via asm in PM", "[int][asm][dpmi]") {
    IntTestEnv e;
    e.assemble(
        "MOV AX, 0001h\n"
        "CALL 0F000h:0050h\n"
        "MOV AX, 0003h\n"
        "INT 31h"
    );
    e.run(5);
    REQUIRE(e.dpmi.isActive());
    REQUIRE((e.cpu.getCR(0) & 1) != 0);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg16(cpu::AX) == 0x0008); // Selector increment = 8
}

TEST_CASE("INT 31h AX=0000h/0001h Allocate and Free Descriptors via asm in PM", "[int][asm][dpmi]") {
    IntTestEnv e;
    e.assemble(
        "MOV AX, 0001h\n"
        "CALL 0F000h:0050h\n"
        "MOV AX, 0000h\n"
        "MOV CX, 0001h\n"
        "INT 31h\n"
        "MOV BX, AX\n"
        "MOV AX, 0001h\n"
        "INT 31h"
    );
    // 1: MOV AX, 1
    // 2: CALL
    // 3: DPMI entry HLE
    // 4: MOV AX, 0000h
    // 5: MOV CX, 0001h
    // 6: INT 31h (alloc descriptor)
    // 7: MOV BX, AX
    // 8: MOV AX, 0001h
    // 9: INT 31h (free descriptor)
    e.run(9);
    REQUIRE(e.dpmi.isActive());
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    // Check that allocated selector was an LDT selector (bit 2 set) with RPL=3
    uint16_t sel = e.cpu.getReg16(cpu::BX);
    REQUIRE((sel & 0x04) != 0);
    REQUIRE((sel & 0x03) == 3);
}

TEST_CASE("INT 31h AX=0200h/0201h Get/Set RM Interrupt Vector via asm in PM", "[int][asm][dpmi]") {
    IntTestEnv e;
    e.assemble(
        "MOV AX, 0001h\n"
        "CALL 0F000h:0050h\n"
        // Set vector 65h to 1234h:5678h
        "MOV AX, 0201h\n"
        "MOV BL, 65h\n"
        "MOV CX, 1234h\n"
        "MOV DX, 5678h\n"
        "INT 31h\n"
        // Read it back
        "MOV AX, 0200h\n"
        "MOV BL, 65h\n"
        "INT 31h"
    );
    e.run(11);
    REQUIRE(e.dpmi.isActive());
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg16(cpu::CX) == 0x1234);
    REQUIRE(e.cpu.getReg16(cpu::DX) == 0x5678);
}

TEST_CASE("INT 31h AX=0501h/0502h Extended Memory Allocate and Free via asm in PM", "[int][asm][dpmi]") {
    IntTestEnv e;
    e.assemble(
        "MOV AX, 0001h\n"
        "CALL 0F000h:0050h\n"
        // Allocate 64KB block (BX:CX = 0x0001:0x0000)
        "MOV AX, 0501h\n"
        "MOV BX, 0001h\n"
        "MOV CX, 0000h\n"
        "INT 31h\n"
        // Free block using handle in SI:DI
        "MOV AX, 0502h\n"
        "INT 31h"
    );
    e.run(8);
    REQUIRE(e.dpmi.isActive());
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
}

TEST_CASE("INT 31h AX=0300h Simulate RM Interrupt via asm in PM", "[int][asm][dpmi]") {
    IntTestEnv e;
    // Enter PM first
    e.assemble(
        "MOV AX, 0001h\n"
        "CALL 0F000h:0050h\n"
        "NOP"
    );
    e.run(4);
    REQUIRE(e.dpmi.isActive());

    // Prepare Real Mode Call Structure at DS:0x200 (linear = DS.base + 0x200)
    uint32_t dsBase = e.cpu.getSegBase(cpu::DS);
    uint32_t structAddr = dsBase + 0x200;
    for (int i = 0; i < 50; ++i) {
        e.mem.write8(structAddr + i, 0);
    }
    // Set EAX in call structure (offset 0x1C) = 0x3000 (INT 21h AH=30h Get DOS Version)
    e.mem.write32(structAddr + 0x1C, 0x00003000);

    // Now execute INT 31h AX=0300h BL=21h ES:DI=DS:0200h
    e.cpu.setEIP(IntTestEnv::CODE_OFF);
    // Point ES to DS so ES:DI points to the call structure
    e.cpu.setSegReg(cpu::ES, e.cpu.getSegReg(cpu::DS));
    e.decoder.loadSegment(cpu::ES, e.cpu.getSegReg(cpu::DS));

    e.assemble(
        "MOV AX, 0300h\n"
        "MOV BL, 21h\n"
        "MOV BH, 00h\n"
        "MOV CX, 0000h\n"
        "MOV DI, 0200h\n"
        "INT 31h"
    );
    e.run(6);
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));

    // Verify DOS version in the call structure (offset 0x1C = EAX)
    // AL = 5 (major), AH = 0 (minor) -> AX = 0x0005
    uint32_t retEAX = e.mem.read32(structAddr + 0x1C);
    REQUIRE((retEAX & 0xFF) == 6); // DOS 6.22
    REQUIRE(((retEAX >> 8) & 0xFF) == 22);
}

TEST_CASE("INT 31h AX=0006h/0007h/0008h Base and Limit via asm in PM", "[int][asm][dpmi]") {
    IntTestEnv e;
    e.assemble(
        "MOV AX, 0001h\n"
        "CALL 0F000h:0050h\n"
        // 1. Allocate 1 descriptor
        "MOV AX, 0000h\n"
        "MOV CX, 0001h\n"
        "INT 31h\n"
        "MOV BX, AX\n"          // Save selector in BX
        // 2. Set base address to 0x00025000 (CX:DX = 0x0002:0x5000)
        "MOV AX, 0007h\n"
        "MOV CX, 0002h\n"
        "MOV DX, 5000h\n"
        "INT 31h\n"
        // 3. Set segment limit to 0x0FFF (CX:DX = 0x0000:0x0FFF)
        "MOV AX, 0008h\n"
        "MOV CX, 0000h\n"
        "MOV DX, 0FFFh\n"
        "INT 31h\n"
        // 4. Read base back
        "MOV AX, 0006h\n"
        "INT 31h\n"
        // 5. Free descriptor
        "MOV AX, 0001h\n"
        "INT 31h"
    );
    e.run(18);
    REQUIRE(e.dpmi.isActive());
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg16(cpu::CX) == 0x0002);
    REQUIRE(e.cpu.getReg16(cpu::DX) == 0x5000);
}

TEST_CASE("INT 31h AX=0100h/0101h DOS Memory Block via asm in PM", "[int][asm][dpmi]") {
    IntTestEnv e;
    e.assemble(
        "MOV AX, 0001h\n"
        "CALL 0F000h:0050h\n"
        // Allocate 16 paragraphs (256 bytes) of DOS memory
        "MOV AX, 0100h\n"
        "MOV BX, 0010h\n"
        "INT 31h\n"
        // Free DOS memory using selector in DX
        "MOV AX, 0101h\n"
        "INT 31h"
    );
    e.run(8);
    REQUIRE(e.dpmi.isActive());
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
}

TEST_CASE("INT 31h AX=0400h Get Version via asm in PM", "[int][asm][dpmi]") {
    IntTestEnv e;
    e.assemble(
        "MOV AX, 0001h\n"
        "CALL 0F000h:0050h\n"
        "MOV AX, 0400h\n"
        "INT 31h"
    );
    e.run(5);
    REQUIRE(e.dpmi.isActive());
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg8(cpu::AH) == 0);       // Major version 0
    REQUIRE(e.cpu.getReg8(cpu::AL) == 9);       // Minor version 9
    REQUIRE((e.cpu.getReg16(cpu::BX) & 1) != 0);// 32-bit client support
    REQUIRE(e.cpu.getReg8(cpu::CL) == 4);       // 486 processor
}

TEST_CASE("INT 31h AX=0604h Get Page Size via asm in PM", "[int][asm][dpmi]") {
    IntTestEnv e;
    e.assemble(
        "MOV AX, 0001h\n"
        "CALL 0F000h:0050h\n"
        "MOV AX, 0604h\n"
        "INT 31h"
    );
    e.run(5);
    REQUIRE(e.dpmi.isActive());
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg16(cpu::BX) == 0);
    REQUIRE(e.cpu.getReg16(cpu::CX) == 4096);
}

TEST_CASE("INT 31h AX=0900h/0901h/0902h Virtual Interrupt State via asm in PM", "[int][asm][dpmi]") {
    IntTestEnv e;
    e.assemble(
        "MOV AX, 0001h\n"
        "CALL 0F000h:0050h\n"
        // 1. Disable virtual interrupts (AX=0900h)
        "MOV AX, 0900h\n"
        "INT 31h\n"
        // 2. Query virtual interrupts (AX=0902h) -> should be 0
        "MOV AX, 0902h\n"
        "INT 31h\n"
        "MOV BL, AL\n"
        // 3. Enable virtual interrupts (AX=0901h)
        "MOV AX, 0901h\n"
        "INT 31h\n"
        // 4. Query virtual interrupts (AX=0902h) -> should be 1
        "MOV AX, 0902h\n"
        "INT 31h\n"
        "MOV BH, AL"
    );
    e.run(13);
    REQUIRE(e.dpmi.isActive());
    REQUIRE(!(e.cpu.getEFLAGS() & cpu::FLAG_CARRY));
    REQUIRE(e.cpu.getReg8(cpu::BL) == 0); // was disabled
    REQUIRE(e.cpu.getReg8(cpu::BH) == 1); // was enabled
}
