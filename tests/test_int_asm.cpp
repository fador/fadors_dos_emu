// test_int_asm.cpp – assembler-driven integration tests for interrupt handlers.
// Each test assembles real x86 instructions, writes them into memory, and runs
// them through the full InstructionDecoder pipeline.

#include "test_framework.hpp"
#include <cstdio>
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
    cpu::InstructionDecoder decoder;
    cpu::Assembler assembler;

    static constexpr uint16_t CODE_SEG = 0x1000;
    static constexpr uint16_t CODE_OFF = 0x100;
    static constexpr uint32_t CODE_LINEAR = (CODE_SEG << 4) + CODE_OFF;

    IntTestEnv()
        : dos(cpu, mem), bios(cpu, mem, kbd, pit, pic),
          decoder(cpu, mem, iobus, bios, dos)
    {
        bios.initialize();
        dos.initialize();
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
    REQUIRE((major == 3 || major == 5));
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
