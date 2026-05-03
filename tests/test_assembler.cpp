#include "test_framework.hpp"
#include "cpu/Assembler.hpp"
#include <sstream>
#include <iomanip>

using namespace fador::cpu;

static std::string hexStr(const std::vector<uint8_t>& v) {
    std::ostringstream os;
    for (size_t i = 0; i < v.size(); ++i) {
        if (i) os << ' ';
        os << std::hex << std::setw(2) << std::setfill('0') << (int)v[i];
    }
    return os.str();
}

// ── Helper: assemble one line and verify bytes ──
static void expectBytes(const std::string& line, const std::vector<uint8_t>& expected,
                        uint32_t origin = 0x100) {
    Assembler a;
    auto r = a.assembleLine(line, origin);
    if (!r.error.empty()) {
        throw std::runtime_error("Assembly failed for '" + line + "': " + r.error);
    }
    if (r.bytes != expected) {
        throw std::runtime_error("Mismatch for '" + line + "': got " + hexStr(r.bytes)
                                 + " expected " + hexStr(expected));
    }
}

// ============================================================================
//  No-operand instructions
// ============================================================================

TEST_CASE("Assembler - no-operand instructions", "[assembler]") {
    expectBytes("NOP",   {0x90});
    expectBytes("HLT",   {0xF4});
    expectBytes("CLC",   {0xF8});
    expectBytes("STC",   {0xF9});
    expectBytes("CLI",   {0xFA});
    expectBytes("STI",   {0xFB});
    expectBytes("CLD",   {0xFC});
    expectBytes("STD",   {0xFD});
    expectBytes("CMC",   {0xF5});
    expectBytes("SAHF",  {0x9E});
    expectBytes("LAHF",  {0x9F});
    expectBytes("CBW",   {0x98});
    expectBytes("CWD",   {0x99});
    expectBytes("PUSHA", {0x60});
    expectBytes("POPA",  {0x61});
    expectBytes("PUSHF", {0x9C});
    expectBytes("POPF",  {0x9D});
    expectBytes("IRET",  {0xCF});
    expectBytes("LEAVE", {0xC9});
    expectBytes("MOVSB", {0xA4});
    expectBytes("MOVSW", {0xA5});
    expectBytes("STOSB", {0xAA});
    expectBytes("STOSW", {0xAB});
    expectBytes("LODSB", {0xAC});
    expectBytes("LODSW", {0xAD});
    expectBytes("CMPSB", {0xA6});
    expectBytes("CMPSW", {0xA7});
    expectBytes("SCASB", {0xAE});
    expectBytes("SCASW", {0xAF});
    expectBytes("AAA",   {0x37});
    expectBytes("AAS",   {0x3F});
    expectBytes("DAA",   {0x27});
    expectBytes("DAS",   {0x2F});
    expectBytes("AAM",   {0xD4, 0x0A});
    expectBytes("AAD",   {0xD5, 0x0A});
    expectBytes("INTO",  {0xCE});
}

// ============================================================================
//  MOV
// ============================================================================

TEST_CASE("Assembler - MOV register immediate", "[assembler]") {
    expectBytes("MOV AL, 42h",   {0xB0, 0x42});
    expectBytes("MOV BL, 0FFh",  {0xB3, 0xFF});
    expectBytes("MOV AX, 1234h", {0xB8, 0x34, 0x12});
    expectBytes("MOV CX, 0ABCDh",{0xB9, 0xCD, 0xAB});
    expectBytes("MOV DX, 0",     {0xBA, 0x00, 0x00});
    expectBytes("MOV SP, 0FFFEh",{0xBC, 0xFE, 0xFF});
}

TEST_CASE("Assembler - MOV register-register", "[assembler]") {
    // MOV AX, BX → 0x89 D8 (r/m16, r16 form)
    expectBytes("MOV AX, BX",   {0x89, 0xD8});
    expectBytes("MOV AL, BL",   {0x88, 0xD8});
}

TEST_CASE("Assembler - MOV segment registers", "[assembler]") {
    // MOV DS, AX → 0x8E D8
    expectBytes("MOV DS, AX", {0x8E, 0xD8});
    // MOV AX, DS → 0x8C D8
    expectBytes("MOV AX, DS", {0x8C, 0xD8});
}

TEST_CASE("Assembler - MOV memory", "[assembler]") {
    // MOV [BX], AX → 0x89, 0x07
    expectBytes("MOV [BX], AX",  {0x89, 0x07});
    // MOV AX, [SI] → 0x8B, 0x04
    expectBytes("MOV AX, [SI]",  {0x8B, 0x04});
    // MOV BYTE PTR [BX], 42h → C6 07 42
    expectBytes("MOV BYTE [BX], 42h", {0xC6, 0x07, 0x42});
    // MOV WORD [BX], 1234h → C7 07 34 12
    expectBytes("MOV WORD [BX], 1234h", {0xC7, 0x07, 0x34, 0x12});
}

// ============================================================================
//  ALU: ADD, SUB, AND, OR, XOR, CMP, ADC, SBB
// ============================================================================

TEST_CASE("Assembler - ALU AL/AX short forms", "[assembler]") {
    expectBytes("ADD AL, 5",   {0x04, 0x05});
    expectBytes("ADD AX, 1000h", {0x05, 0x00, 0x10});
    expectBytes("SUB AL, 1",   {0x2C, 0x01});
    expectBytes("CMP AL, 0",   {0x3C, 0x00});
    expectBytes("AND AL, 0Fh", {0x24, 0x0F});
    expectBytes("OR  AL, 80h", {0x0C, 0x80});
    expectBytes("XOR AX, AX",  {0x31, 0xC0}); // r/m16, r16 form (both valid)
}

TEST_CASE("Assembler - ALU register-register", "[assembler]") {
    expectBytes("ADD BX, CX", {0x01, 0xCB}); // r/m16, r16 form
    expectBytes("SUB AX, DX", {0x29, 0xD0});
    expectBytes("AND CX, BX", {0x21, 0xD9});
}

TEST_CASE("Assembler - ALU register-immediate sign-ext", "[assembler]") {
    // ADD BX, 5 → 0x83 0xC3 0x05 (sign-extended imm8)
    expectBytes("ADD BX, 5", {0x83, 0xC3, 0x05});
    // SUB CX, 1000h → 0x81 0xE9 0x00 0x10 (full imm16)
    expectBytes("SUB CX, 1000h", {0x81, 0xE9, 0x00, 0x10});
}

// ============================================================================
//  PUSH / POP
// ============================================================================

TEST_CASE("Assembler - PUSH/POP", "[assembler]") {
    expectBytes("PUSH AX", {0x50});
    expectBytes("PUSH BX", {0x53});
    expectBytes("PUSH DI", {0x57});
    expectBytes("POP  AX", {0x58});
    expectBytes("POP  BP", {0x5D});
    expectBytes("PUSH ES", {0x06});
    expectBytes("PUSH DS", {0x1E});
    expectBytes("POP  DS", {0x1F});
    expectBytes("POP  ES", {0x07});
    expectBytes("PUSH 42h", {0x6A, 0x42}); // imm8 sign-ext
    expectBytes("PUSH 1234h", {0x68, 0x34, 0x12}); // imm16
}

// ============================================================================
//  INC / DEC
// ============================================================================

TEST_CASE("Assembler - INC/DEC", "[assembler]") {
    expectBytes("INC AX", {0x40});
    expectBytes("INC CX", {0x41});
    expectBytes("INC DI", {0x47});
    expectBytes("DEC AX", {0x48});
    expectBytes("DEC SI", {0x4E});
    expectBytes("INC AL", {0xFE, 0xC0});
    expectBytes("DEC BL", {0xFE, 0xCB});
}

// ============================================================================
//  Shifts / Rotates
// ============================================================================

TEST_CASE("Assembler - Shifts", "[assembler]") {
    expectBytes("SHL AX, 1",  {0xD1, 0xE0});
    expectBytes("SHR BX, 1",  {0xD1, 0xEB});
    expectBytes("SAR CX, CL", {0xD3, 0xF9});
    expectBytes("SHL DX, 4",  {0xC1, 0xE2, 0x04});
    expectBytes("ROL AL, 1",  {0xD0, 0xC0});
    expectBytes("ROR BL, CL", {0xD2, 0xCB});
}

// ============================================================================
//  NOT / NEG / MUL / DIV
// ============================================================================

TEST_CASE("Assembler - NOT/NEG", "[assembler]") {
    expectBytes("NOT AX", {0xF7, 0xD0});
    expectBytes("NEG BX", {0xF7, 0xDB});
    expectBytes("NOT AL", {0xF6, 0xD0});
    expectBytes("NEG CL", {0xF6, 0xD9});
}

TEST_CASE("Assembler - MUL/DIV", "[assembler]") {
    expectBytes("MUL BX",  {0xF7, 0xE3});
    expectBytes("DIV CX",  {0xF7, 0xF1});
    expectBytes("MUL BL",  {0xF6, 0xE3});
    expectBytes("IDIV DX", {0xF7, 0xFA});
}

TEST_CASE("Assembler - x87 env/state save-restore", "[assembler]") {
    expectBytes("FNSTENV [3000h]", {0xD9, 0x36, 0x00, 0x30});
    expectBytes("FLDENV [3000h]",  {0xD9, 0x26, 0x00, 0x30});
    expectBytes("FNSAVE [3100h]",  {0xDD, 0x36, 0x00, 0x31});
    expectBytes("FRSTOR [3100h]",  {0xDD, 0x26, 0x00, 0x31});
    expectBytes("FNSTENV DWORD [3000h]", {0x66, 0xD9, 0x36, 0x00, 0x30});
    expectBytes("FLDENV DWORD [3000h]",  {0x66, 0xD9, 0x26, 0x00, 0x30});
    expectBytes("FNSAVE DWORD [3100h]",  {0x66, 0xDD, 0x36, 0x00, 0x31});
    expectBytes("FRSTOR DWORD [3100h]",  {0x66, 0xDD, 0x26, 0x00, 0x31});
}

// ============================================================================
//  Jumps
// ============================================================================

TEST_CASE("Assembler - JMP/CALL short", "[assembler]") {
    // JMP to self (origin=0x100, target=0x100 → disp = 0x100 - 0x102 = -2 = 0xFE)
    expectBytes("JMP 100h", {0xEB, 0xFE}, 0x100);
    // JMP forward short (origin=0x100, target=0x110 → disp = 0x110 - 0x102 = 14 = 0x0E)
    expectBytes("JMP 110h", {0xEB, 0x0E}, 0x100);
}

TEST_CASE("Assembler - Jcc", "[assembler]") {
    // JZ 0x110 from origin 0x100 → disp = 0x110 - 0x102 = 0x0E
    expectBytes("JZ 110h",  {0x74, 0x0E}, 0x100);
    expectBytes("JNZ 110h", {0x75, 0x0E}, 0x100);
    expectBytes("JC 110h",  {0x72, 0x0E}, 0x100);
    expectBytes("JNC 110h", {0x73, 0x0E}, 0x100);
}

TEST_CASE("Assembler - CALL near", "[assembler]") {
    // CALL 200h from origin 0x100 → disp = 0x200 - 0x103 = 0xFD → E8 FD 00
    expectBytes("CALL 200h", {0xE8, 0xFD, 0x00}, 0x100);
}

TEST_CASE("Assembler - RET/RETF", "[assembler]") {
    expectBytes("RET",    {0xC3});
    expectBytes("RET 4",  {0xC2, 0x04, 0x00});
    expectBytes("RETF",   {0xCB});
    expectBytes("RETF 2", {0xCA, 0x02, 0x00});
}

// ============================================================================
//  INT
// ============================================================================

TEST_CASE("Assembler - INT", "[assembler]") {
    expectBytes("INT 21h", {0xCD, 0x21});
    expectBytes("INT 3",   {0xCC});
    expectBytes("INT 10h", {0xCD, 0x10});
}

// ============================================================================
//  XCHG
// ============================================================================

TEST_CASE("Assembler - XCHG", "[assembler]") {
    expectBytes("XCHG AX, BX", {0x93}); // short form
    expectBytes("XCHG AX, CX", {0x91});
    expectBytes("XCHG BX, AX", {0x93});
}

// ============================================================================
//  LEA
// ============================================================================

TEST_CASE("Assembler - LEA", "[assembler]") {
    // LEA AX, [BX+SI] → 8D 00
    expectBytes("LEA AX, [BX+SI]", {0x8D, 0x00});
    // LEA DX, [BP+4] → 8D 56 04
    expectBytes("LEA DX, [BP+4]",  {0x8D, 0x56, 0x04});
}

// ============================================================================
//  TEST
// ============================================================================

TEST_CASE("Assembler - TEST", "[assembler]") {
    expectBytes("TEST AL, 1",    {0xA8, 0x01});
    expectBytes("TEST AX, 0FFh", {0xA9, 0xFF, 0x00});
}

// ============================================================================
//  IN / OUT
// ============================================================================

TEST_CASE("Assembler - IN/OUT", "[assembler]") {
    expectBytes("IN  AL, 60h",  {0xE4, 0x60});
    expectBytes("IN  AX, 20h",  {0xE5, 0x20});
    expectBytes("IN  AL, DX",   {0xEC});
    expectBytes("IN  AX, DX",   {0xED});
    expectBytes("OUT 60h, AL",  {0xE6, 0x60});
    expectBytes("OUT DX, AL",   {0xEE});
    expectBytes("OUT DX, AX",   {0xEF});
}

// ============================================================================
//  REP prefix
// ============================================================================

TEST_CASE("Assembler - REP prefix", "[assembler]") {
    expectBytes("REP MOVSB",  {0xF3, 0xA4});
    expectBytes("REP STOSW",  {0xF3, 0xAB});
    expectBytes("REPNE SCASB",{0xF2, 0xAE});
}

// ============================================================================
//  Segment override
// ============================================================================

TEST_CASE("Assembler - segment override", "[assembler]") {
    // MOV AX, ES:[BX] → 26 8B 07
    expectBytes("MOV AX, ES:[BX]", {0x26, 0x8B, 0x07});
}

// ============================================================================
//  ENTER
// ============================================================================

TEST_CASE("Assembler - ENTER", "[assembler]") {
    expectBytes("ENTER 8, 0", {0xC8, 0x08, 0x00, 0x00});
}

// ============================================================================
//  Multi-line assembleBlock
// ============================================================================

TEST_CASE("Assembler - assembleBlock multi-line", "[assembler]") {
    Assembler a;
    auto r = a.assembleBlock("MOV AX, 1234h\nMOV BX, 5678h\nINT 21h", 0x100);
    REQUIRE(r.error.empty());
    // MOV AX,1234h = B8 34 12 (3 bytes)
    // MOV BX,5678h = BB 78 56 (3 bytes)
    // INT 21h      = CD 21    (2 bytes)
    REQUIRE(r.bytes.size() == 8);
    REQUIRE(r.bytes[0] == 0xB8);
    REQUIRE(r.bytes[3] == 0xBB);
    REQUIRE(r.bytes[6] == 0xCD);
    REQUIRE(r.bytes[7] == 0x21);
}

// ============================================================================
//  Error handling
// ============================================================================

TEST_CASE("Assembler - unknown mnemonic error", "[assembler]") {
    Assembler a;
    auto r = a.assembleLine("FOOBAR", 0x100);
    REQUIRE(!r.error.empty());
}

TEST_CASE("Assembler - empty line returns no bytes", "[assembler]") {
    Assembler a;
    auto r = a.assembleLine("", 0x100);
    REQUIRE(r.error.empty());
    REQUIRE(r.bytes.empty());
}

// ============================================================================
//  0F-prefix: MOVZX/MOVSX
// ============================================================================

TEST_CASE("Assembler - MOVZX/MOVSX", "[assembler]") {
    // MOVZX AX, BL → 0F B6 C3
    expectBytes("MOVZX AX, BL", {0x0F, 0xB6, 0xC3});
    // MOVSX CX, DL → 0F BE CA
    expectBytes("MOVSX CX, DL", {0x0F, 0xBE, 0xCA});
}

TEST_CASE("Assembler - BSWAP", "[assembler]") {
    expectBytes("BSWAP EAX", {0x0F, 0xC8});
}

// ============================================================================
//  BT family
// ============================================================================

TEST_CASE("Assembler - BT/BTS/BTR/BTC", "[assembler]") {
    // BT AX, 5 → 0F BA E0 05
    expectBytes("BT AX, 5",   {0x0F, 0xBA, 0xE0, 0x05});
    // BTS BX, CX → 0F AB CB
    expectBytes("BTS BX, CX", {0x0F, 0xAB, 0xCB});
}

// ============================================================================
//  LES / LDS
// ============================================================================

TEST_CASE("Assembler - LES/LDS", "[assembler]") {
    // LES AX, [BX] → C4 07
    expectBytes("LES AX, [BX]", {0xC4, 0x07});
    // LDS DX, [SI] → C5 14
    expectBytes("LDS DX, [SI]", {0xC5, 0x14});
}
