# AI Agent Instructions for Fador's DOS Emulator

Welcome. You are assisting in building Fador's DOS Emulator.

## Core Directives
1. **Language & Standard:** Write modern C++20. Use `<concepts>`, `<ranges>`, `std::span`, `std::unique_ptr`, `constexpr`, etc. Avoid raw C-style arrays where `std::array` or `std::vector` fits better.
2. **Architecture:** The emulator is strictly decoupled. The CPU, Memory, and Hardware Devices must NOT depend on any UI backend. UI (Console/Text mode vs Graphical) must be implementations of a standard interchangeable interface.
3. **Reference Material:** The `docs/` folder contains comprehensive documentation (Ralph Brown's interrupt list).
   - `INTERRUP.*`: Interrupt definitions
   - `PORTS.*`: I/O port definitions
   - `OPCODES.LST`: Intel opcodes
   *Always consult these files when implementing an interrupt or port handler.*
4. **Development Strategy:**
   - Prioritize 16-bit real mode and core 386 instructions first.
   - Bootstrapping: Start by successfully loading and running simple DOS `.COM` files.
   - Memory Model: Implement a flat 1MB 8-bit byte array for conventional memory routing first.
   - Testing: Write unit tests for your CPU instruction implementations.
5. **Memory Initialisation:** Conventional memory above the BDA (0x500–0x9FFFF) is filled with a deterministic pseudo-random pattern (LCG: `val = val * 141 + 3`, seed 0xCC) in `MemoryBus::MemoryBus()` to simulate dirty DOS RAM. IVT/BDA (0x000–0x4FF) remains zero-filled so BIOS `initialize()` can write proper values on top. The EXE/COM loader then writes image data over the fill. **Do not change this initialisation to all-zeros or all-0xCC** — programs compiled with Borland's mixed-model Turbo Vision (e.g. TC 2.01) read uninitialised far-heap memory: all-zeros causes false end-of-table sentinel matches; all-0xCC causes buffer overflows from unterminated strings. The LCG pattern provides ~255 non-zero bytes between natural zeros, avoiding both failure modes.
6. **Build System:** Maintain the `CMakeLists.txt`. Ensure it compiles consistently natively on all target platforms (Windows, Linux, macOS) without relying on OS-specific extensions where C++ standard library equivalents exist.
7. **Built-in Disassembler:** A read-only x86 real-mode disassembler lives in `src/cpu/Disassembler.{hpp,cpp}`. It reads bytes from `MemoryBus` and produces `DisassembledInstruction` structs (address, length, mnemonic, hex bytes) without modifying any state. Use `disassembleAt()`, `disassembleRange()`, or `disassembleAround()` to inspect code. When adding new opcodes to `InstructionDecoder`, mirror the decoding in the disassembler's `disasmOpcode()` / `disasmOpcode0F()` methods.
8. **Debugger & Cycle Stop:** The interactive debugger (`src/ui/Debugger.{hpp,cpp}`) provides single-step, register dump, memory dump, disassembly, and inline assembly commands. The `--stop-after=N` CLI flag halts execution after N instruction cycles and calls `Debugger::dumpState()` to print registers and surrounding disassembly. The `--exec="asm"` flag assembles and executes instructions without loading a program.
9. **Built-in Assembler:** A real-mode x86 assembler lives in `src/cpu/Assembler.{hpp,cpp}`. It converts assembly text into machine-code bytes via `assembleLine()` (single instruction) or `assembleBlock()` (multi-line). The assembler supports the same instruction subset as `InstructionDecoder`: ALU, MOV, shifts, jumps (short/near), Jcc, CALL/RET, PUSH/POP, INC/DEC, NOT/NEG, MUL/DIV, IN/OUT, string ops, segment overrides, MOVZX/MOVSX, BT family, LEA, LES/LDS, ENTER/LEAVE, INT, XCHG, TEST, and more. When adding new opcodes to `InstructionDecoder`, add corresponding encoding support in the assembler.
10. **Temporary Debug Code Policy:** Do not leave `fprintf(stderr, ...)` tracing, hardcoded address watchpoints, synthetic key injection, or per-program diagnostic blocks in production code. Use the `LOG_*` macros for permanent logging. If you add temporary instrumentation during a debugging session, remove it before committing.
11. **Testing Strategy:** Unit tests use a custom lightweight framework (`tests/test_framework.hpp`). Tests are organized by subsystem: CPU behavior, instruction decoding, addressing modes, interrupts, BIOS HLE, DOS services, memory management, directories, drive/search, hardware devices, assembler, and assembler-driven interrupt integration tests. When adding new interrupt handlers or instructions, add corresponding tests. The assembler-driven tests in `test_int_asm.cpp` exercise the full pipeline (assemble → write to memory → decode/execute → HLE handler) and should be extended for new handlers.
