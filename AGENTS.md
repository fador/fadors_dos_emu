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
5. **Build System:** Maintain the `CMakeLists.txt`. Ensure it compiles consistently natively on all target platforms (Windows, Linux, macOS) without relying on OS-specific extensions where C++ standard library equivalents exist.
6. **Built-in Disassembler:** A read-only x86 real-mode disassembler lives in `src/cpu/Disassembler.{hpp,cpp}`. It reads bytes from `MemoryBus` and produces `DisassembledInstruction` structs (address, length, mnemonic, hex bytes) without modifying any state. Use `disassembleAt()`, `disassembleRange()`, or `disassembleAround()` to inspect code. When adding new opcodes to `InstructionDecoder`, mirror the decoding in the disassembler's `disasmOpcode()` / `disasmOpcode0F()` methods.
7. **Debugger & Cycle Stop:** The interactive debugger (`src/ui/Debugger.{hpp,cpp}`) provides single-step, register dump, memory dump, and disassembly commands. The `--stop-after=N` CLI flag halts execution after N instruction cycles and calls `Debugger::dumpState()` to print registers and surrounding disassembly.
