# Fador's DOS Emulator

!note! This software is more or less AI hallucination, work in progress.

## Overview
Fador's DOS Emulator is a multiplatform MS-DOS emulator written in modern C++20. The primary goal is to support all Intel 80386 instructions and most standard BIOS and DOS interrupts.

## Features
- **CPU Emulation:** Full Intel 80386 instruction set support.
- **System Emulation:** Comprehensive support for PC architecture, memory mapping, and I/O ports.
- **Realistic Memory Model:** Conventional memory (0x500–0x9FFFF) is pre-filled with a deterministic pseudo-random LCG pattern (seed 0xCC, `val = val * 141 + 3`) to simulate dirty DOS RAM. IVT/BDA (0x000–0x4FF) remains zero-filled so BIOS can write proper values. This prevents programs that rely on non-zero far-heap contents (e.g. Turbo C 2.01 Turbo Vision) from misbehaving — the LCG pattern provides ~255 non-zero bytes between natural zeros, avoiding both false-sentinel and unterminated-string failure modes.
- **BIOS & DOS APIs:** Extensive support for INT 10h, INT 13h, INT 16h, INT 1Ah, INT 21h, and more. Refer to the `docs/` folder (Ralph Brown's interrupt list) for specifications.
- **Hardware Devices:** PIC (8259), PIT (8254), Keyboard Controller (8042), VGA controller with text-mode VRAM.
- **Program Loaders:** `.COM` and MZ `.EXE` file loading with full relocation support, PSP construction, and MCB-based memory management.
- **Built-in Disassembler:** Read-only x86 real-mode disassembler that produces human-readable assembly from memory without modifying CPU state. Supports single-byte and two-byte (`0F`) opcodes, all prefix combinations, ModR/M and SIB addressing.
- **Built-in Assembler:** x86 real-mode assembler (`src/cpu/Assembler.{hpp,cpp}`) that converts assembly text into machine code bytes. Supports ALU ops, MOV, shifts, jumps (short/near), Jcc, CALL/RET, PUSH/POP, INC/DEC, NOT/NEG, MUL/DIV, IN/OUT, string ops, segment overrides, MOVZX/MOVSX, BT family, LEA, LES/LDS, ENTER/LEAVE, INT, XCHG, TEST, and more. Used by the interactive debugger and `--exec` CLI mode.
- **Interactive Debugger:** Step through instructions, inspect registers, dump memory, disassemble code, and assemble instructions directly into memory.
- **Cycle-limited Execution:** Stop emulation after a configurable number of instruction cycles and automatically dump CPU registers and surrounding disassembly.
- **Cross-Platform Interface:** Initially implemented as a text-mode console application with ANSI terminal rendering.
- **Decoupled Architecture:** Designed to easily integrate a separate graphical view (e.g., SDL2, OpenGL) in the future without changing the core emulator logic.

## Build Instructions
The project uses CMake for cross-platform building. A C++20 compatible compiler is required.

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

## Running
```bash
./fadors_emu [options] <program.com|exe> [program-args...]
```

### Options
| Flag | Description |
|------|-------------|
| `--himem` | Enable high memory (HMA/XMS) support |
| `--stop-after=N` | Stop execution after N instruction cycles and dump CPU state |
| `--dump-on-exit` | Dump CPU state when the program terminates |
| `--exec="asm"` | Assemble and execute instructions without loading a program (semicolons separate lines) |
| `--debug=<cats>` | Enable debug logging for categories: `cpu`, `video`, `dos` (comma-separated) |

### Examples
```bash
# Run a COM file
./fadors_emu game.com

# Stop after 5000 cycles and inspect state
./fadors_emu --stop-after=5000 program.com

# Run with CPU debug logging
./fadors_emu --debug=cpu program.exe

# Assemble and execute instructions directly
./fadors_emu --exec="MOV AX, 1234h;INT 21h"

# Launch interactive debugger (no program)
./fadors_emu
```

### Interactive Debugger Commands
| Command | Description |
|---------|-------------|
| `s` | Single-step one instruction |
| `c` | Continue execution |
| `r` | Print all registers |
| `d [addr]` | Dump memory at address (hex) |
| `u [addr] [n]` | Disassemble n instructions at address |
| `a [addr]` | Enter assembly mode (type instructions, blank line to exit) |
| `w addr bytes` | Write hex bytes directly to memory |
| `q` | Quit emulator |
