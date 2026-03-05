# Fador's DOS Emulator

!note! This software is more or less AI hallucination, work in progress.

## Overview
Fador's DOS Emulator is a multiplatform MS-DOS emulator written in modern C++20. The primary goal is to support all Intel 80386 instructions and most standard BIOS and DOS interrupts.

## Features
- **CPU Emulation:** Full Intel 80386 instruction set support.
- **System Emulation:** Comprehensive support for PC architecture, memory mapping, and I/O ports.
- **Realistic Memory Model:** Conventional memory (0x500–0x9FFFF) is pre-filled with a non-zero pattern (`0xCC`) to simulate residual data left by COMMAND.COM and device drivers on real hardware. BIOS, DOS, and program loaders overwrite the relevant regions; only truly uninitialised areas retain the fill. This prevents programs that rely on non-zero far-heap contents (e.g. Turbo C 2.01 Turbo Vision) from misbehaving.
- **BIOS & DOS APIs:** Extensive support for INT 10h, INT 13h, INT 16h, INT 1Ah, INT 21h, and more. Refer to the `docs/` folder (Ralph Brown's interrupt list) for specifications.
- **Hardware Devices:** PIC (8259), PIT (8254), Keyboard Controller (8042), VGA controller with text-mode VRAM.
- **Program Loaders:** `.COM` and MZ `.EXE` file loading with full relocation support, PSP construction, and MCB-based memory management.
- **Built-in Disassembler:** Read-only x86 real-mode disassembler that produces human-readable assembly from memory without modifying CPU state. Supports single-byte and two-byte (`0F`) opcodes, all prefix combinations, ModR/M and SIB addressing.
- **Interactive Debugger:** Step through instructions, inspect registers, dump memory, and disassemble code at arbitrary addresses.
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
| `--debug=<cats>` | Enable debug logging for categories: `cpu`, `video`, `dos` (comma-separated) |

### Examples
```bash
# Run a COM file
./fadors_emu game.com

# Stop after 5000 cycles and inspect state
./fadors_emu --stop-after=5000 program.com

# Run with CPU debug logging
./fadors_emu --debug=cpu program.exe

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
| `q` | Quit emulator |
