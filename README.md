# Fador's DOS Emulator

Work in progress. The emulator surface is expanding quickly and some subsystems are still incomplete.

## Overview
Fador's DOS Emulator is a multiplatform MS-DOS emulator written in modern C++20. The project targets real-mode DOS software, 80386 protected-mode workloads, and DOS extenders while keeping CPU, memory, hardware devices, and UI frontends decoupled.

## Features
- **CPU Emulation:** 16-bit Real Mode and 32-bit Protected Mode execution, descriptor-table and control-register instructions, string operations, and REP fast paths. The emulator now also includes a substantial x87 implementation covering stack management, 32/64/80-bit loads and stores, integer conversion, arithmetic, comparisons, rounding, transcendental helpers (`FSIN`, `FCOS`, `FSINCOS`, `FPTAN`, `FPATAN`, `FYL2X`, `F2XM1`, etc.), and x87 environment/state save-restore (`FLDENV`, `FNSTENV`, `FNSAVE`, `FRSTOR`).
- **Protected Mode & DOS Extenders:** DPMI host support for protected-mode DOS extenders such as DOS/4GW, including GDT/IDT handling, mode-switch entry points, segment reload callbacks, and app/host interrupt plumbing.
- **Memory Model:** Up to 32 MB of physical memory with deterministic dirty-RAM initialization above the IVT/BDA. DOS conventional memory is managed through an MCB chain with DOS-compatible allocation, free, and resize behavior.
- **Expanded and Extended Memory:** EMS services via `INT 67h` with an `EMMXXXX0` device, page-frame mappings, and private API stubs, plus optional HIMEM/XMS support wired through `INT 2Fh` and far-call dispatch stubs.
- **BIOS & DOS APIs:** Support for core BIOS and DOS interrupts including `INT 10h`, `INT 13h`, `INT 15h`, `INT 16h`, `INT 1Ah`, `INT 21h`, `INT 2Fh`, `INT 31h`, `INT 33h`, and `INT 67h`. Refer to the `docs/` folder (Ralph Brown's interrupt list) for specifications.
- **Hardware Devices:** PIC 8259, PIT 8254, DMA 8237, keyboard controller, joystick, VGA text-mode controller, AdLib (OPL2), and Sound Blaster DSP emulation.
- **Program Loading and Overlays:** `.COM` and MZ `.EXE` loading with relocation support, PSP/environment block creation, MCB-backed program memory, embedded 32-bit payload launching, and VROOMM/FBOV overlay loading for segmented applications.
- **Built-in Disassembler:** Read-only x86 real-mode disassembler that produces human-readable assembly from memory without modifying CPU state. Supports prefixes, ModR/M and SIB decoding, and one-byte plus `0F` opcode forms.
- **Built-in Assembler:** x86 real-mode assembler (`src/cpu/Assembler.{hpp,cpp}`) used by the interactive debugger and `--exec` CLI mode. Supports ALU ops, MOV, shifts, jumps, Jcc, CALL/RET, PUSH/POP, INC/DEC, NOT/NEG, MUL/DIV, IN/OUT, string ops, segment overrides, MOVZX/MOVSX, BT family, LEA, LES/LDS, ENTER/LEAVE, INT, XCHG, TEST, and more.
- **Interactive Debugger and CLI Tooling:** Single-step execution, register and memory inspection, disassembly, inline assembly, `--exec` for one-shot assembly execution, and `--stop-after`, `--stop-after-debugger`, and `--dump-on-exit` state dumps.
- **Benchmark Modes:** Built-in `decoder-loop`, `rep-movsb`, and `rep-movsd` benchmarks for measuring interpreter hot paths without full guest startup noise.
- **Frontends:** Text-mode terminal rendering by default, with optional SDL rendering when the project is built with SDL2.
- **Test Coverage:** Custom unit tests cover CPU execution, instruction decoding, assembler/disassembler behavior, BIOS/DOS/DPMI services, memory managers, interrupts, and hardware devices.
- **Decoupled Architecture:** CPU, memory, devices, and UI frontends remain separated so rendering backends can change without rewriting the emulation core.

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
| `--himem` | Enable HIMEM/XMS support |
| `--sdl` | Force the SDL frontend when SDL2 support is available |
| `--no-sdl` | Force headless/text-mode execution even in SDL builds |
| `--throttle-ips=N` | Pace guest execution to roughly `N` retired instructions per second |
| `--throttle-machine=name` | Apply a rough named speed preset: `8088`, `286-12`, `386dx-33`, `486dx2-66`, or `pentium-90` |
| `--stop-after=N` | Stop execution after N instruction cycles and dump CPU state |
| `--stop-after-debugger` | When `--stop-after` triggers, enter the interactive debugger at that exact instruction instead of exiting |
| `--dump-on-exit` | Dump CPU state when the program terminates |
| `--bench=decoder-loop\|rep-movsb\|rep-movsd` | Run an in-process benchmark without loading a program |
| `--bench-steps=N` | Number of measured instruction steps for `--bench` |
| `--bench-warmup=N` | Number of warmup instruction steps before `--bench` timing |
| `--exec="asm"` | Assemble and execute instructions without loading a program (semicolons separate lines) |
| `--debug=<cats>` | Enable debug logging for categories: `cpu`, `video`, `dos` (comma-separated) |

### Examples
```bash
# Run a COM file
./fadors_emu game.com

# Run headless with HIMEM/XMS enabled
./fadors_emu --himem --no-sdl program.exe

# Run with an execution throttle of about 4 MIPS
./fadors_emu --throttle-ips=4000000 program.exe

# Run with a rough 486DX2/66-class throttle preset
./fadors_emu --throttle-machine=486dx2-66 program.exe

# Use the SDL frontend when it is available in the build
./fadors_emu --sdl program.exe

# Stop after 5000 cycles and inspect state
./fadors_emu --stop-after=5000 program.com

# Break into the interactive debugger exactly at the stop-after point
./fadors_emu --stop-after=5000 --stop-after-debugger program.com

# Run with CPU debug logging
./fadors_emu --debug=cpu program.exe

# Assemble and execute instructions directly
./fadors_emu --exec="MOV AX, 1234h;INT 21h"

# Benchmark the decoder hot loop without process-startup noise
./fadors_emu --no-sdl --bench=decoder-loop --bench-warmup=500000 --bench-steps=5000000

# Benchmark REP MOVSB throughput with the current interpreter
./fadors_emu --no-sdl --bench=rep-movsb --bench-warmup=500000 --bench-steps=5000000

# Benchmark REP MOVSD throughput with the width-aware MOVS fast path
./fadors_emu --no-sdl --bench=rep-movsd --bench-warmup=500000 --bench-steps=5000000

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
