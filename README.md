# Fador's DOS Emulator

Work in progress. The emulator surface is expanding quickly and some subsystems are still incomplete.

## Overview
Fador's DOS Emulator is a multiplatform MS-DOS emulator written in modern C++20. The project targets real-mode DOS software, 80386 protected-mode workloads, and DOS extenders while keeping CPU, memory, hardware devices, and UI frontends decoupled.

## Features
- **CPU Emulation:** 16-bit Real Mode and 32-bit Protected Mode execution, descriptor-table and control-register instructions, string operations, and REP fast paths. The emulator now also includes a substantial x87 implementation covering stack management, 32/64/80-bit loads and stores, integer conversion, arithmetic, comparisons, rounding, transcendental helpers (`FSIN`, `FCOS`, `FSINCOS`, `FPTAN`, `FPATAN`, `FYL2X`, `F2XM1`, etc.), and x87 environment/state save-restore (`FLDENV`, `FNSTENV`, `FNSAVE`, `FRSTOR`).
- **Protected Mode & DOS Extenders:** DPMI 0.9 host for protected-mode DOS extenders, including GDT/IDT handling, mode-switch entry points, segment reload callbacks, and app/host interrupt plumbing. DOS/4GW (bound or loaded through `DOS4GW.EXE`), PMODE/W, DOS/32A, CauseWay and DJGPP programs run; `tests/compat` checks each of them. 386 paging (CR0.PG, CR3, INVLPG, 4 MB pages) is available to raw-mode software.
- **Memory Model:** 128 MB of physical memory: conventional memory managed through a DOS-compatible MCB chain, an XMS/DPMI pool from 1 MB to 112 MB, 8 MB of SVGA video memory at `0x07000000` (the VBE linear frame buffer), and host tables (DPMI LDT/IDT/GDT) above it so programs can never allocate over them.
- **Expanded and Extended Memory:** EMS services via `INT 67h` with an `EMMXXXX0` device, page-frame mappings, and private API stubs, plus optional HIMEM/XMS 3.0 support (32-bit sizes beyond 64 MB) wired through `INT 2Fh` and far-call dispatch stubs.
- **BIOS & DOS APIs:** Support for core BIOS and DOS interrupts including `INT 10h`, `INT 13h`, `INT 15h`, `INT 16h`, `INT 1Ah`, `INT 21h`, `INT 2Fh`, `INT 31h`, `INT 33h`, and `INT 67h`. Refer to the `docs/` folder (Ralph Brown's interrupt list) for specifications.
- **Hardware Devices:** PIC 8259, PIT 8254, DMA 8237, keyboard controller, joystick, VGA (text, CGA, EGA/VGA planar, mode 13h and Mode X), AdLib (OPL2), Sound Blaster DSP, and MPU-401 MIDI interface emulation.
- **SVGA / VESA:** VBE 2.0 (`INT 10h` AX=4F00h-4F0Ah) with 8/15/16/32-bit modes from 320x200 to 1024x768, a banked A000h window, a linear frame buffer, and the protected-mode interface.
- **Audio:** AdLib OPL2 (YM3812) FM synthesis with 9 channels, 2 operators per channel, ADSR envelopes, and waveform selection. Sound Blaster DSP emulation for 8/16-bit mono/stereo DMA playback (single-cycle, auto-init, high-speed). MPU-401 MIDI UART at port 0x330–0x331 with an OPL2-based General MIDI synthesizer supporting 128 instruments across 16 MIDI channels with velocity, pitch bend, sustain, and 9-voice polyphonic note allocation.
- **Program Loading and Overlays:** `.COM` and MZ `.EXE` loading with relocation support, PSP/environment block creation, MCB-backed program memory, embedded 32-bit payload launching, and VROOMM/FBOV overlay loading for segmented applications.
- **Built-in Disassembler:** Read-only x86 real-mode disassembler that produces human-readable assembly from memory without modifying CPU state. Supports prefixes, ModR/M and SIB decoding, and one-byte plus `0F` opcode forms.
- **Built-in Assembler:** x86 real-mode assembler (`src/cpu/Assembler.{hpp,cpp}`) used by the interactive debugger and `--exec` CLI mode. Supports ALU ops, MOV, shifts, jumps, Jcc, CALL/RET, PUSH/POP, INC/DEC, NOT/NEG, MUL/DIV, IN/OUT, string ops, segment overrides, MOVZX/MOVSX, BT family, LEA, LES/LDS, ENTER/LEAVE, INT, XCHG, TEST, and more.
- **Interactive Debugger and CLI Tooling:** Single-step execution, one-line instruction tracing with raw bytes and register snapshots, register and memory inspection, disassembly, inline assembly, `--exec` for one-shot assembly execution, and `--stop-after`, `--stop-after-debugger`, `--instruction-trace`, and `--dump-on-exit` state dumps.
- **Benchmark Modes and Profiles:** Built-in `decoder-loop`, `rep-movsb`, and `rep-movsd` benchmarks for interpreter hot paths, plus fixed-workload DOS programs (`tests/perf`) that report guest MIPS for integer, memory, x87, call, string, VGA and 16-bit real-mode code.
- **Headless Mode:** `--headless` runs without a window or terminal, with a time limit, PNG screenshots and scripted key presses, for automated testing of DOS programs.
- **Frontends:** Text-mode terminal rendering by default, with optional SDL rendering when the project is built with SDL2.
- **Test Coverage:** Custom unit tests cover CPU execution, instruction decoding, assembler/disassembler behavior, BIOS/DOS/DPMI services, memory managers, paging, interrupts, and hardware devices. A differential fuzzer (`cpu_fuzz`) compares integer instructions against the host CPU.
- **Decoupled Architecture:** CPU, memory, devices, and UI frontends remain separated so rendering backends can change without rewriting the emulation core.

## Build Instructions
The project uses CMake for cross-platform building. A C++20 compatible compiler is required.

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

## Testing
```bash
./build/fadors_tests                       # unit tests (TEST_FILTER=substring to select)
./build/cpu_fuzz [count] [seed]            # differential integer-instruction fuzzer (x86-64 hosts)
python3 tests/perf/run_profiles.py build/fadors_emu tests/perf   # performance profiles, MIPS table
tests/compat/build.sh /tmp/compat && tests/compat/run.sh build/fadors_emu /tmp/compat
```
See `tests/perf/README.md` and `tests/compat/README.md` for details and reference numbers.

## Running
```bash
./fadors_emu [options] <program.com|exe> [program-args...]
```

### Options
| Flag | Description |
|------|-------------|
| `--help`, `-h`, `-?` | Print usage information and exit |
| `--himem` | Enable HIMEM/XMS support |
| `--mount=L:/path` | Mount a host directory as drive `L` (repeatable) |
| `--env=NAME=VALUE` | Add or replace a DOS environment variable (repeatable). Defaults: `BLASTER=A220 I5 D1 H5 P330 T6`, `CAUSEWAY=DPMI` |
| `--headless` | No window, terminal output or host input. Exit status is 124 when `--max-seconds` expires |
| `--max-seconds=S` | Stop after `S` seconds of wall-clock time |
| `--screenshot=FILE.png` | Save the screen (text or graphics mode) as PNG at exit |
| `--screenshot-at=S` | Take the `--screenshot` `S` seconds after start instead |
| `--keys=T:KEY,...` | Press `KEY` at `T` seconds (letters, digits, `ESC`, `ENTER`, `SPACE`, `TAB`, `BKSP`, arrows, `F1`-`F10`, `LCTRL`, `LALT`, `LSHIFT`, ...) |
| `--sdl` | Force the SDL frontend when SDL2 support is available |
| `--no-sdl` | Force headless/text-mode execution even in SDL builds |
| `--throttle-ips=N` | Pace guest execution to roughly `N` retired instructions per second |
| `--throttle-machine=name` | Apply a rough named speed preset: `8088`, `286-12`, `386dx-33`, `486dx2-66`, or `pentium-90` |
| `--stop-after=N` | Stop execution after N instruction cycles and dump CPU state |
| `--stop-after-debugger` | When `--stop-after` triggers, enter the interactive debugger at that exact instruction instead of exiting |
| `--dump-on-exit` | Dump CPU state when the program terminates |
| `--instruction-trace=N` | Print one trace line per executed instruction for the next `N` instructions |
| `--instruction-trace-from=ADDR` | Delay `--instruction-trace` until CS:EIP reaches the given hex linear address |
| `--instruction-trace-step` | Pause between each instruction trace line (ENTER to step, `sN` to skip N, `q` to quit, `d` for debugger) |
| `--trace=<cats>` | Enable trace-level logging for category `cpu`, `video`, `dos` (comma-separated) |
| `--debug=<cats>` | Enable debug logging for categories: `cpu`, `video`, `dos` (comma-separated) |
| `--bench=decoder-loop\|rep-movsb\|rep-movsd` | Run an in-process benchmark without loading a program |
| `--bench-steps=N` | Number of measured instruction steps for `--bench` |
| `--bench-warmup=N` | Number of warmup instruction steps before `--bench` timing |
| `--exec="asm"` | Assemble and execute instructions without loading a program (semicolons separate lines) |

### Examples
```bash
# Run a COM file
./fadors_emu game.com

# Run headless with HIMEM/XMS enabled
./fadors_emu --himem --no-sdl program.exe

# Run a game for 20 seconds without a window, press ENTER after 5 s
# and ESC after 15 s, and keep a screenshot taken at 12 s
./fadors_emu --headless --max-seconds=20 --keys=5:ENTER,15:ESC \
    --screenshot=shot.png --screenshot-at=12 game.exe

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

# Trace the first 32 instructions from program start
./fadors_emu --instruction-trace=32 program.com

# Wait for a code address, then trace the next 16 instructions
./fadors_emu --instruction-trace=16 --instruction-trace-from=12345 program.com

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
| `s` | Single-step one instruction and print one trace line |
| `t [n] [addr]` | Trace and execute `n` instructions, optionally waiting for a hex linear address before printing |
| `c` | Continue execution |
| `r` | Print all registers |
| `d [addr]` | Dump memory at address (hex) |
| `u [addr] [n]` | Disassemble n instructions at address |
| `a [addr]` | Enter assembly mode (type instructions, blank line to exit) |
| `w addr bytes` | Write hex bytes directly to memory |
| `q` | Quit emulator |
