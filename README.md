# Fador's DOS Emulator

Work in progress. The emulator surface is expanding quickly and some subsystems are still incomplete.

## Overview
Fador's DOS Emulator is a multiplatform MS-DOS emulator written in modern C++20. The project targets real-mode DOS software, 80386 protected-mode workloads, and DOS extenders while keeping CPU, memory, hardware devices, and UI frontends decoupled.

## Features
- **CPU Emulation:** 16-bit Real Mode and 32-bit Protected Mode execution, descriptor-table and control-register instructions, string operations, and REP fast paths. The x87 FPU is bit-accurate: registers are 80-bit (64-bit significand), precision and rounding control, all exception flags (with stack faults, masked and unmasked responses, rebiased over/underflow results), condition codes, NaN propagation, BCD, the 16/32-bit real/protected `FSTENV`/`FSAVE` layouts and unmasked-exception delivery (`#MF` with `CR0.NE=1`, otherwise `FERR#` → IRQ13 → `INT 75h` → `INT 02h`) follow real hardware. On GCC/Clang x86/x86-64 builds the arithmetic runs on the host's x87; elsewhere (MSVC, ARM64) a portable software 80-bit implementation is used (transcendentals there are within 16 ulps of a real x87).
- **Selectable CPU Models:** `--cpu=386|486|486dx2|486dx4|pentium|pentiumpro` (default `486dx2`) sets the reset signature, `CPUID` (none on the 386 and early 486; `GenuineIntel` leaves advertising only emulated features), which `EFLAGS`/`CR0`/`CR4` bits exist, the model-gated instructions (`BSWAP`/`CMPXCHG`/`XADD`/`INVD`/`WBINVD`/`INVLPG` from the 486, `RDTSC`/`RDMSR`/`WRMSR`/`CMPXCHG8B` from the Pentium, `CMOVcc`/`FCMOVcc`/`FCOMI` on the Pentium Pro raise `#UD` elsewhere) and per-instruction clock costs. `POPF`/`IRET` follow the 386 privilege rules (reserved bits, `IOPL` only at CPL 0, `IF` only when CPL <= IOPL, V86 IOPL-sensitive instructions fault), so CPU-detection routines see the selected model. A guest cycle counter (`CPU::cycles()`) drives emulated time; `RDTSC` and MSR 10h count emulated time at the model's clock.
- **Protected Mode & DOS Extenders:** DPMI 0.9 host for protected-mode DOS extenders, including GDT/IDT handling, mode-switch entry points, segment reload callbacks, and app/host interrupt plumbing. DOS/4GW (bound or loaded through `DOS4GW.EXE`), PMODE/W, DOS/32A, CauseWay and DJGPP programs run; `tests/compat` checks each of them. A VCPI 1.0 server (INT 67h AX=DE00h-DE0Ch) sits on the EMS driver for extenders that use VCPI (`--dpmi=off` / `--vcpi=off` select the path). Software that owns protected mode itself (raw mode, VCPI clients, V86 monitors) gets the 386/486/Pentium protection model: 386 paging (CR0.PG, CR3, INVLPG, 4 MB pages) with #PF restart, privilege levels, call gates, TSS stack switches, hardware task switches (JMP/CALL to a TSS or task gate, IDT task gates, IRET with NT, busy bits and back links, 286 TSSs), the I/O permission bitmap for IN/OUT/INS/OUTS, virtual-8086 mode with the Pentium's VME/PVI virtual interrupt flag and interrupt redirection, `#GP` for privileged instructions at CPL > 0, `#NM` for x87 instructions under CR0.EM/TS (software FPU emulators), and the debug registers (instruction and data breakpoints, DR6/DR7, RF, general detect) plus TF single-stepping.
- **Memory Model:** 128 MB of physical memory: conventional memory managed through a DOS-compatible MCB chain, an XMS/DPMI pool from 1 MB to 112 MB, 8 MB of SVGA video memory at `0x07000000` (the VBE linear frame buffer), and host tables (DPMI LDT/IDT/GDT) above it so programs can never allocate over them.
- **Expanded and Extended Memory:** EMS services via `INT 67h` with an `EMMXXXX0` device, page-frame mappings, and private API stubs, plus optional HIMEM/XMS 3.0 support (32-bit sizes beyond 64 MB) wired through `INT 2Fh` and far-call dispatch stubs.
- **BIOS & DOS APIs:** Support for core BIOS and DOS interrupts including `INT 10h`, `INT 13h`, `INT 15h`, `INT 16h`, `INT 1Ah`, `INT 21h`, `INT 2Fh`, `INT 31h`, `INT 33h`, and `INT 67h`. Refer to the `docs/` folder (Ralph Brown's interrupt list) for specifications.
- **Hardware Devices:** PIC 8259, PIT 8254 (all six counter modes, gates, latch and read-back commands, BCD), MC146818 RTC/CMOS (periodic, alarm and update interrupts), DMA 8237, keyboard controller, joystick, VGA (text, CGA, EGA/VGA planar, mode 13h and Mode X), AdLib (OPL2), Sound Blaster DSP, and MPU-401 MIDI interface emulation.
- **Serial and Parallel Ports:** 16550A UARTs (COM1 3F8h IRQ 4, COM2 2F8h IRQ 3; COM3/COM4 optional) with 16-byte FIFOs, trigger levels, the character timeout interrupt, exact loopback mode, modem status deltas and character timing from the baud rate on emulated time; a Microsoft serial mouse, file output and a TCP null modem can be attached (`--com1=...`). Standard parallel ports (LPT1 378h IRQ 7) with the PC's status polarity, a printer-to-file device, a Covox Speech Thing and a Disney Sound Source (16-byte FIFO played at 7 kHz) mixed into the audio output (`--lpt1=...`). `INT 14h` (AH=00h-05h) and `INT 17h` drive them through their registers with the BDA time-outs; the BDA port tables and equipment word list the configured ports. See [Serial and parallel ports](#serial-and-parallel-ports).
- **VGA:** Register-accurate IBM VGA: attribute controller (index/data flip-flop, palette, colour select, pel panning, blink/intensity), DAC with PEL mask and read/write auto-increment, sequencer, graphics controller (write modes 0-3, read modes 0-1, latches, odd/even and chain-4 with the hardware plane layout), CRTC with protect bit and overflow registers, misc output (3Bxh/3Dxh addressing). Port 3DAh retrace timing follows the programmed CRTC timings (70 Hz for 400/350-line modes, 60 Hz for 480-line modes) from an injectable clock. The renderer derives the picture from the registers, so tweaked modes (Mode X, 320x400, 360x480, split screen, smooth scrolling, 80x50/90x30 text, custom fonts in plane 2) display like on hardware; INT 10h mode sets load the IBM VGA parameter tables, default palettes and ROM fonts.
- **SVGA / VESA:** VBE 2.0 (`INT 10h` AX=4F00h-4F0Ah) with 8/15/16/32-bit modes from 320x200 to 1024x768, a banked A000h window, a linear frame buffer, and the protected-mode interface.
- **Audio:** AdLib OPL2 (YM3812) FM synthesis with 9 channels, 2 operators per channel, ADSR envelopes, and waveform selection. Sound Blaster DSP emulation for 8/16-bit mono/stereo DMA playback (single-cycle, auto-init, high-speed). MPU-401 MIDI UART at port 0x330–0x331 with an OPL2-based General MIDI synthesizer supporting 128 instruments across 16 MIDI channels with velocity, pitch bend, sustain, and 9-voice polyphonic note allocation.
- **Program Loading and Overlays:** `.COM` and MZ `.EXE` loading with relocation support, PSP/environment block creation, MCB-backed program memory, embedded 32-bit payload launching, and VROOMM/FBOV overlay loading for segmented applications.
- **Built-in Disassembler:** Read-only x86 real-mode disassembler that produces human-readable assembly from memory without modifying CPU state. Supports prefixes, ModR/M and SIB decoding, and one-byte plus `0F` opcode forms.
- **Built-in Assembler:** x86 real-mode assembler (`src/cpu/Assembler.{hpp,cpp}`) used by the interactive debugger and `--exec` CLI mode. Supports ALU ops, MOV, shifts, jumps, Jcc, CALL/RET, PUSH/POP, INC/DEC, NOT/NEG, MUL/DIV, IN/OUT, string ops, segment overrides, MOVZX/MOVSX, BT family, LEA, LES/LDS, ENTER/LEAVE, INT, XCHG, TEST, and more.
- **Interactive Debugger and CLI Tooling:** Single-step execution, one-line instruction tracing with raw bytes and register snapshots, register and memory inspection, disassembly, inline assembly, `--exec` for one-shot assembly execution, and `--stop-after`, `--stop-after-debugger`, `--instruction-trace`, and `--dump-on-exit` state dumps.
- **Host SIMD:** Frame conversion (mode 13h, Mode X, 16-colour planar, VBE 8/15/16/24/32bpp), audio mixing/conversion and REP STOS fills use kernels chosen at startup from the host CPU's features (CPUID/XGETBV: SSE2, SSE4, AVX2; NEON on ARM64), with a portable scalar fallback. The build itself stays at the baseline ISA. See [Host SIMD](#host-simd).
- **Benchmark Modes and Profiles:** Built-in `decoder-loop`, `rep-movsb`, `rep-movsd` and `rep-stosd` benchmarks for interpreter hot paths, host-SIMD kernel benchmarks (`render-*`, `audio-mix`, `mem-fill`), plus fixed-workload DOS programs (`tests/perf`) that report guest MIPS for integer, memory, x87, call, string, VGA and 16-bit real-mode code.
- **Emulated Time:** One virtual clock drives the PIT, RTC, Sound Blaster, OPL timers, PC speaker and the port 61h refresh bit. By default the guest runs at full host speed while timers stay real-time-correct; `--throttle-*` sets a fixed machine speed and `--deterministic` makes timing a pure function of the executed instructions' cycle costs; `HLT` and blocking BIOS/DOS calls sleep until the next interrupt; the BIOS/DOS date and time follow emulated time (see [Emulated time](#emulated-time)).
- **Headless Mode:** `--headless` runs without a window or terminal, with a time limit, PNG screenshots and scripted key presses, for automated testing of DOS programs.
- **Frontends:** Text-mode terminal rendering by default, with optional SDL rendering when the project is built with SDL2.
- **Test Coverage:** Custom unit tests cover CPU execution, instruction decoding, assembler/disassembler behavior, BIOS/DOS/DPMI services, memory managers, paging, interrupts, and hardware devices. Differential fuzzers compare integer instructions (`cpu_fuzz`) and x87 instruction sequences (`fpu_fuzz`: 80-bit stack, status/tag words, memory results) against the host CPU.
- **Decoupled Architecture:** CPU, memory, devices, and UI frontends remain separated so rendering backends can change without rewriting the emulation core.

## Build Instructions
The project uses CMake for cross-platform building. A C++20 compatible compiler is required.

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

`-DFADOR_SOFT_X87=ON` uses the portable software x87 back end even on hosts
with a native x87 (it is selected automatically for MSVC and non-x86 hosts),
so it can be tested on Linux too, e.g. in a second build directory:
`cmake -S . -B build-soft -DFADOR_SOFT_X87=ON`.

## Testing
```bash
./build/fadors_tests                       # unit tests (TEST_FILTER=substring to select)
./build/cpu_fuzz [count] [seed]            # differential integer-instruction fuzzer (x86-64 hosts)
./build/fpu_fuzz [count] [seed] [filter]   # differential x87 fuzzer: emulator vs. the host x87 (x86-64 hosts)
./build/fpu_fuzz --backend [count] [seed]  # soft 80-bit back end vs. the host x87, operation by operation
python3 tests/perf/run_profiles.py build/fadors_emu tests/perf   # performance profiles, MIPS table
tests/compat/build.sh /tmp/compat && tests/compat/run.sh build/fadors_emu /tmp/compat
```
See `tests/perf/README.md` and `tests/compat/README.md` for details and reference numbers.

## Host SIMD

`src/utils/HostCpu.*` detects the host's instruction-set extensions once
(CPUID and XGETBV on x86, the architecture on ARM64) and the startup log
lists them with the chosen level, e.g.
`Host CPU: GenuineIntel (...) features: sse2 ... avx2 ... erms fsrm; SIMD kernels: avx2 (auto)`.
`src/utils/simd` holds the kernels: a scalar reference for each, and SSE2,
SSE4 (SSSE3 + SSE4.1), AVX2 and NEON variants where they are faster. The
variants are compiled per function with target attributes (no global
`-mavx2`), so one binary runs on every x86-64 CPU and picks the best level
at startup. `--host-simd=LEVEL` or `FADOR_HOST_SIMD=LEVEL` forces a level
(an unsupported one falls back to the best supported level below it). All
levels produce bit-identical output (`tests/test_host_simd*.cpp`).

Kernel timings on the development VM (Xeon Sapphire Rapids, 2.1 GHz,
`--bench=...`, best of 15 rounds; "before" is the per-pixel code the
renderer used previously; for VBE the scalar kernel is the previous loop;
the host is shared, so expect some noise):

| Benchmark | Before | Scalar | SSE2 | SSE4 | AVX2 |
|-----------|-------:|-------:|-----:|-----:|-----:|
| `render-13h` (320x200 frame) | 42.6 us | 19.9 us | 19.1 us | 20.0 us | 19.1 us |
| `render-modex` (320x200) | 45.3 us | 19.4 us | 19.4 us | 19.4 us | 19.1 us |
| `render-planar` (mode 12h, 640x480) | 1804 us | 149 us | 144 us | 115 us | 95 us |
| `render-vbe` 15bpp (640x480) | 166 us | 166 us | 80 us | 80 us | 47 us |
| `render-vbe` 16bpp | 164 us | 164 us | 82 us | 80 us | 50 us |
| `render-vbe` 24bpp | 217 us | 217 us | 233 us | 49 us | 50 us |
| `render-vbe` 32bpp | 188 us | 188 us | 72 us | 72 us | 70 us |
| `audio-mix` float to int16 (8192 samples) | 27.8 us | 27.8 us | 2.5 us | 2.5 us | 1.3 us |
| `audio-mix` int16 to float | 1.5 us | 1.5 us | 0.9 us | 0.9 us | 0.6 us |
| `mem-fill` 16 KiB dword pattern | 346 ns | 346 ns | 84 ns | 84 ns | 84 ns |

The 8bpp palette lookup (mode 13h, VBE 8bpp) stays scalar at every level:
an AVX2 gather was only ~1.2x faster here and is much slower on CPUs with
the Gather Data Sampling microcode mitigation. Bulk copies (the REP MOVS
fast path) keep using `std::memmove`, which already matched or beat
hand-written AVX2 and `REP MOVSB` copies at every size measured. Fills of
1 KiB and more use `REP STOSD` on CPUs with ERMS (about 3x faster than
vector stores for L1/L2-sized blocks). REP STOSB/W/D into plain RAM now
fills each 4096-element batch at once: `--bench=rep-stosd` (16 KiB per
REP STOSD) went from 8.1 us to 0.034 us per step, and the `mem32` profile
from 5.99 s to 5.57 s.

## Running
```bash
./fadors_emu [options] <program.com|exe> [program-args...]
./fadors_emu [options] <directory>      # DOS prompt with the directory as C:
```
A program given on the command line runs with its directory's parent as
`C:` and its own directory as the current one (`C:\GAME\GAME.EXE`); with
`--mount=C:/path` the given directory stays the root and the program's
directory below it becomes current.  Programs started from the DOS prompt
run exactly like that too, with interrupts, sound and the headless options.

### Options
| Flag | Description |
|------|-------------|
| `--help`, `-h`, `-?` | Print usage information and exit |
| `--himem` | Enable HIMEM/XMS support |
| `--cpu=MODEL` | Emulated CPU: `386` (386DX-33 + 387), `486` (early 486DX-33, no CPUID), `486dx2` (default, 486DX2-66 with CPUID), `486dx4` (DX4-100), `pentium` (P54C-100: TSC, MSRs, CMPXCHG8B, CR4 with VME/PVI/DE) or `pentiumpro` (adds CMOVcc/FCMOVcc/FCOMI) |
| `--no-fpu` | No x87 coprocessor (a 386 without a 387, a 486SX): x87 instructions do nothing, so `FNINIT`/`FNSTSW` probes find no FPU; CR0.ET (386), CPUID and the INT 11h equipment word report it. With CR0.EM set, x87 instructions raise `#NM` (INT 7) for a software emulator instead |
| `--dpmi=off` | No DPMI host: INT 2Fh AX=1687h reports none, so extenders fall back to VCPI or raw mode (`--dpmi=on` is the default) |
| `--vcpi=off` | No VCPI server: INT 67h AX=DE00h fails (`--vcpi=on` is the default) |
| `--mount=L:/path` | Mount a host directory as drive `L` (repeatable) |
| `--env=NAME=VALUE` | Add or replace a DOS environment variable (repeatable). Defaults: `BLASTER=A220 I5 D1 H5 P330 T6`, `CAUSEWAY=DPMI` |
| `--headless` | No window, terminal output or host input. Exit status is 124 when `--max-seconds` expires |
| `--max-seconds=S` | Stop after `S` seconds of wall-clock time (of emulated time with `--deterministic`) |
| `--screenshot=FILE.png` | Save the screen (text or graphics mode) as PNG at exit |
| `--screenshot-at=S` | Take the `--screenshot` `S` seconds after start instead (emulated seconds with `--deterministic`) |
| `--keys=T:KEY,...` | Press `KEY` at `T` seconds (letters, digits, `ESC`, `ENTER`, `SPACE`, `TAB`, `BKSP`, arrows, `F1`-`F10`, `LCTRL`, `LALT`, `LSHIFT`, ...); emulated seconds with `--deterministic` |
| `--sdl` | Force the SDL frontend when SDL2 support is available |
| `--no-sdl` | Force headless/text-mode execution even in SDL builds |
| `--throttle-ips=N` | Run at a fixed `N` instructions per second of emulated time; the host sleeps when the guest gets ahead |
| `--throttle-machine=name` | Run at a fixed machine speed: the preset selects the CPU model (cycle table, CPUID; unless `--cpu` is given, then it only sets the MHz and warns on a mismatch) and its clock: `8088`, `286-12`, `386sx-16`, `386dx-25`, `386dx-33`, `486sx-25`, `486dx-33`, `486dx2-66`, `486dx4-100`, `pentium-60`, `pentium-90`, `pentium-100`, `pentium-133`, `pentiumpro-150`, `pentiumpro-200` (aliases `xt`, `286`, `386`, `486`, `486dx4`, `pentium`, `ppro`, ...). See the table below |
| `--deterministic` | Emulated time = guest clock cycles / CPU clock (the `--cpu` model's nominal clock, or the `--throttle-*` speed) and never reads the host clock: headless runs, screenshots and `--audio-wav` recordings are reproducible. Execution is not paced; the date and time start at 1994-06-01 12:00:00 |
| `--start-time=YYYY-MM-DDTHH:MM:SS` | Date and time the emulated RTC and DOS clock start at (default: host local time, or the fixed `--deterministic` start) |
| `--stop-after=N` | Stop execution after N instruction cycles and dump CPU state |
| `--stop-after-debugger` | When `--stop-after` triggers, enter the interactive debugger at that exact instruction instead of exiting |
| `--dump-on-exit` | Dump CPU state when the program terminates |
| `--no-fast-path` | Run every instruction through the legacy decoder (no predecoded instruction cache); for debugging the CPU |
| `--instruction-trace=N` | Print one trace line per executed instruction for the next `N` instructions |
| `--instruction-trace-from=ADDR[#N]` | Delay `--instruction-trace` until CS:EIP reaches the given hex linear address (the `N`-th time with `#N`) |
| `--instruction-trace-step` | Pause between each instruction trace line (ENTER to step, `sN` to skip N, `q` to quit, `d` for debugger) |
| `--io-trace=LO[-HI]` | Log every port read and write in the hex range, with the CS:EIP that made it, to stderr |
| `--audio-wav=FILE.wav` | Record the sound output (AdLib/OPL3, Sound Blaster, PC speaker, MIDI); the log reports its peak level |
| `--trace=<cats>` | Enable trace-level logging for category `cpu`, `video`, `dos` (comma-separated) |
| `--debug=<cats>` | Enable debug logging for categories: `cpu`, `video`, `dos` (comma-separated) |
| `--bench=decoder-loop\|rep-movsb\|rep-movsd\|rep-stosd` | Run an in-process interpreter benchmark without loading a program |
| `--bench=render-13h\|render-modex\|render-planar\|render-vbe\|audio-mix\|mem-fill` | Time the host-SIMD kernels at every SIMD level the host supports |
| `--host-simd=auto\|avx2\|sse4\|sse2\|neon\|scalar` | Force the host-SIMD kernel level (default `auto`; also `FADOR_HOST_SIMD=...`) |
| `--bench-steps=N` | Number of measured instruction steps for `--bench` |
| `--bench-warmup=N` | Number of warmup instruction steps before `--bench` timing |
| `--exec="asm"` | Assemble and execute instructions without loading a program (semicolons separate lines) |
| `--com1=DEV` ... `--com4=DEV` | What is attached to a serial port: `none` (port present, nothing attached; COM1/COM2 default), `off` (no port; COM3/COM4 default), `mouse`, `file:PATH`, `tcp:HOST:PORT`, `listen:PORT` |
| `--lpt1=DEV` ... `--lpt3=DEV` | What is attached to a parallel port: `none` (LPT1 default), `off` (LPT2/LPT3 default), `file:PATH`, `covox`, `disney` |

### DPMI and VCPI

Both are present by default. Extenders that look for DPMI first (DOS/4GW,
PMODE/W, DOS/32A, DJGPP) keep using the DPMI host; software that only
uses VCPI when no DPMI host exists can be tested with `--dpmi=off`, and
`--vcpi=off` hides the VCPI server.

A real VCPI server (EMM386, QEMM) runs DOS in virtual-8086 mode. Here
DOS runs in real mode, so the server (`src/hw/VCPI.*`) switches between
real mode and the client's protected mode, and makes the difference
invisible apart from CR0.PE reading 0 while DOS runs:

- DE01h fills page table 0 with an identity map of the first megabyte
  (EMS pages are copied in and out of the page frame, so DE06h returns
  `page << 12`) and returns three descriptors for the client's GDT: the
  server's 32-bit code segment (base F0000h; the entry point is
  `0F FF E4; RETF`, an HLE trap) and two data segments.
- DE0Ch from real mode loads the client's CR3, GDTR, IDTR, LDTR and TR
  (marking the TSS busy), sets PE and PG, opens the A20 gate and jumps to
  the client with interrupts disabled. DS, ES, FS and GS are null.
- DE0Ch through the protected-mode entry (far CALL with the VCPI stack
  frame) turns PE and PG off, reloads IDTR = 0:3FFh (the IVT), restores the
  A20 gate and resumes the given CS:EIP, SS:ESP, DS, ES, FS, GS with
  interrupts disabled, as a V86 server would enter V86 mode.
- DE04h/DE05h hand out 4 KB pages from XMS (taken in 1 MB chunks),
  DE03h counts them, DE02h reports the top of the XMS pool, DE07h-DE0Bh
  read CR0, the debug registers and the 8259 vector bases.

### Emulated time

All timed devices read one virtual clock (`src/hw/VirtualClock.hpp`): the
8254 PIT (IRQ 0, speaker, port 61h refresh toggle every ~15 us and OUT2),
the RTC (seconds, UIP, periodic/alarm/update-ended IRQ 8), Sound Blaster
DMA, OPL timers, the PC speaker and audio generation (44100 frames per
emulated second). Virtual time advances with the guest clock cycles the
CPU model charges per instruction (`CPU::cycles()`, see `--cpu`), so a
timer read in the middle of a loop sees time move. `src/hw/MachineTime.*`
connects the CPU model to the clock. How emulated time relates to the host
clock depends on the pacing mode:

| Mode | Selected by | Behaviour |
|------|-------------|-----------|
| auto | default | The guest runs as fast as the host allows; its rate is measured continuously so emulated time tracks the wall clock. After a host stall (debugger, window drag) emulated time jumps by at most 20 ms. With an audio device, the audio queue level nudges the clock so it neither underruns nor grows. |
| fixed | `--throttle-ips`, `--throttle-machine` | Emulated time = guest cycles / clock rate (`--throttle-ips`: instructions / N); the host sleeps while emulated time is ahead of the wall clock. |
| deterministic | `--deterministic` | Emulated time = guest cycles / clock rate; the host clock is never read and execution is not paced. `--max-seconds`, `--keys` and `--screenshot-at` count emulated seconds. |

Machine presets (`--throttle-machine`) and the CPU model they select. The
emulator models 386 and later CPUs: older and slower parts use the 386
cycle table with the clock divided by a correction (their cycles per
instruction relative to a 386: 8088 ~14.5, 286 ~10, 386SX ~5 against 4.5),
and they still execute the 386 instruction set.

| Preset | CPU model | Guest clock |
|--------|-----------|-------------|
| `8088` | `386` | 4.77 MHz / 3.2 |
| `286-12` | `386` | 12 MHz / 2.2 |
| `386sx-16` | `386` | 16 MHz / 1.1 |
| `386dx-25`, `386dx-33` | `386` | 25, 33 MHz |
| `486sx-25`, `486dx-33` | `486` | 25, 33 MHz |
| `486dx2-66` | `486dx2` | 66 MHz |
| `486dx4-100` | `486dx4` | 100 MHz |
| `pentium-60` ... `pentium-133` | `pentium` | 60, 90, 100, 133 MHz |
| `pentiumpro-150`, `pentiumpro-200` | `pentiumpro` | 150, 200 MHz |

Without a preset the model runs at its nominal clock (`386`/`486`: 33 MHz,
`486dx2`: 66, `486dx4` and `pentium`: 100, `pentiumpro`: 200).

- **Time-stamp counter:** `RDTSC` (and MSR 10h) returns emulated
  nanoseconds x the core clock (the preset's MHz or the model's nominal
  clock), in every pacing mode, made strictly increasing, plus the `WRMSR`
  offset. In fixed and deterministic pacing it equals the guest cycle count
  while the guest does not wait; in auto pacing the guest runs at host
  speed but a program calibrating `RDTSC` against the PIT still measures
  the nominal MHz. The TSC keeps counting while the CPU is halted.
- **Waiting:** `HLT` with interrupts enabled sleeps until an interrupt is
  delivered: emulated time jumps to the next device event (PIT IRQ 0, RTC,
  Sound Blaster block end; at most 1 ms per step) in fixed and
  deterministic pacing, and the host thread sleeps (<= 1 ms at a time) in
  auto pacing. `HLT` with interrupts disabled logs a warning and continues.
  Blocking BIOS/DOS input (`INT 16h` AH=00h, `INT 21h` AH=01h/07h/08h/0Ah),
  `INT 21h` AH=06h/0Bh polls, `INT 28h` and `INT 2Fh` AX=1680h wait the
  same way when the guest calls them in a tight loop (less than 2048
  cycles of work in between). Halted time does not add to `CPU::cycles()`.
- **HLE services** are charged nominal cycles (`INT 10h` 300, `INT 13h`
  1000, `INT 16h` 100, `INT 21h` 500) for the BIOS/DOS code they replace.
- **Date and time:** the RTC starts at the host's local time (fixed
  1994-06-01 12:00:00 with `--deterministic`, or `--start-time`) and then
  runs on emulated time. The BIOS loads the tick count at 0040:006Ch from
  it; INT 8 wraps the count at midnight and sets the midnight flag.
  `INT 1Ah` AH=00h/01h use the tick count, AH=02h-05h the RTC. DOS (AH=2Ah
  -2Dh) works like MS-DOS's CLOCK$ driver: the time of day is the tick
  count, the date advances on the midnight flag, and setting the date or
  time also sets the RTC. Host file timestamps are kept as they are.

At exit the emulator logs the emulated time, guest cycles, time spent
halted and the BIOS tick count next to the host time and MIPS.
`tests/timing/check_time.py build/fadors_emu` runs end-to-end checks
(RDTSC against the PIT, HLT, DOS clock, reproducibility).

### Serial and parallel ports

| Port | I/O | IRQ | Default |
|------|-----|-----|---------|
| COM1 | 3F8h | 4 | present, nothing attached |
| COM2 | 2F8h | 3 | present, nothing attached |
| COM3 / COM4 | 3E8h / 2E8h | none | off |
| LPT1 | 378h | 7 (control bit 4) | present, nothing attached |
| LPT2 / LPT3 | 278h / 3BCh | none | off |

Serial devices (`--comN=`):

- `none`: the UART exists (programs and `INT 14h` see it) but CTS/DSR/DCD are low and nothing is received.
- `mouse`: Microsoft 2-button serial mouse driven by the host mouse (the same input `INT 33h` uses). It identifies itself with `M` when a driver raises DTR and RTS and sends 3-byte packets at 1200 baud, so serial mouse drivers (CTMOUSE, MOUSE.COM /C1) and games with their own serial mouse code work.
- `file:PATH`: everything transmitted is written to `PATH` (serial printer, debug logs); CTS/DSR/DCD are active.
- `tcp:HOST:PORT` / `listen:PORT`: a null modem cable over TCP, connecting to a peer or accepting one client; raw bytes, CTS/DSR/DCD active while connected. Two emulator instances can talk to each other with `listen:5000` and `tcp:127.0.0.1:5000`.

Parallel devices (`--lptN=`):

- `none`: the inputs float high (status 7Fh), the BIOS reports "not ready".
- `file:PATH`: a printer that writes every strobed byte (`INT 17h`, DOS printing, direct port I/O) to `PATH`.
- `covox`: Covox Speech Thing, an 8-bit DAC on the data port.
- `disney`: Disney Sound Source: samples are latched into a 16-byte FIFO by pulsing SELECT IN (control bit 3) and played at 7 kHz; status bit 6 reports a full FIFO, which is what the games' detection routines check.

`INT 14h`/`INT 17h` wait for the port as the ROM BIOS does, with the time-outs at 40:7Ch (COM, 1 by default) and 40:78h (LPT, 20) counted in units of 0.25 s of emulated time.

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

# Run as a 386DX/33 (386 model and cycle table at 33 MHz)
./fadors_emu --throttle-machine=386dx-33 program.exe

# Reproducible run on a Pentium 133 with a given date and time
./fadors_emu --headless --deterministic --throttle-machine=pentium-133 \
    --start-time=1996-03-01T09:00:00 --screenshot=shot.png program.exe

# Reproducible headless run: same screenshot and recording every time
./fadors_emu --headless --deterministic --max-seconds=20 --keys=5:ENTER \
    --screenshot=shot.png --audio-wav=out.wav game.exe

# Serial mouse on COM1, Disney Sound Source on LPT1
./fadors_emu --com1=mouse --lpt1=disney game.exe

# Print to a file; connect COM2 to another emulator listening on port 5000
./fadors_emu --lpt1=file:printout.txt --com2=tcp:127.0.0.1:5000 term.exe

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

# Same, but start at the 200th time that address is reached
./fadors_emu --instruction-trace=400 --instruction-trace-from=37BF8#200 game.exe

# See what a game does with the Sound Blaster, and record what it plays
./fadors_emu --headless --max-seconds=10 --io-trace=220-22F --audio-wav=out.wav game.exe

# Start the DOS prompt in a directory and type a command into it
./fadors_emu --headless --max-seconds=30 --keys=1:G,1.1:A,1.2:M,1.3:E,1.4:ENTER games/

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

# Compare the frame conversion of mode 12h at every host SIMD level
./fadors_emu --no-sdl --bench=render-planar

# Run with the portable scalar kernels only
./fadors_emu --host-simd=scalar game.exe

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
