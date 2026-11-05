# Emulator Development Plan

## Phase 1: Core Foundation
- [x] **Project Setup**
  - [x] Initial CMake C++20 configuration
  - [x] Documentation and agent guidelines
  - [x] Logging macros and debugging utilities
- [x] **Memory Subsystem**
  - [x] 1MB Conventional memory bus mapping
  - [x] Read/Write interface handling (8-bit, 16-bit, 32-bit)
  - [x] VGA Text Mode VRAM (0xB8000) mapping

## Phase 2: CPU Emulation (Intel 80386)
- [x] **CPU State & Registers**
  - [x] General Purpose Registers (EAX, EBX, ECX, EDX, ESI, EDI, EBP, ESP)
  - [x] Segment Registers (CS, DS, ES, FS, GS, SS) and Descriptor Caches
  - [x] EFLAGS register
  - [x] Control/Debug Registers (CR0-CR4, DR0-DR7)
- [x] **Instruction Decoding & Execution**
  - [x] Instruction Fetcher (Prefixes, ModR/M, SIB, Displacement, Immediate)
  - [x] Arithmetic and Logic Unit (ALU) instructions
  - [x] Control flow (JMP, CALL, RET, INT, IRET)
  - [x] Data transfer (MOV, PUSH, POP, PUSAD/POPAD, CS/DS offsets)
  - [x] String operations (MOVS, STOS, LODS, SCAS, CMPS)
  - [x] Advanced 386 protected mode instructions (LGDT, SGDT, LIDT, SIDT, LMSW, SMSW, MOV CR)
- [x] **x87 FPU Coprocessor**
  - [x] Stack management (FLD, FST, FSTP, FXCH, FINIT)
  - [x] 32/64/80-bit memory loads and stores
  - [x] Integer conversion (FILD, FIST, FISTP)
  - [x] Arithmetic (FADD, FSUB, FMUL, FDIV, FABS, FCHS, FSQRT, FPREM)
  - [x] Comparisons (FCOM, FCOMP, FCOMPP, FTST, FXAM)
  - [x] Rounding and control (FRNDINT, FWAIT, FNOP, FLDCW, FSTCW, FLDSW, FSTSW)
  - [x] Transcendentals (FSIN, FCOS, FSINCOS, FPTAN, FPATAN, FYL2X, F2XM1, FSCALE)
  - [x] Environment save/restore (FLDENV, FNSTENV, FNSAVE, FRSTOR)
  - [x] D9 /3 FSTP ST(i) alias (Borland-compiled code)
- [x] **Interrupt Pipeline**
  - [x] CPU exception handling
  - [x] Hardware interrupt pin routing

## Phase 3: Hardware Devices
- [x] Port I/O dispatching (IOBus)
- [x] PIC (8259)
- [x] PIT (8254)
- [x] Keyboard Controller (8042)
- [x] VGA Controller (Text/Graphics modes)
- [x] DMA Controller (8237)
- [x] Joystick (game port 0x201)
- [x] CMOS/RTC (MC146818 at ports 0x70-0x71)
  - [x] Unit tests covering all registers, RTC time, status, memory sizing, checksum, NMI mask

## Phase 4: BIOS Emulation
- [x] System initialization (IVT setup)
- [x] INT 10h (Video Services) - Emulated text-mode logic
- [x] INT 13h (Disk Services) - Basic floppy/hard drive image support
- [x] INT 16h (Keyboard Services)
- [x] INT 1Ah (Time of Day / RTC Services)
- [x] INT 33h (Mouse Services)
- [x] INT 11h, 12h, 14h, 15h, 17h, 1Bh, 1Ch, 1Dh, 1Eh, 1Fh (system stubs)
- [x] INT 2Fh (Multiplex/TSR/Network)
- [x] INT 31h (DPMI protected-mode services)
- [x] INT 67h (EMS/LIM 4.0 expanded memory)

## Phase 5: DOS Emulation (MS-DOS functionality)
- [x] COM File Loader
- [x] EXE File Loader (MZ header parsing, relocation calculation)
- [x] Native 32-bit DOS/4GW Extender capability
- [x] VROOMM/FBOV overlay loading for segmented Borland applications
- [x] Program Segment Prefix (PSP) construction
- [x] Environment block creation from parent
- [x] INT 20h (Terminate Program)
- [x] INT 21h (DOS Services)
  - [x] Console I/O (Ah=01h, 02h, 09h, 0Ah, etc.)
  - [x] File I/O (Open, Close, Read, Write, Lseek) using handles
  - [x] Process Control (Ah=4Ch)
  - [x] Directory Operations (Ah=39h, 3Ah, 3Bh, 47h)
  - [x] Memory Management (Ah=48h, 49h, 4Ah)
  - [x] Drive Information (Ah=0Eh, 19h, 36h)
  - [x] System/Country Information (Ah=38h)

## Phase 6: Front-End UI
- [x] VRAM-to-Terminal Rendering (Text mode 80x25)
- [x] Keyboard Input Mapping (conio.h based for Windows)
- [x] Built-in Debugger CLI
  - [x] Register inspection
  - [x] Memory dump
  - [x] Single-step execution
  - [x] Cycle-limited execution
  - [x] Inline assembly mode (`a [addr]`)
  - [x] Direct byte writing (`w addr bytes`)
- [x] Direct disassembly view
- [x] **SDL2 Graphical Frontend**
  - [x] SDL2 window with VGA text-mode rendering
  - [x] Keyboard input through SDL2 events
  - [x] Frame-rate limited rendering (~30 FPS)
  - [x] Optional via `--sdl` / `--no-sdl` flags
- [x] Instruction throttle system (`--throttle-ips=N`, `--throttle-machine=name`)
- [ ] GUI renderer pixel-perfect VGA/CGA graphics mode rendering

## Phase 7: Audio Subsystem
- [x] **Native Audio Backend**
  - [x] Cross-platform audio (SDL2 Audio) with ring-buffered streaming
  - [x] PCM mixing of multiple sources
- [x] **AdLib / OPL3 (YMF262 via ymfm)**
  - [x] Ports 0x388–0x38B and the Sound Blaster's FM ports (base+0..3, +8, +9)
  - [x] OPL timers and status reads with ISA-like latency (detection works)
- [x] **Sound Blaster 16 (DSP 4.05)**
  - [x] I/O ports 0x220–0x22F, BLASTER=A220 I5 D1 H5 P330 T6
  - [x] 8-bit and 16-bit DMA playback (single-cycle, auto-init, high-speed)
  - [x] Mono and stereo, signed/unsigned, SB Pro stereo via the mixer
  - [x] IRQ 5 on block completion (8-bit 22Eh / 16-bit 22Fh acknowledge)
  - [x] SB16 mixer registers, IRQ/DMA select (80h/81h), interrupt status
  - [x] Direct DAC (command 10h), silence blocks, speaker on/off
  - [x] Clocked by emulated time, not by the host audio device
- [x] **PC speaker** (PIT channel 2 square waves, port 61h bit-banging, PWM)
- [x] **Output Mixer** — AdLib + SB PCM + PC speaker into one stream
- [x] `--audio-wav=FILE` records the output (headless tests check the level)

## Phase 8: Built-in Assembler & Disassembler
- [x] x86 real-mode disassembler (`src/cpu/Disassembler.{hpp,cpp}`)
  - [x] Prefix, ModR/M, SIB decoding
  - [x] One-byte and 0F opcode tables
- [x] x86 real-mode assembler (`src/cpu/Assembler.{hpp,cpp}`)
  - [x] Tokenizer, parser, encoder pipeline
  - [x] ALU, MOV, shifts, jumps (short/near), Jcc
  - [x] CALL/RET, PUSH/POP, INC/DEC, NOT/NEG, MUL/DIV
  - [x] IN/OUT, string ops, segment overrides
  - [x] MOVZX/MOVSX, BT family, LEA, LES/LDS
  - [x] ENTER/LEAVE, INT, XCHG, TEST, flags ops
- [x] Debugger integration (`a` command, `w` command)
- [x] CLI `--exec="asm"` mode for headless assembly execution
- [x] Assembler unit tests (`test_assembler.cpp`)
- [x] Assembler-driven interrupt integration tests (`test_int_asm.cpp`)
  - [x] INT 10h (13 subfunctions), INT 16h, INT 1Ah
  - [x] INT 11h, INT 12h, INT 13h (3 subfunctions)
  - [x] INT 14h, INT 15h, INT 17h stubs
  - [x] INT 33h (5 subfunctions), INT 2Fh
  - [x] INT 20h, INT 21h (~20 subfunctions incl. file I/O roundtrip)
  - [x] Full integration test (set mode + print string + terminate)

## Phase 9: Benchmarking & Diagnostics
- [x] **Benchmark Modes** (`--bench=name`)
  - [x] `decoder-loop` — measure raw instruction decoder throughput
  - [x] `rep-movsb` — measure byte-string copy fast path
  - [x] `rep-movsd` — measure dword-string copy fast path
  - [x] `--bench-steps=N` and `--bench-warmup=N` for tuning
- [x] **Instruction Tracing**
  - [x] `--instruction-trace=N` (conditional, with optional start address)
  - [x] `--instruction-trace-from=ADDR#N` starts at the N-th hit of an address
  - [x] `--instruction-trace-step` interactive stepping through traces
- [x] **Port I/O tracing** (`--io-trace=LO-HI`, with the CS:EIP of each access)
- [x] **Crash Handler**
  - [x] SEH/signal crash dump with registers, EIP disassembly, stack, VRAM
- [x] **Headless runs** (`--headless`, `--max-seconds`, `--screenshot[-at]`, `--keys`)
- [x] **Performance profiles** (`tests/perf`: fixed-workload DOS programs, MIPS table)
- [x] **Differential CPU fuzzer** (`cpu_fuzz`, integer instructions vs. the host CPU)
- [ ] Cycle-weighted `--throttle-machine` presets (currently instructions/second)
- [ ] Further interpreter speed-ups (decoded-instruction cache or a JIT)

## Phase 10: Memory & Extended Memory
- [x] **HIMEM/XMS (INT 2Fh + far-call dispatcher)**
  - [x] XMS driver detection via INT 2Fh/4300h
  - [x] XMS function dispatch: version, request/ unlock HMA, global/ local enable A20
  - [x] XMS memory management: allocate, lock, unlock, free, resize, query
  - [x] XMS 3.0 32-bit functions (88h, 89h, 8Eh, 8Fh)
- [x] **EMS (INT 67h)**
  - [x] EMMXXXX0 device detection
  - [x] Page frame mapping and EMS memory management
  - [x] Private API stubs for expanded memory

## Phase 11: Protected Mode & DOS Extenders
- [x] **DPMI Host (INT 31h)**
  - [x] GDT/LDT management and segment allocation
  - [x] IDT entry setup and interrupt descriptor plumbing
  - [x] Mode-switch entry points (real→protected→real)
  - [x] Segment reload callbacks into the instruction decoder
  - [x] App/host interrupt routing
- [x] **DOS/4GW Support**
  - [x] Embedded 32-bit payload launching from MZ stub
  - [x] DOS/4GW header parsing and linear execution
  - [x] VROOMM/FBOV overlay loading for segmented Borland applications
  - [x] Stub-loaded DOS/4GW (`DOS4GW.EXE` next to the program)
- [x] **Other extenders** (see `tests/compat`)
  - [x] PMODE/W
  - [x] DOS/32A
  - [x] CauseWay (through DPMI; `CAUSEWAY=DPMI` is set by default)
  - [x] DJGPP / CWSDPMI-style clients
- [x] 386 paging for raw-mode software (CR0.PG, CR3, INVLPG, PSE)
- [x] Page faults delivered to the guest (#PF with instruction restart)
- [x] Privilege levels: call gates, inter-ring CALL/RET/IRET, TSS stack switch
      (for software that owns protected mode itself)
- [ ] CauseWay in raw mode (CAUSEWAY=DPMI is the default; raw mode still fails)
- [ ] VCPI (INT 67h DE00h) for extenders that prefer it over DPMI

## Missing Features / Compatibility Gaps
- [ ] More complete INT 10h (Video) support:
  - [x] Implement all major AH functions (mode set, cursor, page, palette, block transfer, etc.)
  - [x] Support for graphics modes (VGA, EGA, CGA)
  - [x] BIOS video state save/restore (AH=1Ch)
  - [x] VESA BIOS Extensions 2.0 (banked, linear frame buffer, PM interface)
- [ ] More complete INT 13h, 16h, 1Ah, 21h subfunctions
- [ ] Fallback/logging for unhandled INTs (warn with vector/reg state)
- [ ] Stubs for common but unimplemented INTs to avoid program crashes
- [x] Automated test coverage for INT HLEs
- [ ] GUI renderer pixel-perfect VGA/CGA hardware-accelerated graphics
- [ ] CD-ROM / ISO image support (MSCDEX / INT 2Fh/1500h)
- [ ] Serial port (INT 14h) actual emulation
- [ ] Parallel port (INT 17h) actual emulation
- [ ] Network (packet driver / NE2000) emulation

## Phase 12: DOS 6.22 Command Prompt (COMMAND.COM)
- [x] **Drive Manager** (`src/hw/DriveManager.{hpp,cpp}`)
  - [x] Drive letter → host directory mount table
  - [x] Per-drive current working directory tracking
  - [x] Path resolution with `..` collapse and mount-root escape prevention
  - [x] Unit tests (`test_drive_manager.cpp`)
- [x] **DOS Shell** (`src/hw/DosShell.{hpp,cpp}`)
  - [x] Prompt display with `$P$G` template expansion
  - [x] Buffered keyboard input (INT 21h AH=0Ah)
  - [x] DOS version updated to 6.22 (AH=30h)
  - [x] Internal commands: DIR, CD, MD, RD, COPY, DEL, REN, TYPE, TREE, MOVE
  - [x] Internal commands: ECHO, SET, PATH, PROMPT, CLS, VER, VOL, DATE, TIME
  - [x] Internal commands: MEM, HELP, MOUNT, UNMOUNT, EXIT, BREAK, VERIFY
  - [x] External program execution with PATH search, run through the same
        loop as a program given on the command line (IRQs, audio, --keys,
        --max-seconds); the shell keeps a small resident block
  - [x] Drive mounting via `MOUNT` command and `--mount=` CLI flag
  - [x] Directory-as-C: mounting when directory path given as argument
- [x] **INT 21h Updates**
  - [x] AH=0Ah buffered keyboard input (real implementation)
  - [x] AH=0Eh/19h drive operations delegated to DriveManager
  - [x] AH=3Bh/47h per-drive CWD via DriveManager
  - [x] AH=36h real disk space via `std::filesystem::space()`
  - [x] `resolvePath()` delegates to DriveManager
- [x] **BAT Scripting** (`src/hw/BatchInterpreter.{hpp,cpp}`)
  - [x] `%0`-`%9` parameter substitution
  - [x] `%VAR%` environment variable expansion
  - [x] `ECHO`, `ECHO OFF`, `@`, `REM`, `PAUSE`
  - [x] `GOTO` / `:labels`, `IF`, `FOR`
  - [x] `CALL`, `SHIFT`, `ERRORLEVEL`
  - [x] `CHOICE` (DOS 6.22 specific)
  - [x] Unit tests (`test_batch.cpp` — 36 tests)
  - [ ] Unit tests (`test_batch.cpp`)

## Known Use-Case Issues
- [ ] Cannon Fodder 2: silent.  After opening TITLE.AMF its loader grows its
      heap with AH=4Ah, which fails because a 2-paragraph block it allocated
      sits right after the program block; the music is then not loaded.
      Needs comparison with a real DOS run to find the earlier divergence.
- [ ] Modplay Pro plays about 7% sharp through the Sound Blaster direct DAC
      (the DAC rate is exact, so this is probably its own pitch table).
- Use-case configuration notes (not emulator bugs): DOOM 1.1 reads its
  settings from C:\DOOMDATA\DEFAULT.CFG and stores the SB port in decimal
  (`snd_sbport 544`); PUKE_W01.MOD is a 6-channel module that Modplay Pro
  (4 channels) rejects; XTC-Play needs module files to play.
