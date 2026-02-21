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
  - [x] Data transfer (MOV, PUSH, POP)
  - [x] String operations (MOVS, STOS, LODS, SCAS, CMPS)
  - [ ] Advanced 386 protected mode instructions (later)
- [x] **Interrupt Pipeline**
  - [x] CPU exception handling
  - [x] Hardware interrupt pin routing

## Phase 3: Hardware Devices
- [x] Port I/O dispatching (IOBus)
- [x] PIC (8259)
- [x] PIT (8254)
- [x] Keyboard Controller (8042)
- [ ] CMOS/RTC (Optional)

## Phase 4: BIOS Emulation
- [x] System initialization (IVT setup)
- [x] INT 10h (Video Services) - Emulated text-mode logic
- [x] INT 13h (Disk Services) - Basic floppy/hard drive image support
- [x] INT 16h (Keyboard Services)
- [x] INT 1Ah (Time of Day / RTC Services)

## Phase 5: DOS Emulation (MS-DOS functionality)
- [x] COM File Loader
- [x] EXE File Loader (MZ header parsing, relocation calculation)
- [x] Program Segment Prefix (PSP) construction
- [x] INT 20h (Terminate Program)
- [x] INT 21h (DOS Services)
  - [x] Console I/O (Ah=01h, 02h, 09h, 0Ah, etc.)
  - [x] File I/O (Open, Close, Read, Write, Lseek) using handles
  - [x] Process Control (Ah=4Ch)
  - [x] Directory Operations (Ah=39h, 3Ah, 3Bh, 47h)
  - [x] Memory Management (Ah=48h, 49h, 4Ah)
  - [x] Drive Information (Ah=0Eh, 19h, 36h)

## Phase 6: Front-End UI
- [x] VRAM-to-Terminal Rendering (Text mode 80x25)
- [x] Keyboard Input Mapping (conio.h based for Windows)
- [x] Built-in Debugger CLI
  - [x] Register inspection
  - [x] Memory dump
  - [x] Single-step execution
- [ ] Direct disassembly view
- [ ] GUI (Optional, maybe SDL2 later)
- [ ] Decoupled video module
- [ ] GUI renderer (e.g., SDL2 / OpenGL) focusing on pixel-perfect VGA/CGA rendering

## Phase 7: Graphical Extension (Future)

## Missing Features / Compatibility Gaps
- [ ] High-level emulation (HLE) for more INT vectors:
  - [ ] INT 11h (Equipment List)
  - [ ] INT 12h (Memory Size)
  - [ ] INT 14h (Serial Port)
  - [ ] INT 15h (System Services)
  - [ ] INT 17h (Printer)
  - [ ] INT 2Fh (Multiplex/TSR/Network)
  - [ ] INT 1Bh, 1Ch, 1Dh, 1Eh, 1Fh (System/Timer/Video/ROM)
- [ ] More complete INT 10h (Video) support:
  - [ ] Implement all major AH functions (mode set, cursor, page, palette, block transfer, etc.)
  - [ ] Support for graphics modes (VGA, EGA, CGA)
  - [ ] BIOS video state save/restore
  - [ ] VESA extensions (optional)
- [ ] More complete INT 13h, 16h, 1Ah, 21h subfunctions
- [ ] Fallback/logging for unhandled INTs (warn with vector/reg state)
- [ ] Stubs for common but unimplemented INTs to avoid program crashes
- [ ] Automated test coverage for INT HLEs
