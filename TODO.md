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
- [ ] **CPU State & Registers**
  - [ ] General Purpose Registers (EAX, EBX, ECX, EDX, ESI, EDI, EBP, ESP)
  - [ ] Segment Registers (CS, DS, ES, FS, GS, SS) and Descriptor Caches
  - [ ] EFLAGS register
  - [ ] Control/Debug Registers (CR0-CR4, DR0-DR7)
- [ ] **Instruction Decoding & Execution**
  - [ ] Instruction Fetcher (Prefixes, ModR/M, SIB, Displacement, Immediate)
  - [ ] Arithmetic and Logic Unit (ALU) instructions
  - [ ] Control flow (JMP, CALL, RET, INT, IRET)
  - [ ] Data transfer (MOV, PUSH, POP)
  - [ ] String operations (MOVS, STOS, LODS, SCAS, CMPS)
  - [ ] Advanced 386 protected mode instructions (later)
- [ ] **Interrupt Pipeline**
  - [ ] CPU exception handling
  - [ ] Hardware interrupt pin routing

## Phase 3: Hardware Devices
- [ ] Programmable Interval Timer (PIT 8253/8254)
- [ ] Programmable Interrupt Controller (PIC 8259A)
- [ ] Keyboard Controller (8042)
- [ ] CMOS / RTC
- [ ] Basic Port I/O dispatching

## Phase 4: BIOS Emulation
- [ ] System initialization (IVT setup)
- [ ] INT 10h (Video Services) - Emulated text-mode logic
- [ ] INT 13h (Disk Services) - Basic floppy/hard drive image support
- [ ] INT 16h (Keyboard Services)
- [ ] INT 1Ah (Time of Day / RTC Services)

## Phase 5: DOS Emulation (MS-DOS functionality)
- [ ] COM File Loader
- [ ] EXE File Loader (MZ header parsing, relocation calculation)
- [ ] Program Segment Prefix (PSP) construction
- [ ] INT 20h (Terminate Program)
- [ ] INT 21h (DOS Services)
  - [ ] Console I/O (Ah=01h, 02h, 09h, 0Ah, etc.)
  - [ ] File I/O (Open, Close, Read, Write, Lseek) using handles
  - [ ] Directory Operations
  - [ ] Process Control (Ah=4Ch)
  - [ ] Memory Management (Ah=48h, 49h, 4Ah)

## Phase 6: Front-End UI
- [ ] Console/Terminal Text-Mode Renderer backend
- [ ] Raw keyboard input capture mapper
- [ ] Basic built-in debugger CLI (disassembly, memory dumping, stepping)

## Phase 7: Graphical Extension (Future)
- [ ] Decoupled video module
- [ ] GUI renderer (e.g., SDL2 / OpenGL) focusing on pixel-perfect VGA/CGA rendering
