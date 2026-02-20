# Fador's DOS Emulator

## Overview
Fador's DOS Emulator is a multiplatform MS-DOS emulator written in modern C++20. The primary goal is to support all Intel 80386 instructions and most standard BIOS and DOS interrupts.

## Features
- **CPU Emulation:** Full Intel 80386 instruction set support.
- **System Emulation:** Comprehensive support for PC architecture, memory mapping, and I/O ports.
- **BIOS & DOS APIs:** Extensive support for INT 10h, INT 13h, INT 21h, etc. Refer to the `docs/` folder (Ralph Brown's interrupt list) for specifications.
- **Cross-Platform Interface:** Initially implemented as a text-mode console application.
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
./fadors_emu [optional_executable.com/.exe]
```
