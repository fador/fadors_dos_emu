# Performance profiles

Fixed-workload DOS programs used to measure the interpreter's speed. Each
profile runs a known amount of guest work, and the emulator prints
`Executed N instructions in T s` at exit, so results are comparable between
builds and independent of host start-up time.

| Profile  | Program / args        | Exercises                                          |
|----------|-----------------------|----------------------------------------------------|
| `int32`  | `perfbench int 100`   | 32-bit ALU, branches, IMUL (DJGPP, DPMI flat)      |
| `mem32`  | `perfbench mem 3000`  | `memset`/`memcpy` (REP STOS/MOVS) and array walks  |
| `fpu32`  | `perfbench fpu 10`    | x87 via DJGPP libm sin/cos/sqrt/fmod: FLD/FST m64, FMUL/FADD, FXCH, FIST, FSQRT |
| `call32` | `perfbench call 1000` | recursive CALL/RET, stack traffic                  |
| `str32`  | `perfbench str 20`    | libc `strcpy`/`strlen`/`strcmp`/`strchr`          |
| `vga32`  | `perfbench vga 100`   | mode 13h writes via `dosmemput` and `_farpokeb`    |
| `real16` | `perf16 60`           | 16-bit real mode: ALU, far memory, A000h writes    |

## Running

```
python3 tests/perf/run_profiles.py build/fadors_emu tests/perf [--repeat 3] [--only int32 mem32] [--json out.json]
```

The prebuilt `perfbench.exe` and `perf16.exe` are checked in so no DOS
toolchain is needed. To rebuild them:

```
i586-pc-msdosdjgpp-gcc -O2 perfbench.c -o perfbench.exe -lm     # DJGPP (gcc 12.2)
wcl -q -bt=dos -ml -ox perf16.c -fe=perf16.exe                   # Open Watcom v2
```

For a CPU profile of the emulator itself, build with
`-DCMAKE_BUILD_TYPE=RelWithDebInfo` and run a single profile under `perf`:

```
perf record -g build-rwd/fadors_emu --headless tests/perf/perfbench.exe int 100
perf report --no-children
```

## Reference numbers

Release build, GCC 13, one core of a shared cloud x86-64 VM.  "Before" is
8120f53, "after" the interpreter work below; CPU time, best of 3 runs of
each binary back to back (MIPS = guest instructions per microsecond).
Instruction counts and guest cycle counts are unchanged.

| Profile  | Instructions | Before (s) | After (s) | MIPS before | MIPS after | Speed-up |
|----------|-------------:|-----------:|----------:|------------:|-----------:|---------:|
| int32    |  130,022,722 |      2.092 |     0.953 |        62.2 |      136.4 |    2.20x |
| mem32    |  344,251,426 |      5.435 |     2.050 |        63.3 |      167.9 |    2.65x |
| fpu32    |   73,822,748 |      1.330 |     0.849 |        55.5 |       87.0 |    1.57x |
| call32   |  183,722,450 |      2.900 |     1.216 |        63.4 |      151.1 |    2.39x |
| str32    |  161,014,638 |      2.450 |     1.124 |        65.7 |      143.3 |    2.18x |
| vga32    |  171,388,953 |      2.602 |     1.203 |        65.9 |      142.4 |    2.16x |
| real16   |   95,291,617 |      1.571 |     0.748 |        60.7 |      127.4 |    2.10x |
| geomean  |              |            |           |        62.3 |      134.2 |    2.15x |

What made the difference: lazy arithmetic flags (`LazyFlags.hpp`), the
predecoded fast path with its decoded-instruction cache (`FastPath.cpp`),
bursts of fast-path instructions in `InstructionDecoder::run()`, and bulk
REP MOVS/STOS into the VGA window.  `--no-fast-path` runs the legacy
decoder alone (lazy flags still apply).  The interpreter work before that (13 -> 65
MIPS) was inline register and memory accessors, a flat-RAM fast path in
`MemoryBus`, inline instruction fetch, cheaper logging guards and avoiding
store-forwarding stalls in `InstructionDecoder::step`.

To check that a change keeps the guest's behaviour, compare a
`--deterministic` run with and without `--no-fast-path` (and with the
previous build): the instruction count, the "Guest CPU: ... cycles" line,
the emulated time and the program output must be identical.

Under callgrind, use `--deterministic`: in auto pacing a slowed-down host
makes the virtual clock (and the OPL3 synthesis it drives) dominate the
profile.

The x87 runs on the host FPU in normal x86/x86-64 GCC/Clang builds. A
`-DFADOR_SOFT_X87=ON` build (portable software 80-bit arithmetic, as used
by MSVC and non-x86 hosts) runs `fpu32` noticeably slower.
