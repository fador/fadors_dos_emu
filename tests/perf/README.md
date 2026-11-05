# Performance profiles

Fixed-workload DOS programs used to measure the interpreter's speed. Each
profile runs a known amount of guest work, and the emulator prints
`Executed N instructions in T s` at exit, so results are comparable between
builds and independent of host start-up time.

| Profile  | Program / args        | Exercises                                          |
|----------|-----------------------|----------------------------------------------------|
| `int32`  | `perfbench int 100`   | 32-bit ALU, branches, IMUL (DJGPP, DPMI flat)      |
| `mem32`  | `perfbench mem 3000`  | `memset`/`memcpy` (REP STOS/MOVS) and array walks  |
| `fpu32`  | `perfbench fpu 10`    | x87: FSIN/FCOS/FSQRT/FPREM                         |
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

Release build, GCC 13, one core of a cloud x86-64 VM, median of 3 runs
(MIPS = guest instructions per microsecond):

| Profile  | Instructions | Before (s) | After (s) | MIPS before | MIPS after |
|----------|-------------:|-----------:|----------:|------------:|-----------:|
| int32    |  130,022,166 |       9.98 |      2.00 |        13.0 |       65.0 |
| mem32    |  344,250,870 |      26.80 |      5.25 |        12.8 |       65.6 |
| fpu32    |   73,821,957 |       5.82 |      1.36 |        12.7 |       54.5 |
| call32   |  183,721,894 |      14.08 |      2.37 |        13.0 |       77.5 |
| str32    |  161,014,082 |      12.05 |      2.02 |        13.3 |       79.8 |
| vga32    |  171,388,397 |      13.25 |      2.53 |        12.9 |       67.8 |
| real16   |   95,291,579 |       7.32 |      1.50 |        13.0 |       63.4 |
| geomean  |              |            |           |        13.0 |       67.2 |

"Before" is the emulator prior to the interpreter work (inline register and
memory accessors, a flat-RAM fast path in `MemoryBus`, inline instruction
fetch, cheaper logging guards and avoiding store-forwarding stalls in
`InstructionDecoder::step`).
