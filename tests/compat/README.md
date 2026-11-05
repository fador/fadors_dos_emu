# DOS extender compatibility programs

Small 32-bit programs, each linked against every common DOS extender, that
exercise what games depend on: libc start-up, `printf`, libm, large
`malloc`, file I/O, and hooking the timer (INT 8, reprogrammed to 72.8 Hz
and chained) and keyboard (INT 9) interrupts.

| Program        | Checks                                                     |
|----------------|------------------------------------------------------------|
| `hello32.c`    | argv, x87 math, 4 MB `malloc`, create/read/delete a file    |
| `irq32.c`      | INT 8/9 handlers, chaining, BIOS tick count, 8 MB `malloc`  |
| `clocktest.c`  | DJGPP `clock()` and `__dpmi_yield` against the BIOS ticks   |
| `sbtest.c`     | Sound Blaster 16 DSP, IRQ, 8/16-bit DMA, direct DAC, mixer, OPL2/OPL3 at 388h and the SB ports, PC speaker (real mode and DOS/4GW); the run also checks that `--audio-wav` recorded sound |

Extenders: DOS/4GW (stub + `DOS4GW.EXE`), PMODE/W, DOS/32A, CauseWay
(Open Watcom), and DJGPP's own DPMI client.

```
export WATCOM=/path/to/openwatcom PATH=$WATCOM/binl64:$PATH   # + DJGPP on PATH
tests/compat/build.sh /tmp/compat
tests/compat/run.sh build/fadors_emu /tmp/compat
```

`run.sh` prints `ok`/`FAIL` per program and exits non-zero on any failure.
The binaries are not checked in because the extender runtimes are
third-party; `build.sh` takes them from the Watcom installation.
