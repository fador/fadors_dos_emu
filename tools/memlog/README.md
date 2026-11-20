# MEMLOG

A small resident DOS program that logs the DOS memory calls of whatever runs
after it (INT 21h AH=48h, 49h, 4Ah, 58h: registers in and out, carry) to
`C:\MEMLOG.TXT`. After every AH=48h and after a failed AH=4Ah it also writes
the MCB chain.

Use it to compare this emulator with a reference DOS (DOSBox or a real
machine) when a program's memory behaviour differs:

```
nasm -f bin -o MEMLOG.COM memlog.asm
```

Then, in the reference DOS, run `MEMLOG` followed by the program, and compare
`MEMLOG.TXT` with the emulator's `--debug=dos` log (or run MEMLOG in the
emulator too). MEMLOG itself stays resident below the program, so segment
values shift by its size; compare the layout, not absolute segments.
