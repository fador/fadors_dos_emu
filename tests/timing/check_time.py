#!/usr/bin/env python3
"""End-to-end checks of the emulated time base (CPU model <-> VirtualClock).

usage: check_time.py <fadors_emu> [--keep DIR]

Generates tiny .COM programs (hand-assembled below, no DOS toolchain needed),
runs them headless and checks:
  - RDTSC measured against the PIT (BIOS tick count) gives the CPU model's
    clock within 1 % in deterministic, fixed and auto pacing, with busy-wait
    and HLT-wait loops;
  - HLT with IF=1 sleeps until IRQ 0: 100 HLTs are exactly 100 ticks, the
    emulated time is 100 PIT periods, and in auto pacing the host CPU idles;
  - DOS/BIOS date and time follow emulated time, start at the fixed
    --deterministic / --start-time moment, AH=2Dh sets the clock, midnight
    advances the date, and two deterministic runs print identical output;
  - a blocking INT 16h read idles the host in auto pacing.
Exits non-zero on the first failure.
"""
import argparse, os, re, resource, subprocess, sys, tempfile, time

PIT_HZ = 105e6 / 88  # 1193181.8 Hz
TICK_S = 65536 / PIT_HZ


class Asm:
    """Minimal two-pass helper: raw bytes, labels, rel8 jumps, abs16 refs."""

    def __init__(self):
        self.items = []

    def b(self, *bs):
        self.items.append(("b", bytes(bs)))
        return self

    def label(self, name):
        self.items.append(("l", name))
        return self

    def jr(self, opcode, name):  # Jcc/JMP/LOOP rel8
        self.items.append(("j", opcode, name))
        return self

    def call(self, name):  # CALL rel16
        self.items.append(("c", name))
        return self

    def addr(self, name):  # 16-bit offset of a label (COM: + 100h)
        self.items.append(("a", name))
        return self

    def build(self):
        sizes = {"b": None, "l": 0, "j": 2, "c": 3, "a": 2}
        labels, pc = {}, 0x100
        for it in self.items:
            if it[0] == "l":
                labels[it[1]] = pc
            pc += len(it[1]) if it[0] == "b" else sizes[it[0]]
        out, pc = bytearray(), 0x100
        for it in self.items:
            kind = it[0]
            if kind == "b":
                out += it[1]
            elif kind == "j":
                rel = labels[it[2]] - (pc + 2)
                assert -128 <= rel <= 127, it
                out += bytes([it[1], rel & 0xFF])
            elif kind == "c":
                rel = labels[it[1]] - (pc + 3)
                out += bytes([0xE8, rel & 0xFF, (rel >> 8) & 0xFF])
            elif kind == "a":
                v = labels[it[1]]
                out += bytes([v & 0xFF, v >> 8])
            pc += len(it[1]) if kind == "b" else sizes[kind]
        return bytes(out)


def common_tail(a):
    """exit + print_hex32 (EAX) + newline subroutines + data."""
    a.b(0xB8, 0x00, 0x4C, 0xCD, 0x21)  # mov ax,4C00h; int 21h
    a.label("hex32")
    a.b(0xB9, 0x08, 0x00)  # mov cx,8
    a.label("hx1")
    a.b(0x66, 0xC1, 0xC0, 0x04)  # rol eax,4
    a.b(0x66, 0x50)  # push eax
    a.b(0x24, 0x0F, 0x04, 0x30, 0x3C, 0x39)  # and al,0Fh; add al,'0'; cmp al,'9'
    a.jr(0x76, "hx2")  # jbe
    a.b(0x04, 0x07)  # add al,7
    a.label("hx2")
    a.b(0x88, 0xC2, 0xB4, 0x02, 0xCD, 0x21)  # mov dl,al; mov ah,2; int 21h
    a.b(0x66, 0x58)  # pop eax
    a.jr(0xE2, "hx1")  # loop
    a.b(0xB2, 0x20, 0xB4, 0x02, 0xCD, 0x21)  # space
    a.b(0xC3)
    a.label("nl")
    a.b(0xB2, 0x0D, 0xB4, 0x02, 0xCD, 0x21, 0xB2, 0x0A, 0xB4, 0x02, 0xCD, 0x21, 0xC3)
    a.label("t0")
    a.b(0, 0, 0, 0)
    a.label("t1")
    a.b(0, 0, 0, 0)


def prog_tsc(use_hlt):
    """RDTSC delta over 18 BIOS ticks (busy or HLT wait), printed in hex."""
    a = Asm()
    a.b(0x31, 0xC0, 0x8E, 0xC0, 0xFB)  # xor ax,ax; mov es,ax; sti
    a.b(0x26, 0x8B, 0x1E, 0x6C, 0x04)  # mov bx,[es:046Ch]
    a.label("w1")
    if use_hlt:
        a.b(0xF4)
    a.b(0x26, 0x3B, 0x1E, 0x6C, 0x04)  # cmp bx,[es:046Ch]
    a.jr(0x74, "w1")  # je
    a.b(0x0F, 0x31)  # rdtsc
    a.b(0x66, 0xA3).addr("t0")  # mov [t0],eax
    a.b(0x66, 0x89, 0x16).addr("t1")  # mov [t1],edx
    a.b(0x26, 0x8B, 0x1E, 0x6C, 0x04, 0x83, 0xC3, 0x12)  # mov bx,[046Ch]; add bx,18
    a.label("w2")
    if use_hlt:
        a.b(0xF4)
    a.b(0x26, 0x3B, 0x1E, 0x6C, 0x04)  # cmp bx,[es:046Ch]
    a.jr(0x75, "w2")  # jne
    a.b(0x0F, 0x31)  # rdtsc
    a.b(0x66, 0x2B, 0x06).addr("t0")  # sub eax,[t0]
    a.b(0x66, 0x1B, 0x16).addr("t1")  # sbb edx,[t1]
    a.call("hex32").call("nl")
    common_tail(a)
    return a.build()


def prog_halt():
    """100 x HLT; prints the BIOS tick delta."""
    a = Asm()
    a.b(0x31, 0xC0, 0x8E, 0xC0, 0xFB)  # xor ax,ax; mov es,ax; sti
    a.b(0x66, 0x26, 0xA1, 0x6C, 0x04)  # mov eax,[es:046Ch]
    a.b(0x66, 0xA3).addr("t0")
    a.b(0xB9, 0x64, 0x00)  # mov cx,100
    a.label("l")
    a.b(0xF4)
    a.jr(0xE2, "l")  # loop
    a.b(0x66, 0x26, 0xA1, 0x6C, 0x04)  # mov eax,[es:046Ch]
    a.b(0x66, 0x2B, 0x06).addr("t0")  # sub eax,[t0]
    a.call("hex32").call("nl")
    common_tail(a)
    return a.build()


def prog_clock():
    """Prints DOS time/date, sets 23:59:58, waits 40 ticks with HLT, prints
    DOS time/date, INT 1Ah AH=00h (ticks vs BDA, midnight flag), AH=02h and
    AH=04h (RTC, BCD)."""
    a = Asm()

    def dos_time_date():
        a.b(0xB4, 0x2C, 0xCD, 0x21)  # mov ah,2Ch; int 21h
        a.b(0x89, 0xC8, 0x66, 0xC1, 0xE0, 0x10, 0x89, 0xD0)  # ax=cx; shl eax,16; ax=dx
        a.call("hex32")
        a.b(0xB4, 0x2A, 0xCD, 0x21)  # mov ah,2Ah; int 21h
        a.b(0x88, 0xC3)  # mov bl,al (weekday)
        a.b(0x89, 0xC8, 0x66, 0xC1, 0xE0, 0x10, 0x89, 0xD0)  # eax = year:month:day
        a.call("hex32")
        a.b(0x66, 0x0F, 0xB6, 0xC3)  # movzx eax,bl
        a.call("hex32").call("nl")

    a.b(0x31, 0xC0, 0x8E, 0xC0, 0xFB)  # xor ax,ax; mov es,ax; sti
    dos_time_date()
    a.b(0xB9, 0x3B, 0x17, 0xBA, 0x00, 0x3A, 0xB4, 0x2D, 0xCD, 0x21)  # 23:59:58.00
    a.b(0x66, 0x0F, 0xB6, 0xC0)  # movzx eax,al
    a.call("hex32").call("nl")
    a.b(0xB9, 0x28, 0x00)  # mov cx,40
    a.label("l")
    a.b(0x26, 0x8B, 0x1E, 0x6C, 0x04)  # mov bx,[es:046Ch]
    a.label("w")
    a.b(0xF4, 0x26, 0x3B, 0x1E, 0x6C, 0x04)  # hlt; cmp bx,[es:046Ch]
    a.jr(0x74, "w")
    a.jr(0xE2, "l")
    dos_time_date()
    a.b(0xB4, 0x00, 0xCD, 0x1A)  # mov ah,0; int 1Ah
    a.b(0x66, 0x0F, 0xB6, 0xC0)  # movzx eax,al (midnight flag)
    a.b(0x52, 0x51)  # push dx; push cx
    a.call("hex32")
    a.b(0x66, 0x58)  # pop eax (cx:dx -> eax = dx<<16|cx)
    a.b(0x66, 0xC1, 0xC0, 0x10)  # rol eax,16 -> cx<<16|dx
    a.call("hex32")
    a.b(0x66, 0x26, 0xA1, 0x6C, 0x04)  # mov eax,[es:046Ch]
    a.call("hex32").call("nl")
    a.b(0xB4, 0x02, 0xCD, 0x1A)  # int 1Ah AH=02h
    a.b(0x89, 0xC8, 0x66, 0xC1, 0xE0, 0x10, 0x89, 0xD0)
    a.call("hex32")
    a.b(0xB4, 0x04, 0xCD, 0x1A)  # int 1Ah AH=04h
    a.b(0x89, 0xC8, 0x66, 0xC1, 0xE0, 0x10, 0x89, 0xD0)
    a.call("hex32").call("nl")
    common_tail(a)
    return a.build()


def prog_getkey():
    """Blocking INT 16h AH=00h; prints AX."""
    a = Asm()
    a.b(0xB4, 0x00, 0xCD, 0x16, 0x66, 0x0F, 0xB7, 0xC0)  # int 16h; movzx eax,ax
    a.call("hex32").call("nl")
    common_tail(a)
    return a.build()


FAILURES = 0


def check(cond, what):
    global FAILURES
    print(("ok    " if cond else "FAIL  ") + what)
    if not cond:
        FAILURES += 1


def run(emu, prog, opts, max_seconds=30):
    before = resource.getrusage(resource.RUSAGE_CHILDREN)
    t = time.monotonic()
    p = subprocess.run([emu, "--headless", f"--max-seconds={max_seconds}"] + opts + [prog],
                       capture_output=True, text=True, cwd=os.path.dirname(prog))
    wall = time.monotonic() - t
    after = resource.getrusage(resource.RUSAGE_CHILDREN)
    cpu = (after.ru_utime - before.ru_utime) + (after.ru_stime - before.ru_stime)
    m = re.search(r"Emulated time ([0-9.e+-]+) s", p.stderr)
    emu_s = float(m.group(1)) if m else None
    lines = [l.strip() for l in p.stdout.splitlines() if l.strip()]
    return lines, emu_s, wall, cpu, p


def hexwords(line):
    return [int(w, 16) for w in line.split()]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("emu")
    ap.add_argument("--keep")
    a = ap.parse_args()
    emu = os.path.abspath(a.emu)
    d = a.keep or tempfile.mkdtemp(prefix="timecheck")
    os.makedirs(d, exist_ok=True)
    progs = {"TSCBUSY.COM": prog_tsc(False), "TSCHLT.COM": prog_tsc(True),
             "HALT.COM": prog_halt(), "CLOCK.COM": prog_clock(),
             "GETKEY.COM": prog_getkey()}
    for name, code in progs.items():
        with open(os.path.join(d, name), "wb") as f:
            f.write(code)
    P = lambda n: os.path.join(d, n)

    # RDTSC against the PIT: the model's clock within 1 %.
    cases = [(["--deterministic", "--cpu=pentium"], 100e6),
             (["--deterministic", "--throttle-machine=pentium-133"], 133e6),
             (["--deterministic", "--cpu=pentiumpro"], 200e6),
             (["--throttle-machine=pentium-90"], 90e6),
             (["--cpu=pentium"], 100e6)]
    for opts, hz in cases:
        for prog in ("TSCBUSY.COM", "TSCHLT.COM"):
            lines, _, _, _, p = run(emu, P(prog), opts)
            try:
                mhz = hexwords(lines[0])[0] / (18 * TICK_S) / 1e6
            except (IndexError, ValueError):
                mhz = float("nan")
            check(abs(mhz * 1e6 / hz - 1) < 0.01,
                  f"{prog} {' '.join(opts)}: RDTSC/PIT = {mhz:.3f} MHz (want {hz / 1e6:g})")

    # HLT sleeps until IRQ 0.
    lines, emu_s, wall, cpu, p = run(emu, P("HALT.COM"), ["--deterministic"])
    ticks = hexwords(lines[0])[0] if lines else -1
    check(ticks == 100, f"deterministic: 100 HLTs = {ticks} ticks")
    check(emu_s is not None and abs(emu_s - 100 * TICK_S) < 0.07,
          f"deterministic: 100 HLTs take {emu_s} s of emulated time (~{100 * TICK_S:.3f} + start-up)")
    check(wall < 3.0, f"deterministic: not paced ({wall:.2f} s host)")
    lines, emu_s, wall, cpu, p = run(emu, P("HALT.COM"), ["--cpu=486dx2"])
    ticks = hexwords(lines[0])[0] if lines else -1
    check(ticks == 100, f"auto: 100 HLTs = {ticks} ticks")
    check(cpu < 0.5 * wall, f"auto: HLT idles the host ({cpu:.2f} s CPU in {wall:.2f} s)")
    lines, emu_s, wall, cpu, p = run(emu, P("HALT.COM"), ["--throttle-machine=486dx-33"])
    check(cpu < 0.5 * wall, f"fixed: HLT idles the host ({cpu:.2f} s CPU in {wall:.2f} s)")

    # Date and time.
    out1 = run(emu, P("CLOCK.COM"), ["--deterministic"])[0]
    out2 = run(emu, P("CLOCK.COM"), ["--deterministic"])[0]
    check(out1 == out2 and len(out1) >= 5, "deterministic: two runs print the same date/time")
    try:
        t0, d0, w0 = hexwords(out1[0])
        set_al = hexwords(out1[1])[0]
        t1, d1, w1 = hexwords(out1[2])
        flag, ticks, bda = hexwords(out1[3])
        rtc_t, rtc_d = hexwords(out1[4])
    except (IndexError, ValueError):
        check(False, f"CLOCK.COM output parses: {out1}")
        return 1
    check(t0 >> 16 == 0x0C00 and (t0 >> 8) & 0xFF == 0,
          f"deterministic start time 12:00:00 (AH=2Ch -> {t0:08X})")
    check(d0 == (1994 << 16 | 6 << 8 | 1) and w0 == 3,
          f"deterministic start date 1994-06-01, Wednesday ({d0:08X} {w0})")
    check(set_al == 0, "AH=2Dh accepts 23:59:58.00")
    check(t1 >> 16 == 0 and (t1 >> 8) & 0xFF in (0, 1),
          f"40 ticks after 23:59:58 it is 00:00:0x ({t1:08X})")
    check(d1 == (1994 << 16 | 6 << 8 | 2) and w1 == 4, f"midnight advanced the DOS date ({d1:08X} {w1})")
    check(flag == 0 and ticks == bda and ticks < 40,
          f"INT 1Ah AH=00h = BDA tick count ({ticks} / {bda}), midnight flag consumed by DOS")
    check(rtc_t >> 16 == 0x0000 and rtc_d == 0x19940602,
          f"INT 1Ah AH=02h/04h RTC follows AH=2Dh and midnight ({rtc_t:08X} {rtc_d:08X})")
    out3 = run(emu, P("CLOCK.COM"), ["--deterministic", "--start-time=2001-02-28T08:30:00"])[0]
    check(out3 and hexwords(out3[0])[0] >> 16 == 0x081E and hexwords(out3[0])[1] == (2001 << 16 | 2 << 8 | 28),
          f"--start-time sets the DOS clock ({out3[:1]})")

    # Blocking keyboard read.
    lines, emu_s, wall, cpu, p = run(emu, P("GETKEY.COM"), ["--deterministic", "--keys=2.5:A"])
    check(lines and hexwords(lines[0])[0] & 0xFF == ord("a") and emu_s and 2.5 <= emu_s < 2.7,
          f"deterministic: INT 16h waits for the scripted key at 2.5 s ({lines[:1]}, {emu_s} s)")
    check(wall < 2.0, f"deterministic: waiting for a key is not paced ({wall:.2f} s host)")
    lines, emu_s, wall, cpu, p = run(emu, P("GETKEY.COM"), ["--keys=1.5:A"])
    check(lines and hexwords(lines[0])[0] & 0xFF == ord("a"), f"auto: INT 16h gets the key ({lines[:1]})")
    check(cpu < 0.5 * wall, f"auto: INT 16h wait idles the host ({cpu:.2f} s CPU in {wall:.2f} s)")

    print("all passed" if FAILURES == 0 else f"{FAILURES} failed")
    return 1 if FAILURES else 0


if __name__ == "__main__":
    sys.exit(main())
