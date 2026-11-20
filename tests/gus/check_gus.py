#!/usr/bin/env python3
"""End-to-end check of the Gravis Ultrasound emulation (--gus).

usage: check_gus.py <fadors_emu> [--keep DIR]

Generates tiny .COM programs (hand-assembled below, no DOS toolchain needed),
runs them headless and checks:
  - with --gus the DOS environment has ULTRASND=240,3,3,11,11 and
    ULTRADIR=C:\\ULTRASND, without it neither;
  - a program that resets the GF1, uploads one period of an unsigned 8-bit
    sine (8 periods, 256 bytes) to DRAM through 8237 channel 3 with the
    invert-MSB bit, polls the DMA terminal count in register 41h, selects
    32 active voices and loops the period on voice 0 at FC=1024 for 1 s is
    recorded with --audio-wav: the tone is 617400/32/32 = 602.9 Hz and its
    peak level is volume F00h (x0.5) x the -6 dB output gain x the -3 dB
    centre pan = 0.175;
  - the deterministic recording is identical with and without
    --no-fast-path;
  - with IRQ 0 masked, HLT wakes for the GUS IRQ 11: a voice with the
    wavetable IRQ enabled reaches its end after 44100 samples at 14 voices
    (1.0 s), the INT 73h handler reads the IRQ source (8Fh: voice 0,
    wavetable) and the program exits after ~1 s of emulated time.
Exits non-zero on the first failure.
"""
import argparse, math, os, re, struct, subprocess, sys, tempfile, wave

BASE = 0x240
REG = BASE + 0x103   # register select
LO = BASE + 0x104    # data low (16-bit writes)
HI = BASE + 0x105    # data high (8-bit writes)
EXPECTED_HZ = 617400 / 32 / 32
EXPECTED_PEAK = 0.5 * 0.5 * 127 / 128 * math.sqrt(0.5)  # centre pan: -3 dB


class Asm:
    """Minimal helper: raw bytes, labels, rel8 jumps, abs16 refs."""

    def __init__(self):
        self.items = []

    def b(self, *bs):
        self.items.append(("b", bytes(bs)))
        return self

    def label(self, name):
        self.items.append(("l", name))
        return self

    def jr(self, opcode, name):
        self.items.append(("j", opcode, name))
        return self

    def addr(self, name):
        self.items.append(("a", name))
        return self

    def build(self):
        sizes = {"l": 0, "j": 2, "a": 2}
        labels, pc = {}, 0x100
        for it in self.items:
            if it[0] == "l":
                labels[it[1]] = pc
            pc += len(it[1]) if it[0] == "b" else sizes[it[0]]
        out, pc = bytearray(), 0x100
        for it in self.items:
            if it[0] == "b":
                out += it[1]
            elif it[0] == "j":
                rel = labels[it[2]] - (pc + 2)
                assert -128 <= rel <= 127, it
                out += bytes([it[1], rel & 0xFF])
            elif it[0] == "a":
                out += struct.pack("<H", labels[it[1]])
            pc += len(it[1]) if it[0] == "b" else sizes[it[0]]
        return bytes(out)

    # Helpers
    def mov_dx(self, v):
        return self.b(0xBA, v & 0xFF, v >> 8)

    def mov_al(self, v):
        return self.b(0xB0, v)

    def mov_ax(self, v):
        return self.b(0xB8, v & 0xFF, v >> 8)

    def outb(self, port, v):  # mov dx,port; mov al,v; out dx,al
        return self.mov_dx(port).mov_al(v).b(0xEE)

    def gus8(self, reg, v):
        return self.outb(REG, reg).outb(HI, v)

    def gus16(self, reg, v):
        return self.outb(REG, reg).mov_dx(LO).mov_ax(v).b(0xEF)


def env_program():
    """Prints the environment block, one variable per line."""
    a = Asm()
    a.b(0x8E, 0x06, 0x2C, 0x00)  # mov es,[2Ch]
    a.b(0x31, 0xF6)              # xor si,si
    a.label("next")
    a.b(0x26, 0x8A, 0x14)        # mov dl,es:[si]
    a.b(0x46)                    # inc si
    a.b(0x84, 0xD2)              # test dl,dl
    a.jr(0x74, "eol")            # jz eol
    a.b(0xB4, 0x02, 0xCD, 0x21)  # mov ah,2; int 21h
    a.jr(0xEB, "next")
    a.label("eol")
    a.b(0xB4, 0x02, 0xB2, 0x0D, 0xCD, 0x21)  # CR
    a.b(0xB4, 0x02, 0xB2, 0x0A, 0xCD, 0x21)  # LF
    a.b(0x26, 0x80, 0x3C, 0x00)  # cmp byte es:[si],0
    a.jr(0x75, "next")
    a.b(0xB8, 0x00, 0x4C, 0xCD, 0x21)
    return a.build()


def tone_program():
    a = Asm()
    # GF1 reset, then running with the DAC on and IRQs off.
    a.gus8(0x4C, 0x00)
    a.gus8(0x4C, 0x01)
    # Physical address of `buf`: AX = low 16 bits, BL = page.
    a.b(0x8C, 0xC8)              # mov ax,cs
    a.b(0x89, 0xC3)              # mov bx,ax
    a.b(0xB1, 0x04, 0xD3, 0xE0)  # mov cl,4; shl ax,cl
    a.b(0xB1, 0x0C, 0xD3, 0xEB)  # mov cl,12; shr bx,cl
    a.b(0x05).addr("buf")        # add ax,buf
    a.b(0x80, 0xD3, 0x00)        # adc bl,0
    a.b(0x89, 0xC7)              # mov di,ax
    # 8237 channel 3: single-cycle memory -> device, 256 bytes.
    a.mov_al(0x07).b(0xE6, 0x0A)  # mask
    a.mov_al(0x4B).b(0xE6, 0x0B)  # mode
    a.b(0xE6, 0x0C)               # clear flip-flop
    a.b(0x89, 0xF8, 0xE6, 0x06)   # mov ax,di; out 06h,al
    a.b(0x88, 0xE0, 0xE6, 0x06)   # mov al,ah; out 06h,al
    a.b(0x88, 0xD8, 0xE6, 0x82)   # mov al,bl; out 82h,al
    a.mov_al(0xFF).b(0xE6, 0x07)
    a.mov_al(0x00).b(0xE6, 0x07)
    # GUS DMA to DRAM 0: go, invert MSB, 8-bit, fastest rate (no IRQ).
    a.gus16(0x42, 0x0000)
    a.gus8(0x41, 0x81)
    a.mov_al(0x03).b(0xE6, 0x0A)  # unmask: the transfer runs
    a.label("poll")
    a.outb(REG, 0x41)
    a.mov_dx(HI).b(0xEC)          # in al,dx
    a.b(0xA8, 0x40)               # test al,40h
    a.jr(0x74, "poll")
    # 32 active voices; voice 0 loops 32 samples at 1 sample per tick.
    a.gus8(0x0E, 0xC0 | 31)
    a.outb(BASE + 0x102, 0)
    a.gus8(0x00, 0x03)
    a.gus16(0x01, 1024)
    a.gus16(0x02, 0x0000).gus16(0x03, 0x0000)
    a.gus16(0x04, 0x0000).gus16(0x05, 32 << 9)
    a.gus16(0x0A, 0x0000).gus16(0x0B, 0x0000)
    a.gus16(0x09, 0xF000)
    a.gus8(0x0C, 7)
    a.gus8(0x0D, 0x03)
    a.gus8(0x4C, 0x03)            # run, DAC on
    a.outb(BASE, 0x08)            # line out on, latches on
    a.gus8(0x00, 0x08)            # loop, go
    # Wait 1 s (INT 15h AH=86h, CX:DX microseconds), stop, exit.
    a.b(0xB4, 0x86, 0xB9, 0x0F, 0x00, 0xBA, 0x40, 0x42, 0xCD, 0x15)
    a.gus8(0x00, 0x03)
    a.b(0xB8, 0x00, 0x4C, 0xCD, 0x21)
    a.label("buf")
    sine = [128 + round(127 * math.sin(2 * math.pi * i / 32)) for i in range(256)]
    a.b(*sine)
    return a.build()


def irq_program():
    a = Asm()
    a.b(0xFA)                                # cli
    a.b(0x31, 0xC0, 0x8E, 0xC0)              # xor ax,ax; mov es,ax
    a.b(0x26, 0xC7, 0x06, 0xCC, 0x01).addr("handler")  # INT 73h offset
    a.b(0x26, 0x8C, 0x0E, 0xCE, 0x01)        # INT 73h segment = cs
    a.b(0xE4, 0x21, 0x0C, 0x01, 0x24, 0xFB, 0xE6, 0x21)  # mask IRQ0, unmask IRQ2
    a.b(0xE4, 0xA1, 0x24, 0xF7, 0xE6, 0xA1)  # unmask IRQ11
    a.gus8(0x4C, 0x00)
    a.gus8(0x4C, 0x01)
    a.outb(BASE + 0x102, 0)
    a.gus8(0x00, 0x03)
    a.gus16(0x01, 1024)
    a.gus16(0x02, 0).gus16(0x03, 0)
    a.gus16(0x04, 44100 >> 7).gus16(0x05, (44100 & 0x7F) << 9)
    a.gus16(0x0A, 0).gus16(0x0B, 0)
    a.gus16(0x09, 0x0000)
    a.gus8(0x0D, 0x03)
    a.gus8(0x4C, 0x07)                       # run, DAC, IRQs
    a.outb(BASE, 0x08)                       # latches on
    a.gus8(0x00, 0x20)                       # go: IRQ at the end, no loop
    a.b(0xFB)                                # sti
    a.label("wait")
    a.b(0xF4)                                # hlt
    a.b(0x80, 0x3E).addr("flag").b(0x00)     # cmp byte [flag],0
    a.jr(0x74, "wait")
    a.b(0xE4, 0x21, 0x24, 0xFE, 0xE6, 0x21)  # unmask IRQ0 again
    a.b(0xA0).addr("flag")                   # mov al,[flag]
    a.b(0x24, 0xC0, 0x3C, 0x40)              # and al,0C0h; cmp al,40h
    a.b(0xB2, ord("X"))
    a.jr(0x75, "print")
    a.b(0xB2, ord("W"))
    a.label("print")
    a.b(0xB4, 0x02, 0xCD, 0x21)
    a.b(0xB8, 0x00, 0x4C, 0xCD, 0x21)
    a.label("handler")
    a.b(0x50, 0x52)                          # push ax; push dx
    a.mov_dx(BASE + 6).b(0xEC)               # IRQ status
    a.outb(REG, 0x8F)
    a.mov_dx(HI).b(0xEC)                     # IRQ source
    a.b(0x2E, 0xA2).addr("flag")             # mov cs:[flag],al
    a.b(0xB0, 0x20, 0xE6, 0xA0, 0xE6, 0x20)  # EOI slave, master
    a.b(0x5A, 0x58, 0xCF)                    # pop dx; pop ax; iret
    a.label("flag")
    a.b(0x00)
    return a.build()


def run(emu, prog, opts, max_seconds=20):
    p = subprocess.run([emu, "--headless", f"--max-seconds={max_seconds}"] +
                       opts + [prog], capture_output=True, text=True,
                       cwd=os.path.dirname(prog))
    return p


def fail(msg, p=None):
    print("FAIL:", msg)
    if p is not None:
        print(p.stdout[-2000:])
        print(p.stderr[-4000:])
    sys.exit(1)


def read_wav(path):
    with wave.open(path, "rb") as w:
        assert w.getnchannels() == 2 and w.getsampwidth() == 2
        rate = w.getframerate()
        raw = w.readframes(w.getnframes())
    n = len(raw) // 4
    data = struct.unpack("<%dh" % (2 * n), raw)
    return rate, data[0::2], data[1::2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("emu")
    ap.add_argument("--keep")
    a = ap.parse_args()
    emu = os.path.abspath(a.emu)
    d = a.keep or tempfile.mkdtemp(prefix="guscheck")
    os.makedirs(d, exist_ok=True)

    envp = os.path.join(d, "ENV.COM")
    with open(envp, "wb") as f:
        f.write(env_program())
    p = run(emu, envp, ["--deterministic", "--gus"])
    if "ULTRASND=240,3,3,11,11" not in p.stdout or \
            "ULTRADIR=C:\\ULTRASND" not in p.stdout:
        fail("ULTRASND/ULTRADIR missing with --gus", p)
    p = run(emu, envp, ["--deterministic", "--gus=220,irq=5,dma=1"])
    if "ULTRASND=220,1,1,5,5" not in p.stdout:
        fail("--gus=220,irq=5,dma=1 not reflected in ULTRASND", p)
    p = run(emu, envp, ["--deterministic"])
    if "ULTRASND" in p.stdout:
        fail("ULTRASND set without --gus", p)
    print("ok   environment")

    tone = os.path.join(d, "GUSTONE.COM")
    with open(tone, "wb") as f:
        f.write(tone_program())
    wavs = []
    for extra in ([], ["--no-fast-path"]):
        wav = os.path.join(d, "tone%d.wav" % len(wavs))
        p = run(emu, tone, ["--deterministic", "--gus", "--audio-wav=" + wav]
                + extra)
        if p.returncode != 0:
            fail("tone program did not exit cleanly (%d)" % p.returncode, p)
        wavs.append(wav)
    rate, left, right = read_wav(wavs[0])
    loud = [i for i, s in enumerate(left) if abs(s) > 1000]
    if not loud:
        fail("recording is silent", p)
    first, last = loud[0], loud[-1]
    dur = (last - first) / rate
    if not 0.95 < dur < 1.05:
        fail("tone lasts %.3f s, expected 1 s" % dur)
    # Steady part: skip 50 ms at both ends.
    lo, hi = first + rate // 20, last - rate // 20
    seg = left[lo:hi]
    ups = [i for i in range(1, len(seg)) if seg[i - 1] < 0 <= seg[i]]
    # Frequency from the first to the last upward zero crossing.
    freq = (len(ups) - 1) * rate / (ups[-1] - ups[0])
    peak = max(abs(s) for s in seg) / 32767
    rms = math.sqrt(sum(s * s for s in seg) / len(seg)) / 32767
    print("     tone %.2f Hz (expected %.2f), peak %.4f (expected %.4f), "
          "rms %.4f" % (freq, EXPECTED_HZ, peak, EXPECTED_PEAK, rms))
    if abs(freq / EXPECTED_HZ - 1) > 0.002:
        fail("tone frequency")
    if abs(peak / EXPECTED_PEAK - 1) > 0.03:
        fail("tone peak level")
    if abs(rms / (EXPECTED_PEAK / math.sqrt(2)) - 1) > 0.03:
        fail("tone rms level")
    if list(left[lo:hi]) != list(right[lo:hi]):
        fail("centre pan: left and right differ")
    print("ok   DMA upload + looped voice recorded at the expected pitch/level")
    with open(wavs[0], "rb") as f0, open(wavs[1], "rb") as f1:
        if f0.read() != f1.read():
            fail("recording differs with --no-fast-path")
    print("ok   deterministic with and without --no-fast-path")

    irqp = os.path.join(d, "GUSIRQ.COM")
    with open(irqp, "wb") as f:
        f.write(irq_program())
    p = run(emu, irqp, ["--deterministic", "--gus"], max_seconds=5)
    m = re.search(r"Emulated time ([0-9.e+-]+) s .* in ([0-9]+) HLTs",
                  p.stderr)
    if p.returncode != 0 or not m:
        fail("IRQ program did not finish (HLT not woken by the GUS?)", p)
    emu_s, halts = float(m.group(1)), int(m.group(2))
    if p.stdout.strip() != "W":
        fail("IRQ source was not a voice 0 wavetable IRQ", p)
    # The program needs ~50 us before the voice starts; without the GUS
    # wake source HLT would only notice the IRQ at the next 1 ms wait slice.
    if not 1.0 <= emu_s < 1.0003 or halts < 1:
        fail("IRQ after %.6f s of emulated time, %d HLTs" % (emu_s, halts), p)
    print("ok   HLT woken by the wavetable IRQ after %.6f s (%d HLTs)" %
          (emu_s, halts))
    print("all passed")


if __name__ == "__main__":
    main()
