/* Sound Blaster / AdLib regression test.
 *
 * Builds as a 16-bit real-mode program (wcl -ml) and as a 32-bit DOS/4GW
 * program (wcl386 -l=dos4g).  Checks DSP reset/version/identify, IRQ
 * delivery, 8-bit single-cycle and auto-init DMA, SB16 16-bit auto-init
 * DMA, direct DAC output, the mixer's IRQ/DMA registers, and OPL2/OPL3
 * detection at 388h and at the Sound Blaster's own OPL ports, then plays
 * an FM note.  One "name ok|FAIL detail" line per check; run the emulator
 * with --audio-wav to listen to (or measure) the result. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <conio.h>
#include <dos.h>
#include <i86.h>
#include <math.h>

#ifdef __386__
#define FAR
static volatile unsigned long *ticks = (unsigned long *)0x46C;
#else
#define FAR __far
static volatile unsigned long FAR *ticks = (unsigned long FAR *)MK_FP(0x40, 0x6C);
#endif

static unsigned base = 0x220, irq = 5, dma8 = 1, dma16 = 5;
static volatile unsigned irqCount = 0;
static void (__interrupt __far *oldIrq)(void);
static int failures = 0;

static void result(const char *name, int ok, const char *fmt, unsigned long a,
                   unsigned long b) {
  printf("%-14s %s ", name, ok ? "ok" : "FAIL");
  printf(fmt, a, b);
  printf("\n");
  if (!ok) failures++;
}

static void __interrupt __far sbIrq(void) {
  irqCount++;
  inp(base + 0x0E); /* acknowledge 8-bit */
  inp(base + 0x0F); /* acknowledge 16-bit */
  if (irq >= 8) outp(0xA0, 0x20);
  outp(0x20, 0x20);
}

static void waitTicks(unsigned long n) {
  unsigned long t = *ticks;
  while (*ticks - t < n) ;
}

static int dspWrite(unsigned char v) {
  long i;
  for (i = 0; i < 100000L; i++)
    if (!(inp(base + 0x0C) & 0x80)) { outp(base + 0x0C, v); return 1; }
  return 0;
}

static int dspRead(void) {
  long i;
  for (i = 0; i < 100000L; i++)
    if (inp(base + 0x0E) & 0x80) return inp(base + 0x0A);
  return -1;
}

static int dspReset(void) {
  int i;
  outp(base + 6, 1);
  for (i = 0; i < 100; i++) inp(base + 6);
  outp(base + 6, 0);
  return dspRead();
}

/* DMA buffer in conventional memory: 32 KB that does not cross a 64 KB
 * physical page (allocate 64 KB and pick the right half). */
static unsigned char FAR *dmaBuf;
static unsigned long dmaPhys;

static int allocDma(void) {
#ifdef __386__
  union REGS r;
  memset(&r, 0, sizeof r);
  r.w.ax = 0x0100;
  r.w.bx = 0x1000; /* 64 KB */
  int386(0x31, &r, &r);
  if (r.x.cflag) return 0;
  dmaPhys = (unsigned long)r.w.ax << 4;
#else
  unsigned seg;
  if (_dos_allocmem(0x1000, &seg) != 0) return 0;
  dmaPhys = (unsigned long)seg << 4;
#endif
  if ((dmaPhys & 0xFFFF) + 0x8000 > 0x10000)
    dmaPhys = (dmaPhys + 0xFFFF) & ~0xFFFFUL;
#ifdef __386__
  dmaBuf = (unsigned char *)dmaPhys;
#else
  dmaBuf = (unsigned char FAR *)MK_FP((unsigned)(dmaPhys >> 4), 0);
#endif
  return 1;
}

static void program8(unsigned long len, int autoInit) {
  static const unsigned char page[4] = {0x87, 0x83, 0x81, 0x82};
  outp(0x0A, 4 | dma8);
  outp(0x0C, 0);
  outp(0x0B, (autoInit ? 0x58 : 0x48) | dma8);
  outp(dma8 * 2, (unsigned char)dmaPhys);
  outp(dma8 * 2, (unsigned char)(dmaPhys >> 8));
  outp(page[dma8], (unsigned char)(dmaPhys >> 16));
  outp(dma8 * 2 + 1, (unsigned char)(len - 1));
  outp(dma8 * 2 + 1, (unsigned char)((len - 1) >> 8));
  outp(0x0A, dma8);
}

static void program16(unsigned long words, int autoInit) {
  static const unsigned char page[4] = {0x8F, 0x8B, 0x89, 0x8A};
  unsigned ch = dma16 - 4;
  unsigned long addr = (dmaPhys >> 1) & 0xFFFF;
  outp(0xD4, 4 | ch);
  outp(0xD8, 0);
  outp(0xD6, (autoInit ? 0x58 : 0x48) | ch);
  outp(0xC0 + ch * 4, (unsigned char)addr);
  outp(0xC0 + ch * 4, (unsigned char)(addr >> 8));
  outp(page[ch], (unsigned char)((dmaPhys >> 16) & 0xFE));
  outp(0xC2 + ch * 4, (unsigned char)(words - 1));
  outp(0xC2 + ch * 4, (unsigned char)((words - 1) >> 8));
  outp(0xD4, ch);
}

static void fillSine8(unsigned len, double hz, double rate) {
  unsigned i;
  for (i = 0; i < len; i++)
    dmaBuf[i] = (unsigned char)(128 + 100 * sin(2 * 3.14159265 * hz * i / rate));
}

static void fillSine16(unsigned words, double hz, double rate) {
  unsigned i;
  short FAR *p = (short FAR *)dmaBuf;
  for (i = 0; i < words; i++)
    p[i] = (short)(20000 * sin(2 * 3.14159265 * hz * i / rate));
}

static void oplWrite(unsigned port, int reg, int val) {
  int i;
  outp(port, reg);
  for (i = 0; i < 6; i++) inp(port);
  outp(port + 1, val);
  for (i = 0; i < 35; i++) inp(port);
}

static int oplDetect(unsigned port) {
  int s1, s2, i;
  oplWrite(port, 4, 0x60);
  oplWrite(port, 4, 0x80);
  s1 = inp(port);
  oplWrite(port, 2, 0xFF);
  oplWrite(port, 4, 0x21);
  for (i = 0; i < 200; i++) inp(port); /* > 80 us */
  s2 = inp(port);
  oplWrite(port, 4, 0x60);
  oplWrite(port, 4, 0x80);
  return (s1 & 0xE0) == 0 && (s2 & 0xE0) == 0xC0 ? (s2 & 0x06) == 0 ? 3 : 2 : 0;
}

int main(void) {
  char *blaster = getenv("BLASTER");
  int v, maj, minr;
  long i;
  unsigned long t0, n;
  unsigned char picBit;

  if (blaster) {
    char *p;
    for (p = blaster; *p; p++) {
      if (*p == 'A' || *p == 'a') base = (unsigned)strtoul(p + 1, NULL, 16);
      if (*p == 'I' || *p == 'i') irq = (unsigned)strtoul(p + 1, NULL, 10);
      if (*p == 'D' || *p == 'd') dma8 = (unsigned)strtoul(p + 1, NULL, 10);
      if (*p == 'H' || *p == 'h') dma16 = (unsigned)strtoul(p + 1, NULL, 10);
    }
  }
  printf("BLASTER A%X I%u D%u H%u\n", base, irq, dma8, dma16);

  v = dspReset();
  result("dsp-reset", v == 0xAA, "read 0x%02lX%.0lu", (unsigned long)v, 0);
  if (v != 0xAA) return 1;

  dspWrite(0xE1);
  maj = dspRead();
  minr = dspRead();
  result("dsp-version", maj >= 1, "%lu.%02lu", (unsigned long)maj, (unsigned long)minr);

  dspWrite(0xE0);
  dspWrite(0x55);
  v = dspRead();
  result("dsp-identify", v == 0xAA, "E0 55 -> 0x%02lX%.0lu", (unsigned long)v, 0);

  dspWrite(0xE4); dspWrite(0x5A); dspWrite(0xE8);
  v = dspRead();
  result("dsp-testreg", v == 0x5A, "0x%02lX%.0lu", (unsigned long)v, 0);

  if (maj >= 4) {
    outp(base + 4, 0x80); v = inp(base + 5);
    result("mixer-irq", (v & 0x0F) != 0, "reg80=0x%02lX%.0lu", (unsigned long)v, 0);
    outp(base + 4, 0x81); v = inp(base + 5);
    result("mixer-dma", (v & 0xEB) != 0, "reg81=0x%02lX%.0lu", (unsigned long)v, 0);
  }

  /* IRQ hook */
  oldIrq = _dos_getvect(irq < 8 ? 8 + irq : 0x68 + irq);
  _dos_setvect(irq < 8 ? 8 + irq : 0x68 + irq, sbIrq);
  picBit = (unsigned char)(1 << (irq & 7));
  if (irq < 8) outp(0x21, inp(0x21) & ~picBit);
  else { outp(0xA1, inp(0xA1) & ~picBit); outp(0x21, inp(0x21) & ~4); }

  irqCount = 0;
  dspWrite(0xF2);
  t0 = *ticks;
  while (!irqCount && *ticks - t0 < 4) ;
  result("irq-f2", irqCount == 1, "irqs=%lu%.0lu", (unsigned long)irqCount, 0);

  if (!allocDma()) { printf("no DMA buffer\n"); return 1; }
  dspWrite(0xD1); /* speaker on */

  /* 8-bit single cycle: 11025 Hz, 5512 bytes = 0.5 s */
  fillSine8(5512, 440, 11025);
  program8(5512, 0);
  irqCount = 0;
  dspWrite(0x40); dspWrite(256 - 1000000L / 11025);
  dspWrite(0x14); dspWrite((5512 - 1) & 0xFF); dspWrite((5512 - 1) >> 8);
  t0 = *ticks;
  while (!irqCount && *ticks - t0 < 40) ;
  n = *ticks - t0;
  result("dma8-single", irqCount == 1 && n >= 6 && n <= 14,
         "irqs=%lu ticks=%lu (0.5 s = 9)", (unsigned long)irqCount, n);

  /* 8-bit auto-init: 2 blocks of 2205 bytes at 22050 Hz -> 10 IRQs/s */
  fillSine8(8820, 660, 22050);
  program8(4410, 1);
  irqCount = 0;
  dspWrite(0x40); dspWrite(256 - 1000000L / 22050);
  dspWrite(0x48); dspWrite((2205 - 1) & 0xFF); dspWrite((2205 - 1) >> 8);
  dspWrite(0x1C);
  waitTicks(18);
  n = irqCount;
  dspWrite(0xDA);
  waitTicks(3);
  result("dma8-auto", n >= 8 && n <= 12, "irqs in 1 s=%lu%.0lu", n, 0);

  if (maj >= 4) {
    /* SB16 16-bit mono auto-init, 22050 Hz, 2 x 2205 samples */
    fillSine16(4410, 880, 22050);
    program16(4410, 1);
    irqCount = 0;
    dspWrite(0x41); dspWrite(22050 >> 8); dspWrite(22050 & 0xFF);
    dspWrite(0xB6); dspWrite(0x10); dspWrite((2205 - 1) & 0xFF); dspWrite((2205 - 1) >> 8);
    waitTicks(18);
    n = irqCount;
    dspWrite(0xD9);
    waitTicks(3);
    result("dma16-auto", n >= 8 && n <= 12, "irqs in 1 s=%lu%.0lu", n, 0);
  }

  /* Direct DAC: a square wave for ~0.5 s */
  t0 = *ticks;
  i = 0;
  while (*ticks - t0 < 9) {
    int k;
    dspWrite(0x10);
    dspWrite((i++ / 40) & 1 ? 0xE0 : 0x20);
    for (k = 0; k < 20; k++) inp(base + 0x0E);
  }
  result("direct-dac", i > 1000, "writes=%lu%.0lu", (unsigned long)i, 0);

  if (irq < 8) outp(0x21, inp(0x21) | picBit);
  else outp(0xA1, inp(0xA1) | picBit);
  _dos_setvect(irq < 8 ? 8 + irq : 0x68 + irq, oldIrq);

  v = oplDetect(0x388);
  result("opl-388", v >= 2, "OPL%lu%.0lu", (unsigned long)v, 0);
  v = oplDetect(base + 8);
  result("opl-base+8", v >= 2, "OPL%lu%.0lu", (unsigned long)v, 0);
  if (maj >= 3) {
    v = oplDetect(base);
    result("opl-base", v >= 2, "OPL%lu%.0lu", (unsigned long)v, 0);
  }

  /* FM note: channel 0, ~440 Hz, 0.5 s */
  oplWrite(0x388, 0x20, 0x01); oplWrite(0x388, 0x40, 0x10);
  oplWrite(0x388, 0x60, 0xF0); oplWrite(0x388, 0x80, 0x77);
  oplWrite(0x388, 0x23, 0x01); oplWrite(0x388, 0x43, 0x00);
  oplWrite(0x388, 0x63, 0xF0); oplWrite(0x388, 0x83, 0x77);
  oplWrite(0x388, 0xA0, 0x41); oplWrite(0x388, 0xB0, 0x32);
  waitTicks(9);
  oplWrite(0x388, 0xB0, 0x12);
  printf("fm-note        played\n");

  /* PC speaker: a 440 Hz PIT tone for 0.5 s, then 0.5 s bit-banged */
  outp(0x43, 0xB6);
  outp(0x42, (unsigned char)(1193182L / 440));
  outp(0x42, (unsigned char)((1193182L / 440) >> 8));
  outp(0x61, inp(0x61) | 3);
  waitTicks(9);
  outp(0x61, inp(0x61) & ~3);
  t0 = *ticks;
  i = 0;
  while (*ticks - t0 < 9) {
    int k;
    outp(0x61, (inp(0x61) & ~3) | ((i++ & 1) << 1));
    for (k = 0; k < 400; k++) inp(0x61);
  }
  outp(0x61, inp(0x61) & ~3);
  printf("pc-speaker     played\n");

  printf("sbtest %s (%d failures)\n", failures ? "FAILED" : "passed", failures);
  return failures ? 1 : 0;
}
