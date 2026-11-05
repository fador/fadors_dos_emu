/* Emulator performance profiles.  usage: perfbench <int|mem|fpu|call|vga|str|all> [scale]
 * Each profile does a fixed amount of work, so host time / instruction
 * counts are comparable between emulator builds. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <dpmi.h>
#include <go32.h>
#include <sys/farptr.h>
#include <sys/movedata.h>

static volatile unsigned sink;

static unsigned prof_int(int n) {          /* ALU / branches / multiply */
  unsigned a = 1, b = 7, c = 0, i, j;
  for (j = 0; j < (unsigned)n; j++)
    for (i = 0; i < 100000; i++) {
      a = a * 1103515245u + 12345u;
      b ^= a >> 7;
      if (a & 0x100) c += b; else c -= a;
      c = (c << 3) | (c >> 29);
    }
  return a ^ b ^ c;
}

static unsigned prof_mem(int n) {          /* memcpy/memset/array walks */
  static unsigned char buf1[65536], buf2[65536];
  unsigned s = 0, i, j;
  for (j = 0; j < (unsigned)n; j++) {
    memset(buf1, j, sizeof buf1);
    memcpy(buf2, buf1, sizeof buf2);
    for (i = 0; i < sizeof buf2; i += 4) s += buf2[i] + (buf1[i ^ 0x55] << 1);
  }
  return s;
}

static unsigned prof_fpu(int n) {          /* x87 math */
  double x = 0.5, y = 0; int i, j;
  for (j = 0; j < n; j++)
    for (i = 1; i < 20000; i++) {
      y += sin(x) * cos(x) + sqrt(x * i) / (1.0 + i);
      x = fmod(x * 1.0001 + 0.001, 6.28);
    }
  return (unsigned)y;
}

static unsigned fib(unsigned k) { return k < 2 ? k : fib(k - 1) + fib(k - 2); }
static unsigned prof_call(int n) {         /* calls / stack */
  unsigned s = 0; int j;
  for (j = 0; j < n; j++) s += fib(20);
  return s;
}

static unsigned prof_vga(int n) {          /* mode 13h framebuffer writes */
  __dpmi_regs r; int f, x, y; unsigned char row[320];
  r.x.ax = 0x13; __dpmi_int(0x10, &r);
  for (f = 0; f < n * 4; f++)
    for (y = 0; y < 200; y++) {
      for (x = 0; x < 320; x++) row[x] = (unsigned char)(x + y + f);
      dosmemput(row, 320, 0xA0000 + y * 320);
    }
  for (f = 0; f < n; f++)
    for (x = 0; x < 64000; x++) _farpokeb(_dos_ds, 0xA0000 + x, (unsigned char)(x + f));
  r.x.ax = 0x03; __dpmi_int(0x10, &r);
  return 0;
}

static unsigned prof_str(int n) {          /* string functions */
  char a[256], b[256]; unsigned s = 0; int i, j;
  for (i = 0; i < 255; i++) a[i] = 'a' + i % 26; a[255] = 0;
  for (j = 0; j < n * 2000; j++) {
    strcpy(b, a); b[j % 255] = 'Z';
    s += strlen(b) + (strcmp(a, b) > 0) + (strchr(b, 'Z') - b);
  }
  return s;
}

int main(int argc, char **argv) {
  const char *p = argc > 1 ? argv[1] : "all";
  int n = argc > 2 ? atoi(argv[2]) : 20;
  int all = !strcmp(p, "all");
  if (all || !strcmp(p, "int")) sink += prof_int(n);
  if (all || !strcmp(p, "mem")) sink += prof_mem(n);
  if (all || !strcmp(p, "fpu")) sink += prof_fpu(n);
  if (all || !strcmp(p, "call")) sink += prof_call(n);
  if (all || !strcmp(p, "str")) sink += prof_str(n);
  if (all || !strcmp(p, "vga")) sink += prof_vga(n);
  printf("perfbench %s %d done (%u)\n", p, n, sink);
  return 0;
}
