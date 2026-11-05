/* 16-bit real-mode performance profile (Open Watcom, small model) */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
static unsigned char far *vga = (unsigned char far *)0xA0000000L;
int main(int argc, char **argv) {
  unsigned a = 1, b = 7, c = 0, i, j, n = argc > 1 ? atoi(argv[1]) : 20;
  static char buf1[16384], buf2[16384];
  long s = 0;
  for (j = 0; j < n; j++)
    for (i = 0; i < 30000; i++) {
      a = a * 25173u + 13849u;
      b ^= a >> 3;
      if (a & 0x10) c += b; else c -= a;
      s += (long)(c & 0xFF) * (b & 0x7F);
    }
  for (j = 0; j < n; j++) { memset(buf1, j, sizeof buf1); memcpy(buf2, buf1, sizeof buf2); s += buf2[j]; }
  for (j = 0; j < n; j++) { unsigned k; for (k = 0; k < 64000u; k++) vga[k] = (unsigned char)(k + j); }
  printf("perf16 %u done (%ld)\n", n, s ^ a ^ c);
  return 0;
}
