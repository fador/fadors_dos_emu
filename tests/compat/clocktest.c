#include <stdio.h>
#include <time.h>
#include <dpmi.h>
#include <go32.h>
#include <sys/farptr.h>
int main(void) {
  clock_t c0 = clock(); unsigned long t0 = _farpeekl(_dos_ds, 0x46c), n = 0;
  while (clock() - c0 < 2 * CLOCKS_PER_SEC && n < 50000000UL) { __dpmi_yield(); n++; }
  printf("clock delta=%ld ticks=%lu loops=%lu\n", (long)(clock() - c0), _farpeekl(_dos_ds, 0x46c) - t0, n);
  n = 0; c0 = clock();
  while (clock() - c0 < CLOCKS_PER_SEC && n < 50000000UL) n++;
  printf("busy clock delta=%ld loops=%lu\n", (long)(clock() - c0), n);
  return 0;
}
