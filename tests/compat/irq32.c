/* Extender IRQ regression test: hooks INT 8 (timer) and INT 9 (keyboard) with
 * Watcom __interrupt handlers, like DOOM and most DOS/4GW games do. */
#include <stdio.h>
#include <stdlib.h>
#include <dos.h>
#include <conio.h>
#include <i86.h>
#include <string.h>

static void (__interrupt __far *old_timer)(void);
static void (__interrupt __far *old_kbd)(void);
static volatile unsigned long ticks = 0;
static volatile unsigned long chained = 0;
static volatile int keys = 0;
static volatile unsigned char lastScan = 0;

static void __interrupt __far timer_isr(void) {
  ticks++;
  if ((ticks & 3) == 0) { chained++; _chain_intr(old_timer); }
  outp(0x20, 0x20);
}

static void __interrupt __far kbd_isr(void) {
  unsigned char sc = inp(0x60);
  lastScan = sc;
  if (!(sc & 0x80)) keys++;
  outp(0x20, 0x20);
}

int main(int argc, char **argv) {
  unsigned long start;
  volatile unsigned long *biosTicks = (unsigned long *)0x46C;
  unsigned long b0;
  int i, ok = 1;
  char *big;
  FILE *f = fopen("RESULT4G.TXT", "w");
  /* reprogram PIT to 72.8 Hz (4x) like many games */
  outp(0x43, 0x36); outp(0x40, 0x00); outp(0x40, 0x40);
  old_timer = _dos_getvect(0x08);
  old_kbd = _dos_getvect(0x09);
  _dos_setvect(0x08, timer_isr);
  _dos_setvect(0x09, kbd_isr);
  b0 = *biosTicks;
  start = ticks;
  while (ticks - start < 146) { /* ~2 s at 72.8 Hz */
    for (i = 0; i < 1000; i++) ;
  }
  _dos_setvect(0x08, old_timer);
  _dos_setvect(0x09, old_kbd);
  outp(0x43, 0x36); outp(0x40, 0x00); outp(0x40, 0x00);
  big = malloc(8 * 1024 * 1024);
  if (big) { memset(big, 0x5A, 8 * 1024 * 1024); ok &= big[8*1024*1024-1] == 0x5A; }
  fprintf(f, "ticks=%lu chained=%lu bios=%lu keys=%d last=%02x big=%s\n", ticks - start,
          chained, *biosTicks - b0, keys, lastScan, big ? "ok" : "fail");
  printf("ticks=%lu chained=%lu bios=%lu keys=%d last=%02x big=%s\n", ticks - start,
          chained, *biosTicks - b0, keys, lastScan, big ? "ok" : "fail");
  fclose(f);
  return ok ? 0 : 1;
}
