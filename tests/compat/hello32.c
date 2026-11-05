#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
int main(int argc, char **argv) {
  int i; double s = 0; char *p;
  printf("hello32 argc=%d argv1=%s\n", argc, argc > 1 ? argv[1] : "-");
  for (i = 1; i < 1000; i++) s += sin(i) * sqrt(i);
  printf("s=%.6f\n", s);
  p = malloc(4 * 1024 * 1024);
  if (!p) { printf("malloc failed\n"); return 2; }
  memset(p, 7, 4 * 1024 * 1024);
  printf("malloc ok %d\n", p[4 * 1024 * 1024 - 1]);
  { FILE *f = fopen("W32.TXT", "w"); fprintf(f, "abc %d\n", 42); fclose(f);
    f = fopen("w32.txt", "r"); fgets(p, 100, f); fclose(f); printf("file: %s", p); remove("w32.txt"); }
  return 3;
}
