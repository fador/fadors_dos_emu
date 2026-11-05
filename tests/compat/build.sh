#!/bin/bash
# Build the DOS-extender compatibility programs.
#
#   tests/compat/build.sh <out-dir>
#
# Needs Open Watcom v2 (WATCOM set, wcl386 on PATH) and, optionally, DJGPP
# (i586-pc-msdosdjgpp-gcc on PATH).  Every program is linked once per
# extender; the extender runtimes that must sit next to the programs
# (DOS4GW.EXE) are copied from the Watcom installation.
set -euo pipefail
src="$(cd "$(dirname "$0")" && pwd)"
out="${1:?usage: build.sh <out-dir>}"
mkdir -p "$out"
cd "$out"

: "${WATCOM:?set WATCOM to the Open Watcom installation}"
export INCLUDE="${INCLUDE:-$WATCOM/h}"
export PATH="$PATH:$WATCOM/binw"   # wlink finds the extender stubs here

for prog in hello32 irq32; do
  for sys in dos4g pmodew dos32a causeway; do
    wcl386 -q -bt=dos -l=$sys -fe="${prog}_${sys}.exe" "$src/$prog.c" >/dev/null
  done
done
cp "$WATCOM/binw/dos4gw.exe" DOS4GW.EXE
rm -f ./*.o ./*.obj

if command -v i586-pc-msdosdjgpp-gcc >/dev/null; then
  i586-pc-msdosdjgpp-gcc -O2 "$src/clocktest.c" -o clock_djgpp.exe
  i586-pc-msdosdjgpp-gcc -O2 "$src/hello32.c" -o hello32_djgpp.exe -lm
else
  echo "DJGPP not found; skipping DJGPP programs" >&2
fi
ls "$out"
