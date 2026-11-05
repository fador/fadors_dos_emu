#!/bin/bash
# Run the DOS-extender compatibility programs headless and check their output.
#
#   tests/compat/run.sh <fadors_emu> <dir-built-by-build.sh>
#
# Prints one line per program and exits non-zero if any check failed.
set -uo pipefail
emu="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
dir="${2:?usage: run.sh <fadors_emu> <dir>}"
cd "$dir"
fail=0

# run <exe> <expected exit code> <regex for stdout> [emulator options...]
run() {
  local exe=$1 want_rc=$2 pattern=$3
  shift 3
  [ -f "$exe" ] || { printf '%-22s SKIP (not built)\n' "$exe"; return; }
  local out rc
  out=$(timeout 120 "$emu" --headless --max-seconds=30 "$@" "$exe" 2>/dev/null | tr -d '\r')
  rc=$?
  if [ "$rc" = "$want_rc" ] && grep -Eq "$pattern" <<<"$out"; then
    printf '%-22s ok\n' "$exe"
  else
    printf '%-22s FAIL (rc=%s)\n' "$exe" "$rc"
    tail -3 <<<"$out" | sed 's/^/    /'
    fail=1
  fi
}

for sys in dos4g pmodew dos32a causeway djgpp; do
  # printf, libm, a 4 MB malloc and file I/O through the extender
  run "hello32_$sys.exe" 3 'file: abc 42'
done
for sys in dos4g pmodew dos32a causeway; do
  # INT 8 at 72.8 Hz with chaining, INT 9 with three scripted keys
  run "irq32_$sys.exe" 0 'ticks=14[5-9] chained=3[5-7] bios=3[5-8] keys=3 .*big=ok' \
      --keys=0.5:A,1.0:B,1.5:C
done
run clock_djgpp.exe 0 'clock delta=[1-9]'

# Sound: every DSP/DMA/IRQ/OPL check passes and the recorded output is
# not silent (the emulator logs the peak level of --audio-wav).
for exe in sbtest_rm.exe sbtest_dos4g.exe; do
  [ -f "$exe" ] || { printf '%-22s SKIP (not built)\n' "$exe"; continue; }
  log=$(timeout 120 "$emu" --headless --max-seconds=30 --audio-wav=sbtest.wav "$exe" 2>&1 | tr -d '\r')
  if grep -q 'sbtest passed' <<<"$log" && grep -Eq 'peak (0\.[1-9]|1)' <<<"$log"; then
    printf '%-22s ok\n' "$exe"
  else
    printf '%-22s FAIL\n' "$exe"
    grep -E 'FAIL|peak' <<<"$log" | sed 's/^/    /'
    fail=1
  fi
  rm -f sbtest.wav
done

exit $fail
