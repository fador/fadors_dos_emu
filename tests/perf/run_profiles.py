#!/usr/bin/env python3
"""Run the emulator performance profiles and print a MIPS table.

usage: run_profiles.py <fadors_emu> <dir-with-perfbench.exe, e.g. tests/perf> [--repeat N] [--json out.json]

Each profile executes a fixed workload inside a DOS program (see
perfbench.c / perf16.c); the emulator's own "Executed N instructions in T s"
line is used, so start-up costs of the host process are excluded.
"""
import argparse, json, os, re, statistics, subprocess, sys

PROFILES = [
    # name,        program,          args
    ("int32",   "perfbench.exe", ["int", "100"]),
    ("mem32",   "perfbench.exe", ["mem", "3000"]),
    ("fpu32",   "perfbench.exe", ["fpu", "10"]),
    ("call32",  "perfbench.exe", ["call", "1000"]),
    ("str32",   "perfbench.exe", ["str", "20"]),
    ("vga32",   "perfbench.exe", ["vga", "100"]),
    ("real16",  "perf16.exe",    ["60"]),
]

LINE = re.compile(r"Executed (\d+) instructions in ([0-9.]+) s")

def run(emu, cwd, prog, args):
    p = subprocess.run([emu, "--headless", "--max-seconds=300", prog] + args,
                       cwd=cwd, capture_output=True, text=True)
    m = LINE.search(p.stderr)
    if not m or p.returncode != 0:
        raise RuntimeError(f"{prog} {args} failed (rc={p.returncode})\n{p.stderr[-2000:]}")
    return int(m.group(1)), float(m.group(2))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("emu")
    ap.add_argument("dir")
    ap.add_argument("--repeat", type=int, default=3)
    ap.add_argument("--json")
    ap.add_argument("--only", nargs="*")
    a = ap.parse_args()
    emu = os.path.abspath(a.emu)
    results = {}
    print(f"{'profile':<8} {'instructions':>14} {'time s':>8} {'MIPS':>8}")
    for name, prog, args in PROFILES:
        if a.only and name not in a.only:
            continue
        runs = [run(emu, a.dir, prog, args) for _ in range(a.repeat)]
        instr = runs[0][0]
        t = statistics.median(r[1] for r in runs)
        mips = instr / t / 1e6
        results[name] = {"instructions": instr, "seconds": t, "mips": mips}
        print(f"{name:<8} {instr:>14,} {t:>8.3f} {mips:>8.1f}")
    geo = statistics.geometric_mean([r["mips"] for r in results.values()])
    print(f"{'geomean':<8} {'':>14} {'':>8} {geo:>8.1f}")
    if a.json:
        with open(a.json, "w") as f:
            json.dump({"profiles": results, "geomean_mips": geo}, f, indent=2)

if __name__ == "__main__":
    sys.exit(main())
