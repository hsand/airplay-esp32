#!/usr/bin/env python3
"""Simulate the v0.3.0 PTP clock and the current one on the same seeded
networks and print an old-vs-new table.

Requires Python 3 and a C compiler; no ESP-IDF installation or hardware.
Run from any directory with: python3 tests/ptp_sim/run.py [--seeds N]
"""

import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile
from concurrent.futures import ThreadPoolExecutor

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

SHIMS = (
    "esp_err.h", "esp_log.h", "esp_timer.h", "esp_mac.h",
    "freertos/FreeRTOS.h", "freertos/task.h", "spiram_task.h",
)

SCENARIOS = [
    ("clean", "clean Wi-Fi, +20 ppm"),
    ("wifi", "busy Wi-Fi, +20 ppm"),
    ("wifi-fast50", "busy Wi-Fi, +50 ppm"),
    ("wifi-slow50", "busy Wi-Fi, -50 ppm"),
    ("wifi-nohook", "busy Wi-Fi, no lwIP hook"),
    ("wifi-no8021as", "no 802.1AS Follow_Up TLV"),
    ("clean-passive", "master never answers, clean"),
    ("clean-passive-fast50", "never answers, clean, +50 ppm"),
    ("clean-passive-slow50", "never answers, clean, -50 ppm"),
    ("wifi-passive", "master never answers, busy"),
    ("wifi-passive-fast50", "never answers, busy, +50 ppm"),
    ("sleep", "phone sleeps 4.4 s at 300 s"),
    ("sleep-passive", "sleep, master never answers"),
    ("gm-change", "anchor moves to new GM at 300 s"),
    ("relay", "direct port lost at 200 s, relay"),
    ("multi", "plus a better-priority other GM"),
    ("storm", "25 s storm: 20-60 ms delays, Delay_Resps lost/late"),
]

KEYS = ["mean_us", "p95_us", "p2p_us", "peak_us", "unlocked_pct", "lock_s",
        "settle_s", "recovery_s"]


def build(build_dir, baseline, extra=(), name=None):
    out = build_dir / (name or ("sim-old" if baseline else "sim-new"))
    cmd = shlex.split(os.environ.get("CC", "cc")) + [
        "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-O2", "-g",
        "-Wno-unused-parameter", "-Wno-unused-function",
        "-I", str(build_dir), "-I", str(HERE),
    ]
    cmd += list(extra)
    if baseline:
        cmd += ["-DSIM_BASELINE"]
    else:
        cmd += ["-I", str(ROOT / "main/network")]
    cmd += [str(HERE / "sim.c"), "-lm", "-o", str(out)]
    subprocess.run(cmd, check=True)
    return out


def run(binary, scenario, seed):
    res = subprocess.run([str(binary), scenario, str(seed)], check=True,
                         capture_output=True, text=True)
    vals = {}
    for tok in res.stdout.split():
        k, v = tok.split("=")
        vals[k] = float(v)
    return vals


def agg(rows, key):
    vals = [r[key] for r in rows]
    if key in ("lock_s", "settle_s", "recovery_s"):
        if any(v < 0 for v in vals):
            return "never" if all(v < 0 for v in vals) else "%.1f*" % max(vals)
        return "%.1f" % max(vals)
    if key in ("p2p_us", "p95_us", "peak_us"):
        return "%.0f" % max(vals)
    if key == "unlocked_pct":
        return "%.1f" % max(vals)
    return "%+.0f" % (sum(vals) / len(vals))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--only", default=None)
    args = ap.parse_args()
    with tempfile.TemporaryDirectory(prefix="ptp-sim-") as tmp:
        build_dir = Path(tmp)
        for name in SHIMS:
            shim = build_dir / name
            shim.parent.mkdir(parents=True, exist_ok=True)
            shim.write_text('#include "mocks.h"\n')
        old = build(build_dir, True)
        new = build(build_dir, False)
        # The new clock with Apple's filter exactly: no lucky-packet
        # selection, no least-squares fast start.
        apple = build(build_dir, False, ("-DPTP_LUCKY_PERCENTILE=100",
                                         "-DPTP_FAST_START_SYNCS=0",
                                         "-Wno-type-limits"), "sim-apple")
        impls = (("old", old), ("new", new), ("apple", apple))
        scen = [s for s in SCENARIOS if not args.only or s[0] == args.only]
        jobs = []
        with ThreadPoolExecutor(max_workers=os.cpu_count() or 4) as pool:
            for name, _ in scen:
                for seed in range(1, args.seeds + 1):
                    for impl, binary in impls:
                        jobs.append((name, impl, seed,
                                     pool.submit(run, binary, name, seed)))
            results = {}
            for name, impl, seed, fut in jobs:
                results.setdefault((name, impl), []).append(fut.result())

    print("PTP clock simulation: v0.3.0 (old) vs this tree (new), %d seeds, "
          "11 min each, steady state from 60 s" % args.seeds)
    print("apple = this tree with Apple's filter exactly (no lucky-packet "
          "selection, no least-squares fast start)")
    print("error = clock's network time at 'now' - truth; mean is signed "
          "(seed average), p95 |err| / p2p / unlocked are the worst seed,")
    print("lock = first locked on the anchor's GM; settle = within 500 us of "
          "the steady mean for 5 s; recovery = from the event's end to "
          "within 1 ms of it for 5 s (worst seed, * = some seed never).")
    hdr = ("%-20s %-4s %8s %8s %8s %8s %7s %6s %8s %8s" %
           ("scenario", "impl", "mean_us", "p95_us", "p2p_us", "peak_us",
            "unlck%",
            "lock_s", "settl_s", "recov_s"))
    print(hdr)
    print("-" * len(hdr))
    for name, desc in scen:
        for impl in ("old", "new", "apple"):
            rows = results[(name, impl)]
            print("%-20s %-4s %8s %8s %8s %8s %7s %6s %8s %8s" % (
                name if impl == "old" else "", impl,
                agg(rows, "mean_us"), agg(rows, "p95_us"),
                agg(rows, "p2p_us"), agg(rows, "peak_us"),
                agg(rows, "unlocked_pct"),
                agg(rows, "lock_s"), agg(rows, "settle_s"),
                agg(rows, "recovery_s") if name in
                ("sleep", "sleep-passive", "gm-change", "relay") else "-"))
    print()
    for name, desc in scen:
        print("  %-20s %s" % (name, desc))


if __name__ == "__main__":
    main()
