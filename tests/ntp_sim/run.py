#!/usr/bin/env python3
"""Simulate the AirPlay 1 timing client, v0.3.0 against the current code.

Both builds run the real ntp_clock.c task loop against the same seeded
simulated sender, Wi-Fi link and drifting crystal (see sim.c), and the table
compares their error in network time "now" against the truth.

Requires Python 3 and a C compiler; no ESP-IDF installation or hardware needed.
Run from any directory with: python3 tests/ntp_sim/run.py
"""

import os
from pathlib import Path
import shlex
import statistics
import subprocess
import sys
import tempfile


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

SHIMS = (
    "esp_err.h", "esp_log.h", "esp_timer.h", "esp_random.h",
    "freertos/FreeRTOS.h", "freertos/task.h",
)

COLUMNS = (
    ("lock", "lock s", 2),
    ("bias", "bias us", 1),
    ("p95", "p95|e| us", 1),
    ("jit", "p95|e-b| us", 1),
    ("pp", "p-p us", 1),
    ("saw", "saw us", 1),
    ("startup", "<60s p95 us", 1),
    ("recovery", "recover s", 1),
)


def build(build_dir, name, source, include, defines):
    out = build_dir / name
    command = shlex.split(os.environ.get("CC", "cc")) + [
        "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-g", "-O1",
        "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        # The FreeRTOS task entry point never reads its parameter.
        "-Wno-unused-parameter",
        "-I", str(build_dir), "-I", str(HERE), "-I", str(include),
        f'-DNTP_SOURCE="{source}"', *defines,
        str(HERE / "sim.c"), "-lm", "-o", str(out),
    ]
    subprocess.run(command, check=True)
    return out


def collect(binary):
    result = subprocess.run([str(binary)], check=True, capture_output=True,
                            text=True)
    rows = {}
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) != 10:
            print(line)
            continue
        name = fields[0]
        values = dict(zip(("lock", "bias", "p95", "jit", "pp", "saw",
                           "startup", "recovery"), map(float, fields[2:])))
        rows.setdefault(name, []).append(values)
    return rows


def table(builds):
    head = f"{'scenario':<11} {'code':<6}" + "".join(
        f" {title:>11}" for _, title, _ in COLUMNS)
    print(head)
    print("-" * len(head))
    old = builds[0][1]
    for name in old:
        for label, runs in builds:
            rows = runs[name]
            cells = []
            for key, _, digits in COLUMNS:
                values = [r[key] for r in rows]
                mean = statistics.fmean(values)
                cells.append(f" {mean:>11.{digits}f}")
            print(f"{name:<11} {label:<6}" + "".join(cells))
        worst = ", ".join("%s %.0f" % (label, max(r["p95"] for r in runs[name]))
                          for label, runs in builds)
        print(f"{'':<11} worst p95|e| us: {worst}")
    print("\nMeans over %d seeds.  bias = mean error (most of it is the fixed "
          "half-difference of the two one-way delays, which no NTP client "
          "can see); p95|e|, p95|e-b| (about the bias) and p-p exclude the "
          "first 60 s and the 120 s after a sender step; saw = mean spread "
          "within 3 s windows; <60s p95 = |e| from lock to 60 s; recover = "
          "after the sender step, until the 3 s mean error stays within 1 ms "
          "of where it was before the step."
          % len(next(iter(old.values()))))
    print("apple = the new code with its frequency acquisition turned off "
          "(NTP_FREQ_ACQUIRE=0), i.e. Apple's clock discipline alone.")


with tempfile.TemporaryDirectory(prefix="ntp-sim-") as temporary:
    build_dir = Path(temporary)
    for name in SHIMS:
        shim = build_dir / name
        shim.parent.mkdir(parents=True, exist_ok=True)
        shim.write_text('#include "mocks.h"\n')

    print("NTP clock simulation", flush=True)
    new = build(build_dir, "sim_new", ROOT / "main/network/ntp_clock.c",
                ROOT / "main/network", ["-DNTP_NEW"])
    apple = build(build_dir, "sim_apple", ROOT / "main/network/ntp_clock.c",
                  ROOT / "main/network", ["-DNTP_NEW", "-DNTP_FREQ_ACQUIRE=0"])
    old = build(build_dir, "sim_old", HERE / "baseline/ntp_clock.c",
                HERE / "baseline", [])
    subprocess.run([str(new), "--units"], check=True)
    table([("v0.3.0", collect(old)), ("apple", collect(apple)),
           ("new", collect(new))])
    sys.stdout.flush()
