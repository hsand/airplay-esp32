#!/usr/bin/env python3
"""Simulate the AirPlay 1 timing client, v0.3.0 against the current code.

Both builds run the real ntp_clock.c task loop against the same seeded
simulated sender, Wi-Fi link and drifting crystal (see sim.c), and the table
compares their error in network time "now" against the truth.  The current
code must then stay within LIMITS and no worse than v0.3.0, or the run exits
non-zero.

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


# Upper bounds for the current code, about 1.25x (means) to 1.5x (worst seed)
# what it measures today, so only a regression trips them: mean p95|e| us,
# worst-seed p95|e| us, mean <60s p95 us, mean recover s.  The simulation is
# seeded and deterministic; the margin absorbs libm and compiler differences.
LIMITS = {
    "lan": (350, 450, 600, None),
    "wifi": (1300, 2000, 1600, None),
    "wifi_bad": (3700, 5500, 5500, None),
    "step_30ms": (2400, 5000, 1600, 260),
    "step_400ms": (1800, 6500, 1600, 150),
}
# Every seed has to lock, and within this.
LOCK_LIMIT_S = 2.0
SEEDS = 8


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


def check(new, old):
    """Returns the failures of the current code against LIMITS and v0.3.0."""
    failures = []

    def mean(rows, key):
        return statistics.fmean(r[key] for r in rows)

    for name, (p95, worst, startup, recovery) in LIMITS.items():
        rows = new.get(name, [])
        if len(rows) != SEEDS:
            failures.append(f"{name}: {len(rows)} of {SEEDS} seeds ran")
            continue
        limits = [("mean p95|e| us", mean(rows, "p95"), p95),
                  ("worst p95|e| us", max(r["p95"] for r in rows), worst),
                  ("mean <60s p95 us", mean(rows, "startup"), startup),
                  ("worst lock s", max(r["lock"] for r in rows),
                   LOCK_LIMIT_S)]
        if recovery is not None:
            limits.append(("mean recover s", mean(rows, "recovery"),
                           recovery))
        for label, value, limit in limits:
            if value > limit:
                failures.append(f"{name}: {label} {value:.1f} > {limit}")
        if min(r["lock"] for r in rows) < 0:
            failures.append(f"{name}: a seed never locked")
        # Without a sender step the frequency acquisition has to pay its way.
        if recovery is None and name in old:
            baseline = mean(old[name], "p95")
            if mean(rows, "p95") > baseline:
                failures.append(f"{name}: mean p95|e| {mean(rows, 'p95'):.1f}"
                                f" us is worse than v0.3.0's {baseline:.1f}")
    return failures


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
    old_rows, new_rows = collect(old), collect(new)
    table([("v0.3.0", old_rows), ("apple", collect(apple)),
           ("new", new_rows)])
    failures = check(new_rows, old_rows)
    for failure in failures:
        print(f"FAIL {failure}")
    sys.stdout.flush()
    if failures:
        sys.exit(f"{len(failures)} limit(s) exceeded")
    print("\nThe current code is within every limit.")
