#!/usr/bin/env python3
"""Simulate the AirPlay 2 buffered playout chain, old (v0.3.0) against new.

Builds sim.c twice -- against the current engine/scheduler/timeline/clock map
in main/audio, and against the v0.3.0 copies in baseline/ -- runs every
scenario through both with the same seed, and prints a table.  The new build
must also meet the expectations at the bottom, so this doubles as a
regression test.

Requires Python 3 and a C compiler; no ESP-IDF installation or hardware needed.
Run from any directory with: python3 tests/playout_sim/run.py [scenario ...]
"""

import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
AUDIO = ROOT / "main/audio"

SHIMS = (
    "esp_err.h", "esp_timer.h", "esp_log.h", "esp_heap_caps.h", "sdkconfig.h",
    "freertos/FreeRTOS.h", "freertos/task.h", "freertos/semphr.h",
    "freertos/portmacro.h",
)

SCENARIOS = (
    "steady", "drift+50", "drift-50", "stall40", "stall80", "glitch8ms",
    "glitch20ms", "clockstep20",
    "gm_change", "pause_resume", "skip_deferred", "lost_packet",
    "reanchor_same", "reanchor_moved",
)

NEW_SOURCES = [AUDIO / f for f in (
    "audio_engine_v2.c", "audio_scheduler.c", "audio_timeline.c",
    "audio_clock_map.c", "audio_epoch.c", "audio_playout_clock.c",
    "audio_flush_queue.c")]
OLD_SOURCES = [HERE / "baseline" / f for f in (
    "audio_engine_v2.c", "audio_scheduler.c", "audio_timeline.c",
    "audio_clock_map.c", "audio_epoch.c")]


def build(build_dir, name, baseline):
    includes = ["-I", str(build_dir), "-I", str(HERE)]
    if baseline:
        includes += ["-I", str(HERE / "baseline")]
    includes += ["-I", str(AUDIO), "-I", str(ROOT / "main/network")]
    sources = OLD_SOURCES if baseline else NEW_SOURCES
    command = shlex.split(os.environ.get("CC", "cc")) + [
        "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-g", "-O2",
        "-Wno-unused-parameter", "-Wno-sign-compare",
        # int64_t is long on the host and long long on the target; the
        # firmware formats are right for the target.
        "-Wno-format",
        *(["-DBASELINE"] if baseline else []),
        *includes, str(HERE / "sim.c"), *map(str, sources),
        "-lm", "-o", str(build_dir / name),
    ]
    subprocess.run(command, check=True)
    return build_dir / name


def run(binary, scenario):
    out = subprocess.run([str(binary), scenario], check=True,
                         stdout=subprocess.PIPE, text=True).stdout
    line = next(l for l in out.splitlines() if l.startswith("RESULT"))
    fields = line.split()[2:]
    return {k: float(v) for k, v in (f.split("=") for f in fields)}


COLUMNS = (
    ("mean_us", "mean|e| us", "{:.0f}"),
    ("p95_us", "p95|e| us", "{:.0f}"),
    ("max_us", "max|e| us", "{:.0f}"),
    ("silent_ms", "silent ms", "{:.0f}"),
    ("steps", "steps", "{:.0f}"),
    ("trims_per_s", "trims/s", "{:.2f}"),
    ("far", "far>100ms", "{:.0f}"),
    ("wrong_track_ms", "old-trk ms", "{:.0f}"),
    ("first_sound_s", "start s", "{:.2f}"),
)

# What the new build must do.  Each is (scenario, field, op, bound).
EXPECT = (
    ("steady", "p95_us", "<", 500),
    ("drift+50", "p95_us", "<", 500),
    ("drift-50", "p95_us", "<", 500),
    ("stall80", "steps", "<=", 1),
    ("stall80", "max_us", "<", 60000),
    ("glitch8ms", "steps", "==", 0),
    ("glitch8ms", "p95_us", "<", 500),
    ("glitch20ms", "steps", "==", 0),
    ("clockstep20", "steps", "==", 1),
    ("gm_change", "far", "==", 0),
    ("gm_change", "p95_us", "<", 500),
    ("pause_resume", "p95_us", "<", 500),
    ("skip_deferred", "wrong_track_ms", "==", 0),
    ("skip_deferred", "p95_us", "<", 500),
    ("reanchor_same", "steps", "==", 0),
    ("reanchor_moved", "far", "==", 0),
)


def main():
    wanted = sys.argv[1:] or list(SCENARIOS)
    with tempfile.TemporaryDirectory(prefix="playout-sim-") as temporary:
        build_dir = Path(temporary)
        for name in SHIMS:
            shim = build_dir / name
            shim.parent.mkdir(parents=True, exist_ok=True)
            shim.write_text('#include "sim_platform.h"\n')
        print("Playout simulation: v0.3.0 (old) against this tree (new)",
              flush=True)
        old = build(build_dir, "sim_old", True)
        new = build(build_dir, "sim_new", False)
        results = {}
        header = "{:<15} {:<4}".format("scenario", "") + "".join(
            " {:>11}".format(title) for _, title, _ in COLUMNS)
        print(header)
        print("-" * len(header))
        for scenario in wanted:
            for label, binary in (("old", old), ("new", new)):
                r = run(binary, scenario)
                results[(scenario, label)] = r
                print("{:<15} {:<4}".format(scenario, label) + "".join(
                    " {:>11}".format(fmt.format(r[key]))
                    for key, _, fmt in COLUMNS), flush=True)
        failures = 0
        ops = {"<": lambda a, b: a < b, "<=": lambda a, b: a <= b,
               "==": lambda a, b: a == b}
        for scenario, field, op, bound in EXPECT:
            if scenario not in wanted:
                continue
            value = results[(scenario, "new")][field]
            if not ops[op](value, bound):
                failures += 1
                print(f"  FAIL new {scenario}: {field} = {value} "
                      f"(want {op} {bound})")
        if failures:
            sys.exit(f"{failures} expectation(s) failed")
        print("all expectations met")


main()
