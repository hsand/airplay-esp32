#!/usr/bin/env python3
"""Run the AirPlay volume law on the host, with each Kconfig law selected.

Requires Python 3 and a C compiler; no ESP-IDF installation or hardware needed.
Run from any directory with: python3 tests/audio_volume/run.py
"""

import os
from pathlib import Path
import shlex
import subprocess
import tempfile


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

with tempfile.TemporaryDirectory(prefix="audio-volume-tests-") as temporary:
    build = Path(temporary)
    print("Volume law host tests", flush=True)
    for label, define in (("Apple", "CONFIG_DAC_VOLUME_LAW_APPLE"),
                          ("legacy", "CONFIG_DAC_VOLUME_LAW_LEGACY")):
        (build / "sdkconfig.h").write_text(f"#define {define} 1\n")
        binary = build / f"test-{label}"
        command = shlex.split(os.environ.get("CC", "cc")) + [
            "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-g", "-O1",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
            "-I", str(build), "-I", str(ROOT / "main/audio"),
            str(HERE / "test.c"), "-lm", "-o", str(binary),
        ]
        print(f"-- {label} law configured", flush=True)
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)
