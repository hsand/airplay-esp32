#!/usr/bin/env python3
"""Run the realtime stream's sync-packet map through jittery senders on the host.

Requires Python 3 and a C compiler; no ESP-IDF installation or hardware needed.
Run from any directory with: python3 tests/rtp_sync/run.py
"""

import os
from pathlib import Path
import shlex
import subprocess
import tempfile


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

# rtp_sync.c has no ESP-IDF dependencies, so it is compiled as it is.
with tempfile.TemporaryDirectory(prefix="rtp-sync-tests-") as temporary:
    build = Path(temporary)
    command = shlex.split(os.environ.get("CC", "cc")) + [
        "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-g", "-O1",
        "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        "-I", str(ROOT / "main/audio"),
        "-I", str(HERE),
        str(HERE / "test.c"), str(ROOT / "main/audio/rtp_sync.c"),
        "-lm", "-o", str(build / "test"),
    ]
    print("RTP sync host tests", flush=True)
    subprocess.run(command, check=True)
    subprocess.run([str(build / "test")], check=True)
