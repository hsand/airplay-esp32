#!/usr/bin/env python3
"""Check the stream SETUP reply bplist against plistlib.

A buffered stream (type 103) answers audioBufferSize, a realtime one (96)
arrivalToRenderLatencyMs, as Apple's receiver does.
Requires Python 3 and a C compiler; no ESP-IDF installation or hardware needed.
Run from any directory with: python3 tests/bplist_setup/run.py
"""

import os
from pathlib import Path
import plistlib
import shlex
import subprocess
import tempfile


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

CASES = (
    # type, data port, control port, audioBufferSize, arrival ms, expected
    (103, 7000, 0, 8388608, -1,
     {"type": 103, "dataPort": 7000, "controlPort": 0,
      "audioBufferSize": 8388608}),
    (96, 6001, 6002, 4096, 52,
     {"type": 96, "dataPort": 6001, "controlPort": 6002,
      "arrivalToRenderLatencyMs": 52}),
    (96, 65535, 300, 4096, -1,
     {"type": 96, "dataPort": 65535, "controlPort": 300}),
    (96, 6001, 6002, 0, 1000,
     {"type": 96, "dataPort": 6001, "controlPort": 6002,
      "arrivalToRenderLatencyMs": 1000}),
)

with tempfile.TemporaryDirectory(prefix="bplist-setup-tests-") as temporary:
    build = Path(temporary)
    (build / "audio_stream.h").write_text(
        "#include <stdbool.h>\n"
        "typedef int audio_stream_type_t;\n"
        "static inline bool audio_stream_uses_buffer(audio_stream_type_t type) {\n"
        "  return type == 103;\n"
        "}\n"
    )
    command = shlex.split(os.environ.get("CC", "cc")) + [
        "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-g", "-O1",
        "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        "-I", str(build), "-I", str(ROOT / "main/plist"),
        str(HERE / "test.c"), str(ROOT / "main/plist/bplist_builder.c"),
        "-o", str(build / "test"),
    ]
    print("Stream SETUP reply tests", flush=True)
    subprocess.run(command, check=True)
    for stream_type, data, control, size, arrival, want in CASES:
        out = subprocess.run(
            [str(build / "test"), str(stream_type), str(data), str(control),
             str(size), str(arrival)],
            check=True, stdout=subprocess.PIPE).stdout
        decoded = plistlib.loads(out)
        assert list(decoded) == ["streams"], decoded
        assert decoded["streams"] == [want], (decoded, want)
    print(f"PASS: {len(CASES)} SETUP replies decode as Apple's, "
          "at every output capacity")
