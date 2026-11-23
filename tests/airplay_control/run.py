#!/usr/bin/env python3
"""Unit tests for the AirPlay 2 control plane: FLUSHBUFFERED range semantics
(the sequence-number flush queue and the timeline sweep), SETRATEANCHORTIME
and FLUSHBUFFERED request validation, timing-peer extraction from SETUP and
SETPEERS(X), and the SETUP / status replies.

Request plists are written by Python's plistlib, an independent encoder, and
the replies are decoded by it.  Requires Python 3 and a C compiler; no ESP-IDF
installation or hardware needed.  Run with: python3 tests/airplay_control/run.py
"""

import os
from pathlib import Path
import plistlib
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

SHIMS = (
    "esp_err.h", "esp_timer.h", "esp_log.h", "esp_heap_caps.h", "sdkconfig.h",
    "freertos/FreeRTOS.h", "freertos/task.h", "freertos/semphr.h",
    "freertos/portmacro.h",
)

TIME = {"networkTimeSecs": 1700000000, "networkTimeFrac": 1 << 63,
        "networkTimeTimelineID": 0x1122334455667788, "networkTimeFlags": 0}

# name -> plist, for the C side to parse.
REQUESTS = {
    "anchor_full": {"rate": 1, "rtpTime": 123456, "firstAudibleRTPTime": 123999,
                    **TIME},
    "anchor_rtp0": {"rate": 1, "rtpTime": 0, **TIME},
    "anchor_no_flags": {"rate": 1, "rtpTime": 5, **{
        k: v for k, v in TIME.items() if k != "networkTimeFlags"}},
    "anchor_no_rtp": {"rate": 1, **TIME},
    "anchor_media_time": {"rate": 1, "mediaTimeValue": 88200,
                          "mediaTimeScale": 44100, **TIME},
    "anchor_big_frac": {"rate": 1, "rtpTime": 1, **TIME,
                        "networkTimeFrac": 0xFFFFFFFF00000000},
    "rate0_bare": {"rate": 0},
    "rate2": {"rate": 2, "rtpTime": 5, **TIME},
    "rate_real1": {"rate": 1.0, "rtpTime": 7, **TIME},
    "rate_real_half": {"rate": 0.5},
    "rate_missing": {"rtpTime": 7, **TIME},
    "flush_deferred": {"flushFromSeq": 100, "flushFromTS": 5000,
                       "flushUntilSeq": 200, "flushUntilTS": 5000},
    "flush_immediate": {"flushUntilSeq": 300, "flushUntilTS": 9000},
    "flush_from_without_ts": {"flushFromSeq": 100, "flushUntilSeq": 200,
                              "flushUntilTS": 9000},
    "flush_no_until": {"flushFromSeq": 1, "flushFromTS": 2},
    "setup_peers": {
        "timingProtocol": "PTP",
        "timingPeerInfo": {"Addresses": ["192.168.1.20", "fe80::1"],
                           "ID": "8A0B2C3D-0000-4000-8000-0123456789AB"},
        "timingPeerList": [
            {"Addresses": ["192.168.1.30", "192.168.1.20"], "ID": "x"},
            {"Addresses": ["10.0.0.7"], "ClockID": 77},
        ],
    },
}
# Top-level arrays (SETPEERS / SETPEERSX).
ARRAYS = {
    "setpeers": ["192.168.1.40", "192.168.1.41", "fe80::2", "192.168.1.40"],
    "setpeersx": [{"Addresses": ["192.168.1.50"], "ID": "a",
                   "SupportsClockPortMatchingOverride": True},
                  {"Addresses": ["192.168.1.51", "::1"], "ID": "b"}],
}


def main():
    with tempfile.TemporaryDirectory(prefix="airplay-control-") as temporary:
        build = Path(temporary)
        for name in SHIMS:
            shim = build / name
            shim.parent.mkdir(parents=True, exist_ok=True)
            shim.write_text('#include "sim_platform.h"\n')
        (build / "audio_stream.h").write_text(
            "#include <stdbool.h>\n"
            "typedef int audio_stream_type_t;\n"
            "static inline bool audio_stream_uses_buffer(audio_stream_type_t t)"
            " {\n  return t == 103;\n}\n")
        plists = build / "plists"
        plists.mkdir()
        for name, value in {**REQUESTS, **ARRAYS}.items():
            (plists / name).write_bytes(
                plistlib.dumps(value, fmt=plistlib.FMT_BINARY))

        command = shlex.split(os.environ.get("CC", "cc")) + [
            "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-g", "-O1",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
            "-Wno-format",
            "-I", str(build), "-I", str(ROOT / "tests/playout_sim"),
            "-I", str(ROOT / "main/audio"), "-I", str(ROOT / "main/rtsp"),
            "-I", str(ROOT / "main/plist"),
            str(HERE / "test.c"),
            str(ROOT / "main/audio/audio_flush_queue.c"),
            str(ROOT / "main/audio/audio_timeline.c"),
            str(ROOT / "main/audio/audio_plc.c"),
            str(ROOT / "main/rtsp/rtsp_anchor.c"),
            str(ROOT / "main/plist/bplist_parser.c"),
            str(ROOT / "main/plist/bplist_builder.c"),
            "-o", str(build / "test"),
        ]
        print("AirPlay control-plane tests", flush=True)
        subprocess.run(command, check=True)
        subprocess.run([str(build / "test"), str(plists), str(build)],
                       check=True)

        # The replies, through an independent decoder.
        setup = plistlib.loads((build / "setup_reply.bplist").read_bytes())
        assert setup == {
            "eventPort": 51234, "timingPort": 0,
            "timingPeerInfo": {"Addresses": ["192.168.1.99"],
                               "ID": "a1b2c3d4-0000-4000-8000-0011223344AA"},
        }, setup
        status = plistlib.loads((build / "status_reply.bplist").read_bytes())
        assert status == {"status": -6705}, status
        print("replies decode as Apple's do: PASS")


main()
