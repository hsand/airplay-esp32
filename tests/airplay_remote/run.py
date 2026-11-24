#!/usr/bin/env python3
"""Unit tests for the receiver's remote control and advertisement.

Covers rtsp_policy.c (feature bits, featuresEx, the _airplay/_raop TXT
records, the per-device pairing identity, the session preemption decision,
command routing, progress across an RTP wrap and audioBufferSize),
rtsp_metadata.c (DMAP and the merged now-playing state), the Events-Salt key
derivation in hap_crypto.c and the sendMediaRemoteCommand plist.

The C side checks the logic; this side checks what needs an independent
implementation: plistlib decodes the command plists, hashlib/hmac computes the
HKDF-SHA512 Events keys, base64 decodes featuresEx, and the pairing identity is
recomputed here.  Requires Python 3 and a C compiler; no ESP-IDF installation
or hardware needed.  Run with: python3 tests/airplay_remote/run.py
"""

import base64
import hashlib
import hmac
import os
from pathlib import Path
import plistlib
import re
import shlex
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

SHIMS = ("esp_err.h", "esp_log.h", "esp_timer.h", "sodium.h")


def hkdf_sha512(salt, ikm, info, length=32):
    prk = hmac.new(salt, ikm, hashlib.sha512).digest()
    okm, block, counter = b"", b"", 1
    while len(okm) < length:
        block = hmac.new(prk, block + info + bytes([counter]),
                         hashlib.sha512).digest()
        okm += block
        counter += 1
    return okm[:length]


def fnv1a64(h, data):
    for byte in data:
        h ^= byte
        h = (h * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


def mix64(x):
    m = 0xFFFFFFFFFFFFFFFF
    x = ((x ^ (x >> 30)) * 0xBF58476D1CE4E5B9) & m
    x = ((x ^ (x >> 27)) * 0x94D049BB133111EB) & m
    return x ^ (x >> 31)


def pairing_identity(mac):
    domain = b"airplay-esp32 pairing identity"
    raw = b""
    for half in range(2):
        h = 0xCBF29CE484222325 ^ half
        h = mix64(fnv1a64(fnv1a64(h, domain), mac))
        raw += h.to_bytes(8, "big")
    raw = bytearray(raw)
    raw[6] = (raw[6] & 0x0F) | 0x50
    raw[8] = (raw[8] & 0x3F) | 0x80
    x = raw.hex().upper()
    return f"{x[:8]}-{x[8:12]}-{x[12:16]}-{x[16:20]}-{x[20:]}"


def bits(value):
    return {b for b in range(64) if value >> b & 1}


def main():
    with tempfile.TemporaryDirectory(prefix="airplay-remote-") as temporary:
        build = Path(temporary)
        for name in SHIMS:
            (build / name).write_text('#include "platform.h"\n')
        # rtsp_metadata.c asks the receiver what is playing; test.c mocks it.
        (build / "audio_receiver.h").write_text(
            "#include <stdbool.h>\n#include <stdint.h>\n"
            "bool audio_receiver_is_playing(void);\n"
            "bool audio_receiver_get_rendered_rtp(uint32_t *rtp);\n")
        (build / "audio_stream.h").write_text(
            "#include <stdbool.h>\n"
            "typedef int audio_stream_type_t;\n"
            "static inline bool audio_stream_uses_buffer(audio_stream_type_t t)"
            " {\n  return t == 103;\n}\n")
        out = build / "out"
        out.mkdir()

        command = shlex.split(os.environ.get("CC", "cc")) + [
            "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-g", "-O1",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
            "-I", str(build), "-I", str(HERE), "-I", str(ROOT / "main"),
            "-I", str(ROOT / "main/rtsp"), "-I", str(ROOT / "main/plist"),
            "-I", str(ROOT / "main/hap"),
            str(HERE / "test.c"),
            str(ROOT / "main/rtsp/rtsp_policy.c"),
            str(ROOT / "main/rtsp/rtsp_metadata.c"),
            str(ROOT / "main/hap/hap_crypto.c"),
            str(ROOT / "main/plist/base64.c"),
            str(ROOT / "main/plist/bplist_builder.c"),
            "-o", str(build / "test"),
        ]
        print("AirPlay remote-control and advertisement tests", flush=True)
        subprocess.run(command, check=True)
        result = subprocess.run([str(build / "test"), str(out)],
                                stdout=subprocess.PIPE, text=True)
        print(result.stdout, end="")
        assert result.returncode == 0, "C checks failed"

        lines = {}
        txt = {"TXT_AIRPLAY": [], "TXT_RAOP": []}
        pis = []
        for line in result.stdout.splitlines():
            key, _, value = line.partition(" ")
            if key in txt:
                txt[key].append(tuple(value.split("=", 1)))
            elif key == "PI":
                pis.append(value.split(" "))
            elif re.fullmatch(r"[A-Z0-9_]+", key):
                lines[key] = value

        # The command plists, as the sender decodes them.
        toggle = plistlib.loads((out / "cmd_toggle.bplist").read_bytes())
        assert toggle == {"type": "sendMediaRemoteCommand", "value": "plps",
                          "modernMediaRemoteCommand": "2"}, toggle
        nxt = plistlib.loads((out / "cmd_next.bplist").read_bytes())
        assert nxt["value"] == "nitm" and \
            nxt["modernMediaRemoteCommand"] == "4", nxt
        dvlc = plistlib.loads((out / "cmd_dvlc.bplist").read_bytes())
        assert dvlc == {"type": "sendMediaRemoteCommand", "value": "dvlc",
                        "volume": 0.75, "isMuted": False,
                        "params": {"volume": 0.75, "isMuted": False}}, dvlc
        print("sendMediaRemoteCommand plists decode as expected: PASS")

        # Events keys: HKDF-SHA512, salt Events-Salt; the receiver writes
        # with Events-Write and reads with Events-Read.
        ikm = bytes((i * 7 + 3) & 0xFF for i in range(64))
        for size in (32, 64):
            write = hkdf_sha512(b"Events-Salt", ikm[:size],
                                b"Events-Write-Encryption-Key")
            read = hkdf_sha512(b"Events-Salt", ikm[:size],
                               b"Events-Read-Encryption-Key")
            assert lines[f"EVENTS{size}_WRITE"] == write.hex(), size
            assert lines[f"EVENTS{size}_READ"] == read.hex(), size
        # The full 64-byte SRP key matters: its first half gives other keys.
        assert lines["EVENTS64_WRITE"] != lines["EVENTS32_WRITE"]
        print("Events-Salt keys match an independent HKDF: PASS")

        # Feature bits.
        v1 = int(lines["FEATURES_V1"], 16)
        v2 = int(lines["FEATURES_V2"], 16)
        v2_art = int(lines["FEATURES_V2_ART"], 16)
        assert v1 == 0x5C4A00, hex(v1)
        assert 47 not in bits(v2) and 50 not in bits(v2), sorted(bits(v2))
        assert {16, 17} <= bits(v2) and 15 not in bits(v2)
        assert bits(v2_art) == bits(v2) | {15}
        # Exactly the old set, less bit 47, plus the metadata bits.
        assert v2 == (0x1C340405C4A00 & ~(1 << 47)) | (1 << 16) | (1 << 17)
        ex = base64.b64decode(lines["FEATURES_EX"] + "==")
        assert ex == v2_art.to_bytes(8, "little").rstrip(b"\0"), ex.hex()
        print(f"features 0x{v2_art:X}: no bit 47 or 50, metadata 15-17, "
              f"featuresEx {lines['FEATURES_EX']}: PASS")

        # TXT records.
        airplay = dict(txt["TXT_AIRPLAY"])
        assert len(airplay) == len(txt["TXT_AIRPLAY"]), "duplicate key"
        assert airplay["deviceid"] == "24:0A:C4:12:34:56"
        assert airplay["model"] == "AudioAccessory5,1"
        assert airplay["pi"] == "PI-VALUE"
        assert airplay["md"] == "0,2" and airplay["vv"] == "1"
        assert airplay["srcvers"] == "377.40.00"
        assert airplay["protovers"] == "1.1" and airplay["flags"] == "0x4"
        raop = dict(txt["TXT_RAOP"])
        assert raop["am"] == "AudioAccessory5,1" and raop["md"] == "0,2"
        assert raop["ft"] == airplay["features"] and raop["vv"] == "1"
        assert raop["vs"] == "377.40.00" and raop["et"] == "0,1,3,5"
        print("TXT records: PASS")

        # Pairing identity: stable, per device, a well-formed v5 UUID.
        seen = set()
        for mac_hex, pi in pis:
            assert pi == pairing_identity(bytes.fromhex(mac_hex)), (mac_hex, pi)
            assert re.fullmatch(
                r"[0-9A-F]{8}-[0-9A-F]{4}-5[0-9A-F]{3}-[89AB][0-9A-F]{3}-"
                r"[0-9A-F]{12}", pi), pi
            seen.add(pi)
        assert len(seen) == len(pis)
        print(f"pairing identity: {len(pis)} MACs, distinct and stable: PASS")


main()
