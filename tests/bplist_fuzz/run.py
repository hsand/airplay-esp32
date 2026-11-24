#!/usr/bin/env python3
"""Run the binary plist parser on valid and hostile plists.

Every lookup in main/plist/bplist_parser.c is compiled with AddressSanitizer
and UndefinedBehaviorSanitizer and fed:

* valid plists written by plistlib (hand-picked AirPlay shapes and seeded
  random ones), whose results must match a model of each lookup written
  here against the decoded plist;
* hand-made hostile plists: counts and lengths up to 2^64 - 1, chosen so
  that `count * per_entry` wraps, bad trailer fields, offsets outside the
  object area, self-referencing containers;
* every truncation and a set of seeded byte mutations of the valid ones.

The hostile inputs only have to be survived: no read outside the buffer, no
undefined behaviour, and no runaway walk (the run has a time limit).

Requires Python 3 and a C compiler; no ESP-IDF installation or hardware needed.
Run from any directory with: python3 tests/bplist_fuzz/run.py
"""

import os
from pathlib import Path
import plistlib
import random
import re
import shlex
import struct
import subprocess
import tempfile


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]

U64 = (1 << 64) - 1


# ---------------------------------------------------------------------------
# A model of each lookup, as the C driver (test.c) calls it.
# ---------------------------------------------------------------------------

def is_int(v):
    return isinstance(v, int) and not isinstance(v, bool)


def as_int64(v):
    """The low 8 bytes of a plist integer, as int64_t."""
    v &= U64
    return v - (1 << 64) if v >= 1 << 63 else v


def int_of(v):
    """What an integer lookup reads."""
    return as_int64(v) if is_int(v) else None


def is_ascii(s):
    return all(ord(c) < 0x80 for c in s)


def readable(s, capacity):
    """A string the parser can copy into a buffer of `capacity` bytes."""
    return isinstance(s, str) and is_ascii(s) and len(s) < capacity


def hexs(b):
    return b.hex() if b else "="


def top_value(top, key, capacity=64):
    if isinstance(top, dict) and key in top and readable(key, capacity):
        return True, top[key]
    return False, None


def model_deep(top):
    visits = [0]

    def walk(obj, depth):
        if depth > 10:
            return None
        visits[0] += 1
        if visits[0] > 1024:
            return None
        if isinstance(obj, dict):
            if "deep" in obj:
                v = obj["deep"]
                return v if isinstance(v, bytes) and len(v) <= 64 else None
            children = [obj[k] for k in sorted(obj)]
        elif isinstance(obj, (list, tuple)):
            children = list(obj)
        else:
            return None
        for child in children:
            found = walk(child, depth + 1)
            if found is not None:
                return found
        return None

    return walk(top, 0)


def streams_of(top):
    ok, v = top_value(top, "streams")
    return v if ok and isinstance(v, (list, tuple)) else None


def stream_info(top, i):
    streams = streams_of(top)
    if streams is None or i >= len(streams) or not isinstance(streams[i], dict):
        return None
    s = streams[i]
    t = int_of(s.get("type"))
    t = -1 if t is None else t
    lens = [len(s[k]) if isinstance(s.get(k), bytes) else 0
            for k in ("ekey", "eiv", "shk")]
    return None if t == -1 else (t, *lens)


def kv_info(top, i):
    streams = streams_of(top)
    if streams is None or i >= len(streams) or not isinstance(streams[i], dict):
        return None
    out = []
    for k in sorted(streams[i]):
        if len(out) == 8:
            break
        if not readable(k, 64):
            continue
        v = streams[i][k]
        if is_int(v):
            out.append((k, 1, 0, as_int64(v)))
        elif isinstance(v, bytes):
            out.append((k, 2, len(v), 0))
        elif isinstance(v, str):
            n = len(v) if is_ascii(v) else len(v.encode("utf-16-be")) // 2
            out.append((k, 3, n, 0))
        elif isinstance(v, plistlib.UID):
            out.append((k, 4, 0, 0))
        elif isinstance(v, (list, tuple)):
            out.append((k, 5, 0, 0))
        elif isinstance(v, dict):
            out.append((k, 6, 0, 0))
        else:
            out.append((k, 0, 0, 0))
    return out or None


def model_crypto(top):
    streams = streams_of(top)
    if streams is None:
        return None
    for i in range(len(streams)):
        info = stream_info(top, i)
        if info is None or info[0] != 96:
            continue
        found = False
        got = []
        for name, length, capacity in zip(("ekey", "eiv", "shk"), info[1:],
                                          (16, 16, 64)):
            ok, v = top_value(top, name)
            if length and ok and isinstance(v, bytes) and len(v) <= 512:
                got.append(v[:capacity])
                found = True
            else:
                got.append(b"")
        return "/".join(hexs(b) for b in got) if found else None
    return None


QUAD = re.compile(r"([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})")


def model_ipv4(start):
    out = []
    visits = [0]

    def walk(obj, depth):
        if depth > 5:
            return
        visits[0] += 1
        if visits[0] > 256:
            return
        if isinstance(obj, str):
            m = QUAD.fullmatch(obj)
            if readable(obj, 64) and m and all(int(g) <= 255
                                               for g in m.groups()):
                addr = ".".join(str(int(g)) for g in m.groups())
                if addr not in out and len(out) < 8:
                    out.append(addr)
        elif isinstance(obj, dict):
            for k in sorted(obj):
                walk(obj[k], depth + 1)
        elif isinstance(obj, (list, tuple)):
            for child in obj:
                walk(child, depth + 1)

    walk(start, 0)
    return "[" + ",".join(out) + "]"


def model(top):
    """The line test.c prints for a valid plist whose root is `top`."""
    fields = {}
    ok, v = top_value(top, "int")
    fields["int"] = str(int_of(v)) if ok and int_of(v) is not None else "-"
    ok, v = top_value(top, "real")
    if ok and isinstance(v, float):
        fields["real"] = v
    elif ok and int_of(v) is not None:
        fields["real"] = float(int_of(v))
    else:
        fields["real"] = "-"
    ok, v = top_value(top, "str")
    fields["str"] = (hexs(v.encode()) if ok and readable(v, 32) else "-")
    ok, v = top_value(top, "data")
    fields["data"] = (hexs(v) if ok and isinstance(v, bytes) and len(v) <= 64
                      else "-")
    deep = model_deep(top)
    fields["deep"] = "-" if deep is None else hexs(deep)
    ok, params = top_value(top, "params")
    nested = "-"
    if ok and isinstance(params, dict) and int_of(params.get("value")) \
            is not None:
        nested = str(int_of(params["value"]))
    fields["nested"] = nested
    streams = streams_of(top)
    fields["streams"] = "-" if streams is None else str(len(streams))
    for i in range(3):
        info = stream_info(top, i)
        fields[f"si{i}"] = "-" if info is None else "/".join(map(str, info))
        kv = kv_info(top, i)
        fields[f"kv{i}"] = "-" if kv is None else ",".join(
            f"{hexs(k.encode())}:{t}:{n}:{iv}" for k, t, n, iv in kv)
    crypto = model_crypto(top)
    fields["crypto"] = "-" if crypto is None else crypto
    ok, peers = top_value(top, "peers")
    fields["ipv4k"] = model_ipv4(peers) if ok else "-"
    fields["ipv4"] = model_ipv4(top)
    return fields


def parse_line(line):
    fields = dict(item.split("=", 1) for item in line.split(" "))
    if fields["real"] != "-":
        fields["real"] = float.fromhex(fields["real"])
    return fields


# ---------------------------------------------------------------------------
# Valid plists.
# ---------------------------------------------------------------------------

def quad(rng):
    return ".".join(str(rng.randrange(256)) for _ in range(4))


def handpicked():
    ekey, eiv, shk = bytes(range(72)), bytes(range(16)), bytes(range(32))
    yield {"streams": [{"type": 96, "ekey": ekey, "eiv": eiv, "shk": shk,
                        "ct": 2, "spf": 352, "audioFormat": 0x40000,
                        "controlPort": 6001, "streamConnectionID": 1 << 62}]}
    yield {"streams": [{"type": 103, "shk": shk, "ct": 8},
                       {"type": 96, "shk": shk[:16], "ekey": ekey[:16]},
                       {"type": True, "shk": shk}],
           "ekey": ekey[:20], "eiv": eiv, "shk": shk}
    yield {"streams": [{"type": 96, "ekey": ekey, "eiv": eiv}],
           "ekey": bytes(600), "eiv": eiv}
    yield {"streams": [], "deep": b""}
    yield {"streams": [1, "two", {"type": 96}]}
    yield {"streams": {"type": 96}}
    yield {"streams": [{"type": 96, "uid": plistlib.UID(3), "list": [1],
                        "dict": {}, "name": "Café", "flag": False,
                        "rate": 1.5, "x" * 63: 1, "y" * 64: 2,
                        "keys": 1, "more": 2, "stuff": 3}]}
    yield {"peers": {"Addresses": ["192.168.1.20", "fe80::1%en0"],
                     "ID": "C4F1A2B3", "SupportsClockPortMatchingOverride":
                     True}, "int": 7}
    yield {"peers": [{"Addresses": ["192.168.1.30", "192.168.1.20"]},
                     {"Addresses": ["10.0.0.7", "10.0.0.7"]}]}
    yield ["192.168.1.40", "fe80::2", "192.168.1.40", "192.168.1.41"]
    yield [{"Addresses": ["192.168.1.50"], "ID": "A"},
           {"Addresses": ["192.168.1.51", "256.1.1.1", "1.2.3", "01.2.3.4"]}]
    yield {"peers": [f"10.0.{i // 256}.{i % 256}" for i in range(300)],
           "int": 5}
    yield {"type": "sendMediaRemoteCommand", "params": {"value": 1234}}
    yield {"type": "updateMRNowPlayingInfo",
           "params": {"value": -5, "other": {"value": 3}}}
    yield {"params": {"value": "nope"}}
    yield {"params": [1, 2]}
    for v in (0, 1, 255, 256, 65535, 65536, (1 << 32) - 1, 1 << 32,
              (1 << 63) - 1, 1 << 63, U64, -1, -(1 << 63)):
        yield {"int": v, "real": v}
    for v in (0.0, -0.0, 1.5, -2.25, 1e300, 5e-324, float("inf")):
        yield {"real": v, "int": v}
    yield {"real": True, "int": False}
    for n in (0, 1, 15, 16, 31, 32):
        yield {"str": "s" * n}
    yield {"str": "Café"}
    yield {"str": "音"}
    for n in (0, 1, 14, 15, 16, 64, 65, 600):
        blob = bytes(i % 251 for i in range(n))
        yield {"data": blob, "deep": blob}
    nested = {"deep": b"\x01\x02"}
    for _ in range(12):
        nested = {"a": nested}
    yield nested
    nested = {"deep": b"\x01"}
    for _ in range(9):
        nested = [nested]
    yield {"x": nested}
    yield {"a": {"deep": "string"}, "b": {"deep": b"\x05"}}
    yield {"a": [1, 2, {"deep": b"\x06"}], "deep2": b"\x07"}
    yield {"str": "same", "x": ["same"] * 40, "y": {"same": "same"}}
    yield "just a string"
    yield 42
    yield []
    yield {}


KEYS = ("int", "real", "str", "data", "deep", "params", "value", "streams",
        "type", "ekey", "eiv", "shk", "peers", "Addresses", "ID", "a", "b")


def random_value(rng, depth):
    kind = rng.randrange(12 if depth < 4 else 8)
    if kind == 0:
        return rng.choice((0, 1, 96, 103, -1, 1 << 40, 1 << 63, U64,
                           rng.randrange(-(1 << 63), 1 << 64)))
    if kind == 1:
        return rng.choice((0.5, -3.0, 1e10, rng.random()))
    if kind == 2:
        n = rng.choice((0, 1, 4, 15, 16, 31, 32, 40))
        return "".join(rng.choice("abcxyz019.") for _ in range(n))
    if kind == 3:
        return quad(rng)
    if kind == 4:
        return rng.choice(("ü", "fe80::1", "直播", "🎵"))
    if kind == 5:
        return rng.randbytes(rng.choice((0, 1, 16, 32, 64, 65, 100)))
    if kind == 6:
        return rng.choice((True, False))
    if kind == 7:
        return plistlib.UID(rng.randrange(1 << 16))
    if kind in (8, 9):
        return [random_value(rng, depth + 1)
                for _ in range(rng.randrange(0, 6))]
    return random_dict(rng, depth + 1)


def random_dict(rng, depth):
    d = {}
    for _ in range(rng.randrange(0, 7)):
        key = rng.choice(KEYS) if rng.random() < 0.8 else "k" * rng.randrange(
            1, 70)
        d[key] = random_value(rng, depth)
    if depth == 0 and rng.random() < 0.4:
        d["streams"] = [random_dict(rng, 2) | ({"type": rng.choice((96, 103))}
                                               if rng.random() < 0.8 else {})
                        for _ in range(rng.randrange(0, 4))]
    return d


def valid_corpus():
    plists = list(handpicked())
    rng = random.Random(20260611)
    plists += [random_dict(rng, 0) for _ in range(400)]
    plists += [random_value(rng, 0) for _ in range(100)]
    return plists


# ---------------------------------------------------------------------------
# Hostile plists, assembled by hand so that every field can be wrong.
# ---------------------------------------------------------------------------

def be(value, size):
    return (value & ((1 << (8 * size)) - 1)).to_bytes(size, "big")


def counted(marker, count, wide=None):
    """A marker with its count; wide forces the 8-byte count form."""
    if count < 15 and not wide:
        return bytes([marker | count])
    return bytes([marker | 0x0F, 0x13]) + be(count, 8)


def ascii_obj(s, count=None):
    return counted(0x50, len(s) if count is None else count) + s.encode()


def int_obj(v):
    return b"\x13" + be(v, 8)


def container(marker, refs, ref_size, count=None):
    """A dict (marker 0xD0: refs are keys then values) or an array."""
    n = len(refs) // 2 if marker == 0xD0 else len(refs)
    return counted(marker, n if count is None else count,
                   wide=count is not None) + b"".join(
        be(r, ref_size) for r in refs)


def assemble(objects, ref_size=1, offset_size=8, top=0, num_objects=None,
             table_offset=None, offsets=None):
    body = bytearray(b"bplist00")
    positions = []
    for obj in objects:
        positions.append(len(body))
        body += obj
    table_at = len(body)
    for pos in (positions if offsets is None else offsets):
        body += be(pos, offset_size)
    body += bytes(6) + bytes([offset_size & 0xFF, ref_size & 0xFF])
    body += be(len(positions) if num_objects is None else num_objects, 8)
    body += be(top, 8)
    body += be(table_at if table_offset is None else table_offset, 8)
    return bytes(body)


HUGE_COUNTS = (1 << 31, (1 << 31) + 1, 1 << 32, (1 << 32) + 1, 1 << 61,
               1 << 62, (1 << 62) + 1, 1 << 63, (1 << 63) + 1, U64 - 1, U64)


def hostile_corpus():
    out = []
    # The reported case: a dict whose count makes 2 * n * ref_size wrap to
    # zero, behind a trailer whose object count is large enough not to stop it.
    for ref_size in (1, 2, 4, 8):
        for count in HUGE_COUNTS:
            for num_objects in (None, U64, count, count + 3):
                # Top-level dict: int/real/str/data/nested/streams/peers.
                out.append(assemble(
                    [container(0xD0, [1, 2], ref_size, count),
                     ascii_obj("params"), int_obj(1)],
                    ref_size=ref_size, num_objects=num_objects))
                # A valid top dict whose "params"/"peers"/"streams" value is
                # the hostile container.
                for key in ("params", "peers", "streams", "deep"):
                    for marker in (0xD0, 0xA0):
                        out.append(assemble(
                            [container(0xD0, [1, 2], ref_size),
                             ascii_obj(key),
                             container(marker, [1, 1], ref_size, count)],
                            ref_size=ref_size, num_objects=num_objects))
                # SETPEERS: a top-level array.
                out.append(assemble(
                    [container(0xA0, [1], ref_size, count),
                     ascii_obj("1.2.3.4")],
                    ref_size=ref_size, num_objects=num_objects))
                # A stream dict inside a valid streams array.
                out.append(assemble(
                    [container(0xD0, [1, 2], ref_size), ascii_obj("streams"),
                     container(0xA0, [3], ref_size),
                     container(0xD0, [4, 5], ref_size, count),
                     ascii_obj("type"), int_obj(96)],
                    ref_size=ref_size, num_objects=num_objects))
                # A streams array whose count wraps.
                out.append(assemble(
                    [container(0xD0, [1, 2], ref_size), ascii_obj("streams"),
                     container(0xA0, [0], ref_size, count)],
                    ref_size=ref_size, num_objects=num_objects))

    # Strings and data whose length wraps `pos + len` or `len * 2`.
    for count in HUGE_COUNTS:
        for marker in (0x40, 0x50, 0x60):
            value = counted(marker, count, wide=True) + b"ab"
            for key in ("str", "data", "deep", "type", "ekey", "peers"):
                out.append(assemble([container(0xD0, [1, 2], 1),
                                     ascii_obj(key), value]))
                out.append(assemble([container(0xD0, [1, 2], 1),
                                     ascii_obj("streams"),
                                     container(0xA0, [3], 1),
                                     container(0xD0, [4, 2], 1),
                                     ascii_obj(key)]))
            # ...and as a key.
            out.append(assemble([container(0xD0, [1, 1], 1), value]))
    # Length markers that are not 1-8 byte integers, or run off the end.
    for len_marker in (b"\x14", b"\x17", b"\x1f", b"\x23", b"\x50", b"\x13"):
        for marker in (0x4F, 0x5F, 0x6F, 0xAF, 0xDF):
            out.append(assemble([container(0xD0, [1, 2], 1), ascii_obj("str"),
                                 bytes([marker]) + len_marker]))
            out.append(assemble([bytes([marker]) + len_marker]))
    # Integers and reals of odd widths, cut short at the end of the objects.
    for marker in range(0x10, 0x30):
        for tail in (b"", b"\x01", b"\x01" * 3, b"\x01" * 7, b"\x01" * 16):
            value = bytes([marker]) + tail
            for key in ("int", "real", "type"):
                out.append(assemble([container(0xD0, [1, 2], 1),
                                     ascii_obj(key), value]))
            out.append(assemble([container(0xD0, [1, 2], 1),
                                 ascii_obj("params"),
                                 container(0xD0, [3, 4], 1),
                                 ascii_obj("value"), value]))

    # Trailer fields.
    good = [container(0xD0, [1, 2, 3, 4], 1), ascii_obj("int"),
            ascii_obj("params"), int_obj(7), container(0xD0, [5, 3], 1),
            ascii_obj("value")]
    base = assemble(good)
    table_at = len(base) - 32 - 8 * len(good)
    for offset_size in (0, 1, 3, 7, 9, 16, 255):
        out.append(base[:-26] + bytes([offset_size]) + base[-25:])
    for ref_size in (0, 2, 9, 255):
        out.append(base[:-25] + bytes([ref_size]) + base[-24:])
    for num_objects in (0, 5, 7, 8, 1 << 32, (1 << 32) + 6, 1 << 61,
                        1 << 63, U64):
        out.append(assemble(good, num_objects=num_objects))
    for top in (5, 6, 7, 1 << 32, U64):
        out.append(assemble(good, top=top))
    for table in (0, 1, 7, 8, 9, table_at - 1, table_at + 1, len(base) - 33,
                  len(base) - 32, len(base) - 31, len(base), len(base) + 1,
                  1 << 32, U64 - 7, U64):
        out.append(assemble(good, table_offset=table))
    # Offsets outside the object area.
    for bad in (0, 1, 7, table_at, table_at + 1, len(base) - 32,
                len(base) - 1, len(base), 1 << 32, U64):
        for which in range(len(good)):
            offsets = [8 + sum(map(len, good[:i])) for i in range(len(good))]
            offsets[which] = bad
            out.append(assemble(good, offsets=offsets))
    # References outside the object table.
    for ref in (6, 7, 0xFF):
        out.append(assemble([container(0xD0, [1, ref], 1),
                             ascii_obj("params")]))
        out.append(assemble([container(0xD0, [ref, 1], 1),
                             ascii_obj("params")]))
        out.append(assemble([container(0xA0, [ref, 0], 1)]))

    # Containers that hold themselves, many times over: a walk that does not
    # bound its work would run for ever.
    for marker in (0xD0, 0xA0):
        refs = [0] * 30 if marker == 0xA0 else [1] * 15 + [0] * 15
        out.append(assemble([container(marker, refs, 1), ascii_obj("x")]))
        out.append(assemble([container(0xD0, [1, 2], 1), ascii_obj("peers"),
                             container(marker, [2] * 30, 1)]))
        out.append(assemble([container(0xD0, [1, 2], 1), ascii_obj("streams"),
                             container(0xA0, [2] * 14 + [0], 1)]))
    return out


def mutations(plists, rng, per_plist):
    for data in plists:
        for _ in range(per_plist):
            b = bytearray(data)
            for _ in range(rng.randrange(1, 5)):
                where = rng.random()
                if where < 0.3:
                    i = len(b) - 32 + rng.randrange(6, 32)  # trailer
                else:
                    i = rng.randrange(8, len(b))
                b[i] = rng.choice((0x00, 0x0F, 0x1F, 0x4F, 0x5F, 0x6F, 0xAF,
                                   0xDF, 0xFF, 0x80, 0x13,
                                   rng.randrange(256), b[i] ^ 0xFF))
            yield bytes(b)


def run(binary, records, timeout):
    stream = b"".join(struct.pack("<I", len(r)) + r for r in records)
    try:
        result = subprocess.run([str(binary)], input=stream,
                                capture_output=True, timeout=timeout)
        status, out, err = f"exit {result.returncode}", result.stdout, \
            result.stderr
        failed = result.returncode != 0
    except subprocess.TimeoutExpired as e:
        status, out, err = f"no exit within {timeout} s", e.stdout or b"", \
            e.stderr or b""
        failed = True
    lines = out.decode().splitlines()
    if failed or len(lines) != len(records):
        culprit = records[len(lines)] if len(lines) < len(records) else b""
        raise SystemExit(
            f"FAIL: {status} after {len(lines)} of {len(records)} records; "
            f"next record {culprit.hex()}\n"
            + err.decode(errors="replace")[-4000:])
    return lines


def main():
    with tempfile.TemporaryDirectory(prefix="bplist-fuzz-tests-") as temporary:
        build = Path(temporary)
        command = shlex.split(os.environ.get("CC", "cc")) + [
            "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-g", "-O1",
            "-fsanitize=address,undefined", "-fno-sanitize-recover=all",
            "-fno-omit-frame-pointer",
            "-I", str(ROOT / "main/plist"),
            str(HERE / "test.c"), str(ROOT / "main/plist/bplist_parser.c"),
            "-o", str(build / "test"),
        ]
        print("Binary plist parser: valid and hostile input", flush=True)
        subprocess.run(command, check=True)
        binary = build / "test"

        values = valid_corpus()
        valid = [plistlib.dumps(v, fmt=plistlib.FMT_BINARY) for v in values]
        lines = run(binary, valid, timeout=120)
        failures = 0
        for value, data, line in zip(values, valid, lines):
            got, want = parse_line(line), model(value)
            if got != want:
                failures += 1
                if failures <= 5:
                    diff = {k: (got.get(k), want[k]) for k in want
                            if got.get(k) != want[k]}
                    print(f"FAIL: {data.hex()}\n  {value!r:.300}\n"
                          f"  (parser, model): {diff}")
        if failures:
            raise SystemExit(f"FAIL: {failures} of {len(valid)} valid plists")

        hostile = hostile_corpus()
        run(binary, hostile, timeout=120)

        truncated = [data[:cut] for data in valid for cut in range(len(data))]
        run(binary, truncated, timeout=300)

        rng = random.Random(1337)
        mutated = list(mutations(valid + hostile[::7], rng, 30))
        run(binary, mutated, timeout=300)

        print(f"PASS: {len(valid)} valid plists match plistlib; "
              f"{len(hostile)} hostile, {len(truncated)} truncated and "
              f"{len(mutated)} mutated plists read safely")


if __name__ == "__main__":
    main()
