"""Print joystick names and a few axis samples."""

from __future__ import annotations

import struct
import sys
from pathlib import Path

from dump_keys import parse_start_records, read_uint_le


def decode(type_str: str, payload: bytes):
    if type_str == "string":
        return payload.decode("utf-8", "replace")
    if type_str == "boolean":
        return payload[0] != 0
    if type_str == "double":
        return struct.unpack_from("<d", payload)[0]
    if type_str == "float[]":
        n = len(payload) // 4
        return list(struct.unpack_from(f"<{n}f", payload))
    return None


def main() -> None:
    path = Path(sys.argv[1])
    starts = parse_start_records(path)
    want = {name for _, name, _ in starts if "Joystick" in name and ("Name" in name or "AxisValues" in name or "Xbox" in name)}
    id_of = {name: eid for eid, name, _ in starts}
    types = {name: t for _, name, t in starts}

    data = path.read_bytes()
    extra_len = struct.unpack_from("<I", data, 8)[0]
    i = 12 + extra_len
    n = len(data)
    last = {}
    samples = {k: [] for k in want if "AxisValues" in k}

    while i < n:
        tag = data[i]
        i += 1
        id_len = (tag & 0x03) + 1
        size_len = ((tag >> 2) & 0x03) + 1
        ts_len = ((tag >> 4) & 0x07) + 1
        need = id_len + size_len + ts_len
        if i + need > n:
            break
        entry_id, i = read_uint_le(data, i, id_len)
        payload_size, i = read_uint_le(data, i, size_len)
        ts, i = read_uint_le(data, i, ts_len)
        if i + payload_size > n:
            break
        payload = data[i : i + payload_size]
        i += payload_size
        for name in want:
            if id_of.get(name) == entry_id:
                val = decode(types[name], payload)
                last[name] = val
                if "AxisValues" in name and len(samples[name]) < 3:
                    samples[name].append((ts, val))

    for k in sorted(last):
        if "Axis" not in k:
            print(f"{k}: {last[k]}")
    print("--- axis samples ---")
    for k, pts in samples.items():
        print(k, pts[:2])


if __name__ == "__main__":
    main()
