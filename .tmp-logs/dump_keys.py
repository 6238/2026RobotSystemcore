"""Dump WPILOG start-record keys matching drive/heading fields."""

from __future__ import annotations

import struct
import sys
from pathlib import Path


def read_uint_le(buf: bytes, offset: int, n: int) -> tuple[int, int]:
    val = 0
    for i in range(n):
        val |= buf[offset + i] << (8 * i)
    return val, offset + n


def parse_start_records(path: Path) -> list[tuple[int, str, str]]:
    data = path.read_bytes()
    if data[:6] != b"WPILOG":
        raise ValueError(f"bad magic: {path}")
    extra_len = struct.unpack_from("<I", data, 8)[0]
    i = 12 + extra_len
    starts: list[tuple[int, str, str]] = []
    n = len(data)
    empty_controls = 0
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
        _, i = read_uint_le(data, i, ts_len)
        if i + payload_size > n:
            break
        payload = data[i : i + payload_size]
        i += payload_size
        if entry_id != 0:
            continue
        if not payload:
            empty_controls += 1
            continue
        kind = payload[0]
        if kind != 0:
            continue
        if len(payload) < 9:
            continue
        eid = struct.unpack_from("<I", payload, 1)[0]
        j = 5
        name_len = struct.unpack_from("<I", payload, j)[0]
        j += 4
        name = payload[j : j + name_len].decode("utf-8", "replace")
        j += name_len
        type_len = struct.unpack_from("<I", payload, j)[0]
        j += 4
        type_str = payload[j : j + type_len].decode("utf-8", "replace")
        starts.append((eid, name, type_str))
    print(f"# empty control records skipped: {empty_controls}", file=sys.stderr)
    return starts


def main() -> None:
    path = Path(sys.argv[1])
    starts = parse_start_records(path)
    needles = (
        "gyro",
        "yaw",
        "odom",
        "chassis",
        "swerve",
        "heading",
        "omega",
        "joystick",
        "xbox",
        "driver",
        "enabled",
        "teleop",
        "auton",
        "robotstate",
        "pose",
        "rotation",
        "stick",
    )
    print(f"# {path.name}: {len(starts)} fields")
    for eid, name, type_str in sorted(starts, key=lambda t: t[1].lower()):
        lname = name.lower()
        if any(n in lname for n in needles):
            print(f"{type_str:28} {name}")


if __name__ == "__main__":
    main()
