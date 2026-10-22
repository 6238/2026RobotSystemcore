"""Coast-down vs disturbance while rate-control is actually holding ω≈0."""

from __future__ import annotations

import json
import math
import struct
import sys
from pathlib import Path

from dump_keys import parse_start_records, read_uint_le

KEYS = {
    "yaw": "/Drive/Gyro/YawPosition",
    "yaw_rate": "/Drive/Gyro/YawVelocityRadPerSec",
    "speeds_set": "/RealOutputs/SwerveChassisSpeeds/Setpoints",
    "enabled": "/DriverStation/Enabled",
    "auto": "/DriverStation/Autonomous",
    "joy": "/DriverStation/Joystick0/AxisValues",
    "state": "/RealOutputs/Superstructure/CurrentSuperState",
}

STICK_DB = 0.10
OMEGA_HOLD = 0.08  # rad/s
V_MOVE = 0.60
RELEASE_FROM = 0.50  # rad/s, a real rotate command
AIM = {"SHOOTING", "SHOOT_INTAKE", "PASSING", "PASS_INTAKE"}
AGE_EDGES = [0.0, 0.10, 0.20, 0.35, 0.50, 0.75, 1.00, 2.00]


def deg(r):
    return math.degrees(r)


def decode(type_str, payload):
    if type_str == "boolean":
        return payload[0] != 0 if payload else False
    if type_str == "double":
        return struct.unpack_from("<d", payload)[0]
    if type_str == "string":
        return payload.decode("utf-8", "replace")
    if type_str == "struct:Rotation2d":
        return struct.unpack_from("<d", payload)[0]
    if type_str == "struct:ChassisSpeeds":
        return struct.unpack_from("<ddd", payload)
    if type_str == "float[]":
        n = len(payload) // 4
        return list(struct.unpack_from(f"<{n}f", payload))
    return None


def extract(path: Path):
    starts = parse_start_records(path)
    want_ids, types = {}, {}
    for key in KEYS.values():
        for eid, name, t in starts:
            if name == key:
                want_ids[eid] = key
                types[key] = t
                break
    series = {k: [] for k in KEYS.values()}
    data = path.read_bytes()
    extra_len = struct.unpack_from("<I", data, 8)[0]
    i = 12 + extra_len
    n = len(data)
    while i < n:
        tag = data[i]
        i += 1
        id_len = (tag & 0x03) + 1
        size_len = ((tag >> 2) & 0x03) + 1
        ts_len = ((tag >> 4) & 0x07) + 1
        if i + id_len + size_len + ts_len > n:
            break
        entry_id, i = read_uint_le(data, i, id_len)
        payload_size, i = read_uint_le(data, i, size_len)
        ts, i = read_uint_le(data, i, ts_len)
        if i + payload_size > n:
            break
        payload = data[i : i + payload_size]
        i += payload_size
        key = want_ids.get(entry_id)
        if key is None:
            continue
        val = decode(types[key], payload)
        if val is not None:
            series[key].append((ts, val))
    return series


def ff(series, t, idx):
    while idx[0] + 1 < len(series) and series[idx[0] + 1][0] <= t:
        idx[0] += 1
    if not series or series[idx[0]][0] > t:
        return None
    return series[idx[0]][1]


def percentile(xs, p):
    if not xs:
        return 0.0
    xs = sorted(xs)
    return xs[min(len(xs) - 1, max(0, int(p * (len(xs) - 1))))]


def mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def age_bin(age):
    for i in range(len(AGE_EDGES) - 1, -1, -1):
        if age + 1e-9 >= AGE_EDGES[i]:
            return AGE_EDGES[i]
    return 0.0


def analyze(path: Path) -> dict:
    series = extract(path)
    idx = {k: [0] for k in KEYS}
    prev_t = None
    prev_omega = None
    hold_age = None

    # event overlay: |yaw| after omega_cmd falls through RELEASE_FROM → HOLD
    releases = []  # list of {t0, yaw_at, samples: [(dt, yaw_dps)]}
    open_release = None

    decay = {b: [] for b in AGE_EDGES}
    integ = {"lt_0p35": 0.0, "0p35_1": 0.0, "ge_1": 0.0}
    late = []
    early = []

    for ts, yaw in series[KEYS["yaw"]]:
        t = ts / 1e6
        dt = (t - prev_t) if prev_t is not None else 0.02
        if dt <= 0 or dt > 0.2:
            dt = 0.02
        prev_t = t
        enabled = ff(series[KEYS["enabled"]], ts, idx["enabled"])
        auto = ff(series[KEYS["auto"]], ts, idx["auto"])
        speeds = ff(series[KEYS["speeds_set"]], ts, idx["speeds_set"])
        yaw_rate = ff(series[KEYS["yaw_rate"]], ts, idx["yaw_rate"])
        joy = ff(series[KEYS["joy"]], ts, idx["joy"])
        state = ff(series[KEYS["state"]], ts, idx["state"])
        if not enabled or auto or speeds is None or yaw_rate is None:
            hold_age = None
            open_release = None
            prev_omega = None
            continue

        vx, vy, omega = speeds
        v = math.hypot(vx, vy)
        stick = joy[3] if joy and len(joy) > 3 else 0.0
        aiming = (state or "") in AIM
        stick_in = abs(stick) <= STICK_DB
        omega_in = abs(omega) <= OMEGA_HOLD
        holding = stick_in and omega_in and v >= V_MOVE and not aiming

        # release event: was rotating, now holding
        if (
            prev_omega is not None
            and abs(prev_omega) >= RELEASE_FROM
            and omega_in
            and stick_in
            and not aiming
        ):
            open_release = {"t0": t, "yaw0_dps": abs(deg(yaw_rate)), "pts": []}
            releases.append(open_release)
        if open_release is not None:
            age = t - open_release["t0"]
            if age <= 1.25 and stick_in and not aiming:
                open_release["pts"].append((round(age, 3), abs(deg(yaw_rate)), abs(deg(omega))))
            else:
                open_release = None

        if holding:
            hold_age = 0.0 if hold_age is None else hold_age + dt
            yaw_dps = abs(deg(yaw_rate))
            dth = abs(yaw_rate) * dt
            decay[age_bin(hold_age)].append(yaw_dps)
            if hold_age < 0.35:
                integ["lt_0p35"] += dth
                early.append(yaw_dps)
            elif hold_age < 1.00:
                integ["0p35_1"] += dth
            else:
                integ["ge_1"] += dth
                late.append(yaw_dps)
        else:
            hold_age = None

        prev_omega = omega

    total = sum(integ.values()) or 1.0
    # stack release overlays onto a coarse grid
    rel_grid = [0.0, 0.05, 0.10, 0.20, 0.35, 0.50, 0.75, 1.00]
    rel_yaw = {g: [] for g in rel_grid}
    rel_cmd = {g: [] for g in rel_grid}
    n_rel = 0
    for rel in releases:
        if len(rel["pts"]) < 4:
            continue
        n_rel += 1
        for age, yaw_dps, cmd_dps in rel["pts"]:
            g = min(rel_grid, key=lambda x: abs(x - age))
            rel_yaw[g].append(yaw_dps)
            rel_cmd[g].append(cmd_dps)

    return {
        "file": path.name,
        "share_settle_lt_0p35s": round(integ["lt_0p35"] / total, 3),
        "share_0p35_1s": round(integ["0p35_1"] / total, 3),
        "share_disturbance_ge_1s": round(integ["ge_1"] / total, 3),
        "hold_heading_deg": round(deg(sum(integ.values())), 1),
        "settle_heading_deg": round(deg(integ["lt_0p35"]), 1),
        "disturbance_heading_deg": round(deg(integ["ge_1"]), 1),
        "early_yaw_p50": round(percentile(early, 0.5), 2),
        "early_yaw_mean": round(mean(early), 2),
        "late_yaw_p50": round(percentile(late, 0.5), 2),
        "late_yaw_mean": round(mean(late), 2),
        "late_frac_ge_5dps": round(sum(1 for x in late if x >= 5) / len(late), 3) if late else 0,
        "n_late": len(late),
        "n_early": len(early),
        "n_releases_used": n_rel,
        "decay_hold": [
            {
                "t": b,
                "n": len(decay[b]),
                "p50": round(percentile(decay[b], 0.5), 2),
                "mean": round(mean(decay[b]), 2),
            }
            for b in AGE_EDGES
        ],
        "release_yaw_p50": [
            {"t": g, "n": len(rel_yaw[g]), "p50": round(percentile(rel_yaw[g], 0.5), 2)}
            for g in rel_grid
        ],
        "release_cmd_p50": [
            {"t": g, "n": len(rel_cmd[g]), "p50": round(percentile(rel_cmd[g], 0.5), 2)}
            for g in rel_grid
        ],
    }


def main():
    results = [analyze(Path(p)) for p in sys.argv[1:]]
    w = [r["hold_heading_deg"] for r in results]
    pooled = {
        "share_settle": round(
            sum(r["settle_heading_deg"] for r in results) / sum(w), 3
        ),
        "share_disturbance": round(
            sum(r["disturbance_heading_deg"] for r in results) / sum(w), 3
        ),
        "late_p50": [r["late_yaw_p50"] for r in results],
        "early_p50": [r["early_yaw_p50"] for r in results],
        "n_releases": sum(r["n_releases_used"] for r in results),
    }
    json.dump({"pooled": pooled, "matches": results}, sys.stdout, indent=2)
    print()


if __name__ == "__main__":
    for p in sys.argv[1:]:
        print(f"analyzing {Path(p).name}", file=sys.stderr)
    main()
