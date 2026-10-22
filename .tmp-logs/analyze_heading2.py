"""Refined heading-walk analysis: joystick confirm, gentle vs violent, omega residual."""

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
    "state": "/RealOutputs/Superstructure/CurrentSuperState",
    "joy": "/DriverStation/Joystick0/AxisValues",
}

OMEGA_HOLD = 0.05  # rad/s
STICK_HOLD = 0.10
V_MOVE = 0.60
MIN_EPISODE = 0.50
AIM_STATES = {"SHOOTING", "SHOOT_INTAKE", "PASSING", "PASS_INTAKE"}


def wrap(a, b):
    return (b - a + math.pi) % (2 * math.pi) - math.pi


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
    want_ids = {}
    types = {}
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
    return xs[int(p * (len(xs) - 1))]


def analyze(path: Path) -> dict:
    series = extract(path)
    idx = {k: [0] for k in KEYS}
    hold_samples = []
    all_teleop_v = 0
    hold_v_time = 0.0
    prev_t = None
    omega_in_hold = []
    stick_in_hold = []
    residual_dps = []
    gentle_windows = []
    all_windows = []

    cur = None
    yaw0 = None
    t0 = None
    acc = None

    def close_window(t, yaw):
        nonlocal cur, yaw0, t0, acc
        if cur is None or acc is None or t - t0 < MIN_EPISODE:
            cur = None
            acc = None
            return
        dt = t - t0
        dtheta = deg(wrap(yaw0, yaw))
        mean_dps = acc["abs_yaw"] / acc["n"]
        mean_v = acc["v"] / acc["n"]
        mean_omega = acc["omega"] / acc["n"]
        mean_stick = acc["stick"] / acc["n"]
        rec = {
            "t0": round(t0, 2),
            "dt": round(dt, 2),
            "dtheta_deg": round(dtheta, 2),
            "yaw_dps": round(mean_dps, 2),
            "v": round(mean_v, 2),
            "omega_cmd": round(mean_omega, 4),
            "stick_rx": round(mean_stick, 3),
        }
        all_windows.append(rec)
        # Gentle: driver holding a heading, not getting spun.
        if 1.5 <= mean_dps <= 18.0 and abs(dtheta) >= 3.0:
            gentle_windows.append(rec)
        cur = None
        acc = None

    for ts, yaw in series[KEYS["yaw"]]:
        enabled = ff(series[KEYS["enabled"]], ts, idx["enabled"])
        auto = ff(series[KEYS["auto"]], ts, idx["auto"])
        speeds = ff(series[KEYS["speeds_set"]], ts, idx["speeds_set"])
        yaw_rate = ff(series[KEYS["yaw_rate"]], ts, idx["yaw_rate"])
        state = ff(series[KEYS["state"]], ts, idx["state"])
        joy = ff(series[KEYS["joy"]], ts, idx["joy"])
        t = ts / 1e6
        dt = (t - prev_t) if prev_t is not None else 0.02
        prev_t = t
        if not enabled or auto or speeds is None or yaw_rate is None:
            if cur is not None:
                close_window(t, yaw)
            continue
        vx, vy, omega = speeds
        v = math.hypot(vx, vy)
        stick = joy[3] if joy and len(joy) > 3 else 0.0
        aiming = (state or "") in AIM_STATES
        holding = (
            v >= V_MOVE
            and abs(omega) <= OMEGA_HOLD
            and abs(stick) <= STICK_HOLD
            and not aiming
        )
        all_teleop_v += 1
        if holding:
            hold_v_time += dt
            hold_samples.append(abs(yaw_rate))
            omega_in_hold.append(abs(omega))
            stick_in_hold.append(abs(stick))
            residual_dps.append(deg(abs(yaw_rate - omega)))
            if cur is None:
                cur = True
                yaw0 = yaw
                t0 = t
                acc = {"n": 0, "abs_yaw": 0.0, "v": 0.0, "omega": 0.0, "stick": 0.0}
            acc["n"] += 1
            acc["abs_yaw"] += abs(deg(yaw_rate))
            acc["v"] += v
            acc["omega"] += abs(omega)
            acc["stick"] += abs(stick)
        elif cur is not None:
            close_window(t, yaw)

    if cur is not None and series[KEYS["yaw"]]:
        close_window(series[KEYS["yaw"]][-1][0] / 1e6, series[KEYS["yaw"]][-1][1])

    def pack(windows, samples_dps=None):
        abs_walk = [abs(w["dtheta_deg"]) for w in windows]
        time_s = sum(w["dt"] for w in windows)
        return {
            "n": len(windows),
            "time_s": round(time_s, 2),
            "mean_abs_walk_deg": round(sum(abs_walk) / len(abs_walk), 2) if abs_walk else 0,
            "p50_abs_walk_deg": round(percentile(abs_walk, 0.5), 2),
            "p90_abs_walk_deg": round(percentile(abs_walk, 0.9), 2),
            "max_abs_walk_deg": round(max(abs_walk), 2) if abs_walk else 0,
            "n_gt5": sum(1 for x in abs_walk if x >= 5),
            "n_gt8": sum(1 for x in abs_walk if x >= 8),
            "n_gt10": sum(1 for x in abs_walk if x >= 10),
            "walk_deg_per_s": round(sum(abs_walk) / time_s, 2) if time_s else 0,
            "worst": sorted(windows, key=lambda w: -abs(w["dtheta_deg"]))[:5],
        }

    dps = [deg(x) for x in hold_samples]
    return {
        "match": path.stem.replace("akit_26-", "").replace("_", " "),
        "file": path.name,
        "hold_samples": len(hold_samples),
        "omega_cmd_mean": round(sum(omega_in_hold) / len(omega_in_hold), 4) if omega_in_hold else 0,
        "omega_cmd_p90": round(percentile(omega_in_hold, 0.9), 4) if omega_in_hold else 0,
        "stick_mean": round(sum(stick_in_hold) / len(stick_in_hold), 4) if stick_in_hold else 0,
        "stick_p90": round(percentile(stick_in_hold, 0.9), 4) if stick_in_hold else 0,
        "yaw_dps_mean": round(sum(dps) / len(dps), 2) if dps else 0,
        "yaw_dps_p50": round(percentile(dps, 0.5), 2),
        "yaw_dps_p90": round(percentile(dps, 0.9), 2),
        "frac_gt2dps": round(sum(1 for x in dps if x >= 2) / len(dps), 3) if dps else 0,
        "frac_gt5dps": round(sum(1 for x in dps if x >= 5) / len(dps), 3) if dps else 0,
        "frac_gt10dps": round(sum(1 for x in dps if x >= 10) / len(dps), 3) if dps else 0,
        "residual_dps_mean": round(sum(residual_dps) / len(residual_dps), 2) if residual_dps else 0,
        "all_hold_windows": pack(all_windows),
        "gentle_walk_windows": pack(gentle_windows),
    }


def main():
    results = []
    for p in sys.argv[1:]:
        path = Path(p)
        print(f"analyzing {path.name}", file=sys.stderr)
        results.append(analyze(path))
    json.dump(results, sys.stdout, indent=2)
    print()


if __name__ == "__main__":
    main()
