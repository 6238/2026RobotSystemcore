"""Measure teleop heading walk while rotation command is ~0."""

from __future__ import annotations

import json
import math
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

from dump_keys import parse_start_records, read_uint_le

KEYS = {
    "yaw": "/Drive/Gyro/YawPosition",
    "yaw_rate": "/Drive/Gyro/YawVelocityRadPerSec",
    "speeds_set": "/RealOutputs/SwerveChassisSpeeds/Setpoints",
    "speeds_meas": "/RealOutputs/SwerveChassisSpeeds/Measured",
    "pose": "/RealOutputs/Odometry/Robot",
    "enabled": "/DriverStation/Enabled",
    "auto": "/DriverStation/Autonomous",
    "state": "/RealOutputs/Superstructure/CurrentSuperState",
    "modules_set": "/RealOutputs/SwerveStates/SetpointsOptimized",
    "modules_meas": "/RealOutputs/SwerveStates/Measured",
}

OMEGA_HOLD = 0.08  # rad/s ~ 4.6 deg/s; stick deadband maps to 0
V_MOVE = 0.50  # m/s
MIN_EPISODE = 0.40  # s
BLOCK_OMEGA = 0.80  # rad/s commanded turn
BLOCK_YAW = 0.15  # rad/s gyro barely moving
AIM_STATES = {
    "SHOOTING",
    "SHOOT_INTAKE",
    "PASSING",
    "PASS_INTAKE",
}


def wrap(a: float, b: float) -> float:
    return (b - a + math.pi) % (2 * math.pi) - math.pi


def deg(rad: float) -> float:
    return math.degrees(rad)


def decode(type_str: str, payload: bytes):
    if type_str == "boolean":
        return payload[0] != 0 if payload else False
    if type_str == "double":
        return struct.unpack_from("<d", payload)[0]
    if type_str == "string":
        return payload.decode("utf-8", "replace")
    if type_str == "struct:Rotation2d":
        return struct.unpack_from("<d", payload)[0]
    if type_str == "struct:ChassisSpeeds":
        vx, vy, omega = struct.unpack_from("<ddd", payload)
        return (vx, vy, omega)
    if type_str == "struct:Pose2d":
        x, y, theta = struct.unpack_from("<ddd", payload)
        return (x, y, theta)
    if type_str == "struct:SwerveModuleState[]":
        n = len(payload) // 16
        return [struct.unpack_from("<dd", payload, i * 16) for i in range(n)]
    return None


def extract(path: Path) -> dict[str, list]:
    starts = parse_start_records(path)
    id_to = {eid: (name, t) for eid, name, t in starts}
    want_ids = {}
    types = {}
    for key in KEYS.values():
        for eid, name, t in starts:
            if name == key:
                want_ids[eid] = key
                types[key] = t
                break
    missing = [k for k in KEYS.values() if k not in types]
    if missing:
        raise RuntimeError(f"{path.name} missing {missing}")

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
        key = want_ids.get(entry_id)
        if key is None:
            continue
        val = decode(types[key], payload)
        if val is not None:
            series[key].append((ts, val))
    return series


def forward_fill(series: list, t: int, last_idx: list[int]):
    """Advance last_idx[0] so series[idx].ts <= t. Return value or None."""
    idx = last_idx[0]
    while idx + 1 < len(series) and series[idx + 1][0] <= t:
        idx += 1
    last_idx[0] = idx
    if not series or series[idx][0] > t:
        return None
    return series[idx][1]


@dataclass
class Episode:
    kind: str
    t0: float
    t1: float
    dtheta_deg: float
    mean_abs_yaw_dps: float
    mean_v: float
    max_abs_dtheta_step_deg: float
    aiming: bool


def episodes_from(samples: list[dict], pred, kind: str) -> list[Episode]:
    out: list[Episode] = []
    cur = None
    yaw0 = None
    abs_yaw_acc = 0.0
    v_acc = 0.0
    n = 0
    max_step = 0.0
    aiming_votes = 0

    def close(t1, yaw):
        nonlocal cur, yaw0, abs_yaw_acc, v_acc, n, max_step, aiming_votes
        if cur is None or n < 2:
            cur = None
            return
        dt = t1 - cur
        if dt < MIN_EPISODE:
            cur = None
            return
        out.append(
            Episode(
                kind=kind,
                t0=cur,
                t1=t1,
                dtheta_deg=deg(wrap(yaw0, yaw)),
                mean_abs_yaw_dps=deg(abs_yaw_acc / n),
                mean_v=v_acc / n,
                max_abs_dtheta_step_deg=deg(max_step),
                aiming=aiming_votes / n > 0.5,
            )
        )
        cur = None

    prev_yaw = None
    for s in samples:
        ok = pred(s)
        if ok and cur is None:
            cur = s["t"]
            yaw0 = s["yaw"]
            abs_yaw_acc = 0.0
            v_acc = 0.0
            n = 0
            max_step = 0.0
            aiming_votes = 0
            prev_yaw = s["yaw"]
        if ok and cur is not None:
            step = abs(wrap(prev_yaw, s["yaw"]))
            max_step = max(max_step, step)
            abs_yaw_acc += abs(s["yaw_rate"])
            v_acc += s["v"]
            n += 1
            if s["aiming"]:
                aiming_votes += 1
            prev_yaw = s["yaw"]
        elif cur is not None:
            close(s["t"], prev_yaw if prev_yaw is not None else s["yaw"])
            prev_yaw = s["yaw"]
        else:
            prev_yaw = s["yaw"]
    if samples and cur is not None:
        close(samples[-1]["t"], samples[-1]["yaw"])
    return out


def analyze(path: Path) -> dict:
    series = extract(path)
    yaw_s = series[KEYS["yaw"]]
    samples = []
    idx = {k: [0] for k in KEYS}

    for ts, yaw in yaw_s:
        enabled = forward_fill(series[KEYS["enabled"]], ts, idx["enabled"])
        auto = forward_fill(series[KEYS["auto"]], ts, idx["auto"])
        speeds = forward_fill(series[KEYS["speeds_set"]], ts, idx["speeds_set"])
        meas = forward_fill(series[KEYS["speeds_meas"]], ts, idx["speeds_meas"])
        yaw_rate = forward_fill(series[KEYS["yaw_rate"]], ts, idx["yaw_rate"])
        state = forward_fill(series[KEYS["state"]], ts, idx["state"])
        mods_set = forward_fill(series[KEYS["modules_set"]], ts, idx["modules_set"])
        mods_meas = forward_fill(series[KEYS["modules_meas"]], ts, idx["modules_meas"])
        if enabled is None or auto is None or speeds is None or yaw_rate is None:
            continue
        vx, vy, omega = speeds
        v = math.hypot(vx, vy)
        meas_omega = meas[2] if meas is not None else float("nan")
        steer_err = 0.0
        if mods_set and mods_meas and len(mods_set) == len(mods_meas):
            steer_err = max(abs(wrap(s[1], m[1])) for s, m in zip(mods_set, mods_meas))
        samples.append(
            {
                "t": ts / 1e6,
                "yaw": yaw,
                "yaw_rate": yaw_rate,
                "omega": omega,
                "meas_omega": meas_omega,
                "v": v,
                "enabled": bool(enabled),
                "auto": bool(auto),
                "aiming": (state or "") in AIM_STATES,
                "steer_err": steer_err,
            }
        )

    teleop = [s for s in samples if s["enabled"] and not s["auto"]]
    auto_s = [s for s in samples if s["enabled"] and s["auto"]]

    def hold_pred(s):
        return s["v"] >= V_MOVE and abs(s["omega"]) <= OMEGA_HOLD and not s["aiming"]

    def aim_hold_pred(s):
        return s["v"] >= V_MOVE and abs(s["omega"]) <= OMEGA_HOLD and s["aiming"]

    def block_pred(s):
        return abs(s["omega"]) >= BLOCK_OMEGA and abs(s["yaw_rate"]) <= BLOCK_YAW

    hold_eps = episodes_from(teleop, hold_pred, "teleop_hold")
    aim_eps = episodes_from(teleop, aim_hold_pred, "aim_hold")
    block_eps = episodes_from(teleop, block_pred, "blocked_turn")

    def summarize(eps: list[Episode]) -> dict:
        if not eps:
            return {
                "n": 0,
                "time_s": 0.0,
                "mean_abs_walk_deg": 0.0,
                "max_abs_walk_deg": 0.0,
                "p90_abs_walk_deg": 0.0,
                "n_gt5": 0,
                "n_gt10": 0,
                "mean_abs_yaw_dps": 0.0,
                "walk_deg_per_s": 0.0,
            }
        abs_walk = [abs(e.dtheta_deg) for e in eps]
        time_s = sum(e.t1 - e.t0 for e in eps)
        abs_walk.sort()
        p90 = abs_walk[int(0.9 * (len(abs_walk) - 1))]
        mean_yaw = sum(e.mean_abs_yaw_dps * (e.t1 - e.t0) for e in eps) / time_s
        return {
            "n": len(eps),
            "time_s": round(time_s, 2),
            "mean_abs_walk_deg": round(sum(abs_walk) / len(abs_walk), 2),
            "max_abs_walk_deg": round(max(abs_walk), 2),
            "p90_abs_walk_deg": round(p90, 2),
            "n_gt5": sum(1 for x in abs_walk if x >= 5),
            "n_gt10": sum(1 for x in abs_walk if x >= 10),
            "mean_abs_yaw_dps": round(mean_yaw, 2),
            "walk_deg_per_s": round(sum(abs_walk) / time_s, 2) if time_s else 0.0,
            "episodes": [
                {
                    "t0": round(e.t0, 2),
                    "dt": round(e.t1 - e.t0, 2),
                    "dtheta_deg": round(e.dtheta_deg, 2),
                    "yaw_dps": round(e.mean_abs_yaw_dps, 2),
                    "v": round(e.mean_v, 2),
                    "aiming": e.aiming,
                }
                for e in sorted(eps, key=lambda x: -abs(x.dtheta_deg))[:8]
            ],
        }

    # Rate while translating with omega~0, vs while aiming
    def rates(rows, pred):
        xs = [abs(s["yaw_rate"]) for s in rows if pred(s)]
        if not xs:
            return {"n": 0, "mean_dps": 0.0, "p90_dps": 0.0, "frac_gt2dps": 0.0, "frac_gt5dps": 0.0}
        xs.sort()
        dps = [deg(x) for x in xs]
        return {
            "n": len(xs),
            "mean_dps": round(sum(dps) / len(dps), 2),
            "p90_dps": round(dps[int(0.9 * (len(dps) - 1))], 2),
            "frac_gt2dps": round(sum(1 for x in dps if x >= 2) / len(dps), 3),
            "frac_gt5dps": round(sum(1 for x in dps if x >= 5) / len(dps), 3),
        }

    steer = [s["steer_err"] for s in teleop if s["v"] >= V_MOVE]
    steer_deg = [deg(x) for x in steer] if steer else []

    teleop_t = (teleop[-1]["t"] - teleop[0]["t"]) if len(teleop) > 1 else 0.0
    hold_time = sum(e.t1 - e.t0 for e in hold_eps)

    return {
        "match": path.name,
        "teleop_s": round(teleop_t, 1),
        "auto_s": round((auto_s[-1]["t"] - auto_s[0]["t"]) if len(auto_s) > 1 else 0.0, 1),
        "teleop_hold_frac": round(hold_time / teleop_t, 3) if teleop_t else 0.0,
        "heading_hold_windows": summarize(hold_eps),
        "aim_hold_windows": summarize(aim_eps),
        "blocked_turn_windows": summarize(block_eps),
        "teleop_translate_hold_yaw": rates(teleop, hold_pred),
        "teleop_aim_hold_yaw": rates(teleop, aim_hold_pred),
        "steer_err_while_moving_deg": {
            "n": len(steer_deg),
            "mean": round(sum(steer_deg) / len(steer_deg), 2) if steer_deg else 0.0,
            "p90": round(sorted(steer_deg)[int(0.9 * (len(steer_deg) - 1))], 2)
            if steer_deg
            else 0.0,
            "max": round(max(steer_deg), 2) if steer_deg else 0.0,
        },
    }


def main() -> None:
    paths = [Path(p) for p in sys.argv[1:]]
    results = []
    for p in paths:
        print(f"analyzing {p.name} ...", file=sys.stderr)
        results.append(analyze(p))
    json.dump(results, sys.stdout, indent=2)
    print()


if __name__ == "__main__":
    main()
