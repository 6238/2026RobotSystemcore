"""Same command / same stick → how tight is ω_gyro? Residual scatter = plant noise."""

from __future__ import annotations

import json
import math
import struct
import sys
from collections import defaultdict
from pathlib import Path

from dump_keys import parse_start_records, read_uint_le

KEYS = {
    "yaw_rate": "/Drive/Gyro/YawVelocityRadPerSec",
    "speeds_set": "/RealOutputs/SwerveChassisSpeeds/Setpoints",
    "speeds_meas": "/RealOutputs/SwerveChassisSpeeds/Measured",
    "enabled": "/DriverStation/Enabled",
    "auto": "/DriverStation/Autonomous",
    "joy": "/DriverStation/Joystick0/AxisValues",
    "state": "/RealOutputs/Superstructure/CurrentSuperState",
}
AIM = {"SHOOTING", "SHOOT_INTAKE", "PASSING", "PASS_INTAKE"}


def deg(r):
    return math.degrees(r)


def decode(type_str, payload):
    if type_str == "boolean":
        return payload[0] != 0 if payload else False
    if type_str == "double":
        return struct.unpack_from("<d", payload)[0]
    if type_str == "string":
        return payload.decode("utf-8", "replace")
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


def mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def std(xs):
    if len(xs) < 2:
        return 0.0
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def percentile(xs, p):
    if not xs:
        return 0.0
    xs = sorted(xs)
    return xs[min(len(xs) - 1, max(0, int(p * (len(xs) - 1))))]


def r2(y, yhat):
    if len(y) < 3:
        return 0.0
    ym = mean(y)
    ss_tot = sum((v - ym) ** 2 for v in y)
    ss_res = sum((a - b) ** 2 for a, b in zip(y, yhat))
    return 0.0 if ss_tot == 0 else 1.0 - ss_res / ss_tot


def rmse(xs):
    return math.sqrt(mean([x * x for x in xs])) if xs else 0.0


def summarize_bin(rows):
    return {
        "n": len(rows),
        "mean": round(mean(rows), 2),
        "std": round(std(rows), 2),
        "p10": round(percentile(rows, 0.1), 2),
        "p50": round(percentile(rows, 0.5), 2),
        "p90": round(percentile(rows, 0.9), 2),
        "spread_p10_p90": round(percentile(rows, 0.9) - percentile(rows, 0.1), 2),
    }


def analyze(path: Path) -> dict:
    series = extract(path)
    idx = {k: [0] for k in KEYS}
    plant_cmd = []
    plant_gyro = []
    plant_meas = []
    stick_rx = []
    stick_gyro = []
    # bins: omega_cmd in 10 deg/s, stick_rx in 0.1
    by_cmd = defaultdict(list)  # cmd_bin -> gyro dps
    by_cmd_v = defaultdict(list)  # (cmd_bin, v_hi) -> gyro
    by_stick = defaultdict(list)
    resid_cmd0_v = defaultdict(list)  # v bin -> residual dps while cmd~0

    t_axis = series[KEYS["yaw_rate"]]
    for ts, yaw_rate in t_axis:
        enabled = ff(series[KEYS["enabled"]], ts, idx["enabled"])
        auto = ff(series[KEYS["auto"]], ts, idx["auto"])
        speeds = ff(series[KEYS["speeds_set"]], ts, idx["speeds_set"])
        meas = ff(series[KEYS["speeds_meas"]], ts, idx["speeds_meas"])
        joy = ff(series[KEYS["joy"]], ts, idx["joy"])
        state = ff(series[KEYS["state"]], ts, idx["state"])
        if not enabled or auto or speeds is None:
            continue
        vx, vy, omega = speeds
        v = math.hypot(vx, vy)
        gyro_dps = deg(yaw_rate)
        cmd_dps = deg(omega)
        meas_dps = deg(meas[2]) if meas is not None else None
        plant_cmd.append(cmd_dps)
        plant_gyro.append(gyro_dps)
        if meas_dps is not None:
            plant_meas.append(meas_dps)
        cmd_bin = int(round(cmd_dps / 10.0)) * 10  # ..., -10, 0, 10, ...
        by_cmd[cmd_bin].append(gyro_dps)
        v_hi = v >= 1.0
        by_cmd_v[(cmd_bin, v_hi)].append(gyro_dps)
        if abs(cmd_dps) <= 5.0:
            vbin = 0 if v < 0.5 else (1 if v < 1.5 else (2 if v < 2.5 else 3))
            resid_cmd0_v[vbin].append(gyro_dps)

        aiming = (state or "") in AIM
        if joy and len(joy) > 3 and not aiming:
            rx = joy[3]
            stick_rx.append(rx)
            stick_gyro.append(gyro_dps)
            sbin = round(rx * 10) / 10.0  # 0.1 stick bins
            by_stick[sbin].append(gyro_dps)

    resid_gyro = [g - c for g, c in zip(plant_gyro, plant_cmd)]
    resid_meas = [m - c for m, c in zip(plant_meas, plant_cmd[: len(plant_meas)])]

    # interesting cmd bins for a table: -60,-30,-10,0,10,30,60 if populated
    show_bins = [-60, -30, -10, 0, 10, 30, 60]
    cmd_table = []
    for b in show_bins:
        rows = by_cmd.get(b, [])
        if len(rows) < 30:
            continue
        rec = summarize_bin(rows)
        rec["cmd_dps"] = b
        rec["resid_std"] = rec["std"]  # gyro std at fixed cmd
        cmd_table.append(rec)

    stick_table = []
    for b in sorted(by_stick):
        rows = by_stick[b]
        if len(rows) < 40:
            continue
        if abs(b) > 0.6:
            continue
        rec = summarize_bin(rows)
        rec["stick_rx"] = round(b, 1)
        stick_table.append(rec)

    v_labels = {0: "v<0.5", 1: "0.5-1.5", 2: "1.5-2.5", 3: "v>=2.5"}
    cmd0_by_v = []
    for vb, lab in v_labels.items():
        rows = resid_cmd0_v.get(vb, [])
        if len(rows) < 30:
            continue
        rec = summarize_bin(rows)
        rec["v"] = lab
        cmd0_by_v.append(rec)

    return {
        "file": path.name,
        "n": len(plant_cmd),
        "r2_gyro_from_cmd": round(r2(plant_gyro, plant_cmd), 3),
        "r2_meas_from_cmd": round(r2(plant_meas, plant_cmd[: len(plant_meas)]), 3)
        if plant_meas
        else 0,
        "rmse_gyro_minus_cmd_dps": round(rmse(resid_gyro), 2),
        "resid_gyro_std_dps": round(std(resid_gyro), 2),
        "resid_meas_std_dps": round(std(resid_meas), 2),
        "r2_stick_to_gyro": round(r2(stick_gyro, [0.0] * len(stick_gyro)), 3)
        if False
        else round(
            r2(stick_gyro, stick_gyro) * 0 + _stick_r2(stick_rx, stick_gyro),
            3,
        ),
        "cmd_bins": cmd_table,
        "stick_bins": stick_table,
        "cmd0_by_speed": cmd0_by_v,
    }


def _stick_r2(rx, gyro):
    # linear model gyro ~ k * rx
    if len(rx) < 10:
        return 0.0
    num = sum(a * b for a, b in zip(rx, gyro))
    den = sum(a * a for a in rx)
    k = num / den if den else 0.0
    yhat = [k * a for a in rx]
    return r2(gyro, yhat)


def main():
    results = []
    for p in sys.argv[1:]:
        print(f"analyzing {Path(p).name}", file=sys.stderr)
        results.append(analyze(Path(p)))
    pooled = {
        "r2_gyro": [r["r2_gyro_from_cmd"] for r in results],
        "rmse": [r["rmse_gyro_minus_cmd_dps"] for r in results],
        "resid_std": [r["resid_gyro_std_dps"] for r in results],
        "r2_stick": [r["r2_stick_to_gyro"] for r in results],
    }
    json.dump({"pooled": pooled, "matches": results}, sys.stdout, indent=2)
    print()


if __name__ == "__main__":
    main()
