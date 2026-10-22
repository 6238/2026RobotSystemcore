"""Split stick-zero yaw into coast-down vs disturbance-while-holding."""

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
}

STICK_DB = 0.10
OMEGA_HOLD = 0.05  # rad/s
V_MOVE = 0.60
TURN_OMEGA = 0.40  # rad/s ~ 23 deg/s, a real rotate command


def wrap(a, b):
    return (b - a + math.pi) % (2 * math.pi) - math.pi


def deg(r):
    return math.degrees(r)


def decode(type_str, payload):
    if type_str == "boolean":
        return payload[0] != 0 if payload else False
    if type_str == "double":
        return struct.unpack_from("<d", payload)[0]
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


BINS = [0.0, 0.05, 0.10, 0.20, 0.30, 0.50, 0.75, 1.00, 1.50, 2.00]


def nearest_bin(t_since):
    """Return bin label for a sample's age in the hold."""
    for b in reversed(BINS):
        if t_since + 1e-9 >= b:
            return b
    return 0.0


def analyze(path: Path) -> dict:
    series = extract(path)
    idx = {k: [0] for k in KEYS}

    # age of stick-zero / omega-zero, and whether last command was a real turn
    stick_zero_age = None
    omega_zero_age = None
    last_turn_sign = 0.0  # last omega_cmd when |omega| > TURN_OMEGA
    last_turn_age = 1e9
    prev_t = None

    # decay curves: |yaw_dps| keyed by bin, split after-turn vs not
    decay_all = {b: [] for b in BINS}
    decay_after_turn = {b: [] for b in BINS}
    decay_no_turn = {b: [] for b in BINS}
    decay_omega_cmd = {b: [] for b in BINS}  # deg/s of logged ω setpoint
    # signed residual: yaw_rate * sign(last_turn)  — positive = leftover momentum
    signed_after_turn = {b: [] for b in BINS}

    # integrated |dtheta| split by age of stick-zero (moving + stick in db)
    integ_early = 0.0  # age < 0.30 s
    integ_mid = 0.0  # 0.30–1.00 s
    integ_late = 0.0  # ≥ 1.00 s
    integ_after_turn_early = 0.0
    integ_after_turn_late = 0.0
    integ_no_turn_late = 0.0

    n_early = n_mid = n_late = 0
    late_yaw = []
    late_yaw_no_turn = []
    late_yaw_after_turn = []

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
        if not enabled or auto or speeds is None or yaw_rate is None:
            stick_zero_age = None
            omega_zero_age = None
            continue

        vx, vy, omega = speeds
        v = math.hypot(vx, vy)
        stick = joy[3] if joy and len(joy) > 3 else 0.0
        stick_in = abs(stick) <= STICK_DB
        omega_in = abs(omega) <= OMEGA_HOLD

        if abs(omega) >= TURN_OMEGA:
            last_turn_sign = 1.0 if omega > 0 else -1.0
            last_turn_age = 0.0
        else:
            last_turn_age += dt

        if stick_in:
            stick_zero_age = 0.0 if stick_zero_age is None else stick_zero_age + dt
        else:
            stick_zero_age = None

        if omega_in:
            omega_zero_age = 0.0 if omega_zero_age is None else omega_zero_age + dt
        else:
            omega_zero_age = None

        # Stick-zero + translating: includes generator still slewing ω down.
        if not (stick_in and v >= V_MOVE and stick_zero_age is not None):
            continue

        yaw_dps = abs(deg(yaw_rate))
        dtheta = abs(yaw_rate) * dt
        age = stick_zero_age
        after_turn = last_turn_age < 1.50  # rotate command in the last 1.5 s
        b = nearest_bin(age)
        decay_all[b].append(yaw_dps)
        decay_omega_cmd[b].append(abs(deg(omega)))
        if after_turn:
            decay_after_turn[b].append(yaw_dps)
            if last_turn_sign != 0:
                signed_after_turn[b].append(deg(yaw_rate) * last_turn_sign)
        else:
            decay_no_turn[b].append(yaw_dps)

        if age < 0.30:
            integ_early += dtheta
            n_early += 1
            if after_turn:
                integ_after_turn_early += dtheta
        elif age < 1.00:
            integ_mid += dtheta
            n_mid += 1
        else:
            integ_late += dtheta
            n_late += 1
            late_yaw.append(yaw_dps)
            if after_turn:
                integ_after_turn_late += dtheta
                late_yaw_after_turn.append(yaw_dps)
            else:
                integ_no_turn_late += dtheta
                late_yaw_no_turn.append(yaw_dps)

    total_integ = integ_early + integ_mid + integ_late
    total_deg = deg(total_integ)

    def pack_curve(src):
        out = []
        for b in BINS:
            xs = src[b]
            out.append(
                {
                    "t": b,
                    "n": len(xs),
                    "p50": round(percentile(xs, 0.5), 2),
                    "p90": round(percentile(xs, 0.9), 2),
                    "mean": round(mean(xs), 2),
                }
            )
        return out

    def pack_signed(src):
        out = []
        for b in BINS:
            xs = src[b]
            out.append(
                {
                    "t": b,
                    "n": len(xs),
                    "mean": round(mean(xs), 2),
                    "p50": round(percentile(xs, 0.5), 2),
                }
            )
        return out

    return {
        "match": path.name,
        "hold_heading_deg": round(total_deg, 1),
        "share_early_lt_0p3s": round(integ_early / total_integ, 3) if total_integ else 0,
        "share_mid_0p3_1s": round(integ_mid / total_integ, 3) if total_integ else 0,
        "share_late_ge_1s": round(integ_late / total_integ, 3) if total_integ else 0,
        "share_after_turn_of_early": round(integ_after_turn_early / integ_early, 3)
        if integ_early
        else 0,
        "late_after_turn_deg": round(deg(integ_after_turn_late), 1),
        "late_no_turn_deg": round(deg(integ_no_turn_late), 1),
        "n_early": n_early,
        "n_mid": n_mid,
        "n_late": n_late,
        "late_yaw_p50": round(percentile(late_yaw, 0.5), 2),
        "late_yaw_p90": round(percentile(late_yaw, 0.9), 2),
        "late_yaw_mean": round(mean(late_yaw), 2),
        "late_no_turn_p50": round(percentile(late_yaw_no_turn, 0.5), 2),
        "late_no_turn_mean": round(mean(late_yaw_no_turn), 2),
        "late_after_turn_p50": round(percentile(late_yaw_after_turn, 0.5), 2),
        "frac_late_ge_5dps": round(sum(1 for x in late_yaw if x >= 5) / len(late_yaw), 3)
        if late_yaw
        else 0,
        "frac_late_no_turn_ge_5dps": round(
            sum(1 for x in late_yaw_no_turn if x >= 5) / len(late_yaw_no_turn), 3
        )
        if late_yaw_no_turn
        else 0,
        "decay_all": pack_curve(decay_all),
        "decay_after_turn": pack_curve(decay_after_turn),
        "decay_no_turn": pack_curve(decay_no_turn),
        "decay_omega_cmd": pack_curve(decay_omega_cmd),
        "signed_after_turn": pack_signed(signed_after_turn),
    }


def main():
    results = []
    for p in sys.argv[1:]:
        path = Path(p)
        print(f"analyzing {path.name}", file=sys.stderr)
        results.append(analyze(path))
    # pooled decay across matches
    pooled = {
        "share_early": mean([r["share_early_lt_0p3s"] for r in results]),
        "share_mid": mean([r["share_mid_0p3_1s"] for r in results]),
        "share_late": mean([r["share_late_ge_1s"] for r in results]),
        "heading_deg": sum(r["hold_heading_deg"] for r in results),
        "late_p50": [r["late_yaw_p50"] for r in results],
        "late_no_turn_p50": [r["late_no_turn_p50"] for r in results],
    }
    json.dump({"pooled": pooled, "matches": results}, sys.stdout, indent=2)
    print()


if __name__ == "__main__":
    main()
