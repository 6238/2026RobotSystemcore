#!/usr/bin/env python3
"""Overlay logged heading vs stick-integrated heading hold.

Rate control: stick at 0 means omega=0, heading is whatever you have.
Heading hold: theta_set += omega_stick * dt while rotating, freeze in deadband.

  python3.13 scripts/overlay_heading_hold.py path/to/match.wpilog

matplotlib is on the Windows Store 3.13 install, not msys python.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
from pathlib import Path

# 2026 DriveCommands (Rebuilt). 2027 template uses deadband 0.1 and squares stick.
DEADBAND = 0.07
CURVE_CENTER_SENS = 1.2
CURVE_MAX_RATE = 0.8
CURVE_EXPO = 3.0
# kSpeedAt12Volts / drive-base radius, COMP ~4.3 m/s over ~0.39 m.
OMEGA_MAX = 11.0

AIM = {"SHOOTING", "SHOOT_INTAKE", "PASSING", "PASS_INTAKE"}
KEYS = {
    "yaw": "/Drive/Gyro/YawPosition",
    "enabled": "/DriverStation/Enabled",
    "auto": "/DriverStation/Autonomous",
    "joy": "/DriverStation/Joystick0/AxisValues",
    "pose": "/RealOutputs/Odometry/Robot",
    "state": "/RealOutputs/Superstructure/CurrentSuperState",
}


def apply_deadband(x: float, db: float) -> float:
    if abs(x) < db:
        return 0.0
    return math.copysign((abs(x) - db) / (1.0 - db), x)


def apply_curve(x: float) -> float:
    return (CURVE_CENTER_SENS * x + (CURVE_MAX_RATE - CURVE_CENTER_SENS) * x**CURVE_EXPO) / CURVE_MAX_RATE


def stick_to_omega(right_x: float, omega_max: float) -> float:
    # DriveCommands uses -getRightX().
    raw = apply_deadband(-right_x, DEADBAND)
    mag = apply_curve(abs(raw)) if raw != 0.0 else 0.0
    return math.copysign(mag, raw) * omega_max


def wrap(a: float, b: float) -> float:
    return (b - a + math.pi) % (2 * math.pi) - math.pi


def decode(type_str: str, payload: bytes):
    if type_str == "boolean":
        return payload[0] != 0 if payload else False
    if type_str == "double":
        return struct.unpack_from("<d", payload)[0]
    if type_str == "string":
        return payload.decode("utf-8", "replace")
    if type_str == "struct:Rotation2d":
        return struct.unpack_from("<d", payload)[0]
    if type_str == "struct:Pose2d":
        return struct.unpack_from("<ddd", payload)
    if type_str == "float[]":
        n = len(payload) // 4
        return list(struct.unpack_from(f"<{n}f", payload))
    return None


def read_uint_le(buf: bytes, offset: int, n: int) -> tuple[int, int]:
    val = 0
    for i in range(n):
        val |= buf[offset + i] << (8 * i)
    return val, offset + n


def parse_log(path: Path) -> dict[str, list]:
    data = path.read_bytes()
    if data[:6] != b"WPILOG":
        raise SystemExit(f"not a wpilog: {path}")
    extra_len = struct.unpack_from("<I", data, 8)[0]
    i = 12 + extra_len
    n = len(data)
    fields: dict[int, tuple[str, str]] = {}
    want_name = set(KEYS.values())
    series = {k: [] for k in KEYS.values()}
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
        if entry_id == 0:
            if not payload or payload[0] != 0 or len(payload) < 9:
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
            if name in want_name:
                fields[eid] = (name, type_str)
            continue
        spec = fields.get(entry_id)
        if spec is None:
            continue
        name, type_str = spec
        val = decode(type_str, payload)
        if val is not None:
            series[name].append((ts, val))
    return series


def ff(series: list, t: int, idx: list[int]):
    while idx[0] + 1 < len(series) and series[idx[0] + 1][0] <= t:
        idx[0] += 1
    if not series or series[idx[0]][0] > t:
        return None
    return series[idx[0]][1]


def reconstruct(series: dict, omega_max: float) -> dict:
    yaw_s = series[KEYS["yaw"]]
    idx = {k: [0] for k in KEYS}
    t, yaw, theta_set, omega_stick, x, y = [], [], [], [], [], []
    enabled_f, auto_f, aim_f = [], [], []
    hold = None
    prev_t = None
    for ts, yaw_now in yaw_s:
        sec = ts / 1e6
        dt = sec - prev_t if prev_t is not None else 0.02
        if dt <= 0 or dt > 0.2:
            dt = 0.02
        prev_t = sec
        enabled = bool(ff(series[KEYS["enabled"]], ts, idx["enabled"]))
        auto = bool(ff(series[KEYS["auto"]], ts, idx["auto"]))
        joy = ff(series[KEYS["joy"]], ts, idx["joy"])
        pose = ff(series[KEYS["pose"]], ts, idx["pose"])
        state = ff(series[KEYS["state"]], ts, idx["state"])
        rx = joy[3] if joy and len(joy) > 3 else 0.0
        w = stick_to_omega(rx, omega_max)
        aiming = (state or "") in AIM
        if enabled and not auto and hold is None:
            hold = yaw_now
        if not enabled:
            hold = None
        elif not auto:
            if abs(w) > 1e-6 and not aiming:
                hold = (hold if hold is not None else yaw_now) + w * dt
            elif hold is None:
                hold = yaw_now
        t.append(sec)
        yaw.append(yaw_now)
        theta_set.append(hold if hold is not None else yaw_now)
        omega_stick.append(w)
        if pose is not None:
            x.append(pose[0])
            y.append(pose[1])
        else:
            x.append(float("nan"))
            y.append(float("nan"))
        enabled_f.append(enabled)
        auto_f.append(auto)
        aim_f.append(aiming)
    return {
        "t": t,
        "yaw": yaw,
        "theta_set": theta_set,
        "omega_stick": omega_stick,
        "x": x,
        "y": y,
        "enabled": enabled_f,
        "auto": auto_f,
        "aim": aim_f,
    }


def unwrap(values: list[float]) -> list[float]:
    out = [values[0]]
    for v in values[1:]:
        out.append(out[-1] + wrap(out[-1], v))
    return out


def plot(rec: dict, title: str, out: Path | None) -> None:
    import matplotlib.pyplot as plt
    import numpy as np

    t = np.array(rec["t"])
    yaw_u = np.degrees(unwrap(rec["yaw"]))
    set_u = np.degrees(unwrap(rec["theta_set"]))
    tele = [i for i, (e, a) in enumerate(zip(rec["enabled"], rec["auto"])) if e and not a]
    if not tele:
        raise SystemExit("no teleop in this log")
    i0 = tele[0]
    yaw_u = yaw_u - yaw_u[i0]
    set_u = set_u - set_u[i0]
    err = set_u - yaw_u

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), height_ratios=[1.2, 1.0])
    ax = axes[0]
    ax.plot(t, yaw_u, color="#4c78a8", lw=1.4, label="actual heading (rate control)")
    ax.plot(t, set_u, color="#f58518", lw=1.4, label="stick-integrated heading hold")
    ax.set_xlim(t[tele[0]], t[tele[-1]])
    y_lo = min(yaw_u[tele].min(), set_u[tele].min()) - 20
    y_hi = max(yaw_u[tele].max(), set_u[tele].max()) + 20
    ax.set_ylim(y_lo, y_hi)
    # shade auto / disabled / aim
    def shade(mask, color, label):
        on = False
        start = t[0]
        first = True
        for i, flag in enumerate(mask):
            if flag and not on:
                start = t[i]
                on = True
            elif not flag and on:
                ax.axvspan(start, t[i], color=color, alpha=0.12, label=label if first else None)
                first = False
                on = False
        if on:
            ax.axvspan(start, t[-1], color=color, alpha=0.12, label=label if first else None)

    shade(rec["auto"], "#54a24b", "auto")
    shade(rec["aim"], "#b279a2", "aim (RT/RB)")
    ax.set_ylabel("heading (deg, unwrapped)")
    ax.set_title(title)
    ax.legend(loc="upper left", frameon=False)
    ax.grid(True, alpha=0.3)

    ax2 = ax.twinx()
    ax2.plot(t, err, color="#e45756", lw=0.8, alpha=0.55, label="hold − actual")
    ax2.set_ylabel("error (deg)")
    ax2.set_xlim(t[tele[0]], t[tele[-1]])
    ax2.legend(loc="upper right", frameon=False)

    fx = axes[1]
    xs = np.array(rec["x"])
    ys = np.array(rec["y"])
    tele_m = np.zeros(len(t), dtype=bool)
    tele_m[tele] = True
    good = np.isfinite(xs) & np.isfinite(ys) & tele_m
    fx.plot(xs[good], ys[good], color="#8c8c8c", lw=0.8, alpha=0.7, label="odometry path")
    # arrows along teleop, subsampled
    step = max(1, len(tele) // 40)
    idxs = tele[::step]
    yaw = np.array(rec["yaw"])
    th = np.array(rec["theta_set"])
    scale = 0.45
    fx.quiver(
        xs[idxs],
        ys[idxs],
        np.cos(yaw[idxs]) * scale,
        np.sin(yaw[idxs]) * scale,
        color="#4c78a8",
        angles="xy",
        scale_units="xy",
        scale=1,
        width=0.004,
        label="actual nose",
    )
    fx.quiver(
        xs[idxs],
        ys[idxs],
        np.cos(th[idxs]) * scale,
        np.sin(th[idxs]) * scale,
        color="#f58518",
        angles="xy",
        scale_units="xy",
        scale=1,
        width=0.004,
        label="hold nose",
    )
    fx.set_aspect("equal")
    fx.set_xlabel("field x (m)")
    fx.set_ylabel("field y (m)")
    fx.legend(loc="best", frameon=False)
    fx.grid(True, alpha=0.3)
    fig.tight_layout()
    if out:
        fig.savefig(out, dpi=140)
        print(f"wrote {out}")
    else:
        plt.show()


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("wpilog", type=Path)
    p.add_argument("--omega-max", type=float, default=OMEGA_MAX)
    p.add_argument("--out", type=Path, default=None, help="save PNG instead of showing")
    args = p.parse_args()
    if not args.wpilog.exists():
        raise SystemExit(f"missing {args.wpilog}")
    series = parse_log(args.wpilog)
    rec = reconstruct(series, args.omega_max)
    plot(rec, args.wpilog.name, args.out)


if __name__ == "__main__":
    main()
