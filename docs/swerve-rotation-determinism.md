# Swerve rotation: deterministic feel

Rate control alone is not deterministic in the way a driver cares about. Rotate stick sets `ω`. Stick at 0 means “keep whatever heading you have.” Module lag, a floor dip, or a slip becomes a heading walk.

Drivers do not memorize “I held 30% rotate for 0.8 s, so I should be at 47°.” They memorize “this deflection feels like this rate” and “when I let go, I stay pointed.” Teleop therefore has three jobs, not one ghost pose.

Translation stays **velocity**. An XY integral is a cursor that yanks back after a slip or a wall.

---

## Teleop stack (what `joystickDrive` does)

| Layer | When | What |
|---|---|---|
| **E — steer velocity FF** | Every `runVelocity` | Unprofiled azimuth position plus `dθ/dt` so Slot0 `kV` is real |
| **B — chassis rate loop** | Rotate stick live | `ω = ω_stick + kRate (ω_stick − ω_gyro)` |
| **A — hold on release** | Rotate stick in deadband | Snapshot heading, PD hold. No integral of stick |

`TeleopRotation` is the A+B chokepoint. Gyro missing → stick passthrough, no hold. `joystickDriveAtAngle` (A button snap to 0°) is unchanged: that is a motion-profiled end heading, not hold-on-release.

A tap of rotate stick accepts a new heading. There is no FAIL latch: hold is “the heading I stopped on,” not a lead the driver cannot see.

### Rejected for live teleop: ghost heading

```
θ_set += ω_stick * dt
ω_out = PID(θ_set − θ_now)
```

The driver is already a position controller looking at the real robot. A ghost they cannot see leads when the plant lags, then lunges on release. Gyro-gate / FAIL / reverse-reset were patches for that. Keep the integral as a **log measurement of intent** (`scripts/overlay_heading_hold.py`), not as the live controller.

---

## Motion-profile rotation (snaps and paths)

Pick an end heading (or a path timestamp). Generate `θ_set(t)`, `ω_set(t)`, and usually `α_set(t)`. PID corrects vs that plan.

| Layer | Profile? | Notes |
|---|---|---|
| Module steer | Not on the request | Unprofiled `Position` plus velocity FF. Motion Magic Expo is configured and never selected. |
| Chassis heading snap | Yes | `joystickDriveAtAngle` (`ANGLE_MAX_VELOCITY` / `ANGLE_MAX_ACCELERATION`) |
| Teleop drive | No | Rate loop while turning, hold on release |
| Path follow | Not in this repo | PathPlanner/Choreo already *is* a timed profile |

**Steer: profile vs velocity FF (do not mix them up)**

- **Motion Magic Expo** — onboard exponential profile to a *discrete* angle. Fast 0° → 90° snaps. Weak when the heading target keeps moving.
- **Position + velocity FF** — still unprofiled position, but `PositionVoltage.withVelocity(dθ/dt)` makes Slot0 `kV` real. A ≥90° optimize flip is not a slew; FF stays 0.

---

## How to choose

| Situation | Use |
|---|---|
| Snap to an angle, path timestamp, known end heading | Motion profile (`joystickDriveAtAngle` / path) |
| Driver streaming a rotate rate | Rate loop (B) |
| Driver holding “straight” | Hold on release (A) |
| Module heading changing every cycle | Position + steer velocity FF (E) |
| Module snap 0° → 90° while parked | Motion Magic Expo is allowed |
| Same stick history → same field heading | Log overlay only. Not the live loop. |
| Want XY to “stay put” after a slip | Not yet. Add-up translation fights the wall. |

---

## Status in this repo

- Hardware / Tuner X: trusted. Concrete campaign constants: `ConcreteMk4TunerConstants`, selected by `RobotIdentity.ACTIVE = CONCRETE_MK4`.
- Steer request: unprofiled position + velocity FF. Drive `kA` unused (no accel on `VelocityVoltage`).
- Teleop: `TeleopRotation` in `joystickDrive` (rate loop + hold on release). Heading snap still profiled PID.
- PathPlanner / setpoint generator: **not in this repo**.
- Measure onboard (encoders, gyro, PDH). AprilTags are a later world-truth overlay.
