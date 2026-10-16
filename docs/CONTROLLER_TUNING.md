# ArduSub Controller Tuning — Surge Velocity & Yaw Rate

This document covers the ArduPilot parameters that directly affect velocity-X
(surge) and yaw-rate tracking when commanding the sub in GUIDED mode via the
mavlink bridge.

> **Pre-condition:** before tuning, verify `AHRS_ORIENTATION = 6`
> (ROTATION_YAW_270) matches the physical Pixhawk mounting (arrow 90° CCW from
> vehicle forward). A 180° AHRS error will make surge and sway appear inverted
> and PID tuning meaningless.

---

## 1 — Surge velocity (X axis)

The bridge sends `vx` in `MAV_FRAME_BODY_FRD`. ArduSub passes it through the
**Position and Speed Controller (PSC)** velocity loop before computing motor
outputs.

### Parameters

| Parameter | Current | Recommended | Effect |
|---|---:|---:|---|
| `PSC_VELXY_P` | 6.0 | 5.0 | Proportional gain on velocity error. Lower → less aggressive correction, more lag. Higher → faster response, risk of oscillation. |
| `PSC_VELXY_I` | 0.5 | 0.5 | Integral gain. Removes steady-state error at cruise. Winds up during long runs → the source of post-stop drift. |
| `PSC_VELXY_D` | 0.0 | **0.8** | Derivative gain. **Most impactful single change.** Damps integrator wind-up oscillation. Currently 0 on real vehicle; SITL has 0.8. |
| `PSC_VELXY_FF` | 0.2 | 0.0 | Feedforward. Adds a bias proportional to the target. At 0.2 it introduces unnecessary thrust before any error exists. |
| `PSC_VELXY_IMAX` | 1000 | **50** | Integrator saturation limit (cm/s²). At 1000 it is effectively unbounded — integrator can accumulate for seconds. Cap to 50 to bound wind-up and wind-down equally. |
| `PSC_POSXY_P` | 1.0 | 2.5 | Outer position loop gain. Only active when the watchdog sends a position-hold setpoint (x=y=z=0 in BODY_OFFSET_NED). Increase to tighten hold. |
| `PSC_VELXY_FILT_HZ` | — | 5.0 | Low-pass on velocity error fed to P/D terms. Lower → smoother but more lag. Raise only if sensor noise is low. |

### What each term does in practice

```
accel_cmd = P·(v_target − v_actual)
           + I·∫(v_target − v_actual) dt      ← wind-up source
           + D·d(v_target − v_actual)/dt      ← wind-down damper
           + FF·v_target                      ← bias, bypass error loop
```

- **Lag after stop** → `PSC_VELXY_IMAX` too high or `PSC_VELXY_D = 0`
- **Overshoot / backward drift** → same root cause, I winds up, D can't damp it
- **Oscillation during cruise** → `PSC_VELXY_P` too high
- **Slow ramp-up** → `PSC_VELXY_P` too low or `PSC_VELXY_FF = 0`

### Tuning procedure

1. Apply `PSC_VELXY_IMAX = 50` and `PSC_VELXY_D = 0.8` first — these are
   parameter-only, no reflash, and have the largest effect on the lag symptom.
2. Run `./scripts/cmd_vel_ramp.py ramp surge 0.20 --ramp 5 --hold 5` and record
   a bag (`/pixhawk/cmd_vel` + `/odometry/filtered/local`).
3. Plot `twist.linear.x` vs the commanded value. Target: tracks within ±0.05 m/s
   at steady state, no backward overshoot after stop.
4. If oscillation appears at steady state → reduce `PSC_VELXY_P` by 0.5 steps.
5. If tracking is sluggish → increase `PSC_VELXY_P` or add `PSC_VELXY_FF = 0.1`.
6. If post-stop drift persists → further reduce `PSC_VELXY_IMAX` (try 30).

---

## 2 — Yaw rate

The bridge sends `yaw_rate` (rad/s). ArduSub passes it through the **Attitude
Controller (ATC)** rate loop.

> **Note:** yaw direction is currently inverted in the firmware mixer
> (MOT_2 and MOT_6 yaw factors have wrong sign). Until the mixer is reflashed,
> the bridge applies `yaw_rate = -msg.angular.z` to compensate. Tuning the
> rate loop is still valid — the sign inversion is downstream of the PID.

### Parameters

| Parameter | Current | Recommended | Effect |
|---|---:|---:|---|
| `ATC_RAT_YAW_P` | 0.08 | **0.18** | Rate loop proportional. At 0.08 the loop barely responds — causes the chatter observed in bags. |
| `ATC_RAT_YAW_I` | 0.0 | **0.018** | Integrates away steady-state yaw error (drag asymmetry, thruster bias). |
| `ATC_RAT_YAW_D` | 0.0 | 0.0 | Keep 0 for yaw — yaw rate sensors are noisy; D amplifies this. Only add if overshoot is severe after raising P/I. |
| `ATC_RAT_YAW_FF` | 0.0 | **0.10** | Feedforward on yaw rate target. Eliminates the "start delay" where the sub pauses before beginning to yaw. |
| `ATC_RAT_YAW_IMAX` | 0.222 | 0.222 | Already correct — leave as-is. |
| `ATC_RAT_YAW_FLTT` | — | 10.0 | Low-pass on the rate target. Smooths step commands. |
| `ATC_RAT_YAW_FLTE` | — | 10.0 | Low-pass on the rate error fed to P/I. |
| `ATC_ANG_YAW_P` | 0.0 | 0.0 | Heading-hold outer loop. Intentionally 0 — pure rate control, no heading reference needed in GUIDED velocity mode. |
| `ATC_SLEW_YAW` | 6000 | **200** | Max yaw acceleration (deg/s²). At 6000 this is a step — causes the chatter spike at onset. 200 is a realistic ramp. |
| `ATC_RATE_Y_MAX` | 180 | **90** | Hard cap on commanded yaw rate (deg/s). 180 is too fast for this hull. |

### What each term does in practice

```
torque_cmd = P·(ω_target − ω_actual)
           + I·∫(ω_target − ω_actual) dt
           + D·d(ω_target − ω_actual)/dt     ← keep 0, sensor noise
           + FF·ω_target
```

- **Chatter / ringing at onset** → `ATC_SLEW_YAW` too high (step input) or `ATC_RAT_YAW_P` too high
- **Sub barely yaws** → `ATC_RAT_YAW_P` too low (0.08 is the current symptom)
- **Yaw never reaches commanded rate** → `ATC_RAT_YAW_I = 0` (no integrator to close SS error)
- **Start delay before yaw begins** → `ATC_RAT_YAW_FF = 0`

### Tuning procedure

1. Apply `ATC_SLEW_YAW = 200` and `ATC_RATE_Y_MAX = 90` first — reduces the
   onset spike regardless of PID values.
2. Set `ATC_RAT_YAW_P = 0.18`, `ATC_RAT_YAW_I = 0.018`, `ATC_RAT_YAW_FF = 0.10`.
3. Run `./scripts/cmd_vel_ramp.py ramp yaw 0.30 --ramp 4 --hold 4` and record
   `/imu/angular_velocity` + `/pixhawk/cmd_vel`.
4. Plot `angular_velocity.z` vs commanded rate. Target: tracks within ±0.05 rad/s,
   no ringing, no >0.1 rad/s overshoot.
5. If ringing persists → reduce `ATC_RAT_YAW_P` in 0.02 steps.
6. If response is sluggish → increase `ATC_RAT_YAW_FF` in 0.05 steps first
   (FF is cleaner than P for this).

---

## 3 — Recommended param block (apply all at once)

Paste into MAVProxy console or load as `.parm` file. Reboot FC after.

```
param set PSC_VELXY_P       5.0
param set PSC_VELXY_D       0.8
param set PSC_VELXY_FF      0.0
param set PSC_VELXY_IMAX    50
param set PSC_POSXY_P       2.5

param set ATC_RAT_YAW_P     0.18
param set ATC_RAT_YAW_I     0.018
param set ATC_RAT_YAW_FF    0.10
param set ATC_SLEW_YAW      200
param set ATC_RATE_Y_MAX    90
param set MOT_SAFE_DISARM   1
```

---

## 4 — Test profiles

All profiles publish to `/pixhawk/cmd_vel` at 20 Hz.

```bash
# Surge — trapezoid, good baseline for lag measurement
./scripts/cmd_vel_ramp.py ramp surge 0.20 --ramp 5 --hold 5

# Surge — bipolar, catches asymmetric thrust (MOT_1 direction check)
./scripts/cmd_vel_ramp.py bipolar surge 0.20 --ramp 3 --hold 2

# Surge — staircase, characterises tracking vs amplitude (finds deadband)
./scripts/cmd_vel_ramp.py staircase surge 0.30 --hold 3

# Yaw rate — trapezoid, primary ATC_RAT_YAW tuning run
./scripts/cmd_vel_ramp.py ramp yaw 0.30 --ramp 4 --hold 4

# Yaw rate — bipolar, exposes direction asymmetry
./scripts/cmd_vel_ramp.py bipolar yaw 0.30 --ramp 3 --hold 2
```

Record with:
```bash
ros2 bag record -s mcap -o ~/Downloads/Rosbags/$(date +%Y%m%d_%H%M%S)_tune \
  /pixhawk/cmd_vel /odometry/filtered/local /imu/angular_velocity \
  /filter/euler /pixhawk/servo_output_raw
```
