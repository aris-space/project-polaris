# adaptive_thruster_estimator

ROS 2 C++ node that identifies the AUV thruster power-law model **online** using
Recursive Least Squares (RLS). During normal DVL operation it continuously
refines `k` and `b` from real velocity data. When DVL bottom-lock is lost it
switches to estimation mode and publishes the predicted surge velocity to keep
the local EKF conditioned — the same role as `thruster_velocity_estimator`, but
with parameters that adapt to changing conditions (wear, biofouling, current).

---

## Algorithm

### Physics model

```
vx = k · sign(u) · |u|^b
```

- `u` — normalised surge command: `(pwm_ch0 − 1500) / 500 ∈ [−1, 1]`
- `vx` — surge velocity (body frame, m/s) from DVL
- `k ≈ 0.780`, `b ≈ 0.964` — fitted on Zermatt 2026-04-29 (used as prior)

### Log linearisation

Taking the log of both sides turns the power law into a linear regression:

```
ln|vx|  =  ln(k)  +  b · ln|u|
  y     =  φᵀ · θ

  y = ln|vx|             (scalar observation)
  φ = [1,  ln|u|]ᵀ      (2×1 regressor)
  θ = [ln(k),  b]ᵀ      (2×1 parameter vector)
```

Sign is stripped before the log and re-applied at estimation time — see
[Sign handling](#sign-handling-for-reverse-thrust).

### RLS with forgetting factor

At each DVL update that passes all gates:

```
K  =  P · φ  /  (λ + φᵀ · P · φ)     Kalman gain          (2×1)
e  =  y  −  φᵀ · θ                    innovation           (scalar)
θ ←  θ  +  K · e                      parameter update
P ←  (1/λ) · (I − K · φᵀ) · P        covariance update
P ←  0.5 · (P + Pᵀ)                   symmetry correction
```

### What the 2×2 P matrix represents

`P` is the covariance of the parameter estimate `θ = [ln(k), b]ᵀ`:

| Entry | Meaning |
|-------|---------|
| `P[0,0]` | Variance of `ln(k)` — uncertainty in the gain |
| `P[1,1]` | Variance of `b` — uncertainty in the exponent |
| `P[0,1]` | Covariance between `ln(k)` and `b` |

**Large P → large gain K → new observations dominate** (fast adaptation).
**Small P → small K → old data is trusted** (slow adaptation).

The `(1/λ)` inflation on every step prevents P from collapsing to zero
permanently, so the filter stays responsive to slow drift. At `λ = 0.995` and
9 Hz DVL the effective memory window is:

```
τ  ≈  1 / ((1 − λ) · f_dvl)  =  1 / (0.005 · 9)  ≈  22 seconds
```

Initialised to `P = 1e4 · I` so the first few seconds of data dominate
and convergence is fast.

### Sign handling for reverse thrust

The log transformation operates on magnitudes only — `ln|u|` and `ln|vx|` —
so reverse and forward thrust enter the exact same linear regression without
special-casing.

Two additional gates ensure only clean thrust data enters the update:
1. `sign(u) == sign(vx)` — excludes deceleration phases where the vehicle is
   still moving forward but the command is already reversed. During those
   transients the drag sign conflicts with the velocity sign and would bias `b`.
2. `|Δvx/Δt| < accel_thresh` — excludes inertial lag (added-mass effects)
   that occur at the start of a thrust command.

At estimation time the sign is restored:
```
v̂x  =  sign(u) · exp(θ[0]) · |u|^θ[1]
```

---

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/sensors/dvl/odometry_cov` | `nav_msgs/Odometry` | Subscribe | DVL velocity + covariance for lock detection and learning |
| `/pixhawk/servo_output_raw` | `std_msgs/Int16MultiArray` | Subscribe | PWM servo commands (channel 0 = surge) |
| `/sensors/thruster/odometry_cov` | `nav_msgs/Odometry` | Publish | Fallback velocity estimate (only when DVL absent) |
| `~/diagnostics` | `std_msgs/Float64MultiArray` | Publish | `[k, b, P00, P11, trace(P), dvl_age_s]` at 1 Hz |

---

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `init_k` | `0.780` | Initial surge gain (m/s) — Zermatt prior |
| `init_b` | `0.964` | Initial exponent — Zermatt prior |
| `lambda` | `0.995` | RLS forgetting factor (≈22 s memory at 9 Hz DVL) |
| `u_min` | `0.10` | Min `\|u\|` to engage learning or estimation |
| `accel_thresh` | `0.05` | Max `\|Δvx/Δt\|` (m/s²) for steady-state gate |
| `dvl_cov_thresh` | `1e-4` | Max `twist.covariance[0]` to declare DVL locked |
| `dvl_timeout_s` | `3.0` | Seconds of DVL silence before fallback activates |
| `fallback_cov_vx` | `0.0057` | Fallback twist covariance `vx` (m/s)² |
| `fallback_cov_vy` | `0.0027` | Fallback twist covariance `vy` (m/s)² |
| `fallback_cov_vz` | `0.0009` | Fallback twist covariance `vz` (m/s)² |
| `surge_channel` | `0` | Index into servo array for surge |
| `pwm_neutral` | `1500` | PWM midpoint (µs) |
| `pwm_range` | `500` | PWM half-range (µs) |

---

## Swapping in as DVL fallback

This node publishes to the same topic as `thruster_velocity_estimator` and is a
drop-in replacement. Only one should run at a time.

In `ekf_localization.launch.py` (or `ekf_anchored.launch.py`), replace:

```python
Node(
    package="ekf_localization_pkg",
    executable="thruster_velocity_estimator",
    name="thruster_velocity_estimator",
    output="screen",
)
```

with:

```python
Node(
    package="adaptive_thruster_estimator",
    executable="adaptive_thruster_estimator",
    name="adaptive_thruster_estimator",
    output="screen",
    parameters=[{
        "init_k": 0.780,
        "init_b": 0.964,
        "lambda": 0.995,
    }],
)
```

The local EKF config (`ekf_local.yaml`) does not need any changes — it already
fuses `odom1: /sensors/thruster/odometry_cov`.

---

## Docker dependency

`libeigen3-dev` is required. It is already installed on the current image
(`3.4.0-2ubuntu2`). If rebuilding from a base ROS image, add to the Dockerfile:

```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev
```

No other new dependencies beyond standard ROS Humble packages.

---

## Verifying it works

### 1 — Node is alive and has sane initial parameters

```bash
ros2 node list | grep adaptive_thruster
# → /adaptive_thruster_estimator

ros2 topic echo /adaptive_thruster_estimator/diagnostics --once
# → data: [k, b, P00, P11, trace(P), dvl_age_s]
# Expected at startup: k≈0.780, b≈0.964, P00≈1e4, dvl_age_s<3.0
```

### 2 — Learning phase: watch P shrink as DVL data arrives

Drive the AUV at a steady surge command and plot `~/diagnostics` in Foxglove
(Raw Messages panel or Time Series). After ~30 s of steady thrust you should
see:

- `P00` and `P11` drop by several orders of magnitude (1e4 → <1e0)
- `k` and `b` drift slightly from the prior and then stabilise
- `trace(P)` falls monotonically while the vehicle is moving

```bash
# Stream diagnostics to terminal
ros2 topic echo /adaptive_thruster_estimator/diagnostics
# data[0]=k  data[1]=b  data[2]=P00  data[3]=P11  data[4]=trace(P)  data[5]=dvl_age
```

If `P` is not shrinking, check that all learning gates are passing:
- DVL locked: `twist.covariance[0]` on `/sensors/dvl/odometry_cov` must be < 1e-4
- Excitation: `|u|` must be > 0.1 (not hovering or drifting)
- Acceleration: acceleration below 0.05 m/s² (steady cruise, not a manoeuvre)
- Sign: command and velocity must have the same sign (not decelerating)

### 3 — Fallback phase: DVL dropout → topic publishes

The fallback topic is **silent** while DVL is active. To verify it publishes
during dropout:

```bash
# Terminal 1 — watch the fallback topic (nothing expected while DVL is live)
ros2 topic hz /sensors/thruster/odometry_cov

# Terminal 2 — simulate DVL dropout by blocking the topic
ros2 run topic_tools drop /sensors/dvl/odometry_cov 1 1   # drop all messages
# After dvl_timeout_s (3 s) you should see ~/diagnostics dvl_age_s > 3
# and /sensors/thruster/odometry_cov start publishing at 10 Hz
```

Alternatively, wait for a natural DVL dropout during a mission and watch the
node log for:
```
[adaptive_thruster_estimator] DVL absent — thruster fallback active (k=0.783 b=0.961)
```

### 4 — Cross-validate fallback velocity against DVL

Run this comparison just before a known DVL dropout (e.g., end of pool, known
shallow-water return). Record both topics at matched throttle settings:

```bash
ros2 bag record /sensors/dvl/odometry_cov \
                /sensors/thruster/odometry_cov \
                /adaptive_thruster_estimator/diagnostics \
                /pixhawk/servo_output_raw
```

In post-processing, compare `dvl.twist.twist.linear.x` with
`thruster.twist.twist.linear.x` at the same command level. Residual should be
< 0.05 m/s in steady-state cruise once P has converged.

### 5 — EKF integration: confirm the EKF uses the fallback

```bash
# Covariance on /odometry/filtered/local should stay bounded during dropout
# (not grow like IMU-only dead-reckoning)
ros2 topic echo /odometry/filtered/local --field pose.covariance[0]
```

If the fallback is being fused, position variance grows slowly (thrust model
error) rather than rapidly (free accelerometer integration).
