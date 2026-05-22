# ekf_localization_pkg

ROS 2 localization stack for the AUV using `robot_localization`. Provides a
two-tier EKF pipeline (local dead-reckoning + GPS-anchored global), a set of
supporting nodes, and a thruster-based DVL fallback for extended DVL outages.

---

## Architecture

```
IMU ──[imu_yaw_correction]──┐
DVL ──────────────────────────┤──> ekf_local (30 Hz) ──> /odometry/filtered/local
Pressure ─────────────────────┘         │
                                 [odometry_validator]
                                         │
GPS ──[datum_watchdog]──> navsat_transform ──┤
                                         │
                              ekf_global (10 Hz) ──> /odometry/filtered/global

TF:  map → odom → base_link

Servo PWM ──[thruster_velocity_estimator]  (silent when DVL present,
                │                           activates on DVL dropout)
                └──────────────────────────> /sensors/thruster/odometry_cov
                                                  └──> fused into ekf_local
```

### Local EKF — always running, 30 Hz

Fuses IMU + DVL + pressure. Produces the `odom → base_link` TF and
`/odometry/filtered/local`. Runs from boot with no GPS dependency.

| Sensor | Topic | States fused |
|--------|-------|--------------|
| IMU (XSens MTi 600) | `/imu/data` | Orientation (roll, pitch, yaw), angular velocity |
| DVL (WaterLinked A50) | `/sensors/dvl/odometry_cov` | Linear velocity (vx, vy, vz) |
| Pressure sensor | `/sensors/pressure/pose_enu` | Z-position (depth) |
| Thruster fallback *(optional)* | `/sensors/thruster/odometry_cov` | Linear velocity — only when DVL absent |

### Global EKF — activated on first quality GNSS fix, 10 Hz

Fuses local EKF output + GPS + pressure (in map frame). Produces the
`map → odom` TF and `/odometry/filtered/global`. The node starts at T=0
in dormant mode; `gnss_datum_watchdog` activates it via SetParameters
once h_acc ≤ 0.5 m, then a one-shot `set_pose` bootstrap initialises
state at the datum.

| Sensor | Topic | States fused |
|--------|-------|--------------|
| Local EKF (validated) | `/odometry/filtered/local_validated` | Orientation, velocity |
| GPS (direct UTM projection) | `/odometry/gps_map` | X, Y position (ENU) |
| Pressure (map-frame relabelled) | `/sensors/pressure/pose_enu_map` | Z position |

---

## Nodes

### `ekf_local_node` / `ekf_global_node`

Standard `robot_localization::ekf_node` instances. Config files:
`config/ekf_local.yaml`, `config/ekf_global.yaml`.

Key tuning decision — `sensor_timeout: 999999`: the EKF only advances on
real measurements and never falls back to predict-to-wall-clock, preventing
multi-day `dt` during rosbag replay clock gaps.

---

### `gnss_datum_watchdog`

Holds the dormant global EKF stack inert until a quality GNSS fix is
confirmed (so null-island 0°,0° can never corrupt the map frame). On
first quality lock it pushes the datum (and `local_anchor_z`) to the
four dormant global-stack nodes via SetParameters, then schedules a
one-shot `set_pose` to bootstrap `ekf_global_node` directly into the
correct state. After lock, every incoming fix is range/haversine-
validated and republished on `/gps/validated`.

**Quality gate:** `status >= 0`, `|lat| > 0.1°`, `h_acc ≤ h_acc_max_m` (requires
UBX-NAV-HPPOSLLH).

| Parameter | Default | Description |
|-----------|---------|-------------|
| `fix_topic` | `/gps/selected` | Raw NavSatFix input |
| `h_acc_topic` | `/ubx_nav_hp_pos_llh` | UBX accuracy source (empty = disable gate) |
| `h_acc_max_m` | `0.50` | Maximum acceptable horizontal accuracy |
| `max_fix_distance_m` | `100000` | Reject fixes > this distance from datum |
| `use_global_ekf` | `true` | Also launch `ekf_global_node` |

---

### `odometry_validator`

Forwards `/odometry/filtered/local` → `/odometry/filtered/local_validated`
while dropping messages whose `header.stamp` jumps forward by more than
`max_forward_jump_s`. Guards the global EKF against wall-clock stamp glitches
that occur during offline rosbag replay when `/clock` is briefly unavailable,
which would otherwise produce `dt ≈ 3 days` and blow up the predict step.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_forward_jump_s` | `60.0` | Reject forward stamp jumps beyond this |
| `max_backward_jump_s` | `1.0` | Reject backward stamp jumps beyond this |

---

### `imu_yaw_correction`

Pre-EKF heading correction. Rotates `/imu/data` orientation about +Z by
`yaw_offset_deg` and republishes on `/imu/data_corrected`. Used by the
anchored launch to calibrate IMU heading against GNSS bearing via a
service-triggered forward-drive maneuver.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `yaw_offset_deg` | `0.0` | CCW yaw correction in degrees |
| `input_topic` | `/imu/data` | Raw IMU input |
| `output_topic` | `/imu/data_corrected` | Corrected IMU output |

**Service:** `~/calibrate_yaw_offset` (`std_srvs/Trigger`) — samples IMU yaw
and GNSS heading over 10 s to auto-compute the offset.

---

### `gnss_anchored_pose`

Simpler alternative to the global EKF. Takes a single GPS fix as a fixed
translation offset from the local EKF origin to the map frame. No ongoing
Kalman fusion of GPS — dead-reckoning only after the anchor is set.

Use when: GPS is available for a short time only, or when global EKF tuning
is not yet complete.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `reanchor_on_each_fix` | `false` | Update offset on every valid fix (causes jumps) |
| `gps_antenna_offset_xyz` | `[0,0,0]` | Lever-arm correction for antenna position |

---

### `global_ekf_to_navsatfix`

Inverse UTM projection of `/odometry/filtered/global` back to lat/lon,
published as `/gps/filtered/global` (NavSatFix). Used for Foxglove map
visualisation and logging.

---

### `thruster_velocity_estimator`

**DVL fallback (static model).** Estimates body-frame surge velocity from the
Pixhawk servo PWM command using fixed parameters (`k=0.780`, `b=0.964`) fitted
on Zermatt 2026-04-29 data. Publishes on `/sensors/thruster/odometry_cov` only
when DVL has been absent for `dvl_timeout_s` (default 3 s). The local EKF fuses
this as `odom1` — see [DVL Fallback](#dvl-fallback-thruster-velocity-model).

For a version that learns `k` and `b` online from DVL data, see
[adaptive_thruster_estimator](../adaptive_thruster_estimator/README.md).

---

### `navsatfix_marker`

Debug utility. Publishes a single NavSatFix point with live-tunable lat/lon/alt
parameters for Foxglove Map panel landmark pins.

---

## Launch Files

| File | Use case |
|------|----------|
| `ekf_localization.launch.py` | **Online** — local + in-place global EKF via GNSS datum watchdog |
| `ekf_anchored.launch.py` | **Online** — local EKF + single-fix anchor (no global EKF) |
| `offline_ekf_replay.launch.py` | Rosbag replay of online stack (`use_sim_time=true`) |
| `offline_anchored_replay.launch.py` | Rosbag replay of anchored stack |
| `ekf_global.launch.py` | Standalone global EKF for debugging |

Common arguments (both online launches):

```bash
ros2 launch ekf_localization_pkg ekf_localization.launch.py \
  use_thruster_fallback:=true \       # enable DVL fallback node (default true)
  use_gnss_datum_watchdog:=true \     # enable GPS / global EKF stack (default true)
  gps_fix_topic:=/fix \
  h_acc_topic:=/ubx_nav_hp_pos_llh
```

---

## DVL Fallback: Thruster Velocity Model

### Background

When DVL bottom-lock is lost (shallow water, air bubbles, steep terrain), the
local EKF has no velocity input and falls back to IMU-only dead-reckoning.
Free-acceleration integration drifts at ~0.01–0.05 m/s² due to residual
accelerometer bias — even with the XSens gravity compensation active.

### Model

A power-law model was fitted on the Zermatt 2026-04-29 rectangle survey
recordings (`zermatt_rectangle_04` and `zermatt_rectangle_06`, 1177 s total,
zero measurable current):

```
u   = (servo_ch0_pwm − 1500) / 500          # normalised surge command ∈ [−1, 1]
vx  = 0.780 · sign(u) · |u|^0.964  m/s     # surge velocity (base_link frame)
vy  = 0.0  m/s                              # depth-hold survey; vy RMS < 0.05 m/s
vz  = 0.0  m/s                              # depth-hold mode;   vz RMS < 0.03 m/s
```

The exponent b ≈ 1 means the relationship is nearly linear across the survey
speed range — consistent with viscous-dominant drag at low Reynolds number.
At half-throttle (`u = 0.5`) the model predicts 0.40 m/s; at 80% throttle
(`u = 0.8`) it predicts 0.63 m/s.

### Simulation Results (EKF-level validation, 18 dropout scenarios, both bags)

Results from a faithful replay of the `ekf_local.yaml` Kalman filter (state
`[vx, vy, ax, ay]`, Q from yaml, DVL measurement noise from bag covariances)
with injected DVL dropouts on moving-vehicle intervals (`|vx| > 0.05 m/s`).
Position error is integrated from velocity state error over the dropout window.

| Dropout | No-fallback pos (p50) | Thrust model pos (p50) | Improvement |
|---------|-----------------------|------------------------|-------------|
| 10 s | 0.93 m | **0.53 m** | 1.7× |
| 30 s | 3.26 m | **1.07 m** | 3.1× |
| 60 s | 7.90 m | **1.66 m** | 4.8× |
| 120 s | 17.9 m | **4.08 m** | 4.4× |

EKF velocity uncertainty P[vx] at 120 s dropout:
- No fallback: **6.1 × 10⁻²** (m/s)² — unbounded covariance growth
- Thrust model: **3.0 × 10⁻⁴** (m/s)² — **202× smaller**; EKF stays conditioned

Representative time-series at vx = −0.316 m/s, 60 s dropout:
```
  t=0s  (DVL active):    both methods — vel_err 0.000 m/s
  t=30s (mid-dropout):   no-fallback vel_err 0.148 m/s / 3.17 m
                         thrust model vel_err 0.015 m/s / 0.91 m
  t=60s (dropout end):   no-fallback vel_err 0.299 m/s / 8.84 m
                         thrust model vel_err 0.019 m/s / 2.78 m
  t=75s (DVL restored):  both — vel_err 0.000 m/s, pos_err 11.69 m
                         ← position offset from dropout does NOT recover
```

Note: position error accumulated during a DVL outage is **not corrected** when
DVL returns — only velocity state is updated by the DVL measurement. The global
EKF (GPS via navsat_transform) will eventually correct the map-frame position.

### EKF Covariance

The node publishes with twist covariance:

```
cov_vx = 0.0057 (m/s)²    # fitted residual σ² from Zermatt data
cov_vy = 0.0027 (m/s)²    # assume-zero uncertainty
cov_vz = 0.0009 (m/s)²    # assume-zero uncertainty (depth hold)
```

DVL covariance diagonal is ~1 × 10⁻⁸ (m/s)². The ratio of ~600 000× ensures
the EKF ignores the thrust model completely the moment DVL recovers — no manual
switching or parameter changes needed.

### Integration

The node is included unconditionally in `ekf_localization.launch.py` and
`ekf_anchored.launch.py`. The local EKF config (`ekf_local.yaml`) has `odom1`
wired to `/sensors/thruster/odometry_cov`. The node is silent until DVL has
been absent for `dvl_timeout_s` (default 3 s), so it has zero effect during
normal operation.

### Limitations

- Model is valid for zero or near-zero water current. In significant current
  the velocity-over-ground estimate will be off by the current magnitude.
- Only surge (forward/backward) is modelled. Lateral velocity during turns is
  not captured; the model outputs `vy = 0` during those periods.
- The servo topic is published at ~2 Hz by MAVLink. Between updates the node
  holds the last known command (zero-order hold).
- Validity is conditional on depth-hold being active (vz ≈ 0). If the AUV is
  ascending or descending under thruster control during a DVL dropout, the
  vz = 0 assumption will introduce depth error.

---

## Sensor Fusion Summary

```
State vector (robot_localization, 15 elements):
  [x, y, z, roll, pitch, yaw, vx, vy, vz, droll, dpitch, dyaw, ax, ay, az]

Local EKF inputs:
  IMU            → roll, pitch, yaw, droll, dpitch, dyaw        (always)
  DVL            → vx, vy, vz                                   (always, ~9 Hz)
  Pressure       → z                                            (always, ~50 Hz)
  Thruster model → vx, vy, vz  (high covariance)               (DVL absent only)

Global EKF inputs:
  Local EKF      → roll, pitch, yaw, vx, vy, vz                (always)
  navsat_transform → x, y  (GPS ENU)                           (GPS present)
```
