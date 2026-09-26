> Describes `src/autonomy/orca_nav2` as of `45aad65b` (`autonomy/main`). Salvaged from the simulation repo and corrected against this tree.

# PurePursuitController3D — Regulated Pure Pursuit for 3D AUVs

A Nav2 controller plugin (`orca_nav2/PurePursuitController3D`) that adapts
[Regulated Pure Pursuit (RPP)](https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller)
to a 3D under-water vehicle. Source: [pure_pursuit_controller_3d.cpp](src/pure_pursuit_controller_3d.cpp).

The controller produces a 3-DoF differential-drive command (`linear.x`,
`linear.z`, `angular.z`) — `linear.y` is intentionally never commanded because
lateral drag on the hull is much higher than longitudinal drag.

> **TL;DR** — XY tracking uses Regulated Pure Pursuit with hysteresis,
> rotate-to-heading, curvature-regulated speed, approach slow-down, and a
> velocity-divergence emergency cutoff. Depth (`linear.z`) is controlled
> independently with a sqrt-deceleration profile.

---

## Architecture overview

```
                    ┌─────────────────────────────────────────┐
   global plan ───► │  setPlan()                              │
                    │  resets hysteresis + stateful flags     │
                    │  (prev_vel_ is NOT reset — see §13)     │
                    └─────────────────────────────────────────┘
                                       │
                                       ▼
   pose, twist  ───► computeVelocityCommands()
                          │
                          ├─► tracking_error_from_plan()  ──► /pure_pursuit_*
                          │   (cross-track XY, vertical, yaw, closest point)
                          │
                          ├─► pure_pursuit_3d()
                          │     │
                          │     ├─ depth   (linear.z) ── sqrt decel
                          │     │
                          │     └─ XY      (linear.x + angular.z)
                          │           ├─ shouldRotateToGoalHeading? → rotateToHeading
                          │           ├─ shouldRotateToPath?       → rotateToHeading
                          │           └─ tracking: x_vel → applyConstraints → curvature·v
                          │
                          ├─► cross-track velocity damping (optional PD term)
                          ├─► acceleration limiting (Limiter)
                          └─► velocity-divergence emergency cutoff
```

The control loop is split into three independent axes that are then merged:

| Axis | Source | Behaviour |
|---|---|---|
| `linear.z` | depth error to lookahead pose | `z_vel` with sqrt deceleration on remaining depth |
| `linear.x` | regulated pure pursuit | scaled down by curvature + approach distance |
| `angular.z` | curvature carrot or rotate-to-heading | `v · κ`, replaced by rotate-in-place when heading error large |

---

## Features

### 1. Two independent lookahead points

A signature change from vanilla pure pursuit. Two carrots are picked from the
plan on every tick:

- **Main carrot** (`lookahead_dist`, optionally velocity-scaled) — drives the
  forward velocity decision and the goal-proximity check.
- **Curvature carrot** (`curvature_lookahead_dist`, fixed) — drives the
  curvature κ used for `angular.z = v · κ` and the rotate-to-path trigger.

**Why two?** With a single velocity-scaled lookahead, a slow speed produces a
short lookahead, which produces a high curvature, which forces the regulator
to slow down further — a feedback loop that can stall the vehicle. The fixed
curvature lookahead breaks this loop. Toggle with `use_fixed_curvature_lookahead`.

### 2. Rotate-to-heading with hysteresis

When the heading error to the curvature carrot exceeds
`rotate_to_heading_min_angle`, the controller stops translating (`linear.x = 0`)
and rotates in place using a **sqrt deceleration profile**:

```
ω_cmd = sign(Δθ) · min( rotate_to_heading_angular_vel ,
                        sqrt(2 · rotate_to_heading_decel · |Δθ|) )
```

Two important design points:

- **Hysteresis** — entry threshold `rotate_to_heading_min_angle`, exit
  threshold `rotate_to_heading_release_angle` (smaller). Without this,
  the controller releases rotation as soon as the error dips below the entry
  threshold, then drifts cross-track on the residual error. The release
  threshold must satisfy `release_angle < v² / (2 · decel)` so the rotation
  finishes inside the deceleration window. See
  [pure_pursuit_controller_3d.cpp:365-375](src/pure_pursuit_controller_3d.cpp#L365-L375).
- **Separate braking parameter** — `rotate_to_heading_decel` is the *real*
  AUV angular braking capability (rad/s²), distinct from `yaw_accel` (the
  command ramp rate). Water drag means the actual braking is much weaker
  than the ramp rate. Lower values = brake earlier = less overshoot.

### 3. Rotate-to-path → tracking handoff

When releasing rotate-to-path, the previous tick's `angular.z` is still in
`prev_vel_`. With small `yaw_accel · dt` the limiter would carry that residual
into the tracking phase and cause overshoot/sway. The controller zeroes
`prev_vel_.angular.z` and `prev_vel_.linear.x` exactly on the rotate→track
edge so the curvature-driven yaw rate ramps from 0. See
[pure_pursuit_controller_3d.cpp:532-539](src/pure_pursuit_controller_3d.cpp#L532-L539).

### 4. Stateful goal-heading alignment

Once the AUV enters `goal_tolerance` in XY, `has_reached_xy_tolerance_`
latches to `true`. From then on, the controller stays in "rotate to final
plan orientation" mode even if currents push it slightly back out of
tolerance. Disable by setting `stateful: false`.

### 5. Curvature-regulated linear velocity

When `use_regulated_linear_vel_scaling: true`, forward speed is scaled down
proportionally as the turning radius shrinks below
`regulated_linear_scaling_min_radius`:

```
v_scaled = v · min(1, R / R_min)
v       = max(v_scaled, regulated_linear_scaling_min_speed)
```

The floor (`min_speed`) prevents the AUV from creeping to a halt mid-turn.

### 6. Approach velocity scaling

In the last `approach_velocity_scaling_dist` metres of the plan, `linear.x`
is linearly scaled by `path_remaining / approach_velocity_scaling_dist`,
floored at `min_approach_linear_velocity`. This is **independent** of the
sqrt deceleration applied by the `Limiter`; both are stacked.

### 7. Optional velocity-scaled lookahead

When `use_velocity_scaled_lookahead: true`, the **main** carrot is recomputed
as `clamp(|speed| · lookahead_time, [min_lookahead_dist, max_lookahead_dist])`.
The **curvature** carrot is unaffected — it is always either the static
`lookahead_dist` or `curvature_lookahead_dist`.

### 8. Cross-track velocity damping (optional PD term)

When `K_cross_vel > 0`, a derivative term is added to `angular.z` based on the
component of measured velocity perpendicular to the path tangent:

```
v_cross   = vel.x·sin(yaw_err) + vel.y·cos(yaw_err)
angular.z -= K_cross_vel · v_cross
```

This penalises lateral drift directly rather than waiting for cross-track
displacement to develop. Disabled by default (`K_cross_vel: 0.0`).

### 9. Velocity-divergence emergency cutoff

A safety net that watches the angle between the velocity vector and the path
tangent. If `enable_velocity_divergence_emergency` is true, the AUV is moving
faster than `min_speed_divergence_check`, and the divergence exceeds
`max_velocity_divergence_rad`, the controller will:

1. Publish `False` on `/pixhawk/arm_cmd` → disarm.
2. Publish `MANUAL` on `/pixhawk/mode_cmd` → manual mode.
3. Throw `std::runtime_error` → aborts the Nav2 action.

Disable with `enable_velocity_divergence_emergency: false` (master switch — this
is how it is shipped in `nav2_params.yaml`) or `max_velocity_divergence_rad: 0.0`.
The mechanism is symmetric: it fires both when the AUV moves *into* the path and
*away* from it. See
[pure_pursuit_controller_3d.cpp:750-779](src/pure_pursuit_controller_3d.cpp#L750-L779).

### 10. Acceleration limiting per-axis

Three `Limiter` instances (one each for `linear.x`, `linear.z`, `angular.z`)
clamp the per-tick step in velocity to `max_a · dt`. Each limiter also offers
a sqrt deceleration profile used for the depth axis and inside
`rotateToHeading`.

Note: nav2's controller server delivers a 2D twist, so `linear.z` history is
maintained on our own copy of `prev_vel_` rather than the velocity passed in.

### 11. Tracking-error telemetry

When `publish_tracking_error: true` (the default), the controller publishes
six topics every tick:

| Topic | Type | Meaning |
|---|---|---|
| `pure_pursuit_cross_track_xy` | `std_msgs/Float64` | signed XY cross-track error (m), + = left of path |
| `pure_pursuit_vertical_error` | `std_msgs/Float64` | depth error vs. closest point (m) |
| `pure_pursuit_yaw_error` | `std_msgs/Float64` | shortest yaw error vs. path tangent (rad) |
| `pure_pursuit_closest_point_map` | `geometry_msgs/PointStamped` | closest point on plan polyline (map frame) |
| `pure_pursuit_robot_pose_map` | `geometry_msgs/PoseStamped` | robot pose transformed into the plan frame |
| `pure_pursuit_robot_twist` | `geometry_msgs/TwistStamped` | the twist passed in by the controller server |

The closest point is computed as a true polyline projection (per-segment
clamped parameter `t ∈ [0,1]`), not as the closest discrete plan pose.

### 12. Independent depth control

`linear.z` is computed independently of the XY tracking logic. It uses
`z_vel` with the `Limiter`'s sqrt-decel profile against the remaining depth
to the main carrot, and is gated by `goal_tolerance`. Because buoyancy and
drag dominate vertical motion, no curvature/approach regulation is needed.

Note: on the vehicle, vertical control is currently disabled downstream — the
`mavlink_bridge` sends `vz = 0` regardless of `linear.z` (see
`docs/SIM_MERGE_PLAN.md` Appendix C.3). The controller still computes it.

### 13. `prev_vel_` persists across replans

`setPlan()` resets the hysteresis and stateful flags but deliberately does
**not** zero `prev_vel_`. Zeroing it would force the acceleration limiter to
clamp the next command from 0, so surge would ramp from scratch on every
replan (the BT replans at 2 Hz whenever `IsStraightPathValid` fails).
Start-from-rest is handled by `deactivate()`, which does zero `prev_vel_`.
See [pure_pursuit_controller_3d.cpp:787-800](src/pure_pursuit_controller_3d.cpp#L787-L800).

---

## Parameters

### Core kinematics

| Parameter | Default | Notes |
|---|---|---|
| `x_vel` | 0.4 | Desired forward velocity (m/s) |
| `x_accel` | 0.4 | Max longitudinal acceleration (m/s²) |
| `z_vel` | 0.2 | Desired vertical velocity (m/s) |
| `z_accel` | 0.2 | Max vertical acceleration (m/s²) |
| `yaw_vel` | 0.4 | Desired yaw rate (rad/s) — used elsewhere; rotate-to-heading uses its own cap |
| `yaw_accel` | 0.4 | Max yaw command ramp rate (rad/s²) — *not* the braking capability |
| `tick_rate` | 20.0 | Controller frequency (Hz), used to convert `accel` → `dv/tick` |
| `K_descelerate` | 0.2 | Soft `v = K · dist` cap inside `Limiter::decelerate` |

### Pure pursuit lookahead

| Parameter | Default | Notes |
|---|---|---|
| `lookahead_dist` | 1.0 | Static main lookahead (m), used when velocity scaling disabled |
| `use_velocity_scaled_lookahead` | false | If true, main lookahead = `speed · lookahead_time` clamped |
| `min_lookahead_dist` | 1.0 | Lower clamp for velocity-scaled lookahead (m) |
| `max_lookahead_dist` | 5.0 | Upper clamp for velocity-scaled lookahead (m) |
| `lookahead_time` | 1.5 | Time-horizon for velocity scaling (s) |
| `use_fixed_curvature_lookahead` | true | If true, use a separate fixed-distance carrot for curvature |
| `curvature_lookahead_dist` | 3.0 | Distance for curvature carrot (m) |

### Rotate-to-heading

| Parameter | Default | Notes |
|---|---|---|
| `use_rotate_to_heading` | true | Master enable for rotate-in-place behaviour |
| `rotate_to_heading_angular_vel` | 0.3 | Max in-place rotation speed (rad/s) |
| `rotate_to_heading_min_angle` | π/4 (0.785) | Entry threshold for rotate-to-path (rad) |
| `rotate_to_heading_release_angle` | 0.2 | Exit threshold (rad), must be smaller — hysteresis |
| `rotate_to_heading_decel` | 0.02 | Real angular braking capability (rad/s²) for sqrt decel |
| `stateful` | true | Latch goal-heading mode once XY tolerance reached |

### Regulated velocity

| Parameter | Default | Notes |
|---|---|---|
| `use_regulated_linear_vel_scaling` | true | Scale forward speed for tight curvature |
| `regulated_linear_scaling_min_radius` | 0.9 | Below this turning radius (m), `v` is scaled |
| `regulated_linear_scaling_min_speed` | 0.05 | Floor speed during sharp turns (m/s) |
| `min_approach_linear_velocity` | 0.05 | Floor speed in final approach (m/s) |
| `approach_velocity_scaling_dist` | 1.0 | Distance over which to ramp down to the floor (m) |

### Tolerances and topics

| Parameter | Default | Notes |
|---|---|---|
| `goal_tolerance` | 0.1 | XY/Z stop-tracking threshold (m) — should be < `GoalChecker3D` tolerances |
| `transform_tolerance` | 1.0 | Max age of a TF used in lookups (s) |
| `publish_tracking_error` | true | Toggle the six telemetry publishers |

### Cross-track damping & emergency cutoff

| Parameter | Default | Notes |
|---|---|---|
| `K_cross_vel` | 0.0 | Damping gain on lateral velocity (rad/s per m/s); 0 = disabled |
| `enable_velocity_divergence_emergency` | true | Master enable for the emergency cutoff |
| `max_velocity_divergence_rad` | 0.0 | Emergency angle threshold (rad); 0 = disabled |
| `min_speed_divergence_check` | 0.02 | Don't fire emergency below this speed (m/s) |

### Live tuning values

The shipped values differ from the plugin defaults above. They are **not**
repeated here, so they cannot go stale: the single source of truth is the
`LongStraightLine` block in
[autonomy_bringup_pkg/params/nav2_params.yaml](../autonomy_bringup_pkg/params/nav2_params.yaml),
used by hardware and simulation alike.

---

## Lifecycle / outputs

- Subscribes (via Nav2): pose, twist, plan.
- Publishes:
  - `/pixhawk/arm_cmd` (`std_msgs/Bool`) — only on emergency disarm.
  - `/pixhawk/mode_cmd` (`std_msgs/String`) — only on emergency mode change.
  - The six `pure_pursuit_*` telemetry topics listed above.
- Throws `std::runtime_error` on velocity-divergence emergency.
- Throws `nav2_core::PlannerException` if `setPlan()` is called with an empty path.
- `setSpeedLimit()` is **not** supported (logs an error if invoked).

---

## Known limitations

- Costmaps are ignored — this controller is for open water, not obstacle avoidance.
- `linear.y` is never commanded (intentional — see hull drag note above).
- The Nav2 velocity smoother is bypassed (it is not 3D-aware).
- `yaw_vel` is read from params but the rotate-to-heading path uses its own
  `rotate_to_heading_angular_vel`. Make sure both are set if you tune one.
