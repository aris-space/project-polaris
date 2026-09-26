> Describes `src/autonomy/orca_nav2` as of `45aad65b` (`autonomy/main`). Salvaged from the simulation repo and corrected against this tree.

# Legacy vs. Regulated PurePursuitController3D — Comparison

This document compares the **current** implementation in
[pure_pursuit_controller_3d.cpp](src/pure_pursuit_controller_3d.cpp) (Regulated
Pure Pursuit, RPP) against the **legacy** Orca4 implementation by Clyde McQueen
(textbook pure pursuit with a goal-behind rotation branch).

For an in-depth feature description of the current controller, see
[PURE_PURSUIT_CONTROLLER.md](PURE_PURSUIT_CONTROLLER.md).

> **TL;DR** — The current implementation is dramatically more robust on
> sharp turns and during the final approach, and adds a hardware safety net
> the old version lacks. The cost is more parameters to tune and a longer
> warm-up because rotate-to-heading replaces a "drive and curve" startup.

---

## At a glance

| Capability | Legacy | Current (RPP) |
|---|:---:|:---:|
| Pure pursuit lookahead (single, static) | ✅ | ✅ |
| **Velocity-scaled** main lookahead | ❌ | ✅ (toggle) |
| **Independent curvature lookahead** | ❌ | ✅ (toggle) |
| Acceleration limiting per axis | ✅ | ✅ |
| Sqrt deceleration to carrot | ✅ | ✅ |
| **Soft `K · dist` deceleration cap** | ❌ | ✅ |
| Goal-behind rotation (if `x_base < 0`) | ✅ | ❌ (replaced) |
| **Rotate-to-heading on large heading error** | ❌ | ✅ |
| **Hysteresis on rotate-to-path** | ❌ | ✅ |
| **Stateful goal-heading latch** | ❌ | ✅ |
| **Curvature-regulated linear velocity** | partial (hard cap) | ✅ (smooth) |
| **Approach velocity scaling** | ❌ | ✅ |
| **Cross-track velocity damping (PD)** | ❌ | ✅ (toggle) |
| **Velocity-divergence emergency cutoff** | ❌ | ✅ (toggle) |
| Tracking-error telemetry (6 topics) | ✅ | ✅ |
| Polyline projection for closest point | ✅ | ✅ |
| Pixhawk arm/mode publishers | ❌ | ✅ (emergency only) |
| `setPlan()` resets hysteresis/latch flags (not `prev_vel_`) | ❌ | ✅ |

Legend: ✅ present, ❌ absent.

---

## What the *legacy* controller did

The legacy version is short and easy to reason about:

1. **`find_goal()`** — walk the plan, lock onto the closest pose, then return
   the first plan pose past `lookahead_dist`. (Same algorithm survives in the
   new version.)
2. **Decision tree in `pure_pursuit_3d()`**:
   ```
   if z_dist > goal_tolerance:        z_vel with sqrt decel
   if xy_dist > goal_tolerance:
       if goal_f_base.x > 0:          # goal ahead
           κ = 2·y / dist²
           if |κ|·x_vel_upper ≤ yaw_vel_lower:
               linear.x = x_vel ; angular.z = κ · x_vel    # constant speed
           else:
               linear.x = yaw_vel / |κ| ; angular.z = ±yaw_vel   # tight-curve cap
           sqrt-decel linear.x against xy_dist
       else:                          # goal behind sub
           rotate in place toward goal (constant ±yaw_vel)
   ```
3. **Acceleration limiting** — three `Limiter` instances clamp `dv` per tick.
4. **Sqrt deceleration** — `v_decel = sqrt(2·a·dist)` against the carrot's
   remaining distance, applied to `linear.x` and `linear.z`.
5. **Tracking telemetry** — same six topics.

`x_error` and `yaw_error` exist as multipliers used inside the curvature
decision (`x_vel_upper`, `yaw_vel_lower`) to model motion-model uncertainty.

---

## What the *current* controller adds

### 1. Two lookahead points instead of one

Legacy uses one carrot for everything. The current version splits into a
**main carrot** (drives forward speed and the goal-proximity check) and a
**curvature carrot** at a separately tunable, fixed distance (drives `κ` and
the rotate-to-path trigger). When `use_velocity_scaled_lookahead: true`, only
the main lookahead scales; curvature stays decoupled — breaking the
slow-speed → short-lookahead → high-κ → slower-still feedback loop.

**Benefit** — stable behaviour at low speed and on tight turns.
**Drawback** — two `find_goal()` walks per tick (still O(N), but doubled).

### 2. Rotate-to-heading replaces "goal behind sub" rotation

The legacy controller only rotated in place when the goal was *behind* the sub
(`x_base < 0`). Otherwise, no matter how large the heading error, it tried to
drive forward and curve. On sharp inter-segment turns this produced wide,
overshooting arcs.

The current version checks heading error to the **curvature carrot** and
rotates in place whenever it exceeds `rotate_to_heading_min_angle` (entry) and
stays in rotation until it drops below `rotate_to_heading_release_angle`
(exit). The rotation itself uses sqrt deceleration with `rotate_to_heading_decel`
(real braking capability, separate from the command ramp rate `yaw_accel`).

**Benefit** — clean cornering, no arc-overshoot on sharp turns.
**Drawback** — at the *start* of a path with a moderate heading error, the
sub now rotates first instead of driving and curving. Net mission time on
gentle paths can be slightly longer.

The legacy "goal behind" case is subsumed: a goal behind the sub means
`|angle_to_curv| > π/2`, which always triggers rotate-to-path.

### 3. Hysteresis on rotate-to-path

Single-threshold rotate-to-heading exits as soon as the error dips below the
trigger. The residual error then drives cross-track drift the moment the
controller switches to forward tracking. The current version uses two
thresholds; the release angle must satisfy
`release_angle < v² / (2 · decel)` so the rotation finishes inside the
deceleration window.

**Benefit** — no chatter at the rotate↔track boundary, no cross-track drift on
release.
**Drawback** — one more parameter to set correctly; if the inequality is
violated, rotation can release too early or never release.

### 4. Rotate→track handoff zeroes `prev_vel_`

When releasing rotate-to-path, the previous tick's `angular.z` is still in
`prev_vel_`. With small `yaw_accel · dt`, the limiter would carry that
residual into the tracking phase and cause overshoot/sway. The current
version zeroes `prev_vel_.angular.z` and `prev_vel_.linear.x` exactly on the
edge.

**Benefit** — clean curvature-driven yaw rate ramps from 0.
**Drawback** — none functionally; one edge-case branch in the code.

### 5. Stateful goal-heading

Legacy: once XY is within `goal_tolerance`, it just stops. If currents push
it back out, it starts pursuing again — potentially cycling.

Current: when XY tolerance is first reached, `has_reached_xy_tolerance_` latches
to `true` and the controller switches to "rotate to final plan orientation"
mode. It stays in that mode until the goal checker accepts the pose.

**Benefit** — final yaw alignment finishes deterministically; no oscillation
near the goal.
**Drawback** — toggleable via `stateful: false` if you don't want the latch.

### 6. Smooth curvature-regulated velocity vs. legacy hard cap

The legacy controller did have *one* form of curvature regulation: when
`|κ| · x_vel_upper > yaw_vel_lower`, it switched to a "tight curve" branch
that clamped `linear.x = yaw_vel / |κ|`. This is a hard threshold — sub
suddenly slows once a yaw-rate ceiling is hit.

The current version scales smoothly:
```
v_scaled = v · min(1, R / R_min)        where R = 1/|κ|
v        = max(v_scaled, regulated_linear_scaling_min_speed)
```
The floor (`min_speed`) prevents creeping to a halt mid-turn.

**Benefit** — gradual, predictable slowdown into curves; no step changes.
**Drawback** — two extra parameters (`min_radius`, `min_speed`).

### 7. Approach velocity scaling

Legacy relied solely on `Limiter::decelerate`'s sqrt profile against the
carrot. The carrot is `lookahead_dist` ahead, so until the *carrot* is close
to the goal, `linear.x` stays at full `x_vel`. With AUV momentum and water
inertia, this often led to overshoot at the goal.

The current version stacks an explicit linear ramp:
`v · path_remaining / approach_velocity_scaling_dist`, floored at
`min_approach_linear_velocity`, applied alongside the existing sqrt decel.

**Benefit** — gentle, controllable touchdown.
**Drawback** — final metre is slower; mission time penalty proportional to
`approach_velocity_scaling_dist`.

### 8. Soft `K · dist` cap inside `Limiter::decelerate`

Legacy `decelerate` returns `min(|v|, sqrt(2·a·d))`. Current version returns
`min(|v|, K · d, sqrt(2·a·d))` — a third, linear-in-distance cap with
gain `K_descelerate`.

**Benefit** — extra-conservative slowdown over long approaches; physically the
sqrt term dominates close in but the linear term takes over far out.
**Drawback** — extra parameter; if `K` is set too small, the sub crawls.

### 9. Cross-track velocity damping (optional PD term)

Pure pursuit reacts to cross-track *position* indirectly (through the carrot).
The current version adds an optional derivative term on lateral velocity:
```
v_cross   = vel.x·sin(yaw_err) + vel.y·cos(yaw_err)
angular.z -= K_cross_vel · v_cross
```
**Benefit** — penalises drift before it grows into displacement.
**Drawback** — extra gain to tune; default off (`K_cross_vel: 0`).

### 10. Velocity-divergence emergency cutoff

The current version watches the angle between the velocity vector and the
path tangent. If the AUV is moving faster than `min_speed_divergence_check`
and the divergence exceeds `max_velocity_divergence_rad` (and the master switch
`enable_velocity_divergence_emergency` is true), it disarms via
`/pixhawk/arm_cmd`, switches to MANUAL via `/pixhawk/mode_cmd`, and throws.

**Benefit** — hardware safety net for runaway-controller scenarios; the
legacy code had no equivalent.
**Drawback** — must be tuned to actual mission dynamics. Set too tight and
it false-fires on currents/disturbances; set too loose and it never fires.
Plugin default is `max_velocity_divergence_rad: 0.0` (off); the shipped
`nav2_params.yaml` sets a threshold but disables the trip with
`enable_velocity_divergence_emergency: false`.

### 11. `setPlan()` resets latched state — but keeps `prev_vel_`

Legacy `setPlan()` just stores the plan. Current version also resets
`has_reached_xy_tolerance_`, `is_rotating_to_path_` and `was_rotating_to_path_`.

It deliberately does **not** reset `prev_vel_`: zeroing it would make the
acceleration limiter clamp the next command from 0, so surge would ramp from
scratch on every replan (the BT replans at 2 Hz whenever `IsStraightPathValid`
fails). Start-from-rest is handled by `deactivate()`, which does zero it.

**Benefit** — re-planning mid-mission no longer carries stale branch state into
the new plan, and does not stutter the velocity.
**Drawback** — none.

---

## What the legacy version did *better*

A balanced view — these are real costs of the new controller:

- **Simplicity.** ~280 lines vs. ~800. The legacy version is a textbook pure
  pursuit and a junior engineer can read it end-to-end in 10 minutes.
- **Fewer parameters.** ~13 params vs. ~30. Easier to tune from scratch on
  a new hull; harder to misconfigure into a deadlock (e.g. the
  `release_angle < v²/(2·decel)` constraint has no analogue).
- **Faster startup on gentle paths.** Legacy starts driving immediately and
  curves toward the lookahead. Current rotates first whenever the heading
  error exceeds `rotate_to_heading_min_angle`, which adds a few seconds.
- **No emergency-disarm risk.** Current's velocity-divergence cutoff *can*
  disarm the vehicle. If thresholds are misconfigured, it can fire during
  legitimate disturbances.
- **No state to debug.** Legacy is stateless beyond `prev_vel_` and the
  plan. Current carries five extra latched/edge flags that affect the next
  tick's branch.

---

## When does the new behaviour matter most?

The largest improvements appear on:

- **Sharp inter-segment turns** (e.g. corners of the rectangular survey path
  flown by [WGS84_mission_starter.py](../autonomy_bringup_pkg/autonomy_bringup_pkg/WGS84_mission_starter.py)).
  Legacy: wide arc, overshoot, cross-track spike. Current: in-place rotate at
  the corner, then resume tracking with `prev_vel_` zeroed.
- **Final approach to goal.** Legacy: full speed until the carrot's sqrt
  decel kicks in, often with overshoot from momentum. Current: stacked
  approach scaling + sqrt decel, then stateful rotate-to-final-yaw.
- **Slow-speed manoeuvres.** Legacy: short carrot-to-AUV distance gives
  noisy κ. Current: fixed curvature lookahead + smooth velocity regulation.
- **Drift recovery.** Legacy: pursues the carrot, no direct lateral-velocity
  feedback. Current (when `K_cross_vel > 0`): damps lateral velocity
  directly.

---

## Migration notes

If you switch a previously-tuned legacy mission over to the current
controller without touching parameters, expect:

1. **Slower starts** — set `rotate_to_heading_min_angle` higher (e.g.
   `1.22 ≈ 70°`, or the higher value now in `nav2_params.yaml`) to keep the legacy behaviour on
   gentle paths.
2. **Slower final approach** — set `approach_velocity_scaling_dist` small
   (e.g. 0.5) or set `min_approach_linear_velocity` close to `x_vel`.
3. **Possible deadlock at the goal** if `release_angle ≥ v²/(2·decel)`.
   Either reduce `release_angle` or raise `rotate_to_heading_decel`.
4. **Spurious emergency disarms** if the divergence trip is enabled
   without realistic mission tuning. Keep
   `enable_velocity_divergence_emergency: false` until basic tracking is
   confirmed clean.

The shipped [nav2_params.yaml](../autonomy_bringup_pkg/params/nav2_params.yaml)
already encodes these trade-offs — start there, not from the plugin defaults.
