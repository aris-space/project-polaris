> Describes `src/autonomy/orca_nav2` as of `45aad65b` (`autonomy/main`). Salvaged from the simulation repo; parameter values quoted are from the original experiment, not current tuning.

# Convergence Behaviour: Two Spawn Cases

How the regulated controller in [pure_pursuit_controller_3d.cpp](src/pure_pursuit_controller_3d.cpp)
fixes the two failure modes the legacy controller shows when the sub starts
*off-axis* relative to a straight-line plan. The plan in both cases runs
along the world **+X axis** (the orange line in the screenshots); the green
trace is the AUV trajectory.

**Historical tuning.** Both cases were recorded in the simulation repo with the
tuning below. That is *not* the current tuning: the single source of truth is
the `LongStraightLine` block in
[autonomy_bringup_pkg/params/nav2_params.yaml](../autonomy_bringup_pkg/params/nav2_params.yaml),
which has since moved on (e.g. a higher rotate-to-heading entry threshold, a
wider release angle, a shorter lookahead). The *mechanisms* explained below are
unchanged in the code; the exact angles and distances quoted are those of the
experiment. The screenshots referred to were not carried over.

```yaml
rotate_to_heading_min_angle: 1.22     # 70° — entry threshold
rotate_to_heading_release_angle: 0.1  # ~5.7° — exit threshold (hysteresis)
rotate_to_heading_decel: 0.05         # rad/s² — real braking capability
curvature_lookahead_dist: 2.5         # fixed-distance carrot for κ
lookahead_dist: 2.5                   # main carrot
regulated_linear_scaling_min_radius: 0.9
approach_velocity_scaling_dist: 1.0
x_vel: 0.4
```

---

## Case 1 — small offset, sub-threshold heading error

```xml
<pose>0 0.6 -0.2  0 0 0.5</pose>     <!-- y=0.6 m, yaw = 0.5 rad ≈ 28.6° -->
```

The sub starts **0.6 m left of the path** with a heading error of **~28.6°**
toward the path.

### Legacy behaviour (top screenshot)

A textbook pure-pursuit oscillation — an **S-curve** that takes several
metres to damp out:

1. Carrot is forward on the path, off to the right of the sub.
2. `κ = 2y_carrot / d²` is large → sub curves hard right.
3. The sub crosses the path at full `x_vel = 0.4 m/s` with leftover lateral
   momentum.
4. Carrot is now on the *left* → curves left.
5. Repeat with decaying amplitude.

The legacy controller has nothing to break this loop:
- No curvature-regulated speed → enters every overshoot at full forward velocity.
- No cross-track velocity feedback → only reacts to lateral *position*, not lateral *velocity*.
- The single static lookahead means the carrot snaps fast enough to keep driving the curl.

### Current behaviour (bottom screenshot)

Heading error 28.6° is **below** `rotate_to_heading_min_angle = 70°`, so
rotate-to-heading does *not* engage. The improvement comes from three
mechanisms working together:

1. **Fixed curvature lookahead (2.5 m).** `κ` is computed from a carrot at
   a fixed forward distance, decoupled from any velocity scaling. The
   curvature seen on entry is moderate, not the tight value a short
   velocity-scaled lookahead would produce.
2. **Curvature-regulated linear velocity.** The first big curve has
   `R = 1/|κ|` smaller than `regulated_linear_scaling_min_radius = 0.9 m`,
   so `linear.x` scales down via
   `v · min(1, R / R_min)` (floored at `regulated_linear_scaling_min_speed`).
   Less forward speed into the corner → less momentum carrying past the
   path → smaller overshoot on the *first* crossing.
3. **Smaller momentum + smaller carrot-side flips → smaller second swing.**
   With less overshoot and the same regulator engaging on the way back,
   the oscillation damps in **one** swing instead of three or four.

The trace in the bottom screenshot shows exactly that: a single, mild
correction onto the line with no visible second oscillation.

> If `K_cross_vel` were enabled (it was `0.0` in this experiment and still is in `nav2_params.yaml`), an
> additional PD term would damp the lateral velocity directly — the case 1
> path would tighten further. The current behaviour is achieved purely by
> regulated-velocity geometry, without that gain turned on.

---

## Case 2 — small offset, *at-threshold* heading error

```xml
<pose>0 0.1 -0.2  0 0 1.22</pose>    <!-- y=0.1 m, yaw = 1.22 rad ≈ 69.9° -->
```

The sub starts **0.1 m left of the path** but pointing **70° off the path
tangent** — almost perpendicular to its desired direction of travel. This is
the worst case for vanilla pure pursuit.

### Legacy behaviour (top screenshot)

A wide **loop-back**: the sub leaves the path region, swings far to the
right, and curves back in.

What the legacy code does, step by step:

1. `goal_f_base.x` is still positive (the carrot is ahead of the sub in
   sub-frame), so the controller takes the *forward-curve* branch — never
   the goal-behind rotate branch.
2. `κ = 2y / d²` is huge (carrot is far to the side in base frame).
3. The "tight curve" ceiling kicks in:
   `linear.x = yaw_vel / |κ|`, `angular.z = ±yaw_vel`. This caps the yaw
   rate but the sub still moves forward at non-zero speed while almost
   sideways — and water drag does the rest. The trajectory bows out before
   the heading catches up.
4. By the time heading is aligned, the sub is metres off-path and has to
   curve back. Result: the loop you see in the screenshot.

The legacy controller's only "rotate in place" mode is when `goal_f_base.x < 0`
(goal *behind* the sub). With a 70° error, the carrot is still in front, so
rotation never engages.

### Current behaviour (bottom screenshot)

Heading error to the curvature carrot is **right at** the entry threshold
`rotate_to_heading_min_angle = 1.22 rad`. The decision tree in
[pure_pursuit_controller_3d.cpp:516-550](src/pure_pursuit_controller_3d.cpp#L516-L549)
takes the rotate-to-path branch:

1. `shouldRotateToPath()` returns `true` → `is_rotating_to_path_ = true`.
2. `rotateToHeading()` sets `linear.x = 0` and rotates in place using sqrt
   deceleration: `ω = sign(Δθ) · min(rotate_to_heading_angular_vel,
   sqrt(2 · rotate_to_heading_decel · |Δθ|))`. With `decel = 0.05`, the
   distance to brake from `ω = 0.2` is `v²/(2·decel) = 0.4 rad ≈ 23°`, so
   the sub starts braking the rotation about 23° before alignment.
3. Hysteresis keeps rotation engaged until error drops below
   `rotate_to_heading_release_angle = 0.1 rad ≈ 5.7°`. The release
   threshold satisfies `release_angle < v²/(2·decel) = 0.4 rad`, so the
   rotation completes inside its braking window — no overshoot, no chatter
   on release.
4. **Rotate→track handoff**: on the first tick after release,
   `was_rotating_to_path_` is `true`, so the controller zeroes
   `prev_vel_.angular.z` and `prev_vel_.linear.x`. The acceleration limiter
   then ramps both from 0 instead of carrying residual rotation velocity
   into the tracking phase.
5. With heading already aligned and only 0.1 m of cross-track left, normal
   pure-pursuit tracking converges in a single, gentle curve.

The bottom screenshot shows the result: a clean S onto the line with no
loop, no overshoot, and effectively no cross-track excursion.

---

## Why 70° as the rotate-to-heading threshold?

> The shipped threshold is now higher than 70° (see `nav2_params.yaml`); the
> reasoning below — stay above typical inter-segment corners — still applies.

`rotate_to_heading_min_angle = 1.22 rad` is deliberately tuned **above 45°**
so that the sub does *not* rotate in place at typical inter-segment turns of
the survey path. Those 45–60° corners are handled by the smooth curvature
regulation in mechanism 2 above — translating while curving is the
physically natural behaviour for an under-actuated AUV when curvature is
moderate.

The threshold then catches the cases where pure pursuit's geometric
assumption breaks down: **heading error so large that curving forward would
require physically impossible lateral acceleration**. Case 2 is exactly that
regime; case 1 is not.

---

## Summary — what each mechanism contributed

| Mechanism | Case 1 fix | Case 2 fix |
|---|:---:|:---:|
| Rotate-to-heading (≥ 70°) | — | ✅ primary |
| Hysteresis on rotate exit | — | ✅ clean release |
| Rotate→track `prev_vel_` zeroing | — | ✅ no kick on resume |
| Fixed curvature lookahead | ✅ stable κ | (rotation phase, n/a) |
| Curvature-regulated linear velocity | ✅ primary | ✅ on resume |
| Approach velocity scaling | (far from goal, n/a) | (far from goal, n/a) |
| Cross-track velocity damping (`K_cross_vel`) | off in shipped config | off in shipped config |
| Acceleration limiter (legacy too) | helps both | helps both |

**Case 1** is fixed by *velocity regulation* alone — same control branch as
the legacy controller, but with smooth curvature-based slowdown that breaks
the overshoot/oscillation cycle.

**Case 2** is fixed by a *different control branch* — rotate-in-place takes
over because the geometric assumptions of forward-curving pure pursuit no
longer hold. The hysteresis + handoff details ensure the transition back to
tracking is clean enough to merge in 0.1 m of cross-track.

Together, the two mechanisms cover the full spread of off-axis spawn
conditions a real mission start can present.
