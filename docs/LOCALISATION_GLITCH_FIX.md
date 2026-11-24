# Localisation Glitch Fix

Branch: `fix/ekf/see_test_glitch`

> **State on `dev` (2026-11):** Fix 1 and Fix 3 are active. The speed-dependent covariance of
> Fix 2 is **not** active any more: `launch_dvl.launch.py` selects `reported_fom` (Fix 3), and
> `speed_sigma_scale` / `dvl_variant` in that launch file have no effect in that mode.
> Two later changes are summarised under [Later changes](#later-changes-after-this-branch).

## Summary

This branch makes two changes to how DVL velocity reaches the EKF:

1. **Each DVL velocity is fused only once.** The driver also republished the last velocity on
   every dead-reckoning report, stamped up to 1 s in the past, so the EKF fused every
   velocity twice, including bad readings. Fixed by moving the `dvl_a50` driver submodule to
   the version that has the existing `publish_odometry_on_dead_reckoning` switch.
2. **The DVL velocity covariance now grows with speed.** It used to be a constant. It now
   starts at the measured stationary noise:

   ```
   var_i = var_stationary_i + (k · |v|)²     i ∈ {vx, vy, vz},  k = 0.0055
   ```

## Fix 1: Duplicate DVL velocity in the EKF

### Problem

The driver (`src/sensors/dvl_a50/src/dvl_a50_ros2.cpp`) publishes `dvl/odometry` on two kinds
of report from the DVL:

- **Velocity reports:** the twist is updated and the message is stamped with
  `time_of_validity`, which has microsecond resolution.
- **Dead-reckoning reports:** only the pose is updated, but the whole message is published
  again. It still carries the **previous twist**, and it is stamped with:

  ```cpp
  rclcpp::Time(static_cast<uint64_t>(double(res["ts"])) * 1e9)
  ```

  The cast truncates `ts` to whole seconds before converting to nanoseconds.

Both EKFs fuse vx, vy and vz from `/sensors/dvl/odometry_cov`, which is a copy of
`dvl/odometry` with covariance added. So:

- every velocity is fused **twice**, and the second copy is stamped up to **1 s in the past**;
- a single bad DVL reading has double weight, and part of it is applied at the wrong time;
- the odometry stream has duplicate stamps and goes backwards in time.
  `scripts/analyze_dvl_dual_publish_stamps.py` shows this in recordings.

### Cause

`config/dvl_a50.yaml` already sets `publish_odometry_on_dead_reckoning: false` to prevent
this. But the submodule was pinned at `dcde2ac`, and that version of the driver never declares
the parameter, so ROS silently ignored it. The driver commit that adds the parameter
(`d837aeb`) was already on the driver's `master`, two commits ahead of the pin.

### Change

- Submodule `src/sensors/dvl_a50`: `dcde2ac` → `d837aeb`.
- With the existing YAML setting (`false`), a dead-reckoning report now only updates
  `odometry.pose` and does not publish. `dvl/odometry` is published once per velocity report,
  stamped with `time_of_validity`.
- `dvl/dead_reckoning` is still published on its own topic.
- The EKFs fuse only the twist from this topic, so they lose no information.
- The jump also includes driver commit `ce76f41`, which adds a `publish_dead_reckoning_topic`
  parameter. `d837aeb` removes it again, so it has no net effect.

### Still open: whole-second truncation of `ts`

`d837aeb` still truncates `ts`. It now only affects the `dvl/dead_reckoning` stamp, and nothing
in `src/` subscribes to that topic, so it no longer reaches the EKF. The fix needs a commit in
the driver repo (`glebplatonov04/dvl_a50`):

```cpp
dead_reckoning_report.header.stamp =
    rclcpp::Time(static_cast<int64_t>(double(res["ts"]) * 1e9));
```

### Verification status

- **Not built yet.** The driver could not be compiled on the host because its dependencies
  are missing there. It has to be built in the dev container.
- After building, check that:
  - the driver logs `publish_odometry_on_dead_reckoning=false` at startup;
  - `/sensors/dvl/odometry` and `/sensors/dvl/velocity` publish at the same rate, with no
    duplicate or backwards stamps. `scripts/analyze_dvl_timestamp_verification.py` checks
    this.

### Reverting

To turn the duplicate publish back on without moving the submodule, set
`publish_odometry_on_dead_reckoning: true` in `config/dvl_a50.yaml`. To restore the old pin,
run `git -C src/sensors/dvl_a50 checkout dcde2ac`.

## Fix 2: Speed-Dependent DVL Covariance

The DVL twist covariance fed to the EKF (`/sensors/dvl/odometry_cov`) is no longer a
constant. It now starts at the measured stationary noise and grows with speed. When the
vehicle is moving, the EKF trusts the DVL velocity less than when it is stationary. At rest,
nothing changes.

### Background

`dvl_odometry_covariance` (`src/sensors/dvl_a50_pkg/dvl_a50_pkg/odometry_covariance_node.py`)
adds twist covariance to the DVL odometry before `robot_localization` uses it. It had two
models:

- **`stationary_tep`** (previously active): fixed diagonal variances measured from stationary
  lake recordings (`measurement_noise_constants.py`). For vx this is 4.42e-6 m²/s²
  (σ ≈ 2.1 mm/s). The value was the same at every speed.
- **`speed_dependent`** (previously unused): σ = max(σ_min, scale · speed), using the Water
  Linked datasheet values.

### Problems with the old models

1. **`stationary_tep` ignores motion.** Noise measured on a stationary DVL sets the variance
   at every speed. Noise from motion (vibration, attitude changes, scale-factor error) never
   enters the covariance, so the EKF is overconfident in the DVL while moving.
2. **The old `speed_dependent` model barely depended on speed.** σ_min / scale = 1 m/s for both
   variants, so the covariance stayed constant below about 1 m/s, which covers most AUV
   operation.
3. **The old `speed_dependent` minimum disagreed with the measurements.** With the
   `performance` variant the floor was σ = 1 mm/s (1e-6 m²/s²). That is about 4× tighter than
   the measured stationary vx noise and about 8× tighter than vy.
4. **The vz model was a guess.** σ_z = 2·σ_xy was not based on any measurement and ignored the
   existing vz floor.
5. **An unknown `dvl_variant` fell back to `standard`.** That is 100× looser than the default
   `performance`.

### Changes

#### `odometry_covariance_node.py`

- `speed_dependent` now adds the stationary variance and a speed term in quadrature:
  `var_i = var_stationary_i + (speed_sigma_scale · |v|)²`.
  - The stationary part is the same per-axis value `stationary_tep` uses, including the vz
    floor and any `lock_variance_*` overrides. At rest, both models output the same values.
  - The speed term adds to the stationary noise at every speed. Below 1 m/s it is no longer
    hidden by a minimum.
  - `sigma_min` and the 2× z factor have been removed.
- New parameter **`speed_sigma_scale`** (k, dimensionless σ per unit speed). It defaults to the
  datasheet accuracy of the selected `dvl_variant` (performance 0.001, standard 0.0101).
- An unknown `dvl_variant` now falls back to `performance`, matching the default.
- The startup log always prints the stationary lock variances. In `speed_dependent` mode it
  also prints `speed_sigma_scale`.
- The no-lock and stale-lock behaviour is unchanged: variances are set to `no_lock_variance`
  (1e6).

#### `launch/launch_dvl.launch.py`

```python
"twist_linear_covariance_model": "speed_dependent",   # was "stationary_tep"
"dvl_variant": "performance",
"speed_sigma_scale": 0.0055,  # between performance (0.001) and standard (0.0101)
```

### Resulting covariance (k = 0.0055)

| Speed | Speed term | vx variance | vy variance | vz variance |
|---|---|---|---|---|
| 0 m/s | 0 | 4.42e-6 | 7.84e-6 | 1.00e-6 |
| 0.25 m/s | 1.9e-6 | 6.3e-6 | 9.7e-6 | 2.9e-6 |
| 0.5 m/s | 7.6e-6 | 1.2e-5 | 1.5e-5 | 8.6e-6 |
| 1.0 m/s | 3.0e-5 | 3.5e-5 | 3.8e-5 | 3.1e-5 |

All values are in m²/s². Speed is the 3D DVL speed |v|.

### Choice of k

The value 0.0055 is the arithmetic midpoint of the two datasheet accuracies (0.1 % and
1.01 %). It is a placeholder, not a fitted value:

- The datasheet figures are long-term accuracy specs, mostly scale-factor error, not
  per-sample noise.
- The geometric midpoint (about 0.0032) would give a gentler curve. At 1 m/s the vx variance
  would be about 1.4e-5.
- **Next step:** fit k from moving data, for example by comparing DVL velocity against GNSS
  velocity on surface runs. Then set `speed_sigma_scale` in the launch file. No code change is
  needed.

### Limitations

- One scalar speed scales all three axes. Fast forward motion also inflates the vy and vz
  variances.
- Only the diagonal entries are set. Cross-axis covariances remain zero.

### Verification status

- Both files compile, and the node passes flake8. The only flake8 warning is a missing newline
  at end of file that was already there.
- **Not yet verified on a bag or in the water.** It has not been confirmed that this change
  removes the observed localisation glitch. Before merging, replay the glitch recording through
  the EKF and compare the trajectory and the twist covariance in `/sensors/dvl/odometry_cov`
  against `stationary_tep`.

### Reverting

To return to the previous behaviour, set `"twist_linear_covariance_model": "stationary_tep"` in
`launch_dvl.launch.py`. The stationary model has not been changed.

## Fix 3: Outlier Rejection (FOM covariance + beam-count gate + EKF Mahalanobis backstop)

Commit `01f4041b` on this branch. This is the part that actually catches the lake glitch, which
Fix 2 alone cannot: the glitch was a **wrong velocity with a small covariance**, so scaling the
covariance by (reported) speed does not help. The root cause was that only **3 DVL beams** were
valid, and a 3-beam velocity solve has **zero redundancy** (3 equations, 3 unknowns, exact fit),
so a single bad beam produces a wild velocity (~27 m/s) that the sensor still reports as valid
with a low figure-of-merit. Three independent layers now guard against this.

### Layer 1 — `reported_fom` covariance model

`odometry_covariance_node.py` gains a third model, now the active one:

```
cov_i = max( reported_var_i , stationary_floor_i )     i in {vx, vy, vz}
```

- `reported_var` is the diagonal of `Dvl.velocity_covar`, which the driver fills from the Water
  Linked 3x3 covariance, else from `diag(fom^2)`.
- Flooring at the stationary variances means a deceptively small FOM can never make the EKF trust
  the DVL *more* than it would a stationary measurement. When the DVL is healthy the FOM is tiny,
  so the floor dominates and the covariance looks constant — this is expected, not a bug.

### Layer 2 — beam-count gate (`min_good_beams`, default 4)

If `Dvl.num_good_beams < min_good_beams`, the update is treated as **no-lock**
(`no_lock_variance`, 1e6), so the EKF effectively ignores it — regardless of what the FOM or
`beam_velocities_valid` claims. This is the layer that kills the 27 m/s glitch: 3 beams give no
residual to check, so the only safe response is to distrust the solve.

### Layer 3 — EKF Mahalanobis backstop

`ekf_local.yaml` and `ekf_global.yaml` add:

```yaml
odom0_twist_rejection_threshold: 8.0
```

An independent, filter-side check: a twist update whose Mahalanobis distance from the filter
prediction exceeds the threshold is rejected. 8.0 is conservative (3-DOF chi-sq_0.99 => sqrt ~ 3.4);
tune it down later from the NIS of good runs.

### Launch change

`launch/launch_dvl.launch.py` now selects the new model. This **supersedes** the Fix 2 launch
setting documented above:

```python
"twist_linear_covariance_model": "reported_fom",   # was "speed_dependent"
"min_good_beams": 4,
```

`speed_dependent` and `stationary_tep` remain available as parameter values.

### Verification status

Validated offline on pool bag `dvl_fix_01_2026_10_20`:

- All **28** sub-4-beam velocity samples were forced to no-lock (1e6) in
  `/sensors/dvl/odometry_cov`, including **6 "3-beam, valid, ~0 FOM"** samples — the exact glitch
  mechanism. A join of every sub-4-beam sample to its nearest `odometry_cov` output showed **zero
  leaks** in both directions.
- The `reported_fom` model is confirmed live (locked variance rises above the floor on a few
  samples, so the FOM path is active — a `stationary_tep` build would be dead-flat).
- The **Mahalanobis layer was not exercised** (no velocity spike occurred; max speed 0.45 m/s).
  To test it, replay the **lake glitch bag** through the EKF and confirm the twist update is
  rejected and `/odometry/filtered/local` no longer jumps.
- Needs a rebuild on the Jetson: `git submodule update --init` + `colcon build` of `dvl_a50`,
  `dvl_a50_pkg`, `ekf_localization_pkg`.

### Reverting

Set `"twist_linear_covariance_model"` back to `"speed_dependent"` (or `"stationary_tep"`) and/or
`"min_good_beams"` to `3` in `launch_dvl.launch.py`, and remove `odom0_twist_rejection_threshold`
from the two EKF YAMLs. Each layer is independent and can be reverted on its own.

## Later changes (after this branch)

These are not part of the branch above; they are listed so this file matches what runs on `dev`.

- **DVL speed cap and jump gate (PR #29).** `odometry_covariance_node` also marks a sample as
  no-lock when it is implausible on its own, whatever the beam count and FOM say
  (`velocity_jump_gate.py`, values in `launch_dvl.launch.py`): speed above `max_speed` 0.9 m/s,
  or a jump from the last accepted sample above `min(max_velocity_jump + max_acceleration · dt,
  max_velocity_jump_limit)` = `min(0.5 + 2.0 · dt, 1.0)` m/s. After `jump_reaccept_samples` = 5
  consistent rejected samples the gate accepts again. This catches 4-beam glitches (1.95 m/s with
  a small FOM in a June recording) that Layer 2 cannot see.
- **EKF velocity process noise (PR #29, PR #30).** In `ekf_local.yaml` and `ekf_global.yaml` the
  process noise is now 0.005 for vx / vy and 0.05 for vz (was 0.0005 / 0.001 / 0.0001). With the
  old values and the Layer 3 threshold of 8, real heave in lake waves and real acceleration after
  a short DVL gap were rejected as outliers (EKF vertical speed lagging by 1.75 s; velocity
  lockout). The threshold itself is unchanged.
- **Thruster fallback and the beam gate.** `thruster_velocity_estimator` (`ekf_localization_pkg`)
  used to count a DVL sample with 3 beams as "DVL present", while Layer 2 needs 4, so in a
  stretch of 3-beam samples the EKF got neither. Two changes (2026-11-24) close that gap:
  the local EKF no longer fuses the fallback's `vz = 0` (only `vx`, `vy`; `odom1_config` in
  `ekf_local.yaml`), and the fallback now uses the same 4-beam rule (`dvl_min_good_beams`,
  default 4). The replay results are in the comments at those two places.
