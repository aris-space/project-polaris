# Simulation merge log

Append-only. Records every place where a VERIFY-FIRST check in `docs/SIM_MERGE_PLAN.md`
contradicted the plan, and what was done instead. Newest entries at the bottom.

## 2026-09-27 — T0.1

- Branch is `autonomy/sim` (not `simulation` as the plan names it), cut from
  `origin/autonomy/main` @ `45aad65`. Decision by Noel.
- `CLAUDE.local.md` states `gnss_datum_watchdog` was removed from the active launch. On
  `autonomy/main` it is still wired in `ekf_localization.launch.py` (`use_navsat_transform`).
  Per invariant 2, `autonomy/main` is treated as truth.
- `scripts/cmd_vel_ramp.py` exists at the repo root on `autonomy/main` (T1.2) — not vendored again.

## 2026-09-27 — T0.2

- Sim-repo `orca_nav2/src` differs from `autonomy/main` exactly as the plan states
  (`max_dist` vs `max_xy_dist`/`max_z_dist`; no `enable_velocity_divergence_emergency`;
  `prev_vel_` reset in `setPlan`). No source changes made.
- Found, out of scope (invariant 6), not fixed: `transform_pose()` in
  `pure_pursuit_controller_3d.cpp` carries a comment saying it uses `TimePointZero`, but the
  code still passes `in_pose.header.stamp` to `lookupTransform`. Comment and code disagree;
  needs a decision by whoever owns the controller.
- The three salvaged docs contained a "live tuning" block copied from the sim repo's
  `nav2_params.yaml`. Removed / relabelled as historical rather than updated, so no tuning
  values are duplicated outside `nav2_params.yaml` (invariant 5).
