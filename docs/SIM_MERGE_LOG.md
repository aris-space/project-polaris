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

## 2026-09-27 — T0.3

- VERIFY-FIRST: only three `mavlink_connection` call sites use `MAVLINK_ROUTER_TCP` /
  `JETSON_IP_ADDRESS` (receiver `self.port`, receiver `self._gcs_port`, publisher `self.port`).
- **Plan gap:** `_gcs_port` also defaults to `tcp:127.0.0.1:5760`. SITL accepts one client per
  serial port, so with only `MAVLINK_RECEIVER_URL`/`MAVLINK_PUBLISHER_URL` set (as T2.3 lists),
  the GCS connection would collide with the publisher on 5760. T2.3 therefore also sets
  `MAVLINK_GCS_URL=tcp:127.0.0.1:5763` (SITL SERIAL2).
- The retry loop lives in a new `mavlink_bridge/connection.py` (one helper instead of three
  copies). With no env vars it makes exactly one `mavutil.mavlink_connection(<default>)` call and
  re-raises, i.e. hardware behaviour is unchanged. Note pymavlink 2.4.49 `mavtcp` already retries
  6× internally; the outer loop is for SITL starting late.
- Hardware sign-off (bench check per `CHECK_AXIS.md`, heartbeat + arm/disarm) still OUTSTANDING.
