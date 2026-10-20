# Ice-Approach Waypoint Maneuver

## Context

The mission is: navigate to a waypoint → hold position → do a controlled rise toward the ice ceiling using the upward ultrasonic sensor → once firm contact is detected, hold light but constant upward thrust against the ice (still holding XY) → after a hold period, descend back to navigation depth → continue to the next waypoint.

Investigation of the `autonomy/behaviour_tree` branch found that the pieces for this already mostly exist, just disconnected:

- The mission CSV schema (`lat,lon,alt,up_down,delta_z_m,hold_s`) already has a per-waypoint `up_down`/`delta_z_m`/`hold_s` flag, parsed into a `WaypointMeta` dataclass and explicitly commented `"Phase A - vertical bump metadata"` in `load_wgs84_points_to_waypoints.py` — it was clearly set aside for exactly this kind of maneuver and never wired up. `mission_waypoint_loader.py` computes this metadata and then silently drops it, publishing only the pose array.
- `/ice_touch_detection/touching` (debounced `std_msgs/Bool`, fuses ultrasonic + IMU + pressure) is the ready-made "firm contact" signal — no new sensing logic needed.
- There is no low-level thrust/force topic anywhere in the stack — vertical motion is only ever a velocity setpoint (`geometry_msgs/Twist` on `/pixhawk/cmd_vel`, FLU) through Pixhawk GUIDED mode. "Light firm contact" has to be realized as a small constant commanded upward velocity sent continuously while physically blocked by the ice.
- There is no closed-loop XY station-keeping controller anywhere in this codebase; every existing "hold" (goal-reached, teleop pause) is just zero commanded velocity relying on hydrodynamic drag. This maneuver's XY hold will follow that same precedent for v1.
- Nav2's `waypoint_follower` already has a pluggable per-waypoint hook (`waypoint_task_executor_plugin`, currently `wait_at_waypoint`, a fixed pause) that runs once between waypoints — after `NavigateToPose` succeeds, before the next one is sent. This is architecturally exactly "goal reached → do maneuver → go to next waypoint," so the maneuver belongs there, not inside `orca4_bt.xml`.

Confirmed decisions (not open questions):
- **Scope**: per-waypoint, reusing the existing `up_down` CSV column. Other waypoints are untouched.
- **Architecture**: a new Nav2 `nav2_core::WaypointTaskExecutor` plugin, replacing `wait_at_waypoint`, built in `orca_nav2` alongside the existing custom plugins (`goal_checker_3d.cpp`, `progress_checker_3d.cpp`, etc.) — no changes to `orca4_bt.xml`.
- **XY hold**: zero-velocity drag hold for v1 (matches existing precedent everywhere else in the codebase).
- **Plain-waypoint pause**: preserved — the new plugin folds in a `plain_waypoint_pause_ms` (default 1250 ms) so non-`up_down` waypoints behave exactly as missions do today.
- **Lost contact mid-hold**: one bounded re-approach attempt; if contact isn't regained, log a warning and proceed to descend — doesn't hard-fail the whole waypoint over a transient blip.

## Implementation

### 1. Publish waypoint metadata (`mission_waypoint_loader.py`)

File: `src/autonomy/autonomy_bringup_pkg/autonomy_bringup_pkg/mission_waypoint_loader.py`

- Swap the import from `process_coordinates` to `process_mission` (`load_wgs84_points_to_waypoints.py:140`), which already returns `(List[PoseStamped], List[WaypointMeta])` — no changes needed in `load_wgs84_points_to_waypoints.py` itself.
- Add a `meta_output_topic` parameter (default `/mission_waypoints_meta`) and a second latched publisher using the same `latched_qos` (TRANSIENT_LOCAL + RELIABLE, matching `/mission_waypoints_enu` exactly, so a plugin subscribing after this loader has already published still gets it).
- In `_on_datum()`, after building `arr`, build a `std_msgs/Float32MultiArray` with `layout.dim = [MultiArrayDimension(label='waypoint', size=len(meta_list), stride=3)]` and `data = [up_down_as_0_or_1, delta_z_m, hold_s] * N` (flattened, index-aligned 1:1 with `arr.poses`). Publish it on the new topic right after `self._pub.publish(arr)`.
- Rationale for `Float32MultiArray` over a new typed message: this metadata has exactly one internal consumer (the new plugin), the encoding is a fixed stride-3 layout that's trivial to decode, and it avoids adding a new message package/dependency for 3 scalars. `custom_msgs` (used elsewhere for raw sensor payloads like `Profile.msg`) was considered and rejected as overkill for this.
- Index alignment is guaranteed downstream: `WGS84_mission_starter.py` forwards `PoseArray.poses` to `FollowWaypoints.Goal.poses` in the same order with no reordering/filtering, and Nav2's `waypoint_follower` calls `processAtWaypoint(pose, curr_waypoint_index)` with `curr_waypoint_index` being the 0-based index into that same list — so the plugin can index its cached metadata array directly by `curr_waypoint_index`.

### 2. New plugin: `IceApproachTaskExecutor`

New file: `src/autonomy/orca_nav2/src/ice_approach_task_executor.cpp` (single self-contained `.cpp`, no header — matches the existing convention of every other `nav2_core` plugin in this package, e.g. `goal_checker_3d.cpp`, `progress_checker_3d.cpp`).

New plugin description file: `src/autonomy/orca_nav2/ice_approach_task_executor_plugin.xml`:
```xml
<library path="ice_approach_task_executor">
  <class name="orca_nav2/IceApproachTaskExecutor" type="orca_nav2::IceApproachTaskExecutor" base_class_type="nav2_core::WaypointTaskExecutor">
    <description>Rise to ice ceiling, hold firm contact, descend, per-waypoint via up_down flag.</description>
  </class>
</library>
```

**Class interface** (`nav2_core::WaypointTaskExecutor`): `initialize(parent, plugin_name)` + `processAtWaypoint(curr_pose, curr_waypoint_index) -> bool`.

**`initialize()`**: reads all parameters below (numeric ones via the existing `PARAMETER()` macro from `orca_nav2/param_macro.hpp`, string topic names declared directly); creates subscriptions to `touching_topic`, `ultrasonic_topic`, `odom_topic`, `mode_topic` (standard QoS depth 10) and to `meta_topic` (**TRANSIENT_LOCAL**, matching the loader's publisher — a QoS mismatch here would silently drop the latched metadata); creates the `cmd_vel_pub_` publisher.

**Threading note (the one genuinely novel piece here):** `processAtWaypoint()` runs synchronously on `waypoint_follower`'s own dedicated callback-group executor thread (same mechanism that already lets `wait_at_waypoint`'s blocking sleep work today without freezing the rest of Nav2) — blocking there for up to a few minutes is safe and expected. But the plugin's own subscriptions (`touching_`, `current_z_`, etc.) must be served by a *separate* callback group with its own spinning executor, otherwise the blocking `while` loops inside `processAtWaypoint()` would deadlock waiting on callbacks that never get spun. Concretely: `initialize()` creates its own `rclcpp::executors::SingleThreadedExecutor`, adds only its own subscriptions' callback group, and spins it on a dedicated background `std::thread` for the plugin's lifetime (joined/stopped in the destructor). Build and bench-test this piece in isolation first (see Verification, Phase 2) before relying on it for the full state machine.

**`processAtWaypoint()` state machine** — `curr_pose.pose.position.z` at entry is the pre-approach navigation depth to return to:
1. Look up metadata by `curr_waypoint_index`. If missing/out of range or `up_down==false` → sleep `plain_waypoint_pause_ms` and return `true` (preserves today's plain-waypoint behavior exactly).
2. Check `/mode_control/current_mode == "GUIDED"`. If not, abort immediately (zero Twist, return `false`).
3. **RISE**: publish `Twist{linear.z = rise_vel}` (zero x/y/angular.z) at `cmd_rate_hz`, ramping down to `crawl_vel` once `/top/ultrasonic/distance` reports within `crawl_dist_m` (if the sensor is valid — if it's stuck at `0.0`, this ramp is simply skipped, since `/ice_touch_detection/touching` already has its own pressure/IMU fallback and is the authoritative signal, not the raw range). Exit on: `touching==true` (success), risen distance ≥ `delta_z_m` from this waypoint's CSV row (abort — no ice present or sensor fault), `rise_timeout_s` elapsed (abort), or GUIDED lost (abort). On abort: zero Twist, best-effort descend back to pre-approach depth, return `false`.
4. **HOLD_CONTACT**: publish `Twist{linear.z = press_vel}` for this waypoint's `hold_s` seconds. If contact is lost for more than `contact_lost_grace_s`, attempt exactly one bounded re-approach (reuse the RISE loop, bounded by `rise_timeout_s`); if regained, resume holding for the remaining time; if not, log a warning and proceed to descend anyway (per confirmed design — don't hard-fail the waypoint over this).
5. **DESCEND**: publish `Twist{linear.z = -descend_vel}` until `|current_z - pre_approach_z| <= depth_tolerance_m` or `descend_timeout_s` elapses.
6. Always publish a zero `Twist` on every exit path (success, abort, or timeout) before returning — the mavlink_bridge's own `cmd_vel` watchdog (zeroes velocity if the topic goes silent for 0.8s) is the real backstop, but the plugin shouldn't rely on it alone.
7. If mode drops out of GUIDED at any point mid-maneuver: stop immediately, zero Twist, return `false`. The plugin does **not** disarm or force a mode change itself — it has no way to know if the mode change was a deliberate pilot takeover, so it only stops commanding and reports failure.

**Parameters** (new `ice_approach_task_executor` block in `nav2_params.yaml`, under `waypoint_follower`):

| Parameter | Default | Purpose |
|---|---|---|
| `rise_vel` | 0.05 m/s | Commanded ascent speed before the crawl ramp-down |
| `press_vel` | 0.03 m/s | Light constant upward thrust during hold |
| `descend_vel` | 0.05 m/s | Descent speed magnitude |
| `crawl_dist_m` | 0.15 m | Ultrasonic range below which RISE slows to `crawl_vel` |
| `crawl_vel` | 0.02 m/s | Slow approach speed near the ceiling |
| `depth_tolerance_m` | 0.10 m | DESCEND completion tolerance |
| `rise_timeout_s` | 60.0 s | Max RISE duration before no-contact abort |
| `descend_timeout_s` | 60.0 s | Max DESCEND duration |
| `contact_lost_grace_s` | 2.0 s | Grace period before triggering one re-approach |
| `plain_waypoint_pause_ms` | 1250 | Pause for non-`up_down` waypoints (replaces `wait_at_waypoint`) |
| `cmd_rate_hz` | 10.0 Hz | Twist publish rate |
| `touching_topic` | `/ice_touch_detection/touching` | |
| `ultrasonic_topic` | `/top/ultrasonic/distance` | |
| `odom_topic` | `/odometry/filtered/local` | |
| `mode_topic` | `/mode_control/current_mode` | |
| `meta_topic` | `/mission_waypoints_meta` | |
| `cmd_vel_topic` | `/pixhawk/cmd_vel` | |

All velocities are comfortably above the mavlink_bridge's 0.01 m/s linear deadband (`ros2_receiver.py`).

### 3. Config wiring

- `src/autonomy/orca_nav2/CMakeLists.txt`: add an `ament_auto_add_library(ice_approach_task_executor SHARED src/ice_approach_task_executor.cpp)` block mirroring the existing `goal_checker_3d` entry (deps: `nav2_util nav2_core rclcpp rclcpp_lifecycle pluginlib geometry_msgs nav_msgs std_msgs`), plus `pluginlib_export_plugin_description_file(nav2_core ice_approach_task_executor_plugin.xml)` and install the new plugin XML alongside the existing ones.
- `src/autonomy/orca_nav2/package.xml`: add the `<nav2_core plugin=.../>` export entry for the new plugin XML (no new `<depend>` needed — all required packages are already declared).
- `src/autonomy/autonomy_bringup_pkg/params/nav2_params.yaml`: replace the `waypoint_follower` block's `waypoint_task_executor_plugin: "wait_at_waypoint"` with `"ice_approach_task_executor"`, and replace the `wait_at_waypoint` parameter sub-block with the `ice_approach_task_executor` block from the table above (this plugin *replaces* `wait_at_waypoint` entirely — Nav2 only supports one task executor plugin at a time — which is why `plain_waypoint_pause_ms` is folded in).

## Critical files

- `src/autonomy/autonomy_bringup_pkg/autonomy_bringup_pkg/mission_waypoint_loader.py` — publish metadata
- `src/autonomy/autonomy_bringup_pkg/autonomy_bringup_pkg/load_wgs84_points_to_waypoints.py` — reuse `process_mission()` and `WaypointMeta` as-is, no changes
- `src/autonomy/orca_nav2/src/ice_approach_task_executor.cpp` — new plugin (reference `goal_checker_3d.cpp` and `progress_checker_3d.cpp` for the nav2_core plugin + `PARAMETER()` macro conventions)
- `src/autonomy/orca_nav2/ice_approach_task_executor_plugin.xml` — new pluginlib description file
- `src/autonomy/orca_nav2/CMakeLists.txt`, `package.xml` — build/export wiring
- `src/autonomy/autonomy_bringup_pkg/params/nav2_params.yaml` — `waypoint_follower` section

## Verification

**Phase 1 — metadata plumbing (no Nav2, no hardware):** launch `mission_waypoint_loader` standalone against `smaller_square.csv` (already has `up_down=1` rows) with a manually-published fake latched `/gnss_datum`. `ros2 topic echo /mission_waypoints_meta` and confirm the stride-3 values match the CSV rows, and that the element count matches `/mission_waypoints_enu`'s pose count.

**Phase 2 — state machine dry run (fake sensors, no wet test):** this is the step that validates the dedicated-executor-thread pattern flagged above as the novel/highest-risk piece. Run `waypoint_follower` with the new plugin against a 1-row fake mission, manually publish fake `/odometry/filtered/local`, `/mode_control/current_mode` (`"GUIDED"`), `/ice_touch_detection/touching`, and `/top/ultrasonic/distance`. Confirm: `/pixhawk/cmd_vel` shows nonzero `linear.z` during RISE, switches to `press_vel` once `touching` goes `true`, holds for `hold_s`, descends, and zeroes on exit. Separately test the no-contact-abort path (never publish `touching=true`, confirm timeout/`delta_z_m` abort and that `stop_on_failure: False` lets the mission continue) and the GUIDED-loss path (flip mode mid-RISE, confirm immediate zero Twist + `false` return).

**Phase 3 — pool test:** build `pool_ice_test.csv` (clone `pool_mission.csv`/`smaller_square.csv`'s `up_down=1` rows, with `delta_z_m`/`hold_s` sized to the pool's actual depth and whatever physical ceiling/fixture is available), select it via `mission.yaml`, and run the full mission with a human on the joystick ready to flip to `MANUAL`. Monitor `/ice_touch_detection/touching`, `/top/ultrasonic/distance`, `/pixhawk/cmd_vel`, `/odometry/filtered/local` live (Foxglove or `ros2 bag record`), comparing against the existing `recordings/heave_guided_lol_2026_10_14-15_33_00` and `recordings/heave_send_waypoints_pool_2026_10_14-15_55_37` pool-test traces as a sanity baseline for achievable heave velocities in GUIDED mode.
