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

## 2026-09-27 — T0.4 / T0.5 / T0.6

- T0.5 VERIFY-FIRST: `autonomy_bringup_pkg/missions/` has the four files the plan lists, and the
  sim repo's `default_mission_origin.json` is byte-identical to it. Vendored only
  `smaller_square.csv` (≈11 m square at the default origin, parses with the real
  `load_wgs84_points_to_waypoints.process_mission`). `straight_line_mission.csv` not taken: it
  has a single active waypoint and adds nothing over `goldbach_straightline_wgs84_mission.csv`.
- §1.2 VERIFY-FIRST: `git ls-remote` confirms `polaris-pressure-filter` = `0a75bdf`,
  `polaris-custom-frame` = `fa43d26`, `master` = `c593f04`. Pin unchanged.

## 2026-09-27 — T1.1 – T1.5

- T1.1: `model.sdf` is reproduced exactly by `generate_model.py model.sdf.in model.sdf 0`
  (thrust-force mode); regenerated with that after adding the banner. Banner is an XML comment
  after the `<?xml?>` declaration and contains no `@` (the generator substitutes `@word`).
- T1.2: `cmd_vel_ramp.py` not vendored (exists at repo-root `scripts/`). `odom_to_tf.py` gained an
  optional `republish_topic` — see the T2.5 entry for why.
- T1.3: plan keeps `ardupilot_gazebo` in `simulation.repos` *and* clones it separately in T2.1.
  Only one is needed: it is a colcon package, so it is built with the other sim deps in
  `/opt/sim_deps` (as in Paul's Dockerfile). Plugin path is therefore
  `/opt/sim_deps/install/ardupilot_gazebo/lib/ardupilot_gazebo`, not `/opt/ardupilot_gazebo/build`.
- **T1.4 deviation (mechanism 1).** The plan deletes the committed `src/simulation/COLCON_IGNORE`
  in sim mode. That dirties the git tree of every sim container (the workspace is bind-mounted),
  and an accidental commit of the deletion silently removes the Jetson gate. Instead the file is
  never touched: sim mode passes the sim package directories explicitly as colcon
  `--base-paths` (verified: colcon only honours `COLCON_IGNORE` during recursive discovery, so an
  explicit base path pointing *at* a package is still found). For interactive `colcon build` in
  the sim container, `Dockerfile.sim` sets `COLCON_DEFAULTS_FILE=/ros2_ws/docker/colcon_sim_defaults.yaml`.
- **T1.4 regression found by testing, avoided.** colcon's default base path is `.`, not `src`:
  `colcon list` finds 37 packages, `colcon list --base-paths src` only 35 (it drops `ping-python`
  and `top_station/.../recorder_controller_node`). The non-sim path therefore passes no
  `--base-paths` at all; the sim path uses `--base-paths . <sim pkgs>`. Verified in
  `project-polaris-docker:dev-x64`: POLARIS_SIM unset → 37 packages, 0 sim; POLARIS_SIM=1 → 39.
- T1.4 mechanism 2: rosdep (via catkin_pkg) also honours `COLCON_IGNORE`; verified that
  `rosdep keys --from-paths src` does not see `ros_gz_*`. Skip keys added anyway as planned.

## 2026-09-27 — T2.1 – T2.6

- **T2.1 VERIFY-FIRST:** `project-polaris-docker:dev` *is* a multi-arch manifest, but per
  `build.yaml` its arm64 half is built from `Dockerfile.arm64` (L4T/Jetson). Deriving from it would
  put the Jetson image under Apple Silicon sim builds, so `Dockerfile.sim` duplicates the
  `Dockerfile.x64` blocks on `ros:humble-ros-base` instead.
- T2.1: ArduPilot prereqs installed explicitly (apt + `empy==3.3.4 pexpect lxml dronecan
  ptyprocess`) instead of `install-prereqs-ubuntu.sh`. First build succeeded; image 7.8 GB.
  `which ardusub` → `/opt/ardupilot/build/sitl/bin/ardusub` @ `0a75bdf`; `gz sim --versions` → 8.15.0;
  `libArduPilotPlugin.so` in `/opt/sim_deps/install/ardupilot_gazebo/lib/ardupilot_gazebo`.
- T2.2: no buildx plugin on the build machine; the legacy builder ignores
  `Dockerfile.sim.dockerignore`, so `build_sim.sh` builds from a temp context holding only
  `simulation.repos`. The dockerignore stays for the buildx multi-arch path in `docker/README.md`.
- T2.3: adds `MAVLINK_GCS_URL=tcp:127.0.0.1:5763` (see T0.3 entry) and `SERIAL2_PROTOCOL 2` in
  `sub.parm`. `sim_launch.py` sets the same URLs as defaults, so it also works outside the
  devcontainer. **Open question for the team:** the devcontainers use `--network=host` with
  `ROS_DOMAIN_ID=37`, the vehicle's domain. A sim on the same LAN as the vehicle would publish
  `/pixhawk/cmd_vel` etc. into the vehicle's graph. Test runs here used `ROS_DOMAIN_ID=87
  ROS_LOCALHOST_ONLY=1`. Not changed, as the plan says keep 37.
- **T2.4:** the lifecycle manager also hard-coded `use_sim_time: False`; it now follows the same
  guarded value (False on hardware). The local-EKF include is passed `use_sim_time` explicitly
  (value on hardware: `false`, equal to its default).
- **T2.5 deviation — odometry wiring.** On `autonomy/main`, `ros2_receiver` subscribes to
  `/odometry/filtered/local` and forwards it to ArduSub as MAVLink ODOMETRY; `sub.parm` makes that
  ArduSub's position/yaw source (`EK3_SRC1_* 6`). Pointing Nav2 at `/odom` (plan's Phase 2 table)
  would leave ArduSub without a position. Instead, ground-truth mode republishes Gazebo odometry
  on `/odometry/filtered/local` (odom_to_tf) and publishes a static `map→odom`. Consequences: the
  real bridge path runs in Phase 2, and the sim override list is `use_sim_time` alone already
  (no `odom_topic` override, no RewrittenYaml in `sim_launch.py` — `autonomy.launch.py` already
  rewrites `use_sim_time`).
- **T2.6 result (ground-truth odom), all run in `polaris:sim`:**
  1. `sim_launch.py` starts; ArduSub SITL ↔ Gazebo JSON link up; no errors.
  2./3. "Frame: CUSTOM" is not printed on the SITL console (it is a STATUSTEXT) and all three
     SITL TCP ports are occupied by the bridge, so it was checked functionally instead: during
     a pure-surge GUIDED command only servo1 (MOT_1, forward thruster) deviated (+16 µs), the other
     five stayed within ±1 µs — only the Polaris CUSTOM mixer does that.
  4. `/clock` 1000 Hz. 5. `use_sim_time` True on controller/planner/bt_navigator/waypoint_follower/
     lifecycle manager. 6. `/pixhawk/heartbeat` published; arm and GUIDED accepted via
     `/pixhawk/arm_cmd` / `/pixhawk/mode_cmd`. 7. `map→odom→base_link` resolves.
  8. `nav2_activate` → all Nav2 nodes active; `NavigateToPose` to (2,0) in `map` → SUCCEEDED after
     +1.24 m of travel (GoalChecker3D `xy_goal_tolerance` 0.8 m); `/pixhawk/cmd_vel` non-zero
     (211 samples); Gazebo `thruster1_joint/cmd_thrust` up to 0.5.
  - The EKF-origin concern (GUIDED needs `GPS_GLOBAL_ORIGIN`, set by `ros2_receiver` only after
    RTK) did not materialise: ArduSub accepted GUIDED and tracked without it.
  - `POLARIS_USE_SIM_TIME=1` with `POLARIS_SIM` unset aborts with the intended RuntimeError.
  9. Jetson regression: see T1.4 entry (37 packages, unchanged).
- **Found, out of scope (invariant 6), not fixed:** on x86 the workspace does not fully build —
  `xsens_mti_ros2_driver` links against arm64 static libs committed under `lib/xspublic/`, and
  the `src/comms/foxglove_bridge` submodule needs `ament_index_cpp/version.h`, absent from
  current Humble apt. Both are unrelated to simulation; the sim uses the apt `foxglove_bridge`.
  Built with `--packages-skip xsens_mti_ros2_driver foxglove_bridge`.

## 2026-09-29 — Correction: thruster geometry is already Polaris (affects T1.1, T2.6, Phase 4)

- The plan (Phase 4, T1.1 banner, T2.6 note) and `POLARIS_FRAME_INTEGRATION.md` say the SDF
  thruster geometry is still BlueROV2 and Phase 4 is blocked on CAD. **Wrong.** The simulation repo's
  `main` got Polaris geometry in `aaa8878` (paulzambelli, 2026-05-04, "Added also a new thruster
  configuration in the model"), and T1.1 vendored that: `generate_model.py` is byte-identical to the
  sim repo @ `59e9468`. It defines `t1_*`..`t6_*` individually ("Polaris CAD, body frame ≡ CoG, Gazebo
  FLU"). The BlueROV2 constants that `POLARIS_FRAME_INTEGRATION.md` Phase 2 says to replace no longer exist.
- Checked every branch of `paulzambelli/project-polaris-simulation-personal`. None has newer geometry than
  `main`. `feature/GoUp` and `tuning/PurePursuiteController3D` still have the old BlueROV2 layout (they
  predate `aaa8878`). `feature/Polaris-ardusub` only changes the spawn pose in `sand.world`.
- Also stale: drag is not BlueROV2-derived. It uses a Polaris hull cylinder (1.78 m × 0.16 m radius), and
  added mass is a fraction of the vehicle sheet. The visual mesh and buoyancy collision box are still BlueROV2-sized.
- First-pass mixer consistency, from the poses: yaw 0.537/0.695 = 0.773 (mixer 0.775), pitch 0.493/0.600
  = 0.82 (mixer 0.833). Yaw signs agree with the *current* (un-"fixed") mixer after the FLU→FRD
  conversion. Not a full check. The full check is T4.2.
- Changes: `model.sdf.in` banner (and regenerated `model.sdf`, banner-only diff), `sim_launch.py` docstring
  and `src/simulation/README.md` now say "Polaris, unverified" instead of "BlueROV2". The no-tuning-transfer
  rule is unchanged. Plan Phase 4 is rewritten from "BLOCKED on CAD" to "verify" (T4.1 confirm source with
  Paul, T4.2 mixer cross-check, T4.3 SITL axis check, T4.4 mesh). `POLARIS_FRAME_INTEGRATION.md` header is updated.
