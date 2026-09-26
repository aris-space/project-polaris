# POLARIS — Simulation Merge Plan

**Target repo:** `aris-space/project-polaris`
**Target branch:** `simulation`, cut from `autonomy/main` (`45aad65`)
**Source of sim code:** `paulzambelli/project-polaris-simulation-personal` (`59e9468`) — stays alive as scratch space, is *not* deleted
**Goal:** one repo that runs the autonomy stack in Gazebo SITL on dev machines and on the Jetson AGX, with no copy-pasted packages or parameters between repos.

---

## 0. Read this first

### 0.1 What this document is

An ordered task list. Each task has a **goal**, the **files** it touches, the **actions**, and an **acceptance check** that must pass before moving on. Tasks inside a phase are ordered; phases are ordered.

### 0.2 How these tasks are written, and why

This plan was written by reading the repos, **not by building them**. No `colcon build` was run, no Gazebo was launched, no hardware was touched. Anything that could not be verified by reading is marked:

> **VERIFY FIRST:** `<command>` — do this before editing, and correct the plan if reality differs.

When a VERIFY-FIRST check contradicts this document, **reality wins**. Note the contradiction in `docs/SIM_MERGE_LOG.md` (create it, append-only) and continue. Do not silently adapt.

### 0.3 Invariants — never violate these

1. **Hardware behaviour must not change.** After every phase, `ros2 launch config_pkg start_system.launch.py` must produce the same node graph, the same parameters and the same MAVLink traffic as it does on `autonomy/main` today. The only permitted hardware-facing change in this entire plan is T0.3, and it is default-preserving.
2. **`autonomy/main` is truth.** Wherever `autonomy/main` and the sim repo differ on a value, algorithm or file, `autonomy/main` wins unless this document names a specific exception. The sim repo is behind, not ahead.
3. **`use_sim_time: True` must be unreachable from any hardware entry point.** Enforced structurally in T2.4, not by convention.
4. **Nothing in `src/simulation/` may ever be built on the Jetson.** Enforced in T1.4.
5. **No duplicated packages.** After Phase 1 there is exactly one `orca_nav2`, one `mavlink_bridge`, one `nav2_params.yaml` in the workspace. If you find yourself copying a tuning value from one file to another, stop — that is the problem this plan exists to remove.
6. **Do not fix unrelated bugs.** Several are catalogued in Appendix C. They are real, they are out of scope, and fixing them here makes this merge unreviewable.

### 0.4 Do-not-touch list

| Path / thing | Why |
|---|---|
| `Dockerfile.arm64` and everything in `aris-space/project-polaris-docker` | The Jetson image is working and is built by a separate pipeline. The sim image lives in *this* repo and derives from `ros:humble-ros-base`, never from L4T. |
| `src/config/config_pkg/config_pkg/constants.py` values | Hardware ports, IPs, RTK thresholds. T0.3 adds an override mechanism; it does not change any value. |
| `src/navigation/ekf_localization_pkg/config/ekf_local.yaml` | Every non-default value carries a bag name and a number. Sim must adapt to it, not the reverse. |
| Any tuning value in `params/nav2_params.yaml` | See T0.4. |
| `surge` / `heave` / `yaw_rate` sign handling in `ros2_receiver.py` | See Appendix C.1. Needs a bench test, not a refactor. |
| The custom mixer in the ArduSub fork | See Appendix C.2. The sim should reproduce the current behaviour, including its bugs. |

---

## 1. Established facts

Verified by reading the repos on 2026-09-22. Commands given so you can re-verify.

### 1.1 Repos and commits

| Thing | Value |
|---|---|
| Main repo `autonomy/main` HEAD | `45aad65` "Merge branch 'autonomy/dev' into autonomy/main. [Sept] Before Übergabe to Noel." |
| Main repo `dev` HEAD | `104e240` — **older** than `autonomy/main` for `mavlink_bridge` |
| Sim repo HEAD | `59e9468` |
| ArduSub fork, branch to use | `polaris-pressure-filter`, HEAD `0a75bdf` |

### 1.2 The ArduSub fork — important

> **The `master` branch of `aris-space/project-polaris-ardusub` has an EMPTY `SUB_FRAME_CUSTOM` case.** The `break;` is commented out, so with `FRAME_CONFIG=7` it falls through into `SUB_FRAME_SIMPLEROV_3` — a three-thruster ROV. Anything built from `master` silently flies the wrong vehicle.

The Polaris mixer exists only on `polaris-custom-frame` and `polaris-pressure-filter`. Both carry byte-identical factors. `polaris-pressure-filter` HEAD `0a75bdf` is what Paul's Dockerfile already pins — keep that pin.

**VERIFY FIRST** before any SITL work:
```bash
git ls-remote --heads https://github.com/aris-space/project-polaris-ardusub.git
# confirm polaris-pressure-filter still exists and note its SHA
```
At boot ArduSub must print `Frame: CUSTOM`. If it prints `SIMPLEROV_3`, the wrong branch was built. Make this a hard gate in T2.6.

Mixer factors are in Appendix A.

### 1.3 The `:dev` image, and what `:sim` must add

`Dockerfile.x64` is `FROM ros:humble-ros-base`, runs as **root**, `ROS_WS=/ros2_ws`, ROS at `/opt/ros/humble`, and already ships `colcon`, `rosdep`, `vcstool`, `nav2`, `robot_localization`, `mavros-msgs`, `marine-acoustic-msgs`, `foxglove-bridge`, `pymavlink`, `MAVProxy`, Node.js and Claude Code.

Missing for sim, and therefore the entire content of the `:sim` delta:
- `gz-harmonic` + `ros-humble-ros-gzharmonic` from `packages.osrfoundation.org`
- the OSRF rosdep list (`00-gazebo.list`)
- `ardupilot_gazebo` (Gazebo system plugin)
- ArduSub SITL built from the pinned fork
- `bluerov2_gz`, `ros2_shared` via `vcs`
- `setuptools<80` (colcon passes `setup.py develop` flags that 80+ removed)
- `rviz2` is **absent** from `ros-base` — do not add it. Use Foxglove, which the team already uses.

### 1.4 What SITL gives you for free

This materially shrinks Phase 3. ArduSub SITL simulates a barometer and reports servo outputs, and `mavlink_publisher` already republishes both. So:

| Consumer | Needs | In sim, comes from |
|---|---|---|
| `pressure_z_ned_to_pose_node` | `sensor_msgs/FluidPressure` on `/pixhawk/scaled_pressure` | **Free** — SITL baro via `mavlink_publisher` |
| `thruster_velocity_estimator` | `/pixhawk/servo_output_raw` | **Free** — SITL servo outputs via `mavlink_publisher` |
| `ekf_local` `imu0` | `sensor_msgs/Imu` on `/imu/data_corrected` | Gazebo IMU sensor → `ros_gz_bridge` → real `imu_yaw_correction` node (T3.1) |
| `ekf_local` `odom0` | `nav_msgs/Odometry` on `/sensors/dvl/odometry_cov` | Synthetic DVL → real `odometry_covariance_node` (T3.2) |
| `gnss_datum_watchdog`, `ros2_receiver.gps_origin_cb` | `NavSatFix` + `ublox_ubx_msgs/UBXNavHPPosLLH` | Synthetic (T3.3) |

**Design rule for Phase 3:** inject synthetic data as far *upstream* as possible, so the real conversion nodes run. Publish `/sensors/dvl/odometry` + `/sensors/dvl/velocity` and let the real `odometry_covariance_node` produce `/sensors/dvl/odometry_cov` — do not publish `odometry_cov` directly. Every node you bypass is a node the sim no longer tests.

### 1.5 Verified topic contract

| Topic | Type | Produced by (hardware) |
|---|---|---|
| `/sensors/dvl/velocity` | `marine_acoustic_msgs/Dvl` | `dvl_a50` driver |
| `/sensors/dvl/odometry` | `nav_msgs/Odometry` | `dvl_a50` driver |
| `/sensors/dvl/odometry_cov` | `nav_msgs/Odometry` | `dvl_a50_pkg/odometry_covariance_node` |
| `/sensors/thruster/odometry_cov` | `nav_msgs/Odometry` | `thruster_velocity_estimator` |
| `/pixhawk/scaled_pressure` | `sensor_msgs/FluidPressure` | `mavlink_publisher` (SCALED_PRESSURE2) |
| `/sensors/pressure/pose_enu` | `geometry_msgs/PoseWithCovarianceStamped` | `pressure_z_ned_to_pose_node` |
| `/imu/data_corrected` | `sensor_msgs/Imu` | `imu_yaw_correction` |
| `/odometry/filtered/local` | `nav_msgs/Odometry` | `ekf_local_node` |

`ekf_local.yaml` reads: `imu0: /imu/data_corrected`, `odom0: /sensors/dvl/odometry_cov`, `odom1: /sensors/thruster/odometry_cov`, `pose0: /sensors/pressure/pose_enu`, `frequency: 30.0`.

---

## 2. Target layout

```
src/
  autonomy/
    autonomy_bringup_pkg/       unchanged location; params stay the single source of truth
    orca_nav2/                  ONE copy, from autonomy/main
  comms/
    mavlink_bridge/             ONE copy, from autonomy/main, connection parameterised
  simulation/                   ← NEW. Never built on the Jetson.
    orca_description/           vendored from orca4 (MIT), meshes + worlds + model.sdf.in
    orca_sim_bringup/           sim_launch.py, cfg/sub.parm, ros_gz bridge, odom_to_tf
    orca_sim_sensors/           synthetic DVL + GNSS (Phase 3)
docker/
  Dockerfile.sim                ← NEW
  build_sim.sh                  ← NEW
.devcontainer/
  devcontainer.json             existing, unchanged (:dev)
  sim/devcontainer.json         ← NEW (:sim)
```

Package names keep the `orca_*` prefix — this avoids touching every `plugin:` key in `nav2_params.yaml` and all four plugin XML files.

**`orca_base` is not vendored.** `base_controller` is dead for Polaris (the real stack sends velocity via `mavlink_bridge`, not RC overrides, and TF comes from `ekf_local`). `odom_to_path_node` is a path visualiser that Foxglove replaces. If something later needs it, vendor it then.

---

## Phase 0 — Reconcile the forks

No new functionality. Ends with one authoritative copy of every shared file.

### T0.1 — Create the branch

```bash
git fetch origin
git checkout -b simulation origin/autonomy/main
```
Add `docs/SIM_MERGE_LOG.md` with a single heading; append to it whenever a VERIFY-FIRST check contradicts this plan.

**Acceptance:** `git log --oneline -1` shows `45aad65`.

---

### T0.2 — `orca_nav2`: confirm `autonomy/main` wins, salvage the docs

`autonomy/main` is **ahead** of the sim repo in two source files. Do not merge anything from the sim copy into these:

| File | `autonomy/main` has | sim copy has |
|---|---|---|
| `src/pure_pursuit_controller_3d.cpp` | `enable_velocity_divergence_emergency_` member + `PARAMETER(...)` registration + use in the divergence check; comment explaining why `prev_vel_` is *not* reset on replan | neither; resets `prev_vel_` on replan |
| `src/is_path_valid_check.cpp` | `max_xy_dist` / `max_z_dist` split with separate ports | single `max_dist` applied to both axes |

**Actions**
1. Change nothing in `src/autonomy/orca_nav2/src/`.
2. Copy these three files from the sim repo into `src/autonomy/orca_nav2/` (documentation only, no code):
   - `PURE_PURSUIT_CONTROLLER.md`
   - `PURE_PURSUIT_CONVERGENCE_CASES.md`
   - `PURE_PURSUIT_LEGACY_VS_REGULATED.md`
3. Read each one and correct any statement that describes the sim variant rather than the `autonomy/main` code — in particular anything about `max_dist` or about `prev_vel_` being reset. Prepend to each: `Describes src/autonomy/orca_nav2 as of <sha>.`
4. Ignore `orca_nav2/config/` and `orca_nav2/launch/` in the sim repo — superseded by `autonomy_bringup_pkg`.

**Acceptance:** `git diff --stat origin/autonomy/main -- src/autonomy/orca_nav2/src/` is empty. Three new `.md` files exist.

---

### T0.3 — `mavlink_bridge`: parameterise the connection, change nothing else

This is the only task in the plan that touches vehicle-facing code. Keep the diff minimal and default-preserving.

**Context.** `autonomy/main`'s bridge is a strict superset of both the `dev` branch's and the sim fork's. It has the RTK `h_acc` gate with `GpsOriginConditions`, the fallback timer, and the `LIN_DEADBAND`/`ANG_DEADBAND` block that `dev` lacks. The sim fork is a *reduction* — it stripped `config_pkg`, `mavros_msgs` and `ublox_ubx_msgs` to stand alone. **Do not adopt the sim fork's file.** The only thing worth taking from it is the idea of an overridable connection URL.

**Actions**

1. `src/comms/mavlink_bridge/mavlink_bridge/ros2_receiver.py`, around line 62:
   ```python
   # Default is the vehicle's mavlink-router. Override for SITL via env var.
   _url = os.getenv("MAVLINK_RECEIVER_URL", Comms.MAVLINK_ROUTER_TCP)
   self.port = mavutil.mavlink_connection(_url)
   ```
   Apply the same pattern to `self._gcs_port` (line ~76) with `MAVLINK_GCS_URL`.

2. `src/comms/mavlink_bridge/mavlink_bridge/mavlink_publisher.py`, around line 86:
   ```python
   _url = os.getenv("MAVLINK_PUBLISHER_URL", f"{Comms.JETSON_IP_ADDRESS}:14600")
   self.port = mavutil.mavlink_connection(_url)
   ```

3. Add a bounded connect-retry around each `mavlink_connection` call: up to `MAVLINK_CONNECT_RETRIES` (default `1`) attempts with `MAVLINK_CONNECT_DELAY_SEC` (default `1.0`). **Default 1 means hardware behaviour is byte-identical to today** — no retry loop, fail fast. SITL sets it higher because ArduSub starts after the bridge.

4. Nothing else changes. Not the type mask, not the deadbands, not `ekf_odom_cb`, not the watchdog.

**Acceptance**
- `git diff origin/autonomy/main -- src/comms/mavlink_bridge/` touches only the lines above.
- With no env vars set, `grep` confirms the effective URLs are `Comms.MAVLINK_ROUTER_TCP` and `{JETSON_IP_ADDRESS}:14600`.
- **VERIFY FIRST:** `grep -rn "MAVLINK_ROUTER_TCP\|JETSON_IP_ADDRESS" src/` — confirm no other call site needs the same treatment.

**Hardware sign-off required.** This file flies the vehicle. Before it merges to `dev`, a human runs the bench check in `CHECK_AXIS.md` and confirms `/pixhawk/heartbeat` and arm/disarm still work.

---

### T0.4 — One `nav2_params.yaml`, forever

**Decision: every tuning value in `src/autonomy/autonomy_bringup_pkg/params/nav2_params.yaml` on `autonomy/main` is correct and stays.** The sim repo's values are older. This explicitly includes, keeping the `autonomy/main` value in every case: `lookahead_dist 1.7`, `x_accel 0.04`, `curvature_lookahead_dist 1.7`, `approach_velocity_scaling_dist 7.0`, `K_descelerate 1.0`, `rotate_to_heading_min_angle 1.3`, `rotate_to_heading_angular_vel 0.25`, `rotate_to_heading_decel 0.1`, `z_goal_tolerance 20.0`, `min_*_velocity_threshold`, `max_velocity_divergence_rad 1.4`, `min_speed_divergence_check 0.35`, `enable_velocity_divergence_emergency false`.

**Mechanism.** Sim does not get its own params file. `sim_launch.py` (T2.5) uses `RewrittenYaml` — the same mechanism `autonomy.launch.py` already uses — to rewrite exactly these keys:

| Key | Hardware | Sim (Phase 2) | Sim (Phase 3 onward) |
|---|---|---|---|
| `use_sim_time` | `False` | `True` | `True` |
| `odom_topic` | `/odometry/filtered/local` | `/odom` | `/odometry/filtered/local` |
| `default_nav_to_pose_bt_xml` | BT path | BT path | BT path |

Note that after Phase 3 the override list collapses to `use_sim_time` alone. That is the point.

**Actions**
1. Leave `nav2_params.yaml` unchanged.
2. Add at the top of the file:
   ```yaml
   # SINGLE SOURCE OF TRUTH for controller tuning, hardware and simulation alike.
   # Simulation overrides only use_sim_time and odom_topic, via RewrittenYaml in
   # src/simulation/orca_sim_bringup/launch/sim_launch.py.
   # If you are about to add a sim-only value here, or copy a value out of here
   # into a sim file, stop and read docs/SIM_MERGE_PLAN.md §T0.4.
   ```
3. Do **not** create `nav2_params_sim_overrides.yaml`. If a genuine sim-only parameter appears later, create it then, containing only keys that differ, and record why in `SIM_MERGE_LOG.md`.

**Acceptance:** `git diff` on `nav2_params.yaml` shows only the comment block.

---

### T0.5 — Retire the forked mission starter

The sim repo's `scripts/WSG84_mission_starter.py` transforms the CSV locally against `missions/default_mission_origin.json`. The real `autonomy_bringup_pkg/WGS84_mission_starter.py` consumes the latched `/mission_waypoints_enu` published by `mission_waypoint_loader` after RTK lock. They are different designs, and the sim one exists only because sim has no `/gnss_datum`.

**Actions**
1. Do not vendor `WSG84_mission_starter.py`, `load_wsg84_points_to_waypoints.py`, `nav2_ready_wait.py`, `pixhawk_ready_wait.py` or `mission_runner.py` from the sim repo. The `autonomy_bringup_pkg` versions are authoritative.
2. Vendor the mission CSVs only if `autonomy_bringup_pkg/missions/` lacks an equivalent. **VERIFY FIRST:** `ls src/autonomy/autonomy_bringup_pkg/missions/` — it already has `default_wgs84_mission.csv`, `default_mission_origin.json`, `goldbach_straightline_wgs84_mission.csv`, `pool_mission.csv`. Take only `smaller_square.csv` and `straight_line_mission.csv` if they are genuinely useful.
3. In Phase 2 the mission cannot be started the hardware way (no `/gnss_datum` yet). Accept that: Phase 2 acceptance uses a direct `NavigateToPose` goal. Phase 3 (T3.3) restores the real path.

**Acceptance:** no file named `*WSG84*` (note the sim repo's spelling) exists anywhere in `src/`.

---

### T0.6 — Salvage the remaining sim documentation

1. Copy `docs/POLARIS_FRAME_INTEGRATION.md` from the sim repo to `docs/`. Add a header noting Phase 1 is done (`FRAME_CONFIG 7` is in `sub.parm`), Phase 2 is blocked on CAD, and the fork SHA it names (`41b8a10e8d`) is stale — current is `0a75bdf`.
2. Do **not** copy `orca_bringup/HARDWARE_MERGE.md`. It describes taking orca4 *to* hardware, which is the opposite of what this repo does, and several of its claims (about `base_controller`, about localisation being a TODO) are false for Polaris. Extract nothing from it except what is already reflected in this plan.
3. Copy `LICENSE` from the sim repo to `src/simulation/LICENSE` (orca4 is MIT, Clyde McQueen) and add attribution to `src/simulation/README.md`.

---

## Phase 1 — Vendor the sim packages and gate the build

### T1.1 — Vendor `orca_description`

```bash
mkdir -p src/simulation
cp -r <sim_repo>/orca_description src/simulation/orca_description
```

**Actions**
1. Delete all `__pycache__/` and `.pyc` from the copy (the sim repo has them committed).
2. Keep the package name `orca_description`.
3. Confirm the hooks that export `GZ_SIM_RESOURCE_PATH` survived the copy (`orca_description/hooks/`).
4. Leave `models/orca4/model.sdf` as the BlueROV2 geometry for now. **Add a banner at the top of `model.sdf.in`:**
   ```
   WARNING: thruster geometry is still BlueROV2, not Polaris.
   Forces and torques in this sim do NOT match the real vehicle.
   Do not transfer PID or PSC tuning from this sim to hardware.
   See docs/SIM_MERGE_PLAN.md Phase 4.
   ```

**Acceptance:** `find src/simulation -name "__pycache__" | wc -l` is `0`. `du -sh src/simulation/orca_description` is roughly 17 MB.

---

### T1.2 — Create `orca_sim_bringup`

New `ament_cmake` package at `src/simulation/orca_sim_bringup/`, built from the sim repo's `orca_bringup` but stripped to sim-only content.

**Take:**
- `cfg/sub.parm` → `cfg/sub.parm` (unchanged; it already has `FRAME_CONFIG 7`)
- `scripts/odom_to_tf.py`
- `scripts/current_vector_node.py`
- `scripts/cmd_vel_ramp.py` — **VERIFY FIRST:** `ls scripts/cmd_vel_ramp.py` at the main repo root. `DEBUG_GUIDED.md` references `./scripts/cmd_vel_ramp.py` there. If it exists, do not vendor a second copy; reference the existing one.
- `launch/sim_launch.py` → rewritten in T2.5

**Do not take:** `params/nav2_params.yaml`, `behavior_trees/`, `missions/`, `params/sim_orca_params.yaml`, `params/sim_mavros_params.yaml`, `launch/bringup.py`, `launch/navigation_launch.py`, `launch/hardware_launch.py`, `scripts/WSG84_mission_starter.py` and friends. All are superseded by `autonomy_bringup_pkg` or are orca4 leftovers.

`package.xml` `exec_depend`s: `ros_gz_bridge`, `ros_gz_sim`, `orca_description`, `autonomy_bringup_pkg`, `mavlink_bridge`, `ekf_localization_pkg`, `rclpy`, `tf2_ros`, `geometry_msgs`, `nav_msgs`.

> `ros_gz_bridge` here is exactly the dependency that must never reach the Jetson. T1.4 handles it.

**Acceptance:** `colcon list` shows `orca_sim_bringup`. `grep -rn "nav2_params" src/simulation/` returns nothing.

---

### T1.3 — External simulation dependencies

Create `src/simulation/simulation.repos`:
```yaml
repositories:
  bluerov2_gz:
    type: git
    url: https://github.com/clydemcqueen/bluerov2_gz
    version: main
  ros2_shared:
    type: git
    url: https://github.com/ptrmu/ros2_shared
    version: master
  ardupilot_gazebo:
    type: git
    url: https://github.com/ArduPilot/ardupilot_gazebo
    version: 4b30a3d8
```

These are consumed by `Dockerfile.sim` (baked into the image), **not** by `setup_submodules.sh`. Keeping them out of the workspace means the Jetson never sees them and `colcon` never tries to build them.

`ardupilot_gazebo` builds a Gazebo system plugin, not a ROS package — it goes on `GZ_SIM_SYSTEM_PLUGIN_PATH` and is built with plain CMake in the Dockerfile.

**Acceptance:** `.gitmodules` is unchanged.

---

### T1.4 — Gate the sim packages out of the Jetson build

Two independent mechanisms, because this failing is a bricked deployment.

**Mechanism 1 — `COLCON_IGNORE`, controlled by the entrypoint.**

In `entrypoint.sh`, before the `colcon build` block:
```bash
# Simulation packages are x86/dev-machine only. The sim image sets POLARIS_SIM=1.
if [ "${POLARIS_SIM:-0}" = "1" ]; then
  rm -f "${ROS_WS}/src/simulation/COLCON_IGNORE"
  echo "[entrypoint] Simulation packages ENABLED (POLARIS_SIM=1)."
else
  touch "${ROS_WS}/src/simulation/COLCON_IGNORE"
  echo "[entrypoint] Simulation packages ignored (POLARIS_SIM unset)."
fi
```
`src/simulation/COLCON_IGNORE` is **committed to git** so the default, in every context including a bare `colcon build` on the Jetson, is "ignored".

**Mechanism 2 — rosdep skip keys.**

Add `ros_gz_bridge ros_gz_sim` to `ROSDEP_SKIP_KEYS` in `entrypoint.sh` when `POLARIS_SIM != 1`. Belt and braces: with `COLCON_IGNORE` present, `rosdep install --from-paths src` should not see those packages anyway, but this makes it harmless if it does.

**Acceptance**
- `POLARIS_SIM=0 colcon list | grep -c simulation` is `0`.
- `POLARIS_SIM=1` with the `COLCON_IGNORE` removed, `colcon list | grep simulation` shows all sim packages.
- `git ls-files src/simulation/COLCON_IGNORE` shows the file tracked.
- **Regression check on the Jetson path:** the existing `docker compose up` flow builds exactly the same package set as before this branch.

---

### T1.5 — Entrypoint adjustments

`entrypoint.sh` otherwise stays as is. Add only:
1. The `POLARIS_SIM` block from T1.4.
2. When `POLARIS_SIM=1`, export the sim environment before the build:
   ```bash
   export GZ_VERSION=harmonic
   export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
   export PATH=/opt/ardupilot/build/sitl/bin:$PATH
   ```
   Paths must match `Dockerfile.sim` (T2.1). **VERIFY FIRST** after building the image: `which ardusub` and `ls $GZ_SIM_SYSTEM_PLUGIN_PATH`.

**Acceptance:** `bash -n entrypoint.sh` passes; the Jetson path prints "Simulation packages ignored".

---

## Phase 2 — Sim image, devcontainer, first run on ground-truth odometry

Phase 2 reproduces what Paul's repo does today, from the merged tree. It deliberately still uses Gazebo ground-truth odometry — that is replaced in Phase 3.

### T2.1 — `docker/Dockerfile.sim`

Multi-arch capable: base on `ros:humble-ros-base`, which is a multi-arch manifest, so the same file builds for `linux/amd64` and `linux/arm64` (Apple Silicon) with no changes. **Never base on `Dockerfile.arm64`** — that is L4T and belongs to the Jetson.

Structure, in this layer order so the expensive layers cache well:

1. Everything from `Dockerfile.x64` up to and including the pip block. Prefer `FROM ghcr.io/aris-space/project-polaris-docker:dev-x64` **only if** a matching arm64 tag exists; otherwise duplicate the apt/pip blocks. **VERIFY FIRST:** `docker manifest inspect ghcr.io/aris-space/project-polaris-docker:dev` — if it is a genuine multi-arch manifest, derive from `:dev`; if not, duplicate.
2. Gazebo Harmonic: OSRF keyring, apt source, `gz-harmonic`, `ros-humble-ros-gzharmonic`, and the OSRF rosdep list at `/etc/ros/rosdep/sources.list.d/00-gazebo.list`.
3. GL/X11 runtime: `libglvnd0 libgl1 libglx0 libegl1 libxext6 libx11-6`. Enough for headless EGL and for GUI on native Linux.
4. ArduSub build prereqs, then — **as a dedicated early layer, pinned by SHA** —
   ```dockerfile
   ARG ARDUSUB_SHA=0a75bdf
   RUN git clone https://github.com/aris-space/project-polaris-ardusub.git /opt/ardupilot \
         --recurse-submodules -b polaris-pressure-filter \
    && cd /opt/ardupilot && git checkout ${ARDUSUB_SHA} \
    && git submodule update --init --recursive
   ```
   Then `waf configure --board sitl && waf build --target bin/ardusub`.
   Putting this before any workspace content means autonomy changes never retrigger a 20-minute firmware build.
   ArduPilot's `install-prereqs-ubuntu.sh` resists running as root. Run it with `SKIP_AP_EXT_ENV=1 SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_GIT_CHECK=1 USER=root`, and if it still refuses, install the prereqs explicitly with apt instead — the waf SITL build needs far less than the full list. **VERIFY FIRST:** this is the single most likely step to fail on first build. Budget an iteration.
5. `ardupilot_gazebo` cloned to `/opt/ardupilot_gazebo`, built with CMake into `/opt/ardupilot_gazebo/build`.
6. `vcs import` of `simulation.repos` into `/opt/sim_deps/src`, `rosdep install`, `colcon build`, and a profile.d hook sourcing `/opt/sim_deps/install/setup.bash`. Keeping these out of `/ros2_ws` means they are not rebuilt on every workspace build.
7. `pip3 install "setuptools<80"`.
8. `ENV POLARIS_SIM=1`, `GZ_VERSION=harmonic`, `GZ_SIM_SYSTEM_PLUGIN_PATH`, `PATH` including `/opt/ardupilot/build/sitl/bin`.

`POLARIS_SIM=1` is set **only** in this image. That is what makes invariant 4 and invariant 3 structural.

**Acceptance**
```bash
docker run --rm <tag> bash -lc 'which ardusub && gz sim --versions && ros2 pkg list | grep ros_gz_bridge && echo $POLARIS_SIM'
```

---

### T2.2 — `docker/build_sim.sh`

The self-hosted runners are down, so this is a local build.

```bash
#!/usr/bin/env bash
set -euo pipefail
TAG="${TAG:-polaris:sim}"
docker build -f docker/Dockerfile.sim -t "$TAG" .
```

Also document, in `docker/README.md`, the manual multi-arch publish path for whoever has a fast machine, so the rest of the team can pull instead of building:
```bash
# on an arm64 machine
docker buildx build --platform linux/arm64 -f docker/Dockerfile.sim \
  -t ghcr.io/aris-space/project-polaris:internal-sim-arm64 --push .
# on an amd64 machine
docker buildx build --platform linux/amd64 -f docker/Dockerfile.sim \
  -t ghcr.io/aris-space/project-polaris:internal-sim-amd64 --push .
# from either
docker buildx imagetools create --tag ghcr.io/aris-space/project-polaris:sim \
  ghcr.io/aris-space/project-polaris:internal-sim-amd64 \
  ghcr.io/aris-space/project-polaris:internal-sim-arm64
```
State plainly in the README: first local build is 30–60 minutes and several GB, dominated by ArduSub and `ardupilot_gazebo`.

---

### T2.3 — `.devcontainer/sim/devcontainer.json`

Copy `.devcontainer/devcontainer.json` and change:
- `"name": "Polaris SIM"`
- `"image": "polaris:sim"` (local tag; switch to the ghcr tag once someone publishes one)
- drop `"--pull=always"` — the image is local
- add `"-e", "POLARIS_SIM=1"`
- add `"-e", "MAVLINK_RECEIVER_URL=tcp:127.0.0.1:5762"`, `"-e", "MAVLINK_PUBLISHER_URL=tcp:127.0.0.1:5760"`, `"-e", "MAVLINK_CONNECT_RETRIES=90"`
- keep `ROS_DOMAIN_ID=37`, `--network=host`, port 8765 for Foxglove

> SITL exposes SERIAL0 on tcp:5760 and SERIAL1 on tcp:5762, and accepts one client per port. Publisher and receiver must not share a port. **VERIFY FIRST** against ArduPilot's SITL port docs if the first connection attempt fails.

Leave the existing `.devcontainer/devcontainer.json` untouched.

---

### T2.4 — `use_sim_time` plumbing, with a structural guard

`autonomy.launch.py` currently hardcodes `use_sim_time = 'False'` with a comment saying it is deliberately not exposed. Preserve that intent while making sim possible.

In `src/autonomy/autonomy_bringup_pkg/launch/autonomy.launch.py`:

```python
import os

# use_sim_time defaults to False and can only be True inside the simulation
# image, which is the only place POLARIS_SIM=1 is ever set. A hardware
# launch that somehow requests True fails loudly instead of freezing every
# node on a /clock that will never publish.
_requested_sim_time = os.getenv('POLARIS_USE_SIM_TIME', '0') == '1'
if _requested_sim_time and os.getenv('POLARIS_SIM') != '1':
    raise RuntimeError(
        'POLARIS_USE_SIM_TIME=1 outside the simulation image. Refusing to '
        'start: use_sim_time=True on hardware freezes every node.'
    )
use_sim_time = 'True' if _requested_sim_time else 'False'
```

An env var rather than a launch argument, so no one can set it from a command line on the boat by habit. `sim_launch.py` sets `POLARIS_USE_SIM_TIME=1` in the environment it launches with.

**Acceptance**
- `ros2 launch config_pkg start_system.launch.py autonomy:=true` on a `:dev` container: every Nav2 node reports `use_sim_time = False` via `ros2 param get`.
- `POLARIS_USE_SIM_TIME=1` on a `:dev` container: launch aborts with the message above.
- In the `:sim` container via `sim_launch.py`: every Nav2 node reports `True`.

---

### T2.5 — `sim_launch.py`

`src/simulation/orca_sim_bringup/launch/sim_launch.py`. Base it on the sim repo's version, with these changes.

**Launch arguments**

| Arg | Default | Meaning |
|---|---|---|
| `gzclient` | `False` | Gazebo GUI. Headless is the default on every platform; GUI is usable on native Linux only. |
| `ardusub` | `True` | Start SITL. |
| `nav` | `True` | Start the Nav2 stack via `autonomy.launch.py`. |
| `ground_truth` | `True` in Phase 2, flips to `False` in Phase 3 | `True` = Gazebo odometry drives TF and Nav2; `False` = the real EKF does. Keep this arg permanently — A/B-ing the EKF against ground truth is the most useful diagnostic the sim offers. |

**Structure**
1. `ExecuteProcess` ArduSub SITL with `--defaults <orca_sim_bringup>/cfg/sub.parm`, `-M JSON`, `--home` from `autonomy_bringup_pkg/missions/default_mission_origin.json` (note: the mission origin now lives in `autonomy_bringup_pkg`, not in the sim package — one copy).
2. `ExecuteProcess` `gz sim -v 3 -r` with `-s` unless `gzclient`.
3. `ros_gz_bridge parameter_bridge` for `/clock`, `/model/orca4/odometry` → `/odom`, `/ocean_current`.
4. `odom_to_tf.py` under `IfCondition(ground_truth)`.
5. `IncludeLaunchDescription` of **`autonomy_bringup_pkg/launch/autonomy.launch.py`** — the real one, not a copy — inside a `TimerAction(period=5.0)`.
6. `IncludeLaunchDescription` of `mavlink_bridge`'s launch file.
7. `RewrittenYaml` applying the T0.4 override table.
8. `SetEnvironmentVariable('POLARIS_USE_SIM_TIME', '1')` as the first action.

**Do not** reimplement planner/controller/BT/waypoint-follower nodes here. If `sim_launch.py` grows a `Node(package='nav2_controller', ...)`, the merge has failed.

**Acceptance (T2.6 gate)**

1. `ros2 launch orca_sim_bringup sim_launch.py` starts without error.
2. ArduSub prints **`Frame: CUSTOM`**. If it prints anything else, stop — wrong firmware branch (§1.2).
3. `mavproxy` / `ros2 param`: `FRAME_CONFIG` reads `7`.
4. `ros2 topic hz /clock` publishes.
5. `ros2 param get /controller_server use_sim_time` → `True`.
6. `ros2 topic echo /pixhawk/heartbeat` shows a connection.
7. TF chain `map → odom → base_link` resolves (`ros2 run tf2_tools view_frames`).
8. Arm + GUIDED, send a `NavigateToPose` 2 m ahead; `/pixhawk/cmd_vel` is non-zero and thruster commands appear on `/model/orca4/joint/thrusterN_joint/cmd_thrust`.
9. **Jetson regression:** on a `:dev` container, `colcon build` produces the same package set as `origin/autonomy/main`.

Expect the vehicle to move *incorrectly* — the geometry is still BlueROV2. Phase 2 tests wiring, not physics.

---

## Phase 3 — Real sensors, real EKF

Goal: `ekf_local` runs in sim on synthesised sensor data, Nav2 consumes `/odometry/filtered/local`, and the RTK datum path executes — so the sim exercises the same graph as the vehicle.

### T3.1 — IMU

**VERIFY FIRST:**
```bash
grep -n "sensor" src/simulation/orca_description/models/orca4/model.sdf | head -40
gz topic -l | grep -i imu    # with the sim running
```
Determines whether the SDF already has a `<sensor type="imu">` publishing on a gz topic, or whether the IMU only feeds `ArduPilotPlugin` over the JSON link (in which case it never reaches ROS).

**If absent:** add `<sensor type="imu" name="imu_sensor">` to the `base_link`, with `<always_on>1</always_on>` and `<update_rate>100</update_rate>` — **VERIFY** the real XSens rate and match it.

**Then:** bridge it to ROS as `sensor_msgs/Imu`, and run the real `imu_yaw_correction` node to produce `/imu/data_corrected`. Do not publish `/imu/data_corrected` directly.

Note the real IMU runs in VRU mode (gyro-integrated yaw, no magnetometer) and drifts. To make the sim honest about the failure mode that caused the global-EKF divergence documented in `CLAUDE.local.md`, add an optional constant yaw-drift injection parameter, default `0.0`.

**Acceptance:** `ros2 topic hz /imu/data_corrected` at the expected rate; orientation broadly tracks Gazebo ground truth.

---

### T3.2 — Synthetic DVL

New `ament_python` package `src/simulation/orca_sim_sensors/`, node `sim_dvl_node`.

- **Subscribes:** `/odom` (Gazebo ground truth)
- **Publishes:** `/sensors/dvl/odometry` (`nav_msgs/Odometry`) and `/sensors/dvl/velocity` (`marine_acoustic_msgs/Dvl`)
- **Does not publish** `/sensors/dvl/odometry_cov` — the real `dvl_a50_pkg/odometry_covariance_node` produces that, and must be launched.

Parameters:

| Param | Default | Note |
|---|---|---|
| `rate_hz` | **VERIFY** against the A50 config | Match the real driver |
| `noise_sigma_xyz` | **VERIFY** against `measurement_noise_constants.py` | Reuse the real numbers rather than inventing any |
| `dropout_enable` | `false` | |
| `dropout_period_s` / `dropout_duration_s` | `60.0` / `10.0` | Exercises the `thruster_velocity_estimator` fallback |
| `bottom_lock_max_altitude_m` | **VERIFY** | Lose lock when too far off the bottom |

Velocity must be **body-frame**, matching the real driver. Getting this frame wrong is the single most likely bug in Phase 3 — verify by driving a pure yaw with zero translation and confirming the DVL twist stays near zero.

Leave a clearly commented seam where a real Gazebo DVL plugin would substitute (follow-up work; Gazebo Harmonic ships none).

**Acceptance:** `/sensors/dvl/odometry_cov` is published by the *real* covariance node; with `dropout_enable:=true`, `/sensors/thruster/odometry_cov` takes over and `/odometry/filtered/local` degrades gracefully rather than jumping.

---

### T3.3 — Synthetic GNSS / RTK

**VERIFY FIRST:** read the launch arguments of `ekf_localization.launch.py` for the exact `gps_fix_topic` and `h_acc_topic` names and defaults. Match them; do not assume.

Node `sim_gnss_node` publishes:
1. `sensor_msgs/NavSatFix` on the configured fix topic, derived from Gazebo ground truth against the mission origin.
2. `ublox_ubx_msgs/UBXNavHPPosLLH` on the configured h_acc topic, with `h_acc` in 0.1 mm units, so `gnss_datum_watchdog` and `ros2_receiver.gps_origin_cb` see the quality field they gate on.

Parameters: `h_acc_m` (default `0.05`, well inside the 0.50 m gate), `rtk_lock_delay_s` (default `10.0`, so the watchdog's wait is exercised rather than skipped), `available` (default `true`; setting false tests the no-datum path).

This is what makes the real mission path work in sim: `/gnss_datum` latches → `mission_waypoint_loader` publishes `/mission_waypoints_enu` → `WGS84_mission_starter` runs unmodified.

**Acceptance:** `ros2 topic echo /gnss_datum --once` returns; `/mission_waypoints_enu` is latched and non-empty; `ros2 run autonomy_bringup_pkg wgs84_mission_starter` completes a mission.

---

### T3.4 — Switch the odometry source

1. Default `ground_truth:=False` in `sim_launch.py`.
2. `RewrittenYaml` sets `odom_topic: /odometry/filtered/local`, making the override list `use_sim_time` only.
3. `odom_to_tf.py` runs only under `IfCondition(ground_truth)`; with the EKF running, `ekf_local_node` owns `odom → base_link` and `ekf_global_node` owns `map → odom`.
4. Watch for the double-launch hazard flagged in `AUTONOMY_CONTROL.md` §5.8 — `start_system.launch.py` can include `ekf_localization.launch.py` twice. `sim_launch.py` must include it exactly once.

**Acceptance**
- Exactly one publisher on `odom → base_link` (`ros2 topic info /tf --verbose`).
- With the sub stationary, `/odometry/filtered/local` pose stays within a few cm of `/odom` ground truth.
- Over a 2-minute mission, EKF-vs-ground-truth XY error is bounded and does not diverge.
- A full mission runs end to end using the same commands as `AUTONOMY_CONTROL.md` §7.

---

## Phase 4 — Polaris thruster geometry — **BLOCKED**

Do not start. Blocked on CAD data that is not currently available in clean form.

Until this phase is complete, the simulation reproduces the *control and software wiring* of the vehicle but **not its dynamics**. Forces and torques are BlueROV2's. Therefore:

> **No PSC, ATC or PID value may be transferred from this sim to the real vehicle.** That includes the Tier-1 "paste from SITL `sub.parm`" table in `DEBUG_GUIDED.md`, whose premise is that SITL is a trusted reference. It is not yet.

Put that sentence in `src/simulation/README.md` as well.

### Data needed, per thruster MOT_1..MOT_6

```
frame convention: x-forward, y-???, z-???     (state it explicitly)
origin:           CoG, or state the offset

MOT_n  position = [x, y, z] m        (from CoG)
       thrust_axis = [ax, ay, az]    (unit vector, direction of positive thrust on the hull)
```
Plus: dry mass (kg); inertia tensor about CoG in the same frame; displaced volume or net buoyancy (N, fresh water); CoB position relative to CoG; thruster model with max forward and reverse thrust (N) at the operating voltage.

### T4.1 — Mixer cross-check (do this *before* touching the SDF)

From the CAD numbers, recompute each motor's contribution: force term is the thrust axis; torque term is `r × axis`; then normalise the columns the way ArduSub does. Compare against the fork's actual factors in Appendix A.

The distinctive numbers to land on are `0.775` (MOT_2 yaw) and `0.833` (MOT_3/MOT_4 pitch). If the recomputed ratios do not reproduce those, **either the CAD numbers or the firmware is wrong**, and that must be resolved before any sim work. This check costs an hour and can invalidate a week.

### T4.2 — Rewrite the SDF geometry

Follow `docs/POLARIS_FRAME_INTEGRATION.md` Phase 2 and 3, which are already written and correct in approach. Key points that remain true:
- `model.sdf` is generated — edit `model.sdf.in` and `generate_model.py`, never the output.
- Keep `<axis>0 0 -1</axis>` and encode direction in link `<pose>` rpy.
- Do not touch the `modelXYZToAirplaneXForwardZDown` / `gazeboXYZToNED` transform blocks.
- Do not paper over sign errors with `MOT_n_DIRECTION` — fix the SDF.

### T4.3 — Optional mesh

A STEP or decimated STL converted to `.dae` gives the sim a Polaris-shaped hull instead of a BlueROV2. Cosmetic, but it stops people misreading screenshots. Independent of T4.1/T4.2 and can be done at any time.

---

## Phase 5 — Guardrails and documentation

### T5.1 — Duplicate-package guard

`scripts/check_no_duplicate_packages.sh`: walk `src/`, collect every `package.xml` `<name>`, exit non-zero on any duplicate. Runs in about a second. Wire it into a `pre-push` hook and document it. This is the cheap local substitute for the CI that the runners cannot currently provide.

### T5.2 — CI, written but dormant

`.github/workflows/build.yml`, triggered on `workflow_dispatch` and pushes to `simulation` only:
- job 1: `:dev`-equivalent build of the workspace with `POLARIS_SIM` unset
- job 2: sim build with `POLARIS_SIM=1`
- job 3: `check_no_duplicate_packages.sh`

`project-polaris` is public, so GitHub-hosted runners are free, including arm64 — no dependence on the self-hosted fleet. **VERIFY:** org settings may restrict Actions; check before relying on it. Leave it dormant until someone confirms.

### T5.3 — Documentation

1. `src/simulation/README.md`: what the sim does and does not model, the Phase 4 warning verbatim, how to run headless and with GUI, the ArduSub SHA pin and how to bump it.
2. Rewrite `src/autonomy/autonomy_bringup_pkg/README.md` against the current tree — it still references `orca_bringup`, `sim_launch.py` and an `orca4` Docker image that do not exist here (`AUTONOMY_CONTROL.md` §5.5).
3. Root `README.md`: fix the repository-structure section, which still lists `src/hardware`, `src/missionplanner`, `src/simulation` against an actual tree of `sensors`, `autonomy`, `controller`, `comms`, `config`, `navigation`, `measurement`, `ice_estimates`, `prototypes`. `src/simulation` becomes true for the first time with this branch.
4. Add a short contract, since Paul's repo stays alive as scratch:

   > **Scratch-space contract.** `project-polaris` is the single source of truth for `orca_nav2`, `mavlink_bridge` and `nav2_params.yaml`. Experiments in personal sim repos are welcome; changes to those three come back as a PR here. Anything else re-creates the divergence this branch removed.

### T5.4 — Update `AUTONOMY_CONTROL.md`

Item 5 in §5 ("No simulation in this repo") becomes resolved. Do not delete the entry — amend it with what now exists and what Phase 4 still lacks.

---

## Appendix A — The custom mixer, as it actually is

From `libraries/AP_Motors/AP_Motors6DOF.cpp`, `SUB_FRAME_CUSTOM`, branch `polaris-pressure-filter` @ `0a75bdf`:

| Motor | roll | pitch | yaw | throttle | forward | lateral | Comment in source |
|---|---:|---:|---:|---:|---:|---:|---|
| MOT_1 | 0 | 0 | 0 | 0 | **+1.0** | 0 | forward thruster (x) |
| MOT_2 | 0 | 0 | **+0.775** | 0 | 0 | **−1.0** | lateral back thruster (y) |
| MOT_3 | **+1.0** | −0.833 | 0 | 0.6 | 0 | 0 | upwards left thruster (z) |
| MOT_4 | **−1.0** | −0.833 | 0 | 0.6 | 0 | 0 | upwards right thruster (z) |
| MOT_5 | 0 | **+1.0** | 0 | **+1.0** | 0 | 0 | upwards front thruster (z) |
| MOT_6 | 0 | 0 | **−1.0** | 0 | 0 | **−1.0** | lateral front thruster (y) |

This is the **un-fixed** version: `DEBUG_GUIDED.md` argues the yaw column is sign-inverted relative to ArduSub's convention and proposes flipping MOT_2 to `−0.775` and MOT_6 to `+1.0`. That fix has **not** been applied. Leave it alone — a geometrically correct sim should reproduce the yaw inversion, which makes it a test for the fix rather than a place to smuggle it in.

---

## Appendix B — Parameter decisions, recorded

| Question | Decision | Basis |
|---|---|---|
| `nav2_params.yaml` tuning | `autonomy/main` wins entirely | Tested on the vehicle; the sim copy is older |
| `mavlink_bridge` | `autonomy/main` wins | Strict superset of both `dev` and the sim fork |
| `orca_nav2` sources | `autonomy/main` wins | Sim copy lacks two features |
| Mission starter | `autonomy_bringup_pkg` wins | Sim variant exists only to work around the missing datum; T3.3 removes the need |
| `sub.parm` | Sim-only for now | Cannot be a hardware reference until Phase 4 |
| Package naming | Keep `orca_*` | Avoids touching every `plugin:` key and four plugin XMLs |
| DVL simulation | Synthetic node first | No Gazebo Harmonic DVL sensor exists |
| Default GUI mode | Headless | Uniform across macOS/Windows/Linux; GUI opt-in on Linux |

---

## Appendix C — Known traps, deliberately out of scope

**C.1 — The surge sign comment.** In `ros2_receiver.cmd_vel_cb`, the comment says *"FLU forward -> negative vx"* while the code does `surge = lx` (positive). One of the two is wrong. `DEBUG_GUIDED.md` describes `surge = -msg.linear.x`, which matches neither. This is exactly the axis that has bitten the team before. **Do not resolve it during this migration** — it needs the props-off bench test in `DEBUG_GUIDED.md` §"Surge: targeted verification". Once the sim has correct geometry (Phase 4), it becomes reproducible there too.

**C.2 — The mixer yaw column.** See Appendix A. Out of scope.

**C.3 — Vertical control is disabled end-to-end.** `vz` is hard-coded `0.0` in the `set_position_target_local_ned_send` call, and `heave` is computed but never sent (`AUTONOMY_CONTROL.md` §5.2). Intentional, matching `feature/a_no_vertical` and `z_goal_tolerance: 20.0`, but undocumented in the code. The sim will therefore not move vertically under autonomy. **Expected — do not "fix" it.** Add a one-line comment at the call site saying so, and nothing more.

**C.4 — Double-launch of the local EKF.** `start_system.launch.py start_ekf:=true` and `odom_local_start:=true` both include `ekf_localization.launch.py`. Pre-existing. Relevant to T3.4 only insofar as `sim_launch.py` must not add a third path.

**C.5 — `emergency_stop_mode_node` is dead code that is still launched.** Pre-existing, unrelated, leave it.

---

## Appendix D — Order of operations, at a glance

```
T0.1 branch
 └─ T0.2 orca_nav2      ─┐
    T0.3 mavlink_bridge ─┤ no behaviour change, hardware sign-off on T0.3
    T0.4 nav2_params    ─┤
    T0.5 mission starter─┤
    T0.6 docs           ─┘
       └─ T1.1 orca_description
          T1.2 orca_sim_bringup
          T1.3 simulation.repos
          T1.4 build gating        ← Jetson regression check here
          T1.5 entrypoint
             └─ T2.1 Dockerfile.sim  ← longest single task, expect iteration
                T2.2 build_sim.sh
                T2.3 devcontainer
                T2.4 use_sim_time guard
                T2.5 sim_launch.py
                T2.6 ACCEPTANCE: sim runs on ground-truth odom
                   └─ T3.1 IMU
                      T3.2 DVL
                      T3.3 GNSS/RTK
                      T3.4 switch odom source
                      T3.5 ACCEPTANCE: full mission on the real EKF
                         └─ T5.1 duplicate guard
                            T5.2 CI (dormant)
                            T5.3 docs
                            T5.4 AUTONOMY_CONTROL.md

Phase 4 — BLOCKED on CAD. Independent of Phase 5.
```

---

*Written 2026-09-22 against `autonomy/main` @ `45aad65`, sim repo @ `59e9468`, ArduSub fork `polaris-pressure-filter` @ `0a75bdf`. Derived by reading the repositories; nothing in this plan was compiled, launched or bench-tested. Every VERIFY FIRST marker is a place the author could not check. Where this document and the tree disagree, the tree is right — record it in `docs/SIM_MERGE_LOG.md`.*
