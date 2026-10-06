# Simulation

Gazebo Harmonic + ArduSub SITL (the Polaris fork) + the **real** autonomy stack. Nav2, the
behaviour tree, `mavlink_bridge` and the Nav2 params are the same files the vehicle runs.
The only thing the sim changes about them is `use_sim_time` and the MAVLink connection URLs.

This is x86 / dev-machine only. It is built only in the `:sim` image (`POLARIS_SIM=1`) and never
on the Jetson (`COLCON_IGNORE` is committed in this directory).

> **No PSC, ATC or PID value may be transferred from this sim to the real vehicle.**
> Thruster geometry, mass and inertia are Polaris values from CAD (`orca_description/scripts/generate_model.py`),
> but they are not yet independently verified, and drag and added mass are hand estimates.
> See `docs/SIM_MERGE_PLAN.md` Phase 4.

- [What runs](#what-runs)
- [1. Get the image](#1-get-the-image)
- [2. Start the container](#2-start-the-container)
- [3. Build the workspace](#3-build-the-workspace)
- [4. Launch the sim](#4-launch-the-sim)
- [5. Using the running sim](#5-using-the-running-sim)
- [Ports](#ports)
- [What is not simulated yet](#what-is-not-simulated-yet)
- [Troubleshooting](#troubleshooting)

---

## What runs

`ros2 launch orca_sim_bringup sim_launch.py` starts:

```
 Gazebo (sand.world, orca4 model w/ Polaris thrusters)
   │  ▲  JSON physics link, UDP 9002
   ▼  │
 ArduSub SITL (FRAME_CONFIG 7, cfg/sub.parm)
   ├─ TCP 5760 ── mavlink_publisher  ─┐
   ├─ TCP 5762 ── ros2_receiver       ├─ mavlink_bridge (unmodified vehicle code)
   ├─ TCP 5763 ── ros2_receiver (GCS heartbeat)
   └─ UDP →14550 ─ free for you: MAVProxy / QGroundControl

 ros_gz_bridge:  /clock, /odom (Gazebo ground truth), /ocean_current
 odom_to_tf:     /odom → TF odom→base_link + /odometry/filtered/local   (ground_truth:=True)
 static TF:      map→odom                                                (ground_truth:=True)
 autonomy.launch.py (after 5 s): Nav2, BT navigator, mission loader, arm watchdog
 foxglove_bridge on ws://localhost:8765
```

With `ground_truth:=True` (the current default), Gazebo's true pose stands in for the EKF.
It is published on the same topic the vehicle's EKF uses, so `ros2_receiver` forwards it to
ArduSub as external navigation exactly as on the boat.

---

## 1. Get the image

No published image exists yet, so build it locally:

```bash
docker/build_sim.sh            # tags polaris:sim
```

The first build takes **30–60 min** and produces a ~8 GB image, mostly ArduSub SITL and
`ardupilot_gazebo`. Rebuilds are cached, and workspace edits never trigger one, because the
workspace is mounted at runtime. For details and the multi-arch publish path, see `docker/README.md`.

## 2. Start the container

**VS Code:** *Dev Containers: Reopen in Container → Polaris SIM* (`.devcontainer/sim/devcontainer.json`).

**Plain docker:**

```bash
docker run -it --rm --network=host --privileged \
  -e ROS_DOMAIN_ID=38 \
  -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$XAUTHORITY":/tmp/.Xauthority:ro -e XAUTHORITY=/tmp/.Xauthority \
  --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e QT_X11_NO_MITSHM=1 \
  -v "$PWD":/ros2_ws polaris:sim
```

- **Runs as `polaris`, not root.** `docker/build_sim.sh` gives it your UID/GID, so files it writes to the
  mounted workspace (`build/`, `install/`, `log/`, git) stay yours. It has password-less `sudo`.
- **Gazebo GUI** needs the X auth cookie (`$XAUTHORITY`) as well as `DISPLAY` and the X socket;
  without it: `unable to open display`. The cookie changes on every desktop login, so restart the
  container after logging in again.
- **`--gpus all`** needs an NVIDIA GPU and the NVIDIA container toolkit. Drop it (and
  `NVIDIA_DRIVER_CAPABILITIES`) otherwise; Gazebo then renders on the CPU (`llvmpipe`), slowly.
  The devcontainer adds it only where a GPU exists (`hostRequirements.gpu: optional`).
- **Don't add `--ipc=host`.** It shares the host's `/dev/shm` with the container, and Fast DDS
  shared-memory ports left there by killed processes can make new nodes hang (typically
  `ros2 topic list` stuck in the ros2 daemon). Without it, `/dev/shm` is fresh on every restart.

> **Do not use `ROS_DOMAIN_ID=37`** (the vehicle's) on a network the vehicle is on. With
> `--network=host`, the sim's `/pixhawk/cmd_vel`, `/pixhawk/arm_cmd` etc. would land in the
> vehicle's ROS graph. For extra safety, add `-e ROS_LOCALHOST_ONLY=1`. The Foxglove bridge still
> works from the same machine.

The MAVLink URLs (`MAVLINK_*_URL`) don't need to be set. `sim_launch.py` supplies the SITL
defaults, and any values already in the environment take precedence.

## 3. Build the workspace

Inside the container, from `/ros2_ws`:

```bash
colcon build --symlink-install --packages-skip xsens_mti_ros2_driver foxglove_bridge
source install/setup.bash
```

- A plain `colcon build` includes `src/simulation/*`, because the image sets
  `COLCON_DEFAULTS_FILE=/ros2_ws/docker/colcon_sim_defaults.yaml`. Add any new sim package there.
- The two skipped packages don't build on x86. The xsens driver ships arm64-only libraries, and the
  `foxglove_bridge` submodule needs a newer Humble. The sim uses the apt `foxglove_bridge` instead.
  This is a pre-existing issue, unrelated to the sim.
- With `--symlink-install`, edits to Python nodes, launch files and `cfg/sub.parm` apply on the next
  launch without a rebuild. C++ (`orca_nav2`) and `package.xml` changes still need one.

## 4. Launch the sim

```bash
ros2 launch orca_sim_bringup sim_launch.py                  # headless (default)
ros2 launch orca_sim_bringup sim_launch.py gzclient:=True   # with Gazebo GUI (native Linux only)
```

| Arg | Default | Meaning |
|---|---|---|
| `gzclient` | `False` | Gazebo GUI. Needs X11: `DISPLAY`, `/tmp/.X11-unix` and `$XAUTHORITY` mounted (section 2). |
| `ardusub` | `True` | Start ArduSub SITL. |
| `nav` | `True` | Start Nav2 via the vehicle's `autonomy.launch.py` (unconfigured, like on the boat). |
| `ground_truth` | `True` | `True`: Gazebo pose stands in for the EKF. `False`: the real EKF (Phase 3, in progress). |
| `gcs_url` | `udpclient:127.0.0.1:14550` | ArduSub SERIAL5 device for a human GCS. |
| `foxglove` | `True` | `foxglove_bridge` on port 8765. |

Startup takes about 10 s. It is ready when `ros2 topic echo --once /pixhawk/heartbeat` returns.
Every launch wipes ArduSub's saved parameters (`-w`) and reloads them from `cfg/sub.parm`.

---

## 5. Using the running sim

Open a second shell in the same container (VS Code: new terminal, or
`docker exec -it <container> bash`), then `source /ros2_ws/install/setup.bash`.

### 5.1 Drive it: a Nav2 goal, the same way as on the vehicle

```bash
# 1. GUIDED + arm (via mavlink_bridge, exactly like the operator does on the boat)
ros2 topic pub --once /pixhawk/mode_cmd std_msgs/msg/String "{data: GUIDED}"
ros2 topic pub --once /pixhawk/arm_cmd  std_msgs/msg/Bool   "{data: true}"
ros2 topic echo --once /pixhawk/heartbeat        # armed: true, mode: GUIDED

# 2. bring Nav2 up (it starts unconfigured)
ros2 run autonomy_bringup_pkg nav2_activate      # "Nav2 is active - safe to send missions."

# 3. send a goal 2 m ahead, in the map frame
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

- **Disarm:** `ros2 topic pub --once /pixhawk/arm_cmd std_msgs/msg/Bool "{data: false}"`.
  `nav2_arm_watchdog` then deactivates Nav2 by itself, as on the vehicle.
- **Deactivate Nav2 by hand:** `ros2 run autonomy_bringup_pkg nav2_deactivate`.
- **Mode names** for `/pixhawk/mode_cmd`: `MANUAL`, `STABILIZE`, `ALT_HOLD`, `GUIDED`, `POSHOLD`, `SURFACE`, …
- **`wgs84_mission_starter` does not work yet.** It waits for `/gnss_datum` (RTK lock), which the
  sim doesn't produce until the synthetic GNSS lands (Phase 3). Use `NavigateToPose` goals for now.
- **No vertical motion under autonomy.** `vz` is hard-coded to 0 in the bridge (intentional,
  `docs/SIM_MERGE_PLAN.md` Appendix C.3).

### 5.2 Drive it: raw velocity commands (bypassing Nav2)

`ros2_receiver` turns `/pixhawk/cmd_vel` (`geometry_msgs/Twist`, body frame) into GUIDED
velocity setpoints. It needs GUIDED + armed, and it sends zero if no message arrives for 0.8 s,
so publish at a rate:

```bash
ros2 topic pub -r 10 /pixhawk/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"      # surge
ros2 topic pub -r 10 /pixhawk/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.2}}"     # yaw
```

Don't run this while Nav2 is driving: both publish on `/pixhawk/cmd_vel`. For ramped
tests, there is `scripts/cmd_vel_ramp.py` at the repo root.

### 5.3 Talk to ArduSub directly (MAVProxy, QGroundControl, pymavlink)

The bridge holds all three SITL TCP ports. The free link for you is **SERIAL5, which sends
MAVLink to UDP 14550**, the port every GCS listens on by default.

```bash
# MAVProxy, inside the container
mavproxy.py --master=udpin:0.0.0.0:14550
#   param show FRAME_CONFIG      → 7
#   param set  PSC_POSXY_P 1.0   (lost on next launch: -w reloads sub.parm)
#   mode MANUAL / arm throttle / disarm / status / watch SERVO_OUTPUT_RAW
```

- **QGroundControl** on the same Linux host connects to UDP 14550 automatically when the container
  runs with `--network=host`. (Tested so far with MAVProxy/pymavlink inside the container only.)
- Only one program can listen on 14550. To use QGC and MAVProxy together, start MAVProxy with
  `--out=udp:127.0.0.1:14551` and point the second tool at 14551.
- **Persistent parameter changes** go in `orca_sim_bringup/cfg/sub.parm`, not via the GCS.
- **Scripting:**
  ```python
  from pymavlink import mavutil
  m = mavutil.mavlink_connection("udpin:0.0.0.0:14550"); m.wait_heartbeat()
  ```
- **SITL files:** `eeprom.bin` and dataflash logs (`logs/*.BIN`) are written to `~/.ros/ardusub_sitl/`
  in the container, which is not in the workspace. Copy them out if you want to keep a log.
- Arming or changing mode from the GCS works, but it bypasses what the vehicle operator does.
  Prefer the ROS topics in 5.1 when testing autonomy.

### 5.4 ROS topics worth watching

| Topic | Type | What |
|---|---|---|
| `/pixhawk/heartbeat` | `mavros_msgs/State` | armed / mode / connected, from ArduSub |
| `/pixhawk/cmd_vel` | `geometry_msgs/Twist` | velocity Nav2 (or you) sends to ArduSub |
| `/pixhawk/servo_output_raw` | `std_msgs/Int16MultiArray` | 6 thruster PWMs (1500 = stop) |
| `/pixhawk/attitude_quaternion` | `geometry_msgs/Quaternion` | ArduSub's attitude |
| `/pixhawk/scaled_pressure` | `sensor_msgs/FluidPressure` | SITL barometer |
| `/pixhawk/DEPTH_TARGET`, `DEPTH_ACHIEVED`, `DEPTH_VELOCITY`, `PID_ACCZ` | `Float32` / array | depth controller telemetry |
| `/odom` | `nav_msgs/Odometry` | Gazebo **ground truth** |
| `/odometry/filtered/local` | `nav_msgs/Odometry` | what Nav2 and ArduSub use as the vehicle pose (= `/odom` while `ground_truth:=True`) |
| `/tf`, `/tf_static` | | `map → odom → base_link` |
| `/clock` | `rosgraph_msgs/Clock` | sim time; every node runs with `use_sim_time: True` |
| `/ocean_current` | `geometry_msgs/Vector3` | current applied in Gazebo (you can publish to it, see [5.7](#57-ocean-current)) |

Useful checks:

```bash
ros2 topic hz /odometry/filtered/local                        # ~50 Hz
ros2 topic echo /pixhawk/servo_output_raw --field data
ros2 param get /controller_server use_sim_time                # True
ros2 run tf2_tools view_frames                                # writes frames.pdf
ros2 bag record -o sim_run /odom /odometry/filtered/local /pixhawk/cmd_vel /pixhawk/servo_output_raw /tf
```

### 5.5 Gazebo side

```bash
gz topic -l                                                       # all gz topics
gz topic -e -t /model/orca4/joint/thruster1_joint/cmd_thrust      # thrust ArduSub commands (N)
gz topic -e -t /model/orca4/pose
```

Thrusters are `thruster1..6`, matching `MOT_1..MOT_6` (see `orca_description/scripts/generate_model.py`).
For the GUI, relaunch with `gzclient:=True`. It needs native Linux with X11; on macOS/Windows,
stay headless and use Foxglove.

### 5.6 Foxglove

Open Foxglove → *Open connection* → `ws://localhost:8765`. The bridge runs inside the sim and
uses sim time. Add a 3D panel with the `map` fixed frame to see TF and the Nav2 path.

### 5.7 Ocean current

```
you / current_vector_node.py ─ROS /ocean_current─▶ ros_gz_bridge ─gz /ocean_current─▶ Hydrodynamics plugin (orca4)
```

- **What it does:** the Gazebo Hydrodynamics plugin (`model.sdf.in`) computes drag and added mass
  from the vehicle's velocity **relative to the water**. A current therefore pushes the vehicle
  along with it, by an amount set by the drag coefficients in `generate_model.py`. Buoyancy and
  thrust are unaffected.
- **Message:** the vector is a water velocity in **m/s, world frame** (Gazebo ENU: x east,
  y north, z up), not body frame.
- **Direction:** the bridge is ROS → Gazebo only. Nothing publishes on `/ocean_current`; you do.
- **The last value sticks.** The plugin keeps applying the last current it received. Stopping
  the publisher does not stop the current, so publish zeros to clear it.
- **QoS:** the bridge subscribes with `transient_local` durability. A default (volatile)
  publisher won't connect, so pass `--qos-durability transient_local`.

Constant current, set by hand:

```bash
ros2 topic pub --once --qos-durability transient_local /ocean_current geometry_msgs/msg/Vector3 "{x: 0.3, y: 0.0, z: 0.0}"
ros2 topic pub --once --qos-durability transient_local /ocean_current geometry_msgs/msg/Vector3 "{}"   # clear
```

Time-varying current: `current_vector_node.py` publishes at 20 Hz. `sim_launch.py` does **not**
start it, so run it yourself:

```bash
ros2 run orca_sim_bringup current_vector_node.py --ros-args \
  -p direction:=con_x -p amplitude:=0.3 -p noise_stddev:=0.05
ros2 param set /current_vector amplitude 0.5                       # change while running
```

| `direction` | Profile on the named axes |
|---|---|
| `con_x` `con_y` `con_z` `con_xy` `con_xz` `con_yz` `con_xyz` | constant `amplitude` |
| `dir_x` `dir_y` `dir_z` `xy` `xz` `yz` `xyz` | sine: `amplitude · sin(2π t / period)` |
| `ramp_x` … `ramp_xyz` | ramp: `amplitude/10 · t` (unbounded, keeps growing) |
| `''` (default) | zero |

| Param | Default | Meaning |
|---|---|---|
| `amplitude` | `0.0` | m/s |
| `period` | `0.0` | s, sine profiles only (`0` → no oscillation) |
| `noise_stddev` | `0.08` | m/s, first-order Gauss-Markov noise; only on axes whose letter appears in `direction`. `0` disables it |
| `noise_time_constant` | `1.5` | s, noise correlation time (`0` → white noise) |

When you stop the node, the last value it sent stays in effect, so clear it with the zero
publish above. Use the current for qualitative robustness tests, e.g. holding a line against a
cross-current. Compare `/odom` with the Nav2 path and watch `/pixhawk/servo_output_raw`. Drag is
hand-estimated, so the drift it produces is not a real disturbance-rejection number.

---

## Ports

| Port | Proto | Used by |
|---|---|---|
| 5760 | TCP | SITL SERIAL0 ↔ `mavlink_publisher` (`MAVLINK_PUBLISHER_URL`) |
| 5762 | TCP | SITL SERIAL1 ↔ `ros2_receiver` (`MAVLINK_RECEIVER_URL`) |
| 5763 | TCP | SITL SERIAL2 ↔ `ros2_receiver` GCS heartbeat (`MAVLINK_GCS_URL`) |
| 14550 | UDP | SITL SERIAL5 → your GCS (`gcs_url` launch arg) |
| 9002 | UDP | ArduSub ↔ Gazebo `ArduPilotPlugin` (JSON physics) |
| 8765 | TCP | Foxglove bridge |

SITL serves **one client per TCP port**. If you point a GCS at 5760/5762/5763, you knock the bridge off.

## What is not simulated yet

| Missing | Consequence | Tracked in |
|---|---|---|
| IMU, DVL, GNSS/RTK sensor streams | the real EKF can't run; `ground_truth:=True` replaces it | `docs/SIM_MERGE_PLAN.md` Phase 3 |
| `/gnss_datum` | `wgs84_mission_starter` can't run; use `NavigateToPose` | T3.3 |
| Verified dynamics | don't transfer tuning (see top of this file) | Phase 4 |
| Polaris hull mesh | the model still *looks* like a BlueROV2 | T4.4 |

## Troubleshooting

- **`ros2_receiver` keeps retrying the connection.** ArduSub isn't up yet (it gets 90 × 1 s). If it
  never connects, look for an `ardusub` error in the launch output, or check whether something
  else holds 5760/5762/5763: `ss -ltnp | grep 576`.
- **Everything seems frozen / timestamps are 0.** Nothing is publishing `/clock`. Gazebo didn't start,
  or ros_gz_bridge died. Check `ros2 topic hz /clock`.
- **Two sims on one machine.** With `--network=host`, they share ports and the ROS domain. Run one
  sim at a time, or give the second container its own network (drop `--network=host`) and its own
  `ROS_DOMAIN_ID`.
- **Wrong vehicle behaviour after bumping ArduSub.** Only the `polaris-pressure-filter` /
  `polaris-custom-frame` branches have the Polaris mixer. `master` flies a 3-thruster SIMPLEROV.
  Quick check: a pure surge command should move only servo 1 in `/pixhawk/servo_output_raw`.
- **`POLARIS_USE_SIM_TIME=1 outside the simulation image`.** You launched `autonomy.launch.py`
  with sim time outside `polaris:sim`. This is intentional: sim time on hardware freezes every node.

## Layout

| Package | What |
|---|---|
| `orca_description/` | Gazebo model (`models/orca4`, generated from `model.sdf.in` by `scripts/generate_model.py`) and `worlds/sand.world` |
| `orca_sim_bringup/` | `launch/sim_launch.py`, `cfg/sub.parm` (ArduSub SITL params), `odom_to_tf.py`, `current_vector_node.py` |
| `orca_sim_sensors/` | synthetic sensors for Phase 3 (in progress) |
| `simulation.repos` | external sim deps baked into the image (`bluerov2_gz`, `ros2_shared`, `ardupilot_gazebo`) |

`model.sdf` is generated. Edit `model.sdf.in` / `generate_model.py`, then run
`python3 scripts/generate_model.py models/orca4/model.sdf.in models/orca4/model.sdf 0`.

## Attribution

`orca_description` and parts of `orca_sim_bringup` are derived from
[orca4](https://github.com/clydemcqueen/orca4) by Clyde McQueen, MIT licensed — see `LICENSE`
in this directory. Adapted for Polaris via Paul Zambelli's `project-polaris-simulation-personal`.
