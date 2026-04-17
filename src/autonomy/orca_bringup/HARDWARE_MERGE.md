# Hardware Merge — Critical Details

Everything you need to know before running Orca4 on the real vehicle.
Use `hardware_launch.py` instead of `sim_launch.py`.

---

## 1. `use_sim_time` — the most important flag

In simulation every node reads the clock from Gazebo via `/clock`.
On hardware the system clock is used.  If any node is left with
`use_sim_time: True` while `/clock` is not publishing, that node
freezes and never processes messages.

### Where it appears and what to do

| File | Location | Status |
|---|---|---|
| `launch/sim_launch.py` | line 61, hardcoded `True` | **Do not use on hardware** |
| `launch/bringup.py` | line 46, default `'True'` | **Do not use directly** — `hardware_launch.py` wraps it correctly |
| `launch/navigation_launch.py` | line 40, default `'true'` | Passed `False` by `hardware_launch.py` |
| `params/nav2_params.yaml` | lines 5, 68, 72, 139, 199, 218, 225 | Overridden at launch via `RewrittenYaml` in `hardware_launch.py` |
| `comms/mavlink_bridge/launch/mavlink_bridge.launch.py` | line 35, default `'true'` | Passed `False` by `hardware_launch.py` |

**`hardware_launch.py` hardcodes `use_sim_time = 'False'` and does not
expose it as a launch argument.**  This is intentional — it cannot be
accidentally set to `True` on the vehicle.

---

## 2. MAVLink connection URLs

The MAVLink bridge defaults to SITL on localhost.  Override with
environment variables **before** launching:

```bash
# Example: Pixhawk reachable via UDP (BlueROV2 companion computer default)
export MAVLINK_PUBLISHER_URL="udp:192.168.2.2:14550"   # read telemetry
export MAVLINK_RECEIVER_URL="udp:192.168.2.2:14551"    # send commands

# Or serial (direct USB/UART):
export MAVLINK_PUBLISHER_URL="serial:/dev/ttyUSB0:115200"
export MAVLINK_RECEIVER_URL="serial:/dev/ttyUSB0:115200"

ros2 launch orca_bringup hardware_launch.py
```

Source files: `mavlink_publisher.py` line 80, `ros2_receiver.py` line 72.

---

## 3. Localization (map → odom TF) — must run continuously

### Why this is critical

The TF chain `map → odom → base_link` must be intact at all times:

- `odom → base_link` is published by `base_controller` (from motion model + ArduSub EKF).
- `map → odom` **must come from an external localization source**.

In simulation Gazebo provides ground-truth odometry and the base_controller
fuses it with a SLAM pose.  On hardware **you must provide a real localization
node** that continuously broadcasts `map → odom`.

### Why it must never stop

The behavior tree recovery is a `Wait` node (the vehicle drifts/hovers in
place for a few seconds).  There is **no re-localise step** in the BT.
If the `map → odom` TF disappears, the Nav2 lifecycle manager cannot
reconfigure and the entire navigation stack stalls — there is no automatic
recovery from a lost TF on this BT.

### Placeholder in `hardware_launch.py`

There is a `# TODO: LOCALIZATION NODE` block in `hardware_launch.py` where
you insert your localization node.  Options:

- **DVL dead-reckoning + USBL initial fix** — most common for AUV
- **Visual SLAM** (RTAB-Map, ORB-SLAM2) — if cameras are available
- **Static TF** (`static_transform_publisher map odom`) — bench testing
  only, will not work in open water

---

## 4. Nodes removed for hardware (do NOT add them back)

| Node / Script | Why it was in simulation | Hardware replacement |
|---|---|---|
| `gz sim` | Gazebo physics engine | Real vehicle |
| `ros_gz_bridge` (parameter_bridge) | Bridges `/clock`, `/odom`, `/ocean_current` from Gazebo | Real clock, real odometry, real current estimation |
| `odom_to_tf.py` | Publishes `odom → base_link` when base controller is disabled | `base_controller` always enabled on hardware |
| `current_vector_node.py` | Simulates ocean current for disturbance testing | DVL or model-based current estimation |
| ArduSub SITL (`ardusub -S ...`) | Software-in-the-loop flight controller | Real Pixhawk running ArduSub |

---

## 5. `orca_bringup/package.xml` — remove simulation dependency

Remove `ros_gz_bridge` from `package.xml` before building for hardware:

```xml
<!-- REMOVE this line: -->
<exec_depend>ros_gz_bridge</exec_depend>
```

`ros_gz_bridge` is only used in `sim_launch.py`.  It is not needed on
hardware and will cause a build/install error if the Gazebo packages are
not present on the vehicle computer.

---

## 6. `cfg/sub.parm` — ArduSub parameters

`cfg/sub.parm` is the ArduSub parameter file.  It is loaded from the same
file for SITL and real hardware, but **verify every value before flashing
the real Pixhawk**:

- Thruster motor channel assignments must match the actual BlueROV2 wiring.
- PID gains tuned in simulation may need re-tuning on hardware.
- Sensor assignments (compass, barometer, DVL input) depend on your frame.

---

## 7. `params/nav2_params.yaml` — controller gains need real-world tuning

The gains below were tuned in Gazebo.  They are a starting point only.

| Parameter | Current sim value | Notes |
|---|---|---|
| `LongStraightLine / x_vel` | `0.4 m/s` | Reduce for first hardware tests |
| `LongStraightLine / x_accel` | `0.4 m/s²` | |
| `LongStraightLine / z_vel` | `0.3 m/s` | |
| `LongStraightLine / z_accel` | `0.3 m/s²` | |
| `LongStraightLine / yaw_vel` | `0.25 rad/s` | |
| `LongStraightLine / lookahead_dist` | `1.0 m` | Increase for smoother path following |
| `goal_checker / xy_goal_tolerance` | `0.8 m` | Increase if DVL drift is large |
| `goal_checker / z_goal_tolerance` | `0.5 m` | |
| `SharpTurn / yaw_P_gain` | `0.02` | Marked as TODO — likely needs tuning |

---

## 8. Launch file delay timers

Both `sim_launch.py` and `hardware_launch.py` wrap the Nav2 + base controller
bringup in a `TimerAction`.

- **Simulation:** 3 s — waits for Gazebo `/clock` and TF from `odom_to_tf.py`.
- **Hardware (`hardware_launch.py`):** 5 s — gives the MAVLink bridge time to
  connect to ArduSub and receive the first heartbeat before Nav2 autostarts.

If the MAVLink bridge takes longer to connect (e.g., slow serial link), increase
this delay.  If Nav2 lifecycle nodes are stuck in `unconfigured` state at
startup, the timer is likely too short.

---

## 9. Static `map → odom` TF in `bringup.py` — hardware gotcha

`bringup.py` line 113–119 publishes a static identity `map → odom` TF when
`base:=False`.  This exists as a simulation convenience.

**This must never fire on hardware.**  `hardware_launch.py` always enables
the base controller and does not include the `UnlessCondition(base)` block,
so this static TF is never published.  If you ever call `bringup.py` directly
on hardware, pass `base:=True`.

---

## 10. Behaviour tree — hardware compatibility

The active BT (`orca4_bt.xml`) is hardware-compatible as-is:

```xml
<ReactiveFallback name="RecoveryFallback">
  <GoalUpdated/>
  <Wait wait_duration="2"/>
</ReactiveFallback>
```

Recovery is a 2-second hover/drift — safe for a real AUV.  There is no
costmap clearing or spin recovery (those would be dangerous underwater).

---

## 11. `base_controller` — must be re-added to CMakeLists.txt

The source file `orca_base/src/base_controller.cpp` exists but was removed
from `orca_base/CMakeLists.txt` (comment in `sim_orca_params.yaml`: *"dead
nodes for simulation-only setup"*).

For hardware it is **required** — it is the only node that:
- publishes the `odom → base_link` TF
- sends RC overrides (x/y/yaw thrust) to ArduSub
- forwards SLAM pose to ArduSub as `VISION_POSITION_ESTIMATE`

Add the following line to `orca_base/CMakeLists.txt` before
`ament_auto_package(...)`:

```cmake
ament_auto_add_executable(base_controller src/base_controller.cpp)
```

Then rebuild:

```bash
colcon build --packages-select orca_base
```

Then uncomment the `Node(package='orca_base', executable='base_controller', ...)`
block inside the `TimerAction` in `hardware_launch.py`.

---

## Quick checklist before first hardware run

- [ ] `ament_auto_add_executable(base_controller ...)` added to `orca_base/CMakeLists.txt`, workspace rebuilt
- [ ] `base_controller` Node uncommented in `hardware_launch.py`
- [ ] `MAVLINK_PUBLISHER_URL` and `MAVLINK_RECEIVER_URL` set in the shell
- [ ] Localization node uncommented and configured in `hardware_launch.py`
- [ ] `ros_gz_bridge` removed from `package.xml`, workspace rebuilt
- [ ] `sub.parm` verified against actual vehicle thruster wiring
- [ ] `use_sim_time` confirmed `False` in all running nodes (`ros2 param get <node> use_sim_time`)
- [ ] `/pixhawk/heartbeat` topic is publishing before starting navigation
- [ ] `/odom` topic is publishing from `base_controller`
- [ ] `map → odom → base_link` TF chain visible in `ros2 run tf2_tools view_frames`
- [ ] Start with reduced `x_vel` / `z_vel` (e.g., 0.2 m/s) for the first test
