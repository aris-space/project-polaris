# autonomy_bringup_pkg

> **Note on this file.** Most of it was written for the simulation setup, where this
> package was called `orca_bringup`. In this repository the package is
> `autonomy_bringup_pkg`, the executables have no `.py` suffix (see `setup.py`), and
> `sim_launch.py` and `current_vector_node.py` do not exist here. The commands in the
> next section are the ones that work on the vehicle; the sections further down that
> mention `sim_launch.py` or the Orca4 Docker paths describe the simulation setup.

## On the vehicle

```bash
# Nav2 stack (not activated yet), as part of the whole system:
ros2 launch config_pkg start_system.launch.py autonomy:=true

# Activate / deactivate Nav2
ros2 run autonomy_bringup_pkg nav2_activate
ros2 run autonomy_bringup_pkg nav2_deactivate

# Whole mission from the loaded waypoint file
ros2 run autonomy_bringup_pkg wgs84_mission_starter

# One waypoint, then hold on station (armed + GUIDED)
ros2 run autonomy_bringup_pkg go_to_waypoint --index 1

# Capture the current position as a waypoint (surface, RTK)
ros2 run autonomy_bringup_pkg capture_waypoint --out <file>.json
```

Both mission tools report when Nav2 gave up on a waypoint ("NOT reached"): the
waypoint follower runs with `stop_on_failure: False`, so the action itself still ends
as "succeeded" in that case.

## Start simulation (for rosbag see below) — simulation setup only
```bash
ros2 launch orca_bringup sim_launch.py (bag:=True)
```

## Start mission
```bash
ros2 run autonomy_bringup_pkg wgs84_mission_starter
```

## Start current in m/s — simulation setup only
`/orca_bringup/scripts/current_vector_node.py`
```bash
ros2 run orca_bringup current_vector_node.py --ros-args -p direction:="x, con_xy, ramp_xyz" -p amplitude:=0.8 -p period:=15.0
```

## Start just mission and send cmd_vel
```bash
ros2 topic pub -1 /pixhawk/arm_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub -1 /pixhawk/mode_cmd std_msgs/msg/String "{data: GUIDED}"
ros2 topic pub -r 20 /pixhawk/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Tracking errors: rosbag -> CSV (Docker vs host)

The Nav2 plugin `orca_nav2/PurePursuitController3D` can publish six diagnostics
(`std_msgs/msg/Float64`) while it is actively following a plan:

| Topic | Type | Meaning |
|-------|------|---------|
| `/pure_pursuit_cross_track_xy` | `std_msgs/Float64` | Horizontal distance to plan polyline (m) |
| `/pure_pursuit_vertical_error` | `std_msgs/Float64` | `robot_z - path_z` at closest XY point (m) |
| `/pure_pursuit_yaw_error` | `std_msgs/Float64` | Shortest angle path heading -> robot yaw (rad, about `[-π, π]`) |
| `/pure_pursuit_closest_point_map` | `geometry_msgs/PointStamped` | Closest point on path in **map** (header `frame_id` = plan frame) |
| `/pure_pursuit_robot_pose_map` | `geometry_msgs/PoseStamped` | Robot pose in **map** |
| `/pure_pursuit_robot_twist` | `geometry_msgs/TwistStamped` | Twist Nav2 passes in (usually from odometry; header frame matches robot pose frame) |

Toggle with `publish_tracking_error` in [`params/nav2_params.yaml`](params/nav2_params.yaml) (e.g. under `LongStraightLine`).

[`sim_launch.py`](launch/sim_launch.py) with `bag:=True` records **only** these six topics (no `/tf`, `/odom`, etc.).

**Important:** Paths below assume a typical **Orca4 Docker** layout: user `orca4`, workspace `/home/orca4/colcon_ws`.

### A. Inside the Docker container (step by step)

**Important:** Paths below use a typical **Orca4 Docker** layout: user `orca4`, workspace
`/home/orca4/colcon_ws`. If your image uses a different user or mount, replace those paths.
``
0. start docker as usual

1. **Start simulation with bag recording**  
   [`sim_launch.py`](launch/sim_launch.py) with `bag:=True` starts `ros2 bag record` for **only** the three topics above (no `/tf`, `/odom`, etc.):
   ```bash
   ros2 launch orca_bringup sim_launch.py bag:=True
   ```
   Leave this running in that terminal.

2. **Run a mission** (second terminal **inside the same container**, with workspace sourced):
   ```bash
   ros2 run autonomy_bringup_pkg wgs84_mission_starter
   ```

4. **Stop cleanly:** Ctrl+C in the launch terminal; wait for `ros2 bag record` to exit.

5. **Find the bag** (created under the directory where you ran `ros2 launch`, usually `~/colcon_ws`):
   ```bash
   ls -lt (~/colcon_ws)
   ```
   Look for `rosbag2_YYYY_MM_DD-HH_MM_SS` with `metadata.yaml` inside.

8. **Export to CSV** (still **inside** the container):
   ```bash
   ros2 run autonomy_bringup_pkg export_tracking_bag_csv ~/colcon_ws/rosbag2_YYYY_MM_DD-HH_MM_SS
   ```
   Optional output directory:
   ```bash
   ros2 run autonomy_bringup_pkg export_tracking_bag_csv ~/colcon_ws/rosbag2_YYYY_MM_DD-HH_MM_SS \
     -o ~/colcon_ws/exports/my_run
   ```

7. **Exported files** (under `csv_export/` next to the bag, or `-o`):

   - **`tracking_errors_long.csv`** - all messages: `msg_type` is `float64`, `point`, `pose`, or `twist`; numeric fields in `v0`...`v12` (see `tracking_export_README.txt`).
   - **`tracking_errors_wide.csv`** - one row per cross-track time; columns include the three errors plus closest XYZ, robot position + quaternion, and twist linear/angular (aligned with as-of merge).
   - **`tracking_export_README.txt`** - column reference.

   Optional plot (needs matplotlib): add `--plot` to the `export_tracking_bag_csv.py` command.

---

### B. On the Linux host (copy results out of Docker)

`docker cp` must be run on the **host**, not inside the container (`docker: command not found` inside the container is expected).

1. **On the host**, open a terminal (e.g. `polaris_pz@...`, **not** `orca4@...`).

2. **Confirm the container name**:
   ```bash
   docker ps
   ```
   Example: `NAMES` column shows `orca4`, or use the `CONTAINER ID`.

3. **Copy the `csv_export` folder** (adjust bag name to match yours):
   ```bash
   docker cp orca4:/home/orca4/colcon_ws/rosbag2_2026_03_28-18_07_19/csv_export \
     ~/Downloads/orca_tracking_csv_export_NAME
   ```
   Or into your host project:
   ```bash
   mkdir -p ~/project-polaris-simulation-personal/analysis
   docker cp orca4:/home/orca4/colcon_ws/rosbag2_2026_03_28-18_07_19/csv_export \
     ~/project-polaris-simulation-personal/analysis/run_180719
   ```

4. **Optional:** copy the **entire** bag if you want to keep the rosbag2 database on the host:
   ```bash
   docker cp orca4:/home/orca4/colcon_ws/rosbag2_2026_03_28-18_07_19 \
     ~/Downloads/rosbag2_2026_03_28-18_07_19
   ```
---

### C. Analysis without ROS (host or other repo)

After you have `csv_export` on the host:

```bash
ls ~/Downloads/orca_tracking_csv_export
```

Example in Python:

```python
import pandas as pd
df = pd.read_csv("tracking_errors_wide.csv")
print(df.describe())
```

---

### Troubleshooting

- **No `rosbag2_*` folder in `ls`**  
  You did not use `bag:=True`, or you launched from a **different** current directory-search:  
  `find ~ -maxdepth 3 -name metadata.yaml 2>/dev/null`

- **`ros2 bag info` shows 0 messages** on the tracking topics  
  Controller was not publishing (`publish_tracking_error: false`, or Nav2 not following a plan long enough).

- **`docker cp` fails**  
  Run it on the **host**; check container name, path inside container, and that the bag/export path exists (`docker exec orca4 ls /home/orca4/colcon_ws/...`).

- **ROS 1 `bagpy`**  
  Targets `.bag` files only; **not** ROS 2 rosbag2. Use `rosbag2_py` / `export_tracking_bag_csv.py` for folders with `metadata.yaml`.


