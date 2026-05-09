## Start simulation (for rosbag see below)
```bash
ros2 launch orca_bringup sim_launch.py
```

## Start mission
```bash
ros2 run orca_bringup WSG84_mission_starter.py
```

## Start current in m/s
`/orca_bringup/scripts/current_vector_node.py`
```bash
ros2 run orca_bringup current_vector_node.py --ros-args -p direction:="dir_x, xy, con_xy, ramp_xyz" -p amplitude:=0.5 -p period:=15.0 -p noise_stddev:=0.1 -p noise_time_constant:=1.5
```

## Start just mission and send cmd_vel
```bash
ros2 topic pub -1 /pixhawk/arm_cmd std_msgs/msg/Bool "{data: true}"
ros2 topic pub -1 /pixhawk/mode_cmd std_msgs/msg/String "{data: GUIDED}"
ros2 topic pub -r 20 /pixhawk/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Make step reponses from max distence of max. dist for replan
Change in `/orca_description/worlds/sand.world` the lines at pose ENU!. So for step would change the y value (2nd one!).
```bash
    <include>
      <uri>model://orca4</uri>
      <pose>0 0 -0.2 0 0 0</pose>
    </include>
```


## Tracking errors: rosbag → CSV (Docker vs host)

The Nav2 plugin `orca_nav2/PurePursuitController3D` can publish six diagnostics
(`std_msgs/msg/Float64`) while it is actively following a plan:

| Topic | Type | Meaning |
|-------|------|---------|
| `/pure_pursuit_cross_track_xy` | `std_msgs/Float64` | Horizontal distance to plan polyline (m) |
| `/pure_pursuit_vertical_error` | `std_msgs/Float64` | `robot_z - path_z` at closest XY point (m) |
| `/pure_pursuit_yaw_error` | `std_msgs/Float64` | Shortest angle path heading → robot yaw (rad, about `[-π, π]`) |
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

1. **Start rosbag manually**
  ```bash
   ros2 bag record --use-sim-time \
  /pure_pursuit_cross_track_xy \
  /pure_pursuit_vertical_error \
  /pure_pursuit_yaw_error \
  /pure_pursuit_closest_point_map \
  /pure_pursuit_robot_pose_map \
  /pure_pursuit_robot_twist \
  /ocean_current
   ```
   and then in new terminal:
   ```bash
   ros2 topic pub --once \
  --qos-durability transient_local \
  /ocean_current geometry_msgs/msg/Vector3 \
  "{x: 0.0, y: 0.0, z: 0.0}"
   ```
   Stop with in rosbag terminal
   ```bash
   Ctrl+C
   ```

2. **Run a mission** (second terminal **inside the same container**, with workspace sourced):
   ```bash
   ros2 run orca_bringup WSG84_mission_starter.py
   ```

4. **Stop cleanly:** Ctrl+C in the launch terminal; wait for `ros2 bag record` to exit.

5. **Find the bag** (created under the directory where you ran `ros2 launch`, usually `~/colcon_ws`):
   ```bash
   ls -lt (~/colcon_ws)
   ```
   Look for `rosbag2_YYYY_MM_DD-HH_MM_SS` with `metadata.yaml` inside.

8. **Export to CSV** (still **inside** the container):
   ```bash
   ros2 run orca_bringup export_tracking_bag_csv.py ~/colcon_ws/rosbag2_YYYY_MM_DD-HH_MM_SS
   ```
   Optional output directory:
   ```bash
   ros2 run orca_bringup export_tracking_bag_csv.py ~/colcon_ws/rosbag2_YYYY_MM_DD-HH_MM_SS \
     -o ~/colcon_ws/exports/my_run
   ```

**Exported files** (under `csv_export/` next to the bag, or `-o`):

   - **`tracking_errors_long.csv`** — all messages: `msg_type` is `float64`, `point`, `pose`, or `twist`; numeric fields in `v0`…`v12` (see `tracking_export_README.txt`).
   - **`tracking_errors_wide.csv`** — one row per cross-track time; columns include the three errors plus closest XYZ, robot position + quaternion, and twist linear/angular (aligned with as-of merge).
   - **`tracking_export_README.txt`** — column reference.

   Optional plot (needs matplotlib): add `--plot` to the `export_tracking_bag_csv.py` command.

---

### B. On the Linux host (copy results out of Docker)

`docker cp` must be run on the **host**, not inside the container (`docker: command not found` inside the container is expected).

1. **On the host**, open a terminal (e.g. `polaris_pz@…`, **not** `orca4@…`).

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
  You did not use `bag:=True`, or you launched from a **different** current directory—search:  
  `find ~ -maxdepth 3 -name metadata.yaml 2>/dev/null`

- **`ros2 bag info` shows 0 messages** on the tracking topics  
  Controller was not publishing (`publish_tracking_error: false`, or Nav2 not following a plan long enough).

- **`docker cp` fails**  
  Run it on the **host**; check container name, path inside container, and that the bag/export path exists (`docker exec orca4 ls /home/orca4/colcon_ws/...`).

- **ROS 1 `bagpy`**  
  Targets `.bag` files only; **not** ROS 2 rosbag2. Use `rosbag2_py` / `export_tracking_bag_csv.py` for folders with `metadata.yaml`.


