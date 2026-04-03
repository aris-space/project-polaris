## Launch files

### [sim_launch.py](launch/sim_launch.py)

Launch ROV or AUV simulation in Gazebo.
Calls [bringup.py](launch/bringup.py).

To see parameters: `ros2 launch --show-args orca_bringup sim_launch.py`

### [bringup.py](launch/bringup.py)

Bring up all core ROV and AUV nodes, including ORB_SLAM2 and Nav2.
Calls [navigation_launch.py](launch/navigation_launch.py).

### [navigation_launch.py](launch/navigation_launch.py)

Nav2 navigation launch file, modified to avoid launch the velocity smoother.

## Scenarios

### Full automation

In a terminal run:
~~~
source src/orca4/setup.bash
ros2 launch orca_bringup sim_launch.py
~~~

Execute a mission in a 2nd terminal:
~~~
source src/orca4/setup.bash
ros2 run orca_bringup mission_runner.py
~~~

### Using MAVProxy

It is possible to launch Gazebo and ArduSub and control the sub using MAVProxy.

Launch a minimal system:
~~~
ros2 launch orca_bringup sim_launch.py base:=false comms:=false nav:=false rviz:=false
~~~

Launch MAVProxy in a 2nd terminal:
~~~
mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out 127.0.0.1:14550 --out 127.0.0.1:14551 --out udp:0.0.0.0:14550 --console
~~~

You can use MAVProxy to send commands directly to ArduSub:
~~~
arm throttle
rc 3 1450
rc 3 1500
mode alt_hold
disarm
~~~

RC channels:
* RC 3 -- vertical
* RC 4 -- yaw
* RC 5 -- forward
* RC 6 -- strafe

### MAVProxy + SLAM

This will bring up a minimal system with SLAM and RViz:
~~~
ros2 launch orca_bringup sim_launch.py base:=false comms:=false nav:=false
~~~

You can use MAVProxy to drive the sub around the seafloor and build a map.

## Tracking errors: rosbag → CSV (Docker vs host)

The Nav2 plugin `orca_nav2/PurePursuitController3D` can publish three diagnostics
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

7. **Exported files** (under `csv_export/` next to the bag, or `-o`):

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

No ROS 2 installation is required on that machine.

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


