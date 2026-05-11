# config_pkg

## RecorderControllerNode

A ROS2 node that acts as a remote-controlled wrapper around `ros2 bag record`. Send it JSON commands over a topic to start/stop bag recordings and log timestamped events alongside them.

### Topics

| Topic | Direction | Description |
|---|---|---|
| `/polaris/recorder/command` | Subscribed | JSON command payloads |
| `/polaris/recorder/status` | Published (1 Hz) | Current recorder state |

Both topics are configurable via node parameters (`command_topic`, `status_topic`).

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `command_topic` | `/polaris/recorder/command` | Topic to receive commands on |
| `status_topic` | `/polaris/recorder/status` | Topic to publish status on |
| `base_output_dir` | `/ros2_ws/recordings/` | Root directory for bag output (mounted outside Docker) |
| `storage_id` | `mcap` | Bag storage format |

### Commands

Send a JSON payload to the command topic with a `"command"` field:

| Command | Description |
|---|---|
| `start_recording` | Spawns `ros2 bag record` as a subprocess; creates a timestamped output folder |
| `stop_recording` | Gracefully shuts down the bag process (SIGINT → SIGTERM → SIGKILL) |
| `add_instant_event` | Appends a one-shot timestamped entry to `events.jsonl` |
| `start_event` | Marks the start of a named event span in `events.jsonl` (one active at a time) |
| `stop_event` | Marks the end of the active event span |
| `set_metadata` | Updates the `name`, `testname`, and `location` fields used in the folder name |

Example payload:

```json
{
  "command": "start_recording",
  "metadata": {
    "name": "flight_test",
    "testname": "hover_stability",
    "location": "teststand_1"
  },
  "recording_options": {
    "mode": "all"
  }
}
```

### Recording Modes

Controlled via `recording_options.mode` in the command payload:

| Mode | Description |
|---|---|
| `all` (default) | Records every topic (`ros2 bag record -a`) |
| `selection` | Records only the topics listed in `include_topics` |
| `exclude_selection` | Records all topics except those in `exclude_topics` |

### Output Structure

The default output directory is `/ros2_ws/recordings/`, which is mounted as a volume outside the Docker container so recordings persist after the container stops.

Each recording produces a folder named `<name>__<testname>__<location>__<timestamp>`:

```
/ros2_ws/recordings/
└── flight_test__hover_stability__teststand_1__20260424_120000/
    ├── bag/                      # rosbag files
    ├── recording_metadata.json   # name, testname, location, created_at
    └── events.jsonl              # append-only log of events
```

`events.jsonl` entries look like:

```json
{"type": "instant", "timestamp": "2026-04-24T12:00:05+00:00", "Event": "liftoff"}
{"type": "start",   "timestamp": "2026-04-24T12:00:10+00:00", "Event": "hover_phase"}
{"type": "stop",    "timestamp": "2026-04-24T12:00:30+00:00", "Event": "hover_phase"}
```


### copy-paste autonomy TRUE
```bash
ros2 launch config_pkg start_system.launch.py autonomy:=true
```

That starts the autonomy process but nav2 is not active yet. When ready, activate it:

```bash
ros2 run orca_bringup nav2_activate.py
```
or manually

```bash
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"
```
run actually the mission:
```bash
ros2 run orca_bringup WSG84_mission_starter.py
```

# How to start docker (on PC at least)

```bash
docker compose up -d
```
```bash
docker logs jetson-container
```
```bash
docker ps
```
```bash
docker exec -it jetson-container bash
```

When launch terminal not stopping in a other terminal:
```bash
pkill -KILL -f "ros2 launch"
```

Then using colcon build and etc.

```bash
rosdep install --from-paths src/autonomy --ignore-src -r -y
colcon build --packages-up-to autonomy_bringup_pkg orca_nav2 --symlink-install
source install/setup.bash
```