# Recorder ROS2 Node (Temporary Location)

This folder intentionally keeps the ROS2 node next to the Foxglove extension for fast iteration.

## Package

- Package name: recorder_controller_node
- Node executable: recorder_controller
- Command topic: /polaris/recorder/command
- Status topic: /polaris/recorder/status

## Command Message Format

Publish std_msgs/msg/String with JSON in data:

{
  "command": "start_recording|stop_recording|add_instant_event|start_event|stop_event",
  "base_output_dir": "~/polaris_bags",
  "metadata": {
    "name": "mission_name",
    "testname": "tank_test_01",
    "location": "harbor"
  },
  "event_name": "optional_event_name",
  "source": "foxglove-panel",
  "timestamp": "2026-04-10T12:00:00Z"
}

## Status Message Format

Node publishes std_msgs/msg/String with JSON in data:

{
  "is_recording": true,
  "active_event": "maneuver_a",
  "recording_id": "mission__test__location__20260410_120000",
  "base_output_dir": "/home/user/polaris_bags",
  "output_path": "/home/user/polaris_bags/...",
  "started_at": "...",
  "metadata": {"name": "...", "testname": "...", "location": "..."},
  "last_error": null
}

## Build and Run

From your ROS2 workspace root (recommended final location), place or move this package into src, then:

1. colcon build --packages-select recorder_controller_node
2. source install/setup.bash
3. ros2 run recorder_controller_node recorder_controller

## Parameters

- command_topic (default: /polaris/recorder/command)
- status_topic (default: /polaris/recorder/status)
- base_output_dir (default: ~/polaris_bags)
