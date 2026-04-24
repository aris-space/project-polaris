# Recorder ROS2 Node (Migrated)

The active recorder controller node now lives in the main ROS workspace package.

## Active Location

- Package: config_pkg
- Source: src/config/config_pkg/config_pkg/recorder_controller.py
- Executable: recorder_controller
- Command topic: /polaris/recorder/command
- Status topic: /polaris/recorder/status

## Run

1. colcon build --packages-select config_pkg
2. source install/setup.bash
3. ros2 run config_pkg recorder_controller

## Parameters

- command_topic (default: /polaris/recorder/command)
- status_topic (default: /polaris/recorder/status)
- base_output_dir (default: /ros2_ws/recordings/)
- storage_id (default: mcap)

## Note

This folder is kept for historical reference only.
