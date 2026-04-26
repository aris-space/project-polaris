# Record-POLARIS

Foxglove extension panel for controlling a rosbag2 recorder controller node.

## Implemented Panel Features

- Start recording
- Stop recording
- Recording status indicator in panel
- Add instant event
- Start long-duration event
- Stop long-duration event
- Metadata inputs: name, testname, location

## Topics

- Command topic: /polaris/recorder/command
- Status topic: /polaris/recorder/status

Both are configurable from panel settings.

## Local Development

1. npm install
2. npm run build
3. npm run local-install

Panel name in Foxglove:

Recorder Controller (POLARIS)

## ROS2 Node

The matching ROS 2 node is in:

src/config/config_pkg/config_pkg/recorder_controller.py

Run it from the ROS package as:

ros2 run config_pkg recorder_controller

The panel-node contract is topic-based JSON, so your colleagues can import the node into their ROS 2 launch/code without refactoring the panel.
