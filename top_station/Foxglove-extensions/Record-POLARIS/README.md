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

## ROS2 Node (Temporary Co-Located)

The matching ROS2 node is currently in:

RECORDER_ROS_NODE/recorder_controller_node

You can keep it here while developing and move it into your ROS2 src folder later.
The panel-node contract is topic-based JSON, so moving the node will not require panel refactoring.
