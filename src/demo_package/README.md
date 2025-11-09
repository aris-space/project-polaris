# demo_package

Minimal ROS2 Python demo package providing a talker and listener node.

How to try locally (assuming ROS2 environment is sourced):

1. Build the workspace:

```bash
colcon build --symlink-install
```

2. Source the workspace and run nodes in separate terminals:

```bash
. install/setup.bash
ros2 run demo_package talker
ros2 run demo_package listener
```
