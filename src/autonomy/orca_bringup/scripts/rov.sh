# Set Pixhawk mode for manual/ROV control via custom MAVLink bridge.
ros2 topic pub --once /pixhawk/mode_cmd std_msgs/msg/String "{data: MANUAL}"
