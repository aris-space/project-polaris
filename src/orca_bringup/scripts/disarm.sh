# Disarm Pixhawk via custom MAVLink bridge.
ros2 topic pub --once /pixhawk/arm_cmd std_msgs/msg/Bool "{data: false}"
