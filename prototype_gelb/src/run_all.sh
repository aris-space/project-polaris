#!/bin/bash

# Load ROS2 environment in each terminal automatically
ROS_SETUP="source /opt/ros/jazzy/setup.bash;"


sudo apt install pigpio python3-pigpio

# Start joy_node in new terminal
gnome-terminal -- bash -c "$ROS_SETUP ros2 run joy joy_node; exec bash"
sleep 5
echo "Started joy_node."

# Start custom pwm_node in new terminal
gnome-terminal -- bash -c "$ROS_SETUP sudo pigpiod; exec bash"
gnome-terminal -- bash -c "$ROS_SETUP python3 pwm_node.py; exec bash"



echo "All nodes started."
echo "You can now control the vehicle using the joystick..."


'''
To run this bash file, you first need to make it executable with the command:
chmod +x run_all.sh
Then you can execute it with:
./run_all.sh

'''