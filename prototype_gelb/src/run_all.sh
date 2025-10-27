#!/bin/bash

# Load ROS2 environment in each terminal automatically
ROS_SETUP="source /opt/ros/jazzy/setup.bash;"

# Start joy_node in new terminal
gnome-terminal -- bash -c "$ROS_SETUP ros2 run joy joy_node; exec bash"
sleep 5
echo "Started joy_node."

# Start custom joy_mavros_node in new terminal
gnome-terminal -- bash -c "$ROS_SETUP python3 joy_mavros_node.py; exec bash"

# Start MAVROS in new terminal
gnome-terminal -- bash -c "$ROS_SETUP ros2 launch mavros apm.launch fcu_url:=udp://:14550@192.168.2.2:14555; exec bash"
sleep 10

echo "All nodes started."
echo "Arming the vehicle and setting to MANUAL mode..."

# Run service calls in new terminal (so you see feedback)
gnome-terminal -- bash -c "$ROS_SETUP ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'; exec bash"


echo "Vehicle armed and set to MANUAL mode."
echo "You can now control the vehicle using the joystick..."


'''
To run this bash file, you first need to make it executable with the command:
chmod +x run_all.sh
Then you can execute it with:
./run_all.sh

'''