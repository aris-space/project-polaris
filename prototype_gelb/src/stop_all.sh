#!/bin/bash

# Stop all ROS2 and MAVROS related processes

echo "Stopping joy_node..."
pkill -f "ros2 run joy joy_node"

echo "Stopping joy_mavros_node.py..."
pkill -f "joy_mavros_node.py"

echo "Stopping MAVROS..."
pkill -f "ros2 launch mavros"

sleep 1

echo "Checking for remaining ROS2 or MAVROS processes..."
ps aux | grep -E "ros2|mavros|python3"

echo "All processes terminated."

'''
To run this bash file, you first need to make it executable with the command:
chmod +x stop_all.sh
Then you can execute it with:
./stop_all.sh

'''