#!/bin/bash

# Stop all ROS2 and MAVROS related processes

echo "Stopping joy_node..."
pkill -f "ros2 run joy joy_node"

echo "Stopping pwm_node.py..."
pkill -f "pwm_node.py"

sleep 1

echo "Stopping pigpiod..."
sudo service pigpiod stop
sleep 1
sudo pkill -f pigpiod

echo "All processes terminated."

'''
To run this bash file, you first need to make it executable with the command:
chmod +x stop_all.sh
Then you can execute it with:
./stop_all.sh

'''