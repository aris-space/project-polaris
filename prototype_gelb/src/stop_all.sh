#!/bin/bash

echo "Stopping joy_node..."
pkill -f "ros2 run joy joy_node" || true

echo "Stopping pwm_node.py..."
pkill -f "pwm_node.py" || true

echo "Stopping pigpiod..."
sudo pkill -f pigpiod || true

echo "All processes terminated."
