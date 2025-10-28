#!/bin/bash
set -euo pipefail

# Start pigpiod (only if you want it inside the container;
# if you run pigpiod on the Pi host, you can comment this out)
echo "Starting pigpiod..."
sudo pigpiod || true

# Start joy_node
echo "Starting joy_node..."
ros2 run joy joy_node &
JOY_PID=$!

# Start custom pwm_node
echo "Starting pwm_node..."
python3 /app/src/pwm_node.py &
PWM_PID=$!

echo "All nodes started. Press Ctrl+C to stop."

# Wait for either process to exit
wait -n

# If one dies, clean up the rest
kill $JOY_PID $PWM_PID 2>/dev/null || true
