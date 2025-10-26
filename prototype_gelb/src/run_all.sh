#!/bin/bash

# Load ROS2 environment
source /opt/ros/jazzy/setup.bash

# Start nodes
ros2 run joy joy_node &
sleep 5  # wait briefly for the node to start
echo "Started joy_node."

python3 joy_mavros_node.py &
# Start MAVROS
ros2 launch mavros apm.launch fcu_url:=udp://:14550@192.168.2.2:14555 &
# wait a bit longer for MAVROS to fully start
sleep 10

echo "All nodes started."
echo "Arming the vehicle and setting to MANUAL mode..."

# Call services to arm and set MANUAL mode
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'MANUAL'}"

echo "Vehicle armed and set to MANUAL mode."
echo "You can now control the vehicle using the joystick..."


'''
To run this bash file, you first need to make it executable with the command:
chmod +x run_all.sh
Then you can execute it with:
./run_all.sh

'''