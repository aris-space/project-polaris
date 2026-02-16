"""
This node should act as the logic for switching between different modes of operation for the robot. It will subscribe to the topics published by the foxglove_bridge
and determine which mode the robot should be in based on the incoming data. It will then publish the current mode to a topic and redirects the control commands to the appropriate topics for the current mode. 
The modes include manual control, manual depth hold, emergency stop and later also the autonomous modes.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import UInt8

class ModeControlNode(Node):
    def __init__(self):
        super().__init__('mode_control_node')
        self.current_mode = 'manual_control'  # Default mode
        self.mode_publisher = self.create_publisher(String, 'current_mode', 10)

    def command_callback(self, msg):
        command = msg.data
        if self.safety_button_pressed():
            if command == '0':
                self.current_mode = 'emergency_stop'
            elif command == '1':
                self.current_mode = 'manual_control'
            elif command == '2':
                self.current_mode = 'manual_depth_hold' 

        # Add more conditions for other modes as needed

        # Publish the current mode
        mode_msg = String()
        mode_msg.data = self.current_mode
        self.mode_publisher.publish(mode_msg)
    
    def safety_button_pressed(self):
        # This function should check the state of the safety button
        # For now, we will just return True to allow mode switching

        #TODO
        return True