"""
Emergency stop node that disables all thruster output.

When the current mode (published by mode_control_node) is 'emergency_stop',
this node continuously publishes neutral values on pixhawk/manual_control,
which results in zero thrust on all axes. Joystick input is ignored entirely.

Subscribes to:
  - current_mode (std_msgs/String): active mode published by mode_control_node

Publishes:
  - pixhawk/manual_control (std_msgs/Int16MultiArray): 6-element array of
    neutral values [0, 0, 500, 0, 0, 0] to zero out all thrust.

Only publishes when current_mode == 'emergency_stop'.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray


class EmergencyStopModeNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_mode_node')

        self.current_mode = ''

        # Subscribe to the current mode published by mode_control_node
        self.mode_subscription = self.create_subscription(
            String,
            '/mode_control/current_mode',
            self.mode_callback,
            10,
        )

        # Publish neutral commands to the pixhawk via ros2_receiver
        self.manual_control_publisher = self.create_publisher(
            Int16MultiArray,
            '/pixhawk/manual_control',
            10,
        )

        # Timer to continuously publish neutral values while in emergency stop
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('EmergencyStopModeNode: Node has been initialized')

    def mode_callback(self, msg):
        """Called when a new mode is published by mode_control_node."""
        self.current_mode = msg.data
        self.get_logger().info(f'Mode updated: {self.current_mode}')

    def timer_callback(self):
        """Continuously publishes neutral values when in emergency_stop mode."""
        if self.current_mode != 'emergency_stop':
            return

        mc_msg = Int16MultiArray()
        mc_msg.data = [0, 0, 500, 0, 0, 0]  # Neutral values for surge, sway, heave, roll, pitch, yaw
        self.manual_control_publisher.publish(mc_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopModeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
