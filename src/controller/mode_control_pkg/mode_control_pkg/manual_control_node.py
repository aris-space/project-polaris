"""
Manual control node that translates joystick inputs from Foxglove (/joy topic)
into a MANUAL_CONTROL message for the Pixhawk (pixhawk/manual_control topic).
Only active when the current mode (published by mode_control_node) is 'manual_control'.

The Int16MultiArray published on pixhawk/manual_control has the following layout:
  data[0] = x        (forward/back,  -1000 to 1000)
  data[1] = y        (lateral,       -1000 to 1000)
  data[2] = z        (throttle/depth, 0 to 1000)
  data[3] = r        (yaw,           -1000 to 1000)
  data[4] = s        (roll,          -1000 to 1000)
  data[5] = t        (pitch,         -1000 to 1000)
  data[6] = buttons  (bitmask)
  data[7] = buttons2 (bitmask)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Joy
from config.config import Config


class ManualControlNode(Node):
    def __init__(self):
        super().__init__('manual_control_node')

        self.current_mode = ''

        # Joystick axis / button indices from config
        self.left_stick_horizontal_axis = Config.get_joy_left_stick_horizontal_axis()
        self.left_stick_vertical_axis = Config.get_joy_left_stick_vertical_axis()
        self.right_stick_horizontal_axis = Config.get_joy_right_stick_horizontal_axis()
        self.right_stick_vertical_axis = Config.get_joy_right_stick_vertical_axis()
        self.l2_axis = Config.get_joy_l2_axis()
        self.r2_axis = Config.get_joy_r2_axis()
        self.l1_button = Config.get_joy_l1_button()
        self.r1_button = Config.get_joy_r1_button()

        # Subscribe to the current mode published by mode_control_node
        self.mode_subscription = self.create_subscription(
            String,
            'current_mode',
            self.mode_callback,
            10,
        )

        # Subscribe to joystick input from foxglove_bridge
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10,
        )

        # Publish manual control commands to the pixhawk via ros2_receiver
        self.manual_control_publisher = self.create_publisher(
            Int16MultiArray,
            'pixhawk/manual_control',
            10,
        )

        self.get_logger().info('ManualControlNode: Node has been initialized')

    def mode_callback(self, msg):
        """Called when a new mode is published by mode_control_node."""
        self.current_mode = msg.data
        self.get_logger().info(f'Mode updated: {self.current_mode}')

    def joy_callback(self, msg):
        """Called when a joystick message arrives from /joy. Only processes if mode is manual_control."""
        if self.current_mode != 'manual_control':
            return

        manual_control_msg = self.map_joy_to_manual_control(msg)
        self.manual_control_publisher.publish(manual_control_msg)

    def map_joy_to_manual_control(self, joy_msg):
        """
        Maps joystick axes/buttons to an Int16MultiArray for MANUAL_CONTROL.
        joy_msg.axes    -> list of floats (-1.0 to 1.0 for sticks, varies for triggers)
        joy_msg.buttons -> list of ints   (0 or 1)

        Must return Int16MultiArray with 8 values: [x, y, z, r, s, t, buttons, buttons2]
        All axis values are int16: -1000 to 1000 (z: 0 to 1000)
        """
        mc_msg = Int16MultiArray()

        lx = joy_msg.axes[self.left_stick_horizontal_axis]  # left stick X
        ly = joy_msg.axes[self.left_stick_vertical_axis]    # left stick Y
        rx = joy_msg.axes[self.right_stick_horizontal_axis] # right stick X
        ry = joy_msg.axes[self.right_stick_vertical_axis] # right stick Y

        # Triggers: commonly +1 unpressed, -1 pressed -> normalize to [0,1]
        l2_raw = joy_msg.axes[self.l2_axis]
        r2_raw = joy_msg.axes[self.r2_axis]
        l2 = (1.0 - l2_raw) * 0.5  # [0..1]
        r2 = (1.0 - r2_raw) * 0.5  # [0..1]

        # net vertical: + up, - down 
        net = r2 - l2  # [-1..1]

        # roll: + right, - left 
        roll = rx - lx  # [-1..1]

        # TODO: map joy_msg.axes / joy_msg.buttons to the 6 axes + button bitmasks
        x = int(ly * 1000)        # forward/back
        y = int(lx * 1000)        # lateral
        z = int((net+1)*500)      # throttle/depth (neutral)
        r = int(rx * 1000)        # yaw
        s = int(roll * 1000)        # roll
        t = int(ry * 1000)        # pitch


        mc_msg.data = [x, y, z, r, s, t]
        return mc_msg


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
