"""
Manual control node for 6DOF joystick control of the submarine.

Subscribes to:
  - /joy (sensor_msgs/Joy)       : joystick input forwarded by foxglove_bridge
  - current_mode (std_msgs/String): active mode published by mode_control_node

Publishes:
  - pixhawk/manual_control (std_msgs/Int16MultiArray): 6-element array consumed
    by the mavlink_bridge ros2_receiver, which sends it as a MAVLink MANUAL_CONTROL
    message to the Pixhawk.

Only processes joystick input when current_mode == 'manual_control'.

Int16MultiArray layout (6 values):
  data[0] = x   (surge:  forward/back,  -1000 to 1000)
  data[1] = y   (sway:   lateral,       -1000 to 1000)
  data[2] = z   (heave:  throttle/depth, 0 to 1000, 500 = neutral)
  data[3] = r   (yaw:    rotation,      -1000 to 1000)
  data[4] = s   (roll:   MAVLink 2 extension, -1000 to 1000)
  data[5] = t   (pitch:  MAVLink 2 extension, -1000 to 1000)
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

        # Latest control values (neutral defaults), updated by joy_callback
        self.latest_msg = Int16MultiArray()
        self.latest_msg.data = [0, 0, 500, 0, 0, 0]

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

        # Timer to publish at a steady 20Hz rate
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('ManualControlNode: Node has been initialized')

    def mode_callback(self, msg):
        """Called when a new mode is published by mode_control_node."""
        self.current_mode = msg.data
        self.get_logger().info(f'Mode updated: {self.current_mode}')

    def joy_callback(self, msg):
        """Called when a joystick message arrives from /joy. Updates stored values."""
        if self.current_mode != 'manual_control':
            return

        self.latest_msg = self.map_joy_to_manual_control(msg)

    def timer_callback(self):
        """Publishes the latest control values at 20Hz."""
        if self.current_mode != 'manual_control':
            return

        self.manual_control_publisher.publish(self.latest_msg)

    def map_joy_to_manual_control(self, joy_msg):
        """
        Maps joystick axes/buttons to an Int16MultiArray for MANUAL_CONTROL.
        joy_msg.axes    -> list of floats (-1.0 to 1.0 for sticks, varies for triggers)
        joy_msg.buttons -> list of ints   (0 or 1)

        Returns Int16MultiArray with 6 values: [x, y, z, r, s, t]
        All axis values are int16: -1000 to 1000 (z: 0 to 1000, 500 = neutral)
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
