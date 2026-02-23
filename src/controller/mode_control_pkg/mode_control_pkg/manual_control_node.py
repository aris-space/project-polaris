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
from config_pkg.constants import JoyControlMapping

# If no /joy message is received for this duration (seconds), send neutral values
JOY_TIMEOUT = 0.2


class ManualControlNode(Node):
    def __init__(self):
        super().__init__('manual_control_node')

        self.current_mode = ''

        # Neutral defaults for 6DOF
        self.neutral_msg = Int16MultiArray()
        self.neutral_msg.data = [0, 0, 500, 0, 0, 0]

        # Latest control values, updated by joy_callback
        self.latest_msg = Int16MultiArray()
        self.latest_msg.data = [0, 0, 500, 0, 0, 0]

        # Timestamp of last received /joy message
        self.last_joy_time = self.get_clock().now()

        # Joystick axis / button indices from config_pkg constants
        self.surge_axis = JoyControlMapping.LINEAR_SPEED_X_AXIS_IDX
        self.sway_axis = JoyControlMapping.LINEAR_SPEED_Y_AXIS_IDX
        self.yaw_axis = JoyControlMapping.YAW_RATE_AXIS_IDX
        self.pitch_axis = JoyControlMapping.PITCH_RATE_AXIS_IDX
        self.l2_axis = JoyControlMapping.LINEAR_SPEED_Z_FORWARD_AXIS_IDX
        self.r2_axis = JoyControlMapping.LINEAR_SPEED_Z_BACKWARD_AXIS_IDX
        self.roll_neg_button = JoyControlMapping.ROLL_RATE_NEGATIVE_AXIS_IDX
        self.roll_pos_button = JoyControlMapping.ROLL_RATE_POSITIVE_AXIS_IDX

        # Subscribe to the current mode published by mode_control_node
        self.mode_subscription = self.create_subscription(
            String,
            '/mode_control/current_mode',
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
            '/pixhawk/manual_control',
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
        """Called when a joystick message arrives from /joy. Updates stored values and timestamp."""
        if self.current_mode != 'manual_control':
            return

        self.last_joy_time = self.get_clock().now()
        self.latest_msg = self.map_joy_to_manual_control(msg)

    def timer_callback(self):
        """Publishes at 20Hz. Falls back to neutral if /joy times out."""
        if self.current_mode != 'manual_control':
            return

        elapsed = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if elapsed > JOY_TIMEOUT:
            self.manual_control_publisher.publish(self.neutral_msg)
        else:
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

        surge = joy_msg.axes[self.surge_axis]
        sway = joy_msg.axes[self.sway_axis]
        yaw = joy_msg.axes[self.yaw_axis]
        pitch = joy_msg.axes[self.pitch_axis]

        # Triggers: -1 unpressed, +1 pressed -> normalize to [0,1]
        l2_raw = joy_msg.axes[self.l2_axis]
        r2_raw = joy_msg.axes[self.r2_axis]
        l2 = (l2_raw + 1.0) * 0.5  # [0..1]
        r2 = (r2_raw + 1.0) * 0.5  # [0..1]

        # net vertical: + up, - down
        heave_net = r2 - l2  # [-1..1]

        # roll via L1/R1 buttons: +1 right, -1 left, 0 neither/both
        roll_neg = joy_msg.buttons[self.roll_neg_button]
        roll_pos = joy_msg.buttons[self.roll_pos_button]
        roll = float(roll_pos - roll_neg)  # [-1..1]

        #invert y and r to match the behavior of the PS4 controller
        x = int(surge * 1000)           # forward/back
        y = int(-sway * 1000)            # lateral
        z = int((heave_net + 1) * 500)  # throttle/depth (500 = neutral)
        r = int(-yaw * 1000)             # yaw
        s = int(roll * 1000)            # roll
        t = int(pitch * 1000)           # pitch

        mc_msg.data = [x, y, z, r, s, t]
        return mc_msg


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
