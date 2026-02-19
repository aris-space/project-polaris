"""
Manual altitude hold control node for 4DOF joystick control of the submarine.

The Pixhawk is set to DEPTH_HOLD mode by mode_control_node, which means the
Pixhawk auto-holds the current depth. The z axis here adjusts the depth target:
500 = hold current depth, >500 = ascend, <500 = descend. Roll and pitch are
auto-stabilized by the Pixhawk and not controlled by the joystick.

Subscribes to:
  - /joy (sensor_msgs/Joy)       : joystick input forwarded by foxglove_bridge
  - current_mode (std_msgs/String): active mode published by mode_control_node

Publishes:
  - pixhawk/manual_control (std_msgs/Int16MultiArray): 6-element array consumed
    by the mavlink_bridge ros2_receiver, which sends it as a MAVLink MANUAL_CONTROL
    message to the Pixhawk.

Only processes joystick input when current_mode == 'manual_depth_hold'.

Int16MultiArray layout (6 values, roll and pitch fixed at 0):
  data[0] = x   (surge:  forward/back,  -1000 to 1000)
  data[1] = y   (sway:   lateral,       -1000 to 1000)
  data[2] = z   (heave:  depth target,   0 to 1000, 500 = hold)
  data[3] = r   (yaw:    rotation,      -1000 to 1000)
  data[4] = s   (roll:   always 0, auto-stabilized by Pixhawk)
  data[5] = t   (pitch:  always 0, auto-stabilized by Pixhawk)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Joy
from config_pkg.constants import JoyControlMapping

# If no /joy message is received for this duration (seconds), send neutral values
JOY_TIMEOUT = 0.2


class ManualAltitudeHoldControlNode(Node):
    def __init__(self):
        super().__init__("manual_altitude_hold_control_node")

        self.current_mode = ""

        # Neutral defaults for 4DOF
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
        self.l2_axis = JoyControlMapping.LINEAR_SPEED_Z_FORWARD_AXIS_IDX
        self.r2_axis = JoyControlMapping.LINEAR_SPEED_Z_BACKWARD_AXIS_IDX

        # Subscribe to the current mode published by mode_control_node
        self.mode_subscription = self.create_subscription(
            String,
            "/mode_control/current_mode",
            self.mode_callback,
            10,
        )

        # Subscribe to joystick input from foxglove_bridge
        self.joy_subscription = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10,
        )

        # Publish manual control commands to the pixhawk via ros2_receiver
        self.manual_control_publisher = self.create_publisher(
            Int16MultiArray,
            "/pixhawk/manual_control",
            10,
        )

        # Timer to publish at a steady 20Hz rate
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            "ManualAltitudeHoldControlNode: Node has been initialized"
        )

    def mode_callback(self, msg):
        """Called when a new mode is published by mode_control_node."""
        self.current_mode = msg.data
        self.get_logger().info(f"Mode updated: {self.current_mode}")

    def joy_callback(self, msg):
        """Called when a joystick message arrives from /joy. Updates stored values and timestamp."""
        if self.current_mode != "manual_depth_hold":
            return

        self.last_joy_time = self.get_clock().now()
        self.latest_msg = self.map_joy_to_manual_control(msg)

    def timer_callback(self):
        """Publishes at 20Hz. Falls back to neutral if /joy times out."""
        if self.current_mode != "manual_depth_hold":
            return

        elapsed = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if elapsed > JOY_TIMEOUT:
            self.manual_control_publisher.publish(self.neutral_msg)
        else:
            self.manual_control_publisher.publish(self.latest_msg)

    def map_joy_to_manual_control(self, joy_msg):
        """
        Maps joystick axes/buttons to an Int16MultiArray for MANUAL_CONTROL in DEPTH_HOLD mode.
        joy_msg.axes    -> list of floats (-1.0 to 1.0 for sticks, varies for triggers)
        joy_msg.buttons -> list of ints   (0 or 1)

        Returns Int16MultiArray with 4 values: [x, y, z, r]
        All axis values are int16: -1000 to 1000 (z: 0 to 1000, 500 = hold depth)
        """
        mc_msg = Int16MultiArray()

        surge = joy_msg.axes[self.surge_axis]
        sway = joy_msg.axes[self.sway_axis]
        yaw = joy_msg.axes[self.yaw_axis]

        # Triggers: commonly +1 unpressed, -1 pressed -> normalize to [0,1]
        l2_raw = joy_msg.axes[self.l2_axis]
        r2_raw = joy_msg.axes[self.r2_axis]
        l2 = (1.0 - l2_raw) * 0.5  # [0..1]
        r2 = (1.0 - r2_raw) * 0.5  # [0..1]

        # net vertical: + up, - down
        heave_net = r2 - l2  # [-1..1]

        x = int(surge * 1000)  # surge: forward/back
        y = int(sway * 1000)  # sway: lateral
        z = int((heave_net + 1) * 500)  # heave: depth target (500 = hold current depth)
        r = int(yaw * 1000)  # yaw: rotation

        self.get_logger().debug(
            f"Mapped joy axes to manual control: surge={surge:.2f}, sway={sway:.2f}, yaw={yaw:.2f}, l2={l2:.2f}, r2={r2:.2f} -> x={x}, y={y}, z={z}, r={r}"
        )
        mc_msg.data = [
            x,
            y,
            z,
            r,
            0,
            0,
        ]  # Extend to 6 values for compatibility with 6DOF (roll and pitch = 0)
        return mc_msg


def main(args=None):
    rclpy.init(args=args)
    node = ManualAltitudeHoldControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
