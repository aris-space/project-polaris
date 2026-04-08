"""
Manual position hold control node for 6DOF joystick control of the submarine.

The Pixhawk is set to POSHOLD mode by mode_control_node. In this mode, the
vehicle holds position when joystick inputs are neutral, and accepts manual
6DOF command inputs while the sticks/triggers/buttons are actuated.

Subscribes to:
  - /joy (sensor_msgs/Joy)        : joystick input forwarded by foxglove_bridge
  - /mode_control/current_mode (std_msgs/String): active mode published by mode_control_node

Publishes:
  - /pixhawk/manual_control (std_msgs/Int16MultiArray): 6-element array consumed
    by the mavlink_bridge ros2_receiver, which sends it as a MAVLink MANUAL_CONTROL
    message to the Pixhawk.

Only processes joystick input when current_mode == 'manual_position_hold'.

Int16MultiArray layout (6 values):
  data[0] = x   (surge:  forward/back,  -1000 to 1000)
  data[1] = y   (sway:   lateral,       -1000 to 1000)
  data[2] = z   (heave:  throttle/depth, 0 to 1000, 500 = neutral)
  data[3] = r   (yaw:    rotation,      -1000 to 1000)
  data[4] = s   (roll:   MAVLink 2 extension, -1000 to 1000)
  data[5] = t   (pitch:  MAVLink 2 extension, -1000 to 1000)


TODO: - For poshold the pixhawk needs some position reference to hold. Maybe feeding the dvl straight or feeding odometry values from the EKF.

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Joy
from config_pkg.constants import JoyControlMapping

# If no /joy message is received for this duration (seconds), send neutral values
JOY_TIMEOUT = 0.2


class ManualPositionHoldNode(Node):
    def __init__(self):
        super().__init__("manual_position_hold_node")

        # Input-source tags are provided by joy_handler_node in Joy.header.frame_id.
        self.declare_parameter("keyboard_source_frame_id", "keyboard")
        self.declare_parameter("controller_source_frame_id", "controller")

        # Source-specific gains (live-tunable via ROS params).
        self.declare_parameter("controller_gain_x", 1000.0)
        self.declare_parameter("controller_gain_y", 500.0)
        self.declare_parameter("controller_gain_z", 250.0)
        self.declare_parameter("controller_gain_r", 500.0)
        self.declare_parameter("controller_gain_s", 300.0)
        self.declare_parameter("controller_gain_t", 300.0)
        self.declare_parameter("controller_axis_deadzone", 0.05)

        self.declare_parameter("keyboard_gain_x", 1000.0)
        self.declare_parameter("keyboard_gain_y", 500.0)
        self.declare_parameter("keyboard_gain_z", 250.0)
        self.declare_parameter("keyboard_gain_r", 500.0)
        self.declare_parameter("keyboard_gain_s", 300.0)
        self.declare_parameter("keyboard_gain_t", 300.0)
        self.declare_parameter("keyboard_x_single_press_gain", 500.0)
        self.declare_parameter("keyboard_x_double_press_gain", 1000.0)
        self.declare_parameter("keyboard_x_double_press_window_s", 0.2)

        self.current_mode = ""
        self.keyboard_x_prev_pressed = False
        self.keyboard_x_last_press_time = None
        self.keyboard_x_last_press_direction = 0
        self.keyboard_x_boost_active = False

        self.neutral_msg = Int16MultiArray()
        self.neutral_msg.data = [0, 0, 500, 0, 0, 0]

        self.latest_msg = Int16MultiArray()
        self.latest_msg.data = [0, 0, 500, 0, 0, 0]

        self.last_joy_time = self.get_clock().now()

        self.surge_axis = JoyControlMapping.LINEAR_SPEED_X_AXIS_IDX
        self.sway_axis = JoyControlMapping.LINEAR_SPEED_Y_AXIS_IDX
        self.yaw_axis = JoyControlMapping.YAW_RATE_AXIS_IDX
        self.pitch_axis = JoyControlMapping.PITCH_RATE_AXIS_IDX
        self.l2_axis = JoyControlMapping.LINEAR_SPEED_Z_FORWARD_AXIS_IDX
        self.r2_axis = JoyControlMapping.LINEAR_SPEED_Z_BACKWARD_AXIS_IDX
        self.roll_neg_button = JoyControlMapping.ROLL_RATE_NEGATIVE_AXIS_IDX
        self.roll_pos_button = JoyControlMapping.ROLL_RATE_POSITIVE_AXIS_IDX

        self.mode_subscription = self.create_subscription(
            String,
            "/mode_control/current_mode",
            self.mode_callback,
            10,
        )

        self.joy_subscription = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10,
        )

        self.manual_control_publisher = self.create_publisher(
            Int16MultiArray,
            "/pixhawk/manual_control",
            10,
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("ManualPositionHoldNode: Node has been initialized")

    def mode_callback(self, msg):
        self.current_mode = msg.data
        self.get_logger().info(f"Mode updated: {self.current_mode}")

    def joy_callback(self, msg):
        if self.current_mode != "manual_position_hold":
            return

        self.last_joy_time = self.get_clock().now()
        self.latest_msg = self.map_joy_to_manual_control(msg)

    def timer_callback(self):
        if self.current_mode != "manual_position_hold":
            return

        elapsed = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        if elapsed > JOY_TIMEOUT:
            self.manual_control_publisher.publish(self.neutral_msg)
        else:
            self.manual_control_publisher.publish(self.latest_msg)

    def map_joy_to_manual_control(self, joy_msg):
        mc_msg = Int16MultiArray()

        surge = joy_msg.axes[self.surge_axis]
        sway = joy_msg.axes[self.sway_axis]
        yaw = joy_msg.axes[self.yaw_axis]
        pitch = joy_msg.axes[self.pitch_axis]

        # Triggers: -1 unpressed, +1 pressed -> normalize to [0,1]
        l2_raw = joy_msg.axes[self.l2_axis]
        r2_raw = joy_msg.axes[self.r2_axis]
        l2 = (l2_raw + 1.0) * 0.5
        r2 = (r2_raw + 1.0) * 0.5

        # net vertical: + up, - down
        heave_net = r2 - l2

        # roll via L1/R1 buttons: +1 right, -1 left, 0 neither/both
        roll_neg = joy_msg.buttons[self.roll_neg_button]
        roll_pos = joy_msg.buttons[self.roll_pos_button]
        roll = float(roll_pos - roll_neg)

        gain_prefix = "keyboard" if self._is_keyboard_source(joy_msg) else "controller"
        if gain_prefix == "controller":
            deadzone = (
                self.get_parameter("controller_axis_deadzone")
                .get_parameter_value()
                .double_value
            )
            surge = self._apply_deadzone(surge, deadzone)
            sway = self._apply_deadzone(sway, deadzone)
            yaw = self._apply_deadzone(yaw, deadzone)
            pitch = self._apply_deadzone(pitch, deadzone)
            heave_net = self._apply_deadzone(heave_net, deadzone)

        gain_x = self._get_gain(gain_prefix, "x")
        gain_y = self._get_gain(gain_prefix, "y")
        gain_z = self._get_gain(gain_prefix, "z")
        gain_r = self._get_gain(gain_prefix, "r")
        gain_s = self._get_gain(gain_prefix, "s")
        gain_t = self._get_gain(gain_prefix, "t")
        if gain_prefix == "keyboard":
            gain_x = self._get_keyboard_x_gain(surge)

        x = self._clamp_int(surge * gain_x, -1000, 1000)
        y = self._clamp_int(-sway * gain_y, -1000, 1000)
        z = self._clamp_int(500.0 + heave_net * gain_z, 0, 1000)
        r = self._clamp_int(-yaw * gain_r, -1000, 1000)
        s = self._clamp_int(roll * gain_s, -1000, 1000)
        t = self._clamp_int(pitch * gain_t, -1000, 1000)

        mc_msg.data = [x, y, z, r, s, t]
        return mc_msg

    def _is_keyboard_source(self, joy_msg):
        frame_id = joy_msg.header.frame_id.strip().lower()
        keyboard_frame_id = (
            self.get_parameter("keyboard_source_frame_id")
            .get_parameter_value()
            .string_value.strip()
            .lower()
        )
        return frame_id == keyboard_frame_id

    def _get_gain(self, gain_prefix, axis):
        param_name = f"{gain_prefix}_gain_{axis}"
        return self.get_parameter(param_name).get_parameter_value().double_value

    def _get_keyboard_x_gain(self, surge):
        is_pressed = abs(surge) > 0.5
        direction = 1 if surge > 0.0 else (-1 if surge < 0.0 else 0)

        if is_pressed and not self.keyboard_x_prev_pressed:
            now = self.get_clock().now()
            window_s = (
                self.get_parameter("keyboard_x_double_press_window_s")
                .get_parameter_value()
                .double_value
            )
            within_window = False
            if self.keyboard_x_last_press_time is not None:
                dt_s = (now - self.keyboard_x_last_press_time).nanoseconds / 1e9
                within_window = dt_s <= window_s
            self.keyboard_x_boost_active = (
                within_window and direction == self.keyboard_x_last_press_direction
            )
            self.keyboard_x_last_press_time = now
            self.keyboard_x_last_press_direction = direction

        if not is_pressed:
            self.keyboard_x_boost_active = False

        self.keyboard_x_prev_pressed = is_pressed

        if self.keyboard_x_boost_active:
            return (
                self.get_parameter("keyboard_x_double_press_gain")
                .get_parameter_value()
                .double_value
            )
        return (
            self.get_parameter("keyboard_x_single_press_gain")
            .get_parameter_value()
            .double_value
        )

    @staticmethod
    def _clamp_int(value, lower, upper):
        return int(max(lower, min(upper, value)))

    @staticmethod
    def _apply_deadzone(value, deadzone):
        if abs(value) <= deadzone:
            return 0.0
        return value


def main(args=None):
    rclpy.init(args=args)
    node = ManualPositionHoldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
