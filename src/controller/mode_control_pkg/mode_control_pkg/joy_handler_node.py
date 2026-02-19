"""
Joy Handler Node - Fuses multiple joy input sources into a single /joy topic.

Subscribes to:
  - /joy_controller (sensor_msgs/Joy): PS4 controller input
  - /joy_keyboard   (sensor_msgs/Joy): Keyboard movement input mapped as Joy
  - /joy_mode       (sensor_msgs/Joy): Keyboard mode input mapped as Joy

Publishes:
  - /joy (sensor_msgs/Joy): Fused joy output consumed by downstream nodes

Fusion priority logic:
  1. Keyboard active                  -> keyboard + mode (if active), controller ignored
  2. Mode active, keyboard inactive   -> mode + controller (mode buttons stripped from controller)
  3. Neither keyboard nor mode active -> pass through controller as-is

A source is considered "active" when a message with non-neutral content
(any axis deviating from its neutral value, or any pressed button) was received
within DELTA_T seconds.  Neutral values: 0.0 for stick axes, 1.0 for L2/R2 triggers.

The output message always has exactly NUM_AXES axes and NUM_BUTTONS buttons,
regardless of the source message sizes, so downstream nodes can safely index
into the arrays.

Foxglove controller layout (button IDs):
  0  Mode switch safety   (X)          8  Share           16 PS (DO NOT USE)
  1  -                     (O)          9  Options         17 Emergency Stop (Touchpad)
  2  Settings safety       (Square)    10  L-stick press
  3  -                     (Triangle)  11  R-stick press
  4  Roll left             (L1)        12  D-pad up   (Mode: Alt Hold / Arm)
  5  Roll right            (R1)        13  D-pad down
  6  Z up                  (L2)        14  D-pad left (Mode: 6DOF / Stabilisation)
  7  Z down                (R2)        15  D-pad right

Foxglove controller layout (axis IDs):
  0  Y left/right       (left stick)   3  Yaw   (right stick horizontal)
  1  X forward/backward (left stick)   4  Pitch (right stick vertical)
  2  L2 trigger                        5  R2 trigger
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

DELTA_T = 0.2
PUBLISH_RATE_HZ = 20.0

NUM_BUTTONS = 18  # indices 0..17 (highest: 17 = Touchpad / Emergency Stop)
NUM_AXES = 6  # indices 0..5  (highest: 5 = R2 trigger axis)

# Neutral (unpressed) value for each axis.  Sticks rest at 0.0; L2/R2 triggers rest at 1.0.
NEUTRAL_AXES = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0]
AXIS_DEADZONE = 0.05

# Foxglove layout: button indices that belong to the "mode" group
MODE_BUTTON_INDICES = [
    0,   # Mode switch safety button (X)
    2,   # Settings safety button (Square)
    12,  # D-pad up  - Mode: Manual Altitude Hold / Settings: Arm Pixhawk
    13,  # D-pad down
    14,  # D-pad left - Mode: Manual 6DOF / Setting: Toggle Stabilisation
    15,  # D-pad right
    17,  # Emergency Stop (Touchpad)
]


class JoyHandlerNode(Node):
    def __init__(self):
        super().__init__("joy_handler_node")

        self.last_controller_msg = None
        self.last_keyboard_msg = None
        self.last_mode_msg = None

        self.last_controller_time = None
        self.last_keyboard_time = None
        self.last_mode_time = None

        self.create_subscription(Joy, "/joy_controller", self._controller_cb, 10)
        self.create_subscription(Joy, "/joy_keyboard", self._keyboard_cb, 10)
        self.create_subscription(Joy, "/joy_mode", self._mode_cb, 10)

        self.joy_publisher = self.create_publisher(Joy, "/joy", 10)

        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_joy)

        self.get_logger().info("JoyHandlerNode initialized")

    # ---- Subscription callbacks ------------------------------------------------

    def _controller_cb(self, msg):
        self.last_controller_msg = msg
        self.last_controller_time = self.get_clock().now()

    def _keyboard_cb(self, msg):
        self.last_keyboard_msg = msg
        self.last_keyboard_time = self.get_clock().now()

    def _mode_cb(self, msg):
        self.last_mode_msg = msg
        self.last_mode_time = self.get_clock().now()

    # ---- Fusion / publish loop -------------------------------------------------

    def _publish_joy(self):
        now = self.get_clock().now()

        keyboard_active = self._is_active(
            self.last_keyboard_msg, self.last_keyboard_time
        )
        mode_active = self._is_active(self.last_mode_msg, self.last_mode_time)

        output = None

        if keyboard_active:
            if mode_active:
                output = self._fuse_keyboard_and_mode(
                    self.last_keyboard_msg, self.last_mode_msg
                )
            else:
                output = self._normalize(self.last_keyboard_msg)
                self._zero_mode_buttons(output)
        elif mode_active:
            if self.last_controller_msg is not None:
                output = self._fuse_mode_and_controller(
                    self.last_mode_msg, self.last_controller_msg
                )
            else:
                output = self._mode_only(self.last_mode_msg)
        else:
            if self.last_controller_msg is not None:
                output = self._normalize(self.last_controller_msg)

        if output is None:
            return

        output.header.stamp = now.to_msg()
        self.joy_publisher.publish(output)

    # ---- Activity detection ----------------------------------------------------

    def _is_active(self, msg, timestamp):
        """True when a non-neutral message arrived within the last DELTA_T seconds."""
        if msg is None or timestamp is None:
            return False
        elapsed = (self.get_clock().now() - timestamp).nanoseconds / 1e9
        if elapsed > DELTA_T:
            return False
        return self._has_nonzero(msg)

    @staticmethod
    def _has_nonzero(msg):
        for i, val in enumerate(msg.axes):
            if i >= NUM_AXES:
                break
            if abs(val - NEUTRAL_AXES[i]) > AXIS_DEADZONE:
                return True
        for val in msg.buttons:
            if val != 0:
                return True
        return False

    # ---- Fusion strategies -----------------------------------------------------

    def _fuse_keyboard_and_mode(self, keyboard_msg, mode_msg):
        """Keyboard provides movement, /joy_mode provides mode buttons."""
        output = self._normalize(keyboard_msg)
        for idx in MODE_BUTTON_INDICES:
            if idx < len(mode_msg.buttons):
                output.buttons[idx] = mode_msg.buttons[idx]
        return output

    def _fuse_mode_and_controller(self, mode_msg, controller_msg):
        """Controller provides movement (mode buttons stripped), /joy_mode provides mode buttons."""
        output = self._normalize(controller_msg)
        self._zero_mode_buttons(output)
        for idx in MODE_BUTTON_INDICES:
            if idx < len(mode_msg.buttons):
                output.buttons[idx] = mode_msg.buttons[idx]
        return output

    # ---- Helpers ---------------------------------------------------------------

    @staticmethod
    def _zero_mode_buttons(msg):
        for idx in MODE_BUTTON_INDICES:
            msg.buttons[idx] = 0

    @staticmethod
    def _mode_only(mode_msg):
        """Keep only mode buttons from the mode message; all axes and other buttons stay neutral."""
        out = Joy()
        out.header = mode_msg.header
        out.axes = list(NEUTRAL_AXES)
        out.buttons = [0] * NUM_BUTTONS
        for idx in MODE_BUTTON_INDICES:
            if idx < len(mode_msg.buttons):
                out.buttons[idx] = mode_msg.buttons[idx]
        return out

    @staticmethod
    def _normalize(msg):
        """Create a fixed-size Joy message from a potentially shorter source."""
        out = Joy()
        out.header = msg.header
        out.axes = list(NEUTRAL_AXES)
        out.buttons = [0] * NUM_BUTTONS
        for i in range(min(len(msg.axes), NUM_AXES)):
            out.axes[i] = msg.axes[i]
        for i in range(min(len(msg.buttons), NUM_BUTTONS)):
            out.buttons[i] = msg.buttons[i]
        return out


def main(args=None):
    rclpy.init(args=args)
    node = JoyHandlerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
