"""
Central mode-switching node. Translates joystick safety+button combos into
mode changes and arms/disarms the Pixhawk.

Mode strings match ArduSub flight modes exactly so that /mode_control/current_mode
and /pixhawk/mode_cmd are always in sync (single source of truth):
  MANUAL    – full 6DOF manual control
  ALT_HOLD  – depth-hold; pilot controls surge/sway/yaw, Pixhawk holds depth
  STABILIZE – attitude-stabilized manual control
  GUIDED    – autonomous waypoint following (Pixhawk handles control loop)

Button layout:
  MODE safety (Triangle) + D-pad Left   → MANUAL
  MODE safety (Triangle) + D-pad Up     → ALT_HOLD
  MODE safety (Triangle) + D-pad Down   → STABILIZE
  MODE safety (Triangle) + D-pad Right  → GUIDED
  SETTING safety (Square) + D-pad Up    → Arm
  SETTING safety (Square) + D-pad Down  → Disarm
  SETTING safety (Square) + D-pad Left  → Toggle collision avoidance
  SETTING safety (Square) + D-pad Right → Reboot Pixhawk
  L3 or R3 (any time, rising edge)      → Emergency disarm + reset to MANUAL
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy
from mavros_msgs.msg import State
from config_pkg.constants import JoyControlMapping, CONTROLLER_LAYOUT


class ModeControlNode(Node):
    def __init__(self):
        super().__init__("mode_control_node")
        self.mode = "MANUAL"  # fallback until first heartbeat arrives
        self.prev_mode = None
        self._mode_initialized = False

        # Debounce states for button presses
        self.prev_arm_button_state = False
        self.prev_disarm_button_state = False
        self.prev_emergency_button_state = False
        self.prev_collision_avoidance_button_state = False
        self.prev_reboot_button_state = False
        self.collision_avoidance_active = False

        self.mode_publisher = self.create_publisher(String, "/mode_control/current_mode", 10)
        self.pixhawk_mode_publisher = self.create_publisher(String, "/pixhawk/mode_cmd", 10)
        self.arm_cmd_publisher = self.create_publisher(Bool, "/pixhawk/arm_cmd", 10)
        self.reboot_cmd_publisher = self.create_publisher(Bool, "/pixhawk/reboot_cmd", 10)
        self.collision_avoidance_checking_publisher = self.create_publisher(
            Bool, "/collision_avoidance/checking", 10
        )
        self.joy_subscriber = self.create_subscription(Joy, "/joy", self.command_callback, 10)
        self.heartbeat_subscriber = self.create_subscription(
            State, "/pixhawk/heartbeat", self._heartbeat_cb, 10
        )

        self.get_logger().info("Mode Control Node started. Waiting for Pixhawk heartbeat to sync initial mode...")

    def command_callback(self, msg):
        buttons = msg.buttons
        axes = msg.axes
        setting_on = self.setting_safety_button_pressed(msg)

        # Arm/disarm (SETTING safety held) — runs independently of the mode/emergency chain below.
        # JETSON layout shares the D-pad Down button between disarm and MODE_SPARE_2; debounce is
        # reset whenever setting safety is released so the two contexts don't bleed into each other.
        if setting_on:
            if CONTROLLER_LAYOUT == "DESKTOP":
                cur_arm = axes[JoyControlMapping.SETTING_ARM_DISARM_AXIS_IDX] == 1.0
                cur_disarm = axes[JoyControlMapping.SETTING_ARM_DISARM_AXIS_IDX] == -1.0
            else:
                cur_arm = buttons[JoyControlMapping.SETTING_ARM_BUTTON_IDX] == 1
                cur_disarm = buttons[JoyControlMapping.SETTING_DISARM_BUTTON_IDX] == 1
            if cur_arm and not self.prev_arm_button_state:
                self.publish_arm_cmd(True)
            if cur_disarm and not self.prev_disarm_button_state:
                self.publish_arm_cmd(False)
            self.prev_arm_button_state = cur_arm
            self.prev_disarm_button_state = cur_disarm
        else:
            self.prev_arm_button_state = False
            self.prev_disarm_button_state = False
            self.prev_reboot_button_state = False

        # 1. Emergency disarm: L3 or R3, rising edge → disarm and reset to MANUAL
        cur_emergency = (
            buttons[JoyControlMapping.EMERGENCY_STOP_BUTTON_IDX_LEFT] == 1
            or buttons[JoyControlMapping.EMERGENCY_STOP_BUTTON_IDX_RIGHT] == 1
        )
        if cur_emergency and not self.prev_emergency_button_state:
            self.publish_arm_cmd(False)
            self._set_mode("MANUAL")
        self.prev_emergency_button_state = cur_emergency

        # 2. Mode switching (MODE safety held, no emergency active)
        if not cur_emergency and self.mode_safety_button_pressed(msg):
            if CONTROLLER_LAYOUT == "DESKTOP":
                if axes[JoyControlMapping.MODE_MANUAL_AXES_IDX] == 1.0:
                    self._set_mode("MANUAL")
                elif axes[JoyControlMapping.MODE_ALT_HOLD_AXES_IDX] == 1.0:
                    self._set_mode("ALT_HOLD")
                elif axes[JoyControlMapping.MODE_SPARE_2_DPAD_AXES_IDX] == -1.0:
                    self._set_mode("STABILIZE")
                elif axes[JoyControlMapping.MODE_SPARE_1_DPAD_AXES_IDX] == -1.0:
                    self._set_mode("GUIDED")
            else:
                if buttons[JoyControlMapping.MODE_MANUAL_BUTTON_IDX] == 1:
                    self._set_mode("MANUAL")
                elif buttons[JoyControlMapping.MODE_ALT_HOLD_BUTTON_IDX] == 1:
                    self._set_mode("ALT_HOLD")
                elif buttons[JoyControlMapping.MODE_SPARE_2_DPAD_BUTTON_IDX] == 1:
                    self._set_mode("STABILIZE")
                elif buttons[JoyControlMapping.MODE_SPARE_1_DPAD_BUTTON_IDX] == 1:
                    self._set_mode("GUIDED")

        # 3. Settings (SETTING safety held, no emergency) — collision avoidance toggle + reboot
        elif setting_on and not cur_emergency:
            cur_ca = (
                axes[JoyControlMapping.SETTING_COLLISION_AVOIDANCE_AXIS_IDX] == 1.0
                if CONTROLLER_LAYOUT == "DESKTOP"
                else buttons[JoyControlMapping.SETTING_COLLISION_AVOIDANCE_BUTTON_IDX] == 1
            )
            if cur_ca and not self.prev_collision_avoidance_button_state:
                self.collision_avoidance_active = not self.collision_avoidance_active
                self.publish_collision_avoidance_checking(self.collision_avoidance_active)
            self.prev_collision_avoidance_button_state = cur_ca

            cur_reboot = (
                axes[JoyControlMapping.SETTING_COLLISION_AVOIDANCE_AXIS_IDX] == -1.0
                if CONTROLLER_LAYOUT == "DESKTOP"
                else buttons[JoyControlMapping.SETTING_REBOOT_BUTTON_IDX] == 1
            )
            if cur_reboot and not self.prev_reboot_button_state:
                self.publish_reboot_cmd()
            self.prev_reboot_button_state = cur_reboot

    def _heartbeat_cb(self, msg: State):
        if not self._mode_initialized:
            self._mode_initialized = True
            self.get_logger().info(f"Mode initialized from Pixhawk heartbeat: {msg.mode}")

        if msg.mode != self.prev_mode:
            self.mode = msg.mode
            self.prev_mode = msg.mode
            mode_msg = String()
            mode_msg.data = msg.mode
            self.mode_publisher.publish(mode_msg)
            self.get_logger().info(f"Mode synced from Pixhawk heartbeat: {msg.mode}")

    def _set_mode(self, mode: str):
        """Publish mode to /mode_control/current_mode and /pixhawk/mode_cmd together."""
        if mode == self.prev_mode:
            return
        self.mode = mode
        self.prev_mode = mode
        mode_msg = String()
        mode_msg.data = mode
        self.mode_publisher.publish(mode_msg)
        self.pixhawk_mode_publisher.publish(mode_msg)
        self.get_logger().info(f"Mode → {mode}")

    def publish_arm_cmd(self, arm: bool):
        msg = Bool()
        msg.data = arm
        self.arm_cmd_publisher.publish(msg)
        self.get_logger().info(f"{'Arm' if arm else 'Disarm'} command sent")

    def publish_reboot_cmd(self):
        msg = Bool()
        msg.data = True
        self.reboot_cmd_publisher.publish(msg)
        self.get_logger().info("Reboot command sent to Pixhawk")

    def publish_collision_avoidance_checking(self, active: bool):
        msg = Bool()
        msg.data = active
        self.collision_avoidance_checking_publisher.publish(msg)
        self.get_logger().info(f"Collision avoidance: {'enabled' if active else 'disabled'}")

    def mode_safety_button_pressed(self, msg):
        return msg.buttons[JoyControlMapping.MODE_SAFETY_BUTTON_IDX] == 1

    def setting_safety_button_pressed(self, msg):
        return msg.buttons[JoyControlMapping.SETTING_SAFETY_BUTTON_IDX] == 1


def main(args=None):
    rclpy.init(args=args)
    node = ModeControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
