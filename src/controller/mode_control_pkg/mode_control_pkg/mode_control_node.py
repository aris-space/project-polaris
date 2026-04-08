"""
This node should act as the logic for switching between different modes of operation for the robot. It will subscribe to the topics published by the foxglove_bridge
and determine which mode the robot should be in based on the incoming data. It will then publish the current mode to a topic and redirects the control commands to the appropriate topics for the current mode.
The modes include manual control, manual depth hold, emergency stop and later also the autonomous modes.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy
from config_pkg.constants import JoyControlMapping, CONTROLLER_LAYOUT


class ModeControlNode(Node):
    def __init__(self):
        super().__init__("mode_control_node")
        self.current_mode = "manual_control"
        self.pixhawk_mode = "MANUAL"  # To track the current mode for Pixhawk
        self.prev_mode = None  # To track changes
        self.prev_pixhawk_mode = None
        self.last_pixhawk_mode_before_stabilization = (
            "MANUAL"  # To track the last mode before entering stabilization
        )

        # Debounce states for button presses
        self.prev_arm_button_state = 0
        self.prev_disarm_button_state = 0
        self.prev_stabilization_button_state = 0
        self.prev_collision_avoidance_button_state = 0
        self.current_collision_avoidance_checking_state = False

        # Publishers & Subscribers
        self.mode_publisher = self.create_publisher(
            String, "/mode_control/current_mode", 10
        )
        self.pixhawk_mode_publisher = self.create_publisher(
            String, "/pixhawk/mode_cmd", 10
        )
        self.arm_cmd_publisher = self.create_publisher(Bool, "/pixhawk/arm_cmd", 10)
        self.joy_subscriber = self.create_subscription(
            Joy, "/joy", self.command_callback, 10
        )

        self.collision_avoidance_checking_publisher = self.create_publisher(
            Bool, "/collision_avoidance/checking", 10
        )

        self.get_logger().info("Mode Control Node Started. Default: manual_control")

    """--------------------------------------------- Callback functions for the subscribers ---------------------------------------------"""


    def command_callback(self, msg):
        buttons = msg.buttons
        axes = msg.axes

        # 1. High Priority: Emergency Stop (Touchpad Button)
        if buttons[JoyControlMapping.EMERGENCY_STOP_BUTTON_IDX_LEFT] == 1 or buttons[JoyControlMapping.EMERGENCY_STOP_BUTTON_IDX_RIGHT] == 1:
            self.current_mode = "emergency_stop"
            if self.pixhawk_mode != "MANUAL":
                self.pixhawk_mode = (
                    "MANUAL"  # Ensure Pixhawk is in MANUAL for emergency stop
                )

        # 2. Mode Switching Logic (Requires Safety Button Pressed)
        elif self.mode_safety_button_pressed(msg):
            if (
                CONTROLLER_LAYOUT == "DESKTOP"
                and axes[JoyControlMapping.MODE_MANUAL_AXES_IDX] == 1.0
            ) or (
                CONTROLLER_LAYOUT != "DESKTOP"
                and buttons[JoyControlMapping.MODE_MANUAL_BUTTON_IDX] == 1
            ):
                # MANUAL CONTROL MODE
                self.current_mode = "manual_control"
                self.pixhawk_mode = "MANUAL"

            elif (
                CONTROLLER_LAYOUT == "DESKTOP"
                and axes[JoyControlMapping.MODE_ALT_HOLD_AXES_IDX] == 1.0
                or (
                    CONTROLLER_LAYOUT != "DESKTOP"
                    and buttons[JoyControlMapping.MODE_ALT_HOLD_BUTTON_IDX] == 1
                )
            ):
                #MANUAL DEPTH HOLD MODE
                self.current_mode = "manual_depth_hold"
                self.pixhawk_mode = "ALT_HOLD"

            elif (
                CONTROLLER_LAYOUT == "DESKTOP"
                and axes[JoyControlMapping.MODE_SPARE_1_DPAD_AXES_IDX] == -1.0
                or (
                    CONTROLLER_LAYOUT != "DESKTOP"
                    and buttons[JoyControlMapping.MODE_SPARE_1_DPAD_BUTTON_IDX] == 1
                )
            ):
                # SPARE MODE 1
                pass

            elif (
                CONTROLLER_LAYOUT == "DESKTOP"
                and axes[JoyControlMapping.MODE_SPARE_2_DPAD_AXES_IDX] == -1.0
                or (
                    CONTROLLER_LAYOUT != "DESKTOP"
                    and buttons[JoyControlMapping.MODE_SPARE_2_DPAD_BUTTON_IDX] == 1
                )
            ):
                # PID TUNING STEP INPUTS MODE
                self.current_mode = "step_inputs_mode"
                self.pixhawk_mode = "STABILIZATION"

        # 3. Setting Control (Requires Setting Safety Button Pressed)
        elif self.setting_safety_button_pressed(msg):
            # 3.1. Arm Command
            current_arm_button_state = (
                axes[JoyControlMapping.SETTING_ARM_DISARM_AXIS_IDX] == 1.0
                if CONTROLLER_LAYOUT == "DESKTOP"
                else buttons[JoyControlMapping.SETTING_ARM_BUTTON_IDX] == 1
            )
            if current_arm_button_state and not self.prev_arm_button_state:
                self.publish_arm_cmd(True)
            self.prev_arm_button_state = current_arm_button_state

            # 3.2. Disarm Command
            current_disarm_button_state = (
                axes[JoyControlMapping.SETTING_ARM_DISARM_AXIS_IDX] == -1.0
                if CONTROLLER_LAYOUT == "DESKTOP"
                else buttons[JoyControlMapping.SETTING_DISARM_BUTTON_IDX] == 1
            )
            if current_disarm_button_state and not self.prev_disarm_button_state:
                self.publish_arm_cmd(False)
            self.prev_disarm_button_state = current_disarm_button_state

            # 3.3. Stabilization Setting Toggle
            current_stabilization_button_state = (
                axes[JoyControlMapping.SETTING_STABILIZATION_AXIS_IDX] == 1.0
                if CONTROLLER_LAYOUT == "DESKTOP"
                else
                buttons[JoyControlMapping.SETTING_STABILIZATION_BUTTON_IDX] == 1)
            if current_stabilization_button_state and not self.prev_stabilization_button_state:
                if self.current_mode != "manual_control":
                    self.get_logger().info(
                        "STABILIZATION Setting not available in current mode"
                    )
                else:
                    self.get_logger().info("Toggling Stabilization Setting")
                    if self.pixhawk_mode == "STABILIZATION":
                        self.pixhawk_mode = self.last_pixhawk_mode_before_stabilization
                    else:
                        self.last_pixhawk_mode_before_stabilization = self.pixhawk_mode
                        self.pixhawk_mode = "STABILIZATION"
            self.prev_stabilization_button_state = current_stabilization_button_state

            # 3.4. Collision Avoidance Setting Toggle
            current_collision_avoidance_button_state = (
                axes[JoyControlMapping.SETTING_COLLISION_AVOIDANCE_AXIS_IDX] == -1.0
                if CONTROLLER_LAYOUT == "DESKTOP"
                else buttons[JoyControlMapping.SETTING_COLLISION_AVOIDANCE_BUTTON_IDX] == 1)
            if current_collision_avoidance_button_state and not self.prev_collision_avoidance_button_state:
                self.get_logger().info("Toggling Collision Avoidance Setting")
                if self.current_mode == "emergency_stop":
                    self.get_logger().info(
                        "Collision Avoidance not available in Emergency Stop mode"
                    )
                elif self.current_collision_avoidance_checking_state:
                    self.current_collision_avoidance_checking_state = False
                    self.publish_collision_avoidance_checking(False)
                elif not self.current_collision_avoidance_checking_state:
                    self.current_collision_avoidance_checking_state = True
                    self.publish_collision_avoidance_checking(True)
            self.prev_collision_avoidance_button_state = current_collision_avoidance_button_state

        # 4. Only publish and log if the state has actually changed
        if self.current_mode != self.prev_mode:
            self.publish_mode()
            self.prev_mode = self.current_mode

        if self.pixhawk_mode != self.prev_pixhawk_mode:
            self.publish_pixhawk_mode()
            self.prev_pixhawk_mode = self.pixhawk_mode
            # Update Pixhawk mode tracking if needed

    """--------------------------------------------- helper functions for the callback functions ---------------------------------------------"""

    def publish_mode(self):
        mode_msg = String()
        mode_msg.data = self.current_mode
        self.mode_publisher.publish(mode_msg)
        self.get_logger().info(f"Mode changed! New Mode: {self.current_mode}")

    def publish_pixhawk_mode(self):
        pixhawk_mode_msg = String()
        pixhawk_mode_msg.data = self.pixhawk_mode
        self.pixhawk_mode_publisher.publish(pixhawk_mode_msg)

    def publish_arm_cmd(self, arm_bool):
        arm_cmd_msg = Bool()
        arm_cmd_msg.data = arm_bool
        self.arm_cmd_publisher.publish(arm_cmd_msg)

    def publish_collision_avoidance_checking(self, checking_bool):
        checking_msg = Bool()
        checking_msg.data = checking_bool
        self.get_logger().info(f"Published /collision_avoidance/checking: {checking_bool}")
        self.collision_avoidance_checking_publisher.publish(checking_msg)

    def mode_safety_button_pressed(self, msg):
        # This function should check the state of the safety button
        # For now, we will just return True to allow mode switching
        buttons = msg.buttons
        return buttons[JoyControlMapping.MODE_SAFETY_BUTTON_IDX] == 1

    def setting_safety_button_pressed(self, msg):
        buttons = msg.buttons
        return buttons[JoyControlMapping.SETTING_SAFETY_BUTTON_IDX] == 1


"""--------------------------------------------- main function ---------------------------------------------"""


def main(args=None):
    rclpy.init(args=args)
    mode_control_node = ModeControlNode()
    rclpy.spin(mode_control_node)
    mode_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
