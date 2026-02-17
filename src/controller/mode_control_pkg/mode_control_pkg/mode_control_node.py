"""
This node should act as the logic for switching between different modes of operation for the robot. It will subscribe to the topics published by the foxglove_bridge
and determine which mode the robot should be in based on the incoming data. It will then publish the current mode to a topic and redirects the control commands to the appropriate topics for the current mode.
The modes include manual control, manual depth hold, emergency stop and later also the autonomous modes.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy


class ModeControlNode(Node):
    def __init__(self):
        super().__init__("mode_control_node")
        self.current_mode = "manual_control"
        self.pixhawk_mode = "MANUAL"  # To track the current mode for Pixhawk
        self.prev_mode = None  # To track changes
        self.prev_pixhawk_mode = None

        # Publishers & Subscribers
        self.mode_publisher = self.create_publisher(
            String, "mode_control/current_mode", 10
        )
        self.pixhawk_mode_publisher = self.create_publisher(
            String, "pixhawk/mode_cmd", 10
        )
        self.joy_subscriber = self.create_subscription(
            Joy, "joy", self.command_callback, 10
        )

        self.get_logger().info("Mode Control Node Started. Default: manual_control")

    def mode_control_callback(self, msg):
        axes = msg.axes
        buttons = msg.buttons

    def command_callback(self, msg):
        buttons = msg.buttons
        axes = msg.axes

        # 1. High Priority: Emergency Stop (Button 3 / Triangle)
        if buttons[2] == 1:
            self.current_mode = "emergency_stop"
            if self.pixhawk_mode != "MANUAL":
                self.pixhawk_mode = (
                    "MANUAL"  # Ensure Pixhawk is in MANUAL for emergency stop
                )

        # 2. Mode Switching Logic (Requires Safety Button 0 / X)
        elif self.safety_button_pressed(msg):
            if axes[7] == 1.0:  # D-pad Up
                self.current_mode = "manual_control"
                self.pixhawk_mode = "MANUAL"
                # TODO: here add the option for stabilization mode

            elif axes[6] == 1.0:  # D-pad Left
                self.current_mode = "manual_depth_hold"
                self.pixhawk_mode = "ALT_HOLD"

        # 3. Only publish and log if the state has actually changed
        if self.current_mode != self.prev_mode:
            self.publish_mode()
            self.prev_mode = self.current_mode

        if self.pixhawk_mode != self.prev_pixhawk_mode:
            self.publish_pixhawk_mode()
            self.prev_pixhawk_mode = self.pixhawk_mode
            # Update Pixhawk mode tracking if needed

    def publish_mode(self):
        mode_msg = String()
        mode_msg.data = self.current_mode
        self.mode_publisher.publish(mode_msg)
        self.get_logger().info(f"Mode changed! New Mode: {self.current_mode}")

    def publish_pixhawk_mode(self):
        pixhawk_mode_msg = String()
        pixhawk_mode_msg.data = self.pixhawk_mode
        self.pixhawk_mode_publisher.publish(pixhawk_mode_msg)

    def safety_button_pressed(self, msg):
        # This function should check the state of the safety button
        # For now, we will just return True to allow mode switching
        buttons = msg.buttons
        return buttons[0] == 1  # Assuming button 0 (X) is the safety button


def main(args=None):
    rclpy.init(args=args)
    mode_control_node = ModeControlNode()
    rclpy.spin(mode_control_node)
    mode_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
