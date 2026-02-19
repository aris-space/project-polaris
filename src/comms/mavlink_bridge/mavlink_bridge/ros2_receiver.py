import rclpy
from rclpy.node import Node
import os

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil
from std_msgs.msg import String
from std_msgs.msg import Bool, Int16MultiArray
from mavros_msgs.msg import OverrideRCIn


class MavlinkBridgeReceiver(Node):
    """
    Node that is supposed to translate ROS2 messages that it receives to Mavlink for the pixhawk
    """

    def __init__(self):
        # "mavlink_bridge" is the name of the node
        super().__init__("mavlink_bridge_receiver")

        self.pixhawk_mode = (
            "MANUAL"  # To track the current mode for Pixhawk (e.g., MANUAL, ALT_HOLD)
        )

        # configures serial port the pixhawk is connected to and the baud rate
        self.port = mavutil.mavlink_connection(
            "/dev/ttyTHS1", baud=57600
        )  # For sending commands to Pixhawk
        # self.port_in = mavutil.mavlink_connection(
        #     "/dev/ttyTHS1", baud=57600
        # )  # For receiving messages from Pixhawk (e.g., heartbeats, status)

        # Wait for a heartbeat so we know the target system IDs. Code can get stuck here meaning we didn't receive any heartbeat
        self.port.wait_heartbeat()
        self.get_logger().info(
            f"Heartbeat received from system {self.port.target_system}"
        )

        # Subscribe to RC override messages from ROS2 topic "pixhawk/rc_override" and then calls the rc_override_cb (translator) function when a message arrives. Accepts only RCIn messages
        self.rc_override_subscriber = self.create_subscription(
            OverrideRCIn,
            "/pixhawk/rc_override",
            self.rc_override_cb,
            10,  # overrideRCIn is a 8 integer array, so the function currently only accepts that input type
        )

        self.manual_control_subscriber = self.create_subscription(
            Int16MultiArray,
            "/pixhawk/manual_control",
            self.manual_control_cb,
            10,
        )

        # subscribe to the pixhawk/mode_cmd topic and calls mode_selection_cb
        self.mode_selection_subscriber = self.create_subscription(
            String, "/pixhawk/mode_cmd", self.mode_selection_cb, 10
        )

        self.arm_disarm_subscriber = self.create_subscription(
            Bool, "/pixhawk/arm_cmd", self.arm_disarm_cb, 10
        )

        self.get_logger().info("MavlinkBridgeReceiver: Node has been initialized")

    """--------------------------------------------- Callback functions for the subscribers ---------------------------------------------"""

    def rc_override_cb(self, msg):
        """
        Called automatically when a message arrives on "pixhawk/rc_override" topic.
        Converts the ROS2 OverrideRCIn message to MAVLink RC_OVERRIDE and sends it to Pixhawk.
        """
        self.get_logger().info(f"Received ROS2 RC override: {msg.channels}")

        channels = msg.channels

        # Send MAVLink RC_CHANNELS_OVERRIDE message
        # Arguments: target_system, target_component and the different RCOverride values in channels
        self.port.mav.rc_channels_override_send(
            self.port.target_system,  # Target system ID
            self.port.target_component,  # Target component ID
            channels[0],
            channels[1],
            channels[2],
            channels[3],
            channels[4],
            channels[5],
            channels[6],
            channels[7],
        )

    def manual_control_cb(self, msg):
        """
        Called when a message arrives in the pixhawk/manual_control topic. The message should contain the surge, sway, heave, roll, pitch and yaw values for the manual control command.
        """
        if self.pixhawk_mode == "MANUAL" or self.pixhawk_mode == "STABILIZATION" and len(msg.data) == 6:
            self.send_6dof_command(msg.data)

        elif self.pixhawk_mode == "ALT_HOLD" and len(msg.data) == 6:
            self.send_6dof_command(
                msg.data
            )  # In ALT_HOLD, we typically control surge, sway, heave, and yaw, but not roll and pitch
        elif len(msg.data) == 4:
            self.get_logger().warn(
                f"Received 4DOF manual control command, but current mode {self.pixhawk_mode} may require 6DOF."
            ) 
        else:
            self.get_logger().warn(
                f"Received manual control command in unsupported mode: {self.pixhawk_mode}. Command ignored. (manual_control_cb function in ros2_receiver.py)"
            )

    def arm_disarm_cb(self, msg):
        """
        Called when a message arrives in the pixhawk/arm_cmd topic. The message should contain a Bool (True to arm, False to disarm).
        """
        arm_bool = msg.data
        self.port.mav.command_long_send(
            self.port.target_system,
            self.port.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            1 if arm_bool else 0,  # Param 1: 1 to arm, 0 to disarm
            0,
            0,
            0,
            0,
            0,
            0,  # Unused parameters
        )

        self.get_logger().info(f"Sent {'arm' if arm_bool else 'disarm'} command to Pixhawk")

        # Wait for acknowledgment
        #ack = self.port.recv_match(type="COMMAND_ACK", blocking=True)
        #print(f"Arming status: {ack.result}")  # 0 = Success

    def mode_selection_cb(self, msg):
        """
        Called when a message arrives in the pixhawk/mode_cmd topic. Example: if the message is mapped to "ALT_HOLD" then the sub will perform that function
        id mappings:{'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3, 'GUIDED': 4, 'CIRCLE': 7, 'SURFACE': 9, 'POSHOLD': 16, 'MANUAL': 19}
        """
        self.get_logger().info(f"Received ROS2 RC Mode message: {msg.data}")
        if msg.data == "ALT_HOLD":
            # Set mode to ALT_HOLD (Depth Hold for ArduSub)
            # Base mode 209 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
            mode_id = 2
            self.port.mav.set_mode_send(
                self.port.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            self.pixhawk_mode = "ALT_HOLD"  # Update the tracked Pixhawk mode
            self.get_logger().info("Sent ALT_HOLD mode command")

        elif msg.data == "MANUAL":
            mode_id = 19
            self.port.mav.set_mode_send(
                self.port.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            self.pixhawk_mode = "MANUAL"  # Update the tracked Pixhawk mode
            self.get_logger().info("Sent MANUAL mode command")

        elif msg.data == "STABILIZATION":
            mode_id = 0
            self.port.mav.set_mode_send(
                self.port.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            self.pixhawk_mode = "STABILIZATION"  # Update the tracked Pixhawk mode
            self.get_logger().info("Sent STABILIZATION mode command")

    """--------------------------------------------- helper functions for the callback functions ---------------------------------------------"""

    def send_4dof_command(self, control_input):
        """
        Input values: -1000 to 1000 (except heave, see below)
        """
        self._logger.info(f"Sending 4DOF command with control input: {control_input}")
        surge, sway, heave, yaw = control_input
        self.port.mav.manual_control_send(
            self.port.target_system,
            int(surge),  # x: Forward/Back
            int(sway),  # y: Left/Right
            int(heave),  # z: Up/Down (range 0-1000, 500 is neutral)
            int(yaw),  # r: Yaw
            0,  # buttons bitmask
        )

    def send_4dof_command_test(self, control_input):
        """
        Input values: -1000 to 1000 (except heave, see below)
        """
        self._logger.info(f"DUMMY FUNCTION Sending 4DOF command with control input")
        self.port.mav.manual_control_send(
            self.port.target_system,
            123,  # x: Forward/Back
            123,  # y: Left/Right
            500,  # z: Up/Down (range 0-1000, 500 is neutral)
            123,  # r: Yaw
            0,  # buttons bitmask
        )

    def send_6dof_command(self, control_input):
        """
        Note: Extension fields (s, t) are usually enabled in
        newer MAVLink 2.0 implementations. This has to be tested!
        Input values: -1000 to 1000 (except heave, see below)
        """
        #self._logger.info(f"Sending 6DOF command with control input: {control_input}")
        surge, sway, heave, yaw, roll, pitch = control_input
        self.port.mav.manual_control_send(
            self.port.target_system,
            int(surge),  # x
            int(sway),  # y
            int(heave),  # z (0-1000)
            int(yaw),  # r
            0,  # buttons
            0,  # buttons 2
            3,  # MAVLINK_MSG_MANUAL_CONTROL_FIELD_FLAGS_ENABLE_EXTENSION (enables s and t fields)
            int(pitch),  # s (Extension 1)
            int(roll),  # t (Extension 2)
        )

    """--------------------------------------------- main function ---------------------------------------------"""


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeReceiver()
    rclpy.spin(node)  # Keeps the node running and processing callbacks
    rclpy.shutdown()
