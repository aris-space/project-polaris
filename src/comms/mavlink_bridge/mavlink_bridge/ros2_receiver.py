from geometry_msgs import msg
import rclpy
from rclpy.node import Node
import logging, os
from datetime import datetime
from config_pkg.constants import Logs, Comms

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil
from std_msgs.msg import String, Bool, Int16MultiArray
from mavros_msgs.msg import OverrideRCIn

# For Autonomy
# https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/PositionTarget.html
from geometry_msgs.msg import Twist # Needs to be adapted for sending cmd_vel
from geometry_msgs.msg import PoseStamped  # Kept if you still need it elsewhere


log_dir = os.path.expanduser(Logs.LOG_DIR)
os.makedirs(log_dir, exist_ok=True)
log_file = os.path.join(log_dir, f"ros2_receiver_{datetime.now():%Y%m%d_%H%M%S}.log")


class MavlinkBridgeReceiver(Node):
    """
    Node that is supposed to translate ROS2 messages that it receives to Mavlink for the pixhawk
    """

    def __init__(self):
        # "mavlink_bridge" is the name of the node
        super().__init__("mavlink_bridge_receiver")

        self._file_logger = logging.getLogger("ros2_receiver")

        # set level defines from what message type onwards the message is logged. the different levels are:
        # Logging levels (lowest → highest):
        # DEBUG    = detailed diagnostic data (high-frequency sensor + internal state)
        # INFO     = normal operational messages (mode changes, summaries)
        # WARNING  = unexpected situations that do not stop operation
        # ERROR    = recoverable failures
        # CRITICAL = unrecoverable failures; system may be unusable
        self._file_logger.setLevel(logging.INFO)
        self.file_logger = logging.getLogger("ros2_receiver_file")
        file_handler = logging.FileHandler(log_file)
        formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
        file_handler.setFormatter(formatter)
        self._file_logger.addHandler(file_handler)

        self.pixhawk_mode = (
            "MANUAL"  # To track the current mode for Pixhawk (e.g., MANUAL, ALT_HOLD)
        )

        # configures serial port the pixhawk is connected to and the baud rate
        self.port = mavutil.mavlink_connection(
            Comms.SERIAL_PORT1, baud=Comms.SERIAL1_BAUD_RATE
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
            Comms.SUB_QOS_DEPTH,  # overrideRCIn is a 8 integer array, so the function currently only accepts that input type
        )

        self.manual_control_subscriber = self.create_subscription(
            Int16MultiArray,
            "/pixhawk/manual_control",
            self.manual_control_cb,
            Comms.SUB_QOS_DEPTH,
        )

        self.guided_setpoint_subscriber = self.create_subscription(
            Twist, # Depending on the msg type from imports
            "/pixhawk/cmd_vel",
            self.cmd_vel_cb,
            Comms.SUB_QOS_DEPTH,
        )

        # subscribe to the pixhawk/mode_cmd topic and calls mode_selection_cb
        self.mode_selection_subscriber = self.create_subscription(
            String, "/pixhawk/mode_cmd", self.mode_selection_cb, Comms.SUB_QOS_DEPTH
        )

        self.arm_disarm_subscriber = self.create_subscription(
            Bool, "/pixhawk/arm_cmd", self.arm_disarm_cb, Comms.SUB_QOS_DEPTH
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
        if (
            self.pixhawk_mode == "MANUAL"
            or self.pixhawk_mode == "STABILIZATION"
            or self.pixhawk_mode == "ALT_HOLD"
        ) and len(msg.data) == 6:
            self.send_6dof_command(msg.data)

        elif len(msg.data) == 4:
            self.get_logger().warn(
                f"Received 4DOF manual control command, but current mode {self.pixhawk_mode} may require 6DOF."
            )
            self._file_logger.warning(
                f"Received 4DOF manual control command, but current mode {self.pixhawk_mode} may require 6DOF. Command ignored. (manual_control_cb function in ros2_receiver.py)"
            )
        else:
            self.get_logger().warn(
                f"Received manual control command in unsupported mode: {self.pixhawk_mode}. Command ignored. (manual_control_cb function in ros2_receiver.py)"
            )
            self._file_logger.warning(
                f"Received manual control command in unsupported mode: {self.pixhawk_mode}. Command ignored. (manual_control_cb function in ros2_receiver.py)"
            )

    # Question: Is ROS really ussing ENU and ArduSub NED? - Yes
    # TODO: Should we use: set_position_target_local_ned_send or SET_POSITION_TARGET_GLOBAL_INT? Unsure which one.
    # TODO: See https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
    """If we send also set_positions:"""
    """"
    def guided_setpoint_cb(self, msg):
        
        Called when a message arrives in the pixhawk/guided_setpoint topic.
        The message should contain the desired position, velocityand yaw-angle for the guided setpoint command.
        

        # 1. Convert ROS Lat/Lon to MAVLink Global Int (Standard WGS84)
        lat_int = int(msg.latitude * 1e7)
        lon_int = int(msg.longitude * 1e7)
        alt_meters = float(msg.altitude)  # Ensure it's a float

        # 2. Velocity stays in NED (m/s)
        # ROS ENU (vx, vy, vz) -> NED (vy, vx, -vz)
        vel_x_ned = msg.velocity.y
        vel_y_ned = msg.velocity.x
        vel_z_ned = -msg.velocity.z

        # 3. Yaw Transformation: ROS (East 0, CCW) -> ArduPilot (North 0, CW)
        # We also normalize to 0-2pi range to be safe
        yaw_ned = (math.pi / 2.0) - msg.yaw
        while yaw_ned < 0:
            yaw_ned += 2 * math.pi
        while yaw_ned > 2 * math.pi:
            yaw_ned -= 2 * math.pi

        # Bitmask: 2496 (Ignore Accel and Yaw-Rate)
        type_mask = 2496

        # Send MAVLink SET_POSITION_TARGET_GLOBAL_INT (86)
        self.port.mav.set_position_target_global_int_send(
            0,  # time_boot_ms
            self.port.target_system,
            self.port.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Alt is relative to home
            type_mask,
            lat_int,
            lon_int,
            alt_meters,
            vel_x_ned,
            vel_y_ned,
            vel_z_ned,
            0,
            0,
            0,  # Acceleration (ignored)
            yaw_ned,  # Yaw in Radians
            0,  # Yaw-Rate (ignored)
        )
    """
    # For Autonomy if we send just x, z lin.velocity and yaw rate.
    def cmd_vel_cb(self, msg):
        # msg is geometry_msgs.msg.Twist        
        # ArduSub needs GUIDED mode for velocity setpoints
        if self.pixhawk_mode != "GUIDED":
            return

        # 1. Map ROS ENU (Body) to ArduSub NED (Body)
        # ROS X (Forward) -> NED X (Surge)
        # ROS Y (Left)    -> NED Y (Sway) - We set this to 0 if not used
        # ROS Z (Up)      -> NED Z (Heave) - Flip sign because Z is down in NED
        surge = float(msg.linear.x)
        heave = -float(msg.linear.z) 
        
        # ROS Angular Z (CCW) -> NED Yaw Rate (CW) - Flip sign
        yaw_rate = -float(msg.angular.z)

        # 2. Define the Type Mask
        # Bits: 1,2,3 (Pos), 5 (Vel Y), 7,8,9 (Acc), 11 (Yaw Angle) are IGNORED
        # Bits: 4 (Vel X), 6 (Vel Z), 12 (Yaw Rate) are USED
        type_mask = 3031 # 0b0000101111010111

        # 3. Send to Pixhawk
        # Using MAV_FRAME_BODY_OFFSET_NED so "Forward" is relative to the sub's nose
        self.port.mav.set_position_target_local_ned_send(
            0,                                              # time_boot_ms
            self.port.target_system,
            self.port.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,      # Frame: Body-Relative
            type_mask,
            0.0, 0.0, 0.0,                                  # Position (ignored)
            surge, 0.0, heave,                              # Velocities (m/s)
            0.0, 0.0, 0.0,                                  # Acceleration (ignored)
            0.0,                                            # Yaw Angle (ignored)
            yaw_rate                                        # Yaw Rate (rad/s)
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

        self.get_logger().info(
            f"Sent {'arm' if arm_bool else 'disarm'} command to Pixhawk"
        )

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
            self._file_logger.info("Sent ALT_HOLD mode command")
        elif msg.data == "MANUAL":
            mode_id = 19
            self.port.mav.set_mode_send(
                self.port.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            self.pixhawk_mode = "MANUAL"  # Update the tracked Pixhawk mode
            self.get_logger().info("Sent MANUAL mode command")
            self._file_logger.info("Sent MANUAL mode command")
        elif msg.data == "STABILIZATION":
            mode_id = 0
            self.port.mav.set_mode_send(
                self.port.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            self.pixhawk_mode = "STABILIZATION"  # Update the tracked Pixhawk mode
            self.get_logger().info("Sent STABILIZATION mode command")
            self._file_logger.info("Sent STABILIZATION mode command")
        elif msg.data == "GUIDED":
            mode_id = 4
            self.port.mav.set_mode_send(
                self.port.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            self.pixhawk_mode = "GUIDED"
            self.get_logger().info("Sent GUIDED mode command")
            self._file_logger.info("Sent GUIDED mode command")

    """--------------------------------------------- helper functions for the callback functions ---------------------------------------------"""

    def send_4dof_command(self, control_input):
        """
        Input values: -1000 to 1000 (except heave, see below)
        """
        self._file_logger.info(
            f"Sending 4DOF command with control input: {control_input}"
        )
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
        self._file_logger.info(
            f"DUMMY FUNCTION Sending 4DOF command with control input"
        )
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
        self.get_logger().info(
            f"Sending 6DOF command with control input: {control_input}"
        )
        self._file_logger.info(
            f"Sending 6DOF command with control input: {control_input}"
        )
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
