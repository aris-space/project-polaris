import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import logging, os
import math
from datetime import datetime
from config_pkg.constants import Logs, Comms, Ports

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil
from std_msgs.msg import String
from std_msgs.msg import Bool, Int16MultiArray
from nav_msgs.msg import Odometry
from mavros_msgs.msg import OverrideRCIn
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix

from mavlink_bridge.odom_mavlink import (
    nan_pose_covariance,
    nan_velocity_covariance,
    ros_odom_to_mavlink_odometry,
)


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
            Comms.MAVLINK_ROUTER_TCP
        )  # For sending commands to Pixhawk via mavlink-router
        # self.port_in = mavutil.mavlink_connection(
        #     "/dev/ttyTHS1", baud=115200
        # )  # For receiving messages from Pixhawk (e.g., heartbeats, status)

        # Wait for a heartbeat so we know the target system IDs. Code can get stuck here meaning we didn't receive any heartbeat
        self.port.wait_heartbeat()
        self.get_logger().info(
            f"Heartbeat received from system {self.port.target_system}"
        )
        # Make companion telemetry appear under vehicle sysid in QGC tools.
        self.port.mav.srcSystem = self.port.target_system
        self.port.mav.srcComponent = (
            mavutil.mavlink.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
        )

        self.declare_parameter("external_odom_quality", 100)
        self.declare_parameter("external_odom_max_rate_hz", 30.0)
        
        self._odom_reset_counter = 0
        self._external_odom_last_send_ns = 0
        self._gps_origin_sent = False
        self._gps_origin_valid_count = 0

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

        self.pixhawk_reboot_subscriber = self.create_subscription(
            Bool, "/pixhawk/reboot_cmd", self.reboot_cb, Comms.SUB_QOS_DEPTH
        )

        self.external_odom_subscriber = self.create_subscription(
            Odometry,
            "/odometry/filtered/local",
            self.external_odom_cb,
            qos_profile_sensor_data,
        )

        self.gps_fix_subscriber = self.create_subscription(NavSatFix,"/gps/filtered", self.gps_origin_cb, Comms.SUB_QOS_DEPTH)

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

        # 2. Type mask (ArduSub GCS_MAVLink_Sub.cpp): vel_ignore is true if ANY of
        # MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE bits (vx,vy,vz) are set — so we must not
        # set VY_IGNORE when commanding vx,vz; otherwise guided_set_velocity() is skipped.
        m = mavutil.mavlink
        type_mask = (
            m.POSITION_TARGET_TYPEMASK_X_IGNORE
            | m.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | m.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | m.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | m.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | m.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | m.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )

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

            self.get_logger().info("Requesting SCALED_PRESSURE2 message stream from Pixhawk...")
            self.port.mav.command_long_send(
                self.port.target_system,
                self.port.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,  # confirmation
                mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2,  # message ID = 137
                20000,  # interval in microseconds (20ms = 50Hz)
                0, 0, 0, 0, 0,
            )
            self.get_logger().info("SCALED_PRESSURE2 request sent (interval=20ms)")
        
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
            # ArduSub GUIDED requires an external position source (ExternalNav
            # from our ODOMETRY stream, DVL, USBL, ...). Pre-arm will complain
            # until EK3 has a valid position estimate.
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
        # self.get_logger().info(
        #     f"Sending 6DOF command with control input: {control_input}"
        # )
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
    
    def reboot_cb(self, msg):
        """
        Called when a message arrives in the pixhawk/reboot_cmd topic. The message should contain a Bool (True to reboot, False to do nothing).
        """
        if msg.data:
            self.port.mav.command_long_send(
                self.port.target_system,
                self.port.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0,
                1, #1 to reboot, 2 for shutdown
                0,
                0,
                0,
                0,
                0,
                0,
            )
            self.get_logger().info("Sent reboot command to Pixhawk")
            self._file_logger.info("Sent reboot command to Pixhawk")

    def gps_origin_cb(self, msg: NavSatFix):
        if self._gps_origin_sent:
            return
        if msg.status.status < 0:
            self._gps_origin_valid_count = 0
            return
        self._gps_origin_valid_count += 1
        if self._gps_origin_valid_count < 5:
            return
        time_usec = (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec) // 1000
        self.port.mav.set_gps_global_origin_send(
            self.port.target_system,
            int(msg.latitude  * 1e7),
            int(msg.longitude * 1e7),
            int(msg.altitude  * 1e3),
            time_usec,
        )
        self.port.mav.set_home_position_send(
            self.port.target_system,
            int(msg.latitude * 1e7),
            int(msg.longitude * 1e7),
            int(msg.altitude * 1e3),
            0.0, 0.0, 0.0,         # x, y, z local NED (unknown)
            [1.0, 0.0, 0.0, 0.0],  # quaternion
            0.0, 0.0, 0.0,         # approach_x, approach_y, approach_z
            time_usec,
        )
        self._gps_origin_sent = True
        self.get_logger().info(
            f"GPS_GLOBAL_ORIGIN sent: lat={msg.latitude:.7f}, lon={msg.longitude:.7f}, alt={msg.altitude:.2f}m"
        )
        self._file_logger.info(
            f"GPS_GLOBAL_ORIGIN sent to Pixhawk: lat={msg.latitude:.7f}, lon={msg.longitude:.7f}, alt={msg.altitude:.2f}m"
        )

    # Currently sending position, velocity, attitude, rates. Later then seperated and different frequencies.
    def external_odom_cb(self, msg):
        """Stream nav_msgs/Odometry to FCU as MAVLink ODOMETRY (ArduPilot external nav)."""
        try:
            self._external_odom_cb_impl(msg)
        except Exception as e:
            self.get_logger().error(f"external_odom_cb failed: {e}", throttle_duration_sec=5.0)

    def _external_odom_cb_impl(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        max_hz = self.get_parameter("external_odom_max_rate_hz").get_parameter_value().double_value
        if max_hz > 0.0:
            min_interval_ns = int(1e9 / max_hz)
            if now_ns - self._external_odom_last_send_ns < min_interval_ns:
                return
        self._external_odom_last_send_ns = now_ns

        # Timestamp from message header in microseconds
        time_usec = (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec) // 1000

        # Position: ENU → NED
        x_ned =  msg.pose.pose.position.y
        y_ned =  msg.pose.pose.position.x
        z_ned = -msg.pose.pose.position.z

        # Orientation: ENU/FLU → NED/FRD via 3-quaternion chain
        #   q_ned_frd = q_ENU_to_NED ⊗ q_ros_flu ⊗ q_FLU_to_FRD
        # q_ENU_to_NED = (0, √0.5, √0.5, 0), q_FLU_to_FRD = (0, 1, 0, 0)
        _s = math.sqrt(0.5)
        w2 = msg.pose.pose.orientation.w
        x2 = msg.pose.pose.orientation.x
        y2 = msg.pose.pose.orientation.y
        z2 = msg.pose.pose.orientation.z

        # Step 1: q_tmp = q_ENU_to_NED ⊗ q_ros_flu
        w1, x1, y1, z1 = 0.0, _s, _s, 0.0
        wt = w1*w2 - x1*x2 - y1*y2 - z1*z2
        xt = w1*x2 + x1*w2 + y1*z2 - z1*y2
        yt = w1*y2 - x1*z2 + y1*w2 + z1*x2
        zt = w1*z2 + x1*y2 - y1*x2 + z1*w2

        # Step 2: q_ned_frd = q_tmp ⊗ (0, 1, 0, 0)  [FLU→FRD: 180° around x]
        q_frd = [-xt, wt, zt, -yt]

        # Linear velocity: body FLU -> body FRD
        vx =  msg.twist.twist.linear.x
        vy = -msg.twist.twist.linear.y
        vz = -msg.twist.twist.linear.z

        # Angular rates: body FLU -> body FRD
        rollspeed  =  msg.twist.twist.angular.x
        pitchspeed = -msg.twist.twist.angular.y
        yawspeed   = -msg.twist.twist.angular.z

        # Pose covariance: ENU→NED, permutation p=[1,0,2,3,4,5], signs s=[1,1,-1,1,-1,-1]
        # C_NED[i,j] = s[i]*s[j] * C_ENU[p[i]*6 + p[j]]
        _POSE_COV_MAP = [
            ( 7, 1), ( 6, 1), ( 8,-1), ( 9, 1), (10,-1), (11,-1),  # row 0 (North)
            ( 0, 1), ( 2,-1), ( 3, 1), ( 4,-1), ( 5,-1),            # row 1 (East)
            (14, 1), (15,-1), (16, 1), (17, 1),                      # row 2 (Down)
            (21, 1), (22,-1), (23,-1),                               # row 3 (roll)
            (28, 1), (29, 1),                                        # row 4 (pitch)
            (35, 1),                                                 # row 5 (yaw)
        ]

        # Twist covariance: body FLU→FRD, permutation p=[0,1,2,3,4,5], signs s=[1,-1,-1,1,-1,-1]
        # C_FRD[i,j] = s[i]*s[j] * C_FLU[i*6 + j]
        _TWIST_COV_MAP = [
            ( 0, 1), ( 1,-1), ( 2,-1), ( 3, 1), ( 4,-1), ( 5,-1),  # row 0 (fwd)
            ( 7, 1), ( 8, 1), ( 9,-1), (10, 1), (11, 1),            # row 1 (right)
            (14, 1), (15,-1), (16, 1), (17, 1),                      # row 2 (down)
            (21, 1), (22,-1), (23,-1),                               # row 3 (roll)
            (28, 1), (29, 1),                                        # row 4 (pitch)
            (35, 1),                                                 # row 5 (yaw)
        ]

        pose_cov  = [float(msg.pose.covariance[i])  * s for i, s in _POSE_COV_MAP]
        twist_cov = [float(msg.twist.covariance[i]) * s for i, s in _TWIST_COV_MAP]

        qual = self.get_parameter("external_odom_quality").get_parameter_value().integer_value
        qual = max(-1, min(100, int(qual)))

        m = mavutil.mavlink
        self.port.mav.odometry_send(
            time_usec,
            m.MAV_FRAME_LOCAL_FRD,
            m.MAV_FRAME_BODY_FRD,
            x_ned, y_ned, z_ned,
            q_frd,
            vx, vy, vz,
            rollspeed, pitchspeed, yawspeed,
            pose_cov,
            twist_cov,
            self._odom_reset_counter,
            m.MAV_ESTIMATOR_TYPE_VISION,
            qual,
        )


    """--------------------------------------------- main function ---------------------------------------------"""
def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeReceiver()
    rclpy.spin(node)  # Keeps the node running and processing callbacks
    rclpy.shutdown()
