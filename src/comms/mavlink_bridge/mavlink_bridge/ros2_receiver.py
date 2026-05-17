import queue
import threading
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import logging, os
import math
import time
from datetime import datetime
from config_pkg.constants import Logs, Comms, Ports

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil
from std_msgs.msg import String
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int16MultiArray
from mavros_msgs.msg import OverrideRCIn
from nav_msgs.msg import Odometry

#
from geometry_msgs.msg import Twist
from ublox_ubx_msgs.msg import UBXNavHPPosLLH

#
from rclpy.parameter import Parameter as RclpyParameter
from .pid_param_map import PID_PARAM_MAP, normalize_mavlink_param_id

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

        # Wait for a heartbeat so we know the target system IDs. Code can get stuck here meaning we didn't receive any heartbeat
        self.port.wait_heartbeat()
        self.get_logger().info(
            f"Heartbeat received from system {self.port.target_system}"
        )

        # Dedicated GCS heartbeat connection — separate from self.port so sysid=255
        # does not contaminate odometry/command messages.
        # ArduSub FS_GCS_ENABLE watches for MAV_TYPE_GCS heartbeats; if they stop
        # (e.g. Jetson crashes), ArduSub triggers its GCS failsafe automatically.
        self._gcs_port = mavutil.mavlink_connection(Comms.MAVLINK_ROUTER_TCP)
        self._gcs_port.mav.srcSystem = 255
        self._gcs_port.mav.srcComponent = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
        self.create_timer(1.0, self._gcs_heartbeat_cb)
        self.get_logger().info(
            "GCS heartbeat sender active (sysid=255, 1 Hz) — FS_GCS_ENABLE failsafe armed"
        )

        self.declare_parameter("external_odom_quality", 100)
        self.declare_parameter("external_odom_max_rate_hz", 30.0)

        # PID tuning parameters — declared early so Foxglove sees them before FC fetch completes.
        # Default 9999.0 is a sentinel: any param still at 9999.0 after startup failed to fetch from FC.
        self.param_map = PID_PARAM_MAP
        self._pending_mavlink_params = []
        self._mavlink_defer_timer = None
        self._reverting = False  # prevents revert calls from re-triggering MAVLink send
        self.declare_pid_parameter_defaults()
        self.add_on_set_parameters_callback(self.on_params_changed)

        # PID fetch from FC — uses the existing TCP connection (self.port).
        # _port_lock serialises sends from the fetch thread vs. the spin-thread drain loop.
        self._port_lock = threading.Lock()
        self._param_value_queue: queue.Queue = queue.Queue(maxsize=2000)
        self._pid_fetch_result_queue: queue.Queue = queue.Queue(maxsize=1)

        self._odom_reset_counter = 0
        self._external_odom_last_send_ns = 0

        self._gps_origin_sent = False
        self._gps_origin_valid_count = 0

        # Depth monitoring state (populated by MAVLink drain loop)
        self._vfrhud_alt = float(
            "nan"
        )  # VFR_HUD.alt   — baro depth (m, negative = submerged)
        self._vfrhud_climb = float(
            "nan"
        )  # VFR_HUD.climb — vertical velocity (m/s, negative = descending)
        self._nav_alt_error = float(
            "nan"
        )  # NAV_CONTROLLER_OUTPUT.alt_error (desired - actual, m)
        self._pid_desired = float("nan")  # PID_TUNING axis=4 fields
        self._pid_achieved = float("nan")
        self._pid_P = float("nan")
        self._pid_I = float("nan")
        self._pid_D = float("nan")
        # Timestamps of last receive — used to suppress stale publishes
        self._t_nav = 0.0  # last NAV_CONTROLLER_OUTPUT receive time (monotonic)
        self._t_pid = 0.0  # last PID_TUNING axis=4 receive time
        _STALE_S = 0.5  # treat data older than this as absent
        self._STALE_S = _STALE_S

        # Request VFR_HUD (74), NAV_CONTROLLER_OUTPUT (62) at 10 Hz
        # PID_TUNING (98) is enabled via GCS_PID_MASK param on the FC (already set to 8)
        for _msg_id in (74, 62):
            self.port.mav.command_long_send(
                self.port.target_system,
                self.port.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                _msg_id,
                100_000,
                0,
                0,
                0,
                0,
                0,
            )
        # Legacy fallback covering VFR_HUD + NAV_CONTROLLER_OUTPUT + PID_TUNING streams
        for _stream in (
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
        ):
            self.port.mav.request_data_stream_send(
                self.port.target_system,
                self.port.target_component,
                _stream,
                10,
                1,
            )

        self._depth_target_pub = self.create_publisher(
            Float32, "/pixhawk/DEPTH_TARGET", 10
        )
        self._depth_achieved_pub = self.create_publisher(
            Float32, "/pixhawk/DEPTH_ACHIEVED", 10
        )
        self._depth_velocity_pub = self.create_publisher(
            Float32, "/pixhawk/DEPTH_VELOCITY", 10
        )
        # [desired, achieved, P, I, D]
        self._pid_accz_pub = self.create_publisher(
            Float32MultiArray, "/pixhawk/PID_ACCZ", 10
        )

        # Drain incoming MAVLink at 50 Hz (non-blocking); publish at 5 Hz
        self.create_timer(0.02, self._mavlink_drain_cb)
        self.create_timer(0.20, self._depth_publish_cb)

        # One-shot trigger (1 s after init) → starts PID fetch background thread.
        # Apply-timer polls the result queue every 0.5 s and sets ROS params once done.
        self._pid_fetch_trigger = self.create_timer(1.0, self._start_pid_fetch_cb)
        self.create_timer(0.5, self._apply_fetched_pid_cb)

        # # Subscribe to RC override messages from ROS2 topic "pixhawk/rc_override" and then calls the rc_override_cb (translator) function when a message arrives. Accepts only RCIn messages
        # self.rc_override_subscriber = self.create_subscription(
        #     OverrideRCIn,
        #     "/pixhawk/rc_override",
        #     self.rc_override_cb,
        #     Comms.SUB_QOS_DEPTH,  # overrideRCIn is a 8 integer array, so the function currently only accepts that input type
        # )

        self.manual_control_subscriber = self.create_subscription(
            Int16MultiArray,
            "/pixhawk/manual_control",
            self.manual_control_cb,
            Comms.SUB_QOS_DEPTH,
        )

        self.odometry_subscriber = self.create_subscription(
            Odometry, "/odometry/filtered/local", self.ekf_odom_cb, Comms.SUB_QOS_DEPTH
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

        # LAND TEST: cmd_vel subscription disabled so no velocity setpoints reach the FC.
        # self.guided_setpoint_subscriber = self.create_subscription(
        #     Twist,  # Depending on the msg type from imports
        #     "/pixhawk/cmd_vel",
        #     self.cmd_vel_cb,
        #     Comms.SUB_QOS_DEPTH,
        # )

        # cmd_vel watchdog: ArduSub's GUIDED controller holds the last commanded
        # velocity until GUID_TIMEOUT (~3 s) elapses. We override that here: if
        # no cmd_vel arrives within 0.3 s, send one zero-velocity setpoint so the
        # sub stops within ~0.4 s of the upstream publisher going silent.
        self._CMD_VEL_TIMEOUT_S = 4 # 0.3
        self._cmd_vel_last_msg_t = 0.0
        self._cmd_vel_was_active = False
        self._cmd_vel_watchdog = self.create_timer(0.1, self._cmd_vel_watchdog_cb)

        self.gps_fix_subscriber = self.create_subscription(
            UBXNavHPPosLLH,
            "/ubx_nav_hp_pos_llh",
            self.gps_origin_cb,
            Comms.SUB_QOS_DEPTH,
        )

        self.get_logger().info("MavlinkBridgeReceiver: Node has been initialized")

    """--------------------------------------------- Callback functions for the subscribers ---------------------------------------------"""

    # def rc_override_cb(self, msg):
    #     """
    #     Called automatically when a message arrives on "pixhawk/rc_override" topic.
    #     Converts the ROS2 OverrideRCIn message to MAVLink RC_OVERRIDE and sends it to Pixhawk.
    #     """
    #     self.get_logger().info(f"Received ROS2 RC override: {msg.channels}")

    #     channels = msg.channels

    #     # Send MAVLink RC_CHANNELS_OVERRIDE message
    #     # Arguments: target_system, target_component and the different RCOverride values in channels
    #     self.port.mav.rc_channels_override_send(
    #         self.port.target_system,  # Target system ID
    #         self.port.target_component,  # Target component ID
    #         channels[0],
    #         channels[1],
    #         channels[2],
    #         channels[3],
    #         channels[4],
    #         channels[5],
    #         channels[6],
    #         channels[7],
    #     )

    def manual_control_cb(self, msg):
        """
        Called when a message arrives in the pixhawk/manual_control topic. The message should contain the surge, sway, heave, roll, pitch and yaw values for the manual control command.
        """
        if (self.pixhawk_mode in ("MANUAL", "STABILIZE", "ALT_HOLD")) and len(
            msg.data
        ) == 6:
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

            self.get_logger().info(
                "Requesting SCALED_PRESSURE2 message stream from Pixhawk..."
            )
            self.port.mav.command_long_send(
                self.port.target_system,
                self.port.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,  # confirmation
                mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2,  # message ID = 137
                20000,  # interval in microseconds (20ms = 50Hz)
                0,
                0,
                0,
                0,
                0,
            )
            self.get_logger().info("SCALED_PRESSURE2 request sent (interval=20ms)")
            self.pixhawk_mode = "ALT_HOLD"
            self.get_logger().info("Sent ALT_HOLD mode command")
            self._file_logger.info("Sent ALT_HOLD mode command")
        elif msg.data == "MANUAL":
            mode_id = 19
            self.port.mav.set_mode_send(
                self.port.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            self.pixhawk_mode = "MANUAL"
            self.get_logger().info("Sent MANUAL mode command")
            self._file_logger.info("Sent MANUAL mode command")
        elif msg.data == "STABILIZE":
            mode_id = 0
            self.port.mav.set_mode_send(
                self.port.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
            self.pixhawk_mode = "STABILIZE"
            self.get_logger().info("Sent STABILIZE mode command")
            self._file_logger.info("Sent STABILIZE mode command")
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

    # def send_4dof_command(self, control_input):
    #     """
    #     Input values: -1000 to 1000 (except heave, see below)
    #     """
    #     self._file_logger.info(
    #         f"Sending 4DOF command with control input: {control_input}"
    #     )
    #     surge, sway, heave, yaw = control_input
    #     self.port.mav.manual_control_send(
    #         self.port.target_system,
    #         int(surge),  # x: Forward/Back
    #         int(sway),  # y: Left/Right
    #         int(heave),  # z: Up/Down (range 0-1000, 500 is neutral)
    #         int(yaw),  # r: Yaw
    #         0,  # buttons bitmask
    #     )

    def send_6dof_command(self, control_input):
        """
        Forwards a 6-tuple to MAVLink MANUAL_CONTROL (transport only, no frame
        conversion happens here).

        Input convention — the caller (manual_control_node /
        manual_altitude_hold_control_node) must already have converted from
        ROS-FLU to MANUAL_CONTROL FRD before calling this:
            control_input[0] = surge   in [-1000, +1000], + = forward
            control_input[1] = sway    in [-1000, +1000], + = right (FRD)
            control_input[2] = heave   in [    0,  1000], 500 = neutral, > 500 = up
            control_input[3] = yaw     in [-1000, +1000], + = CW from above (FRD)
            control_input[4] = roll    in [-1000, +1000], + = roll right
            control_input[5] = pitch   in [-1000, +1000], + = nose up

        MAVLink MANUAL_CONTROL field meanings as ArduSub interprets them:
            x = surge, y = sway, z = heave, r = yaw,
            s = PITCH (extension 1), t = ROLL (extension 2)

        Note that MANUAL_CONTROL.s carries pitch and .t carries roll — so this
        function maps control_input[5] (pitch) → s and control_input[4] (roll)
        → t. Intentional and correct; do not "fix" by reordering.
        """
        self._file_logger.info(
            f"Sending 6DOF command with control input: {control_input}"
        )
        surge, sway, heave, yaw, roll, pitch = control_input
        self.port.mav.manual_control_send(
            self.port.target_system,
            int(surge),  # x  = surge
            int(sway),   # y  = sway
            int(heave),  # z  = heave (0-1000, 500 = neutral)
            int(yaw),    # r  = yaw
            0,           # buttons
            0,           # buttons2
            3,           # enabled_extensions = 0b11 → enable s and t fields
            int(pitch),  # s  = pitch  (MANUAL_CONTROL.s carries pitch in ArduSub)
            int(roll),   # t  = roll   (MANUAL_CONTROL.t carries roll  in ArduSub)
        )

    def reboot_cb(self, msg):
        """
        Called when a message arrives in the pixhawk/reboot_cmd topic. The message should contain a Bool (True to reboot, False to do nothing).
        NOTE: ArduSub rejects this command if the vehicle is armed (MAV_RESULT_DENIED).
        Always disarm before rebooting.
        """
        if msg.data:
            self.port.mav.command_long_send(
                self.port.target_system,
                self.port.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0,
                1,  # 1 to reboot, 2 for shutdown
                0,
                0,
                0,
                0,
                0,
                0,
            )
            self.get_logger().info(
                "Sent reboot command to Pixhawk (vehicle must be disarmed or Pixhawk will deny)"
            )
            self._file_logger.info("Sent reboot command to Pixhawk")

            ack = self.port.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
            if ack is None:
                self.get_logger().warn("Reboot: no ACK received from Pixhawk within 3s")
                self._file_logger.warning("Reboot: no ACK received")
            elif ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self.get_logger().error(
                    f"Reboot rejected by Pixhawk (MAV_RESULT={ack.result}). Is the vehicle disarmed?"
                )
                self._file_logger.error(f"Reboot rejected: MAV_RESULT={ack.result}")

    def ekf_odom_cb(self, msg):
        """
        Receives filtered odometry from /odometry/filtered/local (ENU/FLU, ROS convention)
        and forwards as MAVLink ODOMETRY to Pixhawk (NED/FRD, MAVLink convention).

        Frame conversions applied:
          Position:      ENU→NED  x_NED=y_ENU,  y_NED=x_ENU,  z_NED=-z_ENU
          Orientation:   q_NED = q_ENU_to_NED ⊗ q_ENU
                         q_ENU_to_NED = (w=0, x=√0.5, y=√0.5, z=0)
          Velocity:      body FLU→FRD  vx unchanged, vy=-vy, vz=-vz
          Angular rates: body FLU→FRD  roll unchanged, pitch and yaw negated
        """
        # Limit send rate to avoid flooding the serial port.
        now_ns = self.get_clock().now().nanoseconds
        max_hz = (
            self.get_parameter("external_odom_max_rate_hz")
            .get_parameter_value()
            .double_value
        )
        if max_hz > 0.0:
            min_interval_ns = int(1e9 / max_hz)
            if now_ns - self._external_odom_last_send_ns < min_interval_ns:
                return
        self._external_odom_last_send_ns = now_ns

        # Timestamp from message header in microseconds
        time_usec = (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec) // 1000

        # Position: ENU → NED
        x = msg.pose.pose.position.y
        y = msg.pose.pose.position.x
        z = -msg.pose.pose.position.z

        # Orientation: apply ENU→NED rotation then express in MAVLink [w,x,y,z] order
        # q_ENU_to_NED = (w=0, x=√0.5, y=√0.5, z=0)
        _s = math.sqrt(0.5)
        w2 = msg.pose.pose.orientation.w
        x2 = msg.pose.pose.orientation.x
        y2 = msg.pose.pose.orientation.y
        z2 = msg.pose.pose.orientation.z

        w1, x1, y1, z1 = 0.0, _s, _s, 0.0
        wt = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        xt = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        yt = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        zt = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        # Step 2: q_ned_frd = q_tmp (0, 1, 0, 0)  [FLU→FRD: 180° around x]
        q = [-xt, wt, zt, -yt]

        """OLD VERSION"""
        # q = [wt, xt, yt, zt]

        # Linear velocity: body FLU → body FRD (robot_localization outputs body-frame twist)
        vx = msg.twist.twist.linear.x
        vy = -msg.twist.twist.linear.y
        vz = -msg.twist.twist.linear.z

        # Angular rates: body FLU → body FRD (roll unchanged, pitch and yaw negated)
        rollspeed = msg.twist.twist.angular.x
        pitchspeed = -msg.twist.twist.angular.y
        yawspeed = -msg.twist.twist.angular.z

        # Pose covariance: ENU→NED, permutation p=[1,0,2,3,4,5], signs s=[1,1,-1,1,-1,-1]
        # C_NED[i,j] = s[i]*s[j] * C_ENU[p[i]*6 + p[j]]
        _POSE_COV_MAP = [
            (7, 1),
            (6, 1),
            (8, -1),
            (9, 1),
            (10, -1),
            (11, -1),  # row 0 (North)
            (0, 1),
            (2, -1),
            (3, 1),
            (4, -1),
            (5, -1),  # row 1 (East)
            (14, 1),
            (15, -1),
            (16, 1),
            (17, 1),  # row 2 (Down)
            (21, 1),
            (22, -1),
            (23, -1),  # row 3 (roll)
            (28, 1),
            (29, 1),  # row 4 (pitch)
            (35, 1),  # row 5 (yaw)
        ]

        # Twist covariance: body FLU→FRD, permutation p=[0,1,2,3,4,5], signs s=[1,-1,-1,1,-1,-1]
        # C_FRD[i,j] = s[i]*s[j] * C_FLU[i*6 + j]
        _TWIST_COV_MAP = [
            (0, 1),
            (1, -1),
            (2, -1),
            (3, 1),
            (4, -1),
            (5, -1),  # row 0 (fwd)
            (7, 1),
            (8, 1),
            (9, -1),
            (10, 1),
            (11, 1),  # row 1 (right)
            (14, 1),
            (15, -1),
            (16, 1),
            (17, 1),  # row 2 (down)
            (21, 1),
            (22, -1),
            (23, -1),  # row 3 (roll)
            (28, 1),
            (29, 1),  # row 4 (pitch)
            (35, 1),  # row 5 (yaw)
        ]

        pose_cov = [float(msg.pose.covariance[i]) * s for i, s in _POSE_COV_MAP]
        twist_cov = [float(msg.twist.covariance[i]) * s for i, s in _TWIST_COV_MAP]

        qual = (
            self.get_parameter("external_odom_quality")
            .get_parameter_value()
            .integer_value
        )
        qual = max(-1, min(100, int(qual)))

        m = mavutil.mavlink
        self.port.mav.odometry_send(
            time_usec,
            m.MAV_FRAME_LOCAL_FRD,
            m.MAV_FRAME_BODY_FRD,
            x,
            y,
            z,
            q,
            vx,
            vy,
            vz,
            rollspeed,
            pitchspeed,
            yawspeed,
            pose_cov,
            twist_cov,
            self._odom_reset_counter,
            m.MAV_ESTIMATOR_TYPE_VISION,
            qual,
        )

    def _mavlink_drain_cb(self):
        """Drain up to 20 incoming MAVLink messages per 20 ms tick (non-blocking)."""
        with self._port_lock:
            for _ in range(20):
                msg = self.port.recv_match(blocking=False)
                if msg is None:
                    break
                t = msg.get_type()
                if t == "VFR_HUD":
                    self._vfrhud_alt = msg.alt
                    self._vfrhud_climb = msg.climb
                elif t == "NAV_CONTROLLER_OUTPUT":
                    self._nav_alt_error = msg.alt_error
                    self._t_nav = time.monotonic()
                elif t == "PID_TUNING" and getattr(msg, "axis", None) == 4:
                    self._pid_desired = msg.desired
                    self._pid_achieved = msg.achieved
                    self._pid_P = msg.P
                    self._pid_I = msg.I
                    self._pid_D = msg.D
                    self._t_pid = time.monotonic()
                elif t == "PARAM_VALUE":
                    try:
                        self._param_value_queue.put_nowait(msg)
                    except queue.Full:
                        pass

    def _depth_publish_cb(self):
        """Publish depth topics at 5 Hz. Suppresses stale controller data."""
        now = time.monotonic()
        nav_fresh = (now - self._t_nav) < self._STALE_S
        pid_fresh = (now - self._t_pid) < self._STALE_S

        if math.isfinite(self._vfrhud_alt):
            msg = Float32()
            msg.data = float(self._vfrhud_alt)
            self._depth_achieved_pub.publish(msg)

        if (
            self.pixhawk_mode == "ALT_HOLD"
            and nav_fresh
            and math.isfinite(self._vfrhud_alt)
            and math.isfinite(self._nav_alt_error)
        ):
            msg = Float32()
            # alt_error = desired - actual  →  desired = actual + alt_error
            msg.data = float(self._vfrhud_alt + self._nav_alt_error)
            self._depth_target_pub.publish(msg)

        if math.isfinite(self._vfrhud_climb):
            msg = Float32()
            msg.data = float(self._vfrhud_climb)
            self._depth_velocity_pub.publish(msg)

        if pid_fresh and math.isfinite(self._pid_desired):
            msg = Float32MultiArray()
            msg.data = [
                float(self._pid_desired),
                float(self._pid_achieved),
                float(self._pid_P),
                float(self._pid_I),
                float(self._pid_D),
            ]
            self._pid_accz_pub.publish(msg)

    """--------------------------------------------- PID fetch from FC (startup) -----------------------------------------------"""

    def _start_pid_fetch_cb(self):
        """One-shot timer callback: cancel self, then start the background fetch thread."""
        self._pid_fetch_trigger.cancel()
        self._pid_fetch_trigger = None
        threading.Thread(target=self._thread_fetch_pid_params, daemon=True).start()

    def _thread_fetch_pid_params(self):
        """Fetch PID gains from the FC using the receiver's existing TCP connection.

        Runs in a daemon thread. PARAM_VALUE messages are received by _mavlink_drain_cb
        and forwarded to _param_value_queue; this thread only sends requests (under the
        port lock) and reads from the queue.
        """
        fl = self._file_logger
        fetched = {}
        try:
            upper_to_ros = {m.upper(): r for r, m in self.param_map.items()}
            needed = set(upper_to_ros.keys())

            fl.info("PID fetch: sending PARAM_REQUEST_LIST over TCP ...")
            with self._port_lock:
                self.port.mav.param_request_list_send(
                    self.port.target_system, self.port.target_component
                )

            list_deadline = time.monotonic() + 45.0
            idle_timeouts = 0
            while needed and time.monotonic() < list_deadline:
                try:
                    msg = self._param_value_queue.get(timeout=0.5)
                except queue.Empty:
                    idle_timeouts += 1
                    if idle_timeouts >= 10:  # 5 s of silence → give up on list
                        fl.warning(
                            "PID fetch: no PARAM_VALUE for 5 s, moving to retries"
                        )
                        break
                    continue
                idle_timeouts = 0
                pid = normalize_mavlink_param_id(msg.param_id).upper()
                if pid in needed:
                    ros_name = upper_to_ros[pid]
                    fetched[ros_name] = float(msg.param_value)
                    needed.discard(pid)
                    fl.info(f"PID from FC: {pid} = {msg.param_value}")

            # Individual retries for anything still missing after the full list
            for ros_name, mav_name in self.param_map.items():
                if ros_name in fetched:
                    continue
                want = mav_name.strip().upper()
                fl.info(f"PID fetch: retrying {mav_name} individually ...")
                with self._port_lock:
                    self.port.param_fetch_one(mav_name)
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline:
                    try:
                        msg = self._param_value_queue.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    if normalize_mavlink_param_id(msg.param_id).upper() == want:
                        fetched[ros_name] = float(msg.param_value)
                        fl.info(f"PID from FC (retry): {mav_name} = {msg.param_value}")
                        break

            for ros_name in self.param_map:
                if ros_name not in fetched:
                    fl.error(
                        f"PID fetch FAILED for {self.param_map[ros_name]} ({ros_name});"
                        " sentinel 9999.0"
                    )
                    fetched[ros_name] = 9999.0

        except Exception as e:
            fl.error(f"PID fetch thread error: {e}")
            for ros_name in self.param_map:
                fetched.setdefault(ros_name, 9999.0)

        try:
            self._pid_fetch_result_queue.put_nowait(fetched)
        except queue.Full:
            pass
        fl.info(f"PID fetch: complete ({len(fetched)} params)")

    def _apply_fetched_pid_cb(self):
        """Spin-thread timer: apply fetched PID params to ROS params without pushing back to FC."""
        try:
            fetched = self._pid_fetch_result_queue.get_nowait()
        except queue.Empty:
            return
        params_to_set = [
            RclpyParameter(name, RclpyParameter.Type.DOUBLE, value)
            for name, value in fetched.items()
        ]
        self._reverting = True  # skip on_params_changed → FC push
        try:
            self.set_parameters(params_to_set)
        finally:
            self._reverting = False
        self.get_logger().info(f"Applied {len(fetched)} PID params fetched from FC")

    """------------------------------------------- pid gains as rosparams functions --------------------------------------------"""

    def declare_pid_parameter_defaults(self):
        """Declare tuning parameters with sentinel 9999.0 so failed fetches are immediately visible."""
        for ros_name in self.param_map:
            self.declare_parameter(ros_name, 9999.0)

    def on_params_changed(self, params):
        """Queue tuning parameter changes for deferred send to FC."""
        if self._reverting:
            return SetParametersResult(successful=True)
        for p in params:
            if p.name in self.param_map:
                try:
                    old_val = float(self.get_parameter(p.name).value)
                except Exception:
                    old_val = 9999.0
                self._pending_mavlink_params.append(
                    (self.param_map[p.name], float(p.value), p.name, old_val)
                )
        if self._pending_mavlink_params and self._mavlink_defer_timer is None:
            self._mavlink_defer_timer = self.create_timer(
                0.02, self._flush_mavlink_param_queue
            )
        return SetParametersResult(successful=True)

    def _flush_mavlink_param_queue(self):
        """Send each queued param_set without blocking the spin thread."""
        if self._mavlink_defer_timer is not None:
            self._mavlink_defer_timer.cancel()
            self._mavlink_defer_timer = None
        batch = self._pending_mavlink_params
        self._pending_mavlink_params = []
        for mav_param_id, new_val, ros_name, old_val in batch:
            self.get_logger().info(f"Setting ArduSub {mav_param_id} = {new_val}")
            self.port.mav.param_set_send(
                self.port.target_system,
                self.port.target_component,
                mav_param_id.encode("utf-8"),
                new_val,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
            self.get_logger().info(f"Queued ArduSub param write for {mav_param_id}")

    def gps_origin_cb(self, msg: UBXNavHPPosLLH):
        if self._gps_origin_sent:
            return
        if msg.invalid_lon or msg.invalid_lat or msg.invalid_hmsl:
            self._gps_origin_valid_count = 0
            return
        self._gps_origin_valid_count += 1
        if self._gps_origin_valid_count < 5:
            return

        # UBX-NAV-HPPOSLLH units:
        #   lat, lon:  deg * 1e7 (already MAVLink degE7 scale)
        #   hmsl:      mm, height above mean sea level (already MAVLink altitude scale)
        lat_e7 = int(msg.lat)
        lon_e7 = int(msg.lon)
        alt_mm = int(msg.hmsl)

        time_usec = (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec) // 1000
        self.port.mav.set_gps_global_origin_send(
            self.port.target_system,
            lat_e7,
            lon_e7,
            alt_mm,
            time_usec,
        )
        self.port.mav.set_home_position_send(
            self.port.target_system,
            lat_e7,
            lon_e7,
            alt_mm,
            0.0,
            0.0,
            0.0,  # x, y, z local NED (unknown)
            [1.0, 0.0, 0.0, 0.0],  # quaternion
            0.0,
            0.0,
            0.0,  # approach_x, approach_y, approach_z
            time_usec,
        )
        self._gps_origin_sent = True
        lat_deg = lat_e7 * 1e-7
        lon_deg = lon_e7 * 1e-7
        alt_m = alt_mm * 1e-3
        self.get_logger().info(
            f"GPS_GLOBAL_ORIGIN sent (MSL): lat={lat_deg:.7f}, lon={lon_deg:.7f}, alt={alt_m:.2f}m"
        )
        self._file_logger.info(
            f"GPS_GLOBAL_ORIGIN sent to Pixhawk (MSL): lat={lat_deg:.7f}, lon={lon_deg:.7f}, alt={alt_m:.2f}m"
        )

    def cmd_vel_cb(self, msg):
        # LAND TEST: body of cmd_vel_cb commented out so no velocity setpoints reach the FC.
        return
        # # msg is geometry_msgs.msg.Twist
        # # ArduSub needs GUIDED mode for velocity setpoints
        # if self.pixhawk_mode != "GUIDED":
        #     return
        #
        # # 1. Map ROS FLU body frame -> ArduSub MAV_FRAME_BODY_FRD.
        # # Input convention is REP-103 FLU (matches pure_pursuit_controller_3d):
        # surge    = float(msg.linear.x)   # FLU forward -> negative vx
        # heave    = -float(msg.linear.z)   # FLU up      -> -down (FRD spec)
        # yaw_rate = -float(msg.angular.z)  # FLU CCW     -> -CW   (FRD spec)
        #
        # # 2. Type mask (ArduSub GCS_MAVLink_Sub.cpp): vel_ignore is true if ANY of
        # # MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE bits (vx,vy,vz) are set — so we must not
        # # set VY_IGNORE when commanding vx,vz; otherwise guided_set_velocity() is skipped.
        # m = mavutil.mavlink
        # type_mask = (
        #     m.POSITION_TARGET_TYPEMASK_X_IGNORE
        #     | m.POSITION_TARGET_TYPEMASK_Y_IGNORE
        #     | m.POSITION_TARGET_TYPEMASK_Z_IGNORE
        #     | m.POSITION_TARGET_TYPEMASK_AX_IGNORE
        #     | m.POSITION_TARGET_TYPEMASK_AY_IGNORE
        #     | m.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        #     | m.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        # )
        #
        # # 3. Send to Pixhawk in MAV_FRAME_BODY_FRD ("Forward" relative to nose).
        # self.port.mav.set_position_target_local_ned_send(
        #     0,  # time_boot_ms
        #     self.port.target_system,
        #     self.port.target_component,
        #     mavutil.mavlink.MAV_FRAME_BODY_FRD,  # Frame: Body-Relative
        #     type_mask,
        #     0.0,
        #     0.0,
        #     0.0,  # Position (ignored)
        #     surge,
        #     0.0,
        #     0.0,  # Velocities (m/s)
        #     0.0,
        #     0.0,
        #     0.0,  # Acceleration (ignored)
        #     0.0,  # Yaw Angle (ignored)
        #     yaw_rate,  # Yaw Rate (rad/s)
        # )
        #
        # # Refresh watchdog: arms the timeout zero-send when cmd_vel goes silent.
        # self._cmd_vel_last_msg_t = self.get_clock().now().nanoseconds * 1e-9
        # self._cmd_vel_was_active = True

    def _cmd_vel_watchdog_cb(self):
        """If no /pixhawk/cmd_vel arrives within _CMD_VEL_TIMEOUT_S, send one
        zero-velocity setpoint and disarm. Re-arms automatically when cmd_vel
        resumes. Bypasses ArduSub's ~3 s GUID_TIMEOUT so the sub stops within
        ~0.4 s of the upstream publisher going silent (Ctrl+C, controller
        crash, mode change, mission completion)."""
        return
        if not self._cmd_vel_was_active:
            return
        if self.pixhawk_mode != "GUIDED":
            self._cmd_vel_was_active = False
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._cmd_vel_last_msg_t < self._CMD_VEL_TIMEOUT_S:
            return
        m = mavutil.mavlink
        type_mask = (
            m.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | m.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | m.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | m.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | m.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | m.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | m.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | m.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        self.port.mav.set_position_target_local_ned_send(
            0,
            self.port.target_system,
            self.port.target_component,
            m.MAV_FRAME_BODY_OFFSET_NED,
            type_mask,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        self._cmd_vel_was_active = False
        self.get_logger().info("cmd_vel watchdog: timeout, sent zero-velocity setpoint")

    def _gcs_heartbeat_cb(self):
        """Send 1 Hz GCS heartbeat. ArduSub FS_GCS_ENABLE failsafes if these stop arriving."""
        self._gcs_port.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,  # base_mode
            0,  # custom_mode
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def _send_disarm(self):
        """Send a disarm command to the Pixhawk. Called on node shutdown."""
        try:
            self.port.mav.command_long_send(
                self.port.target_system,
                self.port.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,  # 0 = disarm
                21196,  # force disarm, bypasses safety checks
                0,
                0,
                0,
                0,
                0,
            )
            self.get_logger().info("Shutdown disarm command sent to Pixhawk")
            self._file_logger.info("Shutdown disarm command sent to Pixhawk")
        except Exception as e:
            self._file_logger.error(f"Shutdown disarm failed: {e}")

    def destroy_node(self):
        self._send_disarm()
        super().destroy_node()

    """--------------------------------------------- main function ---------------------------------------------"""


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeReceiver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
