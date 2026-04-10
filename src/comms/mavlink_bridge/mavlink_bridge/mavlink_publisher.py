import math
import os
import queue
import time
import logging
import threading
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from pymavlink import mavutil
from rcl_interfaces.srv import SetParameters as SetParametersSrv
from datetime import datetime
from std_msgs.msg import Int16MultiArray, Float32
from mavros_msgs.msg import State, RCIn, ManualControl  # HEARTBEAT  # RC_CHANNELS
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import (
    Imu,  # ATTITUDE
    BatteryState,  # BATTERY_STATUS
    FluidPressure,  # SCALED_PRESSURE (depth)
)
from config_pkg.constants import Comms
from custom_msgs.msg import PidTuning
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from .pid_param_map import PID_PARAM_MAP, normalize_mavlink_param_id

# MAVLink PID_TUNING sends one axis per message (axis, desired, achieved). ArduPilot axis IDs 0–5
# map to roll, pitch, yaw, thrust/depth, then position/translation (vehicle-specific).
_PID_TUNING_AXIS_TO_FIELDS = (
    ("roll_des", "roll_ach"),
    ("pitch_des", "pitch_ach"),
    ("yaw_des", "yaw_ach"),
    ("depth_des", "depth_ach"),
    ("pos_x_des", "pos_x_ach"),
    ("pos_y_des", "pos_y_ach"),
)
# specifies the directory where logs are saved and the name of the log files
log_dir = os.path.expanduser("~/polaris_logs")
os.makedirs(log_dir, exist_ok=True)
log_file = os.path.join(log_dir, f"mavlink_{datetime.now():%Y%m%d_%H%M%S}.log")


class DualLogger:
    def __init__(self, ros_logger, file_logger):
        self._ros = ros_logger
        self._file = file_logger

    def info(self, msg):
        self._ros.info(msg)
        self._file.info(msg)

    def debug(self, msg):
        self._ros.debug(msg)
        self._file.debug(msg)

    def warning(self, msg):
        self._ros.warning(msg)
        self._file.warning(msg)

    def error(self, msg):
        self._ros.error(msg)
        self._file.error(msg)


class MavlinkBridgeSender(Node):
    """
    This node is supposed to publish mavlink data directly from the pixhawk to ROS2 topics and so that it can be heart by the output monitor
    """

    def __init__(self):
        super().__init__("mavlink_bridge_publisher")

        self._file_logger = logging.getLogger(
            "mavlink"
        )  # creates or gets logger instance

        # set level defines from what message type onwards the message is logged. the different levels are:
        # Logging levels (lowest → highest):
        # DEBUG    = detailed diagnostic data (high-frequency sensor + internal state)
        # INFO     = normal operational messages (mode changes, summaries)
        # WARNING  = unexpected situations that do not stop operation
        # ERROR    = recoverable failures
        # CRITICAL = unrecoverable failures; system may be unusable
        self._file_logger.setLevel(logging.INFO)

        # the handler actually writes to the specified file
        file_handler = logging.FileHandler(log_file)
        formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
        file_handler.setFormatter(formatter)
        self._file_logger.addHandler(file_handler)

        self.ros_logger = self.get_logger()  # get_logger is the ros logger object

        self.logger = DualLogger(self.ros_logger, self._file_logger)

        self.port = mavutil.mavlink_connection(
            f"{Comms.JETSON_IP_ADDRESS}:14600"
        )  # UDP connection to companion computer (BlueOS)
        # Serial is reserved for ros2_receiver (one process per tty — shared reads corrupt MAVLink).

        # Do not block __init__ forever: without a UDP heartbeat, nothing below ran → no ROS publishers.
        try:
            try:
                self.port.wait_heartbeat(timeout=60.0)
            except TypeError:
                self.port.wait_heartbeat()
        except Exception as e:
            self.logger.error(
                f"No UDP MAVLink heartbeat ({e}). "
                "Check BlueOS/MAVProxy and JETSON_IP; using target_system=1."
            )
            self.port.target_system = 1
            self.port.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        else:
            self.logger.info(f"UDP MAVLink heartbeat: system {self.port.target_system}")
            if self.port.target_component == 0:
                self.port.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1

        # Lock: main spin (mavlink_callback) vs PID param fetch thread.
        self._mav_lock = threading.Lock()
        self._pid_queue: queue.Queue = queue.Queue(maxsize=1)
        self._pid_apply_timer = None
        # PID push to ros2_receiver: never block spin() for long waits (hurts Foxglove param queries).
        self._pid_pending_fetched = None  # dict[str, float] | None
        self._pid_set_cli = None
        self._pid_set_cli_srv = None  # last /node/set_parameters name
        self._pid_chunk_batches = []
        self._pid_chunk_values = None
        self._pid_chunk_receiver = None

        self.last_heartbeat_time = None

        '''TEST: Requesting data stream for debugging'''
        self.get_logger().warn("Requesting POSITION_TARGET_LOCAL_NED message stream from Pixhawk...")
        self.port.mav.command_long_send(
            self.port.target_system, self.port.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            85,       # The MAVLink message ID for POSITION_TARGET_LOCAL_NED
            100000,   # Interval in microseconds (10Hz)
            0, 0, 0, 0, 0 # Unused parameters
        )
        self.get_logger().warn("POSITION_TARGET_LOCAL_NED request sent (interval=100ms)")

        self.heartbeat_publisher = self.create_publisher(
            State, "/pixhawk/heartbeat", 10
        )

        self.attitude_publisher = self.create_publisher(
            Vector3, "/pixhawk/attitude", 10
        )

        # self.rc_channel_publisher = self.create_publisher(
        #     RCIn, "/pixhawk/rc_channels", 10
        # )

        self.battery_consumed_publisher = self.create_publisher(
            Float32, "/pixhawk/battery_consumed", 10
        )

        self.battery_publisher = self.create_publisher(
            BatteryState, "/pixhawk/battery", 10
        )

        self.scaled_pressure_publisher = self.create_publisher(
            FluidPressure, "/pixhawk/scaled_pressure", 10
        )

        self.manual_control_publisher = self.create_publisher(
            Int16MultiArray, "/pixhawk/out/manual_control", 10
        )

        # Single message: all 6 controller pairs (desired vs achieved) for plotting.
        self._pid_tuning = PidTuning()
        self.pid_tuning_publisher = self.create_publisher(
            PidTuning, "/pixhawk/PID_TUNING", 10
        )

        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )

        # Dynamic battery diagnostic thresholds (can be changed at runtime via ros2 param set)
        self.declare_parameter("battery_min_voltage", 12.0)
        self.declare_parameter("battery_max_voltage", 16.8)
        self.battery_min_voltage = float(
            self.get_parameter("battery_min_voltage").value
        )
        self.battery_max_voltage = float(
            self.get_parameter("battery_max_voltage").value
        )
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.declare_parameter("sync_fc_pid_to_receiver", True)
        self.declare_parameter("receiver_node_for_pid_sync", "ros2_receiver")
        # Small batches keep ros2_receiver responsive to foxglove_bridge list/get_parameters (~5s timeout).
        self.declare_parameter("receiver_pid_push_chunk_size", 6)
        # If BlueOS relays a HEARTBEAT with MAV_AUTOPILOT_INVALID, set false to still publish /pixhawk/heartbeat.
        self.declare_parameter("heartbeat_skip_invalid_autopilot", True)

        # Request MANUAL_CONTROL messages at 10 Hz
        self.logger.info("Requesting MANUAL_CONTROL message stream from Pixhawk...")
        self.port.mav.command_long_send(
            self.port.target_system,
            self.port.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,  # confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_MANUAL_CONTROL,  # message ID = 69
            100000,  # interval in microseconds (100ms = 10Hz)
            0,
            0,
            0,
            0,
            0,
        )
        self.logger.info(
            f"MANUAL_CONTROL request sent (msg_id={mavutil.mavlink.MAVLINK_MSG_ID_MANUAL_CONTROL}, interval=100ms)"
        )

        # ADDED: Request SCALED_PRESSURE2 messages at 50 Hz
        self.logger.info("Requesting SCALED_PRESSURE2 message stream from Pixhawk...")
        self.port.mav.command_long_send(
            self.port.target_system,
            self.port.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,  # confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2,  # message ID = 137
            20000,  # interval in microseconds (20ms = 50Hz)
            0, 0, 0, 0, 0,
        )
        self.logger.info("SCALED_PRESSURE2 request sent (interval=20ms)")

        # ADDED: Request ATTITUDE messages at 50 Hz
        self.logger.info("Requesting ATTITUDE message stream from Pixhawk...")
        self.port.mav.command_long_send(
            self.port.target_system,
            self.port.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,  # confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,  # message ID = 30
            20000,  # interval in microseconds (20ms = 50Hz)
            0, 0, 0, 0, 0,
        )
        self.logger.info("ATTITUDE request sent (interval=20ms)")

        # Request PID_TUNING messages at 20 Hz
        pid_tuning_msg_id = getattr(mavutil.mavlink, "MAVLINK_MSG_ID_PID_TUNING", 194)
        self.logger.info("Requesting PID_TUNING message stream from Pixhawk...")
        self.port.mav.command_long_send(
            self.port.target_system,
            self.port.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,  # confirmation
            pid_tuning_msg_id,
            50000,  # interval in microseconds (50ms = 20Hz)
            0,
            0,
            0,
            0,
            0,
        )
        self.logger.info(
            f"PID_TUNING request sent (msg_id={pid_tuning_msg_id}, interval=50ms)"
        )


        self.msg_type_counter = {
            "HEARTBEAT": 0,
            "ATTITUDE": 0,
            "RC_CHANNELS": 0,
            "BATTERY_STATUS": 0,
            # "SCALED_PRESSURE2": 0,
            "MANUAL_CONTROL": 0,
        }
        self.msg_type_counter_interval = 10

        # Telemetry: single-threaded spin() — all ROS calls stay on the executor thread.
        self.timer = self.create_timer(0.02, self.mavlink_callback)  # 50 Hz
        self.create_timer(1.0, self.heartbeat_checker_cb)

        self._pid_sync_timer = None
        if self.get_parameter("sync_fc_pid_to_receiver").value:
            self._pid_sync_timer = self.create_timer(0.5, self._deferred_pid_sync_cb)
        else:
            self.logger.info("sync_fc_pid_to_receiver=false; ros2_receiver keeps PID defaults")

    def _deferred_pid_sync_cb(self):
        """MAVLink PID fetch runs in a daemon thread; ROS apply runs on the main thread via queue."""
        if self._pid_sync_timer is not None:
            self._pid_sync_timer.cancel()
            self._pid_sync_timer = None
        self._pid_apply_timer = self.create_timer(0.2, self._apply_pid_from_queue)
        threading.Thread(target=self._thread_fetch_pid_params, daemon=True).start()

    def _thread_fetch_pid_params(self):
        """Pymavlink only (no rclpy). Log via file logger — safe from non-ROS threads."""
        fl = self._file_logger
        fetched = {}
        try:
            fl.info("PID fetch thread: requesting PARAM_VALUE from FC (UDP) ...")

            upper_to_ros = {m.upper(): r for r, m in PID_PARAM_MAP.items()}
            needed = set(upper_to_ros.keys())

            with self._mav_lock:
                self.port.mav.param_request_list_send(
                    self.port.target_system, self.port.target_component
                )
            list_deadline = time.monotonic() + 45.0
            idle_timeouts = 0
            while needed and time.monotonic() < list_deadline:
                with self._mav_lock:
                    msg = self.port.recv_match(
                        type="PARAM_VALUE", blocking=True, timeout=0.1
                    )
                if msg is None:
                    idle_timeouts += 1
                    if idle_timeouts >= 20:
                        break
                    continue
                idle_timeouts = 0
                pid = normalize_mavlink_param_id(msg.param_id).upper()
                if pid in needed:
                    ros_name = upper_to_ros[pid]
                    fetched[ros_name] = float(msg.param_value)
                    needed.discard(pid)
                    fl.info(f"PID from FC {pid} = {msg.param_value}")

            for ros_name, mav_name in PID_PARAM_MAP.items():
                if ros_name in fetched:
                    continue
                want = mav_name.strip().upper()
                with self._mav_lock:
                    self.port.param_fetch_one(mav_name)
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    with self._mav_lock:
                        msg = self.port.recv_match(
                            type="PARAM_VALUE", blocking=True, timeout=0.1
                        )
                    if msg is None:
                        continue
                    if normalize_mavlink_param_id(msg.param_id).upper() == want:
                        fetched[ros_name] = float(msg.param_value)
                        fl.info(f"PID from FC {mav_name} = {msg.param_value}")
                        break
        except Exception as e:
            fl.error(f"PID fetch thread failed: {e}")
            fetched = {}

        try:
            self._pid_queue.put_nowait(fetched)
        except queue.Full:
            pass
        fl.info(f"PID fetch thread: done ({len(fetched)} params).")

    def _stop_pid_apply_timer(self):
        if self._pid_apply_timer is not None:
            self._pid_apply_timer.cancel()
            self._pid_apply_timer = None

    def _apply_pid_from_queue(self):
        """Cooperative: short wait_for_service only; chunk set_parameters so ros2_receiver stays responsive."""
        try:
            while True:
                self._pid_pending_fetched = self._pid_queue.get_nowait()
        except queue.Empty:
            pass

        if self._pid_pending_fetched is None:
            return

        fetched = self._pid_pending_fetched
        if not fetched:
            self.logger.warning(
                "No PID parameters received from FC; ros2_receiver keeps defaults"
            )
            self._pid_pending_fetched = None
            self._stop_pid_apply_timer()
            return

        receiver_node = self.get_parameter("receiver_node_for_pid_sync").value
        srv_name = f"/{receiver_node}/set_parameters"

        if self._pid_set_cli is None or self._pid_set_cli_srv != srv_name:
            self._pid_set_cli = self.create_client(SetParametersSrv, srv_name)
            self._pid_set_cli_srv = srv_name

        if not self._pid_set_cli.wait_for_service(timeout_sec=0.3):
            # Keep _pid_pending_fetched and timer; retry next tick — do not block spin for minutes.
            return

        # Service ready: split into small batches so foxglove_bridge can interleave list/get on ros2_receiver.
        names = list(fetched.keys())
        chunk = max(1, int(self.get_parameter("receiver_pid_push_chunk_size").value))
        self._pid_chunk_batches = [names[i : i + chunk] for i in range(0, len(names), chunk)]
        self._pid_chunk_values = fetched
        self._pid_chunk_receiver = receiver_node
        self._pid_pending_fetched = None
        self._stop_pid_apply_timer()
        self._push_next_pid_chunk()

    def _push_next_pid_chunk(self):
        """Send one chunk; chain until done (short async RPCs)."""
        if not self._pid_chunk_batches:
            recv = self._pid_chunk_receiver
            n = len(self._pid_chunk_values) if self._pid_chunk_values else 0
            self._pid_chunk_values = None
            self._pid_chunk_receiver = None
            self.logger.info(f"Pushed PID parameter(s) to '{recv}' (total {n})")
            return

        batch = self._pid_chunk_batches.pop(0)
        fv = self._pid_chunk_values
        req = SetParametersSrv.Request()
        req.parameters = [
            Parameter(name, Parameter.Type.DOUBLE, fv[name]).to_parameter_msg()
            for name in batch
        ]
        future = self._pid_set_cli.call_async(req)
        future.add_done_callback(self._on_pid_chunk_done)

    def _on_pid_chunk_done(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.logger.warning(f"PID parameter chunk failed: {e}")
            self._pid_chunk_batches = []
            self._pid_chunk_values = None
            return
        if resp is not None:
            failed = [r for r in resp.results if not r.successful]
            if failed:
                self.logger.warning(f"Some parameters in chunk failed: {failed}")
        self._push_next_pid_chunk()

    def _on_set_parameters(self, params):
        """Validate and apply dynamic parameter updates at runtime."""
        new_min = self.battery_min_voltage
        new_max = self.battery_max_voltage

        for param in params:
            if param.name == "battery_min_voltage":
                new_min = float(param.value)
            elif param.name == "battery_max_voltage":
                new_max = float(param.value)

        if new_min >= new_max:
            return SetParametersResult(
                successful=False,
                reason="battery_min_voltage must be smaller than battery_max_voltage",
            )

        self.battery_min_voltage = new_min
        self.battery_max_voltage = new_max
        self.logger.info(
            f"Updated battery thresholds: min={self.battery_min_voltage:.2f}V, "
            f"max={self.battery_max_voltage:.2f}V"
        )
        return SetParametersResult(successful=True)

    def mavlink_callback(self):
        """Timer callback - drain UDP buffer and publish to ROS (same pattern as dev branch)."""
        with self._mav_lock:
            while True:
                msg = self.port.recv_match(blocking=False)
                if msg is None:
                    break
                mtype = msg.get_type()
                if mtype == "PID_TUNING":
                    self.handle_pid_tuning(msg)
                elif mtype == "HEARTBEAT":
                    self.handle_heartbeat(msg)
                elif mtype == "ATTITUDE":
                    self.handle_attitude(msg)
                elif mtype == "MANUAL_CONTROL":
                    self.handle_manual_control(msg)
                elif mtype == "BATTERY_STATUS":
                    self.handle_battery(msg)
                elif mtype == "SCALED_PRESSURE2":
                    self.handle_scaled_pressure(msg)

    def message_counter(self, msg_type: str) -> bool:
        """Only process every Nth message per message type."""
        count = self.msg_type_counter.get(msg_type, 0) + 1
        self.msg_type_counter[msg_type] = count

        if count % self.msg_type_counter_interval == 0:
            self.msg_type_counter[msg_type] = 0
            return True
        else:
            return False

    def handle_heartbeat(self, msg):
        """Process HEARTBEAT message and publish to ROS2"""
        skip_invalid = bool(self.get_parameter("heartbeat_skip_invalid_autopilot").value)
        if skip_invalid and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            self.ros_logger.debug(
                "Skipping HEARTBEAT (autopilot=INVALID); set heartbeat_skip_invalid_autopilot=false to publish"
            )
            return

        self.last_heartbeat_time = self.get_clock().now()

        if not self.message_counter("HEARTBEAT"):
            return

        ros_msg = State()
        # MAVLink system status (uint8)
        ros_msg.system_status = msg.system_status

        # Map custom_mode integer to human-readable flight mode name
        # Using ArduSub mapping (change to mode_mapping_acm for ArduCopter, etc.)
        mode_mapping = mavutil.mode_mapping_sub
        ros_msg.mode = mode_mapping.get(
            msg.custom_mode, f"UNKNOWN({msg.custom_mode})"
        )  # gets the name equivalent of the msg.custom_mode int using mode_mapping.get(). second entry is if its unknown

        # Based on the bitmask definition in mavutil
        ros_msg.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        self.heartbeat_publisher.publish(ros_msg)
        self.logger.info(
            f"Published Heartbeat: Status={ros_msg.system_status}, Mode={ros_msg.mode}, Armed={ros_msg.armed}"
        )

        self.flight_mode = mavutil.mode_string_v10(msg)

    def heartbeat_checker_cb(self):
        """1 Hz watchdog: publishes heartbeat health even when no heartbeat arrives."""
        now = self.get_clock().now()

        if self.last_heartbeat_time is None:
            age_s = float("inf")
        else:
            age_s = (now - self.last_heartbeat_time).nanoseconds / 1e9

        status = DiagnosticStatus()
        status.name = "Pixhawk: Heartbeat"

        if age_s < 2.0:
            status.level = DiagnosticStatus.OK
            status.message = "OK"
        elif age_s < 5.0:
            status.level = DiagnosticStatus.WARN
            status.message = f"Heartbeat delayed ({age_s:.1f}s)"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = f"Heartbeat lost ({age_s:.1f}s)"

        status.values = [
            KeyValue(key="age_s", value=f"{age_s:.2f}" if age_s != float("inf") else "never"),
        ]

        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = now.to_msg()
        diag_msg.status.append(status)
        self.diagnostic_publisher.publish(diag_msg)


    def handle_attitude(self, msg):
        """Process ATTITUDE message and publish to ROS2"""
        ros_msg = Vector3()

        #Convert Euler angles (radians) to Quaternion
        ros_msg.x = msg.roll
        ros_msg.y = msg.pitch
        ros_msg.z = msg.yaw

        # # Euler to Quaternion conversion
        # cy = math.cos(yaw * 0.5)
        # sy = math.sin(yaw * 0.5)
        # cp = math.cos(pitch * 0.5)
        # sp = math.sin(pitch * 0.5)
        # cr = math.cos(roll * 0.5)
        # sr = math.sin(roll * 0.5)

        # ros_msg.orientation.w = cr * cp * cy + sr * sp * sy
        # ros_msg.orientation.x = sr * cp * cy - cr * sp * sy
        # ros_msg.orientation.y = cr * sp * cy + sr * cp * sy
        # ros_msg.orientation.z = cr * cp * sy - sr * sp * cy

        self.attitude_publisher.publish(ros_msg)
        

    # def handle_rc_channels(self, msg):
    #     """Process RC_CHANNELS message and publish to ROS2"""
    #     ros_msg = RCIn()
    #     # First 4 channels
    #     ros_msg.channels = [msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw]

    #     self.rc_channel_publisher.publish(ros_msg)
    #     self.logger.info(f"Published RC: {ros_msg.channels}")

    def handle_battery(self, msg):
        """Process BATTERY_STATUS message and publish to ROS2"""

        if not self.message_counter("BATTERY_STATUS"):
            return

        ros_msg = BatteryState()
        # current_battery is in 10*mA (centiamperes), divide by 100 to get Amperes
        ros_msg.current = float(msg.current_battery) / 100.0
        # battery_remaining is percentage (0-100), ROS2 expects 0.0-1.0
        ros_msg.percentage = float(msg.battery_remaining) / 100.0

        ros_msg.voltage = float(msg.voltages[0]) / 1000.0

        
        consumed_msg = Float32()
        consumed_msg.data = float(msg.current_consumed)  # raw mAh from Pixhawk
        self.battery_consumed_publisher.publish(consumed_msg)

        self.battery_publisher.publish(ros_msg)
        self.logger.info(
            f"Published Battery: Current={ros_msg.current:.2f}A, Voltage={ros_msg.voltage:.2f}V"
        )
    

        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()

        # Battery current 
        status_current = DiagnosticStatus()
        status_current.name = "Battery: Current"
        status_current.level = DiagnosticStatus.OK
        status_current.message = f"{ros_msg.current:.2f}A"
        status_current.values = [KeyValue(key="current_A", value=f"{ros_msg.current:.2f}")]
        diag_msg.status.append(status_current)
        

        status_voltage = DiagnosticStatus()
        status_voltage.name = "Battery: Voltage"
        status_voltage.level = DiagnosticStatus.OK
        status_voltage.message = f"{ros_msg.voltage:.2f}V"
        status_voltage.values = [KeyValue(key="voltage", value=f"{ros_msg.voltage:.2f}V")]
        
        if ros_msg.voltage < self.battery_min_voltage:
            status_voltage.level = DiagnosticStatus.ERROR
            status_voltage.message = "Voltage is critically low"
        elif ros_msg.voltage < self.battery_min_voltage + 0.5:
            status_voltage.level = DiagnosticStatus.WARN
            status_voltage.message = "Voltage is close to minimum"
        else:
            status_voltage.level = DiagnosticStatus.OK
            status_voltage.message = f"{ros_msg.voltage:.2f}V"
        
        diag_msg.status.append(status_voltage)

        self.diagnostic_publisher.publish(diag_msg)

    def handle_scaled_pressure(self, msg):
        """Process SCALED_PRESSURE2(this is the bluerobotics pressure sensor) message and publish to ROS2"""
        # if not self.message_counter("SCALED_PRESSURE2"):
        #     return

        ros_msg = FluidPressure()
        # Differential pressure: MAVLink uses hPa, ROS2 expects Pa (multiply by 100)
        ros_msg.fluid_pressure = float(msg.press_abs) * 100.0

        self.scaled_pressure_publisher.publish(ros_msg)
        #self.logger.info(f"Published Pressure: Diff={ros_msg.fluid_pressure} Pa")

    def handle_manual_control(self, msg):
        """Process MANUAL_CONTROL message and publish to ROS2"""
        # This is a placeholder for handling manual control messages if needed
        if not self.message_counter("MANUAL_CONTROL"):
            return

        self.logger.info("manual control callback triggered")
        ros_msg = Int16MultiArray()
        ros_msg.data = [msg.x, msg.y, msg.z, msg.r, msg.buttons, msg.s, msg.t]
        self.manual_control_publisher.publish(ros_msg)
        self.logger.info(
            f"Published Manual Control: x={msg.x}, y={msg.y}, z={msg.z}, r={msg.r}, s={msg.s}, t={msg.t}"
        )

    def handle_pid_tuning(self, msg):
        """Map MAVLink PID_TUNING into custom_msgs/PidTuning (6 DoF × desired/achieved)."""
        pt = self._pid_tuning

        # If the FC sends an extended PID_TUNING with all fields in one message, copy them.
        if hasattr(msg, "roll_des") and hasattr(msg, "roll_ach"):
            pt.roll_des = float(msg.roll_des)
            pt.roll_ach = float(msg.roll_ach)
            pt.pitch_des = float(getattr(msg, "pitch_des", 0.0))
            pt.pitch_ach = float(getattr(msg, "pitch_ach", 0.0))
            pt.yaw_des = float(getattr(msg, "yaw_des", 0.0))
            pt.yaw_ach = float(getattr(msg, "yaw_ach", 0.0))
            pt.depth_des = float(getattr(msg, "depth_des", 0.0))
            pt.depth_ach = float(getattr(msg, "depth_ach", 0.0))
            pt.pos_x_des = float(getattr(msg, "pos_x_des", 0.0))
            pt.pos_x_ach = float(getattr(msg, "pos_x_ach", 0.0))
            pt.pos_y_des = float(getattr(msg, "pos_y_des", 0.0))
            pt.pos_y_ach = float(getattr(msg, "pos_y_ach", 0.0))
        else:
            # Standard MAVLink: axis + desired + achieved per message — merge into state.
            axis = int(getattr(msg, "axis", 255))
            des = float(getattr(msg, "desired", 0.0))
            ach = float(getattr(msg, "achieved", 0.0))
            if 0 <= axis < len(_PID_TUNING_AXIS_TO_FIELDS):
                d_name, a_name = _PID_TUNING_AXIS_TO_FIELDS[axis]
                setattr(pt, d_name, des)
                setattr(pt, a_name, ach)

        self.pid_tuning_publisher.publish(pt)
     


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeSender()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
