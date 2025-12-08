import math
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from mavros_msgs.msg import (
    State,  # HEARTBEAT
    RCIn,  # RC_CHANNELS
)
from sensor_msgs.msg import (
    Imu,  # ATTITUDE
    BatteryState,  # BATTERY_STATUS
    FluidPressure,  # SCALED_PRESSURE (depth)
)


class MavlinkBridgeSender(Node):
    """
    This node is supposed to publish mavlink data directly from the pixhawk to ROS2 topics
    """

    def __init__(self):
        super().__init__("mavlink_bridge_publisher")

        self.port = mavutil.mavlink_connection("udp:10.5.11.50:14600")

        self.port.wait_heartbeat()
        self.get_logger().info(
            f"Heartbeat received from system {self.port.target_system}"
        )

        self.heartbeat_publisher = self.create_publisher(State, "pixhawk/heartbeat", 10)

        self.attitude_publisher = self.create_publisher(Imu, "pixhawk/attitude", 10)

        self.rc_channel_publisher = self.create_publisher(
            RCIn, "pixhawk/rc_channels", 10
        )

        self.battery_publisher = self.create_publisher(
            BatteryState, "pixhawk/battery", 10
        )

        self.scaled_pressure_publisher = self.create_publisher(
            FluidPressure, "pixhawk/scaled_pressure", 10
        )

        self.timer = self.create_timer(0.5, self.mavlink_callback)

    def mavlink_callback(self):
        """Timer callback - drains all buffered MAVLink messages and routes them"""
        # Process ALL available messages in the buffer (not just one)
        while True:
            msg = self.port.recv_match(blocking=False)
            if msg is None:
                break  # No more messages in buffer

        if msg is not None:
            self.get_logger().info(f"Received: {msg.get_type()}")

            if msg.get_type() == "HEARTBEAT":
                self.handle_heartbeat(msg)
            elif msg.get_type() == "ATTITUDE":
                self.handle_attitude(msg)
            elif msg.get_type() == "RC_CHANNELS":
                self.handle_rc_channels(msg)
            elif msg.get_type() == "BATTERY_STATUS":
                self.handle_battery(msg)
            elif msg.get_type() == "SCALED_PRESSURE":
                self.handle_scaled_pressure(msg)

    def handle_heartbeat(self, msg):
        """Process HEARTBEAT message and publish to ROS2"""
        ros_msg = State()
        # MAVLink system status (uint8)
        ros_msg.system_status = msg.system_status

        # Map custom_mode integer to human-readable flight mode name
        # Using ArduSub mapping (change to mode_mapping_acm for ArduCopter, etc.)
        mode_mapping = mavutil.mode_mapping_sub
        ros_msg.mode = mode_mapping.get(msg.custom_mode, f"UNKNOWN({msg.custom_mode})")

        # Based on the bitmask definition in mavutil
        ros_msg.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        self.heartbeat_publisher.publish(ros_msg)
        self.get_logger().info(
            f"Published Heartbeat: Status={ros_msg.system_status}, Mode={ros_msg.mode}, Armed={ros_msg.armed}"
        )

    def handle_attitude(self, msg):
        """Process ATTITUDE message and publish to ROS2"""
        ros_msg = Imu()

        # Convert Euler angles (radians) to Quaternion
        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw

        # Euler to Quaternion conversion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        ros_msg.orientation.w = cr * cp * cy + sr * sp * sy
        ros_msg.orientation.x = sr * cp * cy - cr * sp * sy
        ros_msg.orientation.y = cr * sp * cy + sr * cp * sy
        ros_msg.orientation.z = cr * cp * sy - sr * sp * cy

        self.attitude_publisher.publish(ros_msg)
        self.get_logger().info(
            f"Published Attitude: Roll={roll:.2f}, Pitch={pitch:.2f}, Yaw={yaw:.2f}"
        )

    def handle_rc_channels(self, msg):
        """Process RC_CHANNELS message and publish to ROS2"""
        ros_msg = RCIn()
        # First 4 channels
        ros_msg.channels = [msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw]

        self.rc_channel_publisher.publish(ros_msg)
        self.get_logger().info(f"Published RC: {ros_msg.channels}")

    def handle_battery(self, msg):
        """Process BATTERY_STATUS message and publish to ROS2"""
        ros_msg = BatteryState()
        # current_battery is in 10*mA (centiamperes), divide by 100 to get Amperes
        ros_msg.current = float(msg.current_battery) / 100.0
        # battery_remaining is percentage (0-100), ROS2 expects 0.0-1.0
        ros_msg.percentage = float(msg.battery_remaining) / 100.0

        self.battery_publisher.publish(ros_msg)
        self.get_logger().info(
            f"Published Battery: Current={ros_msg.current:.2f}A, Remaining={ros_msg.percentage:.0%}"
        )

    def handle_scaled_pressure(self, msg):
        """Process SCALED_PRESSURE message and publish to ROS2"""
        ros_msg = FluidPressure()
        # Differential pressure: MAVLink uses hPa, ROS2 expects Pa (multiply by 100)
        ros_msg.fluid_pressure = float(msg.press_diff) * 100.0

        self.scaled_pressure_publisher.publish(ros_msg)
        self.get_logger().info(f"Published Pressure: Diff={ros_msg.fluid_pressure} Pa")


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeSender()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
