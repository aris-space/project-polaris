import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import String
from mavros_msgs.msg import (
    State,  # HEARTBEAT
    RCIn,  # RC_CHANNELS
    VFR_HUD,  # VFR_HUD
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
        """Timer callback - checks for MAVLink messages and routes them"""
        msg = self.port.recv_match(blocking=False)

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
        # Base mode bitmap
        ros_msg.base_mode = msg.base_mode
        # Custom mode (flight mode)
        ros_msg.custom_mode = str(msg.custom_mode)

        self.heartbeat_publisher.publish(ros_msg)
        self.get_logger().info(
            f"Published Heartbeat: Status={ros_msg.system_status}, Base={ros_msg.base_mode}, Custom={ros_msg.custom_mode}"
        )

    def handle_attitude(self, msg):
        """Process ATTITUDE message and publish to ROS2"""
        ros_msg = Imu()
        # ROS2 uses quaternions, but we can put Euler angles in for debugging if needed
        # Or just mapping directly to show the data transfer
        ros_msg.orientation.x = float(msg.roll)
        ros_msg.orientation.y = float(msg.pitch)
        ros_msg.orientation.z = float(msg.yaw)
        # Note: Proper implementation would convert Euler to Quaternion

        self.attitude_publisher.publish(ros_msg)
        self.get_logger().info(
            f"Published Attitude: Roll={msg.roll:.2f}, Pitch={msg.pitch:.2f}, Yaw={msg.yaw:.2f}"
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
        # Current battery (cA -> A conversion typically needed, but mapping raw for now)
        ros_msg.current = float(msg.current_battery)

        self.battery_publisher.publish(ros_msg)
        self.get_logger().info(f"Published Battery: Current={ros_msg.current}")

    def handle_scaled_pressure(self, msg):
        """Process SCALED_PRESSURE message and publish to ROS2"""
        ros_msg = FluidPressure()
        # Differential pressure
        ros_msg.fluid_pressure = float(msg.press_diff)

        self.scaled_pressure_publisher.publish(ros_msg)
        self.get_logger().info(f"Published Pressure: Diff={ros_msg.fluid_pressure}")


def main(args=None):
    rclpy.init(args=args)
    node = MavlinkBridgeSender()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
