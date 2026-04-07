import math
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import (
    State,  # HEARTBEAT
    RCIn,  # RC_CHANNELS
)
from sensor_msgs.msg import (
    Imu,  # ATTITUDE
    BatteryState,  # BATTERY_STATUS
    FluidPressure,  # SCALED_PRESSURE (depth)
)


class OutputMonitor(Node):
    # MAV_STATE mapping (system_status values)
    SYSTEM_STATUS_MAP = {
        0: "UNINIT",
        1: "BOOT",
        2: "CALIBRATING",
        3: "STANDBY",
        4: "ACTIVE",
        5: "CRITICAL",
        6: "EMERGENCY",
        7: "POWEROFF",
        8: "FLIGHT_TERMINATION",
    }

    def __init__(self):
        super().__init__("output_monitor")

        # Initialize state variables
        self.mode = "UNKNOWN"
        self.armed = False
        self.system_status = "UNKNOWN"
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.rc_channels = []
        self.battery_current = 0.0
        self.pressure_diff = 0.0

        # Create subscriptions
        self.create_subscription(State, "/pixhawk/heartbeat", self.heartbeat_cb, 10)
        self.create_subscription(Imu, "/pixhawk/attitude", self.attitude_cb, 10)
        self.create_subscription(RCIn, "/pixhawk/rc_channels", self.rc_cb, 10)
        self.create_subscription(BatteryState, "/pixhawk/battery", self.battery_cb, 10)
        self.create_subscription(
            FluidPressure, "/pixhawk/scaled_pressure", self.pressure_cb, 10
        )

        self.timer = self.create_timer(0.5, self.print_dashboard)  # 2Hz refresh rate

    def heartbeat_cb(self, msg):
        # msg.mode is already a human-readable string (e.g., "STABILIZE", "MANUAL")
        self.mode = msg.mode
        self.armed = msg.armed
        # Map system_status integer to human-readable string
        self.system_status = self.SYSTEM_STATUS_MAP.get(
            msg.system_status, f"UNKNOWN({msg.system_status})"
        )

    def attitude_cb(self, msg):
        # Convert quaternion back to Euler angles for display
        w = msg.orientation.w
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z

        # Quaternion to Euler angles conversion
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        self.roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            self.pitch = math.copysign(math.pi / 2, sinp)
        else:
            self.pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def rc_cb(self, msg):
        self.rc_channels = msg.channels

    def battery_cb(self, msg):
        self.battery_current = msg.current

    def pressure_cb(self, msg):
        self.pressure_diff = msg.fluid_pressure

    # this function actually print the dashboard
    def print_dashboard(self):
        # Clear screen code
        print("\033[H\033[J", end="")

        print("=== MAVLINK BRIDGE MONITOR ===")

        print(f"Mode: {self.mode}")
        print(f"Armed: {self.armed}")
        print(f"System Status: {self.system_status}")

        print(f"Roll: {self.roll}")
        print(f"Pitch: {self.pitch}")
        print(f"Yaw: {self.yaw}")

        print(f"Battery Current: {self.battery_current}")
        print(f"Pressure Diff: {self.pressure_diff}")

        print(f"RC Channels: {list(self.rc_channels)}")
        print("============================")


def main(args=None):
    rclpy.init(args=args)
    node = OutputMonitor()
    rclpy.spin(node)  # keeps the node running
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
