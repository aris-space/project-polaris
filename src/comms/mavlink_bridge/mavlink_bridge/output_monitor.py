import rclpy
from rclpy.node import Node
from std_msgs.msg import String
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

    def __init__(self):
        super().__init__("output_monitor")

        # Initialize state variables
        self.mode = "UNKNOWN"
        self.armed = False
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.rc_channels = []
        self.battery_current = 0.0
        self.pressure_diff = 0.0

        # Create subscriptions
        self.create_subscription(State, "pixhawk/heartbeat", self.heartbeat_cb, 10)
        self.create_subscription(Imu, "pixhawk/attitude", self.attitude_cb, 10)
        self.create_subscription(RCIn, "pixhawk/rc_channels", self.rc_cb, 10)
        self.create_subscription(BatteryState, "pixhawk/battery", self.battery_cb, 10)
        self.create_subscription(
            FluidPressure, "pixhawk/scaled_pressure", self.pressure_cb, 10
        )

        self.timer = self.create_timer(0.5, self.print_dashboard)  # 2Hz refresh rate

    def heartbeat_cb(self, msg):
        self.mode = msg.custom_mode
        self.armed = msg.armed

    def attitude_cb(self, msg):
        # Taking orientation.x/y/z directly as mapped in publisher
        self.roll = msg.orientation.x
        self.pitch = msg.orientation.y
        self.yaw = msg.orientation.z

    def rc_cb(self, msg):
        self.rc_channels = msg.channels

    def battery_cb(self, msg):
        self.battery_current = msg.current

    def pressure_cb(self, msg):
        self.pressure_diff = msg.fluid_pressure

    def print_dashboard(self):
        # Clear screen code
        print("\033[H\033[J", end="")

        print("=== MAVLINK BRIDGE MONITOR ===")

        print(f"Mode: {self.mode}")
        print(f"Armed: {self.armed}")

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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
