import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from sensor_msgs.msg import FluidPressure
import numpy as np
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class IceEstimation(Node):

    def __init__(self):
        super().__init__("ice_estimation")

        # --- Initialize stored values ---
        self.x = None
        self.y = None
        self.depth = None
        self.omega = None
        self.pressure = None
        self.timestamp = None
        self.roll = None
        self.pitch = None

        # --- Subscriptions ---
        self.pressure_sub = self.create_subscription(
            FluidPressure, "/pixhawk/scaled_pressure", self.pressure_callback, 10
        )

        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/filtered", self.gps_callback, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered/local", self.z_callback, 10
        )

        self.sonar_sub = self.create_subscription(
            Float64, "/ping_sonar/distance", self.sonar_callback, 10
        )

        # --- Publisher ---
        self.publisher_ = self.create_publisher(Float64MultiArray, "/ice_thickness", 10)

        self.timer = self.create_timer(0.1, self.listener_callback)  # 10 Hz
        self.get_logger().info("Ice Thickness Node Started")

    # --- PRESSURE ---
    def pressure_callback(self, msg):
        self.pressure = msg.fluid_pressure  # in Pascal

    # --- ODOMETRY (DEPTH) ---
    def z_callback(self, msg):
        z = msg.pose.pose.position.z
        self.depth = -z  # ENU → depth positive down
        # --- Orientation quaternion ---
        q = msg.pose.pose.orientation

        quat = [q.x, q.y, q.z, q.w]

        # Convert to roll, pitch, yaw
        roll, pitch, yaw = euler_from_quaternion(quat)

        # Store in degrees (your formula expects degrees!)
        self.roll = np.degrees(roll)
        self.pitch = np.degrees(pitch)

    # --- GPS ---
    def gps_callback(self, msg):
        self.x = msg.longitude
        self.y = msg.latitude
        self.timestamp = msg.header.stamp

    # --- SONAR ---
    def sonar_callback(self, msg):
        self.omega = msg.data  # meters

    def ice_thickness(self, omega, pitch, roll, pressure):

        rho_s = 300.0
        rho_water = 1000.0
        rho_ice = 917.0
        g = 9.81
        h_s = 0.2
        P_ext = 0.0

        # Convert pressure from bar to Pa
        P = pressure

        # Water column height
        v = P / (rho_water * g)

        sensor_offset = 0.05
        v_druck = v - sensor_offset

        # Correct omega for pitch & roll
        omega_corr = omega * np.cos(np.deg2rad(pitch)) * np.cos(np.deg2rad(roll))

        # Validity mask
        if abs(pitch) > 25 or abs(roll) > 10:
            return float("nan")

        # Thickness formula
        T = (1.0 / rho_ice) * (
            (v_druck - omega_corr) * rho_water - h_s * rho_s - P_ext / g
        )

        return float(T)

    def listener_callback(self):

        pressure = self.pressure
        omega = self.omega
        depth = self.depth
        pitch = self.pitch
        roll = self.roll

        data_points = [
            self.x,
            self.y,
            self.pressure,
            self.omega,
            self.depth,
            self.roll,
            self.pitch,
        ]
        if any(val is None for val in data_points):
            self.get_logger().warn(
                "Warte auf alle Sensordaten (GPS, Druck, Sonar, Odom)...",
                throttle_duration_sec=2.0,
            )
            return

        thickness = self.ice_thickness(omega, depth, pitch, roll, pressure)

        # --- Skip invalid ---
        if np.isnan(thickness):
            self.get_logger().warn("Invalid thickness (pitch/roll too large)")
            return

        # --- Publish ---
        msg = Float64MultiArray()
        msg.data = [
            float(self.get_clock().now().nanoseconds * 1e-9),  # timestamp
            float(self.x),
            float(self.y),
            float(thickness),
        ]

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"[t, x, y, T] = [{msg.data[0]:.2f}, {msg.data[1]:.6f}, {msg.data[2]:.6f}, {msg.data[3]:.3f}]"
        )


def main(args=None):
    rclpy.init(args=args)
    node = IceEstimation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
