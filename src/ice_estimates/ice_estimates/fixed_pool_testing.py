import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import FluidPressure, Imu
import numpy as np
from tf_transformations import euler_from_quaternion


class FixedPoolTesting(Node):
    def __init__(self):
        super().__init__("ice_estimation")

        # QoS Profil: SensorData ist oft kompatibler für Bags
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.omega = None
        self.pressure = None
        self.roll = None
        self.pitch = None

        # --- Subscriptions mit explizitem QoS ---
        self.create_subscription(
            FluidPressure,
            "/pixhawk/scaled_pressure",
            self.pressure_callback,
            qos_sensor,
        )
        self.create_subscription(
            String, "/ping_sonar/distance", self.sonar_callback, qos_sensor
        )
        self.create_subscription(Imu, "/imu/data", self.imu_callback, qos_sensor)

        self.publisher_ = self.create_publisher(Float64MultiArray, "/ice_thickness", 10)
        self.timer = self.create_timer(0.1, self.listener_callback)
        self.get_logger().info("Ice Thickness Node gestartet - Diagnose aktiv")

    def imu_callback(self, msg):
        q = msg.orientation
        # Check ob Orientierung überhaupt vorhanden ist
        if q.w == 0.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0:
            return
        roll_rad, pitch_rad, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.roll, self.pitch = np.degrees(roll_rad), np.degrees(pitch_rad)

    def pressure_callback(self, msg):
        self.pressure = msg.fluid_pressure

    def sonar_callback(self, msg):
        try:
            # strip() entfernt versteckte Zeilenumbrüche
            self.omega = float(msg.data.strip())
        except:
            pass

    def listener_callback(self):
        # --- DIAGNOSE-LOG ---
        missing = []
        if self.pressure is None:
            missing.append("Druck")
        if self.omega is None:
            missing.append("Sonar")
        if self.roll is None:
            missing.append("IMU/Roll")

        if missing:
            self.get_logger().warn(
                f"Warte auf: {', '.join(missing)}", throttle_duration_sec=2.0
            )
            return

        # Berechnung (deine Logik)
        v_druck = (self.pressure / (1000.0 * 9.81)) - 0.05
        omega_corr = (
            self.omega * np.cos(np.deg2rad(self.pitch)) * np.cos(np.deg2rad(self.roll))
        )

        if abs(self.pitch) > 25 or abs(self.roll) > 10:
            return

        thickness = (1.0 / 917.0) * ((v_druck - omega_corr) * 1000.0 - (0.2 * 300.0))

        t = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f"Dicke: {thickness:.3f}m | Druck: {self.pressure:.1f} | Sonar: {self.omega:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FixedPoolTesting())
    rclpy.shutdown()
