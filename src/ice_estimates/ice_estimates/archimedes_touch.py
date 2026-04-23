import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, String, Bool  # String hinzufügen
from sensor_msgs.msg import FluidPressure, Imu, NavSatFix
from nav_msgs.msg import Odometry
import numpy as np
from tf_transformations import euler_from_quaternion
import json


class ArchimedesTesting(Node):

    def __init__(self):
        super().__init__("ice_estimation")

        # --- Initialisierung ---
        self.omega = 0.05  # Default offset for "touching ice"
        self.pressure = None
        self.roll = None
        self.pitch = None
        self.latitude = None
        self.longitude = None
        self.depth = None
        self.is_touching = False

        # --- Subscriptions ---
        # TODO: change to keller pressure sensor (see topic list excel sheet, take absolute pressure for same procedure as pixhawk scaled pressure)
        # /sensors/keller26x/abs_pressure	sensor_msgs/FluidPressure
        self.pressure_sub = self.create_subscription(
            FluidPressure, "/pixhawk/scaled_pressure", self.pressure_callback, 10
        )

        # TODO: remove this for sub touching ice take in the correct offset
        # Takes imu, keller pressure and ultrasonic sensor data and detects if sub is touching ice	/ice_touch_detection/touching	std_msgs/Bool
        """self.sonar_sub = self.create_subscription(
            String, "/ping_sonar/distance", self.sonar_callback, 10
        )"""

        self.pressure_sub = self.create_subscription(
            FluidPressure, "/sensors/keller26x/abs_pressure", self.pressure_callback, 10
        )

        # TODO: Add position in GPS coordinates. Take them from ... Topic ask Noel or Gleb for Coordinate Topic
        # TODO: (Optional) change source for orientation to filtered Odometry topic. Use it to detect
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/filtered", self.gps_callback, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered/local", self.gps_callback, 10
        )

        self.touch_sub = self.create_subscription(
            Bool, "/ice_touch_detection/touching", self.touch_callback, 10
        )

        # --- Publisher ---
        self.publisher_ = self.create_publisher(Float64MultiArray, "/ice_thickness", 10)

        # Timer für die Berechnung (10 Hz)
        self.timer = self.create_timer(0.1, self.listener_callback)
        self.get_logger().info("Ice Thickness Node gestartet (Ohne GPS)")

        # --- ODOMETRY (DEPTH) ---

    def odom_callback(self, msg):
        # Depth: In ENU, Z is up. If sub is underwater, Z is negative.
        self.depth = -msg.pose.pose.position.z

        # Orientation
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll_rad, pitch_rad, _ = euler_from_quaternion(quat)
        self.roll = np.degrees(roll_rad)
        self.pitch = np.degrees(pitch_rad)

    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def pressure_callback(self, msg):
        # Keller 26X provides Absolute Pressure in Pascal
        self.pressure = msg.fluid_pressure

    def touch_callback(self, msg):
        self.is_touching = msg.data

    def ice_thickness(self, omega, pressure, pitch, roll):
        rho_s = 0.0
        rho_water = 1000.0
        rho_ice = 917.0
        g = 9.81
        h_s = 0.0

        p_surface = 95903.0

        # Tiefe aus Druck berechnen
        v_druck = ((pressure - p_surface) / (rho_water * g)) - 0.05

        # Sonar-Korrektur durch Neigung
        omega_corr = omega * np.cos(np.deg2rad(pitch)) * np.cos(np.deg2rad(roll))

        # Plausibilitäts-Check
        if abs(pitch) > 25 or abs(roll) > 10:
            return float("nan")

        # Formel für Eisdicke T
        T = (1.0 / rho_ice) * ((v_druck - omega_corr) * rho_water - h_s * rho_s)
        return float(T)

    def listener_callback(self):
        # Prüfen ob alle benötigten Daten da sind (ohne GPS x,y)
        if any(
            val is None for val in [self.pressure, self.omega, self.roll, self.pitch]
        ):
            self.get_logger().warn(
                "Warte auf Sensordaten (Druck, Sonar, IMU)...",
                throttle_duration_sec=2.0,
            )
            return

        thickness = self.ice_thickness(self.omega, self.pressure, self.pitch, self.roll)

        if np.isnan(thickness):
            return

        # Zeitstempel generieren
        current_time = self.get_clock().now().nanoseconds * 1e-9

        # Nachricht publishen
        msg = Float64MultiArray()
        msg.data = [current_time, thickness, self.pressure, self.omega]

        lat = self.latitude if self.latitude is not None else 0.0
        lon = self.longitude if self.longitude is not None else 0.0
        self.publisher_.publish(msg)

        # --- DEIN LOG-OUTPUT ---
        self.get_logger().info(
            f"Zeit: {current_time:.2f}s | "
            f"Eisdicke: {thickness:.3f}m | "
            f"Druck: {self.pressure:.1f}Pa | "
            f"Tiefe: {self.depth:.2f}m | "
            f"Lat: {self.lat:.6f} | "
            f"Lon: {self.lon:.6f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArchimedesTesting()  # Hier die richtige Klasse aufrufen
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


# TODO: Messungen in ein csv schreiben und im Ordner measurements abspeichern
