import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, String  # String hinzufügen
from sensor_msgs.msg import FluidPressure, Imu
import numpy as np
from tf_transformations import euler_from_quaternion
import json


class PoolTesting(Node):

    def __init__(self):
        super().__init__("ice_estimation")

        # --- Initialisierung ---
        self.omega = None
        self.pressure = None
        self.roll = None
        self.pitch = None

        # --- Subscriptions ---
        self.pressure_sub = self.create_subscription(
            FluidPressure, "/pixhawk/scaled_pressure", self.pressure_callback, 10
        )

        self.sonar_sub = self.create_subscription(
            String, "/ping_sonar/distance", self.sonar_callback, 10
        )

        self.imu_sub = self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)

        # --- Publisher ---
        self.publisher_ = self.create_publisher(Float64MultiArray, "/ice_thickness", 10)

        # Timer für die Berechnung (10 Hz)
        self.timer = self.create_timer(0.1, self.listener_callback)
        self.get_logger().info("Ice Thickness Node gestartet (Ohne GPS)")

    def imu_callback(self, msg):
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll_rad, pitch_rad, _ = euler_from_quaternion(quat)
        self.roll = np.degrees(roll_rad)
        self.pitch = np.degrees(pitch_rad)

    def pressure_callback(self, msg):
        self.pressure = msg.fluid_pressure  # Pascal

    def sonar_callback(self, msg):
        try:
            # Wir laden den String als JSON-Objekt
            data = json.loads(msg.data)

            # Wir holen uns den Wert 'distance' (der in mm ist)
            # und wandeln ihn in Meter um
            self.omega = float(data["distance"]) / 1000.0

            # Optional: Logge den Wert einmal, um sicher zu sein
            # self.get_logger().info(f"Sonar Distanz: {self.omega} m")

        except Exception as e:
            self.get_logger().error(f"Fehler beim Parsen der Sonar-JSON: {e}")

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
        self.publisher_.publish(msg)

        # --- DEIN LOG-OUTPUT ---
        self.get_logger().info(
            f"Zeit: {current_time:.2f}s | "
            f"Eisdicke: {thickness:.3f}m | "
            f"Druck: {self.pressure:.1f}Pa | "
            f"Sonar: {self.omega:.3f}m"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoolTesting()  # Hier die richtige Klasse aufrufen
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
