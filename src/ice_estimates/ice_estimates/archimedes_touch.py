import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import FluidPressure, NavSatFix
from nav_msgs.msg import Odometry
import numpy as np
from tf_transformations import euler_from_quaternion
import csv
import os
from datetime import datetime


class ArchimedesTesting(Node):

    def __init__(self):
        super().__init__("ice_estimation")

        # --- Initialisierung ---
        self.omega = 0.05  # Versatz/Abstand zum Eis
        self.pressure = None
        self.roll = None
        self.pitch = None
        self.latitude = 0.0
        self.longitude = 0.0
        self.depth = 0.0
        self.is_touching = False

        # --- Subscriptions ---
        # Keller Drucksensor
        self.pressure_sub = self.create_subscription(
            FluidPressure, "/sensors/keller26x/abs_pressure", self.pressure_callback, 10
        )

        # GPS Position
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/filtered/global", self.gps_callback, 10
        )

        # Odometrie (Tiefe & Lage) - FIX: odom_callback statt gps_callback
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered/local", self.odom_callback, 10
        )

        # Touch Detection
        self.touch_sub = self.create_subscription(
            Bool, "/ice_touch_detection/touching", self.touch_callback, 10
        )

        # --- Publisher ---
        self.publisher_ = self.create_publisher(Float64MultiArray, "/ice_thickness", 10)

        self.timer = self.create_timer(0.1, self.listener_callback)
        self.get_logger().info("Ice Thickness Node gestartet. Warte auf Kontakt...")

    def odom_callback(self, msg):
        self.depth = -msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll_rad, pitch_rad, _ = euler_from_quaternion(quat)
        self.roll = np.degrees(roll_rad)
        self.pitch = np.degrees(pitch_rad)

    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def pressure_callback(self, msg):
        self.pressure = msg.fluid_pressure

    def touch_callback(self, msg):
        self.is_touching = msg.data

    def ice_thickness(self, pressure, pitch, roll):
        rho_water, rho_ice, g = 1000.0, 917.0, 9.81
        p_surface = 95903.0

        # Tiefe aus Druck
        v_druck = ((pressure - p_surface) / (rho_water * g)) - 0.05
        # Neigungskorrektur
        omega_corr = self.omega * np.cos(np.deg2rad(pitch)) * np.cos(np.deg2rad(roll))

        if abs(pitch) > 25 or abs(roll) > 10:
            return float("nan")

        T = (1.0 / rho_ice) * ((v_druck - omega_corr) * rho_water)
        return float(T)

    def listener_callback(self):
        # 1. PRÜFUNG: Berühren wir überhaupt das Eis?
        if not self.is_touching:
            # Optional: Logge alle 5 Sekunden, dass wir noch suchen
            self.get_logger().info(
                "Kein Eiskontakt... Suche läuft.", throttle_duration_sec=5.0
            )
            return

        # 2. PRÜFUNG: Sind alle Sensordaten da?
        if any(val is None for val in [self.pressure, self.roll, self.pitch]):
            self.get_logger().warn(
                "Warte auf Druck/Odom Daten...", throttle_duration_sec=2.0
            )
            return

        thickness = self.ice_thickness(self.pressure, self.pitch, self.roll)

        if np.isnan(thickness) or thickness < 0:
            return

        current_time = self.get_clock().now().nanoseconds * 1e-9

        # Nachricht publishen
        msg = Float64MultiArray()
        msg.data = [
            current_time,
            thickness,
            self.pressure,
            self.latitude,
            self.longitude,
            self.depth,
        ]
        self.publisher_.publish(msg)

        # LOG-OUTPUT (Genau wie gewünscht)
        self.get_logger().info(
            f"Zeit: {current_time:.2f}s | "
            f"Eisdicke: {thickness:.3f}m | "
            f"Druck: {self.pressure:.1f}Pa | "
            f"Tiefe: {self.depth:.2f}m | "
            f"Lat: {self.latitude:.6f} | "
            f"Lon: {self.longitude:.6f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArchimedesTesting()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


# TODO: Messungen in ein csv schreiben und im Ordner measurements abspeichern
