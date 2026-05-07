import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool, Float64
from sensor_msgs.msg import FluidPressure, NavSatFix
from nav_msgs.msg import Odometry
import numpy as np
from tf_transformations import euler_from_quaternion
import csv
import os
from datetime import datetime


_REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))

_CSV_FIELDS = [
    'timestamp_s', 'latitude', 'longitude',
    'pressure_pa', 'depth_m', 'roll_deg', 'pitch_deg', 'ice_thickness_m',
]


class ArchimedesTesting(Node):

    def __init__(self):
        super().__init__("ice_estimation")

        # --- Initialisierung ---
        self.omega = 0.210  # BlueRobotics sensor to ice contact point (m)
        self.pressure = None
        self.surface_pressure = None
        self.roll = None
        self.pitch = None
        self.latitude = 0.0
        self.longitude = 0.0
        self.depth = 0.0
        self.is_touching = False
        self._was_touching = False
        self._touch_buffer = []

        # --- CSV Setup ---
        start_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        folder = os.path.join(_REPO_ROOT, 'measurements', start_str)
        os.makedirs(folder, exist_ok=True)

        self._raw_file = open(os.path.join(folder, f'measurements_raw_{start_str}.csv'), 'w', newline='')
        self._av_file  = open(os.path.join(folder, f'measurements_av_{start_str}.csv'),  'w', newline='')
        self._raw_writer = csv.DictWriter(self._raw_file, fieldnames=_CSV_FIELDS)
        self._av_writer  = csv.DictWriter(self._av_file,  fieldnames=_CSV_FIELDS)
        self._raw_writer.writeheader()
        self._av_writer.writeheader()

        self.get_logger().info(f"Messdaten werden gespeichert in: {folder}")

        # --- Subscriptions ---
        self.pressure_sub = self.create_subscription(
            FluidPressure, "/pixhawk/scaled_pressure", self.pressure_callback, 10
        )
        self.surface_pressure_sub = self.create_subscription(
            Float64, "/sensors/pressure/p_surface_pa", self.surface_pressure_callback, 10
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/filtered/global", self.gps_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered/local", self.odom_callback, 10
        )
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

    def surface_pressure_callback(self, msg):
        self.surface_pressure = msg.data

    def touch_callback(self, msg):
        self.is_touching = msg.data

    def ice_thickness(self, pressure, pitch, roll):
        rho_water, rho_ice, g = 1000.0, 917.0, 9.81

        gauge_pressure = pressure - self.surface_pressure
        v_druck = gauge_pressure / (rho_water * g)

        omega_corr = self.omega

        if abs(pitch) > 25 or abs(roll) > 10:
            return float("nan")

        T = (1.0 / rho_ice) * ((v_druck - omega_corr) * rho_water)
        return float(T)

    def _flush_average(self):
        if not self._touch_buffer:
            return
        avg_row = {k: float(np.mean([r[k] for r in self._touch_buffer])) for k in _CSV_FIELDS}
        self._av_writer.writerow(avg_row)
        self._av_file.flush()
        self._touch_buffer.clear()

    def listener_callback(self):
        if not self.is_touching:
            # Transition: touching ended → write averaged row
            if self._was_touching:
                self._flush_average()
            self._was_touching = False
            return

        if any(val is None for val in [self.pressure, self.surface_pressure, self.roll, self.pitch]):
            return

        thickness = self.ice_thickness(self.pressure, self.pitch, self.roll)

        if np.isnan(thickness) or thickness < 0:
            self._was_touching = True
            return

        current_time = self.get_clock().now().nanoseconds * 1e-9

        row = {
            'timestamp_s':    current_time,
            'latitude':        self.latitude,
            'longitude':       self.longitude,
            'pressure_pa':     self.pressure,
            'depth_m':         self.depth,
            'roll_deg':        self.roll,
            'pitch_deg':       self.pitch,
            'ice_thickness_m': thickness,
        }

        # Write raw measurement immediately
        self._raw_writer.writerow(row)
        self._raw_file.flush()

        # Accumulate for session average
        self._touch_buffer.append(row)
        self._was_touching = True

        # Publish
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

    def destroy_node(self):
        # Flush any remaining touch session on shutdown
        self._flush_average()
        self._raw_file.close()
        self._av_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArchimedesTesting()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
