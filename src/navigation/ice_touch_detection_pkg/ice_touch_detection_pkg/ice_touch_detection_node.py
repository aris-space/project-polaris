"""ROS2 node for detecting contact between AUV tower and the ice ceiling.

Geometry (all distances in metres, body frame: X forward, Y left, Z up):
  - Tower top contact point: 0.148 m above, 0.650 m behind the ultrasonic sensor
  - Pressure sensor:         0.136 m below the tower top (directly beneath it)

Detection strategy:
  1. Primary  – geometric: project tower height into world-Z using IMU quaternion,
                compare against ultrasonic measurement projected the same way.
  2. Fallback – when ultrasonic reports constant 0 s (sensor fault), switch to
                pressure-only near-surface threshold.
  3. Enhancement – sudden acceleration spike on the IMU while near the surface
                   also triggers a touch, independent of the primary signal.
"""

from __future__ import annotations

import math
from collections import deque

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Imu
from std_msgs.msg import Bool, Float32

_GRAVITY_MS2 = 9.81


class IceTouchDetectionNode(Node):
    """Fuse ultrasonic, IMU and pressure data to detect ice contact."""

    def __init__(self) -> None:
        super().__init__("ice_touch_detection_node")

        # ── geometry ──────────────────────────────────────────────────────────
        self.declare_parameter("tower_height_m", 0.148)
        self.declare_parameter("tower_horizontal_offset_m", 0.650)
        self.declare_parameter("pressure_sensor_offset_m", 0.136)

        # ── geometric detection ───────────────────────────────────────────────
        self.declare_parameter("touch_tolerance_m", 0.02)
        self.declare_parameter("max_valid_angle_deg", 45.0)

        # ── ultrasonic validity ───────────────────────────────────────────────
        # If ≥ zero_ratio_threshold of the last zero_window readings are zero,
        # the sensor is considered faulty and will be ignored.
        self.declare_parameter("ultrasonic_zero_window", 15)
        self.declare_parameter("ultrasonic_zero_ratio_threshold", 0.8)

        # ── pressure ──────────────────────────────────────────────────────────
        # water_density_kgm3: use 1000 for freshwater, 1025 for seawater.
        # pressure_near_surface_pa: used as cross-check when ultrasonic is valid.
        # pressure_fallback_pa: primary threshold when ultrasonic is invalid.
        self.declare_parameter("water_density_kgm3", 1000.0)
        self.declare_parameter("pressure_near_surface_pa", 2000.0)
        self.declare_parameter("pressure_fallback_pa", 1800.0)

        # ── IMU collision detection ───────────────────────────────────────────
        # Maintain a rolling window of acceleration magnitudes; if the latest
        # value exceeds the window mean by > threshold, an impact is inferred.
        self.declare_parameter("use_imu_collision", True)
        self.declare_parameter("imu_collision_window", 30)
        self.declare_parameter("imu_collision_min_samples", 10)
        self.declare_parameter("imu_collision_threshold_ms2", 3.5)

        # ── debounce ──────────────────────────────────────────────────────────
        self.declare_parameter("confirm_count", 3)
        self.declare_parameter("clear_count", 5)
        self.declare_parameter("publish_rate_hz", 10.0)

        # ── topics ────────────────────────────────────────────────────────────
        self.declare_parameter("ultrasonic_topic", "/top/ultrasonic/distance")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("acceleration_topic", "/imu/acceleration")
        self.declare_parameter("pressure_topic", "/sensors/keller26x/gauge_pressure")
        self.declare_parameter("output_topic", "/ice_touch_detection/touching")

        # ── runtime state ─────────────────────────────────────────────────────
        self._latest_distance: float | None = None
        self._latest_imu: Imu | None = None
        self._latest_pressure: float | None = None

        self._ultrasonic_window: deque[float] = deque(
            maxlen=int(self.get_parameter("ultrasonic_zero_window").value)
        )
        self._accel_window: deque[float] = deque(
            maxlen=int(self.get_parameter("imu_collision_window").value)
        )

        self._is_touching = False
        self._confirm_count = 0
        self._clear_count = 0

        # ── subscriptions ─────────────────────────────────────────────────────
        ultrasonic_topic = str(self.get_parameter("ultrasonic_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)
        accel_topic = str(self.get_parameter("acceleration_topic").value)
        pressure_topic = str(self.get_parameter("pressure_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.create_subscription(Float32, ultrasonic_topic, self._ultrasonic_cb, 10)
        self.create_subscription(Imu, imu_topic, self._imu_cb, 10)
        self.create_subscription(Vector3Stamped, accel_topic, self._accel_cb, 10)
        self.create_subscription(FluidPressure, pressure_topic, self._pressure_cb, 10)

        self._pub = self.create_publisher(Bool, output_topic, 10)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / rate, self._detection_tick)

        self.get_logger().info(
            f"IceTouchDetectionNode started"
            f" | ultrasonic: {ultrasonic_topic}"
            f" | imu: {imu_topic}"
            f" | pressure: {pressure_topic}"
            f" | output: {output_topic}"
        )

    # ── subscriber callbacks ───────────────────────────────────────────────────

    def _ultrasonic_cb(self, msg: Float32) -> None:
        self._latest_distance = float(msg.data)
        self._ultrasonic_window.append(self._latest_distance)

    def _imu_cb(self, msg: Imu) -> None:
        self._latest_imu = msg

    def _accel_cb(self, msg: Vector3Stamped) -> None:
        magnitude = math.sqrt(
            msg.vector.x ** 2 + msg.vector.y ** 2 + msg.vector.z ** 2
        )
        self._accel_window.append(magnitude)

    def _pressure_cb(self, msg: FluidPressure) -> None:
        self._latest_pressure = float(msg.fluid_pressure)

    # ── sensor validity ────────────────────────────────────────────────────────

    def _is_ultrasonic_valid(self) -> bool:
        """Return False when the sensor is stuck at zero (fault condition)."""
        window_size = int(self.get_parameter("ultrasonic_zero_window").value)
        ratio_threshold = float(
            self.get_parameter("ultrasonic_zero_ratio_threshold").value
        )
        if len(self._ultrasonic_window) < window_size:
            return True  # not enough history yet; assume valid
        zero_count = sum(1 for v in self._ultrasonic_window if v == 0.0)
        return (zero_count / len(self._ultrasonic_window)) < ratio_threshold

    # ── geometric touch check ──────────────────────────────────────────────────

    def _geometric_touching(
        self, distance: float, qx: float, qy: float, qz: float, qw: float
    ) -> bool:
        """Project tower height and ultrasonic range onto world-Z; compare.

        The ultrasonic sensor fires along body-Z (upward). The tower contact
        point sits at body position (-offset, 0, height) relative to the sensor.
        Both are projected to world-Z via the IMU rotation matrix row [2,:].

          R[2,0] = 2*(qx*qz - qy*qw)
          R[2,1] = 2*(qy*qz + qx*qw)
          R[2,2] = 1 - 2*(qx² + qy²)

        Touching when:
          distance * R[2,2]  <=  tower_world_z  +  tolerance
        """
        tower_h = float(self.get_parameter("tower_height_m").value)
        tower_offset = float(self.get_parameter("tower_horizontal_offset_m").value)
        tolerance = float(self.get_parameter("touch_tolerance_m").value)
        max_angle_rad = math.radians(
            float(self.get_parameter("max_valid_angle_deg").value)
        )

        roll, pitch = _roll_pitch_from_quat(qx, qy, qz, qw)
        if abs(roll) > max_angle_rad or abs(pitch) > max_angle_rad:
            self.get_logger().warn(
                f"Orientation out of geometric validity range: "
                f"roll={math.degrees(roll):.1f}° pitch={math.degrees(pitch):.1f}°"
            )
            return False

        # Third row of rotation matrix (body → world), world-Z components
        r20 = 2.0 * (qx * qz - qy * qw)
        r21 = 2.0 * (qy * qz + qx * qw)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

        # Tower body position relative to ultrasonic sensor: (-offset, 0, height)
        tower_world_z = r20 * (-tower_offset) + r21 * 0.0 + r22 * tower_h

        # World-Z component of the ultrasonic measurement (beam along body-Z)
        ultrasonic_world_z = r22 * distance

        return ultrasonic_world_z <= tower_world_z + tolerance

    # ── pressure helpers ───────────────────────────────────────────────────────

    def _pressure_depth_m(self, gauge_pa: float) -> float:
        density = float(self.get_parameter("water_density_kgm3").value)
        return gauge_pa / (density * _GRAVITY_MS2)

    def _pressure_near_surface(self, gauge_pa: float) -> bool:
        threshold = float(self.get_parameter("pressure_near_surface_pa").value)
        return gauge_pa < threshold

    def _pressure_fallback_touching(self, gauge_pa: float) -> bool:
        threshold = float(self.get_parameter("pressure_fallback_pa").value)
        return gauge_pa < threshold

    # ── IMU collision detection ────────────────────────────────────────────────

    def _imu_collision_detected(self) -> bool:
        """Detect sudden spike in acceleration magnitude indicating an impact."""
        if not bool(self.get_parameter("use_imu_collision").value):
            return False
        min_samples = int(self.get_parameter("imu_collision_min_samples").value)
        threshold = float(self.get_parameter("imu_collision_threshold_ms2").value)
        if len(self._accel_window) < min_samples:
            return False
        history = list(self._accel_window)
        # Compare latest sample against the rolling mean of all preceding samples
        baseline = sum(history[:-1]) / len(history[:-1])
        return (history[-1] - baseline) > threshold

    # ── main detection loop ────────────────────────────────────────────────────

    def _compute_raw_touch(self) -> bool:
        """Return instantaneous (non-debounced) touching state."""
        distance = self._latest_distance
        imu = self._latest_imu
        pressure = self._latest_pressure

        if pressure is None and (distance is None or imu is None):
            return False

        ultrasonic_valid = (distance is not None) and self._is_ultrasonic_valid()
        collision = self._imu_collision_detected()

        # ── path 1: ultrasonic + IMU available ────────────────────────────────
        if ultrasonic_valid and imu is not None and distance is not None:
            q = imu.orientation
            norm_sq = q.x**2 + q.y**2 + q.z**2 + q.w**2
            if norm_sq < 0.5:
                # Quaternion not initialised; treat as invalid orientation
                ultrasonic_valid = False
            else:
                touching = self._geometric_touching(distance, q.x, q.y, q.z, q.w)
                if touching:
                    depth_str = (
                        f"{self._pressure_depth_m(pressure):.3f} m"
                        if pressure is not None
                        else "unknown"
                    )
                    self.get_logger().debug(
                        f"Geometric touch: dist={distance:.3f} m  "
                        f"pressure_depth={depth_str}"
                    )
                    return True
                # Collision spike while verified near surface → touching
                if collision and pressure is not None and self._pressure_near_surface(pressure):
                    self.get_logger().debug(
                        f"IMU collision near surface: accel spike + "
                        f"pressure={pressure:.0f} Pa"
                    )
                    return True
                return False

        # ── path 2: ultrasonic faulty – rely on pressure (+ optional IMU) ─────
        if pressure is not None:
            if self._pressure_fallback_touching(pressure):
                self.get_logger().debug(
                    f"Pressure fallback touch: {pressure:.0f} Pa  "
                    f"(depth ≈ {self._pressure_depth_m(pressure):.3f} m)"
                )
                return True
            if collision and self._pressure_near_surface(pressure):
                self.get_logger().debug(
                    f"IMU collision near surface (ultrasonic invalid): "
                    f"pressure={pressure:.0f} Pa"
                )
                return True

        return False

    def _detection_tick(self) -> None:
        confirm = int(self.get_parameter("confirm_count").value)
        clear = int(self.get_parameter("clear_count").value)

        raw = self._compute_raw_touch()

        if raw:
            self._confirm_count += 1
            self._clear_count = 0
            if self._confirm_count >= confirm:
                if not self._is_touching:
                    self.get_logger().info("ICE CONTACT DETECTED")
                self._is_touching = True
        else:
            self._clear_count += 1
            self._confirm_count = 0
            if self._clear_count >= clear:
                if self._is_touching:
                    self.get_logger().info("Ice contact cleared")
                self._is_touching = False

        msg = Bool()
        msg.data = self._is_touching
        self._pub.publish(msg)


# ── helpers ────────────────────────────────────────────────────────────────────


def _roll_pitch_from_quat(
    qx: float, qy: float, qz: float, qw: float
) -> tuple[float, float]:
    """Extract roll and pitch (radians) from a unit quaternion (body → world)."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))  # clamp for numerical safety
    pitch = math.asin(sinp)

    return roll, pitch


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = IceTouchDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
