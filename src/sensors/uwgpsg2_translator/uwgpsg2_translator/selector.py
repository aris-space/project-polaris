"""
Select GNSS vs SBL NavSatFix for /gps/selected.

- Publishes /fix to /gps/selected only when horizontal accuracy is within
  max_horizontal_accuracy_m at horizontal_accuracy_confidence (default 95%),
  otherwise relies on SBL after the usual stale timeout.
- Horizontal accuracy: optional match to ublox UBXNavHPPosLLH.h_acc; else derived
  from NavSatFix.position_covariance (ENU), consistent with ublox_nav_sat_fix_hp_node.
"""
from __future__ import annotations

import math
from collections import deque
from statistics import NormalDist, stdev

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 10**9 + int(stamp.nanosec)


def _horizontal_sigma_m_from_enu_covariance_row_major(cov: list[float]) -> float | None:
    """Largest horizontal 1-sigma (sqrt of max eigenvalue of East–North 2x2 block)."""
    if len(cov) < 9:
        return None
    a = float(cov[0])
    b = float(cov[1])
    d = float(cov[4])
    trace = a + d
    det = a * d - b * b
    disc = trace * trace - 4.0 * det
    if disc < 0.0:
        disc = 0.0
    lam_max = 0.5 * (trace + math.sqrt(disc))
    return math.sqrt(max(0.0, lam_max))


def horizontal_accuracy_m_from_nav_sat_fix(msg: NavSatFix) -> float | None:
    """
    Scalar horizontal uncertainty (meters, ~1-sigma worst horizontal axis) from /fix.

    NavSatFix has no h_acc field; ublox_hp fills ENU covariance from UBX-NAV-COV.
    """
    t = int(msg.position_covariance_type)
    cov = list(msg.position_covariance)
    if t == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
        return None
    if t == NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN:
        if len(cov) < 5:
            return None
        return math.sqrt(max(0.0, float(cov[0]), float(cov[4])))
    if t in (
        NavSatFix.COVARIANCE_TYPE_KNOWN,
        NavSatFix.COVARIANCE_TYPE_APPROXIMATED,
    ):
        return _horizontal_sigma_m_from_enu_covariance_row_major(cov)
    return None


def sigma_to_confidence_bound_m(sigma_m: float, confidence: float) -> float:
    """Convert a 1-sigma scalar to a symmetric normal confidence bound."""
    z = NormalDist().inv_cdf(0.5 + 0.5 * confidence)
    return sigma_m * z


def _unwrap_stdev(yaws: list[float]) -> float:
    """Std of a yaw sequence (rad) after unwrapping the ±π boundary."""
    TWO_PI = 2.0 * math.pi
    unwrapped = [yaws[0]]
    for i in range(1, len(yaws)):
        diff = (yaws[i] - yaws[i - 1] + math.pi) % TWO_PI - math.pi
        unwrapped.append(unwrapped[-1] + diff)
    return stdev(unwrapped)


class Selector(Node):
    def __init__(self):
        super().__init__("selector")

        self.declare_parameter("max_horizontal_accuracy_m", 4.0)
        self.declare_parameter("horizontal_accuracy_confidence", 0.95)
        self.declare_parameter("gps_stale_seconds", 2.0)
        self.declare_parameter("use_horizontal_accuracy_gate", True)
        # If set, prefer h_acc from ublox UBXNavHPPosLLH when stamp matches /fix (see ubx_stamp_match_max_ns).
        self.declare_parameter("ubx_nav_hp_pos_llh_topic", "")
        self.declare_parameter("ubx_stamp_match_max_ns", 50_000_000)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("imu_heading_stable_window_s", 30.0)
        self.declare_parameter("imu_heading_stable_threshold_deg", 0.1)
        self.declare_parameter("imu_heading_timeout_s", 600.0)

        self._max_h_acc_m = (
            self.get_parameter("max_horizontal_accuracy_m").get_parameter_value().double_value
        )
        self._h_acc_confidence = (
            self.get_parameter("horizontal_accuracy_confidence")
            .get_parameter_value()
            .double_value
        )
        if self._h_acc_confidence <= 0.0 or self._h_acc_confidence >= 1.0:
            self.get_logger().warning(
                f"horizontal_accuracy_confidence={self._h_acc_confidence} is outside (0, 1); "
                "falling back to 0.95."
            )
            self._h_acc_confidence = 0.95
        self._gps_stale_s = (
            self.get_parameter("gps_stale_seconds").get_parameter_value().double_value
        )
        self._use_gate = (
            self.get_parameter("use_horizontal_accuracy_gate").get_parameter_value().bool_value
        )
        ubx_topic = (
            self.get_parameter("ubx_nav_hp_pos_llh_topic").get_parameter_value().string_value.strip()
        )
        self._ubx_match_max_ns = (
            self.get_parameter("ubx_stamp_match_max_ns").get_parameter_value().integer_value
        )
        imu_topic = (
            self.get_parameter("imu_topic").get_parameter_value().string_value.strip()
        )
        self._imu_window_s = (
            self.get_parameter("imu_heading_stable_window_s").get_parameter_value().double_value
        )
        self._imu_threshold_rad = math.radians(
            self.get_parameter("imu_heading_stable_threshold_deg")
            .get_parameter_value()
            .double_value
        )
        self._imu_timeout_s = (
            self.get_parameter("imu_heading_timeout_s").get_parameter_value().double_value
        )

        self.get_logger().info(
            f"Selector: gate={self._use_gate}, max_horizontal_accuracy_m={self._max_h_acc_m}, "
            f"horizontal_accuracy_confidence={self._h_acc_confidence}, gps_stale_s={self._gps_stale_s}"
        )
        self.get_logger().info(
            f"IMU yaw gate: topic={imu_topic!r}, window={self._imu_window_s}s, "
            f"threshold={math.degrees(self._imu_threshold_rad):.3f}deg, "
            f"timeout={self._imu_timeout_s}s"
        )

        self.ekf_gps_publisher = self.create_publisher(
            NavSatFix, "/gps/selected", qos_profile_sensor_data
        )
        self.ekf_gps_subscriber = self.create_subscription(
            NavSatFix, "/fix", self.ekf_gps_callback, qos_profile_sensor_data
        )
        self.sbl_gps_subscriber = self.create_subscription(
            NavSatFix, "/waterlinked_ugps/navsatfix", self.sbl_gps_callback, qos_profile_sensor_data
        )

        # Last time we published an *accepted* GNSS fix to /gps/selected (ROS clock).
        self._last_accepted_gps_time = None
        self._ubx_ring: deque[tuple[int, float]] = deque(maxlen=256)
        self._imu_ring: deque[tuple[float, float]] = deque()
        self._imu_heading_stable: bool = False
        self._imu_first_stamp_s: float | None = None
        self._last_suppression_log_s: float = 0.0

        self.create_subscription(Imu, imu_topic, self._imu_callback, qos_profile_sensor_data)

        if ubx_topic:
            try:
                from ublox_ubx_msgs.msg import UBXNavHPPosLLH

                self.create_subscription(
                    UBXNavHPPosLLH,
                    ubx_topic,
                    self._ubx_hp_pos_llh_callback,
                    qos_profile_sensor_data,
                )
                self.get_logger().info(f"Subscribed to UBX HPPosLLH for h_acc: {ubx_topic!r}")
            except ImportError:
                self.get_logger().error(
                    "ubx_nav_hp_pos_llh_topic set but ublox_ubx_msgs is not available; "
                    "install ublox_ubx_msgs or clear the parameter."
                )

    def _ubx_hp_pos_llh_callback(self, msg) -> None:
        # h_acc: mm * 0.1 -> meters
        h_m = float(msg.h_acc) * 1e-4
        self._ubx_ring.append((_stamp_ns(msg.header.stamp), h_m))

    def _horizontal_accuracy_m_for_fix(self, msg: NavSatFix) -> float | None:
        t_fix = _stamp_ns(msg.header.stamp)
        best_h: float | None = None
        best_dt = self._ubx_match_max_ns + 1
        for t_ubx, h_m in self._ubx_ring:
            dt = abs(t_fix - t_ubx)
            if dt < best_dt:
                best_dt = dt
                best_h = h_m
        if best_h is not None and best_dt <= self._ubx_match_max_ns:
            return best_h
        return horizontal_accuracy_m_from_nav_sat_fix(msg)

    def _imu_callback(self, msg: Imu) -> None:
        q = msg.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self._imu_first_stamp_s is None:
            self._imu_first_stamp_s = stamp_s

        if self._imu_heading_stable:
            return

        self._imu_ring.append((stamp_s, yaw))
        while self._imu_ring and (stamp_s - self._imu_ring[0][0]) > self._imu_window_s:
            self._imu_ring.popleft()

        if (stamp_s - self._imu_first_stamp_s) > self._imu_timeout_s:
            self._imu_heading_stable = True
            self.get_logger().warning(
                f"IMU yaw gate: timeout after {self._imu_timeout_s:.0f}s — "
                "forcing stable=True. GPS datum may be inaccurate."
            )
            return

        if len(self._imu_ring) < 10:
            return
        if (stamp_s - self._imu_ring[0][0]) < self._imu_window_s:
            return

        std_rad = _unwrap_stdev([y for _, y in self._imu_ring])
        if std_rad < self._imu_threshold_rad:
            self._imu_heading_stable = True
            self.get_logger().info(
                f"IMU yaw gate: heading stable — std={math.degrees(std_rad):.4f}deg "
                f"over {self._imu_window_s:.0f}s window. GPS output now enabled."
            )

    def ekf_gps_callback(self, msg: NavSatFix) -> None:
        if not self._imu_heading_stable:
            now_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if (now_s - self._last_suppression_log_s) >= 10.0:
                self._last_suppression_log_s = now_s
                self.get_logger().debug(
                    "IMU yaw gate: suppressing /fix → /gps/selected (heading not yet stable)."
                )
            return

        if not self._use_gate:
            self._last_accepted_gps_time = self.get_clock().now()
            self.ekf_gps_publisher.publish(msg)
            return

        h_m = self._horizontal_accuracy_m_for_fix(msg)
        if h_m is None:
            self.get_logger().debug(
                "Skipping /fix for /gps/selected: no horizontal accuracy (UNKNOWN covariance "
                "and no matching UBX h_acc)."
            )
            return

        h_bound_m = sigma_to_confidence_bound_m(h_m, self._h_acc_confidence)
        if h_bound_m > self._max_h_acc_m:
            self.get_logger().debug(
                f"Skipping /fix for /gps/selected: horizontal accuracy "
                f"{self._h_acc_confidence:.0%} bound {h_bound_m:.2f} m "
                f"> {self._max_h_acc_m:.2f} m."
            )
            return

        self._last_accepted_gps_time = self.get_clock().now()
        self.ekf_gps_publisher.publish(msg)

    def sbl_gps_callback(self, msg: NavSatFix) -> None:
        if not self._imu_heading_stable:
            now_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if (now_s - self._last_suppression_log_s) >= 10.0:
                self._last_suppression_log_s = now_s
                self.get_logger().debug(
                    "IMU yaw gate: suppressing SBL → /gps/selected (heading not yet stable)."
                )
            return

        if self._last_accepted_gps_time is None:
            self.ekf_gps_publisher.publish(msg)
            return

        duration_since_good_gps = (
            self.get_clock().now() - self._last_accepted_gps_time
        ).nanoseconds / 1e9

        if duration_since_good_gps > self._gps_stale_s:
            self.ekf_gps_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    selector = Selector()
    rclpy.spin(selector)
    selector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
