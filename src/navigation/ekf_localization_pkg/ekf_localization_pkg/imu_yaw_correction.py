"""
imu_yaw_correction — pre-EKF yaw bias correction for the local EKF's IMU input.

Subscribes to /imu/data, rotates the orientation quaternion by yaw_offset_deg
about +Z (ENU CCW), and republishes on /imu/data_corrected. The local EKF
should be configured to use that as its imu0 source so the EKF *and*
everything downstream (controllers, /odometry/filtered/local, the local TF
tree) all operate on a calibrated heading.

Angular velocity and linear acceleration pass through unchanged: they are
in the sensor body frame, which is unaffected by a rotation of the world
frame's yaw reference.

Service ~/calibrate_yaw_offset (std_srvs/Trigger) triggers a GNSS-based
calibration. After the service call returns, drive the AUV forward in a
straight line for yaw_calibration_duration_s seconds.

Heading sources, in priority order:
  1. UBX-NAV-PVT head_mot (Doppler course over ground), gated by head_acc.
     This is the receiver's own COG estimate, computed from carrier-phase
     Doppler — much lower noise than differentiating two position fixes.
  2. Two-fix haversine bearing (start fix vs latest fix). Used as fallback
     if head_mot was not valid often enough during the window.

The calibration sets yaw_offset_deg = circular_avg(GNSS bearing) -
circular_avg(raw IMU yaw), via the live parameter callback.

Parameters
----------
yaw_offset_deg              Rotation about +Z applied to orientation.
                            Live-tunable. Default 0.
input_topic                 Default /imu/data
output_topic                Default /imu/data_corrected
gps_topic                   Default /fix (used for fallback bearing)
ubx_pvt_topic               Default /ubx_nav_pvt. Empty disables head_mot path.
yaw_calibration_duration_s  Calibration window length (default 10.0)
yaw_calibration_min_distance_m  Min distance for the two-fix fallback to be
                                accepted (default 3.0)
head_acc_max_deg            Max head_mot uncertainty per sample (default 5.0)
head_mot_min_samples        Min valid head_mot samples in the window before
                            preferring head_mot over two-fix (default 5)
"""
from __future__ import annotations

import math

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, NavSatFix
from std_srvs.srv import Trigger

try:
    from ublox_ubx_msgs.msg import UBXNavPVT as _UBXNavPVT
    _UBX_PVT_AVAILABLE = True
except ImportError:
    _UBXNavPVT = None
    _UBX_PVT_AVAILABLE = False


_EARTH_R_M = 6_371_000.0


# ── geometry helpers ──────────────────────────────────────────────────────────

def _gnss_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlon / 2.0) ** 2
    return 2.0 * _EARTH_R_M * math.asin(math.sqrt(a))


def _gnss_bearing_enu_rad(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Two-fix bearing → ENU yaw (rad). 0 = +x = East, +π/2 = +y = North."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    y = math.sin(dlon) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlon)
    bearing_ned = math.atan2(y, x)              # 0 = N, +π/2 = E (CW from N)
    yaw_enu = math.pi / 2.0 - bearing_ned
    return math.atan2(math.sin(yaw_enu), math.cos(yaw_enu))


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _ubx_scaled_to_deg(raw) -> float:
    """u-blox UBX-NAV-PVT head_mot/head_acc fields. Spec says int32/uint32 scaled
    by 1e-5 deg, but some ROS wrappers pre-scale to plain degrees.
    Heuristic: |raw| > 720 → still in raw scaled int form; else assume degrees."""
    v = float(raw)
    if abs(v) > 720.0:
        return v * 1e-5
    return v


def _head_mot_to_enu_rad(head_mot_raw) -> float:
    """u-blox head_mot (CW from N, deg) → ENU yaw (rad, CCW from E)."""
    bearing_ned = math.radians(_ubx_scaled_to_deg(head_mot_raw))
    yaw_enu = math.pi / 2.0 - bearing_ned
    return math.atan2(math.sin(yaw_enu), math.cos(yaw_enu))


# ── node ─────────────────────────────────────────────────────────────────────

class ImuYawCorrection(Node):

    def __init__(self) -> None:
        super().__init__("imu_yaw_correction")

        self.declare_parameter("yaw_offset_deg", 0.0)
        self.declare_parameter("input_topic", "/imu/data")
        self.declare_parameter("output_topic", "/imu/data_corrected")
        self.declare_parameter("gps_topic", "/fix")
        self.declare_parameter("ubx_pvt_topic", "/ubx_nav_pvt")
        self.declare_parameter("yaw_calibration_duration_s", 10.0)
        self.declare_parameter("yaw_calibration_min_distance_m", 3.0)
        self.declare_parameter("head_acc_max_deg", 5.0)
        self.declare_parameter("head_mot_min_samples", 5)

        input_topic: str = self.get_parameter("input_topic").value
        output_topic: str = self.get_parameter("output_topic").value
        gps_topic: str = self.get_parameter("gps_topic").value
        ubx_pvt_topic: str = self.get_parameter("ubx_pvt_topic").value

        # Yaw rotation cache (half-angle quaternion for orientation rotation about +Z).
        self._yaw_offset_rad: float = 0.0
        self._q_yaw_w: float = 1.0
        self._q_yaw_z: float = 0.0
        self._update_yaw_cache(float(self.get_parameter("yaw_offset_deg").value))

        # Latest sensor state (for calibration).
        self._latest_imu: Imu | None = None
        self._latest_fix: NavSatFix | None = None

        # Calibration state.
        self._cal_active: bool = False
        self._cal_start_time = None
        self._cal_start_fix: NavSatFix | None = None
        self._cal_yaw_samples: list[float] = []
        self._cal_head_mot_samples: list[tuple[float, float]] = []  # (yaw_enu_rad, head_acc_deg)
        self._cal_timer = None

        # I/O.
        # Publisher: RELIABLE so downstream consumers (local EKF, anything
        # else fusing /imu/data_corrected) never silently lose messages
        # under callback-queue load. The Xsens publishes /imu/data RELIABLE
        # at 100 Hz; we preserve that delivery semantic. Earlier the
        # publisher used qos_profile_sensor_data (BEST_EFFORT, depth=5).
        # The bags recorded with that earlier code carry BEST_EFFORT in
        # their stored offered_qos_profiles — see metadata.yaml — and
        # cannot be retroactively upgraded. Future bags recorded with this
        # fix in place will store RELIABLE.
        _imu_pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(Imu, output_topic, _imu_pub_qos)
        # Subscribers stay sensor-style (BEST_EFFORT) so they're compatible
        # with both RELIABLE and BEST_EFFORT upstreams. /imu/data is
        # published RELIABLE by the Xsens driver; /gps and /ubx_nav_pvt
        # are usually RELIABLE too. RELIABLE→BEST_EFFORT is a compatible
        # combo, so this works in both live and offline-replay paths.
        self.create_subscription(Imu, input_topic, self._on_imu, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, gps_topic, self._on_gps, qos_profile_sensor_data)

        self._ubx_enabled = bool(ubx_pvt_topic) and _UBX_PVT_AVAILABLE
        if self._ubx_enabled:
            self.create_subscription(
                _UBXNavPVT, ubx_pvt_topic, self._on_ubx_pvt, qos_profile_sensor_data
            )
        elif ubx_pvt_topic and not _UBX_PVT_AVAILABLE:
            self.get_logger().warn(
                "ublox_ubx_msgs not available — head_mot path disabled, only "
                "two-fix bearing fallback will be used in calibration."
            )

        self.add_on_set_parameters_callback(self._on_param_change)
        self.create_service(Trigger, "~/calibrate_yaw_offset", self._on_calibrate_request)

        self.get_logger().info(
            f"ImuYawCorrection: in='{input_topic}' out='{output_topic}'  "
            f"yaw_offset_deg={math.degrees(self._yaw_offset_rad):+.3f}°"
        )
        self.get_logger().info(
            f"Calibration sources: gps='{gps_topic}', "
            f"ubx_pvt='{ubx_pvt_topic if self._ubx_enabled else 'disabled'}'  "
            f"(window {float(self.get_parameter('yaw_calibration_duration_s').value):.1f}s)"
        )

    # ------------------------------------------------------------------

    def _update_yaw_cache(self, yaw_offset_deg: float) -> None:
        self._yaw_offset_rad = math.radians(yaw_offset_deg)
        self._q_yaw_w = math.cos(self._yaw_offset_rad / 2.0)
        self._q_yaw_z = math.sin(self._yaw_offset_rad / 2.0)

    def _on_param_change(self, params) -> SetParametersResult:
        for p in params:
            if p.name == "yaw_offset_deg":
                try:
                    val = float(p.value)
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False, reason="yaw_offset_deg must be a number"
                    )
                self._update_yaw_cache(val)
                self.get_logger().info(
                    f"yaw_offset_deg → {val:+.3f}° (applies to next /imu/data message)"
                )
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------

    def _on_imu(self, msg: Imu) -> None:
        self._latest_imu = msg

        out = Imu()
        out.header = msg.header
        out.orientation_covariance     = msg.orientation_covariance
        out.angular_velocity            = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration            = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        if self._q_yaw_z == 0.0:
            out.orientation = msg.orientation
        else:
            qx = msg.orientation.x
            qy = msg.orientation.y
            qz = msg.orientation.z
            qw = msg.orientation.w
            cz, sz = self._q_yaw_w, self._q_yaw_z
            # q_out = q_yaw_offset ⊗ q_in,   q_yaw_offset = (0, 0, sz, cz)
            out.orientation.w = cz * qw - sz * qz
            out.orientation.x = cz * qx - sz * qy
            out.orientation.y = cz * qy + sz * qx
            out.orientation.z = cz * qz + sz * qw

        self._pub.publish(out)

    def _on_gps(self, msg: NavSatFix) -> None:
        if msg.status.status < 0:
            return
        if abs(msg.latitude) < 0.1 and abs(msg.longitude) < 0.1:
            return
        self._latest_fix = msg

    def _on_ubx_pvt(self, msg) -> None:
        if not self._cal_active:
            return
        # Extract head_mot/head_acc; tolerate field-name variations across ublox driver versions.
        try:
            head_mot_raw = msg.head_mot
            head_acc_raw = msg.head_acc
        except AttributeError:
            return
        head_acc_deg = _ubx_scaled_to_deg(head_acc_raw)
        max_acc_deg = float(self.get_parameter("head_acc_max_deg").value)
        if head_acc_deg <= 0.0 or head_acc_deg > max_acc_deg:
            return
        yaw_enu = _head_mot_to_enu_rad(head_mot_raw)
        self._cal_head_mot_samples.append((yaw_enu, head_acc_deg))

    # ------------------------------------------------------------------
    # Calibration

    def _on_calibrate_request(self, request, response):
        duration = float(self.get_parameter("yaw_calibration_duration_s").value)
        if self._cal_active:
            response.success = False
            response.message = "Yaw calibration already in progress"
            return response
        if self._latest_imu is None:
            response.success = False
            response.message = "No /imu/data received yet"
            return response
        if self._latest_fix is None:
            response.success = False
            response.message = "No /fix received yet (will need it for two-fix fallback)"
            return response

        self._cal_active = True
        self._cal_start_time = self.get_clock().now()
        self._cal_start_fix = self._latest_fix
        self._cal_yaw_samples = []
        self._cal_head_mot_samples = []
        self._cal_timer = self.create_timer(0.1, self._on_calibration_tick)

        response.success = True
        response.message = (
            f"Yaw calibration started — drive forward in a straight line for {duration:.0f} s. "
            f"Heading source: {'head_mot (preferred)' if self._ubx_enabled else 'two-fix only (head_mot disabled)'}."
        )
        self.get_logger().info(response.message)
        return response

    def _on_calibration_tick(self) -> None:
        if not self._cal_active or self._cal_start_time is None:
            return

        if self._latest_imu is not None:
            o = self._latest_imu.orientation
            self._cal_yaw_samples.append(_yaw_from_quat(o.x, o.y, o.z, o.w))

        duration = float(self.get_parameter("yaw_calibration_duration_s").value)
        elapsed = (self.get_clock().now() - self._cal_start_time).nanoseconds / 1e9
        if elapsed >= duration:
            self._finish_calibration()

    def _finish_calibration(self) -> None:
        if self._cal_timer is not None:
            self._cal_timer.cancel()
            self._cal_timer = None
        self._cal_active = False

        if not self._cal_yaw_samples:
            self.get_logger().warn("Yaw calibration aborted — no IMU yaw samples")
            return

        # Pick heading source.
        min_head_mot = int(self.get_parameter("head_mot_min_samples").value)
        bearing_rad = None
        source_str = ""
        if len(self._cal_head_mot_samples) >= min_head_mot:
            sx = sum(math.sin(b) for b, _ in self._cal_head_mot_samples)
            cx = sum(math.cos(b) for b, _ in self._cal_head_mot_samples)
            bearing_rad = math.atan2(sx, cx)
            avg_acc = sum(a for _, a in self._cal_head_mot_samples) / len(self._cal_head_mot_samples)
            source_str = (
                f"head_mot (N={len(self._cal_head_mot_samples)}, "
                f"avg head_acc={avg_acc:.2f}°)"
            )
        else:
            # Two-fix fallback.
            if self._cal_start_fix is None or self._latest_fix is None:
                self.get_logger().warn(
                    f"Yaw calibration aborted — head_mot insufficient "
                    f"({len(self._cal_head_mot_samples)} samples, need {min_head_mot}) "
                    "and no fixes for fallback"
                )
                return
            distance = _gnss_distance_m(
                self._cal_start_fix.latitude, self._cal_start_fix.longitude,
                self._latest_fix.latitude,    self._latest_fix.longitude,
            )
            min_dist = float(self.get_parameter("yaw_calibration_min_distance_m").value)
            if distance < min_dist:
                self.get_logger().warn(
                    f"Yaw calibration aborted — head_mot insufficient "
                    f"({len(self._cal_head_mot_samples)} samples) and fallback "
                    f"two-fix only moved {distance:.2f} m (need ≥ {min_dist:.2f} m)"
                )
                return
            bearing_rad = _gnss_bearing_enu_rad(
                self._cal_start_fix.latitude, self._cal_start_fix.longitude,
                self._latest_fix.latitude,    self._latest_fix.longitude,
            )
            source_str = f"two-fix bearing (distance={distance:.2f} m)"

        # Circular average of IMU yaw across the window.
        sx = sum(math.sin(y) for y in self._cal_yaw_samples)
        cx = sum(math.cos(y) for y in self._cal_yaw_samples)
        avg_yaw_rad = math.atan2(sx, cx)

        diff = bearing_rad - avg_yaw_rad
        diff_wrapped = math.atan2(math.sin(diff), math.cos(diff))
        new_offset_deg = math.degrees(diff_wrapped)

        # Apply via parameter; the on-set callback updates the rotation cache.
        self.set_parameters([
            Parameter("yaw_offset_deg", Parameter.Type.DOUBLE, new_offset_deg)
        ])

        self.get_logger().info(
            f"Yaw calibration complete. Source: {source_str}. "
            f"GNSS bearing(ENU)={math.degrees(bearing_rad):+.3f}°, "
            f"avg IMU yaw={math.degrees(avg_yaw_rad):+.3f}°  →  "
            f"yaw_offset_deg={new_offset_deg:+.3f}° (applied to /imu/data_corrected)."
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuYawCorrection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
