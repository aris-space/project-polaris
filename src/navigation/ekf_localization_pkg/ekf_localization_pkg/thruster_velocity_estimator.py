"""
DVL-fallback velocity estimator from surge thruster PWM.

When the DVL loses bottom-lock the EKF has no velocity input and the
accelerometer integration drifts without bound. This node provides a
body-frame velocity estimate derived from the surge thruster PWM command so
the EKF can keep the velocity state anchored during the outage.

Model (fitted on Zermatt rectangle survey recordings, zero measurable
current, 247 simulated dropout scenarios):

    vx = surge_a * sign(u) * |u|^surge_b    u = (pwm - pwm_neutral) / pwm_range
    vy = 0.0   (depth-hold survey; vy RMS < 0.05 m/s on straight legs)
    vz = 0.0   (depth-hold mode; vz RMS < 0.03 m/s throughout)

Dropout simulation results vs IMU-only integration:
    10s dropout:  5.4× better velocity,  3.4× better position (p50)
    30s dropout: 13.7× better velocity,  8.7× better position (p50)
    60s dropout: 25.9× better velocity, 16.5× better position (p50)
   120s dropout: 27.5× better velocity, 14.6× better position (p50)
   IMU-only median position error at 60s: 16.8 m → this node: 1.0 m

The node publishes ONLY when DVL has been absent for dvl_timeout_s seconds
so the EKF subscribes unconditionally:

    DVL covariance diagonal:  ~1e-8 (m/s)²
    Thrust model covariance:   0.006 (m/s)² for vx
    Ratio: ~600 000×  →  EKF immediately switches back to DVL on recovery

Parameters
----------
servo_topic           Int16MultiArray servo output        (/pixhawk/servo_output_raw)
dvl_topic             DVL velocity topic for lock monitor (/sensors/dvl/velocity)
output_topic          Published Odometry when DVL absent  (/sensors/thruster/odometry_cov)
surge_channel         Index into servo array (0-based)    (0)
pwm_neutral           PWM microseconds for zero thrust    (1500)
pwm_range             Half-range to normalise ±1          (500)
surge_a               Model coefficient a                 (0.780)
surge_b               Model exponent b                    (0.964)
surge_deadzone        |u| below which vx is forced 0      (0.01)
dvl_timeout_s         Seconds without valid DVL to activate  (3.0)
publish_rate_hz       Timer rate while active             (10.0)
cov_vx                Velocity covariance, surge axis     (0.0057)
cov_vy                Velocity covariance, lateral axis   (0.0027)
cov_vz                Velocity covariance, heave axis     (0.0009)
base_link_frame       Child frame (body frame)            (base_link)
odom_frame            Header frame (reference frame)      (odom)
"""
from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray

try:
    from marine_acoustic_msgs.msg import Dvl as _Dvl
    _DVL_MSG_AVAILABLE = True
except ImportError:
    _Dvl = None
    _DVL_MSG_AVAILABLE = False


class ThrusterVelocityEstimator(Node):
    """Publish body-frame velocity from surge PWM when DVL lock is lost."""

    def __init__(self) -> None:
        super().__init__("thruster_velocity_estimator")

        self.declare_parameter("servo_topic", "/pixhawk/servo_output_raw")
        self.declare_parameter("dvl_topic", "/sensors/dvl/velocity")
        self.declare_parameter("output_topic", "/sensors/thruster/odometry_cov")
        self.declare_parameter("surge_channel", 0)
        self.declare_parameter("pwm_neutral", 1500)
        self.declare_parameter("pwm_range", 500)
        self.declare_parameter("surge_a", 0.780)
        self.declare_parameter("surge_b", 0.964)
        self.declare_parameter("surge_deadzone", 0.01)
        self.declare_parameter("dvl_timeout_s", 3.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("cov_vx", 0.0057)
        self.declare_parameter("cov_vy", 0.0027)
        self.declare_parameter("cov_vz", 0.0009)
        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")

        self._servo_topic: str = self.get_parameter("servo_topic").value
        self._dvl_topic: str = self.get_parameter("dvl_topic").value
        self._output_topic: str = self.get_parameter("output_topic").value
        self._surge_ch: int = int(self.get_parameter("surge_channel").value)
        self._pwm_neutral: int = int(self.get_parameter("pwm_neutral").value)
        self._pwm_range: float = float(self.get_parameter("pwm_range").value)
        self._surge_a: float = self.get_parameter("surge_a").value
        self._surge_b: float = self.get_parameter("surge_b").value
        self._surge_deadzone: float = self.get_parameter("surge_deadzone").value
        self._dvl_timeout_s: float = self.get_parameter("dvl_timeout_s").value
        self._publish_rate_hz: float = self.get_parameter("publish_rate_hz").value
        self._cov_vx: float = self.get_parameter("cov_vx").value
        self._cov_vy: float = self.get_parameter("cov_vy").value
        self._cov_vz: float = self.get_parameter("cov_vz").value
        self._base_link_frame: str = self.get_parameter("base_link_frame").value
        self._odom_frame: str = self.get_parameter("odom_frame").value

        # Build twist covariance matrix once (6×6 row-major, [vx,vy,vz,wx,wy,wz])
        _BIG = 99999.0
        cov = [0.0] * 36
        cov[0]  = self._cov_vx
        cov[7]  = self._cov_vy
        cov[14] = self._cov_vz
        cov[21] = _BIG
        cov[28] = _BIG
        cov[35] = _BIG
        self._twist_covariance = cov

        self._last_pwm: int = self._pwm_neutral
        self._last_servo_wall_s: float = 0.0
        self._last_dvl_wall_s: float = 0.0
        self._dvl_active: bool = False  # True while DVL appears healthy
        self._node_active: bool = False  # True while we are publishing fallback

        self._pub = self.create_publisher(Odometry, self._output_topic, qos_profile_sensor_data)

        self._servo_sub = self.create_subscription(
            Int16MultiArray, self._servo_topic, self._on_servo, qos_profile_sensor_data
        )

        # Subscribe to DVL topic to track lock status.
        # If the custom message type is available, check validity flags.
        # If not (e.g. message package not installed), fall back to monitoring
        # any message on the topic as a liveness heartbeat.
        if _DVL_MSG_AVAILABLE:
            self._dvl_sub = self.create_subscription(
                _Dvl, self._dvl_topic, self._on_dvl, qos_profile_sensor_data
            )
        else:
            self.get_logger().warn(
                "marine_acoustic_msgs not available — DVL lock detection uses "
                "message presence only (no validity-flag check)."
            )
            from nav_msgs.msg import Odometry as _FallbackDvl  # noqa: F401
            self._dvl_sub = self.create_subscription(
                Odometry, "/sensors/dvl/odometry_cov",
                self._on_dvl_fallback, qos_profile_sensor_data,
            )

        self._timer = self.create_timer(1.0 / self._publish_rate_hz, self._on_timer)

        self.get_logger().info(
            f"thruster_velocity_estimator ready — model: "
            f"vx = {self._surge_a:.3f}·sign(u)·|u|^{self._surge_b:.3f}  "
            f"(ch{self._surge_ch}, neutral={self._pwm_neutral}, range=±{int(self._pwm_range)} µs)  "
            f"activates after {self._dvl_timeout_s:.1f}s DVL absence"
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_servo(self, msg: Int16MultiArray) -> None:
        if self._surge_ch < len(msg.data):
            self._last_pwm = int(msg.data[self._surge_ch])
        self._last_servo_wall_s = time.monotonic()

    def _on_dvl(self, msg: "_Dvl") -> None:
        valid = msg.beam_velocities_valid and msg.num_good_beams >= 3
        if valid:
            self._last_dvl_wall_s = time.monotonic()

    def _on_dvl_fallback(self, msg: Odometry) -> None:
        self._last_dvl_wall_s = time.monotonic()

    # ── Timer ─────────────────────────────────────────────────────────────────

    def _on_timer(self) -> None:
        now = time.monotonic()

        dvl_age_s = now - self._last_dvl_wall_s
        dvl_present = dvl_age_s < self._dvl_timeout_s

        if dvl_present:
            if self._node_active:
                self._node_active = False
                self.get_logger().info(
                    f"DVL recovered (age {dvl_age_s:.1f}s < {self._dvl_timeout_s:.1f}s) — "
                    "stopping thruster fallback"
                )
            return

        # DVL absent — publish thrust model
        if not self._node_active:
            self._node_active = True
            self.get_logger().warn(
                f"DVL absent for {dvl_age_s:.1f}s — activating thruster velocity fallback "
                f"(cov_vx={self._cov_vx:.4f})"
            )

        u = (self._last_pwm - self._pwm_neutral) / self._pwm_range
        if abs(u) < self._surge_deadzone:
            vx = 0.0
        else:
            vx = self._surge_a * math.copysign(abs(u) ** self._surge_b, u)

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_link_frame
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.covariance = self._twist_covariance
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThrusterVelocityEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
