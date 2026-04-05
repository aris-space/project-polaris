"""
Subscribes to /sensors/dvl/odometry and republishes to /sensors/dvl/odometry_cov
with twist covariance for robot_localization.

Default (twist_linear_covariance_model == "stationary_tep"):
  When bottom lock is held: constant diagonal linear twist variances from pool
  stationary_02 analysis (lock-masked, header.stamp) — see
  measurement_noise_constants.py and recordings/stationary_tep_stats.json.
  Those base variances are **not** edited when tuning; use
  lock_linear_variance_bias_drift_inflation_factor (default > 1) to widen the
  covariance slightly for slow DVL bias / drift seen in long stationary runs.

Alternative (twist_linear_covariance_model == "speed_dependent"):
  DVL-A50 datasheet-style σ proportional to speed (standard / performance variant).

When bottom lock is lost or velocity topic is stale: linear variances set to
no_lock_variance (large m²/s²) so the EKF effectively ignores the update.

Parameters:
    twist_linear_covariance_model: "stationary_tep" | "speed_dependent"
    dvl_variant        (string): used only for speed_dependent mode
    no_lock_variance   (double): Variance when no lock [m²/s²] (default: 1e6)
    lock_variance_vx   (double): override stationary vx variance when model is stationary_tep
    lock_variance_vy   (double): override stationary vy variance
    lock_variance_vz   (double): override stationary vz variance (before floor)
    lock_variance_vz_floor (double): max(raw vz variance, floor); set 0 to disable
    lock_linear_variance_bias_drift_inflation_factor (double): multiply vx/vy/vz
        lock variances in stationary_tep only (default 1.15); base lock_variance_*
        and constants file stay the pure stationary sample values.
    angular_covariance (double): Angular rate variance (unused by DVL)
    velocity_stale_timeout_sec (double): lock flag timeout [s] (default: 0.5)
    dedupe_same_stamp_twist (bool): If true, do not publish when this message has
        the same header.stamp and (within epsilon) the same linear twist as the
        previous /sensors/dvl/odometry sample. Suppresses the redundant second
        odometry publish from the DVL driver (dead reckoning) that reuses the
        velocity twist at an identical stamp — avoids double EKF injections.
    dedupe_twist_epsilon (double): max abs delta per linear velocity component [m/s]
"""


import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from marine_acoustic_msgs.msg import Dvl

from dvl_a50_pkg.measurement_noise_constants import (
    DVL_LOCK_LINEAR_VARIANCE_X_M2_S2,
    DVL_LOCK_LINEAR_VARIANCE_Y_M2_S2,
    DVL_LOCK_LINEAR_VARIANCE_Z_RAW_M2_S2,
    DEFAULT_LOCK_LINEAR_VARIANCE_Z_FLOOR_M2_S2,
)

# DVL-A50 accuracy specs from Water Linked
DVL_VARIANTS = {
    "standard": {
        "sigma_scale": 0.0101,   # ±1.01% of measured speed
        "sigma_min": 0.01,       # floor at ~1 cm/s std dev
    },
    "performance": {
        "sigma_scale": 0.001,    # ±0.1% of measured speed
        "sigma_min": 0.001,      # floor at ~1 mm/s std dev
    },
}


class OdometryCovarianceNode(Node):

    def __init__(self):
        super().__init__("dvl_odometry_covariance")

        self.declare_parameter("twist_linear_covariance_model", "stationary_tep")
        self.declare_parameter("dvl_variant", "performance")
        self.declare_parameter("no_lock_variance", 1.0e6)
        self.declare_parameter("lock_variance_vx", DVL_LOCK_LINEAR_VARIANCE_X_M2_S2)
        self.declare_parameter("lock_variance_vy", DVL_LOCK_LINEAR_VARIANCE_Y_M2_S2)
        self.declare_parameter("lock_variance_vz", DVL_LOCK_LINEAR_VARIANCE_Z_RAW_M2_S2)
        self.declare_parameter(
            "lock_variance_vz_floor", DEFAULT_LOCK_LINEAR_VARIANCE_Z_FLOOR_M2_S2
        )
        self.declare_parameter(
            "lock_linear_variance_bias_drift_inflation_factor",
            1.15,
        )
        self.declare_parameter("angular_covariance", 1000000.0)
        self.declare_parameter("velocity_stale_timeout_sec", 0.5)
        self.declare_parameter("dedupe_same_stamp_twist", True)
        self.declare_parameter("dedupe_twist_epsilon", 1.0e-9)

        self._cov_model = (
            self.get_parameter("twist_linear_covariance_model").value or "stationary_tep"
        ).strip().lower()
        if self._cov_model not in ("stationary_tep", "speed_dependent"):
            self.get_logger().error(
                f"Unknown twist_linear_covariance_model '{self._cov_model}', "
                "using 'stationary_tep'."
            )
            self._cov_model = "stationary_tep"

        variant = self.get_parameter("dvl_variant").value
        if variant not in DVL_VARIANTS:
            self.get_logger().error(
                f"Unknown dvl_variant '{variant}', "
                f"expected one of {list(DVL_VARIANTS.keys())}. "
                f"Falling back to 'standard'."
            )
            variant = "standard"

        self.sigma_scale = DVL_VARIANTS[variant]["sigma_scale"]
        self.sigma_min = DVL_VARIANTS[variant]["sigma_min"]
        self.no_lock_var = float(self.get_parameter("no_lock_variance").value)
        self._lock_var_vx = float(self.get_parameter("lock_variance_vx").value)
        self._lock_var_vy = float(self.get_parameter("lock_variance_vy").value)
        self._lock_var_vz_raw = float(self.get_parameter("lock_variance_vz").value)
        self._lock_var_vz_floor = float(
            self.get_parameter("lock_variance_vz_floor").value
        )
        self._lock_bias_drift_inflation = float(
            self.get_parameter(
                "lock_linear_variance_bias_drift_inflation_factor"
            ).value
        )
        if self._lock_bias_drift_inflation <= 0.0:
            self.get_logger().warning(
                "lock_linear_variance_bias_drift_inflation_factor <= 0; using 1.0"
            )
            self._lock_bias_drift_inflation = 1.0
        self.ang_cov = self.get_parameter("angular_covariance").value
        self.velocity_stale_timeout_sec = (
            self.get_parameter("velocity_stale_timeout_sec").value
        )
        self._dedupe_same_stamp = bool(
            self.get_parameter("dedupe_same_stamp_twist").value
        )
        self._dedupe_eps = float(self.get_parameter("dedupe_twist_epsilon").value)
        if self._dedupe_eps < 0.0:
            self.get_logger().warning("dedupe_twist_epsilon < 0; using 0.0")
            self._dedupe_eps = 0.0

        self.bottom_lock = False
        self.last_velocity_stamp = None
        self._prev_odom_stamp_ns: int | None = None
        self._prev_odom_lx: float | None = None
        self._prev_odom_ly: float | None = None
        self._prev_odom_lz: float | None = None

        self.vel_sub = self.create_subscription(
            Dvl,
            "/sensors/dvl/velocity",
            self.velocity_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "/sensors/dvl/odometry",
            self.odometry_callback,
            10,
        )
        self.pub = self.create_publisher(
            Odometry,
            "/sensors/dvl/odometry_cov",
            10,
        )

        vxv, vyv, vzv = self._stationary_lock_linear_variances_effective()
        self.get_logger().info(
            f"DVL odometry_cov: model={self._cov_model}, "
            f"no_lock_variance={self.no_lock_var}, "
            f"velocity_stale_timeout_sec={self.velocity_stale_timeout_sec}"
            + (
                f", stationary_tep: base_lock_var_vx_vy_vz=({self._lock_var_vx:g},"
                f"{self._lock_var_vy:g},{self._effective_lock_vz_variance():g}), "
                f"bias_drift_inflation={self._lock_bias_drift_inflation:g}, "
                f"effective_lock_var_vx_vy_vz=({vxv:g},{vyv:g},{vzv:g})"
                if self._cov_model == "stationary_tep"
                else ""
            )
            + (
                f", speed_dep: variant={variant}, sigma_scale={self.sigma_scale}"
                if self._cov_model == "speed_dependent"
                else ""
            )
            + (
                f", dedupe_same_stamp_twist={self._dedupe_same_stamp}, "
                f"dedupe_twist_epsilon={self._dedupe_eps:g}"
                if self._dedupe_same_stamp
                else ", dedupe_same_stamp_twist=false"
            )
        )

    def _effective_lock_vz_variance(self) -> float:
        z = self._lock_var_vz_raw
        f = self._lock_var_vz_floor
        if f > 0.0:
            return max(z, f)
        return z

    def _stationary_lock_linear_variances_effective(self) -> tuple[float, float, float]:
        """Vx, Vy, Vz variances [m²/s²] for stationary_tep after vz floor and drift inflation."""
        inf = self._lock_bias_drift_inflation
        return (
            self._lock_var_vx * inf,
            self._lock_var_vy * inf,
            self._effective_lock_vz_variance() * inf,
        )

    def velocity_callback(self, msg: Dvl):
        # Driver maps DVL velocity_valid -> beam_velocities_valid.
        # Use this as bottom-lock proxy and guard it with staleness timeout.
        self.bottom_lock = msg.beam_velocities_valid
        self.last_velocity_stamp = self.get_clock().now()

    def _lock_is_stale(self) -> bool:
        if self.last_velocity_stamp is None:
            return True

        age = (self.get_clock().now() - self.last_velocity_stamp).nanoseconds / 1e9
        return age > self.velocity_stale_timeout_sec

    def _twist_linear_near(
        self,
        ax: float,
        ay: float,
        az: float,
        bx: float,
        by: float,
        bz: float,
    ) -> bool:
        e = self._dedupe_eps
        return (
            abs(ax - bx) <= e
            and abs(ay - by) <= e
            and abs(az - bz) <= e
        )

    def odometry_callback(self, msg: Odometry):
        stamp_ns = int(msg.header.stamp.sec) * 10**9 + int(msg.header.stamp.nanosec)
        lin = msg.twist.twist.linear
        lx, ly, lz = float(lin.x), float(lin.y), float(lin.z)

        if self._dedupe_same_stamp and self._prev_odom_stamp_ns is not None:
            if stamp_ns == self._prev_odom_stamp_ns and self._twist_linear_near(
                lx,
                ly,
                lz,
                self._prev_odom_lx,
                self._prev_odom_ly,
                self._prev_odom_lz,
            ):
                return

        self._prev_odom_stamp_ns = stamp_ns
        self._prev_odom_lx = lx
        self._prev_odom_ly = ly
        self._prev_odom_lz = lz

        cov = list(msg.twist.covariance)

        if self._lock_is_stale() or not self.bottom_lock:
            cov[0] = self.no_lock_var
            cov[7] = self.no_lock_var
            cov[14] = self.no_lock_var
        elif self._cov_model == "stationary_tep":
            evx, evy, evz = self._stationary_lock_linear_variances_effective()
            cov[0] = evx
            cov[7] = evy
            cov[14] = evz
        else:
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            vz = msg.twist.twist.linear.z
            speed = math.sqrt(vx * vx + vy * vy + vz * vz)

            sigma_xy = max(self.sigma_min, self.sigma_scale * speed)
            sigma_z = 2.0 * sigma_xy

            cov[0] = sigma_xy * sigma_xy
            cov[7] = sigma_xy * sigma_xy
            cov[14] = sigma_z * sigma_z

        cov[21] = self.ang_cov   # wx — not available
        cov[28] = self.ang_cov   # wy — not available
        cov[35] = self.ang_cov   # wz — not available

        msg.twist.covariance = cov
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryCovarianceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
