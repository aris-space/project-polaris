"""
Subscribes to /sensors/dvl/odometry and republishes to /sensors/dvl/odometry_cov
with twist covariance for robot_localization.

Default (twist_linear_covariance_model == "stationary_tep"):
  When bottom lock is held: constant diagonal linear twist variances from pool
  stationary_02 analysis (lock-masked, header.stamp) — see
  measurement_noise_constants.py and recordings/stationary_tep_stats.json.
  Those variances are used directly (after optional vz floor).

Alternative (twist_linear_covariance_model == "speed_dependent"):
  Stationary variances plus a speed-proportional term, added in quadrature:
    var_i = var_stationary_i + (speed_sigma_scale * |v|)^2
  The stationary part is the same as stationary_tep (incl. vz floor), so at rest
  both models agree and the covariance grows smoothly with speed.

Alternative (twist_linear_covariance_model == "reported_fom"):
  Use the DVL's own figure-of-merit variance (Dvl.velocity_covar diagonal, filled by
  the driver from the Water Linked 3x3 cov, else diag(fom^2)), floored at the
  stationary variances so a deceptively small FOM can never make the EKF over-trust
  the update. This is the recommended model: it adapts to the sensor's self-reported
  quality instead of scaling by the (possibly corrupt) reported speed.

Outlier rejection (all models): if the DVL returns fewer than `min_good_beams` good
beams, the solution has no redundancy (a bad beam can produce a wild velocity with a
low FOM), so the update is treated as no-lock. A Mahalanobis gate on the EKF side
(odom0_twist_rejection_threshold in ekf_local/global.yaml) is the independent backstop.

When bottom lock is lost or velocity topic is stale: linear variances set to
no_lock_variance (large m²/s²) so the EKF effectively ignores the update.

Parameters:
    twist_linear_covariance_model: "stationary_tep" | "speed_dependent" | "reported_fom"
    min_good_beams     (int): min good beams for a valid update (default 4; <4 -> no-lock)
    dvl_variant        (string): picks the default speed_sigma_scale (speed_dependent only)
    speed_sigma_scale  (double): σ per unit speed [-]; defaults to the variant's
                                 datasheet accuracy (speed_dependent only)
    no_lock_variance   (double): Variance when no lock [m²/s²] (default: 1e6)
    lock_variance_vx   (double): override stationary vx variance (both models)
    lock_variance_vy   (double): override stationary vy variance
    lock_variance_vz   (double): override stationary vz variance (before floor)
    lock_variance_vz_floor (double): max(raw vz variance, floor); set 0 to disable
    angular_covariance (double): Angular rate variance (unused by DVL)
    velocity_stale_timeout_sec (double): lock flag timeout [s] (default: 0.5)
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

# DVL-A50 long-term accuracy specs from Water Linked, used as the default
# speed_sigma_scale. Replace with a value fitted from moving data when available.
DVL_VARIANTS = {
    "standard": {
        "sigma_scale": 0.0101,   # ±1.01% of measured speed
    },
    "performance": {
        "sigma_scale": 0.001,    # ±0.1% of measured speed
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
        self.declare_parameter("angular_covariance", 1000000.0)
        self.declare_parameter("velocity_stale_timeout_sec", 0.5)
        # Outlier rejection: minimum good beams for a trustworthy 3D solution. With
        # only 3 beams the solve has zero redundancy (3 unknowns, 3 equations, exact
        # fit) so a bad beam yields a wildly wrong velocity with a deceptively low FOM
        # and nothing in the sensor can flag it. Require 4 by default; fewer -> treat
        # as no-lock (no_lock_variance) so the EKF ignores the update.
        self.declare_parameter("min_good_beams", 4)

        self._cov_model = (
            self.get_parameter("twist_linear_covariance_model").value or "stationary_tep"
        ).strip().lower()
        if self._cov_model not in ("stationary_tep", "speed_dependent", "reported_fom"):
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
                f"Falling back to 'performance'."
            )
            variant = "performance"

        self.declare_parameter(
            "speed_sigma_scale", DVL_VARIANTS[variant]["sigma_scale"]
        )
        self.speed_sigma_scale = float(self.get_parameter("speed_sigma_scale").value)
        self.no_lock_var = float(self.get_parameter("no_lock_variance").value)
        self._lock_var_vx = float(self.get_parameter("lock_variance_vx").value)
        self._lock_var_vy = float(self.get_parameter("lock_variance_vy").value)
        self._lock_var_vz_raw = float(self.get_parameter("lock_variance_vz").value)
        self._lock_var_vz_floor = float(
            self.get_parameter("lock_variance_vz_floor").value
        )
        self.ang_cov = self.get_parameter("angular_covariance").value
        self.velocity_stale_timeout_sec = (
            self.get_parameter("velocity_stale_timeout_sec").value
        )
        self.min_good_beams = int(self.get_parameter("min_good_beams").value)

        self.bottom_lock = False
        self.last_velocity_stamp = None
        # Latest beam count + the DVL's own reported velocity (co)variance diagonal
        # (figure-of-merit based; the driver fills Dvl.velocity_covar from the Water
        # Linked 3x3 cov, else diag(fom^2)). Updated in velocity_callback, consumed in
        # odometry_callback. Start pessimistic so updates are ignored until we see data.
        self.num_good_beams = 0
        self._reported_var = (self.no_lock_var, self.no_lock_var, self.no_lock_var)

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
            f"velocity_stale_timeout_sec={self.velocity_stale_timeout_sec}, "
            f"min_good_beams={self.min_good_beams}, "
            f"lock_var_vx_vy_vz=({vxv:g},{vyv:g},{vzv:g})"
            + (
                f", speed_dep: variant={variant}, "
                f"speed_sigma_scale={self.speed_sigma_scale:g}"
                if self._cov_model == "speed_dependent"
                else ""
            )
        )

    def _effective_lock_vz_variance(self) -> float:
        z = self._lock_var_vz_raw
        f = self._lock_var_vz_floor
        if f > 0.0:
            return max(z, f)
        return z

    def _stationary_lock_linear_variances_effective(self) -> tuple[float, float, float]:
        """Vx, Vy, Vz stationary variances [m²/s²] after vz floor."""
        return (
            self._lock_var_vx,
            self._lock_var_vy,
            self._effective_lock_vz_variance(),
        )

    def velocity_callback(self, msg: Dvl):
        # Driver maps DVL velocity_valid -> beam_velocities_valid.
        # Use this as bottom-lock proxy and guard it with staleness timeout.
        self.bottom_lock = msg.beam_velocities_valid
        self.num_good_beams = int(msg.num_good_beams)
        # Dvl.velocity_covar is a row-major 3x3; take the diagonal (vx, vy, vz vars).
        vc = msg.velocity_covar
        if len(vc) >= 9:
            self._reported_var = (float(vc[0]), float(vc[4]), float(vc[8]))
        self.last_velocity_stamp = self.get_clock().now()

    def _lock_is_stale(self) -> bool:
        if self.last_velocity_stamp is None:
            return True

        age = (self.get_clock().now() - self.last_velocity_stamp).nanoseconds / 1e9
        return age > self.velocity_stale_timeout_sec

    def odometry_callback(self, msg: Odometry):
        cov = list(msg.twist.covariance)
        evx, evy, evz = self._stationary_lock_linear_variances_effective()

        if (
            self._lock_is_stale()
            or not self.bottom_lock
            or self.num_good_beams < self.min_good_beams
        ):
            # No lock, stale, or too few beams for a redundant (outlier-checkable)
            # solution -> make the update negligible so the EKF effectively ignores it.
            cov[0] = self.no_lock_var
            cov[7] = self.no_lock_var
            cov[14] = self.no_lock_var
        elif self._cov_model == "reported_fom":
            # Use the DVL's own FOM-based variance, floored at the stationary values so
            # a deceptively small FOM can never make the EKF over-trust the update.
            rvx, rvy, rvz = self._reported_var
            cov[0] = max(rvx, evx)
            cov[7] = max(rvy, evy)
            cov[14] = max(rvz, evz)
        elif self._cov_model == "stationary_tep":
            cov[0] = evx
            cov[7] = evy
            cov[14] = evz
        else:
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            vz = msg.twist.twist.linear.z
            speed = math.sqrt(vx * vx + vy * vy + vz * vz)

            speed_var = (self.speed_sigma_scale * speed) ** 2
            cov[0] = evx + speed_var
            cov[7] = evy + speed_var
            cov[14] = evz + speed_var

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