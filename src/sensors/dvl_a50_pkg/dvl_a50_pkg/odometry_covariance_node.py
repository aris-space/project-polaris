"""
Subscribes to /sensors/dvl/odometry and republishes to /sensors/dvl/odometry_cov
with dynamically computed twist covariance.

DVL-A50 accuracy variants:
  - "standard":    ±1.01% of measured speed  (sigma_scale = 0.0101)
  - "performance": ±0.1%  of measured speed  (sigma_scale = 0.001)

Covariance model:
  - σ_xy = max(σ_min, σ_scale * |v|)   speed-dependent horizontal std dev
  - σ_z  = 2 * σ_xy                    vertical is noisier
  - variance = σ²                       placed on twist covariance diagonal
  - angular covariance = large variance  DVL does not measure angular rate

Quality-based inflation:
  - Subscribes to /sensors/dvl/velocity for the beam_velocities_valid flag
  - When bottom lock is lost: covariance inflated to no_lock_variance (1.0 m²/s²)
    so the EKF effectively ignores the measurement
  - When bottom lock is held: normal speed-dependent covariance
  - If velocity messages are stale for longer than velocity_stale_timeout_sec,
    treat as no lock

Parameters:
    dvl_variant        (string): "standard" or "performance"        (default: "performance")
    no_lock_variance   (double): Variance when bottom lock lost     (default: 1.0)
    angular_covariance (double): Angular rate variance              (default: 1000000.0)
    velocity_stale_timeout_sec (double): lock flag timeout [s]      (default: 0.5)
"""


import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from marine_acoustic_msgs.msg import Dvl

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

        self.declare_parameter("dvl_variant", "performance")
        self.declare_parameter("no_lock_variance", 1.0)
        self.declare_parameter("angular_covariance", 1000000.0)
        self.declare_parameter("velocity_stale_timeout_sec", 0.5)

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
        self.no_lock_var = self.get_parameter("no_lock_variance").value
        self.ang_cov = self.get_parameter("angular_covariance").value
        self.velocity_stale_timeout_sec = (
            self.get_parameter("velocity_stale_timeout_sec").value
        )

        self.bottom_lock = False
        self.last_velocity_stamp = None

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

        self.get_logger().info(
            f"DVL covariance injector: variant={variant}, "
            f"sigma_scale={self.sigma_scale}, sigma_min={self.sigma_min}, "
            f"no_lock_variance={self.no_lock_var}, "
            f"velocity_stale_timeout_sec={self.velocity_stale_timeout_sec}"
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

    def odometry_callback(self, msg: Odometry):
        cov = list(msg.twist.covariance)

        if self._lock_is_stale() or not self.bottom_lock:
            cov[0] = self.no_lock_var
            cov[7] = self.no_lock_var
            cov[14] = self.no_lock_var
        else:
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            vz = msg.twist.twist.linear.z
            speed = math.sqrt(vx * vx + vy * vy + vz * vz)

            sigma_xy = max(self.sigma_min, self.sigma_scale * speed)
            sigma_z = 2.0 * sigma_xy

            cov[0] = sigma_xy * sigma_xy     # vx variance
            cov[7] = sigma_xy * sigma_xy     # vy variance
            cov[14] = sigma_z * sigma_z      # vz variance

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
