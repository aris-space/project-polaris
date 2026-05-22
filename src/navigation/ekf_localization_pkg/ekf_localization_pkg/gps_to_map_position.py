"""
gps_to_map_position — project /gps/validated (NavSatFix) directly to a
true-ENU map-frame Odometry on /odometry/gps_map.

Why this exists
---------------

`navsat_transform_node` with `use_odometry_yaw: true` rotates each GPS
measurement by the boat's current odom-frame yaw before publishing on
`/odometry/gps`. On this stack (where local odom is already true ENU via
`head_mot` calibration), that rotation is a no-op in principle — but
empirically it introduces a ~30° rotation that contaminates downstream
gps_floored, EKF state, and the back-projection. The 7 m mean offset of
/gps/filtered/global vs /fix on grid_02 traces directly to it.

`gnss_anchored_pose` doesn't have this problem because it does the lat/lon
→ UTM → (UTM − datum) projection itself, with no IMU/odom yaw involvement.
The result is genuinely in true ENU, and anchored matches /fix to 0.6 m
mean (vs global EKF's 7.3 m).

This node packages anchored's projection logic as a standalone publisher
so the global EKF can consume a clean true-ENU map-frame GPS measurement
without going through navsat_transform's rotation. Result is the same as
anchored's underlying math but available as an input to the EKF — so we
keep the EKF's sensor fusion (DVL, IMU, pressure between GPS updates,
proper uncertainty propagation, predict step) and gain the projection
correctness.

Output is byte-for-byte the same shape as gps_odom_cov_floor's
`/odometry/gps_floored` (cov-floored, NaN-guarded, off-diagonals zeroed,
frame_id="map") so the EKF can subscribe to either interchangeably.

Datum is passed as parameters by the launch (same path
`gnss_datum_watchdog` already uses to send datum_lat/lon to
`global_ekf_to_navsatfix_node`).

Parameters
----------
input_topic       NavSatFix to consume (default /gps/validated).
output_topic      Odometry topic to publish (default /odometry/gps_map).
output_frame_id   header.frame_id of output (default "map").
datum_lat         Datum latitude (degrees) — anchor of the map frame.
datum_lon         Datum longitude (degrees).
datum_alt         Datum altitude (m) — subtracted from each fix's altitude.
min_pos_cov_m2    Diagonal pose-covariance floor (m²). Default 0.25 to
                  match gps_odom_cov_floor's empirically-tuned value.
"""
from __future__ import annotations

import math

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

from pyproj import Transformer


def _utm_epsg(lat: float, lon: float) -> str:
    zone = int((lon + 180.0) / 6.0) + 1
    hemisphere = "6" if lat >= 0.0 else "7"
    return f"EPSG:32{hemisphere}{zone:02d}"


class GpsToMapPosition(Node):
    def __init__(self) -> None:
        super().__init__("gps_to_map_position")

        self.declare_parameter("input_topic", "/gps/validated")
        self.declare_parameter("output_topic", "/odometry/gps_map")
        self.declare_parameter("output_frame_id", "map")
        self.declare_parameter("datum_lat", 0.0)
        self.declare_parameter("datum_lon", 0.0)
        self.declare_parameter("datum_alt", 0.0)
        self.declare_parameter("min_pos_cov_m2", 0.25)

        in_topic: str = str(self.get_parameter("input_topic").value)
        out_topic: str = str(self.get_parameter("output_topic").value)
        self._frame_id: str = str(self.get_parameter("output_frame_id").value)
        self._floor: float = float(self.get_parameter("min_pos_cov_m2").value)

        # Datum state: starts dormant (zero/null-island = "not yet set").
        # Activated by parameter update from gnss_datum_watchdog when it
        # locks on a valid GNSS fix. _datum_valid gates publishing —
        # without a valid datum, GPS-fix arrivals are received but not
        # republished as map-frame Odometry.
        self._datum_lat: float = float(self.get_parameter("datum_lat").value)
        self._datum_lon: float = float(self.get_parameter("datum_lon").value)
        self._datum_alt: float = float(self.get_parameter("datum_alt").value)
        self._datum_valid: bool = False
        self._to_utm = None
        self._datum_e: float = 0.0
        self._datum_n: float = 0.0
        if self._is_datum_valid(self._datum_lat, self._datum_lon):
            self._activate_datum(self._datum_lat, self._datum_lon, self._datum_alt)

        self._n_in = 0
        self._n_nan_replaced = 0
        self._n_floored = 0

        self._sub = self.create_subscription(
            NavSatFix, in_topic, self._on_fix, qos_profile_sensor_data
        )
        self._pub = self.create_publisher(Odometry, out_topic, 10)

        # Dynamic-datum: watchdog calls /gps_to_map_position/set_parameters
        # after lock to push datum_lat/lon/alt. We re-init the UTM
        # transformer and start publishing.
        self.add_on_set_parameters_callback(self._on_param_change)

        if self._datum_valid:
            self.get_logger().info(
                f"gps_to_map_position: {in_topic} -> {out_topic}, "
                f"datum SET AT LAUNCH ({self._datum_lat:.7f}, "
                f"{self._datum_lon:.7f}, {self._datum_alt:.2f} m), "
                f"frame_id='{self._frame_id}'"
            )
        else:
            self.get_logger().info(
                f"gps_to_map_position: {in_topic} -> {out_topic}, "
                f"DORMANT (waiting for datum_lat/lon to be set via "
                f"set_parameters service from gnss_datum_watchdog), "
                f"frame_id='{self._frame_id}', "
                f"min_pos_cov_m2={self._floor:.4f} ({self._floor ** 0.5:.3f} m 1σ)"
            )

    @staticmethod
    def _is_datum_valid(lat: float, lon: float) -> bool:
        return abs(lat) > 0.1 or abs(lon) > 0.1

    def _activate_datum(self, lat: float, lon: float, alt: float) -> None:
        epsg = _utm_epsg(lat, lon)
        self._to_utm = Transformer.from_crs("EPSG:4326", epsg, always_xy=True)
        self._datum_e, self._datum_n = self._to_utm.transform(lon, lat)
        self._datum_lat = lat
        self._datum_lon = lon
        self._datum_alt = alt
        self._datum_valid = True
        self.get_logger().info(
            f"datum activated: ({lat:.7f}, {lon:.7f}, {alt:.2f} m) -> "
            f"{epsg} E={self._datum_e:.3f} N={self._datum_n:.3f}"
        )

    def _on_param_change(self, params) -> SetParametersResult:
        new_lat = self._datum_lat
        new_lon = self._datum_lon
        new_alt = self._datum_alt
        for p in params:
            if p.name == "datum_lat":
                new_lat = float(p.value)
            elif p.name == "datum_lon":
                new_lon = float(p.value)
            elif p.name == "datum_alt":
                new_alt = float(p.value)
        if self._is_datum_valid(new_lat, new_lon):
            self._activate_datum(new_lat, new_lon, new_alt)
        return SetParametersResult(successful=True)

    def _on_fix(self, msg: NavSatFix) -> None:
        if not self._datum_valid:
            return  # dormant — no datum yet
        if msg.status.status < 0:
            return
        if abs(msg.latitude) < 0.1 and abs(msg.longitude) < 0.1:
            return

        e, n = self._to_utm.transform(msg.longitude, msg.latitude)
        dx = e - self._datum_e
        dy = n - self._datum_n
        dz = float(msg.altitude) - self._datum_alt

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame_id
        out.child_frame_id = "base_link"
        out.pose.pose.position.x = dx
        out.pose.pose.position.y = dy
        out.pose.pose.position.z = dz
        # Orientation: identity. We don't fuse orientation from GPS.
        out.pose.pose.orientation.x = 0.0
        out.pose.pose.orientation.y = 0.0
        out.pose.pose.orientation.z = 0.0
        out.pose.pose.orientation.w = 1.0

        # Build pose.covariance from NavSatFix.position_covariance.
        # NavSatFix conv: row-major 3x3 over (E, N, U). We map to Odometry
        # pose.covariance over (x, y, z, roll, pitch, yaw) — i.e. the
        # position 3x3 fills the (x, y, z) sub-block.
        cov = [0.0] * 36
        nsf_cov = msg.position_covariance
        # Diagonal floor + NaN guard for x, y, z (indices 0, 7, 14)
        for nsf_idx, odom_idx, name in (
            (0, 0, "x"),
            (4, 7, "y"),
            (8, 14, "z"),
        ):
            v = float(nsf_cov[nsf_idx])
            if not math.isfinite(v) or v < self._floor:
                cov[odom_idx] = self._floor
                if not math.isfinite(v):
                    self._n_nan_replaced += 1
                else:
                    self._n_floored += 1
            else:
                cov[odom_idx] = v
        # Roll/pitch/yaw — not fused, but populate with "unobserved" magnitude
        # so robot_localization's eigenvalue checks don't choke.
        for odom_idx in (21, 28, 35):
            cov[odom_idx] = 1.0
        # Off-diagonals stay zero — clean diagonal R for the EKF update.
        out.pose.covariance = cov

        # Twist not used (zeros). Set twist cov diag to "unobserved" too.
        twist_cov = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            twist_cov[i] = 1.0
        out.twist.covariance = twist_cov

        self._n_in += 1
        self._pub.publish(out)

        if self._n_in % 100 == 0:
            self.get_logger().info(
                f"forwarded {self._n_in} fixes  "
                f"(floored: {self._n_floored}, nan_replaced: {self._n_nan_replaced})"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsToMapPosition()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
