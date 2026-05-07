"""
Converts /odometry/filtered/global (map frame) to a NavSatFix on /gps/filtered/global.

navsat_transform cannot do this directly because feeding the global EKF back into
navsat_transform creates a circular dependency.  This node applies the same inverse
UTM projection that navsat_transform uses for /gps/filtered, but sourced from the
GPS-corrected global EKF output.

Math: the map frame is a local ENU frame anchored at the datum.  For global EKF
position (x, y, z) in map frame:
    UTM_E = datum_UTM_E + x
    UTM_N = datum_UTM_N + y
    alt   = datum_alt + z
Inverse UTM projection → lat/lon.

Parameters
----------
datum_lat         Datum latitude  (degrees)  — forwarded from gnss_datum_watchdog
datum_lon         Datum longitude (degrees)
datum_alt         Datum altitude  (meters)
global_odom_topic Source odometry topic  (default /odometry/filtered/global)
output_topic      Output NavSatFix topic (default /gps/filtered/global)
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus

from pyproj import Transformer


def _utm_epsg(lat: float, lon: float) -> str:
    zone = int((lon + 180.0) / 6.0) + 1
    hemisphere = "6" if lat >= 0.0 else "7"
    return f"EPSG:32{hemisphere}{zone:02d}"


class GlobalEkfToNavsatFix(Node):

    def __init__(self) -> None:
        super().__init__("global_ekf_to_navsatfix_node")

        self.declare_parameter("datum_lat", 0.0)
        self.declare_parameter("datum_lon", 0.0)
        self.declare_parameter("datum_alt", 0.0)
        self.declare_parameter("global_odom_topic", "/odometry/filtered/global")
        self.declare_parameter("output_topic", "/gps/filtered/global")

        datum_lat: float = self.get_parameter("datum_lat").value
        datum_lon: float = self.get_parameter("datum_lon").value
        datum_alt: float = self.get_parameter("datum_alt").value
        odom_topic: str  = self.get_parameter("global_odom_topic").value
        out_topic: str   = self.get_parameter("output_topic").value

        if abs(datum_lat) < 0.1 and abs(datum_lon) < 0.1:
            self.get_logger().fatal(
                f"Datum appears to be null-island ({datum_lat}, {datum_lon}) — "
                "check that datum_lat/datum_lon are forwarded from gnss_datum_watchdog."
            )
            raise RuntimeError("Invalid datum: null-island")

        epsg = _utm_epsg(datum_lat, datum_lon)
        to_utm = Transformer.from_crs("EPSG:4326", epsg, always_xy=True)
        self._from_utm = Transformer.from_crs(epsg, "EPSG:4326", always_xy=True)

        self._datum_e, self._datum_n = to_utm.transform(datum_lon, datum_lat)
        self._datum_alt = datum_alt

        self.get_logger().info(
            f"Datum: lat={datum_lat:.7f}°  lon={datum_lon:.7f}°  alt={datum_alt:.1f} m  "
            f"→ {epsg}  E={self._datum_e:.3f}  N={self._datum_n:.3f}"
        )

        self._pub = self.create_publisher(
            NavSatFix, out_topic, QoSProfile(depth=10)
        )
        self._sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"Subscribed to '{odom_topic}', publishing NavSatFix on '{out_topic}'"
        )

    def _on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        lon_out, lat_out = self._from_utm.transform(
            self._datum_e + x, self._datum_n + y
        )

        # Diagonal from 6×6 row-major pose covariance [x, y, z, rx, ry, rz].
        cov = msg.pose.covariance
        cov_e   = max(0.0, float(cov[0]))
        cov_n   = max(0.0, float(cov[7]))
        cov_alt = max(0.0, float(cov[14]))

        fix = NavSatFix()
        fix.header.stamp    = msg.header.stamp
        fix.header.frame_id = "map"
        fix.status.status   = NavSatStatus.STATUS_FIX
        fix.status.service  = NavSatStatus.SERVICE_GPS
        fix.latitude        = lat_out
        fix.longitude       = lon_out
        fix.altitude        = self._datum_alt + z
        fix.position_covariance = [
            cov_e, 0.0,   0.0,
            0.0,   cov_n, 0.0,
            0.0,   0.0,   cov_alt,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self._pub.publish(fix)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GlobalEkfToNavsatFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
