"""
Converts /odometry/filtered/global (map frame) to a NavSatFix on /gps/filtered/global.

navsat_transform cannot do this directly because feeding the global EKF back into
navsat_transform creates a circular dependency.  This node applies the same inverse
UTM projection that navsat_transform uses for /gps/filtered, but sourced from the
GPS-corrected global EKF output.

Math: the map frame is a local frame anchored at the datum.  With `use_odometry_yaw`
on navsat_transform, the map X/Y axes are rotated relative to UTM East/North by
the local-EKF yaw at GNSS-lock time.  For global EKF position (x, y, z) in map
frame, the lat/lon of the boat are recovered by:

    utm_dx = x · cos(yaw_lock) - y · sin(yaw_lock)
    utm_dy = x · sin(yaw_lock) + y · cos(yaw_lock)
    UTM_E  = datum_UTM_E + utm_dx
    UTM_N  = datum_UTM_N + utm_dy
    alt    = datum_alt + z

If `map_yaw_offset_rad` is 0, the formula collapses to the naïve `UTM_E = datum_E + x`
that matches an ENU-aligned map frame. Empirically the naïve form was producing
a ~7 m heading-dependent offset against /fix on grid_02 because navsat_transform's
lock-time yaw (with use_odometry_yaw=true and the IMU's gyro-only VRU initialisation)
puts the map frame at a non-zero angle to UTM. Passing the watchdog's captured yaw
through this parameter undoes that rotation cleanly.

Parameters
----------
datum_lat         Datum latitude  (degrees)  — forwarded from gnss_datum_watchdog
datum_lon         Datum longitude (degrees)
datum_alt         Datum altitude  (meters)
map_yaw_offset_rad  Angle of map-X relative to UTM East at GNSS-lock time
                    (radians, CCW positive). Default 0.0 (no rotation; matches the
                    legacy behaviour). Pass the watchdog's `local_anchor_yaw` to
                    correctly back-project a rotated map frame.
global_odom_topic Source odometry topic  (default /odometry/filtered/global)
output_topic      Output NavSatFix topic (default /gps/filtered/global)
"""
from __future__ import annotations

import math

import rclpy
from rcl_interfaces.msg import SetParametersResult
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
        self.declare_parameter("map_yaw_offset_rad", 0.0)
        self.declare_parameter("global_odom_topic", "/odometry/filtered/global")
        self.declare_parameter("output_topic", "/gps/filtered/global")

        map_yaw: float   = float(self.get_parameter("map_yaw_offset_rad").value)
        odom_topic: str  = self.get_parameter("global_odom_topic").value
        out_topic: str   = self.get_parameter("output_topic").value

        # Pre-compute trig terms once at startup. If yaw is zero, the rotation
        # is identity and _on_odom takes the (slightly cheaper) no-rotate path.
        self._map_yaw_offset_rad = map_yaw
        self._map_cos = math.cos(map_yaw)
        self._map_sin = math.sin(map_yaw)
        self._apply_rotation = abs(map_yaw) > 1e-9

        # Datum state: dormant until set via parameter callback by
        # gnss_datum_watchdog at GNSS-lock time.
        self._datum_lat: float = float(self.get_parameter("datum_lat").value)
        self._datum_lon: float = float(self.get_parameter("datum_lon").value)
        self._datum_alt: float = float(self.get_parameter("datum_alt").value)
        self._datum_valid: bool = False
        self._from_utm = None
        self._datum_e: float = 0.0
        self._datum_n: float = 0.0
        if self._is_datum_valid(self._datum_lat, self._datum_lon):
            self._activate_datum(self._datum_lat, self._datum_lon, self._datum_alt)

        self._pub = self.create_publisher(
            NavSatFix, out_topic, QoSProfile(depth=10)
        )
        self._sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos_profile_sensor_data
        )
        self.add_on_set_parameters_callback(self._on_param_change)

        if self._datum_valid:
            self.get_logger().info(
                f"global_ekf_to_navsatfix: datum SET AT LAUNCH, subscribed to "
                f"'{odom_topic}', publishing on '{out_topic}'"
            )
        else:
            self.get_logger().info(
                f"global_ekf_to_navsatfix: DORMANT (waiting for datum_lat/lon "
                f"via set_parameters from watchdog), subscribed to '{odom_topic}', "
                f"output '{out_topic}'"
            )

    @staticmethod
    def _is_datum_valid(lat: float, lon: float) -> bool:
        return abs(lat) > 0.1 or abs(lon) > 0.1

    def _activate_datum(self, lat: float, lon: float, alt: float) -> None:
        epsg = _utm_epsg(lat, lon)
        to_utm = Transformer.from_crs("EPSG:4326", epsg, always_xy=True)
        self._from_utm = Transformer.from_crs(epsg, "EPSG:4326", always_xy=True)
        self._datum_e, self._datum_n = to_utm.transform(lon, lat)
        self._datum_alt = alt
        self._datum_lat = lat
        self._datum_lon = lon
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

    def _on_odom(self, msg: Odometry) -> None:
        if not self._datum_valid:
            return  # dormant — no datum yet
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Rotate from map frame to UTM ENU before adding to datum.
        if self._apply_rotation:
            utm_dx = x * self._map_cos - y * self._map_sin
            utm_dy = x * self._map_sin + y * self._map_cos
        else:
            utm_dx = x
            utm_dy = y

        lon_out, lat_out = self._from_utm.transform(
            self._datum_e + utm_dx, self._datum_n + utm_dy
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
