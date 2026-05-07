"""
Debug NavSatFix marker — publishes a single NavSatFix from runtime parameters
so you can drop a point on Foxglove's Map panel by editing lat/lon/alt in
the Parameters panel.

Useful for marking landmarks, dive targets, the dock location, or just
sanity-checking what the Map panel renders.

Parameters
----------
topic         Output NavSatFix topic       (default /gps/marker)
publish_rate  Hz                            (default 1.0)
lat           Latitude  (degrees)           (default 0.0 — null-island)
lon           Longitude (degrees)           (default 0.0)
alt           Altitude  (meters)            (default 0.0)
frame_id      Header frame_id               (default 'map')

All of lat / lon / alt / frame_id are live-tunable. Edit them in the Foxglove
Parameters panel under /navsatfix_marker (or via 'ros2 param set') and the
next published message uses the new values.

Usage
-----
    ros2 run ekf_localization_pkg navsatfix_marker
    # then in Foxglove or terminal:
    ros2 param set /navsatfix_marker lat 47.328875
    ros2 param set /navsatfix_marker lon  8.572727
    # add a Map panel subscribed to /gps/marker
"""
from __future__ import annotations

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class NavSatFixMarker(Node):

    def __init__(self) -> None:
        super().__init__("navsatfix_marker")

        self.declare_parameter("topic", "/gps/marker")
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("lat", 0.0)
        self.declare_parameter("lon", 0.0)
        self.declare_parameter("alt", 0.0)
        self.declare_parameter("frame_id", "map")

        topic: str = self.get_parameter("topic").value
        publish_rate: float = float(self.get_parameter("publish_rate").value)
        self._lat: float = float(self.get_parameter("lat").value)
        self._lon: float = float(self.get_parameter("lon").value)
        self._alt: float = float(self.get_parameter("alt").value)
        self._frame_id: str = self.get_parameter("frame_id").value

        self._pub = self.create_publisher(NavSatFix, topic, 10)
        period = max(1e-3, 1.0 / publish_rate)
        self._timer = self.create_timer(period, self._publish)

        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f"NavSatFixMarker: topic={topic} rate={publish_rate} Hz "
            f"frame={self._frame_id}  lat={self._lat:.7f} lon={self._lon:.7f} alt={self._alt:.2f}"
        )
        self.get_logger().info(
            "Live-tunable: set lat/lon/alt/frame_id via 'ros2 param set' or "
            "the Foxglove Parameters panel."
        )

    def _publish(self) -> None:
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self._frame_id
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = self._lat
        fix.longitude = self._lon
        fix.altitude = self._alt
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self._pub.publish(fix)

    def _on_param_change(self, params) -> SetParametersResult:
        for p in params:
            try:
                if p.name == "lat":
                    self._lat = float(p.value)
                elif p.name == "lon":
                    self._lon = float(p.value)
                elif p.name == "alt":
                    self._alt = float(p.value)
                elif p.name == "frame_id":
                    self._frame_id = str(p.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason=f"{p.name} must be a number",
                )
        return SetParametersResult(successful=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavSatFixMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
