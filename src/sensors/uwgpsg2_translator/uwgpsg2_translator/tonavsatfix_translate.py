"""
GeoPointStamped -> NavSatFix; merges acoustic std / position_valid from
/waterlinked_ugps/locator_acoustic_quality (Vector3Stamped, same API stamp).
"""

from __future__ import annotations

from collections import OrderedDict

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Vector3Stamped
from geographic_msgs.msg import GeoPointStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 10**9 + int(stamp.nanosec)


class ToNavSatFixTranslator(Node):
    def __init__(self):
        super().__init__("to_navsatfix_translator")
        self.declare_parameter("acoustic_stamp_match_max_ns", 25_000_000)
        self._stamp_slop_ns = (
            self.get_parameter("acoustic_stamp_match_max_ns")
            .get_parameter_value()
            .integer_value
        )

        self._quality_by_stamp: OrderedDict[int, tuple[float, bool]] = OrderedDict()
        self._quality_cache_max = 128
        # Geo may arrive before quality (DDS does not order across topics). Buffer until paired.
        self._pending_geo_by_stamp: OrderedDict[int, GeoPointStamped] = OrderedDict()
        self._pending_geo_max = 128

        self.sub_geo = self.create_subscription(
            GeoPointStamped,
            "/waterlinked_ugps/locator_position_global",
            self.on_geopointstamped,
            10,
        )
        self.sub_quality = self.create_subscription(
            Vector3Stamped,
            "/waterlinked_ugps/locator_acoustic_quality",
            self.on_acoustic_quality,
            10,
        )
        self.pub = self.create_publisher(
            NavSatFix,
            "/waterlinked_ugps/navsatfix",
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "ToNavSatFixTranslator started (acoustic quality fusion enabled)"
        )

    def _pop_pending_geo_for_quality_stamp(self, k: int) -> GeoPointStamped | None:
        """Match pending geo to this quality stamp (exact or within slop)."""
        if k in self._pending_geo_by_stamp:
            return self._pending_geo_by_stamp.pop(k)
        best_pg: int | None = None
        best_dt = self._stamp_slop_ns + 1
        for pk in self._pending_geo_by_stamp:
            dt = abs(k - pk)
            if dt < best_dt:
                best_dt = dt
                best_pg = pk
        if best_pg is not None and best_dt <= self._stamp_slop_ns:
            return self._pending_geo_by_stamp.pop(best_pg)
        return None

    def on_acoustic_quality(self, msg: Vector3Stamped) -> None:
        k = _stamp_ns(msg.header.stamp)
        std_m = float(msg.vector.x)
        valid = bool(msg.vector.y >= 0.5)

        geo = self._pop_pending_geo_for_quality_stamp(k)
        if geo is not None:
            self._publish_navsat_from_geo(geo, std_m, valid)
            return

        self._quality_by_stamp[k] = (std_m, valid)
        while len(self._quality_by_stamp) > self._quality_cache_max:
            self._quality_by_stamp.popitem(last=False)

    def _pop_quality_for_geo(self, header) -> tuple[float, bool]:
        k = _stamp_ns(header.stamp)
        if k in self._quality_by_stamp:
            return self._quality_by_stamp.pop(k)

        best_tq: int | None = None
        best_dt = self._stamp_slop_ns + 1
        for tq in self._quality_by_stamp:
            dt = abs(k - tq)
            if dt < best_dt:
                best_dt = dt
                best_tq = tq
        if best_tq is not None and best_dt <= self._stamp_slop_ns:
            return self._quality_by_stamp.pop(best_tq)
        return None

    def _flush_oldest_pending_geo_as_unknown(self) -> None:
        """Drop oldest pending geo with legacy UNKNOWN NavSatFix (no matching quality yet)."""
        _, geo = self._pending_geo_by_stamp.popitem(last=False)
        self._publish_navsat_from_geo(
            geo, std_m=-1.0, position_valid=False, force_unknown=True
        )

    def _publish_navsat_from_geo(
        self,
        msg: GeoPointStamped,
        std_m: float,
        position_valid: bool,
        *,
        force_unknown: bool = False,
    ) -> None:
        out = NavSatFix()
        out.header = msg.header
        out.header.frame_id = "sbl_link"
        out.latitude = msg.position.latitude
        out.longitude = msg.position.longitude
        out.altitude = msg.position.altitude

        if force_unknown:
            out.status.status = NavSatStatus.STATUS_FIX
            out.status.service = NavSatStatus.SERVICE_GPS
            out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.pub.publish(out)
            return

        if not position_valid:
            out.status.status = NavSatStatus.STATUS_NO_FIX
            out.status.service = NavSatStatus.SERVICE_GPS
            out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.pub.publish(out)
            return

        if std_m <= 0.0:
            out.status.status = NavSatStatus.STATUS_FIX
            out.status.service = NavSatStatus.SERVICE_GPS
            out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            self.pub.publish(out)
            return

        var_h = std_m * std_m
        var_z = max((2.0 * std_m) ** 2, 1.0)
        out.position_covariance[0] = var_h
        out.position_covariance[4] = var_h
        out.position_covariance[8] = var_z
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        out.status.status = NavSatStatus.STATUS_FIX
        out.status.service = NavSatStatus.SERVICE_GPS
        self.pub.publish(out)

    def on_geopointstamped(self, msg: GeoPointStamped):
        meta = self._pop_quality_for_geo(msg.header)
        if meta is not None:
            std_m, position_valid = meta
            self._publish_navsat_from_geo(msg, std_m, position_valid)
            return

        k = _stamp_ns(msg.header.stamp)
        self._pending_geo_by_stamp[k] = msg
        while len(self._pending_geo_by_stamp) > self._pending_geo_max:
            self._flush_oldest_pending_geo_as_unknown()


def main(args=None):
    rclpy.init(args=args)
    node = ToNavSatFixTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
