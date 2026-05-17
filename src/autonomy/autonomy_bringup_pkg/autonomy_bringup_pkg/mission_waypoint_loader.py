#!/usr/bin/env python3
"""Load the mission CSV at autonomy bringup and latch the ENU waypoints.

Subscribes to /gps/selected (NavSatFix, lat/lon) and /ubx_nav_hp_pos_llh
(UBXNavHPPosLLH, h_acc) with the exact same RTK gate as gnss_datum_watchdog:

  - NavSatFix.status >= 0  AND  |lat| > 0.1°  AND  |lon| > 0.1°
  - 0 < h_acc ≤ 0.50 m   (UBX field × 1e-4)

Fires on the first NavSatFix that passes the gate while a fresh h_acc reading
also passes. At that moment:

  1. Reads the mission CSV.
  2. Transforms every (lat, lon, alt) to map-frame ENU via process_coordinates,
     using the locked fix as the origin.
  3. Publishes the resulting PoseStamped[] as a PoseArray on
     /mission_waypoints_enu with TRANSIENT_LOCAL durability, so any later
     subscriber (e.g. WGS84_mission_starter when the operator hits "go")
     receives the latched message immediately — even hours later, even after
     the vehicle has dived and lost GNSS.

This solves the "transform-at-mission-start" bug: the transform happens once,
on the surface, at the same instant the EKF locks its datum. The mission
starter no longer needs a GPS subscription of its own.

The CSV path is a ROS parameter ``mission_csv`` (default: the same
goldbach_straightline file used by mission_waypoints_publisher).
"""

from __future__ import annotations

import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import NavSatFix
from ublox_ubx_msgs.msg import UBXNavHPPosLLH

from config_pkg.constants import GpsOriginConditions
from autonomy_bringup_pkg.load_wgs84_points_to_waypoints import process_coordinates


_H_ACC_MAX_M = GpsOriginConditions.GPS_ORIGIN_H_ACC_MAX_M
_H_ACC_TO_M = GpsOriginConditions.GPS_ORIGIN_H_ACC_TO_M
_NULL_ISLAND_E7 = GpsOriginConditions.GPS_ORIGIN_NULL_ISLAND_E7
_NULL_ISLAND_DEG = _NULL_ISLAND_E7 * 1e-7  # 0.1°

# Maximum age of the latest h_acc reading at the moment a NavSatFix passes its
# own checks. Larger than the typical UBX 5 Hz period, smaller than the
# gnss_datum_watchdog launch lag. If h_acc has been stale longer than this we
# wait for the next fresh reading rather than locking on an outdated value.
_H_ACC_STALE_S = 2.0

# Heartbeat log cadence while waiting for the RTK gate. NTRIP cold-start can
# take minutes; the periodic line lets the operator see the node is alive and
# what the current h_acc looks like, instead of staring at a silent terminal.
_WAIT_HEARTBEAT_S = 30.0


class MissionWaypointLoader(Node):

    def __init__(self) -> None:
        super().__init__('mission_waypoint_loader')

        share = get_package_share_directory('autonomy_bringup_pkg')
        default_csv = f'{share}/missions/goldbach_straightline_wgs84_mission.csv'
        self.declare_parameter('mission_csv', default_csv)
        self.declare_parameter('fix_topic', '/gps/selected')
        self.declare_parameter('h_acc_topic', '/ubx_nav_hp_pos_llh')
        self.declare_parameter('output_topic', '/mission_waypoints_enu')

        self._csv_path: str = str(self.get_parameter('mission_csv').value)
        fix_topic: str = str(self.get_parameter('fix_topic').value)
        h_acc_topic: str = str(self.get_parameter('h_acc_topic').value)
        output_topic: str = str(self.get_parameter('output_topic').value)

        self._locked = False
        self._latest_h_acc_m: float | None = None
        self._latest_h_acc_t: float = 0.0
        self._latest_fix: NavSatFix | None = None
        self._wait_started_t: float = time.monotonic()

        cb_group = MutuallyExclusiveCallbackGroup()

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(PoseArray, output_topic, latched_qos)

        self.create_subscription(
            UBXNavHPPosLLH,
            h_acc_topic,
            self._on_h_acc,
            qos_profile_sensor_data,
            callback_group=cb_group,
        )
        self.create_subscription(
            NavSatFix,
            fix_topic,
            self._on_fix,
            qos_profile_sensor_data,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f"Waiting for RTK fix on '{fix_topic}' "
            f"(h_acc ≤ {_H_ACC_MAX_M * 100:.0f} cm via '{h_acc_topic}') "
            f"to transform mission CSV: {self._csv_path}"
        )
        self.create_timer(
            _WAIT_HEARTBEAT_S, self._log_wait_heartbeat, callback_group=cb_group
        )

    def _log_wait_heartbeat(self) -> None:
        if self._locked:
            return
        waited_s = time.monotonic() - self._wait_started_t
        fix = self._latest_fix
        fix_str = (
            f'lat={fix.latitude:.6f}° lon={fix.longitude:.6f}° status={fix.status.status}'
            if fix is not None
            else 'no NavSatFix yet'
        )
        h_acc_str = (
            f'{self._latest_h_acc_m:.2f} m'
            if self._latest_h_acc_m is not None
            else 'no UBX h_acc yet'
        )
        self.get_logger().info(
            f'[waiting {waited_s:.0f}s] fix: {fix_str}; h_acc: {h_acc_str}; '
            f'threshold ≤ {_H_ACC_MAX_M:.2f} m'
        )

    def _on_h_acc(self, msg: UBXNavHPPosLLH) -> None:
        if self._locked:
            return
        if msg.invalid_lon or msg.invalid_lat or msg.invalid_hmsl:
            self._latest_h_acc_m = None
            return
        self._latest_h_acc_m = float(msg.h_acc) * _H_ACC_TO_M
        self._latest_h_acc_t = time.monotonic()

    def _on_fix(self, msg: NavSatFix) -> None:
        if self._locked:
            return
        self._latest_fix = msg
        if msg.status.status < 0:
            return
        if (
            abs(msg.latitude) < _NULL_ISLAND_DEG
            and abs(msg.longitude) < _NULL_ISLAND_DEG
        ):
            return
        if self._latest_h_acc_m is None:
            return
        if time.monotonic() - self._latest_h_acc_t > _H_ACC_STALE_S:
            return
        if not (0.0 < self._latest_h_acc_m <= _H_ACC_MAX_M):
            return

        self._lock_and_publish(msg)

    def _lock_and_publish(self, fix: NavSatFix) -> None:
        self._locked = True
        lat0 = float(fix.latitude)
        lon0 = float(fix.longitude)
        alt0 = float(fix.altitude)
        h_acc_m = self._latest_h_acc_m or float('nan')

        self.get_logger().info(
            f'RTK lock: lat={lat0:.7f}° lon={lon0:.7f}° alt={alt0:.2f} m '
            f'h_acc={h_acc_m:.2f} m — transforming {self._csv_path}'
        )

        try:
            poses = process_coordinates(self._csv_path, lat0, lon0, alt0)
        except (OSError, ValueError) as exc:
            self.get_logger().error(
                f'Mission CSV load failed ({self._csv_path}): {exc}. '
                'Waypoint loader will retry on the next valid fix.'
            )
            self._locked = False
            return

        arr = PoseArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'map'
        arr.poses = [p.pose for p in poses]
        self._pub.publish(arr)
        self.get_logger().info(
            f'Published {len(arr.poses)} ENU waypoints on {self._pub.topic_name} '
            '(latched, TRANSIENT_LOCAL).'
        )


def main() -> None:
    rclpy.init()
    node = MissionWaypointLoader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
