#!/usr/bin/env python3
"""Load the mission CSV when /gnss_datum locks and latch the ENU waypoints.

Subscribes (latched, TRANSIENT_LOCAL) to /gnss_datum, which gnss_datum_watchdog
publishes exactly once when it locks an RTK-quality fix — the same instant the
EKF datum is set and map->odom goes live. This loader never has to gate on
GNSS itself: the watchdog already did all the quality checks, so this node
just uses whichever fix the EKF chose.

On the first /gnss_datum message:

  1. Reads the mission CSV.
  2. Calls process_mission(lat, lon, alt) to transform every WGS84 point
     to map-frame ENU using the datum as origin, and to parse the optional
     per-waypoint metadata columns (up_down / delta_z_m / hold_s).
  3. Publishes the poses as a PoseArray on /mission_waypoints_enu, and the
     metadata as an index-aligned Float32MultiArray on /mission_waypoints_meta
     (both latched, TRANSIENT_LOCAL), so any later subscriber
     (WGS84_mission_starter when the operator runs the mission, or the
     ice-approach waypoint task executor) sees them immediately — even hours
     later, even after the vehicle has dived and lost GNSS.

Because /gnss_datum is latched, this loader works whether it starts BEFORE
or AFTER the watchdog locks: late-joining still delivers the datum.
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from autonomy_bringup_pkg.load_wgs84_points_to_waypoints import (
    MissionConfigError,
    declare_mission_csv,
    process_mission,
)


# Heartbeat log cadence while waiting for /gnss_datum. The watchdog can take
# minutes during NTRIP cold-start; the periodic line tells the operator this
# node is alive and waiting, not hung.
_WAIT_HEARTBEAT_S = 30.0


class MissionWaypointLoader(Node):

    def __init__(self) -> None:
        super().__init__('mission_waypoint_loader')

        # Required, no default: set via params/mission.yaml (see autonomy.launch.py).
        self._csv_path: str = declare_mission_csv(self)
        self.declare_parameter('datum_topic', '/gnss_datum')
        self.declare_parameter('output_topic', '/mission_waypoints_enu')
        self.declare_parameter('meta_output_topic', '/mission_waypoints_meta')
        datum_topic: str = str(self.get_parameter('datum_topic').value)
        output_topic: str = str(self.get_parameter('output_topic').value)
        meta_output_topic: str = str(self.get_parameter('meta_output_topic').value)

        self._locked = False
        self._wait_started_t: float = time.monotonic()

        cb_group = MutuallyExclusiveCallbackGroup()

        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(PoseArray, output_topic, latched_qos)
        self._meta_pub = self.create_publisher(Float32MultiArray, meta_output_topic, latched_qos)

        # Match the watchdog's QoS exactly: TRANSIENT_LOCAL + RELIABLE.
        # BEST_EFFORT or VOLATILE here would make us miss the latched message
        # if we subscribe after the watchdog has already published.
        self.create_subscription(
            NavSatFix,
            datum_topic,
            self._on_datum,
            latched_qos,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f"Waiting for locked datum on '{datum_topic}' "
            f"(published once by gnss_datum_watchdog after RTK lock) "
            f"to transform mission CSV: {self._csv_path}"
        )
        self.create_timer(
            _WAIT_HEARTBEAT_S, self._log_wait_heartbeat, callback_group=cb_group
        )

    def _log_wait_heartbeat(self) -> None:
        if self._locked:
            return
        waited_s = time.monotonic() - self._wait_started_t
        # /gnss_datum is latched (TRANSIENT_LOCAL), so we will receive it
        # as soon as it's published — even if the EKF is launched many
        # minutes from now. Two reasons we might be waiting:
        #   1. ekf_localization has not been launched yet
        #      (start_system defaults to start_ekf:=false; operator must run
        #      `ros2 launch ekf_localization_pkg ekf_localization.launch.py`
        #      separately once in water).
        #   2. ekf_localization IS running but gnss_datum_watchdog has not
        #      locked an RTK fix yet — check the watchdog log for h_acc
        #      and fix-status progress.
        self.get_logger().info(
            f'[waiting {waited_s:.0f}s] no /gnss_datum yet. Either '
            'ekf_localization.launch.py has not been started, or '
            'gnss_datum_watchdog is still waiting for an RTK fix.'
        )

    def _on_datum(self, msg: NavSatFix) -> None:
        if self._locked:
            return
        self._locked = True

        lat0 = float(msg.latitude)
        lon0 = float(msg.longitude)
        alt0 = float(msg.altitude)
        self.get_logger().info(
            f'Datum received: lat={lat0:.7f}° lon={lon0:.7f}° alt={alt0:.2f} m '
            f'— transforming {self._csv_path}'
        )

        try:
            poses, meta_list = process_mission(self._csv_path, lat0, lon0, alt0)
        except (OSError, ValueError) as exc:
            self.get_logger().error(
                f'Mission CSV load failed ({self._csv_path}): {exc}. '
                'No waypoints published; mission cannot start.'
            )
            return

        arr = PoseArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'map'
        arr.poses = [p.pose for p in poses]
        self._pub.publish(arr)

        # Index-aligned with arr.poses: [up_down (0.0/1.0), delta_z_m, hold_s] per waypoint.
        meta_msg = Float32MultiArray()
        meta_msg.layout.dim = [
            MultiArrayDimension(label='waypoint', size=len(meta_list), stride=3)
        ]
        for m in meta_list:
            meta_msg.data.extend([1.0 if m.up_down else 0.0, float(m.delta_z_m), float(m.hold_s)])
        self._meta_pub.publish(meta_msg)

        self.get_logger().info(
            f'Published {len(arr.poses)} ENU waypoints on {self._pub.topic_name} '
            f'and {len(meta_list)} waypoint-meta rows on {self._meta_pub.topic_name} '
            '(both latched, TRANSIENT_LOCAL).'
        )


def main() -> None:
    rclpy.init()
    try:
        node = MissionWaypointLoader()
    except MissionConfigError as exc:
        rclpy.logging.get_logger('mission_waypoint_loader').fatal(str(exc))
        rclpy.shutdown()
        sys.exit(1)
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
