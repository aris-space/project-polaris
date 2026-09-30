#!/usr/bin/env python3
"""Synthetic GNSS/RTK for the Polaris simulation (docs/SIM_MERGE_PLAN.md T3.3).

Derives a fix from Gazebo ground truth (/odom, ENU metres from the spawn point) against the
one mission origin (autonomy_bringup_pkg/missions/default_mission_origin.json, which is also
ArduSub's --home), and publishes what the vehicle's u-blox driver publishes:

  /fix                  sensor_msgs/NavSatFix           (gnss_datum_watchdog)
  /ubx_nav_hp_pos_llh   ublox_ubx_msgs/UBXNavHPPosLLH   (gnss_datum_watchdog, ros2_receiver)

so the real RTK gates run unmodified: ros2_receiver.gps_origin_cb sets the Pixhawk EKF origin
(GPS_GLOBAL_ORIGIN), and gnss_datum_watchdog latches /gnss_datum.

Until ``rtk_lock_delay_s`` has elapsed, h_acc is reported as ``h_acc_float_m`` (outside the
0.50 m gate), so the gates' waiting path is exercised instead of skipped. ``available:=false``
publishes nothing, to test the no-datum path.

Flat-earth conversion; fine over the few hundred metres a sim mission covers.
"""

import json
import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from ublox_ubx_msgs.msg import UBXNavHPPosLLH

_EARTH_RADIUS_M = 6378137.0


class SimGnssNode(Node):
    def __init__(self):
        super().__init__('sim_gnss_node')
        default_origin = os.path.join(
            get_package_share_directory('autonomy_bringup_pkg'),
            'missions', 'default_mission_origin.json')
        self.declare_parameter('origin_file', default_origin)
        self.declare_parameter('fix_topic', '/fix')
        self.declare_parameter('h_acc_topic', '/ubx_nav_hp_pos_llh')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('h_acc_m', 0.05)
        self.declare_parameter('h_acc_float_m', 2.0)
        self.declare_parameter('rtk_lock_delay_s', 10.0)
        self.declare_parameter('available', True)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        with open(p('origin_file'), encoding='utf-8') as f:
            o = json.load(f)
        self._lat0, self._lon0, self._alt0 = float(o['lat']), float(o['lon']), float(o['alt'])
        self._h_acc_m = float(p('h_acc_m'))
        self._h_acc_float_m = float(p('h_acc_float_m'))
        self._lock_delay_s = float(p('rtk_lock_delay_s'))
        self._available = bool(p('available'))

        self._fix_pub = self.create_publisher(NavSatFix, p('fix_topic'), 10)
        self._hp_pub = self.create_publisher(UBXNavHPPosLLH, p('h_acc_topic'), 10)
        self._odom = None
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        self._t_start = None
        self.create_timer(1.0 / float(p('rate_hz')), self._tick)
        self.get_logger().info(
            f'Synthetic GNSS around origin {self._lat0:.7f},{self._lon0:.7f},{self._alt0:.1f} '
            f'(RTK after {self._lock_delay_s:.0f} s, available={self._available})')

    def _on_odom(self, msg):
        self._odom = msg

    def _tick(self):
        if not self._available or self._odom is None:
            return
        now = self.get_clock().now()
        if self._t_start is None:
            self._t_start = now
        locked = (now - self._t_start).nanoseconds * 1e-9 >= self._lock_delay_s
        h_acc = self._h_acc_m if locked else self._h_acc_float_m

        pos = self._odom.pose.pose.position  # ENU metres from the spawn point (= origin)
        lat = self._lat0 + math.degrees(pos.y / _EARTH_RADIUS_M)
        lon = self._lon0 + math.degrees(
            pos.x / (_EARTH_RADIUS_M * math.cos(math.radians(self._lat0))))
        alt = self._alt0 + pos.z
        stamp = now.to_msg()

        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = 'gnss_link'
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX if locked else NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude, fix.altitude = lat, lon, alt
        var = h_acc ** 2
        fix.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, 4.0 * var]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._fix_pub.publish(fix)

        hp = UBXNavHPPosLLH()
        hp.header.stamp = stamp
        hp.header.frame_id = 'gnss_link'
        lat_e9, lon_e9 = round(lat * 1e9), round(lon * 1e9)
        hp.lat, hp.lat_hp = int(lat_e9 // 100), int(lat_e9 % 100)
        hp.lon, hp.lon_hp = int(lon_e9 // 100), int(lon_e9 % 100)
        alt_01mm = round(alt * 1e4)
        hp.height, hp.height_hp = int(alt_01mm // 10), int(alt_01mm % 10)
        hp.hmsl, hp.hmsl_hp = hp.height, hp.height_hp
        hp.h_acc = int(round(h_acc * 1e4))       # 0.1 mm units
        hp.v_acc = int(round(2.0 * h_acc * 1e4))
        self._hp_pub.publish(hp)


def main():
    rclpy.init()
    node = SimGnssNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
