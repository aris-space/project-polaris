#!/usr/bin/env python3
"""Synthetic u-blox X20P GNSS/RTK for the Polaris simulation.

Publishes what the vehicle's u-blox driver publishes, from Gazebo ground truth (/odom):

  /fix                  sensor_msgs/NavSatFix           (selector, imu_yaw_correction)
  /ubx_nav_hp_pos_llh   ublox_ubx_msgs/UBXNavHPPosLLH   (selector, gnss_datum_watchdog,
                                                         ros2_receiver)

Position is the antenna (gnss_link), as on the vehicle. Both topics carry the same stamp,
so the selector can pair h_acc with the fix.

Sky view
  The receiver only sees the sky while base_link is within ``surface_depth_m`` (5 cm) of
  where it sits when the vehicle floats at the surface, ``surfaced_base_link_z`` (world z;
  the water surface is z = 0). The sim model floats with base_link 0.165 m under water,
  which puts the antenna (0.174 m above base_link) just above it. Deeper than that it
  publishes NO_FIX with ``no_fix_h_acc_m`` (huge), like a powered receiver with a wet
  antenna; the selector and watchdog reject it.

Each surfacing (rolled once, when base_link comes within ``surface_depth_m``)
  ice_layer:=false   NO_FIX for ``acquire_fix_s``, float RTK (``float_h_acc_m``), then
                     RTK fixed after U(rtk_delay_min_s, rtk_delay_max_s).
  ice_layer:=true    one of
                       RTK     (p = ice_p_rtk):   as above, but after U(ice_rtk_delay_min_s,
                                                  ice_rtk_delay_max_s)
                       float   (p = ice_p_float): float RTK only, h_acc U(ice_float_h_acc_min_m,
                                                  ice_float_h_acc_max_m); never fixes
                       nothing (the rest):        NO_FIX for the whole surfacing

Startup
  Until /gnss_datum is latched by gnss_datum_watchdog (or ``startup_rtk_max_s`` passes),
  the receiver behaves as surfaced in open water regardless of depth and ice_layer, so
  the datum always locks on an RTK fix.

Errors
  RTK fixed: white, sigma = rtk_h_acc_m horizontal (2x vertical). Float: first-order
  Gauss-Markov (time constant float_error_tau_s) with sigma = the reported h_acc, so the
  error wanders slowly like a real float solution instead of jumping every epoch.
"""

import math
import random

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from ublox_ubx_msgs.msg import UBXNavHPPosLLH

from orca_sim_sensors.common import default_origin_file, enu_to_llh, load_origin, quat_to_matrix

NO_FIX, FLOAT, RTK = 'no_fix', 'float', 'rtk'
OUTCOME_RTK, OUTCOME_FLOAT, OUTCOME_NONE = 'rtk', 'float', 'nothing'


class SimGnssNode(Node):
    def __init__(self):
        super().__init__('sim_gnss_node')
        self.declare_parameter('origin_file', default_origin_file())
        self.declare_parameter('fix_topic', '/fix')
        self.declare_parameter('h_acc_topic', '/ubx_nav_hp_pos_llh')
        self.declare_parameter('datum_topic', '/gnss_datum')
        self.declare_parameter('frame_id', 'gnss_link')
        # base_link -> gnss_link, from gnss_bringup_pkg/launch/launch_gnss_x20p.launch.py
        self.declare_parameter('antenna_offset', [-0.0057, -0.00024, 0.174])
        self.declare_parameter('rate_hz', 2.0)  # CFG_RATE_MEAS 500 ms on the vehicle
        self.declare_parameter('available', True)
        self.declare_parameter('seed', -1)

        self.declare_parameter('surface_depth_m', 0.05)
        self.declare_parameter('surfaced_base_link_z', -0.165)  # measured: floating at rest
        self.declare_parameter('ice_layer', False)
        self.declare_parameter('acquire_fix_s', 1.0)
        self.declare_parameter('rtk_h_acc_m', 0.02)
        self.declare_parameter('float_h_acc_m', 1.0)
        self.declare_parameter('float_error_tau_s', 30.0)
        self.declare_parameter('no_fix_h_acc_m', 10000.0)
        self.declare_parameter('rtk_delay_min_s', 2.0)
        self.declare_parameter('rtk_delay_max_s', 5.0)
        self.declare_parameter('ice_p_rtk', 0.5)
        self.declare_parameter('ice_p_float', 0.3)
        self.declare_parameter('ice_rtk_delay_min_s', 5.0)
        self.declare_parameter('ice_rtk_delay_max_s', 20.0)
        self.declare_parameter('ice_float_h_acc_min_m', 0.6)
        self.declare_parameter('ice_float_h_acc_max_m', 2.0)
        self.declare_parameter('startup_rtk_until_datum', True)
        self.declare_parameter('startup_rtk_max_s', 300.0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self._origin = load_origin(p('origin_file'))
        self._frame_id = p('frame_id')
        self._antenna = np.array(p('antenna_offset'), dtype=float)
        self._dt = 1.0 / float(p('rate_hz'))
        self._available = bool(p('available'))
        seed = int(p('seed'))
        self._rng = random.Random(None if seed < 0 else seed)

        self._surface_depth = float(p('surface_depth_m'))
        self._surfaced_z = float(p('surfaced_base_link_z'))
        self._ice = bool(p('ice_layer'))
        self._acquire_s = float(p('acquire_fix_s'))
        self._rtk_h = float(p('rtk_h_acc_m'))
        self._float_h = float(p('float_h_acc_m'))
        self._tau = float(p('float_error_tau_s'))
        self._no_fix_h = float(p('no_fix_h_acc_m'))
        self._rtk_delay = (float(p('rtk_delay_min_s')), float(p('rtk_delay_max_s')))
        self._ice_p_rtk = float(p('ice_p_rtk'))
        self._ice_p_float = float(p('ice_p_float'))
        self._ice_rtk_delay = (float(p('ice_rtk_delay_min_s')), float(p('ice_rtk_delay_max_s')))
        self._ice_float_h = (float(p('ice_float_h_acc_min_m')), float(p('ice_float_h_acc_max_m')))
        self._startup = bool(p('startup_rtk_until_datum'))
        self._startup_max_s = float(p('startup_rtk_max_s'))

        self._fix_pub = self.create_publisher(NavSatFix, p('fix_topic'), 10)
        self._hp_pub = self.create_publisher(UBXNavHPPosLLH, p('h_acc_topic'), 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(NavSatFix, p('datum_topic'), self._on_datum, latched)

        self._odom = None
        self._t0 = None
        self._surfaced = False
        self._t_surface = None
        self._outcome = None
        self._rtk_delay_s = 0.0
        self._outcome_h = 0.0
        self._float_err = np.zeros(3)
        self._last_llh = None
        self._mode = None
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'Synthetic GNSS: ice_layer={self._ice}, sky view while base_link is within '
            f'{self._surface_depth:.2f} m of its floating depth (z={self._surfaced_z:.3f} m), '
            + (f'RTK guaranteed at startup until {p("datum_topic")} '
               f'(max {self._startup_max_s:.0f} s), ' if self._startup else 'no startup RTK hold, ')
            + f'available={self._available}')

    def _on_odom(self, msg):
        self._odom = msg

    def _on_datum(self, _msg):
        if self._startup:
            self._startup = False
            self.get_logger().info(
                'Datum latched: startup RTK guarantee ends, sky-view rules apply.')

    def _roll_outcome(self, open_water):
        if open_water:
            return OUTCOME_RTK, self._rng.uniform(*self._rtk_delay), 0.0
        r = self._rng.random()
        if r < self._ice_p_rtk:
            return OUTCOME_RTK, self._rng.uniform(*self._ice_rtk_delay), 0.0
        if r < self._ice_p_rtk + self._ice_p_float:
            return OUTCOME_FLOAT, 0.0, self._rng.uniform(*self._ice_float_h)
        return OUTCOME_NONE, 0.0, 0.0

    def _solution(self, now_s, depth):
        # depth: how far base_link is below its floating-at-the-surface position [m]
        """Return (mode, h_acc_m) for this epoch and advance the surfacing state machine."""
        if self._startup and now_s - self._t0 > self._startup_max_s:
            self._startup = False
            self.get_logger().warn(
                f'No datum after {self._startup_max_s:.0f} s: ending the startup RTK guarantee.')
        surfaced = self._startup or depth <= self._surface_depth
        if surfaced and not self._surfaced:
            self._t_surface = now_s
            self._outcome, self._rtk_delay_s, self._outcome_h = self._roll_outcome(
                open_water=self._startup or not self._ice)
            self._float_err[:] = 0.0
            extra = (f', RTK after {self._rtk_delay_s:.1f} s' if self._outcome == OUTCOME_RTK
                     else f', h_acc {self._outcome_h:.2f} m' if self._outcome == OUTCOME_FLOAT
                     else '')
            self.get_logger().info(
                f'Surfaced{" (startup)" if self._startup else ""}: outcome {self._outcome}{extra}')
        elif not surfaced and self._surfaced:
            self.get_logger().info(
                f'Submerged ({depth:.2f} m below floating depth): sky view lost.')
        self._surfaced = surfaced

        if not surfaced:
            return NO_FIX, self._no_fix_h
        dt = now_s - self._t_surface
        if dt < self._acquire_s or self._outcome == OUTCOME_NONE:
            return NO_FIX, self._no_fix_h
        if self._outcome == OUTCOME_FLOAT:
            return FLOAT, self._outcome_h
        if dt < self._acquire_s + self._rtk_delay_s:
            return FLOAT, self._float_h
        return RTK, self._rtk_h

    def _tick(self):
        if not self._available or self._odom is None:
            return
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = now_s

        pose = self._odom.pose.pose
        p = np.array([pose.position.x, pose.position.y, pose.position.z])
        mode, h_acc = self._solution(now_s, depth=self._surfaced_z - p[2])
        if mode != self._mode:
            self.get_logger().info(f'GNSS solution: {mode} (h_acc {h_acc:.2f} m)')
            self._mode = mode

        antenna = p + quat_to_matrix(pose.orientation) @ self._antenna
        if mode == RTK:
            err = np.array([self._rng.gauss(0, h_acc), self._rng.gauss(0, h_acc),
                            self._rng.gauss(0, 2 * h_acc)])
        elif mode == FLOAT:
            a = math.exp(-self._dt / self._tau)
            sig = np.array([h_acc, h_acc, 2 * h_acc])
            noise = np.array([self._rng.gauss(0, 1) for _ in range(3)])
            self._float_err = a * self._float_err + math.sqrt(1 - a * a) * sig * noise
            err = self._float_err
        else:
            err = None

        if err is not None:
            llh = enu_to_llh(self._origin, *(antenna + err))
            self._last_llh = llh
        else:
            # A receiver without a fix reports its last solution (zeros before the first one).
            llh = self._last_llh or (0.0, 0.0, 0.0)
        self._publish(now.to_msg(), mode, h_acc, llh)

    def _publish(self, stamp, mode, h_acc, llh):
        lat, lon, alt = llh
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = self._frame_id
        # Same mapping as ublox_nav_sat_fix_hp_node: carrier solution (float or fixed) -> GBAS.
        fix.status.status = (NavSatStatus.STATUS_NO_FIX if mode == NO_FIX
                             else NavSatStatus.STATUS_GBAS_FIX)
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude, fix.altitude = lat, lon, alt
        var = h_acc ** 2
        fix.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, 4.0 * var]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._fix_pub.publish(fix)

        hp = UBXNavHPPosLLH()
        hp.header.stamp = stamp
        hp.header.frame_id = self._frame_id
        invalid = mode == NO_FIX
        hp.invalid_lon = hp.invalid_lat = hp.invalid_height = hp.invalid_hmsl = invalid
        hp.invalid_lon_hp = hp.invalid_lat_hp = invalid
        hp.invalid_height_hp = hp.invalid_hmsl_hp = invalid
        lat_e9, lon_e9 = round(lat * 1e9), round(lon * 1e9)
        hp.lat, hp.lat_hp = int(lat_e9 // 100), int(lat_e9 % 100)
        hp.lon, hp.lon_hp = int(lon_e9 // 100), int(lon_e9 % 100)
        alt_01mm = round(alt * 1e4)
        hp.height, hp.height_hp = int(alt_01mm // 10), int(alt_01mm % 10)
        hp.hmsl, hp.hmsl_hp = hp.height, hp.height_hp
        hp.h_acc = min(int(round(h_acc * 1e4)), 2**32 - 1)        # 0.1 mm units
        hp.v_acc = min(int(round(2.0 * h_acc * 1e4)), 2**32 - 1)
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
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
