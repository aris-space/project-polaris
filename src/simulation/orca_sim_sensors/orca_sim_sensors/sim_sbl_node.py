#!/usr/bin/env python3
"""Synthetic Water Linked Underwater GPS G2 (SBL) for the Polaris simulation.

Publishes what uwgpsg2_ros2_interface publishes, so the real uwgpsg2_translator
(to_navsatfix_translator + selector) runs unmodified:

  /waterlinked_ugps/locator_position_global   geographic_msgs/GeoPointStamped
  /waterlinked_ugps/locator_acoustic_quality  geometry_msgs/Vector3Stamped
                                              (x = acoustic std [m], y = 1 if position valid)

Both carry the same stamp (the translator pairs them by stamp). The position is that of the
locator (sbl_link), as on the vehicle.

Accuracy scales with the slant range r from the topside antenna to the locator:

    sigma = sigma_min_m + sigma_per_m * r          (default 1.5 m + 2 % of range)

The horizontal error is white Gaussian with that sigma per axis, and the reported std is
sigma, so the translator's covariance is consistent with the actual error. Depth comes from
the locator's own pressure sensor (small noise). With probability ``invalid_probability``
per update the G2 reports position_valid = false (no acoustic solution).
"""

import random

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry

from orca_sim_sensors.common import default_origin_file, enu_to_llh, load_origin, quat_to_matrix


class SimSblNode(Node):
    def __init__(self):
        super().__init__('sim_sbl_node')
        self.declare_parameter('origin_file', default_origin_file())
        # Topside antenna in the Gazebo world (ENU, m). Default: the mission origin, 1 m deep.
        self.declare_parameter('antenna_enu', [0.0, 0.0, -1.0])
        # base_link -> sbl_link (the locator), from start_waterlinked_interface_bttm_side.launch.py
        self.declare_parameter('locator_offset', [-0.614, -0.000086, 0.197])
        self.declare_parameter('rate_hz', 2.0)  # uwgpsg2_ros2_interface ros_rate
        self.declare_parameter('sigma_min_m', 1.5)
        self.declare_parameter('sigma_per_m', 0.02)
        self.declare_parameter('depth_noise_std_m', 0.05)
        self.declare_parameter('invalid_probability', 0.05)
        self.declare_parameter('seed', -1)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self._origin = load_origin(p('origin_file'))
        self._antenna = np.array(p('antenna_enu'), dtype=float)
        self._locator = np.array(p('locator_offset'), dtype=float)
        self._sigma_min = float(p('sigma_min_m'))
        self._sigma_per_m = float(p('sigma_per_m'))
        self._depth_std = float(p('depth_noise_std_m'))
        self._p_invalid = float(p('invalid_probability'))
        seed = int(p('seed'))
        self._rng = random.Random(None if seed < 0 else seed)

        ns = '/waterlinked_ugps/'
        self._geo_pub = self.create_publisher(GeoPointStamped, ns + 'locator_position_global', 10)
        self._q_pub = self.create_publisher(Vector3Stamped, ns + 'locator_acoustic_quality', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        self._odom = None
        self.create_timer(1.0 / float(p('rate_hz')), self._tick)
        self.get_logger().info(
            f'Synthetic UGPS G2: sigma = {self._sigma_min:.2f} m + {100 * self._sigma_per_m:.1f} % '
            f'of slant range from antenna {self._antenna.tolist()}')

    def _on_odom(self, msg):
        self._odom = msg

    def sigma(self, slant_range_m):
        return self._sigma_min + self._sigma_per_m * slant_range_m

    def _tick(self):
        if self._odom is None:
            return
        pose = self._odom.pose.pose
        p = np.array([pose.position.x, pose.position.y, pose.position.z])
        p = p + quat_to_matrix(pose.orientation) @ self._locator
        sigma = self.sigma(float(np.linalg.norm(p - self._antenna)))
        valid = self._rng.random() >= self._p_invalid

        e = p[0] + self._rng.gauss(0.0, sigma)
        n = p[1] + self._rng.gauss(0.0, sigma)
        u = p[2] + self._rng.gauss(0.0, self._depth_std)
        lat, lon, alt = enu_to_llh(self._origin, e, n, u)

        stamp = self.get_clock().now().to_msg()
        geo = GeoPointStamped()
        geo.header.stamp = stamp
        geo.header.frame_id = 'sbl_link'
        geo.position.latitude, geo.position.longitude, geo.position.altitude = lat, lon, alt
        q = Vector3Stamped()
        q.header = geo.header
        q.vector.x = sigma
        q.vector.y = 1.0 if valid else 0.0
        # Quality first: the translator buffers geo until its quality arrives either way.
        self._q_pub.publish(q)
        self._geo_pub.publish(geo)


def main():
    rclpy.init()
    node = SimSblNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
