#!/usr/bin/env python3
"""EKF-vs-ground-truth error for the Polaris simulation (ground_truth:=False).

Compares the real EKFs against Gazebo ground truth (/odom, world ENU) and publishes, for
``local`` (/odometry/filtered/local, odom frame) and ``global`` (/odometry/filtered/global,
map frame):

  /sim/ekf_error/<name>/position     geometry_msgs/Vector3Stamped   estimate - truth [m], ENU
  /sim/ekf_error/<name>/horizontal   std_msgs/Float64               |dx, dy| [m]
  /sim/ekf_error/<name>/yaw_deg      std_msgs/Float64               estimate - truth, wrapped [deg]

Frame alignment (both EKF frames are ENU-aligned, the IMU heading is true-ENU):
  local   the odom frame starts at the vehicle's pose when ekf_local publishes its first
          estimate, so x/y are compared relative to the truth at that moment; z is absolute
          (pressure-referenced, surface = 0).
  global  the map frame origin is the datum latched on /gnss_datum; it is converted to world
          ENU with the same flat-earth model as the synthetic sensors. Nothing is published
          for ``global`` before the datum exists.

Every ``summary_period_s`` an RMS summary of the horizontal error is logged.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from orca_sim_sensors.common import (
    default_origin_file,
    llh_to_enu,
    load_origin,
    wrap_pi,
    yaw_from_quat,
)


class _Channel:
    def __init__(self, node, name):
        base = f'/sim/ekf_error/{name}/'
        self.name = name
        self.pos_pub = node.create_publisher(Vector3Stamped, base + 'position', 10)
        self.h_pub = node.create_publisher(Float64, base + 'horizontal', 10)
        self.yaw_pub = node.create_publisher(Float64, base + 'yaw_deg', 10)
        self.offset = None  # world ENU of the EKF frame origin
        self.sq_sum, self.n, self.max_h = 0.0, 0, 0.0


class EkfTruthErrorNode(Node):
    def __init__(self):
        super().__init__('ekf_truth_error')
        self.declare_parameter('origin_file', default_origin_file())
        self.declare_parameter('local_topic', '/odometry/filtered/local')
        self.declare_parameter('global_topic', '/odometry/filtered/global')
        self.declare_parameter('datum_topic', '/gnss_datum')
        self.declare_parameter('summary_period_s', 30.0)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self._origin = load_origin(p('origin_file'))

        self._truth = None
        self._local = _Channel(self, 'local')
        self._global = _Channel(self, 'global')
        self.create_subscription(Odometry, '/odom', self._on_truth, qos_profile_sensor_data)
        self.create_subscription(Odometry, p('local_topic'),
                                 lambda m: self._on_estimate(self._local, m), 10)
        self.create_subscription(Odometry, p('global_topic'),
                                 lambda m: self._on_estimate(self._global, m), 10)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(NavSatFix, p('datum_topic'), self._on_datum, latched)
        self.create_timer(float(p('summary_period_s')), self._summary)

    def _on_truth(self, msg):
        self._truth = msg.pose.pose

    def _on_datum(self, msg):
        e, n, _ = llh_to_enu(self._origin, msg.latitude, msg.longitude, msg.altitude)
        self._global.offset = (e, n)
        self.get_logger().info(f'Datum at world ENU ({e:.2f}, {n:.2f}) m: global error enabled.')

    def _on_estimate(self, ch, msg):
        if self._truth is None:
            return
        t = self._truth
        if ch.offset is None:
            if ch is self._global:
                return
            # ekf_local's odom frame starts where the vehicle is when it first publishes.
            ch.offset = (t.position.x - msg.pose.pose.position.x,
                         t.position.y - msg.pose.pose.position.y)
            self.get_logger().info(
                f'Local odom frame origin at world ENU ({ch.offset[0]:.2f}, {ch.offset[1]:.2f}) m.')
        est = msg.pose.pose
        dx = est.position.x + ch.offset[0] - t.position.x
        dy = est.position.y + ch.offset[1] - t.position.y
        dz = est.position.z - t.position.z
        h = math.hypot(dx, dy)
        dyaw = math.degrees(wrap_pi(yaw_from_quat(est.orientation) - yaw_from_quat(t.orientation)))

        v = Vector3Stamped()
        v.header.stamp = msg.header.stamp
        v.header.frame_id = msg.header.frame_id
        v.vector.x, v.vector.y, v.vector.z = dx, dy, dz
        ch.pos_pub.publish(v)
        ch.h_pub.publish(Float64(data=h))
        ch.yaw_pub.publish(Float64(data=dyaw))
        ch.sq_sum += h * h
        ch.n += 1
        ch.max_h = max(ch.max_h, h)

    def _summary(self):
        for ch in (self._local, self._global):
            if ch.n:
                self.get_logger().info(
                    f'{ch.name} EKF horizontal error over the last period: '
                    f'RMS {math.sqrt(ch.sq_sum / ch.n):.2f} m, max {ch.max_h:.2f} m (n={ch.n})')
                ch.sq_sum, ch.n, ch.max_h = 0.0, 0, 0.0


def main():
    rclpy.init()
    node = EkfTruthErrorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
