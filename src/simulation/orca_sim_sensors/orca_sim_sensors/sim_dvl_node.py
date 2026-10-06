#!/usr/bin/env python3
"""Synthetic Water Linked DVL-A50 for the Polaris simulation.

Publishes what the dvl_a50 driver publishes for a velocity report, from Gazebo ground truth:

  /sensors/dvl/velocity   marine_acoustic_msgs/Dvl   (beam_velocities_valid = bottom lock)
  /sensors/dvl/odometry   nav_msgs/Odometry          (twist only, header/child = dvl_a50_link)

The real dvl_a50_pkg odometry_covariance_node turns these into /sensors/dvl/odometry_cov
for the EKFs, exactly as on the vehicle. Dead-reckoning reports are not simulated (the
EKFs fuse only the DVL twist).

Velocity is that of the DVL transducer, v_base + w x r, expressed in dvl_a50_link (mounted
roll = pi, yaw = -pi/4, as launch_dvl.launch.py's static TF), plus white noise with the
vehicle's measured lock variances (dvl_a50_pkg/measurement_noise_constants.py).

Faults
  outliers   With probability ``outlier_probability`` per report, a random error of
             U(outlier_min_mps, outlier_max_mps) in a random direction is added while
             the report still claims bottom lock. This is the case the EKF has to survive.
  dropouts   Start at random (Poisson, ``dropout_rate_per_min``), last
             U(dropout_min_s, dropout_max_s). A fraction ``dropout_silent_fraction`` are
             silent (no messages at all, like a TCP stall); the rest publish
             beam_velocities_valid = false (lost bottom lock).
  range      Lock is also lost when the altitude above ``seafloor_z`` leaves
             [min_altitude_m, max_altitude_m] (A50: 0.05-50 m).
"""

import math
import random

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Vector3
from marine_acoustic_msgs.msg import Dvl
from nav_msgs.msg import Odometry

from dvl_a50_pkg.measurement_noise_constants import (
    DVL_LOCK_LINEAR_VARIANCE_X_M2_S2,
    DVL_LOCK_LINEAR_VARIANCE_Y_M2_S2,
    DVL_LOCK_LINEAR_VARIANCE_Z_RAW_M2_S2,
)
from orca_sim_sensors.common import quat_to_matrix, rpy_to_matrix

# Beam geometry exactly as the dvl_a50 driver fills it.
_BEAM_UNIT_VECS = [
    (-0.6532814824381883, 0.6532814824381883, 0.38268343236508984),
    (-0.6532814824381883, -0.6532814824381883, 0.38268343236508984),
    (0.6532814824381883, -0.6532814824381883, 0.38268343236508984),
    (0.6532814824381883, 0.6532814824381883, 0.38268343236508984),
]
_BEAM_TILT_RAD = math.radians(22.5)


class SimDvlNode(Node):
    def __init__(self):
        super().__init__('sim_dvl_node')
        self.declare_parameter('frame_id', 'dvl_a50_link')
        # base_link -> dvl_a50_link, from dvl_a50_pkg/launch/launch_dvl.launch.py
        self.declare_parameter('mount_xyz', [0.7216, -0.000243, -0.075])
        self.declare_parameter('mount_rpy', [math.pi, 0.0, -math.pi / 4])
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('seed', -1)
        self.declare_parameter('noise_std_mps', [
            math.sqrt(DVL_LOCK_LINEAR_VARIANCE_X_M2_S2),
            math.sqrt(DVL_LOCK_LINEAR_VARIANCE_Y_M2_S2),
            math.sqrt(DVL_LOCK_LINEAR_VARIANCE_Z_RAW_M2_S2),
        ])
        self.declare_parameter('outlier_probability', 0.01)
        self.declare_parameter('outlier_min_mps', 0.2)
        self.declare_parameter('outlier_max_mps', 1.0)
        self.declare_parameter('dropout_rate_per_min', 1.0)
        self.declare_parameter('dropout_min_s', 0.5)
        self.declare_parameter('dropout_max_s', 4.0)
        self.declare_parameter('dropout_silent_fraction', 0.3)
        self.declare_parameter('seafloor_z', -10.0)  # sand.world heightmap pose
        self.declare_parameter('min_altitude_m', 0.05)
        self.declare_parameter('max_altitude_m', 50.0)
        self.declare_parameter('sound_speed', 1500.0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self._frame_id = p('frame_id')
        self._r = np.array(p('mount_xyz'), dtype=float)
        self._R_mount = rpy_to_matrix(*p('mount_rpy'))  # dvl -> base
        self._dt = 1.0 / float(p('rate_hz'))
        seed = int(p('seed'))
        self._rng = random.Random(None if seed < 0 else seed)
        self._noise = np.array(p('noise_std_mps'), dtype=float)
        self._p_outlier = float(p('outlier_probability'))
        self._outlier = (float(p('outlier_min_mps')), float(p('outlier_max_mps')))
        self._dropout_rate_hz = float(p('dropout_rate_per_min')) / 60.0
        self._dropout_len = (float(p('dropout_min_s')), float(p('dropout_max_s')))
        self._silent_fraction = float(p('dropout_silent_fraction'))
        self._seafloor_z = float(p('seafloor_z'))
        self._alt_range = (float(p('min_altitude_m')), float(p('max_altitude_m')))
        self._sound_speed = float(p('sound_speed'))

        self._vel_pub = self.create_publisher(Dvl, '/sensors/dvl/velocity', 10)
        self._odom_pub = self.create_publisher(Odometry, '/sensors/dvl/odometry', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        self._odom = None
        self._dropout_until = None
        self._dropout_silent = False
        self._lock_state = None
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'Synthetic DVL-A50 at {1.0 / self._dt:.0f} Hz: outliers p={self._p_outlier}, '
            f'dropouts {60.0 * self._dropout_rate_hz:.1f}/min')

    def _on_odom(self, msg):
        self._odom = msg

    def _update_dropout(self, now_s):
        if self._dropout_until is not None and now_s >= self._dropout_until:
            self._dropout_until = None
            self.get_logger().info('DVL dropout over.')
        if self._dropout_until is None and self._rng.random() < self._dropout_rate_hz * self._dt:
            length = self._rng.uniform(*self._dropout_len)
            self._dropout_until = now_s + length
            self._dropout_silent = self._rng.random() < self._silent_fraction
            self.get_logger().info(
                f'DVL dropout for {length:.1f} s '
                f'({"silent" if self._dropout_silent else "lock lost"}).')
        return self._dropout_until is not None

    def _tick(self):
        if self._odom is None:
            return
        now = self.get_clock().now()
        dropout = self._update_dropout(now.nanoseconds * 1e-9)
        if dropout and self._dropout_silent:
            return

        pose, tw = self._odom.pose.pose, self._odom.twist.twist
        R_wb = quat_to_matrix(pose.orientation)
        v_b = np.array([tw.linear.x, tw.linear.y, tw.linear.z])
        w_b = np.array([tw.angular.x, tw.angular.y, tw.angular.z])
        v_dvl = self._R_mount.T @ (v_b + np.cross(w_b, self._r))

        dvl_z = pose.position.z + (R_wb @ self._r)[2]
        altitude = float(dvl_z - self._seafloor_z)
        in_range = bool(self._alt_range[0] <= altitude <= self._alt_range[1])
        valid = in_range and not dropout
        if valid != self._lock_state:
            self.get_logger().info(
                f'DVL bottom lock {"acquired" if valid else "lost"} (altitude {altitude:.2f} m).')
            self._lock_state = valid

        if valid:
            v_meas = v_dvl + np.array([self._rng.gauss(0.0, s) for s in self._noise])
            if self._rng.random() < self._p_outlier:
                d = np.array([self._rng.gauss(0.0, 1.0) for _ in range(3)])
                d *= self._rng.uniform(*self._outlier) / max(np.linalg.norm(d), 1e-9)
                v_meas = v_meas + d
                self.get_logger().debug(f'DVL outlier injected: {np.round(d, 3).tolist()} m/s')
        else:
            v_meas = np.zeros(3)  # A50 reports no usable velocity without lock

        stamp = now.to_msg()
        msg = Dvl()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.velocity_mode = Dvl.DVL_MODE_BOTTOM
        msg.dvl_type = Dvl.DVL_TYPE_PISTON
        msg.velocity = Vector3(x=float(v_meas[0]), y=float(v_meas[1]), z=float(v_meas[2]))
        cov = [0.0] * 9
        cov[0], cov[4], cov[8] = (float(s * s) for s in self._noise)
        msg.velocity_covar = cov
        msg.altitude = altitude if valid else -1.0
        msg.course_gnd = float(math.atan2(v_meas[1], v_meas[0]))
        msg.speed_gnd = float(math.hypot(v_meas[0], v_meas[1]))
        msg.sound_speed = self._sound_speed
        msg.beam_ranges_valid = in_range
        msg.beam_velocities_valid = valid
        msg.num_good_beams = 4 if valid else 0
        beam_range = altitude / math.cos(_BEAM_TILT_RAD) if in_range else 0.0
        for i, u in enumerate(_BEAM_UNIT_VECS):
            msg.beam_unit_vec[i] = Vector3(x=u[0], y=u[1], z=u[2])
            msg.range[i] = float(beam_range)
            msg.beam_velocity[i] = float(np.dot(u, v_meas)) if valid else 0.0
        self._vel_pub.publish(msg)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._frame_id
        odom.child_frame_id = self._frame_id
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear = msg.velocity
        tcov = [0.0] * 36
        tcov[0], tcov[7], tcov[14] = cov[0], cov[4], cov[8]
        odom.twist.covariance = tcov
        self._odom_pub.publish(odom)


def main():
    rclpy.init()
    node = SimDvlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
