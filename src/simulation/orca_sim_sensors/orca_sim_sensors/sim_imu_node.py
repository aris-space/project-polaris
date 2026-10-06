#!/usr/bin/env python3
"""Synthetic Xsens Sirius (VRUAHS) for the Polaris simulation.

Publishes /imu/data (sensor_msgs/Imu, frame imu_link) at the rate of the Gazebo IMU sensor
``xsens_imu`` in the orca4 model (100 Hz, = output_data_rate on the vehicle):

  angular_velocity, linear_acceleration
      From the Gazebo IMU (physics: specific force including gravity, FLU at imu_link)
      plus white noise with the vehicle's measured stddevs (xsens_mti_node.yaml), a small
      constant gyro bias, and spikes.
  orientation
      Gazebo ground truth (ENU, from /odom) plus small white noise and a heading drift
      that only grows while the vehicle turns, like VRUAHS (heading is locked while
      rotationally stationary). The truth is already true-ENU, so imu_yaw_correction runs
      with yaw_offset_deg = 0; ``initial_yaw_offset_deg`` adds a boot-frame offset if you
      want to exercise the calibration.
  covariances
      The values the vehicle's driver publishes (xsens_mti_node.yaml stddevs).

Spikes: with probability ``gyro_spike_probability`` / ``accel_spike_probability`` per sample,
a burst of 1..spike_max_samples samples gets a large error on one random axis. Orientation
is never spiked (the Xsens filter output is smooth), so the selector's heading-stability
gate behaves as on the vehicle.
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from orca_sim_sensors.common import quat_from_rpy, quat_multiply


class _Spiker:
    def __init__(self, rng, probability, lo, hi, max_samples):
        self._rng, self._p, self._lo, self._hi, self._max = rng, probability, lo, hi, max_samples
        self._left, self._axis, self._value = 0, 0, 0.0
        self.count = 0

    def __call__(self, vec):
        if self._left == 0 and self._rng.random() < self._p:
            self._left = self._rng.randint(1, self._max)
            self._axis = self._rng.randrange(3)
            self._value = self._rng.choice((-1.0, 1.0)) * self._rng.uniform(self._lo, self._hi)
            self.count += 1
        if self._left > 0:
            self._left -= 1
            vec[self._axis] += self._value
        return vec


class SimImuNode(Node):
    def __init__(self):
        super().__init__('sim_imu_node')
        self.declare_parameter('input_topic', '/sim/xsens/imu_raw')
        self.declare_parameter('output_topic', '/imu/data')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('seed', -1)
        # Measured on the vehicle (xsens_mti_node.yaml, stationary_02).
        self.declare_parameter('gyro_noise_std', [1.4395e-3, 1.4831e-3, 1.4506e-3])      # rad/s
        self.declare_parameter('accel_noise_std', [1.4372e-2, 4.9542e-3, 7.1293e-3])     # m/s^2
        self.declare_parameter('orientation_stddev_reported', [3.5e-3, 4.36e-3, 0.0174533])  # rad
        self.declare_parameter('gyro_bias_std', 1.0e-5)                                  # rad/s
        self.declare_parameter('orientation_noise_deg', [0.02, 0.02, 0.01])
        self.declare_parameter('yaw_drift_deg_per_sqrt_h', 1.0)
        self.declare_parameter('ahs_rate_threshold_deg_s', 0.5)
        self.declare_parameter('initial_yaw_offset_deg', 0.0)
        self.declare_parameter('gyro_spike_probability', 0.002)
        self.declare_parameter('gyro_spike_min', 0.5)    # rad/s
        self.declare_parameter('gyro_spike_max', 3.0)
        self.declare_parameter('accel_spike_probability', 0.002)
        self.declare_parameter('accel_spike_min', 5.0)   # m/s^2
        self.declare_parameter('accel_spike_max', 30.0)
        self.declare_parameter('spike_max_samples', 3)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        seed = int(p('seed'))
        self._rng = random.Random(None if seed < 0 else seed)
        self._frame_id = p('frame_id')
        self._gyro_std = list(p('gyro_noise_std'))
        self._accel_std = list(p('accel_noise_std'))
        self._ori_cov = [s * s for s in p('orientation_stddev_reported')]
        self._gyro_bias = [self._rng.gauss(0.0, float(p('gyro_bias_std'))) for _ in range(3)]
        self._ori_noise = [math.radians(d) for d in p('orientation_noise_deg')]
        self._yaw_rw = math.radians(float(p('yaw_drift_deg_per_sqrt_h'))) / 60.0  # rad/sqrt(s)
        self._ahs_rate = math.radians(float(p('ahs_rate_threshold_deg_s')))
        self._yaw_bias = math.radians(float(p('initial_yaw_offset_deg')))
        n = int(p('spike_max_samples'))
        self._gyro_spike = _Spiker(self._rng, float(p('gyro_spike_probability')),
                                   float(p('gyro_spike_min')), float(p('gyro_spike_max')), n)
        self._accel_spike = _Spiker(self._rng, float(p('accel_spike_probability')),
                                    float(p('accel_spike_min')), float(p('accel_spike_max')), n)

        # Reliable, like the Xsens driver: works for both reliable and best-effort subscribers.
        self._pub = self.create_publisher(Imu, p('output_topic'), 10)
        self.create_subscription(Imu, p('input_topic'), self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        self._q_truth = None
        self._last_t = None
        self.create_timer(30.0, self._report)
        self.get_logger().info(
            f'Synthetic Xsens Sirius: {p("input_topic")} -> {p("output_topic")}, spikes '
            f'p={p("gyro_spike_probability")}/{p("accel_spike_probability")} per sample')

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        self._q_truth = (q.x, q.y, q.z, q.w)

    def _report(self):
        self.get_logger().info(
            f'IMU spikes so far: gyro {self._gyro_spike.count}, accel {self._accel_spike.count}; '
            f'heading drift {math.degrees(self._yaw_bias):+.3f} deg')

    def _on_imu(self, raw):
        if self._q_truth is None:
            return
        t = raw.header.stamp.sec + raw.header.stamp.nanosec * 1e-9
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t

        w = [raw.angular_velocity.x, raw.angular_velocity.y, raw.angular_velocity.z]
        a = [raw.linear_acceleration.x, raw.linear_acceleration.y, raw.linear_acceleration.z]

        # VRUAHS: the heading only drifts while the vehicle is rotating.
        if math.sqrt(sum(c * c for c in w)) > self._ahs_rate and dt > 0.0:
            self._yaw_bias += self._rng.gauss(0.0, self._yaw_rw * math.sqrt(dt))

        q_noise = quat_from_rpy(*(self._rng.gauss(0.0, s) for s in self._ori_noise))
        q = quat_multiply(quat_from_rpy(0.0, 0.0, self._yaw_bias),
                          quat_multiply(self._q_truth, q_noise))

        w = [w[i] + self._gyro_bias[i] + self._rng.gauss(0.0, self._gyro_std[i]) for i in range(3)]
        a = [a[i] + self._rng.gauss(0.0, self._accel_std[i]) for i in range(3)]
        w = self._gyro_spike(w)
        a = self._accel_spike(a)

        msg = Imu()
        msg.header.stamp = raw.header.stamp
        msg.header.frame_id = self._frame_id
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = w
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = a
        for i in range(3):
            msg.orientation_covariance[4 * i] = self._ori_cov[i]
            msg.angular_velocity_covariance[4 * i] = self._gyro_std[i] ** 2
            msg.linear_acceleration_covariance[4 * i] = self._accel_std[i] ** 2
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = SimImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
