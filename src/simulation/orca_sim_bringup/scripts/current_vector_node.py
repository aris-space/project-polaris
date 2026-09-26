#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Vector3
import math

# Match ros_gz_bridge /ocean_current ROS subscription (transient_local); default
# rclpy publisher is volatile and will not connect.
_OCEAN_CURRENT_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
import random

# ros2 run orca_bringup current_vector_node.py


class CurrentVector(Node):

    def __init__(self):
        super().__init__('current_vector')

        self.declare_parameter('direction', '')
        self.declare_parameter('amplitude', 0.0)
        self.declare_parameter('period', 0.0)
        self.declare_parameter('noise_stddev', 0.08)
        self.declare_parameter('noise_time_constant', 1.5)

        self.publisher_ = self.create_publisher(
            Vector3, '/ocean_current', _OCEAN_CURRENT_QOS)

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = self.get_clock().now()
        self.last_time = self.start_time

        self.noise = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def timer_callback(self):
        direction = self.get_parameter('direction').get_parameter_value().string_value
        amplitude = self.get_parameter('amplitude').get_parameter_value().double_value
        period = self.get_parameter('period').get_parameter_value().double_value
        frequency = 1.0 / period if period != 0.0 else 0.0
        noise_stddev = self.get_parameter('noise_stddev').get_parameter_value().double_value
        noise_tau = self.get_parameter('noise_time_constant').get_parameter_value().double_value

        now = self.get_clock().now()
        elapsed_time = now - self.start_time
        t = elapsed_time.nanoseconds / 1e9

        msg = Vector3()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0

        if direction == 'dir_x':
            msg.x = amplitude * math.sin(2*math.pi*frequency*t)
        elif direction == 'dir_y':
            msg.y = amplitude * math.sin(2*math.pi*frequency*t)
        elif direction == 'dir_z':
            msg.z = amplitude * math.sin(2*math.pi*frequency*t)
        elif direction == 'xy':
            msg.x = amplitude * math.sin(2*math.pi*frequency*t)
            msg.y = msg.x
        elif direction == 'xz':
            msg.x = amplitude * math.sin(2*math.pi*frequency*t)
            msg.z = msg.x
        elif direction == 'yz':
            msg.y = amplitude * math.sin(2*math.pi*frequency*t)
            msg.z = msg.y
        elif direction == 'xyz':
            msg.x = amplitude * math.sin(2*math.pi*frequency*t)
            msg.y = msg.x
            msg.z = msg.x
        elif direction == 'con_x':
            msg.x = amplitude
        elif direction == 'con_y':
            msg.y = amplitude
        elif direction == 'con_z':
            msg.z = amplitude
        elif direction == 'con_xy':
            msg.x = amplitude
            msg.y = msg.x
        elif direction == 'con_xz':
            msg.x = amplitude
            msg.z = msg.x
        elif direction == 'con_yz':
            msg.y = amplitude
            msg.z = msg.y
        elif direction == 'con_xyz':
            msg.x = amplitude
            msg.y = msg.x
            msg.z = msg.x
        elif direction == 'ramp_x':
            msg.x = amplitude/10 * t
        elif direction == 'ramp_y':
            msg.y = amplitude/10 * t
        elif direction == 'ramp_z':
            msg.z = amplitude/10 * t
        elif direction == 'ramp_xy':
            msg.x = amplitude/10 * t
            msg.y = msg.x
        elif direction == 'ramp_xz':
            msg.x = amplitude/10 * t
            msg.z = msg.x
        elif direction == 'ramp_yz':
            msg.y = amplitude/10 * t
            msg.z = msg.y
        elif direction == 'ramp_xyz':
            msg.x = amplitude/10 * t
            msg.y = msg.x
            msg.z = msg.x

        # Apply First-Order Gauss-Markov noise to active axes
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if noise_stddev > 0.0:
            if noise_tau > 0.0:
                alpha = math.exp(-dt / noise_tau)
                noise_multiplier = noise_stddev * math.sqrt(1.0 - alpha**2)
            else:
                # Fallback to pure white noise if time constant is 0
                alpha = 0.0
                noise_multiplier = noise_stddev

            if 'x' in direction:
                self.noise['x'] = alpha * self.noise['x'] + noise_multiplier * random.gauss(0.0, 1.0)
                msg.x += self.noise['x']
            if 'y' in direction:
                self.noise['y'] = alpha * self.noise['y'] + noise_multiplier * random.gauss(0.0, 1.0)
                msg.y += self.noise['y']
            if 'z' in direction:
                self.noise['z'] = alpha * self.noise['z'] + noise_multiplier * random.gauss(0.0, 1.0)
                msg.z += self.noise['z']


        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: x=%.2f y=%.2f z=%.2f' % (msg.x, msg.y, msg.z))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CurrentVector()
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
