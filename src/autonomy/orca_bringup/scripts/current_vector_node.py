#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import math

# ros2 run orca_bringup current_vector_node.py


class CurrentVector(Node):

    def __init__(self):
        super().__init__('current_vector')

        self.declare_parameter('direction', '')
        self.declare_parameter('amplitude', 0.0)
        self.declare_parameter('period', 0.0)

        self.publisher_ = self.create_publisher(Vector3, '/ocean_current', 10)        

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        direction = self.get_parameter('direction').get_parameter_value().string_value
        amplitude = self.get_parameter('amplitude').get_parameter_value().double_value
        frequency = 1/self.get_parameter('period').get_parameter_value().double_value

        now = self.get_clock().now()
        elapsed_time = now - self.start_time
        t = elapsed_time.nanoseconds / 1e9

        msg = Vector3()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0

        if direction == 'x':
            msg.x = amplitude * math.sin(2*math.pi*frequency*t)
        elif direction == 'y':
            msg.y = amplitude * math.sin(2*math.pi*frequency*t)
        elif direction == 'z':
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
