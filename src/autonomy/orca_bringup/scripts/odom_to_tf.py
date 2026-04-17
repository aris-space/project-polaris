#!/usr/bin/env python3
"""Publish odom -> base_link from Gazebo (or other) /odom for sim when base controller is off."""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class OdomToTfNode(Node):
    def __init__(self) -> None:
        super().__init__('odom_to_tf')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('parent_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.parent_frame_id = self.get_parameter('parent_frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        self.tf_broadcaster = TransformBroadcaster(self)
        # Match ros_gz_bridge / Gazebo odometry (best-effort); default reliable QoS will not connect.
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Publishing TF {self.parent_frame_id} -> {self.child_frame_id} from {odom_topic}'
        )

    def _on_odom(self, msg: Odometry) -> None:
        try:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.header.frame_id = self.parent_frame_id
            tf_msg.child_frame_id = self.child_frame_id

            tf_msg.transform.translation.x = msg.pose.pose.position.x
            tf_msg.transform.translation.y = msg.pose.pose.position.y
            tf_msg.transform.translation.z = msg.pose.pose.position.z
            tf_msg.transform.rotation = msg.pose.pose.orientation

            self.tf_broadcaster.sendTransform(tf_msg)
        except Exception as e:
            self.get_logger().error(f'odom_to_tf failed to publish transform: {e}')


def main() -> None:
    rclpy.init()
    node = OdomToTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
