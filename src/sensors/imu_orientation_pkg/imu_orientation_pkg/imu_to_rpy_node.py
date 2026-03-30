"""Subscribe to sensor_msgs/Imu, publish roll/pitch/yaw derived from orientation quaternion."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from imu_orientation_pkg.quaternion_rpy import quaternion_to_rpy_xyz


class ImuToRpyNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_to_rpy_node")

        self.declare_parameter("input_topic", "/imu/data")
        self.declare_parameter(
            "output_topic",
            "/sensors/imu/orientation_rpy",
        )
        self.declare_parameter("publish_yaw_deg_topic", "/sensors/imu/yaw_deg")
        self.declare_parameter("publish_yaw_deg", True)
        self.declare_parameter("skip_invalid_orientation", True)

        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        yaw_deg_topic = str(self.get_parameter("publish_yaw_deg_topic").value)

        self._pub = self.create_publisher(Vector3Stamped, out_topic, 10)
        self._pub_yaw_deg = self.create_publisher(Float64, yaw_deg_topic, 10)

        self._sub = self.create_subscription(Imu, in_topic, self._cb, 50)

        self.get_logger().info(
            f"imu_to_rpy: {in_topic} -> {out_topic} (Vector3Stamped x=roll,y=pitch,z=yaw rad)"
        )

    def _cb(self, msg: Imu) -> None:
        q = msg.orientation
        n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if n2 < 1e-10:
            if bool(self.get_parameter("skip_invalid_orientation").value):
                return
        roll, pitch, yaw = quaternion_to_rpy_xyz(q.x, q.y, q.z, q.w)

        out = Vector3Stamped()
        out.header = msg.header
        out.vector.x = roll
        out.vector.y = pitch
        out.vector.z = yaw
        self._pub.publish(out)

        if bool(self.get_parameter("publish_yaw_deg").value):
            fd = Float64()
            fd.data = math.degrees(yaw)
            self._pub_yaw_deg.publish(fd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuToRpyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
