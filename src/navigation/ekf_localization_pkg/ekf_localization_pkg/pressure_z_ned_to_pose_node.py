from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class PressureZNedToPoseNode(Node):
    """Convert /pixhawk/z_ned (NED) into ENU pose-z for robot_localization."""

    def __init__(self) -> None:
        super().__init__("pressure_z_ned_to_pose_node")

        self.declare_parameter("input_topic", "/pixhawk/z_ned")
        self.declare_parameter("output_topic", "/sensors/pressure/pose_enu")
        self.declare_parameter("output_frame_id", "odom")
        self.declare_parameter("z_variance", 0.04)
        self.declare_parameter("unused_variance", 1000000.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.sub = self.create_subscription(Float32, input_topic, self.callback, 30)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, output_topic, 30)

        self.get_logger().info(
            f"Pressure adapter started: {input_topic} (z_ned) -> {output_topic} (z_enu)"
        )

    def callback(self, msg: Float32) -> None:
        # Convert NED z to ENU z.
        z_enu = -float(msg.data)

        out = PoseWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = str(self.get_parameter("output_frame_id").value)

        out.pose.pose.position.x = 0.0
        out.pose.pose.position.y = 0.0
        out.pose.pose.position.z = z_enu
        out.pose.pose.orientation.w = 1.0

        unused_var = float(self.get_parameter("unused_variance").value)
        z_var = float(self.get_parameter("z_variance").value)
        cov = [0.0] * 36
        cov[0] = unused_var
        cov[7] = unused_var
        cov[14] = z_var
        cov[21] = unused_var
        cov[28] = unused_var
        cov[35] = unused_var
        out.pose.covariance = cov

        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PressureZNedToPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
