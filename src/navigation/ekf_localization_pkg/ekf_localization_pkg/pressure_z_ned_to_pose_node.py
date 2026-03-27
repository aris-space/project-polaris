from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from sensor_msgs.msg import FluidPressure
from rclpy.node import Node
from std_msgs.msg import Float64


class PressureZNedToPoseNode(Node):
    """Convert absolute pressure to ENU z for robot_localization (z-only pose)."""

    def __init__(self) -> None:
        super().__init__("pressure_z_ned_to_pose_node")

        self.declare_parameter("input_topic", "/pixhawk/scaled_pressure")
        self.declare_parameter("output_topic", "/sensors/pressure/pose_enu")
        self.declare_parameter("output_frame_id", "odom")
        self.declare_parameter("z_variance", 0.04)
        self.declare_parameter("unused_variance", 1000000.0)
        self.declare_parameter("water_density_kg_m3", 1000.0)
        self.declare_parameter("gravity_m_s2", 9.80665)
        self.declare_parameter("calibration_duration_sec", 8.0)
        # Positive value means sensor is below vehicle origin (depth-positive convention).
        self.declare_parameter("sensor_z_offset_m", 0.0)
        # Optional in-mission recalibration while surfaced.
        self.declare_parameter("update_surface_when_surfaced", False)
        self.declare_parameter("surfaced_depth_threshold_m", 0.15)
        self.declare_parameter("surface_update_alpha", 0.02)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.sub = self.create_subscription(FluidPressure, input_topic, self.callback, 30)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, output_topic, 30)
        self.p_surface_pub = self.create_publisher(
            Float64, "/sensors/pressure/p_surface_pa", 10
        )

        self._calibration_done = False
        self._calibration_start_sec = self.get_clock().now().nanoseconds * 1e-9
        self._calibration_count = 0
        self._calibration_sum_pa = 0.0
        self._p_surface_pa = None

        self.get_logger().info(
            f"Pressure adapter started: {input_topic} (abs pressure) -> {output_topic} (z_enu). "
            "Calibrating surface pressure at startup..."
        )

    def callback(self, msg: FluidPressure) -> None:
        p_abs_pa = float(msg.fluid_pressure)
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if not self._calibration_done:
            self._calibration_sum_pa += p_abs_pa
            self._calibration_count += 1

            duration = float(self.get_parameter("calibration_duration_sec").value)
            if now_sec - self._calibration_start_sec < duration:
                return

            if self._calibration_count == 0:
                self.get_logger().warning("Surface calibration got no samples yet.")
                return

            self._p_surface_pa = self._calibration_sum_pa / float(self._calibration_count)
            self._calibration_done = True
            self.get_logger().info(
                f"Surface calibration done: p_surface={self._p_surface_pa:.2f} Pa "
                f"from {self._calibration_count} samples."
            )

        if self._p_surface_pa is None:
            return

        p_surface_msg = Float64()
        p_surface_msg.data = self._p_surface_pa
        self.p_surface_pub.publish(p_surface_msg)

        rho = float(self.get_parameter("water_density_kg_m3").value)
        g = float(self.get_parameter("gravity_m_s2").value)
        sensor_z_offset_m = float(self.get_parameter("sensor_z_offset_m").value)

        # depth is positive down
        depth_sensor_m = (p_abs_pa - self._p_surface_pa) / (rho * g)
        depth_vehicle_m = depth_sensor_m - sensor_z_offset_m
        # ENU z is positive up
        z_enu = -depth_vehicle_m

        if bool(self.get_parameter("update_surface_when_surfaced").value):
            surfaced_threshold = float(
                self.get_parameter("surfaced_depth_threshold_m").value
            )
            if abs(depth_vehicle_m) <= surfaced_threshold:
                alpha = float(self.get_parameter("surface_update_alpha").value)
                self._p_surface_pa = (1.0 - alpha) * self._p_surface_pa + alpha * p_abs_pa

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
