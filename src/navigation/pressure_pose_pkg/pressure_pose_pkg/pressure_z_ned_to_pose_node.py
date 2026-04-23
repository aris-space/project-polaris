import statistics

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import FluidPressure
from rclpy.node import Node
from std_msgs.msg import Float64


class PressureZNedToPoseNode(Node):
    """Convert absolute (or gauge) fluid pressure to ENU z for robot_localization (z-only pose).

    MAVLink SCALED_PRESSURE / SCALED_PRESSURE2 ``press_abs`` is *absolute* static pressure in hPa
    (your bridge converts to Pa). That is not depth by itself: depth in water is proportional to
    (P_abs - P_surface). Pixhawk sensor calibration corrects scale/offset of the sensor; it does
    not remove the need for a surface reference unless you interpret the stream as gauge pressure
    (see ``fluid_pressure_is_gauge``).

    On startup (absolute mode only), unless ``skip_calibration`` is True the node collects pressure
    samples for ``calibration_duration_s`` seconds, computes their median, and uses it as the
    surface-pressure reference for the rest of the run. The calibrated value is published on
    ``/sensors/pressure/p_surface_pa`` so the Keller node picks it up automatically.
    """

    def __init__(self) -> None:
        super().__init__("pressure_z_ned_to_pose_node")

        self.declare_parameter("input_topic", "/pixhawk/scaled_pressure")
        self.declare_parameter("output_topic", "/sensors/pressure/pose_enu")
        self.declare_parameter("output_frame_id", "odom")
        self.declare_parameter("z_variance", 0.04)
        self.declare_parameter("unused_variance", 1000000.0)
        self.declare_parameter("water_density_kg_m3", 1000.0)
        self.declare_parameter("gravity_m_s2", 9.80665)
        # Positive value means sensor is below vehicle origin (depth-positive convention).
        self.declare_parameter("sensor_z_offset_m", 0.0)
        # Absolute pressure at the water surface (Pa), same units as sensor_msgs/FluidPressure.
        # Used when fluid_pressure_is_gauge is false. Typical: ~101325 Pa at surface; override
        # for barometric altitude or a measured surface value.
        self.declare_parameter("p_surface_pa", 101325.0)
        # If true, treat fluid_pressure as gauge (Pa) relative to surface (0 at surface) so
        # depth = p / (rho*g). Use only if your firmware publishes gauge pressure in this field.
        self.declare_parameter("fluid_pressure_is_gauge", False)
        # Startup calibration: collect samples for this many seconds, then use median as p_surface_pa.
        self.declare_parameter("calibration_duration_s", 10.0)
        # Set True to skip auto-calibration and use p_surface_pa directly (legacy / sim behaviour).
        self.declare_parameter("skip_calibration", False)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.sub = self.create_subscription(FluidPressure, input_topic, self.callback, 30)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, output_topic, 30)
        self.p_surface_pub = self.create_publisher(
            Float64, "/sensors/pressure/p_surface_pa", 10
        )

        gauge = bool(self.get_parameter("fluid_pressure_is_gauge").value)
        p_surf = float(self.get_parameter("p_surface_pa").value)
        skip_cal = bool(self.get_parameter("skip_calibration").value)
        cal_duration = float(self.get_parameter("calibration_duration_s").value)

        self._calibration_samples: list = []
        # Once calibrated (or skipped), _p_surface_pa holds the value to use.
        self._p_surface_pa: float = p_surf
        self._calibrated: bool = skip_cal or gauge

        if not self._calibrated:
            self.get_logger().info(
                f"Surface-pressure calibration started: collecting {input_topic} samples "
                f"for {cal_duration:.0f} s. Using p_surface_pa={p_surf:.1f} Pa in the meantime."
            )
            self._calibration_timer = self.create_timer(cal_duration, self._finish_calibration)
        else:
            self.get_logger().info(
                f"Pressure adapter: {input_topic} -> {output_topic} (z_enu). "
                f"{'Gauge mode: depth = p/(rho*g).' if gauge else f'Absolute mode: p_surface_pa={p_surf:.1f} Pa (calibration skipped).'}"
            )

    def _finish_calibration(self) -> None:
        self._calibration_timer.cancel()
        self._calibrated = True

        if not self._calibration_samples:
            self.get_logger().warning(
                "Calibration window ended with no pressure samples. "
                f"Keeping p_surface_pa={self._p_surface_pa:.2f} Pa."
            )
            return

        median_pa = statistics.median(self._calibration_samples)
        n = len(self._calibration_samples)
        prev = self._p_surface_pa
        self._p_surface_pa = median_pa
        self._calibration_samples.clear()

        # Reflect calibrated value in the ROS2 parameter so it's visible via ros2 param get.
        self.set_parameters([Parameter("p_surface_pa", Parameter.Type.DOUBLE, median_pa)])

        self.get_logger().info(
            f"Surface pressure calibrated: {median_pa:.2f} Pa "
            f"(n={n} samples, was {prev:.2f} Pa)."
        )

    def callback(self, msg: FluidPressure) -> None:
        p_pa = float(msg.fluid_pressure)
        gauge = bool(self.get_parameter("fluid_pressure_is_gauge").value)
        rho = float(self.get_parameter("water_density_kg_m3").value)
        g = float(self.get_parameter("gravity_m_s2").value)
        sensor_z_offset_m = float(self.get_parameter("sensor_z_offset_m").value)

        if not self._calibrated:
            self._calibration_samples.append(p_pa)

        if gauge:
            p_surface_used = 0.0
            depth_sensor_m = p_pa / (rho * g)
        else:
            p_surface_used = self._p_surface_pa
            depth_sensor_m = (p_pa - p_surface_used) / (rho * g)

        p_surface_msg = Float64()
        p_surface_msg.data = p_surface_used
        self.p_surface_pub.publish(p_surface_msg)

        depth_vehicle_m = depth_sensor_m - sensor_z_offset_m
        # ENU z is positive up
        z_enu = -depth_vehicle_m

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
