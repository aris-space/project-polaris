"""Subscribe to sensor_msgs/Imu: roll/pitch from quaternion; yaw from bias-corrected gyro z integration."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from imu_orientation_pkg.quaternion_rpy import quaternion_to_rpy_xyz
from imu_orientation_pkg.yaw_integration import stamp_to_seconds


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
        # If dt between IMU header stamps exceeds this (s), skip that integration step.
        self.declare_parameter("max_dt_sec", 0.25)
        self.declare_parameter(
            "publish_quaternion_yaw_topic",
            "/sensors/imu/yaw_quaternion_rad",
        )
        self.declare_parameter("publish_quaternion_yaw", False)
        self.declare_parameter("gyro_z_deadband_rad_s", 0.025)
        self.declare_parameter("bias_stationary_threshold_rad_s", 0.12)
        self.declare_parameter("bias_learning_rate", 0.01)
        self.declare_parameter("gyro_bias_z_rad_s", 0.0)

        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        yaw_deg_topic = str(self.get_parameter("publish_yaw_deg_topic").value)
        quat_yaw_topic = str(self.get_parameter("publish_quaternion_yaw_topic").value)

        self._pub = self.create_publisher(Vector3Stamped, out_topic, 10)
        self._pub_yaw_deg = self.create_publisher(Float64, yaw_deg_topic, 10)
        self._pub_quat_yaw = self.create_publisher(Float64, quat_yaw_topic, 10)

        self._sub = self.create_subscription(Imu, in_topic, self._cb, 50)

        self._yaw_int = 0.0
        self._last_stamp_sec: float | None = None
        self._last_wz_use: float | None = None
        self._gyro_bias_learned = 0.0

        self.get_logger().info(
            f"imu_to_rpy: {in_topic} -> {out_topic} "
            "(roll/pitch=quat, z=gyro yaw with bias+deadband)"
        )

    def _cb(self, msg: Imu) -> None:
        q = msg.orientation
        n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if n2 < 1e-10:
            if bool(self.get_parameter("skip_invalid_orientation").value):
                return
        roll, pitch, yaw_quat = quaternion_to_rpy_xyz(q.x, q.y, q.z, q.w)

        stamp_sec = stamp_to_seconds(msg.header.stamp.sec, msg.header.stamp.nanosec)
        wz_raw = float(msg.angular_velocity.z)
        deadband = float(self.get_parameter("gyro_z_deadband_rad_s").value)
        stat_thr = float(self.get_parameter("bias_stationary_threshold_rad_s").value)
        alpha = float(self.get_parameter("bias_learning_rate").value)
        manual_bias = float(self.get_parameter("gyro_bias_z_rad_s").value)

        if math.isfinite(wz_raw) and abs(wz_raw) < stat_thr:
            self._gyro_bias_learned = (1.0 - alpha) * self._gyro_bias_learned + alpha * wz_raw

        bias_total = self._gyro_bias_learned + manual_bias
        wz_corr = wz_raw - bias_total if math.isfinite(wz_raw) else 0.0
        wz_use = 0.0 if abs(wz_corr) < deadband else wz_corr

        max_dt = float(self.get_parameter("max_dt_sec").value)
        if self._last_stamp_sec is not None and self._last_wz_use is not None:
            dt = stamp_sec - self._last_stamp_sec
            if (
                math.isfinite(dt)
                and dt > 0.0
                and dt <= max_dt
                and math.isfinite(self._last_wz_use)
            ):
                self._yaw_int += self._last_wz_use * dt
        self._last_stamp_sec = stamp_sec
        self._last_wz_use = wz_use

        out = Vector3Stamped()
        out.header = msg.header
        out.vector.x = roll
        out.vector.y = pitch
        out.vector.z = self._yaw_int
        self._pub.publish(out)

        if bool(self.get_parameter("publish_yaw_deg").value):
            fd = Float64()
            fd.data = math.degrees(self._yaw_int)
            self._pub_yaw_deg.publish(fd)

        if bool(self.get_parameter("publish_quaternion_yaw").value):
            fq = Float64()
            fq.data = yaw_quat
            self._pub_quat_yaw.publish(fq)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuToRpyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
