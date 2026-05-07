"""
Drop /odometry/filtered/local messages with implausibly large forward time
jumps in their header.stamp, and republish the rest to a *_validated topic.

Why: during offline rosbag replay we see the local EKF occasionally emit a
single Odometry message whose header.stamp is wall-clock (today) instead of
sim-time (bag time, days earlier). The exact mechanism inside
robot_localization is not pinned down — most likely a momentary `/clock`
gap during which `nh_->get_clock()->now()` falls back to wall-clock for one
publish cycle — but the consequence on a downstream EKF is catastrophic: it
sees `dt = wall_clock - sim_time ≈ 3 days`, integrates velocity over that
nonsensical interval, and the position state explodes by 10^5 to 10^9 m.

The fix is local: track the last accepted message's stamp; reject any new
message whose stamp jumps forward by more than `max_forward_jump_s` (default
60 s, well above any real bag-replay rate hiccup but well below the
multi-day jump we're filtering out). Backward jumps within
`max_backward_jump_s` are also allowed (bag replay can occasionally deliver
out-of-order messages).

A rejected message is logged at WARN once per occurrence; this is rare in
practice and a high signal in offline replay diagnostics.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


class OdometryValidator(Node):
    def __init__(self) -> None:
        super().__init__("odometry_validator")

        self.declare_parameter("input_topic", "/odometry/filtered/local")
        self.declare_parameter("output_topic", "/odometry/filtered/local_validated")
        self.declare_parameter("max_forward_jump_s", 60.0)
        self.declare_parameter("max_backward_jump_s", 1.0)

        input_topic: str = self.get_parameter("input_topic").value
        output_topic: str = self.get_parameter("output_topic").value
        self._max_forward_ns: int = int(
            float(self.get_parameter("max_forward_jump_s").value) * 1e9
        )
        self._max_backward_ns: int = int(
            float(self.get_parameter("max_backward_jump_s").value) * 1e9
        )

        self._last_accepted_stamp_ns: int | None = None
        self._n_dropped_forward = 0
        self._n_dropped_backward = 0
        self._n_accepted = 0

        self._pub = self.create_publisher(Odometry, output_topic, qos_profile_sensor_data)
        self._sub = self.create_subscription(
            Odometry, input_topic, self._on_msg, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"forwarding {input_topic} → {output_topic} "
            f"(reject if forward jump > {self._max_forward_ns / 1e9:.1f} s "
            f"or backward jump > {self._max_backward_ns / 1e9:.1f} s)"
        )

    def _on_msg(self, msg: Odometry) -> None:
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

        if self._last_accepted_stamp_ns is None:
            # Cold start: accept the first message as reference.
            self._last_accepted_stamp_ns = stamp_ns
            self._n_accepted += 1
            self._pub.publish(msg)
            return

        delta_ns = stamp_ns - self._last_accepted_stamp_ns

        if delta_ns > self._max_forward_ns:
            self._n_dropped_forward += 1
            self.get_logger().warn(
                f"[odom_validator] DROP: forward jump = {delta_ns / 1e9:.3f} s "
                f"(stamp={stamp_ns}, last_accepted={self._last_accepted_stamp_ns}). "
                f"Total dropped (fwd/bwd)={self._n_dropped_forward}/{self._n_dropped_backward}, "
                f"accepted={self._n_accepted}.",
                throttle_duration_sec=1.0,
            )
            return

        if delta_ns < -self._max_backward_ns:
            self._n_dropped_backward += 1
            self.get_logger().warn(
                f"[odom_validator] DROP: backward jump = {delta_ns / 1e9:.3f} s",
                throttle_duration_sec=1.0,
            )
            return

        self._last_accepted_stamp_ns = stamp_ns
        self._n_accepted += 1
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdometryValidator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
