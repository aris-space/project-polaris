"""
Minimal rate probe for a specific transform on /tf.

Subscribes to /tf (tf2_msgs/TFMessage), filters for the configured
parent_frame_id → child_frame_id pair, and logs the rate every
`log_period_s` seconds. Used to verify the rate at which the global EKF
broadcasts map→odom, independent of /odometry/filtered/global's
measurement-gated publish rate.

If TF publishes at the configured EKF frequency while the topic publishes
at GPS rate, autonomy (which typically does TF lookups, not topic subs)
gets full rate even when the topic is throttled.
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage


# /tf is published with a specific QoS in ROS2: RELIABLE, VOLATILE,
# depth 100. Match that here so we don't introduce QoS-side drops.
_TF_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=500,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class TfRateProbe(Node):
    def __init__(self) -> None:
        super().__init__("tf_rate_probe")
        self.declare_parameter("parent_frame_id", "map")
        self.declare_parameter("child_frame_id", "odom")
        self.declare_parameter("log_period_s", 5.0)

        self._parent: str = str(self.get_parameter("parent_frame_id").value)
        self._child: str = str(self.get_parameter("child_frame_id").value)
        self._log_period_s: float = float(
            self.get_parameter("log_period_s").value
        )

        self._count_total = 0
        self._count_window = 0
        self._count_msgs_total = 0  # any /tf message (for context)
        self._t_start_wall = time.monotonic()
        self._t_window_start_wall = self._t_start_wall

        self._sub = self.create_subscription(
            TFMessage, "/tf", self._on_tf, _TF_QOS
        )
        self._timer = self.create_timer(self._log_period_s, self._on_timer)

        self.get_logger().info(
            f"tf_rate_probe: filtering /tf for "
            f"'{self._parent}' -> '{self._child}', "
            f"log every {self._log_period_s:.1f}s (wall clock)"
        )

    def _on_tf(self, msg: TFMessage) -> None:
        # A single TFMessage can carry multiple transforms — count each that
        # matches our target parent/child pair (typical EKF broadcast is
        # one transform per message, but be safe).
        for tr in msg.transforms:
            self._count_msgs_total += 1
            if (
                tr.header.frame_id == self._parent
                and tr.child_frame_id == self._child
            ):
                self._count_total += 1
                self._count_window += 1

    def _on_timer(self) -> None:
        t_now = time.monotonic()
        window_dt = t_now - self._t_window_start_wall
        total_dt = t_now - self._t_start_wall
        win_rate = self._count_window / window_dt if window_dt > 0 else 0.0
        total_rate = self._count_total / total_dt if total_dt > 0 else 0.0
        self.get_logger().info(
            f"[tf-rate-probe] {self._parent}->{self._child}: "
            f"window={self._count_window} in {window_dt:.2f}s "
            f"(= {win_rate:.2f} Hz)  total={self._count_total} "
            f"(= {total_rate:.2f} Hz mean over {total_dt:.1f}s)  "
            f"(any-tf total={self._count_msgs_total})"
        )
        self._count_window = 0
        self._t_window_start_wall = t_now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TfRateProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._on_timer()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
