"""
Minimal publish-rate probe for a single topic.

Subscribes to `topic` with default RELIABLE QoS (matching what the EKF
publishes), increments a counter in the callback, and logs the rate every
`log_period_s` seconds. The callback does NO disk I/O and no math beyond
incrementing — so if this probe shows a different rate than
ekf_offline_diagnostic, the diag's CSV-write callback is the bottleneck,
not the publisher. If both show the same low rate, the publisher really is
publishing that slowly.

Use this to isolate "is robot_localization actually publishing at the
configured frequency, or is something downstream throttling?".
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
from nav_msgs.msg import Odometry


_PROBE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=500,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class TopicRateProbe(Node):
    def __init__(self) -> None:
        super().__init__("topic_rate_probe")
        self.declare_parameter("topic", "/odometry/filtered/global")
        self.declare_parameter("log_period_s", 5.0)

        self._topic: str = str(self.get_parameter("topic").value)
        self._log_period_s: float = float(
            self.get_parameter("log_period_s").value
        )

        self._count_total = 0
        self._count_window = 0
        # Wall-clock for rate measurement so the result is independent of
        # any sim-time weirdness.
        self._t_start_wall = time.monotonic()
        self._t_window_start_wall = self._t_start_wall

        self._sub = self.create_subscription(
            Odometry, self._topic, self._on_msg, _PROBE_QOS
        )
        # Wall-clock timer — bypass any /clock issues so the probe is
        # always reporting at a steady rhythm.
        self._timer = self.create_timer(self._log_period_s, self._on_timer)

        self.get_logger().info(
            f"topic_rate_probe: subscribing to {self._topic}, "
            f"log every {self._log_period_s:.1f}s (wall clock)"
        )

    def _on_msg(self, _msg: Odometry) -> None:
        self._count_total += 1
        self._count_window += 1

    def _on_timer(self) -> None:
        t_now = time.monotonic()
        window_dt = t_now - self._t_window_start_wall
        total_dt = t_now - self._t_start_wall
        win_rate = self._count_window / window_dt if window_dt > 0 else 0.0
        total_rate = self._count_total / total_dt if total_dt > 0 else 0.0
        self.get_logger().info(
            f"[rate-probe] {self._topic}: "
            f"window={self._count_window} in {window_dt:.2f}s "
            f"(= {win_rate:.2f} Hz)  total={self._count_total} "
            f"(= {total_rate:.2f} Hz mean over {total_dt:.1f}s)"
        )
        self._count_window = 0
        self._t_window_start_wall = t_now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TopicRateProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Final summary so the result is preserved if SIGTERM kills the run.
        try:
            node._on_timer()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
