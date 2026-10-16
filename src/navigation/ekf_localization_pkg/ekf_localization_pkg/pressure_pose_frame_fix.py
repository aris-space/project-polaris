"""
pressure_pose_frame_fix — intercept /sensors/pressure/pose_enu and republish
in map frame so the global EKF doesn't need a TF lookup.

Why this exists
---------------

`pressure_pose_pkg` publishes `/sensors/pressure/pose_enu` with
`header.frame_id = "odom"` — its z is the boat's altitude relative to the
local EKF's odom origin (where pressure zeroed at startup).

The local EKF lives in odom frame and consumes this directly — pressure
pins state.z cleanly. The global EKF lives in map frame; consuming the
same message would require robot_localization to TF-transform `odom→map`
on every measurement. **That TF is published by the global EKF itself
based on its drifting state**, which creates the same positive-feedback
loop documented for `/odometry/gps` in the v15 (bug 7) fix. Observed
empirically on the v18 grid_02 recording:

  - local EKF z:      +0.06 m steady (pressure-pinned)
  - anchored shadow z: +0.04 m steady (matches local)
  - **global EKF z:   +0.45 m mean, oscillating between +0.2 and +0.5**

The 0.4 m offset is the steady-state equilibrium of the z-channel feedback
loop. Less catastrophic than the horizontal version (because z has only
pressure pulling on it, not GPS pulling against a contaminated TF) but
still a structural error.

The fix mirrors `gps_odom_cov_floor`'s mechanism for the GPS odometry:
subtract `local_anchor_z` from the message's z to convert from odom-frame
position to map-frame position, then rewrite `header.frame_id = "map"` so
robot_localization skips the TF lookup. The position values are then
genuinely in map frame and the feedback loop is broken.

The local EKF keeps subscribing to the original
`/sensors/pressure/pose_enu` (which is in odom frame, correct for local).
Only the global EKF's `pose0` needs to point at the relabeled topic.

Parameters
----------
input_topic       Default `/sensors/pressure/pose_enu`.
output_topic      Default `/sensors/pressure/pose_enu_map`.
output_frame_id   Default `"map"`. Empty string disables the relabel +
                  subtraction (passthrough).
local_anchor_z    Local-EKF z at GNSS lock. Subtracted from
                  pose.position.z to convert into map frame. Comes from
                  `gnss_datum_watchdog` via launch arg.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped


class PressurePoseFrameFix(Node):
    def __init__(self) -> None:
        super().__init__("pressure_pose_frame_fix")
        self.declare_parameter("input_topic", "/sensors/pressure/pose_enu")
        self.declare_parameter("output_topic", "/sensors/pressure/pose_enu_map")
        self.declare_parameter("output_frame_id", "map")
        self.declare_parameter("local_anchor_z", 0.0)

        in_topic: str = str(self.get_parameter("input_topic").value)
        out_topic: str = str(self.get_parameter("output_topic").value)
        self._output_frame_id: str = str(
            self.get_parameter("output_frame_id").value
        )
        self._anchor_z: float = float(self.get_parameter("local_anchor_z").value)

        self._n_in = 0
        self._n_nan_replaced = 0

        self._sub = self.create_subscription(
            PoseWithCovarianceStamped, in_topic, self._on_pose,
            qos_profile_sensor_data,
        )
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, out_topic, 10,
        )

        if self._output_frame_id:
            mode_msg = (
                f"frame: '{self._output_frame_id}', "
                f"anchor_z={self._anchor_z:+.3f} m subtracted"
            )
        else:
            mode_msg = "passthrough (no relabel, no anchor subtraction)"
        self.get_logger().info(
            f"pressure_pose_frame_fix: {in_topic} -> {out_topic}, {mode_msg}"
        )

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if self._output_frame_id:
            msg.header.frame_id = self._output_frame_id
            msg.pose.pose.position.z -= self._anchor_z

        # NaN guard on the z covariance diagonal (index 14 in 6x6
        # row-major) — same defensive logic as gps_odom_cov_floor.
        # robot_localization runs eigenvalue checks on the full 6x6;
        # a NaN anywhere can poison the update path even on channels
        # that aren't fused.
        cov = list(msg.pose.covariance)
        for idx in (0, 7, 14, 21, 28, 35):
            if not math.isfinite(cov[idx]):
                cov[idx] = 1.0
                self._n_nan_replaced += 1
        msg.pose.covariance = cov

        self._n_in += 1
        self._pub.publish(msg)

        if self._n_in % 500 == 0:
            self.get_logger().info(
                f"forwarded {self._n_in} messages so far  "
                f"(nan_replaced: {self._n_nan_replaced})"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PressurePoseFrameFix()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
