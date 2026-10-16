"""
gps_odom_cov_floor — intercept /odometry/gps and republish with a covariance floor.

Why this exists
---------------

`navsat_transform_node` copies the input GPS NavSatFix's `position_covariance`
into the `/odometry/gps` Odometry message. With RTK Fixed, that covariance can
be ~6 mm² (h_acc = 6 mm → R ≈ 4e-5 m²). When `ekf_global_node` consumes such a
tight measurement, the Kalman update math becomes numerically ill-conditioned:

  K = P · Hᵀ · (H · P · Hᵀ + R)⁻¹

with R ≪ diag(P) and P holding non-zero cross-covariance entries (P[x, ax],
P[x, vx], etc., grown during the pre-GPS predict-correct cycles via the
nonlinear rotation Jacobian), the inverse `(HPHᵀ + R)⁻¹` is dominated by
HPHᵀ. Tiny numerical errors in that inversion produce K entries on the
unsensored rows (ax, ay, az, vx, vy, vz) that are huge. A single GPS update
then yanks state.{ax, vx} to wild values; predict at the next step
integrates that wildness into position; state explodes by orders of
magnitude in one cycle. Empirically observed on rectangle_03 offline:
state.x went from −0.37 m to −5602 m in a single step on the first GPS
update (innovation a sane −6 m).

This node floors the diagonal entries of the pose covariance at a
configurable minimum (default 0.01 m² ≡ 10 cm 1σ). The floor only widens
covariance, never tightens, so we never degrade an honestly-loose RTK Float
measurement. The Kalman gain becomes well-conditioned again and the EKF
update on the first GPS arrival behaves like a normal RTK-Float pull-toward-
measurement instead of a numerical blow-up.

Wire it in by setting `ekf_global.yaml`'s `odom1: /odometry/gps_floored`.

Parameters
----------
input_topic       /odometry/gps source from navsat_transform.
output_topic      /odometry/gps_floored republished with floor applied.
min_pos_cov_m2    Minimum allowed diagonal-pose covariance (m²). Default 0.01
                  ≡ 10 cm 1σ. Raise to 0.04 (20 cm) on bags where the EKF
                  still feels too eager; lower toward 1e-3 once we trust the
                  pipeline numerically.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


class GpsOdomCovFloor(Node):
    def __init__(self) -> None:
        super().__init__("gps_odom_cov_floor")
        self.declare_parameter("input_topic", "/odometry/gps")
        self.declare_parameter("output_topic", "/odometry/gps_floored")
        self.declare_parameter("min_pos_cov_m2", 0.01)
        # Frame relabel + position offset — see _on_odom for the full
        # mechanism. Empty output_frame_id disables both behaviours and
        # the node becomes a pure cov-floor passthrough.
        self.declare_parameter("output_frame_id", "")
        # Local-EKF position at GNSS lock time. navsat_transform's
        # /odometry/gps places its origin at the local-EKF origin (because
        # navsat treats the local EKF's world as its world frame). Map
        # frame's origin is at the GNSS datum, which corresponds to
        # local_anchor in odom frame. Subtracting local_anchor converts
        # the position values into map-frame coordinates.
        self.declare_parameter("local_anchor_x", 0.0)
        self.declare_parameter("local_anchor_y", 0.0)
        self.declare_parameter("local_anchor_z", 0.0)

        in_topic: str = str(self.get_parameter("input_topic").value)
        out_topic: str = str(self.get_parameter("output_topic").value)
        self._floor: float = float(self.get_parameter("min_pos_cov_m2").value)
        self._output_frame_id: str = str(
            self.get_parameter("output_frame_id").value
        )
        self._anchor_x: float = float(self.get_parameter("local_anchor_x").value)
        self._anchor_y: float = float(self.get_parameter("local_anchor_y").value)
        self._anchor_z: float = float(self.get_parameter("local_anchor_z").value)

        self._n_in = 0
        self._n_floored_x = 0
        self._n_floored_y = 0
        self._n_floored_z = 0
        self._n_nan_replaced = 0

        self._sub = self.create_subscription(
            Odometry, in_topic, self._on_odom, qos_profile_sensor_data
        )
        self._pub = self.create_publisher(Odometry, out_topic, 10)

        if self._output_frame_id:
            relabel_msg = (
                f"frame: '{self._output_frame_id}', "
                f"anchor=({self._anchor_x:+.3f}, {self._anchor_y:+.3f}, "
                f"{self._anchor_z:+.3f}) m subtracted"
            )
        else:
            relabel_msg = "passthrough (no relabel, no anchor subtraction)"
        self.get_logger().info(
            f"gps_odom_cov_floor: {in_topic} -> {out_topic}, "
            f"min_pos_cov_m2={self._floor:.4f} ({self._floor ** 0.5:.3f} m 1σ), "
            f"{relabel_msg}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        # ── Frame relabel + anchor subtraction ─────────────────────────────
        # navsat_transform_node treats the local EKF's world (odom) as its
        # world frame, so /odometry/gps positions are in odom frame:
        #     gps_in_odom = local_anchor + (gps_utm - datum_utm)
        # where local_anchor is the local-EKF position at GNSS-lock time
        # and (gps_utm - datum_utm) is the boat's true map-frame position.
        #
        # The global EKF (world_frame=map) needs map-frame measurements.
        # If we leave the message in odom frame, robot_localization tries
        # to transform via map→odom TF — but that TF is published by the
        # global EKF itself based on its drifting state, creating a
        # positive feedback loop:
        #   state drifts → map→odom TF drifts → /odometry/gps appears
        #   shifted by the drift → "innovation" is contaminated by the
        #   drift → state drifts further.
        #
        # Subtracting local_anchor makes the values genuine map-frame
        # coordinates; relabelling header.frame_id="map" tells
        # robot_localization to skip the TF lookup entirely. Innovation
        # is computed directly in map frame between map-frame state and
        # map-frame measurement — no TF involved, no feedback.
        if self._output_frame_id:
            msg.header.frame_id = self._output_frame_id
            msg.pose.pose.position.x -= self._anchor_x
            msg.pose.pose.position.y -= self._anchor_y
            msg.pose.pose.position.z -= self._anchor_z

        # pose.covariance is 6×6 row-major over (x, y, z, roll, pitch, yaw).
        # Diagonal indices: x=0, y=7, z=14, roll=21, pitch=28, yaw=35.
        # NaN/Inf must be replaced with the floor, not floored: a `<` check
        # against NaN returns False and would let an unfloored NaN through to
        # the EKF, blowing the Kalman gain to garbage on first contact.
        #
        # We also zero out the off-diagonals of the position block (x↔y, x↔z,
        # y↔z) and the position↔orientation cross-cov entries. navsat_transform
        # propagates the GPS NavSatFix position_covariance through the
        # UTM→map rotation, which generates cross-correlation entries; if
        # they're large or NaN, the 2×2 R matrix the EKF uses for the x,y
        # update is ill-conditioned (det ≈ 0) and the matrix inverse
        # produces a Kalman gain that's effectively decoupled from the
        # innovation magnitude. v8 grid_02 showed effective K ≈ 0.025 on
        # an RTK-Float update where P/(P+R) predicted K ≈ 0.8 — GPS pulled
        # state by only 0.13 m of a 5.24 m innovation. Zeroing the off-
        # diagonals turns R into pure diag([R_x, R_y]) which is always
        # well-conditioned at the floored magnitudes.
        cov = list(msg.pose.covariance)

        # 1) Floor / NaN-replace the position diagonals.
        for idx, name in ((0, "x"), (7, "y"), (14, "z")):
            v = cov[idx]
            if not math.isfinite(v):
                cov[idx] = self._floor
                self._n_nan_replaced += 1
            elif v < self._floor:
                cov[idx] = self._floor
                if name == "x":
                    self._n_floored_x += 1
                elif name == "y":
                    self._n_floored_y += 1
                else:
                    self._n_floored_z += 1

        # 2) Zero all off-diagonals of the 6×6 (positions and orientations).
        # Symmetric, so we only need to walk i<j and clear (i, j) and (j, i).
        for i in range(6):
            for j in range(i + 1, 6):
                cov[i * 6 + j] = 0.0
                cov[j * 6 + i] = 0.0

        # 3) Replace any remaining NaN/Inf on orientation diagonals with a
        # sane default — the EKF doesn't fuse orientation from this topic
        # (odom1_config has roll/pitch/yaw=false), but a NaN anywhere in the
        # matrix can still poison eigenvalue checks inside robot_localization.
        for idx in (21, 28, 35):
            if not math.isfinite(cov[idx]):
                cov[idx] = 1.0  # 1 rad² ≡ effectively unobserved
                self._n_nan_replaced += 1

        msg.pose.covariance = cov
        self._n_in += 1
        self._pub.publish(msg)

        # Periodic summary so we can confirm the floor is biting.
        if self._n_in % 100 == 0:
            self.get_logger().info(
                f"floored {self._n_in} messages so far  "
                f"(x: {self._n_floored_x}, y: {self._n_floored_y}, "
                f"z: {self._n_floored_z}, nan_replaced: {self._n_nan_replaced})"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsOdomCovFloor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
