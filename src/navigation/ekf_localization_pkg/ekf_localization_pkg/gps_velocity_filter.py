"""
gps_velocity_filter — drop GPS fixes that imply impossible boat velocities.

Why this exists
---------------

The watchdog's h_acc gate (UBX-NAV-HPPOSLLH.h_acc <= 0.5 m) catches GPS
fixes that *report* low confidence. But RTK has a documented failure mode
where the receiver thinks it has fix-quality lock — h_acc reports a few
centimetres — and yet the actual reported position is wrong by several
metres. The fix passes the h_acc gate, lands on /gps/validated, propagates
through gps_to_map_position into the EKF, and the EKF (especially with a
tight cov_floor giving K close to 1) chases the glitch.

Empirically observed on grid_02 at bag t≈191 s: three consecutive
/gps/validated samples reported the boat at (-4.7, -3.1) while the true
position was at (-12, +1). That's a 7.5 m sample-to-sample jump in <1 s,
implying an apparent velocity of ~8 m/s — physically impossible for an
underwater vehicle. The EKF still followed it because K was large.

This node sits between the watchdog and gps_to_map_position. For each new
/gps/validated message it:

  1. Projects lat/lon to UTM via pyproj.
  2. Compares to the previously accepted fix.
  3. Computes the apparent ground-frame velocity as (Δposition / Δt).
  4. If the apparent velocity exceeds max_velocity_m_s, the fix is
     rejected (not republished). Otherwise it's republished on
     /gps/validated_filtered, and becomes the new reference for the
     next velocity check.

A timeout (max_time_gap_s) bypasses the velocity check when the last
accepted fix is too old — the boat could legitimately be returning from
a long GPS dropout, where the position has had time to drift legitimately
far from the previous sample.

The first fix after node startup is always accepted (no previous reference
to compare against).

Parameters
----------
input_topic         /gps/validated (the watchdog's output).
output_topic        /gps/validated_filtered (consumed by gps_to_map_position).
max_velocity_m_s    Maximum plausible apparent velocity (default 3.0 m/s).
                    POLARIS surface speed is typically <1.5 m/s; the
                    3 m/s gate gives 2× safety margin against legitimate
                    quick maneuvers while catching the 7+ m glitches.
max_time_gap_s      If (now - t_last_accepted) > this, bypass the
                    velocity check and accept this fix unconditionally
                    (default 5.0 s).
min_dt_s            Floor on Δt before computing velocity, to avoid
                    division by zero on near-duplicate stamps
                    (default 0.01 s = 10 ms).
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

from pyproj import Transformer


def _utm_epsg(lat: float, lon: float) -> str:
    zone = int((lon + 180.0) / 6.0) + 1
    hemisphere = "6" if lat >= 0.0 else "7"
    return f"EPSG:32{hemisphere}{zone:02d}"


def _stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class GpsVelocityFilter(Node):
    def __init__(self) -> None:
        super().__init__("gps_velocity_filter")

        self.declare_parameter("input_topic", "/gps/validated")
        self.declare_parameter("output_topic", "/gps/validated_filtered")
        self.declare_parameter("max_velocity_m_s", 3.0)
        self.declare_parameter("max_time_gap_s", 5.0)
        self.declare_parameter("min_dt_s", 0.01)

        in_topic: str = str(self.get_parameter("input_topic").value)
        out_topic: str = str(self.get_parameter("output_topic").value)
        self._max_v: float = float(self.get_parameter("max_velocity_m_s").value)
        self._max_gap: float = float(self.get_parameter("max_time_gap_s").value)
        self._min_dt: float = float(self.get_parameter("min_dt_s").value)

        # UTM transformer is built lazily on the first fix (we don't know
        # the zone until we see a lat/lon). After that it's reused.
        self._to_utm = None
        self._last_t: float | None = None
        self._last_e: float | None = None
        self._last_n: float | None = None

        self._n_in = 0
        self._n_accepted = 0
        self._n_rejected = 0
        self._n_timeout_bypass = 0

        self._sub = self.create_subscription(
            NavSatFix, in_topic, self._on_fix, qos_profile_sensor_data
        )
        self._pub = self.create_publisher(NavSatFix, out_topic, 10)

        self.get_logger().info(
            f"gps_velocity_filter: {in_topic} -> {out_topic}, "
            f"max_velocity={self._max_v:.2f} m/s, "
            f"max_time_gap={self._max_gap:.1f} s, "
            f"min_dt={self._min_dt:.3f} s"
        )

    def _on_fix(self, msg: NavSatFix) -> None:
        self._n_in += 1
        # Same null-island / status guards as upstream — defensive.
        if msg.status.status < 0 or abs(msg.latitude) < 0.1:
            return

        # Lazy UTM init on first fix.
        if self._to_utm is None:
            epsg = _utm_epsg(msg.latitude, msg.longitude)
            self._to_utm = Transformer.from_crs(
                "EPSG:4326", epsg, always_xy=True
            )
            self.get_logger().info(
                f"UTM initialised on first fix: {epsg}"
            )

        e, n = self._to_utm.transform(msg.longitude, msg.latitude)
        t = _stamp_s(msg.header.stamp)

        # First fix is always accepted (no reference to compare against).
        if self._last_t is None:
            self._accept(msg, t, e, n, reason="first fix")
            return

        dt = t - self._last_t

        # Timeout bypass: the last accepted fix is so old that we can't
        # use it as a velocity reference (e.g., GPS dropout). Accept
        # unconditionally and reset the reference.
        if dt > self._max_gap:
            self._n_timeout_bypass += 1
            self._accept(
                msg, t, e, n,
                reason=f"timeout bypass (Δt={dt:.2f}s > {self._max_gap:.1f}s)",
            )
            return

        # Pathological Δt (stamps too close to compute meaningful
        # velocity). Treat as "duplicate" and pass through without
        # updating the reference.
        if dt < self._min_dt:
            self._accept(msg, t, e, n, update_ref=False,
                         reason=f"near-duplicate (Δt={dt*1000:.1f}ms)")
            return

        de = e - self._last_e
        dn = n - self._last_n
        d = math.hypot(de, dn)
        v_apparent = d / dt

        if v_apparent > self._max_v:
            self._n_rejected += 1
            self.get_logger().warn(
                f"REJECT  Δ=({de:+.3f}, {dn:+.3f}) m  "
                f"Δt={dt:.2f}s  v_apparent={v_apparent:.2f} m/s "
                f"> {self._max_v:.2f} m/s — likely RTK glitch"
            )
            return

        self._accept(msg, t, e, n,
                     reason=f"v_apparent={v_apparent:.3f} m/s")

    def _accept(self, msg: NavSatFix, t: float, e: float, n: float,
                update_ref: bool = True, reason: str = "") -> None:
        self._pub.publish(msg)
        self._n_accepted += 1
        if update_ref:
            self._last_t = t
            self._last_e = e
            self._last_n = n
        if self._n_accepted % 50 == 0:
            self.get_logger().info(
                f"accepted {self._n_accepted}, rejected {self._n_rejected}, "
                f"timeout_bypass {self._n_timeout_bypass} "
                f"(in: {self._n_in})"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsVelocityFilter()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.get_logger().info(
                f"shutdown: accepted={node._n_accepted} "
                f"rejected={node._n_rejected} "
                f"timeout_bypass={node._n_timeout_bypass} "
                f"(in: {node._n_in})"
            )
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
