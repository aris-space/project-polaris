"""
Anchors the local EKF output (in odom frame) to a GPS-derived map frame and
republishes as /odometry/filtered/global. A simpler alternative to a Kalman
global EKF that fuses GPS+local-EKF: dead-reckoning quality matches the local
EKF exactly (it IS the local EKF), absolute accuracy is ~the GPS accuracy at
the moment of the most recent re-anchor.

How it works
------------
1. On the first valid GPS fix (status>=0, |lat|>0.1°, |lon|>0.1°), record the
   local EKF position at that instant: (Lx0, Ly0, Lz0). Project the GPS fix
   to UTM and treat it as the datum (lat/lon/alt that defines map frame
   origin). Compute fixed offset so the snapshot moment maps to (0,0,0) in
   map frame: offset = -(Lx0, Ly0, Lz0).

2. For every subsequent local-odom message, publish global odom with:
       map_position = local_position + offset
   Orientation/velocity/covariance pass through unchanged (orientation is
   identical in odom and map for ENU-aligned frames; velocity is in
   child_frame_id and is origin-independent).

3. On each subsequent valid GPS fix (if reanchor_on_each_fix is true),
   re-compute offset so the current local position maps to that fix's
   map-frame position: offset = (gx, gy, gz) - (lx, ly, lz). Datum stays
   fixed across re-anchors so the map frame is stable. Re-anchors cause a
   discrete jump in the published position; controllers using map-frame
   pose must tolerate this. Default reanchor_on_each_fix=false matches the
   "set origin once on first fix, dead-reckon afterwards" intent.

Design tradeoffs vs a Kalman global EKF
---------------------------------------
+ No tuning required (Q matrices, R covariances, K stability concerns).
+ Robust to the upstream heading bias: heading errors integrate within odom,
  and a re-anchor (when one happens) snaps absolute position back to GPS
  truth in one step regardless of integration direction.
+ Drop-in TF replacement: this node publishes map→odom.
- No fusion of intermediate GPS measurements between anchors. Drift between
  anchors equals the local EKF's dead-reckoning drift (~1% of distance).
- Position jumps when re-anchored. Controllers must handle that.
- No proper map-frame covariance produced (orientation/velocity covariances
  pass through; position covariance is heuristic — see code).

Anchor quality gate
-------------------
The first /fix is only accepted as the datum if it passes:
  - status >= 0 (NavSatFix valid)
  - |lat| > 0.1° AND |lon| > 0.1° (not null-island)
  - h_acc <= h_acc_max_m (only when h_acc_topic is set and ublox_ubx_msgs
    is available — otherwise this part of the gate is skipped)

This matches the gnss_datum_watchdog logic so the anchor is set from the same
quality-gated fix the watchdog would pick.

Parameters
----------
local_odom_topic        Local EKF output  (default /odometry/filtered/local)
gps_topic               NavSatFix source  (default /gps/validated)
global_odom_topic       Odometry output topic        (default /odometry/filtered/global)
global_navsatfix_topic  NavSatFix output topic       (default /gps/filtered/global)
                        Empty string disables the NavSatFix output.
h_acc_topic             UBXNavHPPosLLH topic for h_acc gate (default '' = disabled)
h_acc_max_m             Max h_acc for anchor fix (default 0.5 m)
reanchor_on_each_fix    Re-anchor on every valid fix (default false)
publish_tf              Broadcast map→odom TF        (default true)
map_frame               Map frame name               (default 'map')
odom_frame              Odom frame name              (default 'odom')
yaw_offset_deg          Constant yaw correction applied to position AND orientation
                        of the published Odometry / NavSatFix (degrees, default 0).
                        Positive = rotate counter-clockwise (right-hand-rule about
                        +Z, ENU convention). Use to cancel a constant IMU heading
                        bias: if the dead-reckoning track appears rotated
                        clockwise relative to GNSS truth, set this to a positive
                        value. Does NOT affect the map→odom TF (only the published
                        topics) so downstream consumers using TF lookups are not
                        silently corrected.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from pyproj import Transformer
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_ros import TransformBroadcaster

try:
    from ublox_ubx_msgs.msg import UBXNavHPPosLLH as _UBXNavHPPosLLH
    _HPPOSLLH_AVAILABLE = True
except ImportError:
    _UBXNavHPPosLLH = None
    _HPPOSLLH_AVAILABLE = False

# UBX-NAV-HPPOSLLH h_acc field is in 0.1 mm units
_HPPOSLLH_H_ACC_TO_M = 1e-4


def _utm_epsg(lat: float, lon: float) -> str:
    zone = int((lon + 180.0) / 6.0) + 1
    hemisphere = "6" if lat >= 0.0 else "7"
    return f"EPSG:32{hemisphere}{zone:02d}"


class GnssAnchoredPose(Node):

    def __init__(self) -> None:
        super().__init__("gnss_anchored_pose")

        self.declare_parameter("local_odom_topic", "/odometry/filtered/local")
        self.declare_parameter("gps_topic", "/gps/validated")
        self.declare_parameter("global_odom_topic", "/odometry/filtered/global")
        self.declare_parameter("global_navsatfix_topic", "/gps/filtered/global")
        self.declare_parameter("h_acc_topic", "")
        self.declare_parameter("h_acc_max_m", 0.5)
        self.declare_parameter("reanchor_on_each_fix", False)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("yaw_offset_deg", 0.0)

        local_odom_topic: str = self.get_parameter("local_odom_topic").value
        gps_topic: str = self.get_parameter("gps_topic").value
        global_odom_topic: str = self.get_parameter("global_odom_topic").value
        global_navsatfix_topic: str = self.get_parameter("global_navsatfix_topic").value
        h_acc_topic: str = self.get_parameter("h_acc_topic").value
        self._h_acc_max_m: float = float(self.get_parameter("h_acc_max_m").value)
        self._reanchor: bool = bool(self.get_parameter("reanchor_on_each_fix").value)
        self._publish_tf: bool = bool(self.get_parameter("publish_tf").value)
        self._map_frame: str = self.get_parameter("map_frame").value
        self._odom_frame: str = self.get_parameter("odom_frame").value
        yaw_offset_deg: float = float(self.get_parameter("yaw_offset_deg").value)
        self._yaw_offset_rad: float = 0.0
        self._cos_yaw: float = 1.0
        self._sin_yaw: float = 0.0
        # Half-angle quaternion for orientation rotation about +Z.
        self._q_yaw_w: float = 1.0
        self._q_yaw_z: float = 0.0
        self._update_yaw_cache(yaw_offset_deg)

        self._anchored: bool = False
        self._offset_x: float = 0.0
        self._offset_y: float = 0.0
        self._offset_z: float = 0.0

        self._datum_set: bool = False
        self._datum_e: float = 0.0
        self._datum_n: float = 0.0
        self._datum_alt: float = 0.0
        self._to_utm: Transformer | None = None
        self._from_utm: Transformer | None = None

        self._latest_local: Odometry | None = None
        self._latest_h_acc_m: float | None = None
        self._latest_fix: NavSatFix | None = None

        self._global_pub = self.create_publisher(Odometry, global_odom_topic, 10)
        self._navsatfix_pub = (
            self.create_publisher(NavSatFix, global_navsatfix_topic, 10)
            if global_navsatfix_topic
            else None
        )
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None

        self.create_subscription(
            NavSatFix, gps_topic, self._on_gps, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, local_odom_topic, self._on_local_odom, qos_profile_sensor_data
        )

        self._h_acc_enabled = bool(h_acc_topic) and _HPPOSLLH_AVAILABLE
        if self._h_acc_enabled:
            self.create_subscription(
                _UBXNavHPPosLLH, h_acc_topic, self._on_hp_pos, qos_profile_sensor_data
            )
        elif h_acc_topic and not _HPPOSLLH_AVAILABLE:
            self.get_logger().warn(
                "ublox_ubx_msgs not available — h_acc gate disabled, falling "
                "back to status + null-island check only."
            )

        gate_str = (
            f"h_acc<={self._h_acc_max_m * 100:.0f} cm via '{h_acc_topic}'"
            if self._h_acc_enabled
            else "status>=0 + null-island check only"
        )
        navsatfix_str = (
            f"navsatfix={global_navsatfix_topic}" if self._navsatfix_pub is not None else "navsatfix=disabled"
        )
        self.get_logger().info(
            f"GnssAnchoredPose: local={local_odom_topic}  gps={gps_topic}  "
            f"out={global_odom_topic}  {navsatfix_str}  reanchor={self._reanchor}  "
            f"map={self._map_frame}  odom={self._odom_frame}"
        )
        self.get_logger().info(f"Anchor gate: {gate_str}")
        if abs(yaw_offset_deg) > 1e-6:
            self.get_logger().info(
                f"Yaw correction: rotating Odometry+NavSatFix by {yaw_offset_deg:+.3f}° "
                "(CCW about +Z) — TF tree NOT rotated"
            )

        # Live tunability: react to runtime updates of yaw_offset_deg
        # (`ros2 param set /gnss_anchored_pose yaw_offset_deg <value>` or via
        # the Foxglove Parameters panel). Other parameters require a relaunch.
        self.add_on_set_parameters_callback(self._on_param_change)

    # ------------------------------------------------------------------

    def _update_yaw_cache(self, yaw_offset_deg: float) -> None:
        """Recompute cos/sin and the half-angle quaternion used for the
        constant rotation applied to position and orientation."""
        self._yaw_offset_rad = math.radians(yaw_offset_deg)
        self._cos_yaw = math.cos(self._yaw_offset_rad)
        self._sin_yaw = math.sin(self._yaw_offset_rad)
        self._q_yaw_w = math.cos(self._yaw_offset_rad / 2.0)
        self._q_yaw_z = math.sin(self._yaw_offset_rad / 2.0)

    def _on_param_change(self, params) -> SetParametersResult:
        """Apply runtime parameter updates. Currently only yaw_offset_deg is
        live-tunable; everything else still requires a relaunch."""
        for p in params:
            if p.name == "yaw_offset_deg":
                try:
                    new_val = float(p.value)
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason="yaw_offset_deg must be a number",
                    )
                self._update_yaw_cache(new_val)
                self.get_logger().info(
                    f"Yaw correction updated → {new_val:+.3f}° "
                    "(applies to next published Odometry/NavSatFix)"
                )
        return SetParametersResult(successful=True)

    def _on_local_odom(self, msg: Odometry) -> None:
        self._latest_local = msg
        if not self._anchored:
            return

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._map_frame
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist

        # Translate to ENU offset from datum.
        raw_x = msg.pose.pose.position.x + self._offset_x
        raw_y = msg.pose.pose.position.y + self._offset_y
        raw_z = msg.pose.pose.position.z + self._offset_z

        # Apply yaw correction (rotation about +Z around the anchor at origin).
        map_x = self._cos_yaw * raw_x - self._sin_yaw * raw_y
        map_y = self._sin_yaw * raw_x + self._cos_yaw * raw_y
        map_z = raw_z

        out.pose.pose.position.x = map_x
        out.pose.pose.position.y = map_y
        out.pose.pose.position.z = map_z

        # Apply same yaw correction to orientation: q_corrected = q_yaw ⊗ q_local
        if self._q_yaw_z != 0.0:
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            cz, sz = self._q_yaw_w, self._q_yaw_z
            out.pose.pose.orientation.w = cz * qw - sz * qz
            out.pose.pose.orientation.x = cz * qx - sz * qy
            out.pose.pose.orientation.y = cz * qy + sz * qx
            out.pose.pose.orientation.z = cz * qz + sz * qw

        self._global_pub.publish(out)

        if self._navsatfix_pub is not None and self._from_utm is not None:
            lon, lat = self._from_utm.transform(
                self._datum_e + map_x, self._datum_n + map_y
            )
            cov = msg.pose.covariance
            cov_e   = max(0.0, float(cov[0]))
            cov_n   = max(0.0, float(cov[7]))
            cov_alt = max(0.0, float(cov[14]))
            fix = NavSatFix()
            fix.header.stamp = msg.header.stamp
            fix.header.frame_id = self._map_frame
            fix.status.status   = NavSatStatus.STATUS_FIX
            fix.status.service  = NavSatStatus.SERVICE_GPS
            fix.latitude        = lat
            fix.longitude       = lon
            fix.altitude        = self._datum_alt + map_z
            fix.position_covariance = [
                cov_e, 0.0,   0.0,
                0.0,   cov_n, 0.0,
                0.0,   0.0,   cov_alt,
            ]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self._navsatfix_pub.publish(fix)

        if self._tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = self._map_frame
            tf.child_frame_id = self._odom_frame
            tf.transform.translation.x = self._offset_x
            tf.transform.translation.y = self._offset_y
            tf.transform.translation.z = self._offset_z
            tf.transform.rotation.w = 1.0
            self._tf_broadcaster.sendTransform(tf)

    def _on_hp_pos(self, msg) -> None:
        self._latest_h_acc_m = msg.h_acc * _HPPOSLLH_H_ACC_TO_M
        # Late-arriving h_acc can unblock anchoring if a fix was already cached.
        if not self._anchored and self._latest_fix is not None:
            self._try_anchor(self._latest_fix)

    def _on_gps(self, msg: NavSatFix) -> None:
        # Basic sanity (cheap, applies to all fixes for both anchor and re-anchor).
        if msg.status.status < 0:
            return
        if abs(msg.latitude) < 0.1 and abs(msg.longitude) < 0.1:
            return

        self._latest_fix = msg

        if not self._anchored:
            self._try_anchor(msg)
            return

        if not self._reanchor:
            return
        if self._latest_local is None:
            return

        lx = self._latest_local.pose.pose.position.x
        ly = self._latest_local.pose.pose.position.y
        lz = self._latest_local.pose.pose.position.z
        gx, gy, gz = self._fix_to_map(msg)
        new_off_x = gx - lx
        new_off_y = gy - ly
        new_off_z = gz - lz
        jump = math.hypot(new_off_x - self._offset_x, new_off_y - self._offset_y)
        self._offset_x = new_off_x
        self._offset_y = new_off_y
        self._offset_z = new_off_z
        self.get_logger().info(
            f"Re-anchored: jumped {jump:.2f} m → "
            f"offset=({self._offset_x:.2f}, {self._offset_y:.2f}, {self._offset_z:.2f})"
        )

    def _try_anchor(self, fix: NavSatFix) -> None:
        """Anchor only when local odom AND quality gate are both satisfied."""
        if self._latest_local is None:
            self.get_logger().info(
                "[anchor wait] /fix received but local EKF has not produced odometry yet",
                throttle_duration_sec=5.0,
            )
            return

        if self._h_acc_enabled:
            if self._latest_h_acc_m is None:
                self.get_logger().warn(
                    "[anchor wait] /fix received but no h_acc message yet on the "
                    "configured h_acc_topic — bag may not contain it. Pass "
                    "h_acc_topic:='' to disable the gate.",
                    throttle_duration_sec=5.0,
                )
                return
            if self._latest_h_acc_m <= 0.0 or self._latest_h_acc_m > self._h_acc_max_m:
                self.get_logger().info(
                    f"[anchor wait] fix lat={fix.latitude:.6f}° lon={fix.longitude:.6f}° "
                    f"rejected: h_acc={self._latest_h_acc_m * 100:.1f} cm "
                    f"> {self._h_acc_max_m * 100:.0f} cm",
                    throttle_duration_sec=5.0,
                )
                return

        lx = self._latest_local.pose.pose.position.x
        ly = self._latest_local.pose.pose.position.y
        lz = self._latest_local.pose.pose.position.z

        self._set_datum_from_fix(fix)
        self._offset_x = -lx
        self._offset_y = -ly
        self._offset_z = -lz
        self._anchored = True
        h_acc_str = (
            f", h_acc={self._latest_h_acc_m * 100:.1f} cm"
            if self._latest_h_acc_m is not None
            else ""
        )
        self.get_logger().info(
            f"Anchored: datum lat={fix.latitude:.7f}° lon={fix.longitude:.7f}° "
            f"alt={fix.altitude:.2f} m{h_acc_str} | "
            f"local at anchor=({lx:.2f}, {ly:.2f}, {lz:.2f}) | "
            f"offset=({self._offset_x:.2f}, {self._offset_y:.2f}, {self._offset_z:.2f})"
        )

    # ------------------------------------------------------------------

    def _set_datum_from_fix(self, msg: NavSatFix) -> None:
        epsg = _utm_epsg(msg.latitude, msg.longitude)
        self._to_utm = Transformer.from_crs("EPSG:4326", epsg, always_xy=True)
        self._from_utm = Transformer.from_crs(epsg, "EPSG:4326", always_xy=True)
        self._datum_e, self._datum_n = self._to_utm.transform(
            msg.longitude, msg.latitude
        )
        self._datum_alt = msg.altitude
        self._datum_set = True
        self.get_logger().info(
            f"Datum set → {epsg}  E={self._datum_e:.3f}  N={self._datum_n:.3f}  "
            f"alt={self._datum_alt:.2f} m"
        )

    def _fix_to_map(self, msg: NavSatFix) -> tuple[float, float, float]:
        e, n = self._to_utm.transform(msg.longitude, msg.latitude)
        return (e - self._datum_e, n - self._datum_n, msg.altitude - self._datum_alt)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GnssAnchoredPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
