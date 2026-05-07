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
gps_antenna_offset_xyz  Body-frame [x, y, z] (metres) of the GPS antenna relative
                        to the AUV reference frame the local EKF tracks (default
                        [0, 0, 0]). When non-zero, the node compensates for the
                        antenna arcing around the centre during yaw rotations: the
                        published Odometry and NavSatFix represent the *antenna*
                        position, matching what raw /fix reports. Without this
                        compensation, on-the-spot yaw rotations produce a constant
                        map-frame translation error of magnitude up to
                        2 · |offset_xy| once the AUV has rotated 180°.
yaw_calibration_duration_s    Window length for service-triggered yaw
                              calibration (default 10 s).
yaw_calibration_min_distance_m  Minimum distance (m) the AUV must travel during
                                the calibration window for the result to be
                                accepted (default 3 m).

Service
-------
~/calibrate_yaw_offset (std_srvs/Trigger):
    Triggers a yaw-offset calibration. After the call returns, drive the AUV
    forward in a straight line. After yaw_calibration_duration_s, the node
    computes the GNSS bearing from the start fix to the latest fix, takes the
    circular average of the EKF yaw samples during the window, and applies
        yaw_offset_deg = bearing_GNSS - avg_yaw_EKF
    via the live parameter callback, so it takes effect on the next published
    message. Aborts (with a warn log) if the AUV moved less than
    yaw_calibration_min_distance_m.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from pyproj import Transformer
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_srvs.srv import Trigger
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


_EARTH_R_M = 6_371_000.0


def _gnss_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Haversine distance between two lat/lon points, in metres."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlon / 2.0) ** 2
    return 2.0 * _EARTH_R_M * math.asin(math.sqrt(a))


def _gnss_bearing_enu_rad(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial bearing from (lat1,lon1) to (lat2,lon2) in ENU yaw convention:
    0 rad = +x = East, +π/2 rad = +y = North, CCW positive."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    y = math.sin(dlon) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlon)
    bearing_ned = math.atan2(y, x)              # 0 = N, +π/2 = E (CW from N)
    yaw_enu = math.pi / 2.0 - bearing_ned       # convert to ENU yaw
    return math.atan2(math.sin(yaw_enu), math.cos(yaw_enu))


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
        self.declare_parameter("gps_antenna_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("yaw_calibration_duration_s", 10.0)
        self.declare_parameter("yaw_calibration_min_distance_m", 3.0)

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

        # GPS antenna body-frame offset (lever-arm). When non-zero, on-the-spot
        # yaw rotations would otherwise show up as a constant translation error
        # in the published track because /fix reports antenna position while the
        # local EKF tracks the AUV reference point.
        ant = self.get_parameter("gps_antenna_offset_xyz").value
        if not isinstance(ant, (list, tuple)) or len(ant) != 3:
            self.get_logger().warn(
                f"gps_antenna_offset_xyz must be a 3-element list; got {ant!r} — using [0,0,0]"
            )
            ant = [0.0, 0.0, 0.0]
        self._antenna_offset = (float(ant[0]), float(ant[1]), float(ant[2]))
        self._antenna_offset_used = any(abs(v) > 1e-6 for v in self._antenna_offset)
        self._yaw_at_anchor: float | None = None

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
        if self._antenna_offset_used:
            self.get_logger().info(
                f"GPS antenna body offset: ({self._antenna_offset[0]:+.3f}, "
                f"{self._antenna_offset[1]:+.3f}, {self._antenna_offset[2]:+.3f}) m — "
                "rotation-difference correction will be applied to published track"
            )

        # Live tunability: react to runtime updates of yaw_offset_deg
        # (`ros2 param set /gnss_anchored_pose yaw_offset_deg <value>` or via
        # the Foxglove Parameters panel). Other parameters require a relaunch.
        self.add_on_set_parameters_callback(self._on_param_change)

        # Service-triggered yaw calibration via a forward-driving maneuver.
        self._cal_active: bool = False
        self._cal_start_time = None
        self._cal_start_fix: NavSatFix | None = None
        self._cal_yaw_samples: list[float] = []
        self._cal_timer = None
        self.create_service(
            Trigger, "~/calibrate_yaw_offset", self._on_calibrate_request
        )
        self.get_logger().info(
            "Yaw calibration service: ~/calibrate_yaw_offset "
            f"(duration {float(self.get_parameter('yaw_calibration_duration_s').value):.1f}s, "
            f"min distance {float(self.get_parameter('yaw_calibration_min_distance_m').value):.2f}m)"
        )

    # ------------------------------------------------------------------

    def _update_yaw_cache(self, yaw_offset_deg: float) -> None:
        """Recompute cos/sin and the half-angle quaternion used for the
        constant rotation applied to position and orientation."""
        self._yaw_offset_rad = math.radians(yaw_offset_deg)
        self._cos_yaw = math.cos(self._yaw_offset_rad)
        self._sin_yaw = math.sin(self._yaw_offset_rad)
        self._q_yaw_w = math.cos(self._yaw_offset_rad / 2.0)
        self._q_yaw_z = math.sin(self._yaw_offset_rad / 2.0)

    @staticmethod
    def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
        """Yaw angle (radians) from a quaternion (z-axis ENU convention)."""
        return math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

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

    # ------------------------------------------------------------------
    # Service-triggered yaw calibration

    def _on_calibrate_request(self, request, response):
        duration = float(self.get_parameter("yaw_calibration_duration_s").value)
        if self._cal_active:
            response.success = False
            response.message = "Yaw calibration already in progress"
            return response
        if self._latest_fix is None:
            response.success = False
            response.message = "No /fix received yet"
            return response
        if self._latest_local is None:
            response.success = False
            response.message = "No local EKF odometry received yet"
            return response

        self._cal_active = True
        self._cal_start_time = self.get_clock().now()
        self._cal_start_fix = self._latest_fix
        self._cal_yaw_samples = []
        # Sample IMU yaw at 10 Hz throughout the window.
        self._cal_timer = self.create_timer(0.1, self._on_calibration_tick)

        response.success = True
        response.message = (
            f"Yaw calibration started — drive forward in a straight line for "
            f"{duration:.0f} s. New yaw_offset_deg will be applied automatically."
        )
        self.get_logger().info(response.message)
        return response

    def _on_calibration_tick(self) -> None:
        if not self._cal_active or self._cal_start_time is None:
            return
        duration = float(self.get_parameter("yaw_calibration_duration_s").value)
        elapsed_ns = (self.get_clock().now() - self._cal_start_time).nanoseconds
        elapsed_s = elapsed_ns / 1e9

        if self._latest_local is not None:
            oq = self._latest_local.pose.pose.orientation
            self._cal_yaw_samples.append(self._yaw_from_quat(oq.x, oq.y, oq.z, oq.w))

        if elapsed_s >= duration:
            self._finish_calibration()

    def _finish_calibration(self) -> None:
        if self._cal_timer is not None:
            self._cal_timer.cancel()
            self._cal_timer = None
        self._cal_active = False

        if self._cal_start_fix is None or self._latest_fix is None:
            self.get_logger().warn("Yaw calibration aborted — missing fixes")
            return
        if not self._cal_yaw_samples:
            self.get_logger().warn("Yaw calibration aborted — no IMU yaw samples")
            return

        end_fix = self._latest_fix
        distance = _gnss_distance_m(
            self._cal_start_fix.latitude, self._cal_start_fix.longitude,
            end_fix.latitude,             end_fix.longitude,
        )
        min_dist = float(self.get_parameter("yaw_calibration_min_distance_m").value)
        if distance < min_dist:
            self.get_logger().warn(
                f"Yaw calibration aborted — only moved {distance:.2f} m "
                f"(need ≥ {min_dist:.2f} m). Drive further next time."
            )
            return

        bearing = _gnss_bearing_enu_rad(
            self._cal_start_fix.latitude, self._cal_start_fix.longitude,
            end_fix.latitude,             end_fix.longitude,
        )
        # Circular average of the IMU yaw samples (handles wrap-around).
        sx = sum(math.sin(y) for y in self._cal_yaw_samples)
        cx = sum(math.cos(y) for y in self._cal_yaw_samples)
        avg_yaw = math.atan2(sx, cx)

        diff = bearing - avg_yaw
        diff_wrapped = math.atan2(math.sin(diff), math.cos(diff))
        new_offset_deg = math.degrees(diff_wrapped)

        # Apply via the parameter system; the on-set callback updates the cache.
        self.set_parameters([
            Parameter("yaw_offset_deg", Parameter.Type.DOUBLE, new_offset_deg)
        ])

        self.get_logger().info(
            f"Yaw calibration complete: distance={distance:.2f} m, "
            f"GNSS bearing(ENU)={math.degrees(bearing):+.3f}°, "
            f"avg IMU yaw={math.degrees(avg_yaw):+.3f}°  →  "
            f"yaw_offset_deg={new_offset_deg:+.3f}° (applied)"
        )

    # ------------------------------------------------------------------

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

        # Translate to ENU offset from datum (centre of the AUV reference frame).
        raw_x = msg.pose.pose.position.x + self._offset_x
        raw_y = msg.pose.pose.position.y + self._offset_y
        raw_z = msg.pose.pose.position.z + self._offset_z

        # GPS antenna lever-arm correction:
        # /fix represents the ANTENNA position; the local EKF tracks the AUV
        # reference. Without this correction, on-the-spot yaw rotations produce
        # a translation error of (R(yaw_now) - R(yaw_anchor)) · antenna_offset.
        if self._antenna_offset_used and self._yaw_at_anchor is not None:
            oq = msg.pose.pose.orientation
            yaw_now = self._yaw_from_quat(oq.x, oq.y, oq.z, oq.w)
            d_cos = math.cos(yaw_now)    - math.cos(self._yaw_at_anchor)
            d_sin = math.sin(yaw_now)    - math.sin(self._yaw_at_anchor)
            ax, ay, _az = self._antenna_offset
            raw_x += d_cos * ax - d_sin * ay
            raw_y += d_sin * ax + d_cos * ay
            # az currently ignored — pitch/roll lever-arm coupling is small for
            # an AUV that operates near level. Add full quaternion rotation if
            # this becomes important.

        # Apply user yaw correction (rotation about +Z around the anchor at origin).
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

        # Capture yaw at anchor moment for the antenna lever-arm correction.
        oq = self._latest_local.pose.pose.orientation
        self._yaw_at_anchor = self._yaw_from_quat(oq.x, oq.y, oq.z, oq.w)

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
