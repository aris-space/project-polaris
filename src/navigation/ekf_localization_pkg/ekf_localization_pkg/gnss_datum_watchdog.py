import math
import signal
import subprocess
import tempfile
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType, ParameterValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

_DEG2RAD = math.pi / 180.0
_EARTH_R_M = 6_371_000.0


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dlat = (lat2 - lat1) * _DEG2RAD
    dlon = (lon2 - lon1) * _DEG2RAD
    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1 * _DEG2RAD) * math.cos(lat2 * _DEG2RAD) * math.sin(dlon / 2) ** 2
    )
    return 2.0 * _EARTH_R_M * math.asin(math.sqrt(a))

try:
    from ublox_ubx_msgs.msg import UBXNavHPPosLLH as _UBXNavHPPosLLH
    _HPPOSLLH_AVAILABLE = True
except ImportError:
    _UBXNavHPPosLLH = None
    _HPPOSLLH_AVAILABLE = False

# UBX-NAV-HPPOSLLH h_acc field is in 0.1 mm units
_HPPOSLLH_H_ACC_TO_M = 1e-4


class GnssDatumWatchdog(Node):
    """
    Waits for a quality-gated GNSS fix, then spawns navsat_transform_node and
    ekf_global_node via navsat_global_ekf.launch.py with that fix as the datum.

    This guarantees navsat_transform never starts at null-island (0°, 0°): the
    GPS-dependent nodes are only launched once a valid position is confirmed.
    The local EKF (IMU + DVL + pressure) runs from system start, independently.

    Quality gate (both must pass when h_acc topic is available):
      - NavSatFix.status >= 0   (valid fix, not STATUS_NO_FIX)
      - |lat| > 0.1° and |lon| > 0.1°   (null-island guard)
      - UBX-NAV-HPPOSLLH h_acc > 0 and <= h_acc_max_m

    Parameters
    ----------
    fix_topic            NavSatFix topic                      (default /gps/selected)
    h_acc_topic          UBXNavHPPosLLH accuracy topic        (default /ubx_nav_hp_pos_llh)
    h_acc_max_m          Max acceptable horizontal accuracy   (default 0.50 m)
    max_fix_distance_m   Max haversine distance from datum    (default 100000 m = 100 km)
    imu_topic            IMU topic forwarded to navsat        (default /imu/data_corrected)
    odom_topic           Local odom forwarded to navsat       (default /odometry/filtered/local)
    navsat_params_file   Base navsat_transform YAML path      (default: package navsat_transform.yaml)
    global_ekf_params_file  Global EKF YAML path             (default: package ekf_global.yaml)
    use_global_ekf       Also launch ekf_global_node         (default true)

    After the datum is set, every incoming fix is validated by:
      - status >= 0 and |lat| > 0.1° (null-island guard)
      - haversine distance from datum <= max_fix_distance_m
    Valid fixes are republished on /gps/validated; navsat_transform subscribes
    to that topic instead of the raw fix topic so garbage fixes never reach it.
    """

    def __init__(self) -> None:
        super().__init__("gnss_datum_watchdog")

        self.declare_parameter("fix_topic", "/gps/selected")
        self.declare_parameter("h_acc_topic", "/ubx_nav_hp_pos_llh")
        self.declare_parameter("h_acc_max_m", 0.50)
        self.declare_parameter("max_fix_distance_m", 100_000.0)
        self.declare_parameter("imu_topic", "/imu/data_corrected")
        self.declare_parameter("odom_topic", "/odometry/filtered/local")
        self.declare_parameter("navsat_params_file", "")
        self.declare_parameter("global_ekf_params_file", "")
        self.declare_parameter("use_global_ekf", True)
        # ── Bootstrap: publish a one-shot PoseWithCovarianceStamped to
        # /ekf_global_node/set_pose at watchdog spawn time, so the global EKF
        # starts from a known-correct state (datum origin, local-EKF orientation)
        # instead of cold-starting at (0,0,0,...) and converging through bad
        # TF lookups during the first second.
        self.declare_parameter("bootstrap_set_pose", True)
        self.declare_parameter("bootstrap_set_pose_topic", "/ekf_global_node/set_pose")
        self.declare_parameter("bootstrap_delay_s", 12.0)
        self.declare_parameter("bootstrap_retry_delay_s", 2.0)
        self.declare_parameter("bootstrap_pose_cov_xy_m2", 0.01)
        self.declare_parameter("bootstrap_pose_cov_z_m2", 0.01)
        self.declare_parameter("bootstrap_pose_cov_rpy_rad2", 0.001)
        # Legacy: grace period for the subprocess-spawn path. Only used
        # when use_inplace_global_stack=False.
        self.declare_parameter("spawn_grace_period_s", 10.0)
        # Architectural switch — True (default) skips subprocess.Popen and
        # pushes datum/anchor via set_parameters to already-running nodes
        # in the main launch. ~1-2 s warmup instead of 30-50 s.
        self.declare_parameter("use_inplace_global_stack", True)
        self.declare_parameter("gps_to_map_node_name", "gps_to_map_position")
        self.declare_parameter("global_ekf_to_navsatfix_node_name", "global_ekf_to_navsatfix_node")
        self.declare_parameter("pressure_pose_frame_fix_node_name", "pressure_pose_frame_fix")

        fix_topic: str = self.get_parameter("fix_topic").value
        h_acc_topic: str = self.get_parameter("h_acc_topic").value
        self._h_acc_max_m: float = self.get_parameter("h_acc_max_m").value
        self._max_fix_distance_m: float = self.get_parameter("max_fix_distance_m").value
        self._imu_topic: str = self.get_parameter("imu_topic").value
        self._odom_topic: str = self.get_parameter("odom_topic").value
        self._navsat_params: str = self.get_parameter("navsat_params_file").value
        self._global_ekf_params: str = self.get_parameter("global_ekf_params_file").value
        self._use_global_ekf: bool = bool(self.get_parameter("use_global_ekf").value)
        self._fix_topic = fix_topic
        self._use_sim_time: bool = bool(self.get_parameter("use_sim_time").value)

        self._bootstrap_set_pose: bool = bool(
            self.get_parameter("bootstrap_set_pose").value
        )
        self._bootstrap_set_pose_topic: str = str(
            self.get_parameter("bootstrap_set_pose_topic").value
        )
        self._bootstrap_delay_s: float = float(
            self.get_parameter("bootstrap_delay_s").value
        )
        self._bootstrap_retry_delay_s: float = float(
            self.get_parameter("bootstrap_retry_delay_s").value
        )
        self._spawn_grace_period_s: float = float(
            self.get_parameter("spawn_grace_period_s").value
        )
        self._use_inplace_global_stack: bool = bool(
            self.get_parameter("use_inplace_global_stack").value
        )
        self._gps_to_map_node: str = str(
            self.get_parameter("gps_to_map_node_name").value
        )
        self._global_ekf_to_navsatfix_node: str = str(
            self.get_parameter("global_ekf_to_navsatfix_node_name").value
        )
        self._pressure_pose_frame_fix_node: str = str(
            self.get_parameter("pressure_pose_frame_fix_node_name").value
        )
        self._bootstrap_cov_xy: float = float(
            self.get_parameter("bootstrap_pose_cov_xy_m2").value
        )
        self._bootstrap_cov_z: float = float(
            self.get_parameter("bootstrap_pose_cov_z_m2").value
        )
        self._bootstrap_cov_rpy: float = float(
            self.get_parameter("bootstrap_pose_cov_rpy_rad2").value
        )
        self._bootstrap_timer = None
        self._bootstrap_pub = None
        # Local-EKF position at the instant of datum-lock. Captured in
        # _spawn_navsat_and_global so that the bootstrap publish can compute
        # (current_local − anchor_local) instead of bootstrapping at a fixed
        # (0, 0). This lets us bootstrap the EKF at the boat's actual current
        # position in map frame, even if the boat has moved during the 12 s
        # between datum-lock and bootstrap publish.
        self._local_anchor_x: float = 0.0
        self._local_anchor_y: float = 0.0
        self._local_anchor_z: float = 0.0
        self._local_anchor_yaw: float = 0.0
        self._local_anchor_valid: bool = False

        self._latest_fix: NavSatFix | None = None
        self._latest_h_acc_m: float | None = None
        self._launched = False
        self._datum_lat: float | None = None
        self._datum_lon: float | None = None
        # Onset-tracking: log the first time global EKF magnitude crosses each
        # threshold so we can pin down when divergence starts (and what local
        # EKF reported at that moment).
        self._diag_levels_hit: dict[float, bool] = {}
        # Onset-tracking: stamp at first crossing for the [DIAG-summary] report.
        self._diag_levels_t: dict[float, float] = {}

        # Counters for the [DIAG-summary] log line.
        self._n_gps_received = 0
        self._n_gps_validated = 0
        self._n_gps_rejected_status = 0
        self._n_gps_rejected_null = 0
        self._n_gps_rejected_haversine = 0
        self._t_node_start: float = self.get_clock().now().nanoseconds * 1e-9
        self._t_lock_s: float | None = None

        # Validated GPS fixes forwarded to navsat_transform (replaces raw topic).
        self._validated_pub = self.create_publisher(
            NavSatFix, "/gps/validated", qos_profile_sensor_data
        )

        # Diagnostic monitors — warn the moment a position exceeds 10 km (way beyond
        # any lake test) to identify whether GPS or global EKF causes the crash.
        self._diag_warn_m = 10_000.0
        self._diag_odom_gps_sub = self.create_subscription(
            Odometry, "/odometry/gps", self._on_diag_odom_gps, 10
        )
        self._diag_global_sub = self.create_subscription(
            Odometry, "/odometry/filtered/global", self._on_diag_global_odom, 10
        )

        # Cache latest local EKF odom so we can log its position at datum-set time.
        self._latest_local_odom: Odometry | None = None
        self._diag_local_sub = self.create_subscription(
            Odometry, self._odom_topic, self._on_local_odom, 10
        )

        self._fix_sub = self.create_subscription(
            NavSatFix, fix_topic, self._on_fix, qos_profile_sensor_data
        )

        if h_acc_topic and _HPPOSLLH_AVAILABLE:
            self._h_acc_sub = self.create_subscription(
                _UBXNavHPPosLLH,
                h_acc_topic,
                self._on_hp_pos,
                qos_profile_sensor_data,
            )
        else:
            self._h_acc_sub = None
            if h_acc_topic and not _HPPOSLLH_AVAILABLE:
                self.get_logger().warn(
                    "ublox_ubx_msgs not available — h_acc quality gate disabled, "
                    "falling back to coordinate + status check only."
                )

        self.get_logger().info(
            f"Waiting for RTK fix on '{fix_topic}' "
            f"(h_acc ≤ {self._h_acc_max_m * 100:.0f} cm via '{h_acc_topic}') "
            f"before launching navsat_transform + global EKF"
        )

        # Periodic [DIAG-summary] every 60 s so a long replay log has periodic
        # checkpoints, not just an end-of-run summary.
        self._summary_timer = self.create_timer(60.0, self._log_diag_summary)

    # ------------------------------------------------------------------

    def _on_fix(self, msg: NavSatFix) -> None:
        self._n_gps_received += 1
        if self._launched:
            self._validate_and_republish(msg)
            return
        if msg.status.status < 0:
            self._n_gps_rejected_status += 1
            return
        self._latest_fix = msg
        self._check_and_launch()

    def _on_hp_pos(self, msg: _UBXNavHPPosLLH) -> None:
        self._latest_h_acc_m = msg.h_acc * _HPPOSLLH_H_ACC_TO_M
        if not self._launched:
            self._check_and_launch()

    def _check_and_launch(self) -> None:
        if self._launched or self._latest_fix is None:
            return

        # Reject coordinates within ~11 km of null island.
        fix = self._latest_fix
        if abs(fix.latitude) < 0.1 and abs(fix.longitude) < 0.1:
            return

        # h_acc gate
        if self._h_acc_sub is not None:
            if self._latest_h_acc_m is None:
                return
            if self._latest_h_acc_m <= 0.0 or self._latest_h_acc_m > self._h_acc_max_m:
                return

        self._launched = True
        self._spawn_navsat_and_global(fix)

    def _validate_and_republish(self, msg: NavSatFix) -> None:
        # Log every incoming fix so we can see exactly what arrives near the crash.
        self.get_logger().info(
            f"[GPS raw] status={msg.status.status} "
            f"lat={msg.latitude:.6f}° lon={msg.longitude:.6f}°",
            throttle_duration_sec=5.0,
        )
        if msg.status.status < 0:
            self._n_gps_rejected_status += 1
            self.get_logger().warn(
                f"[GPS reject] status={msg.status.status} "
                f"lat={msg.latitude:.6f}° lon={msg.longitude:.6f}° — bad fix status"
            )
            return
        if abs(msg.latitude) < 0.1 and abs(msg.longitude) < 0.1:
            self._n_gps_rejected_null += 1
            self.get_logger().warn(
                f"[GPS reject] lat={msg.latitude:.6f}° lon={msg.longitude:.6f}° — null island"
            )
            return
        if self._datum_lat is not None:
            dist = _haversine_m(
                self._datum_lat, self._datum_lon, msg.latitude, msg.longitude
            )
            if dist > self._max_fix_distance_m:
                self._n_gps_rejected_haversine += 1
                self.get_logger().warn(
                    f"[GPS reject] lat={msg.latitude:.6f}° lon={msg.longitude:.6f}° "
                    f"dist={dist / 1000.0:.1f} km from datum "
                    f"(max {self._max_fix_distance_m / 1000.0:.0f} km) — "
                    "likely corrupt/mislabeled NavSatFix; not forwarded to navsat_transform",
                )
                return
            self.get_logger().info(
                f"[GPS valid] lat={msg.latitude:.6f}° lon={msg.longitude:.6f}° "
                f"dist={dist:.0f} m from datum — forwarding to navsat_transform",
                throttle_duration_sec=10.0,
            )
        self._n_gps_validated += 1
        self._validated_pub.publish(msg)

    def _on_local_odom(self, msg: Odometry) -> None:
        self._latest_local_odom = msg

    def _on_diag_odom_gps(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(
            f"[GPS odom] x={x:.2f} m  y={y:.2f} m — navsat_transform → /odometry/gps",
            throttle_duration_sec=5.0,
        )
        if abs(x) > self._diag_warn_m or abs(y) > self._diag_warn_m:
            self.get_logger().warn(
                f"[DIAG] /odometry/gps out of range: "
                f"x={x / 1000.0:.2f} km, y={y / 1000.0:.2f} km — "
                "navsat_transform computed a far GPS odometry; crash imminent. "
                "A bad GPS fix likely slipped through the gateway.",
            )

    def _on_diag_global_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        r = (x * x + y * y) ** 0.5
        for level in (100.0, 1_000.0, 10_000.0):
            if r >= level and not self._diag_levels_hit.get(level, False):
                self._diag_levels_hit[level] = True
                t_now = self.get_clock().now().nanoseconds * 1e-9
                self._diag_levels_t[level] = t_now - self._t_node_start
                lx, ly = (None, None)
                if self._latest_local_odom is not None:
                    lx = self._latest_local_odom.pose.pose.position.x
                    ly = self._latest_local_odom.pose.pose.position.y
                self.get_logger().warn(
                    f"[DIAG-onset] /odometry/filtered/global crossed {level:.0f} m: "
                    f"global=({x:.1f}, {y:.1f}), local=({lx}, {ly}) — "
                    "first crossing of this level"
                )
        if abs(x) > self._diag_warn_m or abs(y) > self._diag_warn_m:
            self.get_logger().warn(
                f"[DIAG] /odometry/filtered/global out of range: "
                f"x={x / 1000.0:.2f} km, y={y / 1000.0:.2f} km — "
                "global EKF diverged; publish_filtered_gps crash imminent. "
                "Check Q values for unsensored states.",
            )

    # ------------------------------------------------------------------

    def _log_diag_summary(self) -> None:
        t_now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = t_now - self._t_node_start
        datum = (
            f"({self._datum_lat:.7f}, {self._datum_lon:.7f})"
            if self._datum_lat is not None
            else "unset"
        )
        time_to_lock = (
            f"{self._t_lock_s:.1f}s" if self._t_lock_s is not None else "not yet"
        )
        thresholds = (
            ", ".join(
                f"{int(level)}m@{self._diag_levels_t[level]:.1f}s"
                for level in sorted(self._diag_levels_hit)
                if self._diag_levels_hit.get(level)
            )
            or "none"
        )
        self.get_logger().info(
            f"[DIAG-summary] elapsed={elapsed:.1f}s "
            f"datum={datum} time_to_lock={time_to_lock} "
            f"gps received={self._n_gps_received} "
            f"validated={self._n_gps_validated} "
            f"rejected(status/null/haversine)="
            f"{self._n_gps_rejected_status}/"
            f"{self._n_gps_rejected_null}/"
            f"{self._n_gps_rejected_haversine} "
            f"diag_thresholds_hit=[{thresholds}]"
        )

    def _spawn_navsat_and_global(self, fix: NavSatFix) -> None:
        self._datum_lat = fix.latitude
        self._datum_lon = fix.longitude
        t_now = self.get_clock().now().nanoseconds * 1e-9
        self._t_lock_s = t_now - self._t_node_start

        h_acc_str = (
            f", h_acc={self._latest_h_acc_m * 100:.1f} cm"
            if self._latest_h_acc_m is not None
            else ""
        )
        local_pos = ""
        if self._latest_local_odom is not None:
            lx = self._latest_local_odom.pose.pose.position.x
            ly = self._latest_local_odom.pose.pose.position.y
            lz = self._latest_local_odom.pose.pose.position.z
            # Local EKF yaw at lock time. Needed by global_ekf_to_navsatfix
            # to rotate state.(x, y) from map frame back to UTM ENU before
            # back-projecting to lat/lon — otherwise /gps/filtered/global
            # ends up heading-rotated relative to /fix by exactly this angle.
            q = self._latest_local_odom.pose.pose.orientation
            lyaw = math.atan2(
                2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y)),
                1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z)),
            )
            local_pos = (
                f" | local EKF odom at datum-set: x={lx:.2f} m  y={ly:.2f} m  "
                f"yaw={math.degrees(lyaw):+.3f}°"
            )
            # Snapshot for bootstrap delta computation. Storing primitives,
            # not the message reference, so the values are immune to
            # subsequent _on_local_odom updates of self._latest_local_odom.
            self._local_anchor_x = float(lx)
            self._local_anchor_y = float(ly)
            self._local_anchor_z = float(lz)
            self._local_anchor_yaw = float(lyaw)
            self._local_anchor_valid = True
        self.get_logger().info(
            f"Valid fix: lat={fix.latitude:.7f}° lon={fix.longitude:.7f}° "
            f"alt={fix.altitude:.1f} m{h_acc_str} — spawning navsat_transform + global EKF"
            f"{local_pos}"
        )

        if self._use_inplace_global_stack:
            self._push_datum_to_inplace_nodes(fix)
            self.get_logger().info(
                "datum pushed to in-place global-stack nodes via set_parameters."
            )
            return self._schedule_bootstrap()

        # ── LEGACY subprocess.Popen path ──────────────────────────────────
        # Write a minimal YAML with just the datum; loaded second in the launch
        # so it overrides any placeholder datum in the base navsat params file.
        datum_yaml = (
            "navsat_transform_node:\n"
            "  ros__parameters:\n"
            f"    datum: [{fix.latitude}, {fix.longitude}, {fix.altitude}]\n"
        )
        datum_file = Path(tempfile.mkdtemp()) / "gnss_datum.yaml"
        datum_file.write_text(datum_yaml)

        cmd = [
            "ros2", "launch", "ekf_localization_pkg", "navsat_global_ekf.launch.py",
            f"datum_yaml:={datum_file}",
            f"datum_lat:={fix.latitude}",
            f"datum_lon:={fix.longitude}",
            f"datum_alt:={fix.altitude}",
            f"gps_fix_topic:=/gps/validated",
            f"imu_topic:={self._imu_topic}",
            f"odom_topic:={self._odom_topic}",
            f"use_global_ekf:={'true' if self._use_global_ekf else 'false'}",
            f"use_sim_time:={'true' if self._use_sim_time else 'false'}",
            # local_anchor: the local EKF's position at this instant. Map
            # frame's origin is the GPS datum, which corresponds to this
            # local-EKF position in odom frame. gps_odom_cov_floor
            # subtracts these from /odometry/gps to convert from odom
            # frame (where navsat publishes) to map frame (where the
            # global EKF lives). Falls back to (0,0,0) if the local odom
            # subscription hasn't delivered a message yet — that case
            # produces a misaligned but stable global track, easier to
            # debug than the alternative TF-feedback divergence.
            f"local_anchor_x:={self._local_anchor_x}",
            f"local_anchor_y:={self._local_anchor_y}",
            f"local_anchor_z:={self._local_anchor_z}",
            f"local_anchor_yaw:={self._local_anchor_yaw}",
        ]
        if self._navsat_params:
            cmd.append(f"navsat_params_file:={self._navsat_params}")
        if self._global_ekf_params:
            cmd.append(f"global_ekf_params_file:={self._global_ekf_params}")

        subprocess.Popen(cmd)
        self.get_logger().info(
            "navsat_transform_node + ekf_global_node launched with precise datum."
        )

        self._schedule_bootstrap()

    def _schedule_bootstrap(self) -> None:
        """Schedule the one-shot set_pose publish to bootstrap ekf_global_node."""
        if not (self._bootstrap_set_pose and self._use_global_ekf):
            return
        if self._bootstrap_pub is None:
            self._bootstrap_pub = self.create_publisher(
                PoseWithCovarianceStamped,
                self._bootstrap_set_pose_topic,
                10,
            )
        self._bootstrap_timer = self.create_timer(
            self._bootstrap_delay_s,
            self._publish_bootstrap_set_pose,
        )
        self.get_logger().info(
            f"[bootstrap] set_pose scheduled for {self._bootstrap_delay_s:.1f}s "
            f"on {self._bootstrap_set_pose_topic}"
        )

    def _push_datum_to_inplace_nodes(self, fix: NavSatFix) -> None:
        """Push datum + anchor to already-running global-stack nodes via
        set_parameters service calls. Fires the calls fire-and-forget;
        we don't block on the responses because each callback runs in
        the same DDS network and is essentially instantaneous once
        services are discovered. If a service isn't ready yet
        (shouldn't happen — main launch starts all nodes at T=0), the
        call fails and is logged."""
        # datum_lat/lon/alt → gps_to_map_position + global_ekf_to_navsatfix
        datum_params = [
            ("datum_lat", float(fix.latitude)),
            ("datum_lon", float(fix.longitude)),
            ("datum_alt", float(fix.altitude)),
        ]
        self._send_set_parameters(
            f"{self._gps_to_map_node}/set_parameters", datum_params
        )
        self._send_set_parameters(
            f"{self._global_ekf_to_navsatfix_node}/set_parameters",
            datum_params,
        )
        # local_anchor_z → pressure_pose_frame_fix
        self._send_set_parameters(
            f"{self._pressure_pose_frame_fix_node}/set_parameters",
            [("local_anchor_z", float(self._local_anchor_z))],
        )

    def _send_set_parameters(self, service_name: str, kv_pairs: list) -> None:
        client = self.create_client(SetParameters, service_name)
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                f"[set_parameters] service '{service_name}' not available "
                "after 5 s — target node may not be running. Skipping."
            )
            return
        request = SetParameters.Request()
        for name, value in kv_pairs:
            request.parameters.append(
                Parameter(name=name, value=value).to_parameter_msg()
            )
        future = client.call_async(request)
        future.add_done_callback(
            lambda f, svc=service_name, kv=kv_pairs: self._on_set_param_done(
                f, svc, kv
            )
        )

    def _on_set_param_done(self, future, service_name: str, kv_pairs: list) -> None:
        try:
            response = future.result()
            for kv, result in zip(kv_pairs, response.results):
                if not result.successful:
                    self.get_logger().warn(
                        f"[set_parameters] {service_name} {kv[0]}={kv[1]}: "
                        f"failed: {result.reason}"
                    )
            self.get_logger().info(
                f"[set_parameters] {service_name} accepted "
                f"{len(kv_pairs)} parameter(s)"
            )
        except Exception as e:
            self.get_logger().warn(
                f"[set_parameters] {service_name}: call failed: {e}"
            )

    def _publish_bootstrap_set_pose(self) -> None:
        # Cancel the firing timer before we either publish or reschedule.
        if self._bootstrap_timer is not None:
            self._bootstrap_timer.cancel()
            self._bootstrap_timer = None

        if self._latest_local_odom is None:
            self.get_logger().warn(
                "[bootstrap] no /odometry/filtered/local seen yet — skipping set_pose. "
                "Global EKF will cold-start from (0,0,0,...)."
            )
            return
        if self._bootstrap_pub is None:
            return  # publisher not created (paranoia)

        # Wait for the EKF's set_pose subscriber to be discovered before
        # publishing. robot_localization's set_pose subscriber is RELIABLE
        # + VOLATILE; if we publish before it's registered, the message is
        # silently dropped (no late-joiner buffering). The launch's
        # TimerAction(period=10.0) doesn't tell us when the EKF process is
        # actually ready to receive — observed lag was ~25–40 s on the
        # Jetson, far longer than our default 12 s bootstrap delay. So
        # check subscriber count and retry.
        n_subs = self._bootstrap_pub.get_subscription_count()
        if n_subs == 0:
            self._bootstrap_retries = getattr(self, "_bootstrap_retries", 0) + 1
            max_retries = 60  # × 2 s polling = 120 s total wait, plenty
            if self._bootstrap_retries > max_retries:
                self.get_logger().error(
                    f"[bootstrap] gave up after {self._bootstrap_retries} retries — "
                    f"no subscriber on {self._bootstrap_set_pose_topic} ever appeared. "
                    "Global EKF cold-started; expect divergence on long bags."
                )
                return
            retry_delay = 2.0
            self.get_logger().info(
                f"[bootstrap] no subscriber on {self._bootstrap_set_pose_topic} yet "
                f"(retry {self._bootstrap_retries}/{max_retries}); polling again in "
                f"{retry_delay:.1f} s"
            )
            self._bootstrap_timer = self.create_timer(
                retry_delay, self._publish_bootstrap_set_pose
            )
            return

        local = self._latest_local_odom
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        # Position in map frame:
        #   - x, y: the boat's CURRENT displacement from the datum, computed
        #     as (local_now - local_at_anchor). This matches what
        #     gnss_anchored_pose would publish for the same instant. A
        #     fixed (0, 0) bootstrap would be wrong by however far the boat
        #     has moved during the ~12 s between datum-lock and this
        #     publish — small for a near-stationary boat, but a real bug
        #     for a moving one.
        #   - z: absolute local z (pressure-fused depth from water surface).
        #     The global EKF doesn't fuse z from any source other than
        #     predict, so the bootstrap z stays the live reference.
        if self._local_anchor_valid:
            dx = float(local.pose.pose.position.x) - self._local_anchor_x
            dy = float(local.pose.pose.position.y) - self._local_anchor_y
        else:
            # Fallback to (0, 0) if for some reason the anchor wasn't captured
            # (shouldn't happen — _spawn_navsat_and_global captures it).
            dx = 0.0
            dy = 0.0
        msg.pose.pose.position.x = dx
        msg.pose.pose.position.y = dy
        msg.pose.pose.position.z = float(local.pose.pose.position.z)
        # Orientation: take the local EKF's converged orientation. By
        # construction the map and odom frames share their yaw axis (the
        # static identity bootstrap inside navsat_global_ekf.launch.py keeps
        # them parallel) so the local-EKF orientation expressed in odom
        # frame is also valid in map frame.
        msg.pose.pose.orientation.x = float(local.pose.pose.orientation.x)
        msg.pose.pose.orientation.y = float(local.pose.pose.orientation.y)
        msg.pose.pose.orientation.z = float(local.pose.pose.orientation.z)
        msg.pose.pose.orientation.w = float(local.pose.pose.orientation.w)
        # Covariance: 6x6 row-major diagonal. Tight on every component to tell
        # the EKF "trust this pose, reset to it." Off-diagonals zero.
        cov = [0.0] * 36
        cov[0]  = self._bootstrap_cov_xy   # x
        cov[7]  = self._bootstrap_cov_xy   # y
        cov[14] = self._bootstrap_cov_z    # z
        cov[21] = self._bootstrap_cov_rpy  # roll
        cov[28] = self._bootstrap_cov_rpy  # pitch
        cov[35] = self._bootstrap_cov_rpy  # yaw
        msg.pose.covariance = cov

        # Compute yaw for the log line.
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw_rad = math.atan2(2.0 * (qw * qz + qx * qy),
                             1.0 - 2.0 * (qy * qy + qz * qz))
        yaw_deg = math.degrees(yaw_rad)

        retries = getattr(self, "_bootstrap_retries", 0)
        self._bootstrap_pub.publish(msg)
        self.get_logger().info(
            f"[bootstrap] Published set_pose to ekf_global_node "
            f"(after {retries} retries, {n_subs} subscriber(s) confirmed): "
            f"pos=({msg.pose.pose.position.x:+.2f}, "
            f"{msg.pose.pose.position.y:+.2f}, "
            f"{msg.pose.pose.position.z:+.2f}) "
            f"(delta from anchor: dx={msg.pose.pose.position.x:+.2f} m  "
            f"dy={msg.pose.pose.position.y:+.2f} m) "
            f"yaw={yaw_deg:+.2f} deg "
            f"cov_xy={self._bootstrap_cov_xy:.4f} cov_rpy={self._bootstrap_cov_rpy:.4f}"
        )

    def destroy_node(self) -> bool:
        try:
            self._log_diag_summary()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GnssDatumWatchdog()

    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_sigterm)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
