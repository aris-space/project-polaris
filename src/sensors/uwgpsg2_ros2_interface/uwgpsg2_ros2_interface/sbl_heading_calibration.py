#!/usr/bin/env python3
"""SBL topside-heading auto-calibration (thesis: tightly-coupled SBL particle filter).

The Waterlinked UGPS topside heading (orientation of the receiver array vs North) is normally typed in
by hand in the GUI, so it is inaccurate. That heading rotates the SBL solution into North-referenced
coordinates, so a wrong value biases every georeferenced fix. This node replaces the manual entry using
the GNSS-calibrated IMU as the heading truth:

  1. Pre-checks: /imu/data_corrected is live AND the imu_yaw_correction node's yaw_offset_deg != 0
     (a real GNSS heading calibration happened); DVL present; SBL fixes arriving; box reachable.
  2. Drive ONE straight forward leg. Collect gated SBL filtered (x,y) + IMU yaw until >= min_distance_m
     driven (DVL dead-reckoning) or collect_timeout_s.
  3. Fit the SBL course (PCA / total-least-squares over the topside-body-frame points).
  4. True course from the IMU (ENU -> compass).
  5. corrected_heading = true_course - sbl_course; write it to the box's static_orientation
     (GET /api/v1/config/generic, require static mode, PUT full object), wait, and verify.

Frames:
  * /imu/data_corrected : sensor_msgs/Imu, ENU (yaw 0 = East, CCW, rad), true North (declination 0).
  * SBL filtered position (locator_position_relative_wrt_topside, geometry_msgs/Vector3Stamped):
    topside BODY frame, x = forward, y = right, z = down. Independent of the configured heading, so no
    lat/lon math is needed.
  * Box heading (config/generic.static_orientation, /position/master.orientation): degrees CW from North.

Run:
  ros2 run uwgpsg2_ros2_interface sbl_heading_calibration
  ros2 launch uwgpsg2_ros2_interface sbl_heading_calibration.launch.py
  # probe box schema only:  --ros-args -p probe:=true
  # compute, do not write:  --ros-args -p apply:=false
"""

import math
import time

import requests

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters

from .sbl_heading_math import (
    norm180,
    norm360,
    yaw_enu_from_quat,
    enu_yaw_to_compass_deg,
    circular_mean_std_deg,
    fit_course,
    corrected_heading,
)


class SblHeadingCalibration(Node):
    def __init__(self):
        super().__init__("sbl_heading_calibration")

        # --- parameters ---
        p = self.declare_parameter
        self.url = p("waterlinked_url", "http://192.168.194.94").value.rstrip("/")
        self.imu_topic = p("imu_corrected_topic", "/imu/data_corrected").value
        self.sbl_topic = p(
            "sbl_filtered_topic", "/waterlinked_ugps/locator_position_relative_wrt_topside"
        ).value
        self.quality_topic = p(
            "sbl_quality_topic", "/waterlinked_ugps/locator_acoustic_quality"
        ).value
        self.dvl_odom_topic = p("dvl_odom_topic", "/sensors/dvl/odometry").value
        self.yaw_node = p("imu_yaw_correction_node", "/imu_yaw_correction").value

        self.min_distance_m = float(p("min_distance_m", 3.0).value)
        self.collect_timeout_s = float(p("collect_timeout_s", 30.0).value)
        self.std_max = float(p("std_max", 5.0).value)
        self.min_fixes = int(p("min_fixes", 8).value)
        self.min_sbl_length_m = float(p("min_sbl_length_m", 2.0).value)
        self.straightness_ratio_max = float(p("straightness_ratio_max", 0.15).value)
        self.yaw_std_max_deg = float(p("yaw_std_max_deg", 8.0).value)
        self.verify_tol_deg = float(p("verify_tol_deg", 1.0).value)
        self.verify_wait_s = float(p("verify_wait_s", 5.0).value)
        self.http_timeout_s = float(p("http_timeout_s", 5.0).value)

        self.apply = bool(p("apply", True).value)
        self.probe = bool(p("probe", False).value)
        self.require_static_mode = bool(p("require_static_mode", True).value)
        self.heading_endpoint = p("heading_config_endpoint", "/api/v1/config/generic").value
        self.heading_key = p("heading_config_key", "static_orientation").value

        # --- state ---
        self._imu_yaw_deg = None          # latest IMU yaw (compass deg CW from N)
        self._imu_count = 0
        self._quality_std = None          # latest acoustic std [m]
        self._quality_valid = False       # latest position_valid flag
        self._dvl_pos = None              # latest DVL dead-reckoning (x,y)
        self._dvl_count = 0

        # collection buffers
        self._sbl_xy = []                 # gated topside-frame (x_fwd, y_right)
        self._leg_yaw_deg = []            # IMU compass yaw per accepted fix
        self._last_sbl_xy = None

        # --- subscriptions ---
        self.create_subscription(Imu, self.imu_topic, self._on_imu, 10)
        self.create_subscription(Vector3Stamped, self.quality_topic, self._on_quality, 10)
        self.create_subscription(Vector3Stamped, self.sbl_topic, self._on_sbl, 10)
        self.create_subscription(Odometry, self.dvl_odom_topic, self._on_dvl, 10)

    # ------------------------------------------------------------------ callbacks
    def _on_imu(self, msg):
        q = msg.orientation
        yaw_enu = yaw_enu_from_quat(q.x, q.y, q.z, q.w)
        self._imu_yaw_deg = enu_yaw_to_compass_deg(yaw_enu)  # ENU -> compass CW from N
        self._imu_count += 1

    def _on_quality(self, msg):
        # driver packs: vector.x = acoustic std [m] (-1 unknown), vector.y = 1 if position_valid
        self._quality_std = float(msg.vector.x)
        self._quality_valid = msg.vector.y >= 0.5

    def _on_dvl(self, msg):
        pos = msg.pose.pose.position
        self._dvl_pos = (pos.x, pos.y)
        self._dvl_count += 1

    def _on_sbl(self, msg):
        """Collect a filtered SBL fix (topside body frame x=forward, y=right), gated by quality."""
        if not getattr(self, "_collecting", False):
            return
        # Gate on acoustic quality when that stream is available; otherwise accept and rely on the
        # straightness / min-length gates applied after the fit (quality topic may be silent if the
        # driver isn't producing a global solution).
        if self._quality_std is not None:
            if not self._quality_valid:
                return
            if self._quality_std <= 0.0 or self._quality_std > self.std_max:
                return
        xy = (float(msg.vector.x), float(msg.vector.y))
        if self._last_sbl_xy is not None and math.dist(xy, self._last_sbl_xy) < 1e-6:
            return  # drop held/duplicate range
        self._last_sbl_xy = xy
        if self._imu_yaw_deg is None:
            return
        self._sbl_xy.append(xy)
        self._leg_yaw_deg.append(self._imu_yaw_deg)

    # ------------------------------------------------------------------ HTTP
    def _get_json(self, path):
        try:
            r = requests.get(self.url + path, timeout=self.http_timeout_s)
        except requests.exceptions.RequestException as exc:
            self.get_logger().error(f"GET {path} failed: {exc}")
            return None
        if r.status_code != 200:
            self.get_logger().error(f"GET {path} -> HTTP {r.status_code}: {r.text[:200]}")
            return None
        return r.json()

    def _put_json(self, path, obj):
        try:
            r = requests.put(self.url + path, json=obj, timeout=self.http_timeout_s)
        except requests.exceptions.RequestException as exc:
            self.get_logger().error(f"PUT {path} failed: {exc}")
            return False
        if r.status_code not in (200, 204):
            self.get_logger().error(f"PUT {path} -> HTTP {r.status_code}: {r.text[:200]}")
            return False
        return True

    def _get_yaw_offset_deg(self):
        """Read yaw_offset_deg from the imu_yaw_correction node via its GetParameters service."""
        srv = f"{self.yaw_node.rstrip('/')}/get_parameters"
        cli = self.create_client(GetParameters, srv)
        if not cli.wait_for_service(timeout_sec=3.0):
            return None  # node not up -> treat as uncalibrated
        req = GetParameters.Request()
        req.names = ["yaw_offset_deg"]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if not fut.done() or fut.result() is None or not fut.result().values:
            return None
        pv = fut.result().values[0]
        # PARAMETER_DOUBLE = 3
        return float(pv.double_value) if pv.type == 3 else None

    # ------------------------------------------------------------------ steps
    def _spin(self, duration_s):
        end = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _distance_travelled(self, start_xy):
        if self._dvl_pos is None or start_xy is None:
            return 0.0
        return math.dist(self._dvl_pos, start_xy)

    def run(self):
        print("\n=== SBL heading calibration ===")
        print(f"box: {self.url}   apply={self.apply}   probe={self.probe}\n")

        # --- probe: dump schema and exit ---
        if self.probe:
            print("-- PROBE: GET /api/v1/config/generic --")
            print(self._get_json("/api/v1/config/generic"))
            print("\n-- PROBE: GET /api/v1/position/master --")
            print(self._get_json("/api/v1/position/master"))
            return

        # --- pre-checks ---
        print("Pre-checks:")
        self._spin(2.0)  # let a few messages arrive

        ok = True
        if self._imu_count > 0:
            print(f"  [OK] {self.imu_topic} publishing (yaw={self._imu_yaw_deg:.1f} deg)")
        else:
            print(f"  [FAIL] no messages on {self.imu_topic}")
            ok = False

        yaw_off = self._get_yaw_offset_deg()
        if yaw_off is None:
            print(f"  [FAIL] could not read yaw_offset_deg from {self.yaw_node} "
                  "(imu_yaw_correction not running?)")
            ok = False
        elif abs(yaw_off) < 1e-6:
            print(f"  [FAIL] yaw_offset_deg = 0  ->  GNSS heading is NOT calibrated")
            ok = False
        else:
            print(f"  [OK] GNSS heading calibrated (yaw_offset_deg = {yaw_off:+.2f} deg)")

        if self._dvl_count > 0:
            print(f"  [OK] {self.dvl_odom_topic} publishing")
        else:
            print(f"  [FAIL] no messages on {self.dvl_odom_topic}")
            ok = False

        master = self._get_json("/api/v1/position/master")
        if master is not None:
            print(f"  [OK] box reachable (master orientation = "
                  f"{master.get('orientation')} deg)")
        else:
            print(f"  [FAIL] box not reachable at {self.url}")
            ok = False

        if not ok:
            print("\nABORT: pre-checks failed "
                  "(if GNSS-related: calibration failed due to uncalibrated GNSS).\n")
            return

        # --- collect one straight leg ---
        print(f"\nDrive a straight forward leg now. Collecting until "
              f"{self.min_distance_m:.1f} m or {self.collect_timeout_s:.0f} s...")
        self._sbl_xy.clear()
        self._leg_yaw_deg.clear()
        self._last_sbl_xy = None
        self._collecting = True
        start_xy = self._dvl_pos
        t0 = time.monotonic()
        last_print = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if start_xy is None:
                start_xy = self._dvl_pos
            dist = self._distance_travelled(start_xy)
            elapsed = time.monotonic() - t0
            if time.monotonic() - last_print > 0.5:
                _, yaw_sigma = (circular_mean_std_deg(self._leg_yaw_deg)
                                if self._leg_yaw_deg else (0.0, 0.0))
                print(f"  collecting... dist={dist:4.1f}/{self.min_distance_m:.1f} m  "
                      f"gated fixes={len(self._sbl_xy):3d}  IMU yaw sigma={yaw_sigma:4.1f} deg",
                      flush=True)
                last_print = time.monotonic()
            if dist >= self.min_distance_m or elapsed >= self.collect_timeout_s:
                break
        self._collecting = False
        dist = self._distance_travelled(start_xy)
        print(f"  collection done: {len(self._sbl_xy)} gated fixes over {dist:.1f} m DVL.\n")

        # --- fit + compute ---
        result = self._compute_heading()
        if result is None:
            return
        sbl_course, true_course, theta_correct, quality = result

        print("Result:")
        print(f"  SBL course (topside frame) = {sbl_course:6.1f} deg")
        print(f"  true course (IMU)          = {true_course:6.1f} deg")
        print(f"  corrected heading (target) = {theta_correct:6.1f} deg")
        print(f"  quality: fixes={quality['n']} length={quality['length']:.1f} m "
              f"straightness={quality['ratio']:.3f} yaw_sigma={quality['yaw_sigma']:.1f} deg")

        qbad = self._quality_reasons(quality)
        if qbad:
            print(f"\nABORT: leg quality insufficient ({'; '.join(qbad)}).\n")
            return

        # --- write + verify ---
        self._write_and_verify(theta_correct)

    def _compute_heading(self):
        n = len(self._sbl_xy)
        if n < 2:
            print("ABORT: no SBL fixes collected (position invalid / box not pinging).\n")
            return None
        gamma, length, ratio = fit_course(self._sbl_xy)
        true_course, yaw_sigma = circular_mean_std_deg(self._leg_yaw_deg)
        theta_correct = corrected_heading(true_course, gamma)
        quality = {"n": n, "length": length, "ratio": ratio, "yaw_sigma": yaw_sigma}
        return gamma, true_course, theta_correct, quality

    def _quality_reasons(self, q):
        bad = []
        if q["n"] < self.min_fixes:
            bad.append(f"only {q['n']} fixes (< {self.min_fixes})")
        if q["length"] < self.min_sbl_length_m:
            bad.append(f"SBL line {q['length']:.1f} m (< {self.min_sbl_length_m})")
        if q["ratio"] > self.straightness_ratio_max:
            bad.append(f"not straight (ratio {q['ratio']:.2f} > {self.straightness_ratio_max})")
        if q["yaw_sigma"] > self.yaw_std_max_deg:
            bad.append(f"IMU yaw unstable ({q['yaw_sigma']:.1f} deg > {self.yaw_std_max_deg})")
        return bad

    def _write_and_verify(self, theta_correct):
        cfg = self._get_json(self.heading_endpoint)
        if not isinstance(cfg, dict):
            print(f"\nABORT: could not GET {self.heading_endpoint}.\n")
            return
        if self.heading_key not in cfg:
            print(f"\nABORT: '{self.heading_key}' not in {self.heading_endpoint}. "
                  f"Full config for inspection:\n{cfg}\n")
            return

        gps, comp = cfg.get("gps"), cfg.get("compass")
        if self.require_static_mode and not (gps == "static" and comp == "static"):
            print(f"\nABORT: topside not in static mode (gps={gps!r}, compass={comp!r}). "
                  "Set GPS & Compass to 'Static' in the GUI, then re-run.\n")
            return

        h_old = float(cfg[self.heading_key])
        delta = norm180(theta_correct - h_old)
        h_new = norm360(h_old + delta)
        m_ori = (self._get_json("/api/v1/position/master") or {}).get("orientation")
        if m_ori is not None and abs(norm180(float(m_ori) - h_old)) > self.verify_tol_deg:
            print(f"  NOTE: config heading ({h_old:.1f}) != master orientation ({m_ori}) "
                  "- box may not be fully static.")

        print(f"\n  prior heading   = {h_old:6.1f} deg")
        print(f"  offset (delta)  = {delta:+6.1f} deg")
        print(f"  new heading     = {h_new:6.1f} deg")

        if not self.apply:
            print("\n  DRY-RUN (apply:=false): not writing. Re-run with apply:=true to calibrate.\n")
            return

        cfg[self.heading_key] = h_new
        print(f"\n  PUT {self.heading_endpoint} ({self.heading_key}={h_new:.1f})...")
        if not self._put_json(self.heading_endpoint, cfg):
            print("\n[FAILED] CALIBRATION FAILED (PUT rejected by box).\n")
            return

        print(f"  waiting {self.verify_wait_s:.0f} s, then verifying...")
        self._spin(self.verify_wait_s)

        got_cfg = (self._get_json(self.heading_endpoint) or {}).get(self.heading_key)
        got_master = (self._get_json("/api/v1/position/master") or {}).get("orientation")
        ok_cfg = got_cfg is not None and abs(norm180(float(got_cfg) - h_new)) <= self.verify_tol_deg
        ok_master = (got_master is not None
                     and abs(norm180(float(got_master) - h_new)) <= self.verify_tol_deg)
        print(f"  read back: config={got_cfg} master.orientation={got_master} (target {h_new:.1f})")
        if ok_cfg and ok_master:
            print("\n[SUCCESS] CALIBRATION SUCCESS (verified within tolerance).\n")
        elif ok_cfg:
            print("\n[PARTIAL] config written, but master.orientation did not update "
                  "(box may need a moment or is not fully static).\n")
        else:
            print("\n[FAILED] CALIBRATION FAILED (value did not persist).\n")


def main(args=None):
    rclpy.init(args=args)
    node = SblHeadingCalibration()
    try:
        node.run()
    except KeyboardInterrupt:
        print("\ninterrupted.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
