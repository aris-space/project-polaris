#!/usr/bin/env python3
"""Calibrate the Waterlinked topside POSITION from the vehicle's GNSS RTK (thesis).

Procedure: manually heave the vehicle so its GNSS antenna is directly above the topside, then run this.
It reads the vehicle's RTK lat/lon, writes it to the box as the topside static position
(config/generic.static_lat/static_lon, static mode), and verifies via /position/master.

    GET  /api/v1/config/generic           -> require gps==compass=="static", read prior static_lat/lon
    (collect a short RTK-fixed window from /fix, gated by /ubx_nav_pvt + /ubx_nav_hp_pos_llh)
    PUT  /api/v1/config/generic (full object, static_lat/lon = mean)
    GET  /api/v1/position/master          -> verify lat/lon within tolerance

Run:
    ros2 launch uwgpsg2_ros2_interface sbl_position_calibration.launch.py                 # vehicle (RTK)
    ros2 launch uwgpsg2_ros2_interface sbl_position_calibration_laptop.launch.py sim_lat:=47.321 sim_lon:=8.554
    ros2 run uwgpsg2_ros2_interface sbl_position_calibration
"""

import time

import requests

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import NavSatFix
from ublox_ubx_msgs.msg import UBXNavPVT, UBXNavHPPosLLH

from .sbl_geo import haversine_m, mean_latlon, max_spread_m

_HPPOSLLH_H_ACC_TO_M = 1e-4  # UBX-NAV-HPPOSLLH h_acc is 0.1 mm units
_CARR_SOLN_FIXED = 2         # ublox_ubx_msgs/CarrSoln: 0 none, 1 float, 2 fixed


class SblPositionCalibration(Node):
    def __init__(self):
        super().__init__("sbl_position_calibration")
        p = self.declare_parameter
        self.url = p("waterlinked_url", "http://192.168.194.94").value.rstrip("/")
        self.source = str(p("source", "gnss").value).lower()
        self.fix_topic = p("fix_topic", "/fix").value
        self.pvt_topic = p("pvt_topic", "/ubx_nav_pvt").value
        self.hpposllh_topic = p("hpposllh_topic", "/ubx_nav_hp_pos_llh").value

        self.require_rtk_fixed = bool(p("require_rtk_fixed", True).value)
        self.h_acc_max_m = float(p("h_acc_max_m", 0.50).value)
        self.collect_seconds = float(p("collect_seconds", 10.0).value)
        self.min_fixes = int(p("min_fixes", 10).value)
        self.max_spread_m = float(p("max_spread_m", 0.5).value)

        self.sim_lat = float(p("sim_lat", 0.0).value)
        self.sim_lon = float(p("sim_lon", 0.0).value)

        self.require_static_mode = bool(p("require_static_mode", True).value)
        self.endpoint = p("config_endpoint", "/api/v1/config/generic").value
        self.lat_key = p("lat_key", "static_lat").value
        self.lon_key = p("lon_key", "static_lon").value
        self.verify_wait_s = float(p("verify_wait_s", 5.0).value)
        self.verify_tol_m = float(p("verify_tol_m", 1.0).value)
        self.http_timeout_s = float(p("http_timeout_s", 5.0).value)

        # GNSS state
        self._carr_soln = None
        self._gnss_fix_ok = False
        self._h_acc_m = None
        self._fix_count = 0
        self._samples = []          # gated (lat, lon)
        self._collecting = False

        if self.source == "gnss":
            # sensor_data QoS (BEST_EFFORT): ublox_nav_sat_fix_hp_node publishes /fix with
            # SensorDataQoS; a RELIABLE subscription is incompatible and receives nothing
            # ("no messages on /fix" -> abort). BEST_EFFORT also matches the RELIABLE UBX topics.
            self.create_subscription(NavSatFix, self.fix_topic, self._on_fix, qos_profile_sensor_data)
            self.create_subscription(UBXNavPVT, self.pvt_topic, self._on_pvt, qos_profile_sensor_data)
            self.create_subscription(
                UBXNavHPPosLLH, self.hpposllh_topic, self._on_hp, qos_profile_sensor_data
            )

    # --- callbacks ---
    def _on_pvt(self, msg):
        try:
            self._carr_soln = int(msg.carr_soln.status)
        except AttributeError:
            self._carr_soln = None
        self._gnss_fix_ok = bool(getattr(msg, "gnss_fix_ok", False))

    def _on_hp(self, msg):
        self._h_acc_m = float(msg.h_acc) * _HPPOSLLH_H_ACC_TO_M

    def _rtk_ok(self):
        if self.require_rtk_fixed and self._carr_soln != _CARR_SOLN_FIXED:
            return False
        if self._carr_soln is None or self._carr_soln < 1:  # need at least float carrier
            return False
        if not self._gnss_fix_ok:
            return False
        if self._h_acc_m is None or not (0.0 < self._h_acc_m <= self.h_acc_max_m):
            return False
        return True

    def _on_fix(self, msg):
        self._fix_count += 1
        if not self._collecting:
            return
        lat, lon = float(msg.latitude), float(msg.longitude)
        if abs(lat) < 0.1 and abs(lon) < 0.1:  # null-island guard
            return
        if not self._rtk_ok():
            return
        self._samples.append((lat, lon))

    # --- HTTP ---
    def _get(self, path):
        try:
            r = requests.get(self.url + path, timeout=self.http_timeout_s)
        except requests.exceptions.RequestException as exc:
            print(f"  GET {path} FAILED: {exc}")
            return None
        if r.status_code != 200:
            print(f"  GET {path} -> HTTP {r.status_code}: {r.text[:200]}")
            return None
        return r.json()

    def _put(self, path, obj):
        try:
            r = requests.put(self.url + path, json=obj, timeout=self.http_timeout_s)
        except requests.exceptions.RequestException as exc:
            print(f"  PUT {path} FAILED: {exc}")
            return False
        if r.status_code not in (200, 204):
            print(f"  PUT {path} -> HTTP {r.status_code}: {r.text[:200]}")
            return False
        return True

    def _spin(self, duration_s):
        end = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _carr_name(self):
        return {0: "NONE", 1: "FLOAT", 2: "FIXED"}.get(self._carr_soln, "?")

    # --- steps ---
    def run(self):
        print(f"\n=== SBL topside position calibration ===\nbox: {self.url}  source={self.source}\n")

        # target position: simulated or from GNSS
        if self.source == "simulated":
            target = (self.sim_lat, self.sim_lon)
            print(f"simulated target: lat={target[0]:.7f} lon={target[1]:.7f}")
        else:
            target = self._collect_gnss()
            if target is None:
                return

        self._write_and_verify(target)

    def _collect_gnss(self):
        print("Pre-checks:")
        self._spin(2.0)
        ok = True
        if self._fix_count > 0:
            print(f"  [OK] {self.fix_topic} publishing")
        else:
            print(f"  [FAIL] no messages on {self.fix_topic}")
            ok = False
        if self._carr_soln is not None:
            print(f"  [OK] {self.pvt_topic}: carr_soln={self._carr_name()} gnss_fix_ok={self._gnss_fix_ok}")
        else:
            print(f"  [FAIL] no {self.pvt_topic} (UBXNavPVT) — cannot confirm RTK")
            ok = False
        if self._h_acc_m is not None:
            print(f"  [OK] {self.hpposllh_topic}: h_acc={self._h_acc_m:.3f} m")
        else:
            print(f"  [FAIL] no {self.hpposllh_topic} (UBXNavHPPosLLH) — no accuracy")
            ok = False
        if not self._rtk_ok():
            need = "FIXED" if self.require_rtk_fixed else "FLOAT+"
            print(f"  [FAIL] not RTK-{need} (carr_soln={self._carr_name()}, "
                  f"gnss_fix_ok={self._gnss_fix_ok}, h_acc={self._h_acc_m} m)")
            ok = False
        else:
            print(f"  [OK] RTK gate satisfied")
        if not ok:
            print("\nABORT: GNSS RTK not ready.\n")
            return None

        print(f"\nHold the vehicle above the topside. Collecting RTK fixes for {self.collect_seconds:.0f} s...")
        self._samples.clear()
        self._collecting = True
        t0 = time.monotonic()
        last = 0.0
        while rclpy.ok() and (time.monotonic() - t0) < self.collect_seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - last > 0.5:
                sp = max_spread_m(self._samples, mean_latlon(self._samples)) if len(self._samples) >= 2 else 0.0
                print(f"  collecting... t={time.monotonic()-t0:3.1f}/{self.collect_seconds:.0f} s  "
                      f"RTK fixes={len(self._samples):3d}  spread={sp:.2f} m", flush=True)
                last = time.monotonic()
        self._collecting = False

        n = len(self._samples)
        if n < self.min_fixes:
            print(f"\nABORT: only {n} RTK fixes (< {self.min_fixes}); need a steadier RTK lock.\n")
            return None
        center = mean_latlon(self._samples)
        spread = max_spread_m(self._samples, center)
        print(f"\ntopside position: {center[0]:.7f}, {center[1]:.7f}  (spread {spread:.2f} m, {n} fixes)")
        if spread > self.max_spread_m:
            print(f"\nABORT: fixes too scattered (spread {spread:.2f} m > {self.max_spread_m} m).\n")
            return None
        return center

    def _write_and_verify(self, target):
        cfg = self._get(self.endpoint)
        if not isinstance(cfg, dict):
            print(f"\nABORT: could not GET {self.endpoint}.\n")
            return
        gps, comp = cfg.get("gps"), cfg.get("compass")
        if self.require_static_mode and not (gps == "static" and comp == "static"):
            print(f"\nABORT: topside not in static mode (gps={gps!r}, compass={comp!r}). "
                  "Set GPS & Compass to 'Static' in the GUI, then re-run.\n")
            return
        if self.lat_key not in cfg or self.lon_key not in cfg:
            print(f"\nABORT: '{self.lat_key}'/'{self.lon_key}' not in {self.endpoint}. "
                  f"Full config:\n  {cfg}\n")
            return

        old = (float(cfg[self.lat_key]), float(cfg[self.lon_key]))
        moved = haversine_m(old[0], old[1], target[0], target[1])
        cfg[self.lat_key] = target[0]
        cfg[self.lon_key] = target[1]
        print(f"\n  prior:  {old[0]:.7f}, {old[1]:.7f}")
        print(f"  new:    {target[0]:.7f}, {target[1]:.7f}   (moved {moved:.1f} m)")

        print(f"\n  PUT {self.endpoint} ({self.lat_key}/{self.lon_key}) ...")
        if not self._put(self.endpoint, cfg):
            print("\n[FAILED] CALIBRATION FAILED (PUT rejected).\n")
            return
        print(f"  waiting {self.verify_wait_s:.0f} s, then verifying...")
        self._spin(self.verify_wait_s)

        master = self._get("/api/v1/position/master") or {}
        m_lat = master.get("lat")
        m_lon = master.get("lon", master.get("long"))
        cfg2 = self._get(self.endpoint) or {}
        c_lat, c_lon = cfg2.get(self.lat_key), cfg2.get(self.lon_key)
        print(f"  read back: config=({c_lat},{c_lon})  master=({m_lat},{m_lon})  target=({target[0]:.7f},{target[1]:.7f})")

        err = None
        if m_lat is not None and m_lon is not None:
            err = haversine_m(float(m_lat), float(m_lon), target[0], target[1])
        if err is not None and err <= self.verify_tol_m:
            print(f"\n[SUCCESS] topside position calibrated (verified within {err:.2f} m).\n")
        elif c_lat is not None and abs(float(c_lat) - target[0]) < 1e-6 and abs(float(c_lon) - target[1]) < 1e-6:
            print("\n[PARTIAL] config updated, but master lat/lon not confirmed within tolerance "
                  "(box may need a moment / not fully static).\n")
        else:
            print("\n[FAILED] position did not persist.\n")


def main(args=None):
    rclpy.init(args=args)
    node = SblPositionCalibration()
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
