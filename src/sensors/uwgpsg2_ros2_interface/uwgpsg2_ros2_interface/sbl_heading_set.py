#!/usr/bin/env python3
"""Minimal laptop-only test of the Waterlinked heading API (thesis).

No sensors, no ROS topics, no vehicle — just the HTTP round-trip, so you can confirm on your laptop
that writing the topside heading actually works and that `static_orientation` is the right field:

    GET  /api/v1/config/generic      -> show current gps/compass mode + static_orientation
    GET  /api/v1/position/master     -> show current effective orientation
    (optional) PUT the full config/generic with static_orientation = <heading>
    wait, then GET both again to verify the value stuck.

This is the write/verify half of sbl_heading_calibration, isolated. Point it at the box over Wi-Fi.

Run:
    ros2 launch uwgpsg2_ros2_interface sbl_heading_set.launch.py                 # read-only probe
    ros2 launch uwgpsg2_ros2_interface sbl_heading_set.launch.py heading:=123.4  # set + verify
    ros2 run uwgpsg2_ros2_interface sbl_heading_set --ros-args -p heading_deg:=123.4
"""

import time

import requests

import rclpy
from rclpy.node import Node

from .sbl_heading_math import norm180, norm360


class SblHeadingSet(Node):
    def __init__(self):
        super().__init__("sbl_heading_set")
        p = self.declare_parameter
        self.url = p("waterlinked_url", "http://192.168.194.94").value.rstrip("/")
        self.endpoint = p("heading_config_endpoint", "/api/v1/config/generic").value
        self.key = p("heading_config_key", "static_orientation").value
        # heading_deg < 0  -> read-only probe (no PUT)
        self.heading_deg = float(p("heading_deg", -1.0).value)
        self.require_static_mode = bool(p("require_static_mode", True).value)
        self.verify_wait_s = float(p("verify_wait_s", 5.0).value)
        self.verify_tol_deg = float(p("verify_tol_deg", 1.0).value)
        self.http_timeout_s = float(p("http_timeout_s", 5.0).value)

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

    def run(self):
        print(f"\n=== SBL heading API test ===\nbox: {self.url}\n")

        cfg = self._get(self.endpoint)
        master = self._get("/api/v1/position/master")
        if not isinstance(cfg, dict):
            print(f"\nABORT: could not GET {self.endpoint} (box not reachable?).\n")
            return
        print(f"GET {self.endpoint}:")
        print(f"  gps={cfg.get('gps')!r}  compass={cfg.get('compass')!r}")
        if self.key in cfg:
            print(f"  {self.key} = {cfg[self.key]}")
        else:
            print(f"  [!] '{self.key}' NOT present. Full config:\n  {cfg}")
        if isinstance(master, dict):
            print(f"GET /api/v1/position/master: orientation = {master.get('orientation')}")

        # read-only probe
        if self.heading_deg < 0:
            print("\n(no heading_deg given -> read-only probe; pass heading:=<deg> to write)\n")
            return

        if self.key not in cfg:
            print(f"\nABORT: '{self.key}' missing from {self.endpoint}; not writing.\n")
            return

        gps, comp = cfg.get("gps"), cfg.get("compass")
        if self.require_static_mode and not (gps == "static" and comp == "static"):
            print(f"\nABORT: box not in static mode (gps={gps!r}, compass={comp!r}). "
                  "Set GPS & Compass to 'Static' in the GUI, then retry.\n")
            return

        target = norm360(self.heading_deg)
        old = float(cfg[self.key])
        cfg[self.key] = target
        print(f"\nPUT {self.endpoint}: {self.key} {old} -> {target} ...")
        if not self._put(self.endpoint, cfg):
            print("\n[FAILED] PUT rejected by box.\n")
            return

        print(f"waiting {self.verify_wait_s:.0f} s, then verifying...")
        time.sleep(self.verify_wait_s)

        got_cfg = (self._get(self.endpoint) or {}).get(self.key)
        got_master = (self._get("/api/v1/position/master") or {}).get("orientation")
        print(f"read back: {self.key}={got_cfg}   master.orientation={got_master}   (target {target})")

        ok_cfg = got_cfg is not None and abs(norm180(float(got_cfg) - target)) <= self.verify_tol_deg
        ok_master = (got_master is not None
                     and abs(norm180(float(got_master) - target)) <= self.verify_tol_deg)
        if ok_cfg and ok_master:
            print("\n[SUCCESS] heading written and verified on both config + master.\n")
        elif ok_cfg:
            print("\n[PARTIAL] config updated, but master.orientation did not follow "
                  "(give it a moment / confirm static mode).\n")
        else:
            print("\n[FAILED] value did not persist.\n")


def main(args=None):
    rclpy.init(args=args)
    node = SblHeadingSet()
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
