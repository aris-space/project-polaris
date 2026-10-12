#!/usr/bin/env python3
"""Capture the current position (with a valid RTK fix) as a single waypoint file.

For pool/field testing of go_to_waypoint without the full mission pipeline: put the vehicle where you
want the target, run this with a good RTK fix, and it writes the current MAP-frame position (from the
global EKF, which is RTK-anchored) to a JSON file. Then drive the vehicle elsewhere and send it back:

    ros2 run autonomy_bringup_pkg capture_waypoint --out /tmp/wp.json
    ros2 run autonomy_bringup_pkg go_to_waypoint --waypoint-file /tmp/wp.json

It gates on RTK quality via /fix (status >= GBAS and covariance-derived h_acc <= h_acc_max), averages a
short window, and stores the map x,y,z (what nav2 consumes) plus lat/lon for the record. Requires the
global EKF to be up (so /odometry/filtered/global and the map frame exist, same as go_to_waypoint needs).

Args:  --out PATH  --seconds S  --h-acc-max M  --min-samples N  --fix-topic T  --odom-topic T
"""

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry


def _hacc_from_fix(msg) -> float:
    """Horizontal accuracy [m] from NavSatFix.position_covariance (sqrt of max E/N variance)."""
    c = msg.position_covariance
    try:
        return math.sqrt(max(float(c[0]), float(c[4])))
    except Exception:
        return float('inf')


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--out', default='captured_waypoint.json', help='Output JSON path.')
    p.add_argument('--seconds', type=float, default=4.0, help='Averaging window [s] (default 4).')
    p.add_argument('--h-acc-max', type=float, default=0.5, help='Max horizontal accuracy [m] (default 0.5).')
    p.add_argument('--min-samples', type=int, default=10, help='Min gated fixes to accept (default 10).')
    p.add_argument('--fix-topic', default='/fix')
    p.add_argument('--odom-topic', default='/odometry/filtered/global')
    args, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = rclpy.create_node('capture_waypoint')

    state = {'fix': None, 'odom': None}

    def on_fix(m): state['fix'] = m
    def on_odom(m): state['odom'] = m

    # BEST_EFFORT (sensor data) QoS: the GNSS driver publishes /fix BEST_EFFORT, so a default
    # RELIABLE subscriber receives NOTHING ("incompatible qos: reliability"). A BEST_EFFORT
    # subscriber is compatible with both BEST_EFFORT (/fix) and RELIABLE (EKF odom) publishers.
    node.create_subscription(NavSatFix, args.fix_topic, on_fix, qos_profile_sensor_data)
    node.create_subscription(Odometry, args.odom_topic, on_odom, qos_profile_sensor_data)

    def spin(dt):
        end = time.monotonic() + dt
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    def rtk_ok(fix):
        if fix is None:
            return False, 'no /fix yet'
        if fix.status.status < 2:   # <2 = no GBAS/RTK carrier solution
            return False, f'status={fix.status.status} (need >=2 / GBAS-RTK)'
        if abs(fix.latitude) < 0.1 and abs(fix.longitude) < 0.1:
            return False, 'null-island fix'
        h = _hacc_from_fix(fix)
        if not (0.0 < h <= args.h_acc_max):
            return False, f'h_acc={h:.3f} m (> {args.h_acc_max})'
        return True, f'h_acc={h:.3f} m'

    try:
        print(f'\n=== capture_waypoint ===\nfix:{args.fix_topic}  odom:{args.odom_topic}\n')
        spin(2.0)
        ok = True
        if state['odom'] is None:
            print(f'  [FAIL] no {args.odom_topic} (global EKF up? map frame needed)'); ok = False
        good, why = rtk_ok(state['fix'])
        print(f'  [{"OK" if good else "FAIL"}] RTK: {why}')
        ok = ok and good
        if not ok:
            print('\nABORT: RTK fix / map pose not ready.\n'); return

        print(f'Collecting for {args.seconds:.0f} s with RTK held...')
        lats=[]; lons=[]; alts=[]; xs=[]; ys=[]; zs=[]; frame='map'
        t0=time.monotonic()
        while rclpy.ok() and time.monotonic()-t0 < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
            good,_ = rtk_ok(state['fix'])
            if good and state['odom'] is not None:
                f=state['fix']; o=state['odom']
                lats.append(f.latitude); lons.append(f.longitude); alts.append(f.altitude)
                xs.append(o.pose.pose.position.x); ys.append(o.pose.pose.position.y); zs.append(o.pose.pose.position.z)
                frame = o.header.frame_id or 'map'

        n=len(xs)
        if n < args.min_samples:
            print(f'\nABORT: only {n} gated samples (< {args.min_samples}); RTK unstable.\n'); return

        def mean(a): return sum(a)/len(a)
        wp = {'frame_id': frame, 'x': mean(xs), 'y': mean(ys), 'z': mean(zs),
              'lat': mean(lats), 'lon': mean(lons), 'alt': mean(alts),
              'h_acc_m': _hacc_from_fix(state['fix']), 'n_samples': n,
              'stamp': time.strftime('%Y-%m-%dT%H:%M:%S')}
        with open(args.out, 'w', encoding='utf-8') as fh:
            json.dump(wp, fh, indent=2)
        print(f'\ncaptured ({n} fixes):  map[{frame}] x={wp["x"]:.2f} y={wp["y"]:.2f} z={wp["z"]:.2f} m')
        print(f'                       lat={wp["lat"]:.7f} lon={wp["lon"]:.7f}  h_acc={wp["h_acc_m"]:.3f} m')
        print(f'wrote {args.out}\n'
              f'-> ros2 run autonomy_bringup_pkg go_to_waypoint --waypoint-file {args.out}\n')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
