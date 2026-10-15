#!/usr/bin/env python3
"""Go to ONE waypoint of the loaded line, then HOLD on station (thesis SBL mission).

Unlike ``wgs84_mission_starter`` (which hands the whole CSV to FollowWaypoints and disarms + sets MANUAL
when the list finishes), this sends a SINGLE waypoint (chosen by 1-based ``--index``) and, on arrival,
**stays armed + GUIDED** so the mavlink-bridge watchdog holds position (XYZ: XY via DVL, depth via pressure)
until you command the next waypoint or take over manually.

Operator flow per station (Option B descent):
  1. (surface) run with --index N to position over the point, or skip and hand-fly roughly over it.
  2. descend to the desired depth in MANUAL.
  3. run again with --index N: it drives to the point AT that held depth and holds there. Record SBL.
  4. switch to MANUAL on the joystick to take over / ascend; run --index N+1 for the next station.

Reuses the validated single-waypoint path from wgs84_mission_starter (FollowWaypoints with one pose).

Usage:
  ros2 run autonomy_bringup_pkg go_to_waypoint --index 1
  ros2 run autonomy_bringup_pkg go_to_waypoint --index 3 --no-hold      # disarm+MANUAL on arrival (old behaviour)
  ros2 run autonomy_bringup_pkg go_to_waypoint --waypoint-file /tmp/wp.json   # target z := current depth
  ros2 run autonomy_bringup_pkg go_to_waypoint --waypoint-file /tmp/wp.json --keep-target-z
  ros2 run autonomy_bringup_pkg go_to_waypoint --index 1 --origin 47.3287,8.5727,453 --file /path/line.csv
"""

import argparse
import sys
import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Odometry
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from mavros_msgs.msg import State as PixhawkHeartbeat
from std_msgs.msg import Bool, String

from autonomy_bringup_pkg.load_wgs84_points_to_waypoints import process_coordinates
from autonomy_bringup_pkg.nav2_ready_wait import wait_for_waypoint_follower_active
from autonomy_bringup_pkg.pixhawk_ready_wait import (
    PixhawkState,
    ensure_armed_and_mode_guided,
    make_heartbeat_callback,
)
from autonomy_bringup_pkg.WGS84_mission_starter import (
    SendGoalResult,
    parse_origin,
    publish_disarm_and_spin,
    publish_manual_and_spin,
    send_goal,
    wait_for_mission_waypoints,
)


def _fmt(p: PoseStamped) -> str:
    q = p.pose.position
    return f'x={q.x:.2f}m  y={q.y:.2f}m  z={q.z:.2f}m'


def _load_waypoint_file(path: str) -> PoseStamped:
    """Load a single waypoint captured by capture_waypoint (JSON, map-frame x,y,z)."""
    with open(path, 'r', encoding='utf-8') as f:
        d = json.load(f)
    ps = PoseStamped()
    ps.header.frame_id = str(d.get('frame_id', 'map'))
    ps.pose.position.x = float(d['x'])
    ps.pose.position.y = float(d['y'])
    ps.pose.position.z = float(d['z'])
    ps.pose.orientation.w = 1.0
    return ps


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--index', type=int, default=None,
                        help='1-based waypoint index into the loaded line (1 = first point).')
    parser.add_argument('--waypoint-file', type=str, default=None,
                        help='Go to a single waypoint captured in a JSON file (map-frame x,y,z) by '
                             'capture_waypoint. Bypasses the mission loader (pool/field tests).')
    parser.add_argument('--no-hold', action='store_true',
                        help='Disarm + MANUAL on arrival (default: stay armed+GUIDED and hold).')
    parser.add_argument('--hold-here', action='store_true',
                        help='On arrival hold where the goal checker declared success (old '
                             'behaviour). Default: shift the hold onto the waypoint itself '
                             '(arrival can be up to xy_goal_tolerance away).')
    parser.add_argument('--keep-target-z', action='store_true',
                        help='Use the waypoint z as stored. Default: replace it with the CURRENT '
                             'depth when the run starts. Heave is never sent to the Pixhawk '
                             '(bridge vz=0, depth is held), but the controller ramps speed down on '
                             'the 3D distance to the goal, so a surface-captured z on a submerged '
                             'run kept it at ~0.1-0.2 m/s at the XY goal -> overshoot.')
    parser.add_argument('--odom-topic', default='/odometry/filtered/global',
                        help='Map-frame odometry used for the current depth (default: global EKF).')
    parser.add_argument('--origin', type=parse_origin, default=None,
                        help='lat,lon,alt_m manual override (default: consume latched /mission_waypoints_enu)')
    parser.add_argument('--file', type=str, default=None, help='Override CSV (requires --origin).')
    args, ros_args = parser.parse_known_args()

    if (args.origin is None) != (args.file is None):
        print('Manual override requires BOTH --origin and --file (or neither).', file=sys.stderr)
        sys.exit(1)
    if (args.waypoint_file is None) == (args.index is None):
        print('Provide exactly one of --index N or --waypoint-file PATH.', file=sys.stderr)
        sys.exit(1)
    if args.index is not None and args.index < 1:
        print('--index is 1-based and must be >= 1.', file=sys.stderr)
        sys.exit(1)

    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)

    node = None
    follow_waypoints = None
    mode_pub = None
    arm_pub = None
    executor = None
    try:
        node = rclpy.create_node('go_to_waypoint',
                                 automatically_declare_parameters_from_overrides=True)
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        follow_waypoints = ActionClient(node, FollowWaypoints, '/follow_waypoints')
        mode_pub = node.create_publisher(String, '/pixhawk/mode_cmd', 10)
        arm_pub = node.create_publisher(Bool, '/pixhawk/arm_cmd', 10)
        status_pub = node.create_publisher(String, '/mission_status', 10)
        hold_offset_pub = node.create_publisher(Vector3, '/pixhawk/hold_offset_enu', 10)

        hb_state = PixhawkState()
        node.create_subscription(PixhawkHeartbeat, '/pixhawk/heartbeat',
                                 make_heartbeat_callback(hb_state), 10)

        # Latest map-frame odometry, for replacing the target z with the current depth.
        # BEST_EFFORT-compatible QoS (works with RELIABLE and BEST_EFFORT publishers).
        odom_state = {'msg': None, 't': 0.0}

        def _on_odom(m):
            odom_state['msg'] = m
            odom_state['t'] = time.monotonic()

        node.create_subscription(Odometry, args.odom_topic, _on_odom, qos_profile_sensor_data)

        # Pick the single target pose: captured-file mode, or index into the loaded line.
        if args.waypoint_file is not None:
            try:
                target = _load_waypoint_file(args.waypoint_file)
            except (OSError, ValueError, KeyError) as e:
                print(f'Error reading --waypoint-file {args.waypoint_file}: {e}', file=sys.stderr)
                sys.exit(1)
            print(f'Going to captured waypoint from {args.waypoint_file}  '
                  f'[{target.header.frame_id}]  {_fmt(target)}')
        else:
            if args.origin is not None:
                lat0, lon0, alt0 = args.origin
                print(f'Manual override: origin={args.origin}  file={args.file}')
                try:
                    poses = process_coordinates(args.file, lat0, lon0, alt0)
                except (OSError, ValueError) as e:
                    print(f'Error loading mission: {e}', file=sys.stderr)
                    sys.exit(1)
            else:
                poses = wait_for_mission_waypoints(executor, node)
                if poses is None:
                    print('No latched waypoints from mission_waypoint_loader; exiting.', file=sys.stderr)
                    sys.exit(1)
            n = len(poses)
            if args.index > n:
                print(f'--index {args.index} out of range: only {n} waypoints loaded (1..{n}).', file=sys.stderr)
                sys.exit(1)
            target = poses[args.index - 1]
            print(f'Going to waypoint {args.index}/{n}:  {_fmt(target)}')

        goal = FollowWaypoints.Goal()
        goal.poses = [target]

        for _ in range(10):
            executor.spin_once(timeout_sec=0.05)

        if not wait_for_waypoint_follower_active(executor, node, timeout_sec=180.0):
            print('Nav2 not ready; exiting.', file=sys.stderr)
            sys.exit(1)

        if not ensure_armed_and_mode_guided(executor, node, arm_pub, mode_pub, hb_state):
            print('Pixhawk not ready (arm + GUIDED); aborting.', file=sys.stderr)
            publish_manual_and_spin(executor, node, mode_pub, spins=20)
            publish_disarm_and_spin(executor, node, arm_pub, spins=20)
            sys.exit(1)

        if not args.keep_target_z:
            # Fresh odometry (<1 s old), waiting up to 3 s for it.
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and (
                    odom_state['msg'] is None or time.monotonic() - odom_state['t'] > 1.0):
                executor.spin_once(timeout_sec=0.05)
            om = odom_state['msg']
            if om is None or time.monotonic() - odom_state['t'] > 1.0:
                print(f'WARNING: no fresh {args.odom_topic}; keeping target z={target.pose.position.z:.2f} m '
                      '(speed near the goal may not ramp down if submerged).', file=sys.stderr)
            elif om.header.frame_id and target.header.frame_id and om.header.frame_id != target.header.frame_id:
                print(f'WARNING: {args.odom_topic} frame "{om.header.frame_id}" != target frame '
                      f'"{target.header.frame_id}"; keeping target z.', file=sys.stderr)
            else:
                z_now = float(om.pose.pose.position.z)
                print(f'Target z {target.pose.position.z:.2f} m -> current depth {z_now:.2f} m '
                      '(horizontal-only approach; depth is held by the Pixhawk).')
                target.pose.position.z = z_now
                goal.poses = [target]

        wp_label = f'waypoint {args.index}' if args.index is not None else 'captured waypoint'
        print(f'>>> Going to {wp_label} <<<')
        result = send_goal(executor, follow_waypoints, goal, node, mode_pub, arm_pub,
                           status_pub=status_pub, total_waypoints=1, poses=[target], hb_state=hb_state)

        if result == SendGoalResult.SUCCESS and rclpy.ok():
            if args.no_hold:
                print('>>> Arrived; --no-hold set: disarming + MANUAL <<<')
                arm_pub.publish(Bool(data=False))
                time.sleep(0.5)
                mode_pub.publish(String(data='MANUAL'))
                executor.spin_once(timeout_sec=0.2)
            else:
                # Leave armed + GUIDED: the mavlink-bridge cmd_vel watchdog holds XYZ
                # (depth via pressure, XY via DVL) once the controller stops publishing.
                for _ in range(10):
                    executor.spin_once(timeout_sec=0.05)
                if not args.hold_here:
                    # Shift the hold from the arrival point onto the waypoint: the goal
                    # checker succeeds up to xy_goal_tolerance away and the bridge's
                    # hold-here (sent ~0.8 s after the last cmd_vel) keeps that point.
                    # Offset = goal - current map pose (true-ENU map axes; the bridge sends
                    # it as LOCAL_OFFSET_NED, i.e. relative to the current position).
                    deadline = time.monotonic() + 2.0
                    while time.monotonic() < deadline and (
                            odom_state['msg'] is None or time.monotonic() - odom_state['t'] > 1.0):
                        executor.spin_once(timeout_sec=0.05)
                    om = odom_state['msg']
                    if om is None or time.monotonic() - odom_state['t'] > 1.0:
                        print(f'WARNING: no fresh {args.odom_topic}; holding where it arrived.',
                              file=sys.stderr)
                    else:
                        de = target.pose.position.x - om.pose.pose.position.x
                        dn = target.pose.position.y - om.pose.pose.position.y
                        hold_offset_pub.publish(Vector3(x=de, y=dn, z=0.0))
                        for _ in range(5):
                            executor.spin_once(timeout_sec=0.05)
                        print(f'>>> Hold shifted onto the waypoint: offset E{de:+.2f} N{dn:+.2f} m '
                              f'({(de * de + dn * dn) ** 0.5:.2f} m; bridge refuses > hold_offset_max_m)')
                print('>>> Arrived. HOLDING on station (armed + GUIDED). <<<')
                if args.index is not None:
                    # Line/index mission: hint the next station. (In --waypoint-file mode
                    # args.index is None, so this hint is skipped — the earlier crash here
                    # was `None + 1`.)
                    print('    Re-run with --index {N} to go to the next point,'.format(N=args.index + 1))
                print('    or switch to MANUAL on the joystick to take over / ascend.')
        elif result == SendGoalResult.FAILURE and rclpy.ok():
            print('>>> Goal failed; disarm + MANUAL <<<')
            publish_manual_and_spin(executor, node, mode_pub, spins=20)
            publish_disarm_and_spin(executor, node, arm_pub, spins=20)
        elif result == SendGoalResult.CANCELED and rclpy.ok():
            # send_goal already set MANUAL + disarm on pilot intervention.
            print('>>> Canceled (pilot intervention). <<<')
            publish_manual_and_spin(executor, node, mode_pub, spins=15)
            publish_disarm_and_spin(executor, node, arm_pub, spins=20)

        print('>>> Done <<<')

    except KeyboardInterrupt:
        if executor is not None and node is not None:
            if mode_pub is not None:
                print('>>> Interrupted, setting Pixhawk mode to MANUAL <<<')
                publish_manual_and_spin(executor, node, mode_pub)
            if arm_pub is not None:
                print('>>> Interrupted, disarming <<<')
                publish_disarm_and_spin(executor, node, arm_pub)
    finally:
        if follow_waypoints is not None:
            follow_waypoints.destroy()
        if executor is not None and node is not None:
            try:
                executor.remove_node(node)
            except Exception:
                pass
            executor.shutdown()
        if node is not None:
            node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
