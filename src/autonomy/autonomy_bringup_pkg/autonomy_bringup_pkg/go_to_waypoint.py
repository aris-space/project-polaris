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
  ros2 run autonomy_bringup_pkg go_to_waypoint --index 1 --origin 47.3287,8.5727,453 --file /path/line.csv
"""

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--index', type=int, required=True,
                        help='1-based waypoint index to go to (1 = first point of the loaded line).')
    parser.add_argument('--no-hold', action='store_true',
                        help='Disarm + MANUAL on arrival (default: stay armed+GUIDED and hold).')
    parser.add_argument('--origin', type=parse_origin, default=None,
                        help='lat,lon,alt_m manual override (default: consume latched /mission_waypoints_enu)')
    parser.add_argument('--file', type=str, default=None, help='Override CSV (requires --origin).')
    args, ros_args = parser.parse_known_args()

    if (args.origin is None) != (args.file is None):
        print('Manual override requires BOTH --origin and --file (or neither).', file=sys.stderr)
        sys.exit(1)
    if args.index < 1:
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

        hb_state = PixhawkState()
        node.create_subscription(PixhawkHeartbeat, '/pixhawk/heartbeat',
                                 make_heartbeat_callback(hb_state), 10)

        # Get the full line, then pick ONE pose.
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

        print(f'>>> Going to waypoint {args.index} <<<')
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
                print('>>> Arrived. HOLDING on station (armed + GUIDED). <<<')
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
