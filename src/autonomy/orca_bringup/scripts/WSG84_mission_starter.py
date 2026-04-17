#!/usr/bin/env python3
"""
Send Nav2 waypoints from ``missions/default_wgs84_mission.csv``.

Origin defaults to ``missions/default_mission_origin.json`` (same as ArduSub home in sim_launch).
Override: ``--origin=lat,lon,alt`` and/or ``--file path.csv``.
"""

from enum import Enum
import argparse
import json
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String

from load_wsg84_points_to_waypoints import process_coordinates
from nav2_ready_wait import wait_for_waypoint_follower_active
from pixhawk_ready_wait import (
    PixhawkState,
    ensure_armed_and_mode_guided,
    make_heartbeat_callback,
)


class SendGoalResult(Enum):
    SUCCESS = 0
    FAILURE = 1
    CANCELED = 2


def publish_manual_and_spin(executor, node, mode_pub, spins: int = 40) -> None:
    """Flush MANUAL mode. Swallows RCLError if the context is already shutting down."""
    if mode_pub is None or node is None or executor is None:
        return
    try:
        mode_pub.publish(String(data='MANUAL'))
    except Exception:
        return
    for _ in range(spins):
        if not rclpy.ok():
            break
        try:
            executor.spin_once(timeout_sec=0.05)
        except Exception:
            break


def publish_disarm_and_spin(executor, node, arm_pub, spins: int = 30) -> None:
    """Flush disarm (arm_cmd False). Swallows RCLError if the context is invalid."""
    if arm_pub is None or node is None or executor is None:
        return
    try:
        arm_pub.publish(Bool(data=False))
    except Exception:
        return
    for _ in range(spins):
        if not rclpy.ok():
            break
        try:
            executor.spin_once(timeout_sec=0.05)
        except Exception:
            break


def default_mission_csv_path() -> str:
    share = get_package_share_directory('orca_bringup')
    return f'{share}/missions/default_wgs84_mission.csv'


def default_mission_origin_path() -> str:
    share = get_package_share_directory('orca_bringup')
    return f'{share}/missions/default_mission_origin.json'


def load_origin_from_share() -> tuple:
    """(lat, lon, alt_m) from default_mission_origin.json — keep in sync with sim_launch ArduSub home."""
    path = default_mission_origin_path()
    with open(path, encoding='utf-8') as f:
        o = json.load(f)
    return float(o['lat']), float(o['lon']), float(o['alt'])


def parse_origin(s: str) -> tuple:
    parts = [p.strip() for p in s.split(',')]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError('expected lat,lon,alt_m (three comma-separated numbers)')
    return float(parts[0]), float(parts[1]), float(parts[2])


def wait_for_follow_waypoints(executor, action_client, timeout_sec: float = 300.0) -> bool:
    start = time.time()
    print('Waiting for Nav2 /follow_waypoints action server...')
    while rclpy.ok():
        if action_client.server_is_ready():
            print('Nav2 /follow_waypoints is available.')
            return True
        if time.time() - start >= timeout_sec:
            print(
                f'Timed out after {timeout_sec:.0f}s waiting for /follow_waypoints. '
                'Check: ros2 action list | grep follow ; ros2 lifecycle get /waypoint_follower'
            )
            return False
        executor.spin_once(timeout_sec=0.1)
    return False


def send_goal(executor, action_client, send_goal_msg, node, mode_pub, arm_pub) -> SendGoalResult:
    goal_handle = None
    try:
        if not wait_for_follow_waypoints(executor, action_client):
            return SendGoalResult.FAILURE

        print('Sending goal...')
        goal_future = action_client.send_goal_async(send_goal_msg)
        executor.spin_until_future_complete(goal_future, timeout_sec=120.0)
        if not goal_future.done():
            print('Timeout waiting for goal acceptance from Nav2.')
            return SendGoalResult.FAILURE
        goal_handle = goal_future.result()

        if goal_handle is None:
            raise RuntimeError('Exception while sending goal: {!r}'.format(goal_future.exception()))

        if not goal_handle.accepted:
            print('Goal rejected')
            return SendGoalResult.FAILURE

        print('Goal accepted with ID: {}'.format(bytes(goal_handle.goal_id.uuid).hex()))
        result_future = goal_handle.get_result_async()
        executor.spin_until_future_complete(result_future)

        result = result_future.result()

        if result is None:
            raise RuntimeError('Exception while getting result: {!r}'.format(result_future.exception()))

        print('Goal completed')
        return SendGoalResult.SUCCESS

    except KeyboardInterrupt:
        if goal_handle is None:
            raise
        if (GoalStatus.STATUS_ACCEPTED == goal_handle.status or
                GoalStatus.STATUS_EXECUTING == goal_handle.status):
            # Send MANUAL before waiting on cancel: default rclpy SIGINT handling (if enabled)
            # invalidates the context during long spin_until_future_complete, breaking publish.
            print('>>> Interrupted, setting Pixhawk mode to MANUAL <<<')
            publish_manual_and_spin(executor, node, mode_pub, spins=25)

            print('Canceling goal...')
            cancel_future = goal_handle.cancel_goal_async()
            executor.spin_until_future_complete(cancel_future)
            cancel_response = cancel_future.result()

            if cancel_response is None:
                exc = cancel_future.exception()
                if exc is not None:
                    raise RuntimeError('Exception while canceling goal: {!r}'.format(exc)) from exc
                print('Cancel finished without response (shutdown?)')

            elif len(cancel_response.goals_canceling) == 0:
                raise RuntimeError('Failed to cancel goal')
            elif len(cancel_response.goals_canceling) > 1:
                raise RuntimeError('More than one goal canceled')
            elif cancel_response.goals_canceling[0].goal_id != goal_handle.goal_id:
                raise RuntimeError('Canceled goal with incorrect goal ID')
            else:
                print('Goal canceled')

            publish_manual_and_spin(executor, node, mode_pub, spins=15)

            print('>>> Interrupted, disarming <<<')
            publish_disarm_and_spin(executor, node, arm_pub, spins=30)
            return SendGoalResult.CANCELED
        raise


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--origin',
        type=parse_origin,
        default=None,
        help='lat,lon,alt_m (default: missions/default_mission_origin.json)',
    )
    parser.add_argument(
        '--file',
        type=str,
        default=None,
        help='Override CSV (default: share/.../missions/default_wgs84_mission.csv)',
    )

    args, ros_args = parser.parse_known_args()

    csv_path = args.file if args.file else default_mission_csv_path()
    if args.origin is not None:
        lat0, lon0, alt0 = args.origin
        print('Using origin from --origin')
    else:
        try:
            lat0, lon0, alt0 = load_origin_from_share()
        except (OSError, KeyError, TypeError, ValueError) as e:
            print(f'Failed to load default origin from {default_mission_origin_path()}: {e}', file=sys.stderr)
            sys.exit(1)
        print(f'Using origin from {default_mission_origin_path()}')

    print(f'Origin (WGS84): lat={lat0}, lon={lon0}, alt={alt0} m')
    print(f'Mission CSV: {csv_path}')

    try:
        poses = process_coordinates(csv_path, lat0, lon0, alt0)
    except (OSError, ValueError) as e:
        print(f'Error loading mission: {e}', file=sys.stderr)
        sys.exit(1)

    goal = FollowWaypoints.Goal()
    goal.poses = poses

    # Do not install rclpy SIGINT/SIGTERM handlers: they call shutdown() and invalidate the
    # context while we are still canceling the Nav2 goal, so /pixhawk/mode_cmd publish fails.
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)

    node = None
    follow_waypoints = None
    mode_pub = None
    arm_pub = None
    executor = None

    try:
        node = rclpy.create_node(
            'wsg84_mission_starter',
            automatically_declare_parameters_from_overrides=True,
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True),
            ],
        )
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        follow_waypoints = ActionClient(node, FollowWaypoints, '/follow_waypoints')
        mode_pub = node.create_publisher(String, '/pixhawk/mode_cmd', 10)
        arm_pub = node.create_publisher(Bool, '/pixhawk/arm_cmd', 10)

        hb_state = PixhawkState()
        node.create_subscription(
            String,
            '/pixhawk/heartbeat',
            make_heartbeat_callback(hb_state),
            10,
        )

        print(f'Loaded {len(goal.poses)} waypoints (map ENU)')

        for _ in range(10):
            executor.spin_once(timeout_sec=0.05)

        if not wait_for_waypoint_follower_active(executor, node, timeout_sec=180.0):
            print('Nav2 not ready; exiting.', file=sys.stderr)
            sys.exit(1)

        # Do not send Nav2 goals until heartbeat confirms arm + GUIDED (avoids orphan path in RViz).
        if not ensure_armed_and_mode_guided(
            executor, node, arm_pub, mode_pub, hb_state
        ):
            print(
                'Pixhawk not ready for mission; skipping Nav2 goal (no path published).',
                file=sys.stderr,
            )
            publish_manual_and_spin(executor, node, mode_pub, spins=20)
            publish_disarm_and_spin(executor, node, arm_pub, spins=20)
            sys.exit(1)

        print('>>> Executing mission <<<')
        mission_result = send_goal(executor, follow_waypoints, goal, node, mode_pub, arm_pub)

        if mission_result == SendGoalResult.SUCCESS and rclpy.ok():
            print('>>> Disarming <<<')
            arm_pub.publish(Bool(data=False))
            time.sleep(0.5)
            print('>>> Setting Pixhawk mode to MANUAL <<<')
            mode_pub.publish(String(data='MANUAL'))
            rclpy.spin_once(node, timeout_sec=0.2)
        elif mission_result == SendGoalResult.CANCELED and rclpy.ok():
            # send_goal already sent MANUAL + disarm; optional flush if drops occurred.
            print('>>> Post-cancel flush (MANUAL + disarm) <<<')
            publish_manual_and_spin(executor, node, mode_pub, spins=15)
            publish_disarm_and_spin(executor, node, arm_pub, spins=20)

        print('>>> Mission complete <<<')

    except KeyboardInterrupt:
        # Interrupt before goal accepted, or re-raised from send_goal — MANUAL then disarm.
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
