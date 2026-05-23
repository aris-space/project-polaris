#!/usr/bin/env python3
"""
Send Nav2 waypoints from the mission CSV transformed at autonomy bringup.

By default, subscribes to the latched ``/mission_waypoints_enu`` topic
published by ``mission_waypoint_loader`` once an RTK-quality fix locks at
bringup. The same RTK gate as ``gnss_datum_watchdog`` is used, so the
waypoint origin matches the EKF datum and the Pixhawk's GPS_GLOBAL_ORIGIN.

This means the operator can hit "go" any time after autonomy bringup —
surface, underwater, hours later — because the transform happens once on
the surface, not at mission start.

Manual override (dry runs, replays): pass both ``--origin lat,lon,alt`` and
``--file path.csv`` to bypass the loader and transform locally.
"""

from enum import Enum
import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.signals import SignalHandlerOptions
from mavros_msgs.msg import State as PixhawkHeartbeat
from std_msgs.msg import Bool, String

from autonomy_bringup_pkg.load_wgs84_points_to_waypoints import process_coordinates
from autonomy_bringup_pkg.nav2_ready_wait import wait_for_waypoint_follower_active
from autonomy_bringup_pkg.pixhawk_ready_wait import (
    PixhawkState,
    ensure_armed_and_mode_guided,
    make_heartbeat_callback,
    mode_matches,
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


def _print_waypoint_summary(poses, csv_path: str) -> None:
    n = len(poses)
    print(f'Loaded {n} waypoints from {csv_path}')
    total_dist = 0.0
    for i, p in enumerate(poses):
        x = p.pose.position.x
        y = p.pose.position.y
        z = p.pose.position.z
        if i + 1 < n:
            nx = poses[i + 1].pose.position.x
            ny = poses[i + 1].pose.position.y
            nz = poses[i + 1].pose.position.z
            d = math.sqrt((nx - x) ** 2 + (ny - y) ** 2 + (nz - z) ** 2)
            total_dist += d
            print(f'  WP#{i+1:2d}: x={x:8.2f}m  y={y:8.2f}m  z={z:7.2f}m  →  {d:.1f}m to next')
        else:
            print(f'  WP#{i+1:2d}: x={x:8.2f}m  y={y:8.2f}m  z={z:7.2f}m  [last]')
    print(f'  Total path length: {total_dist:.1f}m')


_MISSION_WAYPOINTS_TOPIC = '/mission_waypoints_enu'


def wait_for_mission_waypoints(
    executor, node, *, timeout_sec: float = 120.0
) -> list | None:
    """Subscribe to the latched /mission_waypoints_enu and return the PoseStamped[] inside.

    mission_waypoint_loader (started by autonomy.launch.py) waits for an
    RTK-quality fix on the surface, transforms the mission CSV once, and
    latches the result on /mission_waypoints_enu (PoseArray, TRANSIENT_LOCAL).
    We just consume that — the vehicle can be on the surface, underwater, or
    anywhere in between by the time the operator runs this starter.

    The PoseArray frame_id (= 'map') is propagated to each PoseStamped so
    Nav2 plans in the same frame the loader anchored at the RTK datum.
    """
    state = {'poses': None}

    def _cb(msg: PoseArray) -> None:
        if state['poses'] is not None:
            return
        frame = msg.header.frame_id or 'map'
        poses = []
        for p in msg.poses:
            ps = PoseStamped()
            ps.header.stamp = msg.header.stamp
            ps.header.frame_id = frame
            ps.pose = p
            poses.append(ps)
        state['poses'] = poses

    latched_qos = QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )
    sub = node.create_subscription(
        PoseArray, _MISSION_WAYPOINTS_TOPIC, _cb, latched_qos
    )

    deadline = time.time() + timeout_sec
    print(
        f'Waiting for latched mission waypoints on {_MISSION_WAYPOINTS_TOPIC} '
        f'(published by mission_waypoint_loader once RTK locks; up to {timeout_sec:.0f}s)...'
    )
    while rclpy.ok() and time.time() < deadline:
        if state['poses'] is not None:
            break
        executor.spin_once(timeout_sec=0.1)
    node.destroy_subscription(sub)

    if state['poses'] is None:
        print(
            f'Timed out after {timeout_sec:.0f}s waiting for {_MISSION_WAYPOINTS_TOPIC}. '
            f'Check that mission_waypoint_loader has logged an RTK lock '
            f'(ros2 topic echo --once {_MISSION_WAYPOINTS_TOPIC}).'
        )
        return None
    print(f'Received {len(state["poses"])} latched waypoints from {_MISSION_WAYPOINTS_TOPIC}.')
    return state['poses']


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


def send_goal(executor, action_client, send_goal_msg, node, mode_pub, arm_pub,
              status_pub=None, total_waypoints: int = 0, poses=None, hb_state=None) -> SendGoalResult:
    goal_handle = None
    _last_wp = [-1]

    def feedback_cb(feedback_msg):
        wp = feedback_msg.feedback.current_waypoint
        if status_pub is not None:
            status_pub.publish(String(data=f"{wp + 1}/{total_waypoints}"))
        if wp == _last_wp[0]:
            return
        _last_wp[0] = wp
        if poses is not None and wp < len(poses):
            p = poses[wp]
            x, y, z = p.pose.position.x, p.pose.position.y, p.pose.position.z
            print(f'  Heading to WP {wp+1}/{total_waypoints}: x={x:.2f}m  y={y:.2f}m  z={z:.2f}m')
        else:
            print(f'  Heading to WP {wp+1}/{total_waypoints}')

    try:
        if not wait_for_follow_waypoints(executor, action_client):
            return SendGoalResult.FAILURE

        print('Sending goal...')
        goal_future = action_client.send_goal_async(send_goal_msg, feedback_callback=feedback_cb)
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

        # Poll for mission result while monitoring Pixhawk state.  If the
        # pilot disarms or leaves GUIDED mid-mission we cancel the
        # FollowWaypoints goal immediately so waypoint_follower has no active
        # action when the nav2_arm_watchdog sends DEACTIVATE a moment later.
        # Without this, the lifecycle DEACTIVATE hits waypoint_follower while
        # it still has an active goal, causing it to crash (service unreachable)
        # and leaving all other Nav2 nodes stuck in ACTIVE.
        _cancel_sent = False
        _deadline = time.time() + 3600.0
        while rclpy.ok() and not result_future.done() and time.time() < _deadline:
            executor.spin_once(timeout_sec=0.1)
            if hb_state is not None and not _cancel_sent:
                if not hb_state.armed or not mode_matches(hb_state.mode, 'GUIDED'):
                    reasons = []
                    if not hb_state.armed:
                        reasons.append('disarmed')
                    if not mode_matches(hb_state.mode, 'GUIDED'):
                        reasons.append(f'mode={hb_state.mode or "?"}')
                    print(f'>>> Pilot intervention ({", ".join(reasons)}); '
                          'canceling mission goal before Nav2 deactivates <<<')
                    _cancel_sent = True
                    goal_handle.cancel_goal_async()

        if not result_future.done():
            print('Timeout waiting for mission result (Nav2 unreachable?)')
            return SendGoalResult.FAILURE

        result = result_future.result()

        if result is None:
            raise RuntimeError('Exception while getting result: {!r}'.format(result_future.exception()))

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            print('Goal completed')
            return SendGoalResult.SUCCESS
        elif status == GoalStatus.STATUS_CANCELED:
            print('Goal was canceled' + (' by pilot intervention' if _cancel_sent else ' externally'))
            return SendGoalResult.CANCELED
        else:
            print(f'Goal ended with status {status} (Nav2 may have shut down mid-mission)')
            return SendGoalResult.FAILURE

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
        help='lat,lon,alt_m manual override (default: consume latched /mission_waypoints_enu)',
    )
    parser.add_argument(
        '--file',
        type=str,
        default=None,
        help='Override CSV (default: share/.../missions/default_wgs84_mission.csv)',
    )

    args, ros_args = parser.parse_known_args()

    # Manual-override sanity: --origin and --file are an all-or-nothing pair.
    # Without both, you'd either transform a default CSV with an operator-
    # provided origin (footgun) or transform an operator CSV with the latched
    # loader origin (which means the loader already ran — just don't pass --file).
    if (args.origin is None) != (args.file is None):
        print(
            'Manual override requires BOTH --origin and --file (or neither, to '
            'consume the latched /mission_waypoints_enu published by '
            'mission_waypoint_loader at autonomy bringup).',
            file=sys.stderr,
        )
        sys.exit(1)

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
        )
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        follow_waypoints = ActionClient(node, FollowWaypoints, '/follow_waypoints')
        mode_pub = node.create_publisher(String, '/pixhawk/mode_cmd', 10)
        arm_pub = node.create_publisher(Bool, '/pixhawk/arm_cmd', 10)
        status_pub = node.create_publisher(String, '/mission_status', 10)

        hb_state = PixhawkState()
        node.create_subscription(
            PixhawkHeartbeat,
            '/pixhawk/heartbeat',
            make_heartbeat_callback(hb_state),
            10,
        )

        # Two paths:
        #   1. Default — consume the latched PoseArray published by
        #      mission_waypoint_loader, which transformed the CSV at autonomy
        #      bringup using the same RTK gate as gnss_datum_watchdog. Works
        #      whether the vehicle is on the surface or already underwater.
        #   2. Manual override (--origin AND --file together) — transform the
        #      CSV locally with the operator-provided origin. For dry runs and
        #      replays; bypasses the RTK gate.
        if args.origin is not None:
            lat0, lon0, alt0 = args.origin
            csv_path = args.file
            print(f'Manual override: --origin lat={lat0}, lon={lon0}, alt={alt0} m')
            print(f'Mission CSV: {csv_path}')
            try:
                poses = process_coordinates(csv_path, lat0, lon0, alt0)
            except (OSError, ValueError) as e:
                print(f'Error loading mission: {e}', file=sys.stderr)
                sys.exit(1)
        else:
            poses = wait_for_mission_waypoints(executor, node)
            if poses is None:
                print(
                    'No latched waypoints received from mission_waypoint_loader; exiting.',
                    file=sys.stderr,
                )
                sys.exit(1)
            csv_path = '<from mission_waypoint_loader>'

        goal = FollowWaypoints.Goal()
        goal.poses = poses

        _print_waypoint_summary(poses, csv_path)

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
        mission_result = send_goal(executor, follow_waypoints, goal, node, mode_pub, arm_pub,
                                   status_pub=status_pub, total_waypoints=len(goal.poses),
                                   poses=poses, hb_state=hb_state)

        if mission_result == SendGoalResult.SUCCESS and rclpy.ok():
            print('>>> Disarming <<<')
            arm_pub.publish(Bool(data=False))
            time.sleep(0.5)
            print('>>> Setting Pixhawk mode to MANUAL <<<')
            mode_pub.publish(String(data='MANUAL'))
            executor.spin_once(timeout_sec=0.2)
        elif mission_result == SendGoalResult.FAILURE and rclpy.ok():
            # Nav2 died mid-mission; attempt best-effort disarm + MANUAL.
            print('>>> Mission aborted; attempting disarm + MANUAL <<<')
            publish_manual_and_spin(executor, node, mode_pub, spins=20)
            publish_disarm_and_spin(executor, node, arm_pub, spins=20)
        elif mission_result == SendGoalResult.CANCELED and rclpy.ok():
            # send_goal already sent MANUAL + disarm; optional flush if drops occurred.
            print('>>> Post-cancel flush (MANUAL + disarm) <<<')
            publish_manual_and_spin(executor, node, mode_pub, spins=15)
            publish_disarm_and_spin(executor, node, arm_pub, spins=20)

        print('>>> Mission complete <<<')

    except KeyboardInterrupt:
        # Interrupt before goal accepted, or re-raised from send_goal - MANUAL then disarm.
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
