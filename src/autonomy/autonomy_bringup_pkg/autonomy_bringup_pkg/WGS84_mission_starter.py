#!/usr/bin/env python3
"""
Send Nav2 waypoints from ``missions/default_wgs84_mission.csv``.

Origin is read from /ubx_nav_hp_pos_llh (raw u-blox high-precision fix) so that the
WGS84->ENU conversion uses the same altitude datum (MSL, from UBX hmsl) as the
Pixhawk's GPS_GLOBAL_ORIGIN set by mavlink_bridge. Override: ``--origin=lat,lon,alt``
and/or ``--file path.csv``.
"""

from enum import Enum
import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from mavros_msgs.msg import State as PixhawkHeartbeat
from std_msgs.msg import Bool, String
from ublox_ubx_msgs.msg import UBXNavHPPosLLH

from config_pkg.constants import GpsOriginConditions
from autonomy_bringup_pkg.load_wgs84_points_to_waypoints import process_coordinates
from autonomy_bringup_pkg.nav2_ready_wait import wait_for_waypoint_follower_active
from autonomy_bringup_pkg.pixhawk_ready_wait import (
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


def default_mission_csv_path() -> str:
    share = get_package_share_directory('autonomy_bringup_pkg')
    return f'{share}/missions/goldbach_straightline_wgs84_mission.csv'


# RTK gate — mirrors gnss_datum_watchdog (decides when map->odom becomes
# active) and ros2_receiver (decides when Pixhawk GPS_GLOBAL_ORIGIN is set),
# so the mission CSV's ENU origin lands on the same lat/lon as those two.
# UBX-NAV-HPPOSLLH h_acc is in 0.1 mm units.
_GPS_ORIGIN_H_ACC_MAX_M = GpsOriginConditions.GPS_ORIGIN_H_ACC_MAX_M
_GPS_ORIGIN_H_ACC_TO_M = GpsOriginConditions.GPS_ORIGIN_H_ACC_TO_M
_GPS_ORIGIN_NULL_ISLAND_E7 = GpsOriginConditions.GPS_ORIGIN_NULL_ISLAND_E7
_GPS_ORIGIN_FALLBACK_S = GpsOriginConditions.GPS_ORIGIN_FALLBACK_S


def wait_for_gps_origin(
    executor, node, *, timeout_sec: float = 120.0, required_fixes: int = 5
) -> tuple | None:
    """Spin until /ubx_nav_hp_pos_llh yields an RTK-quality fix; returns (lat_deg, lon_deg, alt_msl_m) or None.

    Primary gate (single-shot, mirrors gnss_datum_watchdog):
      - invalid_lon / invalid_lat / invalid_hmsl bits all clear
      - |lat| > 0.1° and |lon| > 0.1° (null-island guard)
      - 0 < h_acc ≤ 0.50 m

    Soft fallback: if 30 s pass with clean-bits messages but h_acc never
    satisfies the threshold, warn once and accept the next `required_fixes`
    consecutive clean-bits messages (the old behaviour). Lets the mission
    start with a degraded origin — but the warning makes it obvious that
    the origin no longer matches the EKF datum.

    Altitude is UBX hmsl (height above mean sea level) — matches the datum
    mavlink_bridge uses for GPS_GLOBAL_ORIGIN.
    """
    state = {
        'result': None,
        'gate': None,                     # 'RTK' or 'fallback' once chosen
        'first_valid_bits_t': 0.0,
        'fallback_consecutive': 0,
        'fallback_warned': False,
        'last_h_acc_m': float('nan'),
    }

    def _cb(msg: UBXNavHPPosLLH):
        if state['result'] is not None:
            return
        if msg.invalid_lon or msg.invalid_lat or msg.invalid_hmsl:
            state['fallback_consecutive'] = 0
            return
        if (
            abs(int(msg.lat)) < _GPS_ORIGIN_NULL_ISLAND_E7
            and abs(int(msg.lon)) < _GPS_ORIGIN_NULL_ISLAND_E7
        ):
            state['fallback_consecutive'] = 0
            return

        now_s = time.time()
        if state['first_valid_bits_t'] == 0.0:
            state['first_valid_bits_t'] = now_s

        h_acc_m = float(msg.h_acc) * _GPS_ORIGIN_H_ACC_TO_M
        state['last_h_acc_m'] = h_acc_m
        rtk_ok = 0.0 < h_acc_m <= _GPS_ORIGIN_H_ACC_MAX_M

        if rtk_ok:
            # UBX units: lat/lon are deg * 1e7 (int32); hmsl is mm (int32).
            state['gate'] = 'RTK'
            state['result'] = (
                msg.lat * 1e-7,
                msg.lon * 1e-7,
                msg.hmsl * 1e-3,
            )
            return

        # h_acc gate failed — check fallback window.
        elapsed = now_s - state['first_valid_bits_t']
        if elapsed < _GPS_ORIGIN_FALLBACK_S:
            return
        if not state['fallback_warned']:
            print(
                f'WARN: h_acc still > {_GPS_ORIGIN_H_ACC_MAX_M:.2f} m after '
                f'{elapsed:.0f}s (latest h_acc={h_acc_m:.2f}m); falling back to '
                f'bits-only check ({required_fixes} consecutive valid messages). '
                'Mission origin will NOT match the EKF datum — vehicle will hit a '
                'point offset by however far this fix is from the true RTK position.'
            )
            state['fallback_warned'] = True
        state['fallback_consecutive'] += 1
        if state['fallback_consecutive'] >= required_fixes:
            state['gate'] = 'fallback'
            state['result'] = (
                msg.lat * 1e-7,
                msg.lon * 1e-7,
                msg.hmsl * 1e-3,
            )

    sub = node.create_subscription(UBXNavHPPosLLH, '/ubx_nav_hp_pos_llh', _cb, 10)
    deadline = time.time() + timeout_sec
    print(
        f'Waiting for RTK origin from /ubx_nav_hp_pos_llh '
        f'(h_acc ≤ {_GPS_ORIGIN_H_ACC_MAX_M * 100:.0f} cm, '
        f'{_GPS_ORIGIN_FALLBACK_S:.0f}s fallback to bits-only, up to {timeout_sec:.0f}s)...'
    )
    while rclpy.ok() and time.time() < deadline:
        if state['result'] is not None:
            break
        executor.spin_once(timeout_sec=0.1)
    node.destroy_subscription(sub)
    if state['result'] is None:
        print(
            f'Timed out after {timeout_sec:.0f}s waiting for /ubx_nav_hp_pos_llh '
            f'(latest h_acc={state["last_h_acc_m"]:.2f}m, '
            f'fallback {state["fallback_consecutive"]}/{required_fixes}).'
        )
        return None
    lat, lon, alt = state['result']
    print(
        f'Origin ({state["gate"]}, MSL): lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f}m, '
        f'h_acc={state["last_h_acc_m"]:.2f}m'
    )
    return state['result']


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
              status_pub=None, total_waypoints: int = 0, poses=None) -> SendGoalResult:
    goal_handle = None

    def feedback_cb(feedback_msg):
        wp = feedback_msg.feedback.current_waypoint
        if status_pub is not None:
            status_pub.publish(String(data=f"{wp + 1}/{total_waypoints}"))
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
        executor.spin_until_future_complete(result_future, timeout_sec=3600.0)

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
            print('Goal was canceled externally')
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
        help='lat,lon,alt_m (default: read from /gps/filtered topic)',
    )
    parser.add_argument(
        '--file',
        type=str,
        default=None,
        help='Override CSV (default: share/.../missions/default_wgs84_mission.csv)',
    )

    args, ros_args = parser.parse_known_args()

    csv_path = args.file if args.file else default_mission_csv_path()

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

        if args.origin is not None:
            lat0, lon0, alt0 = args.origin
            print(f'Using origin from --origin: lat={lat0}, lon={lon0}, alt={alt0} m')
        else:
            origin = wait_for_gps_origin(executor, node)
            if origin is None:
                print('No EKF origin received; exiting.', file=sys.stderr)
                sys.exit(1)
            lat0, lon0, alt0 = origin

        print(f'Mission CSV: {csv_path}')
        try:
            poses = process_coordinates(csv_path, lat0, lon0, alt0)
        except (OSError, ValueError) as e:
            print(f'Error loading mission: {e}', file=sys.stderr)
            sys.exit(1)

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
                                   poses=poses)

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
