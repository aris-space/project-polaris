#!/usr/bin/env python3
"""Activate the Nav2 stack via lifecycle_manager_navigation.

Sends STARTUP (command=0) to /lifecycle_manager_navigation/manage_nodes, then
waits until /waypoint_follower reaches active.

Usage (after `ros2 launch config_pkg start_system.launch.py autonomy:=true`):
    ros2 run autonomy_bringup_pkg nav2_activate.py
"""

import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes

_SERVICE = '/lifecycle_manager_navigation/manage_nodes'
_STARTUP_CMD = 0
_CALL_TIMEOUT_SEC = 90.0
_ACTIVE_TIMEOUT_SEC = 180.0
_POLL_SEC = 0.25


def _wait_for_active(executor, node) -> bool:
    """Poll /waypoint_follower until PRIMARY_STATE_ACTIVE or timeout."""
    client = node.create_client(GetState, '/waypoint_follower/get_state')
    deadline = time.time() + _ACTIVE_TIMEOUT_SEC
    print('Waiting for Nav2 lifecycle: /waypoint_follower -> active...')

    while rclpy.ok() and time.time() < deadline:
        if client.service_is_ready():
            break
        executor.spin_once(timeout_sec=_POLL_SEC)
    else:
        print(f'Timed out waiting for /waypoint_follower/get_state ({_ACTIVE_TIMEOUT_SEC:.0f}s). '
              'Is bringup running with autonomy:=true?')
        return False

    last_log = 0.0
    while rclpy.ok() and time.time() < deadline:
        future = client.call_async(GetState.Request())
        executor.spin_until_future_complete(future, timeout_sec=5.0)
        if not future.done():
            executor.spin_once(timeout_sec=_POLL_SEC)
            continue
        try:
            state_id = future.result().current_state.id
        except Exception:
            executor.spin_once(timeout_sec=_POLL_SEC)
            continue
        if state_id == State.PRIMARY_STATE_ACTIVE:
            print('Nav2 is active - safe to send missions.')
            return True
        now = time.time()
        if now - last_log >= 10.0:
            print(f'  ... still waiting (state={state_id}); bt_navigator must activate first.')
            last_log = now
        executor.spin_once(timeout_sec=_POLL_SEC)

    print(f'Timed out after {_ACTIVE_TIMEOUT_SEC:.0f}s: /waypoint_follower never reached active.\n'
          '  CLI: ros2 lifecycle get /bt_navigator ; ros2 lifecycle get /waypoint_follower')
    return False


def main():
    rclpy.init()
    node = rclpy.create_node('nav2_activate')
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    client = node.create_client(ManageLifecycleNodes, _SERVICE)
    print(f'Waiting for {_SERVICE}...')
    if not client.wait_for_service(timeout_sec=_CALL_TIMEOUT_SEC):
        print(f'Service not available after {_CALL_TIMEOUT_SEC:.0f}s. '
              'Is the autonomy stack running with autonomy:=true?')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    req = ManageLifecycleNodes.Request()
    req.command = _STARTUP_CMD
    print(
        'Sending STARTUP to lifecycle_manager_navigation...\n'
        '  WARNING: planner_server global costmap needs map→odom TF.\n'
        '  This TF is published by the global EKF, which starts only after GPS fix.\n'
        '  If no GPS fix yet, this call will hang until the fix is acquired.\n'
        f'  Waiting up to {_CALL_TIMEOUT_SEC:.0f}s for response...'
    )
    future = client.call_async(req)
    executor.spin_until_future_complete(future, timeout_sec=_CALL_TIMEOUT_SEC)

    if not future.done() or future.result() is None:
        print(
            f'STARTUP service call timed out after {_CALL_TIMEOUT_SEC:.0f}s.\n'
            '  Most likely cause: planner_server is waiting for map→odom TF (no GPS fix yet).\n'
            '  Diagnose node states:\n'
            '    ros2 lifecycle get /controller_server\n'
            '    ros2 lifecycle get /planner_server\n'
            '    ros2 lifecycle get /bt_navigator\n'
            '    ros2 lifecycle get /waypoint_follower\n'
            '  Check GPS stream:  ros2 topic hz /ubx_nav_hp_pos_llh\n'
            '  Check TF:          ros2 run tf2_tools view_frames'
        )
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    if not future.result().success:
        print('lifecycle_manager_navigation rejected STARTUP. Check node logs.')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print('STARTUP accepted - waiting for Nav2 to reach active...')
    ok = _wait_for_active(executor, node)

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
