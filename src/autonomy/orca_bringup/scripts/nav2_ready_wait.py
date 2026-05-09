#!/usr/bin/env python3
"""Wait until Nav2 waypoint_follower is lifecycle-active (whole stack is up).

lifecycle_manager_navigation activates nodes in order: controller_server, planner_server,
behavior_server, bt_navigator, waypoint_follower. If an earlier node never reaches active
(e.g. bt_navigator fails to load a BT plugin .so or parse the behavior tree XML),
waypoint_follower stays inactive forever — that is not fixed by adding more package.xml
dependencies; check the failing node's logs and ``ros2 lifecycle get /bt_navigator``.
"""

import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState

_LIFECYCLE_LABELS = {
    State.PRIMARY_STATE_UNKNOWN: 'unknown',
    State.PRIMARY_STATE_UNCONFIGURED: 'unconfigured',
    State.PRIMARY_STATE_INACTIVE: 'inactive',
    State.PRIMARY_STATE_ACTIVE: 'active',
    State.PRIMARY_STATE_FINALIZED: 'finalized',
}


def _lifecycle_label(state_id: int) -> str:
    return _LIFECYCLE_LABELS.get(state_id, f'state_id={state_id}')


def _get_state_once(executor, node, service_name: str, wait_ready_sec: float = 2.0):
    """Return current_state.id or None if service missing / call failed."""
    client = node.create_client(GetState, service_name)
    deadline = time.time() + wait_ready_sec
    while rclpy.ok() and time.time() < deadline:
        if client.service_is_ready():
            break
        executor.spin_once(timeout_sec=0.05)
    else:
        return None
    future = client.call_async(GetState.Request())
    executor.spin_until_future_complete(future, timeout_sec=5.0)
    if not future.done():
        return None
    try:
        return future.result().current_state.id
    except Exception:
        return None


def wait_for_waypoint_follower_active(
    executor,
    node,
    *,
    timeout_sec: float = 180.0,
    poll_sec: float = 0.25,
) -> bool:
    """
    Poll /waypoint_follower/get_state until PRIMARY_STATE_ACTIVE.

    waypoint_follower is the last node in lifecycle_manager_navigation; when it is active,
    /follow_waypoints exists and the stack is ready for missions.
    """
    client = node.create_client(GetState, '/waypoint_follower/get_state')
    deadline = time.time() + timeout_sec
    print('Waiting for Nav2 lifecycle: /waypoint_follower -> active...')

    while rclpy.ok() and time.time() < deadline:
        if client.service_is_ready():
            break
        executor.spin_once(timeout_sec=poll_sec)
    else:
        print(
            f'Timed out waiting for /waypoint_follower/get_state service ({timeout_sec:.0f}s). '
            'Is bringup running with nav:=true?'
        )
        return False

    last_progress_log = 0.0
    while rclpy.ok() and time.time() < deadline:
        request = GetState.Request()
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=5.0)
        if not future.done():
            executor.spin_once(timeout_sec=poll_sec)
            continue
        try:
            response = future.result()
        except Exception as e:
            print(f'GetState call failed: {e}')
            executor.spin_once(timeout_sec=poll_sec)
            continue
        if response is None:
            executor.spin_once(timeout_sec=poll_sec)
            continue
        if response.current_state.id == State.PRIMARY_STATE_ACTIVE:
            print('Nav2 /waypoint_follower is active; safe to send missions.')
            return True
        now = time.time()
        if now - last_progress_log >= 10.0:
            wf = _lifecycle_label(response.current_state.id)
            print(
                f'  ... still waiting: /waypoint_follower is {wf} '
                f'(bt_navigator must activate first; see lifecycle order in module docstring).'
            )
            last_progress_log = now
        executor.spin_once(timeout_sec=poll_sec)

    wf_id = _get_state_once(executor, node, '/waypoint_follower/get_state')
    bt_id = _get_state_once(executor, node, '/bt_navigator/get_state')
    cs_id = _get_state_once(executor, node, '/controller_server/get_state')
    print(
        f'Timed out after {timeout_sec:.0f}s: /waypoint_follower never reached active.\n'
        f'  Snapshot: waypoint_follower={_lifecycle_label(wf_id) if wf_id is not None else "n/a"}, '
        f'bt_navigator={_lifecycle_label(bt_id) if bt_id is not None else "n/a"}, '
        f'controller_server={_lifecycle_label(cs_id) if cs_id is not None else "n/a"}\n'
        '  If bt_navigator is unconfigured/inactive: check its terminal output for BT XML / plugin '
        'load errors (custom nodes need libis_path_valid_check.so and libis_sharp_turn_check.so '
        'on LD_LIBRARY_PATH via '
        '`source install/setup.bash`).\n'
        '  CLI: ros2 lifecycle get /bt_navigator ; ros2 lifecycle get /waypoint_follower'
    )
    return False
