#!/usr/bin/env python3
"""Publish Nav2 lifecycle state on /diagnostics_software for Foxglove.

Kept separate from /diagnostics (hardware vitals: battery, ultrasonics,
temps, pixhawk heartbeat) so the two surfaces can be shown in their own
Foxglove panels.

Polls /<node>/get_state for every node managed by lifecycle_manager_navigation
at 1 Hz and emits a DiagnosticArray with a single 'Autonomy Stack' status
whose values list contains one row per node (level = worst row). Level mapping:

  ACTIVE                                                    -> OK
  CONFIGURING / ACTIVATING / DEACTIVATING / CLEANINGUP /
    SHUTTINGDOWN / ERRORPROCESSING (any transition state)   -> WARN
  UNCONFIGURED / INACTIVE / FINALIZED / UNKNOWN /
    GetState service unreachable                            -> ERROR

GetState requests are async with a per-call timeout so one wedged node cannot
block the others; if a node's last call is still pending when the timer ticks,
we publish its previous level instead of skipping the cycle.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState


_NODES = (
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
)
_PARENT_NAME = 'Autonomy Stack'
_PUBLISH_PERIOD_SEC = 1.0
_CALL_TIMEOUT_SEC = 2.0
_DIAG_TOPIC = '/diagnostics_software'

_TRANSITION_STATE_IDS = {
    LifecycleState.TRANSITION_STATE_CONFIGURING,
    LifecycleState.TRANSITION_STATE_CLEANINGUP,
    LifecycleState.TRANSITION_STATE_SHUTTINGDOWN,
    LifecycleState.TRANSITION_STATE_ACTIVATING,
    LifecycleState.TRANSITION_STATE_DEACTIVATING,
    LifecycleState.TRANSITION_STATE_ERRORPROCESSING,
}

_STATE_LABELS = {
    LifecycleState.PRIMARY_STATE_UNKNOWN: 'unknown',
    LifecycleState.PRIMARY_STATE_UNCONFIGURED: 'unconfigured',
    LifecycleState.PRIMARY_STATE_INACTIVE: 'inactive',
    LifecycleState.PRIMARY_STATE_ACTIVE: 'active',
    LifecycleState.PRIMARY_STATE_FINALIZED: 'finalized',
    LifecycleState.TRANSITION_STATE_CONFIGURING: 'configuring',
    LifecycleState.TRANSITION_STATE_CLEANINGUP: 'cleaningup',
    LifecycleState.TRANSITION_STATE_SHUTTINGDOWN: 'shuttingdown',
    LifecycleState.TRANSITION_STATE_ACTIVATING: 'activating',
    LifecycleState.TRANSITION_STATE_DEACTIVATING: 'deactivating',
    LifecycleState.TRANSITION_STATE_ERRORPROCESSING: 'errorprocessing',
}


def _state_label(state_id: Optional[int]) -> str:
    if state_id is None:
        return 'service unreachable'
    return _STATE_LABELS.get(state_id, f'state_id={state_id}')


def _level_for_state(state_id: Optional[int]) -> int:
    if state_id is None:
        return DiagnosticStatus.ERROR
    if state_id == LifecycleState.PRIMARY_STATE_ACTIVE:
        return DiagnosticStatus.OK
    if state_id in _TRANSITION_STATE_IDS:
        return DiagnosticStatus.WARN
    return DiagnosticStatus.ERROR


@dataclass
class _NodeTracker:
    name: str
    client: object
    last_state_id: Optional[int] = None
    pending_future: object = None
    pending_started_at: float = 0.0
    _last_level: int = field(default=DiagnosticStatus.ERROR)

    def kick_off_call(self) -> None:
        if self.pending_future is not None:
            return
        if not self.client.service_is_ready():
            self.last_state_id = None
            return
        future = self.client.call_async(GetState.Request())
        self.pending_future = future
        self.pending_started_at = time.monotonic()
        future.add_done_callback(self._on_response)

    def _on_response(self, future) -> None:
        try:
            result = future.result()
        except Exception:
            self.last_state_id = None
            self.pending_future = None
            return
        if result is None:
            self.last_state_id = None
        else:
            self.last_state_id = result.current_state.id
        self.pending_future = None

    def reap_if_stale(self) -> None:
        if self.pending_future is None:
            return
        if time.monotonic() - self.pending_started_at < _CALL_TIMEOUT_SEC:
            return
        self.pending_future = None
        self.last_state_id = None


class Nav2LifecycleDiagnostics(Node):

    def __init__(self) -> None:
        super().__init__('nav2_lifecycle_diagnostics')

        cb_group = MutuallyExclusiveCallbackGroup()
        self._trackers = [
            _NodeTracker(
                name=name,
                client=self.create_client(
                    GetState, f'/{name}/get_state', callback_group=cb_group
                ),
            )
            for name in _NODES
        ]

        self._diag_pub = self.create_publisher(DiagnosticArray, _DIAG_TOPIC, 10)
        self.create_timer(
            _PUBLISH_PERIOD_SEC, self._tick, callback_group=cb_group
        )

        self.get_logger().info(
            f'Publishing Nav2 lifecycle to {_DIAG_TOPIC} at '
            f'{1.0 / _PUBLISH_PERIOD_SEC:.1f} Hz for: {", ".join(_NODES)}'
        )

    def _tick(self) -> None:
        for tracker in self._trackers:
            tracker.reap_if_stale()
            tracker.kick_off_call()
        self._publish()

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()

        node_rows: list[KeyValue] = []
        worst_level = DiagnosticStatus.OK
        active_count = 0
        worst_node = None

        for tracker in self._trackers:
            state_id = tracker.last_state_id
            level = _level_for_state(state_id)
            label = _state_label(state_id)

            node_rows.append(KeyValue(
                key=tracker.name,
                value=label,
            ))

            if state_id == LifecycleState.PRIMARY_STATE_ACTIVE:
                active_count += 1
            if level > worst_level:
                worst_level = level
                worst_node = (tracker.name, label)

        parent = DiagnosticStatus()
        parent.name = _PARENT_NAME
        parent.level = worst_level
        parent.hardware_id = ''
        if worst_level == DiagnosticStatus.OK:
            parent.message = f'{active_count}/{len(self._trackers)} active'
        else:
            node_name, label = worst_node
            parent.message = f'{node_name}: {label}'
        node_rows.sort(key=lambda kv: kv.key)
        parent.values = [
            KeyValue(key='active_count', value=f'{active_count}/{len(self._trackers)}'),
            *node_rows,
        ]

        msg = DiagnosticArray()
        msg.header.stamp = now
        msg.status = [parent]
        self._diag_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = Nav2LifecycleDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
