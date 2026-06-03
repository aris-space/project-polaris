#!/usr/bin/env python3
"""Publish Nav2 lifecycle state on /diagnostics_software for Foxglove.

Kept separate from /diagnostics (hardware vitals: battery, ultrasonics,
temps, pixhawk heartbeat) so the two surfaces can be shown in their own
Foxglove panels.

Polls /<node>/get_state for every node managed by lifecycle_manager_navigation
at 1 Hz and emits a DiagnosticArray with a single 'Autonomy Stack' status
whose values list contains one row per node (level = worst row). Level mapping
for the lifecycle-managed Nav2 nodes:

  ACTIVE                                                    -> OK
  CONFIGURING / ACTIVATING / DEACTIVATING / CLEANINGUP /
    SHUTTINGDOWN / ERRORPROCESSING (any transition state)   -> WARN
  UNCONFIGURED / INACTIVE / FINALIZED / UNKNOWN /
    GetState service unreachable                            -> ERROR

We also include a row for mission_waypoint_loader, which is not a lifecycle
node. Its readiness is inferred from two latched topics: /gnss_datum (published
once by gnss_datum_watchdog on RTK lock) and /mission_waypoints_enu (published
once by the loader after the CSV is transformed against that datum):

  /mission_waypoints_enu received                           -> OK
  /gnss_datum not yet seen                                  -> WARN ('waiting for datum')
  /gnss_datum seen, waypoints absent, within grace window   -> WARN ('datum locked, loading...')
  /gnss_datum seen, waypoints absent, past grace window     -> ERROR (CSV load failed)

Finally we include a 'cmd_vel_output' row that watches /pixhawk/cmd_vel
(controller_server's remapped output) and cross-references whether the Nav2
stack is fully active. This catches the case where every lifecycle node
reports ACTIVE but the controller still isn't producing setpoints:

  Twist seen within freshness window                        -> OK
  no recent Twist AND any Nav2 node not ACTIVE              -> WARN
  no recent Twist AND all Nav2 nodes ACTIVE                 -> ERROR

GetState requests are async with a per-call timeout so one wedged node cannot
block the others; if a node's last call is still pending when the timer ticks,
we publish its previous level instead of skipping the cycle.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Optional

import rclpy
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from action_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseArray, Twist
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from sensor_msgs.msg import NavSatFix


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
_MISSION_LOADER_KEY = 'mission_waypoint_loader'
_MISSION_WAYPOINTS_TOPIC = '/mission_waypoints_enu'
_GNSS_DATUM_TOPIC = '/gnss_datum'
# Grace window after the datum arrives during which "no waypoints yet" is
# still WARN (CSV processing in flight). Past this window with datum present
# but no waypoints, the loader's except branch fired and we go ERROR.
_CSV_LOAD_GRACE_SEC = 5.0

_CMD_VEL_KEY = 'cmd_vel_output'
_CMD_VEL_TOPIC = '/pixhawk/cmd_vel'

# Goal-completion row: informational only. Watches the waypoint_follower
# action server's status topic so we can report whether the current mission
# goal is executing / succeeded / aborted / canceled. Deliberately excluded
# from worst_level and active_count — an aborted goal is operationally
# important but the autonomy *stack* itself is still healthy.
_GOAL_STATUS_KEY = 'goal_status'
_GOAL_STATUS_TOPIC = '/follow_waypoints/_action/status'
# Freshness window: if no Twist arrived in the last _CMD_VEL_FRESH_SEC, the
# controller has stopped publishing. Nav2 controller_server typically runs at
# 20 Hz, so 1 s is comfortably above the period.
_CMD_VEL_FRESH_SEC = 1.0
# Ring buffer depth for the publish-rate readout shown on the OK row.
_CMD_VEL_RATE_WINDOW = 20

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


@dataclass
class _MissionLoaderTracker:
    """Tracks mission_waypoint_loader by watching its two latched topics.

    Subscribes (TRANSIENT_LOCAL + RELIABLE, matching the publishers) to:
      * /gnss_datum         — published once by gnss_datum_watchdog on RTK lock.
      * /mission_waypoints_enu — published once by the loader after the CSV
                                 is transformed against that datum.

    Combining the two signals lets us distinguish 'still waiting for datum'
    (normal during NTRIP cold-start) from 'datum locked but no waypoints'
    (the loader's CSV-load except branch fired — see mission_waypoint_loader
    _on_datum). A short grace window absorbs the time the loader spends
    inside process_coordinates before classifying the silence as a failure.
    """

    waypoint_count: Optional[int] = None
    datum_seen_at: Optional[float] = None

    def on_waypoints(self, msg: PoseArray) -> None:
        self.waypoint_count = len(msg.poses)

    def on_datum(self, _msg: NavSatFix) -> None:
        if self.datum_seen_at is None:
            self.datum_seen_at = time.monotonic()

    def _datum_age_s(self) -> Optional[float]:
        if self.datum_seen_at is None:
            return None
        return time.monotonic() - self.datum_seen_at

    def level(self) -> int:
        if self.waypoint_count is not None:
            return DiagnosticStatus.OK
        age = self._datum_age_s()
        if age is not None and age >= _CSV_LOAD_GRACE_SEC:
            return DiagnosticStatus.ERROR
        return DiagnosticStatus.WARN

    def label(self) -> str:
        if self.waypoint_count is not None:
            return f'loaded ({self.waypoint_count} waypoints)'
        age = self._datum_age_s()
        if age is None:
            return 'waiting for datum'
        if age < _CSV_LOAD_GRACE_SEC:
            return 'datum locked, loading...'
        return 'CSV load failed (datum locked but no waypoints)'


@dataclass
class _CmdVelTracker:
    """Tracks whether controller_server is publishing /pixhawk/cmd_vel.

    Level is decided by the freshness of the most recent Twist combined with
    whether the Nav2 stack is currently fully active:

      Twist seen within _CMD_VEL_FRESH_SEC                -> OK
      no recent Twist AND not all Nav2 nodes active       -> WARN (stack not active)
      no recent Twist AND all Nav2 nodes active           -> ERROR (stack
        active but no cmd_vel — controller is wedged or unable to publish)

    Rate (Hz) is computed from a small ring buffer of arrival timestamps so
    the OK row gives the operator a quick at-a-glance throughput readout.
    """

    timestamps: Deque[float] = field(
        default_factory=lambda: deque(maxlen=_CMD_VEL_RATE_WINDOW)
    )

    def on_twist(self, _msg: Twist) -> None:
        self.timestamps.append(time.monotonic())

    def _is_fresh(self) -> bool:
        if not self.timestamps:
            return False
        return time.monotonic() - self.timestamps[-1] < _CMD_VEL_FRESH_SEC

    def _rate_hz(self) -> Optional[float]:
        if len(self.timestamps) < 2:
            return None
        span = self.timestamps[-1] - self.timestamps[0]
        if span <= 0:
            return None
        return (len(self.timestamps) - 1) / span

    def level(self, nav2_all_active: bool) -> int:
        if self._is_fresh():
            return DiagnosticStatus.OK
        if nav2_all_active:
            return DiagnosticStatus.ERROR
        return DiagnosticStatus.WARN

    def label(self, nav2_all_active: bool) -> str:
        if self._is_fresh():
            rate = self._rate_hz()
            if rate is None:
                return 'publishing'
            return f'publishing ({rate:.1f} Hz)'
        if nav2_all_active:
            return 'stack active but no cmd_vel'
        return 'waiting (stack not active)'


@dataclass
class _GoalStatusTracker:
    """Tracks the latest /follow_waypoints goal via its action status topic.

    Action servers publish a GoalStatusArray on `<action>/_action/status`; we
    pick the most-recently-updated entry and surface its terminal/active
    state. Purely informational — never contributes to the parent rollup.
    """

    last_status: Optional[int] = None

    def on_status(self, msg: GoalStatusArray) -> None:
        if not msg.status_list:
            return
        # status_list can carry several goals (succeeded ones linger briefly);
        # the freshest stamp wins so we always reflect the active mission.
        latest = max(
            msg.status_list,
            key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec),
        )
        self.last_status = latest.status

    def label(self) -> str:
        if self.last_status is None:
            return 'no goal yet'
        return {
            GoalStatus.STATUS_UNKNOWN: 'unknown',
            GoalStatus.STATUS_ACCEPTED: 'accepted',
            GoalStatus.STATUS_EXECUTING: 'executing',
            GoalStatus.STATUS_CANCELING: 'canceling',
            GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_CANCELED: 'canceled',
            GoalStatus.STATUS_ABORTED: 'aborted',
        }.get(self.last_status, f'status={self.last_status}')


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

        self._mission_loader = _MissionLoaderTracker()
        # Match the loader's QoS exactly (TRANSIENT_LOCAL + RELIABLE) so we
        # still receive the latched PoseArray when we start after the loader
        # has already published.
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            PoseArray,
            _MISSION_WAYPOINTS_TOPIC,
            self._mission_loader.on_waypoints,
            latched_qos,
            callback_group=cb_group,
        )
        self.create_subscription(
            NavSatFix,
            _GNSS_DATUM_TOPIC,
            self._mission_loader.on_datum,
            latched_qos,
            callback_group=cb_group,
        )

        # /pixhawk/cmd_vel is a regular 20 Hz Twist stream — default QoS
        # (RELIABLE + VOLATILE, depth 10) matches the Nav2 controller_server
        # publisher.
        #
        # Put this subscription on a SEPARATE reentrant group so the 20 Hz
        # Twist callbacks never queue behind the GetState async futures or
        # the 1 Hz timer running in the MutEx group above. Otherwise a slow
        # tick can delay timestamp updates past _CMD_VEL_FRESH_SEC and we
        # would falsely report 'stack active but no cmd_vel'.
        cmd_vel_cb_group = ReentrantCallbackGroup()
        self._cmd_vel = _CmdVelTracker()
        self.create_subscription(
            Twist,
            _CMD_VEL_TOPIC,
            self._cmd_vel.on_twist,
            10,
            callback_group=cmd_vel_cb_group,
        )

        # Action status topics use the default action QoS profile (RELIABLE,
        # depth 1, VOLATILE) — default subscriber QoS is compatible.
        self._goal_status = _GoalStatusTracker()
        self.create_subscription(
            GoalStatusArray,
            _GOAL_STATUS_TOPIC,
            self._goal_status.on_status,
            10,
            callback_group=cb_group,
        )

        self._diag_pub = self.create_publisher(DiagnosticArray, _DIAG_TOPIC, 10)
        self.create_timer(
            _PUBLISH_PERIOD_SEC, self._tick, callback_group=cb_group
        )

        self.get_logger().info(
            f'Publishing Nav2 lifecycle to {_DIAG_TOPIC} at '
            f'{1.0 / _PUBLISH_PERIOD_SEC:.1f} Hz for: {", ".join(_NODES)} '
            f'(+ {_MISSION_LOADER_KEY} via {_MISSION_WAYPOINTS_TOPIC}, '
            f'+ {_CMD_VEL_KEY} via {_CMD_VEL_TOPIC})'
        )

    def _tick(self) -> None:
        for tracker in self._trackers:
            tracker.reap_if_stale()
            tracker.kick_off_call()
        self._publish()

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()

        nav2_rows: list[KeyValue] = []
        worst_level = DiagnosticStatus.OK
        active_count = 0
        worst_node = None

        for tracker in self._trackers:
            state_id = tracker.last_state_id
            level = _level_for_state(state_id)
            label = _state_label(state_id)

            nav2_rows.append(KeyValue(
                key=tracker.name,
                value=label,
            ))

            if state_id == LifecycleState.PRIMARY_STATE_ACTIVE:
                active_count += 1
            if level > worst_level:
                worst_level = level
                worst_node = (tracker.name, label)

        nav2_all_active = active_count == len(self._trackers)

        loader_level = self._mission_loader.level()
        loader_label = self._mission_loader.label()
        loader_row = KeyValue(key=_MISSION_LOADER_KEY, value=loader_label)
        if loader_level == DiagnosticStatus.OK:
            active_count += 1
        if loader_level > worst_level:
            worst_level = loader_level
            worst_node = (_MISSION_LOADER_KEY, loader_label)

        cmd_vel_level = self._cmd_vel.level(nav2_all_active)
        cmd_vel_label = self._cmd_vel.label(nav2_all_active)
        cmd_vel_row = KeyValue(key=_CMD_VEL_KEY, value=cmd_vel_label)
        if cmd_vel_level == DiagnosticStatus.OK:
            active_count += 1
        if cmd_vel_level > worst_level:
            worst_level = cmd_vel_level
            worst_node = (_CMD_VEL_KEY, cmd_vel_label)

        # +1 for the mission loader, +1 for the cmd_vel output check.
        total = len(self._trackers) + 2

        parent = DiagnosticStatus()
        parent.name = _PARENT_NAME
        parent.level = worst_level
        parent.hardware_id = ''
        if worst_level == DiagnosticStatus.OK:
            parent.message = f'{active_count}/{total} active'
        else:
            node_name, label = worst_node
            parent.message = f'{node_name}: {label}'
        # Nav2 lifecycle nodes on top (A-Z), then mission loader, then the
        # cmd_vel output check at the very bottom (it's the outcome of the
        # rows above doing their job).
        nav2_rows.sort(key=lambda kv: kv.key)
        # goal_status is informational only — not counted in active_count,
        # not factored into worst_level, and not in `total`. Appended last so
        # the operator can still see mission outcome at a glance.
        goal_status_row = KeyValue(
            key=_GOAL_STATUS_KEY, value=self._goal_status.label()
        )
        parent.values = [
            KeyValue(key='active_count', value=f'{active_count}/{total}'),
            *nav2_rows,
            loader_row,
            cmd_vel_row,
            goal_status_row,
        ]

        msg = DiagnosticArray()
        msg.header.stamp = now
        msg.status = [parent]
        self._diag_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = Nav2LifecycleDiagnostics()
    # MultiThreadedExecutor is required for ReentrantCallbackGroup to actually
    # parallelise callbacks. Two threads is enough: one for the MutEx group
    # (timer + GetState futures + latched subscriptions) and one to keep the
    # 20 Hz cmd_vel subscription responsive.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
