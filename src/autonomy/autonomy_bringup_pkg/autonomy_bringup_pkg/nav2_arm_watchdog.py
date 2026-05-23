#!/usr/bin/env python3
"""Auto-deactivate Nav2 when the pilot disarms or leaves GUIDED mid-mission.

Subscribes to ``/pixhawk/heartbeat`` (mavros_msgs/State, published by
mavlink_bridge). The watchdog only becomes eligible to fire after it has
observed at least one heartbeat with armed=True AND mode=GUIDED while
Nav2 is ACTIVE - this confirms the operator actually entered the
autonomy-ready state. After that, if the heartbeat reports either
armed=False or a mode other than GUIDED, the watchdog calls
``/lifecycle_manager_navigation/manage_nodes`` with command=1 (DEACTIVATE).

This prevents controller_server / waypoint_follower from spamming
'Controller failed' / retry messages once the vehicle is no longer
under autonomy control.

The watchdog fires once per mission session and re-arms after Nav2 has
been re-activated AND the operator has returned to armed+GUIDED.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from action_msgs.srv import CancelGoal
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from mavros_msgs.msg import State as PixhawkHeartbeat
from nav2_msgs.srv import ManageLifecycleNodes

from autonomy_bringup_pkg.pixhawk_ready_wait import mode_matches


_MANAGE_SERVICE = '/lifecycle_manager_navigation/manage_nodes'
_GET_STATE_SERVICE = '/waypoint_follower/get_state'
_FW_CANCEL_SERVICE = '/follow_waypoints/_action/cancel_goal'
_DEACTIVATE_CMD = 1
_REQUIRED_MODE = 'GUIDED'
_STATE_POLL_SEC = 1.0
_STATE_CALL_TIMEOUT_SEC = 5.0


class Nav2ArmWatchdog(Node):

    def __init__(self) -> None:
        super().__init__('nav2_arm_watchdog')

        self._fired = False
        self._armed_to_fire = False
        self._nav2_active = False
        self._pending_state_future = None
        self._pending_state_started_at = 0.0

        cb_group = MutuallyExclusiveCallbackGroup()

        # Match default QoS used by mavlink_bridge's heartbeat_publisher
        # (RELIABLE + KEEP_LAST(10) + VOLATILE); using BEST_EFFORT here
        # would silently drop every message.
        self.create_subscription(
            PixhawkHeartbeat,
            '/pixhawk/heartbeat',
            self._on_heartbeat,
            10,
            callback_group=cb_group,
        )

        self._manage_client = self.create_client(
            ManageLifecycleNodes, _MANAGE_SERVICE, callback_group=cb_group
        )
        self._get_state_client = self.create_client(
            GetState, _GET_STATE_SERVICE, callback_group=cb_group
        )
        self._fw_cancel_client = self.create_client(
            CancelGoal, _FW_CANCEL_SERVICE, callback_group=cb_group
        )

        self.create_timer(_STATE_POLL_SEC, self._poll_nav2_state, callback_group=cb_group)

        self.get_logger().info(
            f'Watching /pixhawk/heartbeat; will DEACTIVATE Nav2 on disarm or mode != {_REQUIRED_MODE} '
            'once the vehicle has been observed armed+GUIDED with Nav2 active.'
        )

    def _poll_nav2_state(self) -> None:
        if self._pending_state_future is not None:
            if self._pending_state_future.done():
                self._pending_state_future = None
            elif time.monotonic() - self._pending_state_started_at < _STATE_CALL_TIMEOUT_SEC:
                return
            else:
                # Stale future: drop it and start a fresh poll next tick.
                self._pending_state_future = None
                return

        if not self._get_state_client.service_is_ready():
            # Service not up yet (Nav2 still launching). Leave _nav2_active
            # at its last known value rather than forcing False, which would
            # cause us to miss a trigger if the service blips after startup.
            return

        future = self._get_state_client.call_async(GetState.Request())
        self._pending_state_future = future
        self._pending_state_started_at = time.monotonic()
        future.add_done_callback(self._on_state_response)

    def _on_state_response(self, future) -> None:
        try:
            result = future.result()
        except Exception:
            return
        if result is None:
            return
        was_active = self._nav2_active
        self._nav2_active = result.current_state.id == LifecycleState.PRIMARY_STATE_ACTIVE

        if self._fired and self._nav2_active and not was_active:
            # Operator re-activated Nav2 after a previous trigger. Require
            # them to also re-enter armed+GUIDED before we can fire again.
            self._fired = False
            self._armed_to_fire = False
            self.get_logger().info('Nav2 ACTIVE again; watchdog awaiting armed+GUIDED to re-arm.')

    def _on_heartbeat(self, msg: PixhawkHeartbeat) -> None:
        armed = bool(msg.armed)
        in_guided = mode_matches(msg.mode or '', _REQUIRED_MODE)

        if not self._armed_to_fire:
            if self._nav2_active and armed and in_guided:
                self._armed_to_fire = True
                self.get_logger().info(
                    f'Observed armed+{_REQUIRED_MODE} with Nav2 active; watchdog now armed.'
                )
            return

        if self._fired or not self._nav2_active:
            return

        if armed and in_guided:
            return

        reasons = []
        if not armed:
            reasons.append('disarmed')
        if not in_guided:
            reasons.append(f'mode={msg.mode or "?"}')
        self.get_logger().warn(
            f'Pilot intervention detected ({", ".join(reasons)}); deactivating Nav2.'
        )
        self._fired = True
        self._send_deactivate()

    def _send_deactivate(self) -> None:
        """Cancel any active /follow_waypoints goals first, then deactivate Nav2.

        Sending DEACTIVATE while waypoint_follower has an active FollowWaypoints
        action goal causes it to crash (service unreachable) because on_deactivate
        hits an in-flight goal while bt_navigator is still active.  Canceling first
        gives waypoint_follower a clean state before the lifecycle transition.
        """
        if self._fw_cancel_client.service_is_ready():
            self.get_logger().info(
                'Canceling active FollowWaypoints goals before deactivating Nav2.'
            )
            future = self._fw_cancel_client.call_async(CancelGoal.Request())
            future.add_done_callback(self._on_fw_cancel_done)
        else:
            self._do_deactivate()

    def _on_fw_cancel_done(self, future) -> None:
        try:
            result = future.result()
            if result is not None and result.goals_canceling:
                self.get_logger().info(
                    f'{len(result.goals_canceling)} FollowWaypoints goal(s) canceling; '
                    'proceeding with Nav2 deactivation.'
                )
        except Exception as exc:
            self.get_logger().warn(f'FollowWaypoints cancel raised: {exc}')
        self._do_deactivate()

    def _do_deactivate(self) -> None:
        if not self._manage_client.service_is_ready():
            self.get_logger().error(
                f'{_MANAGE_SERVICE} not ready; cannot deactivate Nav2.'
            )
            return
        req = ManageLifecycleNodes.Request()
        req.command = _DEACTIVATE_CMD
        future = self._manage_client.call_async(req)
        future.add_done_callback(self._on_deactivate_done)

    def _on_deactivate_done(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'DEACTIVATE call raised: {exc}')
            return
        if result is None or not result.success:
            self.get_logger().error('lifecycle_manager_navigation rejected DEACTIVATE.')
            return
        self.get_logger().info('Nav2 DEACTIVATE accepted.')


def main() -> None:
    rclpy.init()
    node = Nav2ArmWatchdog()
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
