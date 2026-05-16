#!/usr/bin/env python3
"""Auto-deactivate Nav2 when the pilot disarms or leaves GUIDED mid-mission.

Subscribes to ``/pixhawk/heartbeat`` (mavros_msgs/State, published by
mavlink_bridge). When Nav2 is active and the heartbeat reports either
``armed=False`` or a mode other than GUIDED, calls
``/lifecycle_manager_navigation/manage_nodes`` with command=1 (DEACTIVATE).

This prevents controller_server / waypoint_follower from spamming
'Controller failed' / retry messages once the vehicle is no longer
under autonomy control.

The watchdog fires once per active session and re-arms after Nav2 has
returned to ACTIVE (tracked by a 1 Hz poll of /waypoint_follower/get_state).
"""

from __future__ import annotations

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from mavros_msgs.msg import State as PixhawkHeartbeat
from nav2_msgs.srv import ManageLifecycleNodes

from autonomy_bringup_pkg.pixhawk_ready_wait import mode_matches


_MANAGE_SERVICE = '/lifecycle_manager_navigation/manage_nodes'
_GET_STATE_SERVICE = '/waypoint_follower/get_state'
_DEACTIVATE_CMD = 1
_REQUIRED_MODE = 'GUIDED'
_STATE_POLL_SEC = 1.0


class Nav2ArmWatchdog(Node):

    def __init__(self) -> None:
        super().__init__('nav2_arm_watchdog')

        self._fired = False
        self._seen_heartbeat = False
        self._nav2_active = False
        self._pending_state_future = None

        cb_group = MutuallyExclusiveCallbackGroup()

        heartbeat_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            PixhawkHeartbeat,
            '/pixhawk/heartbeat',
            self._on_heartbeat,
            heartbeat_qos,
            callback_group=cb_group,
        )

        self._manage_client = self.create_client(
            ManageLifecycleNodes, _MANAGE_SERVICE, callback_group=cb_group
        )
        self._get_state_client = self.create_client(
            GetState, _GET_STATE_SERVICE, callback_group=cb_group
        )

        self.create_timer(_STATE_POLL_SEC, self._poll_nav2_state, callback_group=cb_group)

        self.get_logger().info(
            f'Watching /pixhawk/heartbeat; will DEACTIVATE Nav2 on disarm or mode != {_REQUIRED_MODE}.'
        )

    def _poll_nav2_state(self) -> None:
        if self._pending_state_future is not None and not self._pending_state_future.done():
            return
        if not self._get_state_client.service_is_ready():
            self._nav2_active = False
            return
        future = self._get_state_client.call_async(GetState.Request())
        self._pending_state_future = future
        future.add_done_callback(self._on_state_response)

    def _on_state_response(self, future) -> None:
        self._pending_state_future = None
        try:
            result = future.result()
        except Exception:
            self._nav2_active = False
            return
        if result is None:
            self._nav2_active = False
            return
        was_active = self._nav2_active
        self._nav2_active = result.current_state.id == LifecycleState.PRIMARY_STATE_ACTIVE
        if self._fired and not self._nav2_active:
            # Nav2 has finished deactivating; re-arm once it next comes ACTIVE.
            pass
        if self._fired and self._nav2_active and not was_active:
            # Operator re-activated Nav2 after a previous trigger; re-arm.
            self._fired = False
            self.get_logger().info('Nav2 ACTIVE again; watchdog re-armed.')

    def _on_heartbeat(self, msg: PixhawkHeartbeat) -> None:
        armed = bool(msg.armed)
        mode = msg.mode or ''

        if not self._seen_heartbeat:
            self._seen_heartbeat = True
            return

        trigger = (not armed) or (not mode_matches(mode, _REQUIRED_MODE))
        if not trigger or self._fired or not self._nav2_active:
            return

        reasons = []
        if not armed:
            reasons.append('disarmed')
        if not mode_matches(mode, _REQUIRED_MODE):
            reasons.append(f'mode={mode or "?"}')
        self.get_logger().warn(
            f'Pilot intervention detected ({", ".join(reasons)}); deactivating Nav2.'
        )
        self._fired = True
        self._send_deactivate()

    def _send_deactivate(self) -> None:
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
