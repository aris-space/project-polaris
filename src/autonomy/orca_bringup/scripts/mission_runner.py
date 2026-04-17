#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2022 Clyde McQueen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Run the "huge loop" mission

Code inspired by https://github.com/ros2/ros2cli/blob/rolling/ros2action/ros2action/verb/send_goal.py

Usage:
-- ros2 run orca_bringup mission_runner.py

With the mavlink_bridge stack, ArduSub only receives velocity setpoints from ``ros2_receiver``
when the vehicle is in GUIDED **and** ``ros2_receiver`` has applied that mode (it gates
``/pixhawk/cmd_vel`` on its internal mode). The sub must also be **armed** for motors to run.
Upstream Orca4 often had other nodes (e.g. base_controller / mavros path) that masked this;
this script arms explicitly for the bridge + Nav2 path.
"""

from enum import Enum
import time

import rclpy
import rclpy.logging
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from std_msgs.msg import Header
from std_msgs.msg import String

from nav2_ready_wait import wait_for_waypoint_follower_active
from pixhawk_ready_wait import (
    PixhawkState,
    ensure_armed_and_mode_guided,
    make_heartbeat_callback,
)


class SendGoalResult(Enum):
    SUCCESS = 0     # Goal succeeded
    FAILURE = 1     # Goal failed
    CANCELED = 2    # Goal canceled (KeyboardInterrupt exception)


def make_pose(x: float, y: float, z: float):
    return PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=x, y=y, z=z)))


# Go home (1m deep)
go_home = FollowWaypoints.Goal()
go_home.poses.append(make_pose(x=0.0, y=0.0, z=-1.0))

# Dive to 8m
dive = FollowWaypoints.Goal()
dive.poses.append(make_pose(x=0.0, y=0.0, z=-8.0))

# Big loop, will eventually result in a loop closure
delay_loop = FollowWaypoints.Goal()
delay_loop.poses.append(make_pose(x=0.0, y=0.0, z=-7.0))
for _ in range(2):
    delay_loop.poses.append(make_pose(x=20.0, y=-13.0, z=-7.0))
    delay_loop.poses.append(make_pose(x=10.0, y=-23.0, z=-7.0))
    delay_loop.poses.append(make_pose(x=-10.0, y=-8.0, z=-7.0))
    delay_loop.poses.append(make_pose(x=0.0, y=0.0, z=-7.0))


def wait_for_follow_waypoints(executor, action_client, timeout_sec: float = 300.0) -> bool:
    """
    rclpy's ActionClient.wait_for_server() only sleeps; it never spins the node, so discovery
    for /follow_waypoints can stall. Spin while waiting (see ros2/rclpy#58).

    Use the same MultiThreadedExecutor that holds the node for the whole process: calling
    rclpy.spin_once(node) adds/removes the node from the *global* executor each time, which
    breaks ActionClient waitable/graph handling so server_is_ready() may never become true.
    """
    start = time.time()
    print('Waiting for Nav2 /follow_waypoints action server...')
    while rclpy.ok():
        if action_client.server_is_ready():
            print('Nav2 /follow_waypoints is available.')
            return True
        if time.time() - start >= timeout_sec:
            print(
                f'Timed out after {timeout_sec:.0f}s waiting for /follow_waypoints. '
                'Is sim_launch running with nav:=true? Did you source install/setup.bash? '
                'Check: ros2 action list | grep follow ; ros2 lifecycle get /waypoint_follower'
            )
            return False
        executor.spin_once(timeout_sec=0.1)
    return False


# Send a goal to an action server and wait for the result.
# Cancel the goal if the user hits ^C (KeyboardInterrupt).
def send_goal(executor, action_client, send_goal_msg) -> SendGoalResult:
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
        # Cancel the goal if it's still active
        # TODO(clyde): this seems to work, but a second exception is generated -- why?
        if goal_handle is None:
            raise
        if (GoalStatus.STATUS_ACCEPTED == goal_handle.status or
                GoalStatus.STATUS_EXECUTING == goal_handle.status):
            print('Canceling goal...')
            cancel_future = goal_handle.cancel_goal_async()
            executor.spin_until_future_complete(cancel_future)
            cancel_response = cancel_future.result()

            if cancel_response is None:
                exc = cancel_future.exception()
                if exc is not None:
                    raise RuntimeError('Exception while canceling goal: {!r}'.format(exc)) from exc
                # Context shutting down (e.g. Ctrl+C) can complete the future with no response.
                print('Cancel finished without response (shutdown?)')
                return SendGoalResult.CANCELED

            if len(cancel_response.goals_canceling) == 0:
                raise RuntimeError('Failed to cancel goal')
            if len(cancel_response.goals_canceling) > 1:
                raise RuntimeError('More than one goal canceled')
            if cancel_response.goals_canceling[0].goal_id != goal_handle.goal_id:
                raise RuntimeError('Canceled goal with incorrect goal ID')

            print('Goal canceled')
            return SendGoalResult.CANCELED
        raise


def main():
    node = None
    follow_waypoints = None
    mode_pub = None
    arm_pub = None
    executor = None

    rclpy.init()

    try:
        node = rclpy.create_node(
            'mission_runner',
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

        # Allow discovery; flush a few spins so /pixhawk/* reaches mavlink_bridge reliably.
        for _ in range(10):
            executor.spin_once(timeout_sec=0.05)

        if not wait_for_waypoint_follower_active(executor, node, timeout_sec=180.0):
            print('Nav2 not ready; exiting.')
            return

        if not ensure_armed_and_mode_guided(
            executor, node, arm_pub, mode_pub, hb_state
        ):
            print(
                'Pixhawk not ready for mission; skipping Nav2 goal (no path published).',
                flush=True,
            )
            mode_pub.publish(String(data='MANUAL'))
            for _ in range(25):
                executor.spin_once(timeout_sec=0.05)
            arm_pub.publish(Bool(data=False))
            for _ in range(25):
                executor.spin_once(timeout_sec=0.05)
            return

        print('>>> Executing mission <<<')
        send_goal(executor, follow_waypoints, delay_loop)

        
        if rclpy.ok():
            print('>>> Disarming <<<')
            arm_pub.publish(Bool(data=False))
            time.sleep(0.5)
            print('>>> Setting Pixhawk mode to MANUAL <<<')
            mode_pub.publish(String(data='MANUAL'))
            rclpy.spin_once(node, timeout_sec=0.2)
        

        print('>>> Mission complete <<<')

    except KeyboardInterrupt:
        if arm_pub is not None and rclpy.ok():
            print('>>> Interrupted, disarming <<<')
            arm_pub.publish(Bool(data=False))
            time.sleep(0.3)
        if mode_pub is not None and executor is not None and node is not None and rclpy.ok():
            print('>>> Interrupted, setting Pixhawk mode to MANUAL <<<')
            mode_pub.publish(String(data='MANUAL'))
            executor.spin_once(timeout_sec=0.2)

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

    rclpy.shutdown()


if __name__ == '__main__':
    main()
