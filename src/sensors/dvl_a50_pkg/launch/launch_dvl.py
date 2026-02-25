"""
Launch file for the Water Linked DVL-A50 sensor.

Uses the dvl_a50 driver (https://github.com/ndahn/dvl_a50) with our own
project-specific configuration from dvl_a50_pkg/config/dvl_a50.yaml.

The DVL driver is a lifecycle node, so it must be transitioned through
  unconfigured -> inactive -> active
before it starts publishing data. This launch file handles both transitions
automatically on startup.

Published topics (under /sensors/dvl/):
  - dvl/velocity             (marine_acoustic_msgs/Dvl)
  - dvl/dead_reckoning       (geometry_msgs/PoseWithCovarianceStamped)
  - dvl/odometry             (nav_msgs/Odometry)          — raw from driver
  - dvl/odometry_cov         (nav_msgs/Odometry)          — with twist covariance filled

This launch file also publishes a static base_link -> dvl_a50_link transform
for integration testing.
"""

import os

import launch
import launch.events
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState


def generate_launch_description():
    # Load project-specific config (IP address, speed of sound, etc.)
    config = os.path.join(
        get_package_share_directory("dvl_a50_pkg"),
        "config",
        "dvl_a50.yaml",
    )

    # The dvl_a50_node executable comes from the dvl_a50 C++ driver package
    dvl_node = LifecycleNode(
        namespace="sensors",
        package="dvl_a50",
        executable="dvl_a50_node",
        name="dvl_a50",
        parameters=[config],
        output="screen",
    )

    # Lifecycle transition actions
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(dvl_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(dvl_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    # on process start -> configure
    on_process_start = RegisterEventHandler(
        OnProcessStart(
            target_action=dvl_node,
            on_start=[configure_event],
        )
    )

    # on inactive -> activate
    on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=dvl_node,
            goal_state="inactive",
            entities=[activate_event],
        )
    )

    # on active -> log
    on_activated = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=dvl_node,
            goal_state="active",
            entities=[
                LogInfo(msg="DVL-A50 reached the 'ACTIVE' state"),
            ],
        )
    )

    # Static base_link -> dvl_a50_link transform for testing.
    # Translation is a placeholder; replace with your measured mounting offsets.
    # RPY maps ENU base frame to NED-aligned DVL frame:
    # roll = pi, pitch = 0, yaw = +pi/4.
    static_tf_base_to_dvl = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_dvl",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "3.141592653589793",
            "--pitch",
            "0.0",
            "--yaw",
            "0.7853981633974483",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "dvl_a50_link",
        ],
        output="screen",
    )

    # Computes speed-dependent twist covariance with bottom-lock quality gating
    # Subscribes to /sensors/dvl/velocity (for lock flag) and /sensors/dvl/odometry
    # Publishes /sensors/dvl/odometry_cov
    covariance_node = Node(
        package="dvl_a50_pkg",
        executable="odometry_covariance_node",
        name="dvl_odometry_covariance",
        namespace="sensors",
        parameters=[{
            "dvl_variant": "performance",  # "standard" (±1.01%) or "performance" (±0.1%)
            "no_lock_variance": 1.0,       # variance when bottom lock lost [m²/s²]
            "angular_covariance": 1000000.0,  # very uncertain angular rates (not provided by DVL)
            "velocity_stale_timeout_sec": 0.5,  # stale lock flag timeout
        }],
        output="screen",
    )

    return LaunchDescription([
        dvl_node,
        on_process_start,
        on_inactive,
        on_activated,
        static_tf_base_to_dvl,
        covariance_node,
    ])
