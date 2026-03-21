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
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState


def _parse_bool(raw_value: str) -> bool:
    return raw_value.strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context, *args, **kwargs):
    # Resolve substitutions to concrete Python types for ExecuteLocal internals.
    range_mode = LaunchConfiguration("range_mode")
    respawn = _parse_bool(LaunchConfiguration("respawn").perform(context))
    respawn_delay = float(LaunchConfiguration("respawn_delay").perform(context))

    config = os.path.join(
        get_package_share_directory("dvl_a50_pkg"),
        "config",
        "dvl_a50.yaml",
    )

    # DVL Lifecycle Node
    dvl_node = LifecycleNode(
        namespace="sensors",
        package="dvl_a50",
        executable="dvl_a50_sensor",
        name="dvl_a50",
        parameters=[
            config,
            {"range_mode": range_mode},
        ],
        output="screen",
        respawn=respawn,
        respawn_delay=respawn_delay,
    )

    # Lifecycle transition events
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

    # Event Handlers for State Management
    on_process_start = RegisterEventHandler(
        OnProcessStart(target_action=dvl_node, on_start=[configure_event])
    )

    on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=dvl_node,
            goal_state="inactive",
            entities=[activate_event],
        )
    )

    on_activated = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=dvl_node,
            goal_state="active",
            entities=[LogInfo(msg="DVL-A50 reached the 'ACTIVE' state")],
        )
    )

    # Static Transform
    static_tf_base_to_dvl = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_dvl",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--roll", "3.141592653589793", "--pitch", "0.0", "--yaw", "0.7853981633974483",
            "--frame-id", "base_link",
            "--child-frame-id", "dvl_a50_link",
        ],
        output="screen",
        respawn=respawn,
        respawn_delay=respawn_delay,
    )

    # Covariance Processor
    covariance_node = Node(
        package="dvl_a50_pkg",
        executable="odometry_covariance_node",
        name="dvl_odometry_covariance",
        namespace="sensors",
        parameters=[{
            "dvl_variant": "performance",
            "no_lock_variance": 1.0,
            "angular_covariance": 1000000.0,
            "velocity_stale_timeout_sec": 0.5,
        }],
        output="screen",
        respawn=respawn,
        respawn_delay=respawn_delay,
    )

    return [
        dvl_node,
        on_process_start,
        on_inactive,
        on_activated,
        static_tf_base_to_dvl,
        covariance_node,
    ]


def generate_launch_description():
    # Arguments
    range_mode_arg = DeclareLaunchArgument(
        "range_mode",
        default_value="auto",
        description="DVL range mode: auto, '=a', or 'a<=b'",
    )
    respawn_arg = DeclareLaunchArgument(
        "respawn",
        default_value="true",
        description="Automatically relaunch node if it exits/crashes.",
    )
    respawn_delay_arg = DeclareLaunchArgument(
        "respawn_delay",
        default_value="2.0",
        description="Seconds to wait before restarting a crashed node.",
    )

    return LaunchDescription([
        range_mode_arg,
        respawn_arg,
        respawn_delay_arg,
        OpaqueFunction(function=_launch_setup),
    ])
