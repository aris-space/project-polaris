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

import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, LifecycleTransition, Node

# Fully qualified node name (must match namespace + name on LifecycleNode below).
_DVL_LIFECYCLE_NODE_NAME = "/sensors/dvl_a50"


def _parse_bool(raw_value: str) -> bool:
    return raw_value.strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context, *args, **kwargs):
    # Resolve substitutions to concrete Python types for ExecuteLocal internals.
    range_mode = LaunchConfiguration("range_mode")
    respawn = _parse_bool(LaunchConfiguration("respawn").perform(context))
    respawn_delay = float(LaunchConfiguration("respawn_delay").perform(context))
    configure_delay = float(LaunchConfiguration("configure_delay_sec").perform(context))

    config = os.path.join(
        get_package_share_directory("dvl_a50_pkg"),
        "config",
        "dvl_a50.yaml",
    )

    # DVL Lifecycle Node
    dvl_node = LifecycleNode(
        namespace="sensors",
        package="dvl_a50",
        executable="dvl_a50_node",
        name="dvl_a50",
        parameters=[
            config,
            {"range_mode": range_mode},
        ],
        output="screen",
        respawn=respawn,
        respawn_delay=respawn_delay,
    )

    # Chain configure → activate using the same matchers as upstream launch_ros (start_state +
    # goal_state per transition). Manual OnStateTransition on goal_state alone can miss events
    # or misfire depending on rmw / TransitionEvent labeling.
    auto_lifecycle = LifecycleTransition(
        lifecycle_node_names=[_DVL_LIFECYCLE_NODE_NAME],
        transition_ids=[
            lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        ],
    )

    # Defer: DDS + ~/change_state must exist before the first transition; Jetson/Docker often
    # needs several seconds. "Node not found" from ros2 lifecycle on the host usually means
    # wrong ROS_DOMAIN_ID vs docker-compose or querying before the node appears.
    on_process_start = RegisterEventHandler(
        OnProcessStart(
            target_action=dvl_node,
            on_start=[
                LogInfo(
                    msg=(
                        f"[launch_dvl] Auto lifecycle for {_DVL_LIFECYCLE_NODE_NAME} "
                        f"starts in {configure_delay:g}s (configure then activate)."
                    )
                ),
                TimerAction(period=configure_delay, actions=[auto_lifecycle]),
            ],
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
    configure_delay_arg = DeclareLaunchArgument(
        "configure_delay_sec",
        default_value="8.0",
        description=(
            "Wait after DVL process starts before auto configure+activate "
            "(increase on Jetson/Docker if ~/change_state is not ready yet)."
        ),
    )

    return LaunchDescription([
        range_mode_arg,
        respawn_arg,
        respawn_delay_arg,
        configure_delay_arg,
        OpaqueFunction(function=_launch_setup),
    ])
