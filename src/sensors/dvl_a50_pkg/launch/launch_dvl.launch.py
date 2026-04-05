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
  - dvl/odometry_cov         (nav_msgs/Odometry)          — twist covariance (stationary TEP / lock inflation)

Frame convention (must stay consistent for TF + robot_localization):
  - Driver param ``frame`` sets ``Odometry.header.frame_id`` and
    ``Odometry.child_frame_id`` (twist is in the sensor frame).
  - ``static_tf_base_to_dvl`` publishes base_link -> <frame> using the same
    ``sensor_frame`` launch argument (default dvl_a50_link). Translation from
    CAD Aris DVL_LINK -> CENTER_OF_MASS_LINK (m); RPY extrinsic vs base_link.
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
    sensor_frame = LaunchConfiguration("sensor_frame").perform(context).strip()
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
            {
                "range_mode": range_mode,
                # Overrides dvl_a50.yaml ``frame`` so TF child and odometry frames match.
                "frame": sensor_frame,
            },
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

    # Static Transform: parent must match your robot base; child must match driver ``frame`` /
    # odometry.child_frame_id (see sensor_frame launch argument).
    # CAD (Aris): DVL_LINK -> CENTER_OF_MASS_LINK [mm] converted to m; same signs as measure.
    # Used as base_link -> DVL translation so DVL sits forward of base_link (+x nose convention).
    # extrinsic RPY (rad): pi, 0, -pi/4 (sensor mount vs base_link).
    static_tf_base_to_dvl = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_dvl",
        arguments=[
            "--x", "0.735818",
            "--y", "-0.000483",
            "--z", "-0.067591",
            "--roll", "3.141592653589793",
            "--pitch", "0.0",
            "--yaw", "-0.7853981633974483",
            "--frame-id", "base_link",
            "--child-frame-id", sensor_frame,
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
            "twist_linear_covariance_model": "stationary_tep",
            "dvl_variant": "performance",
            "no_lock_variance": 1.0e6,
            # Widen lock-state cov slightly vs raw stationary_02 sample variances (see node doc).
            "lock_linear_variance_bias_drift_inflation_factor": 1.15,
            "angular_covariance": 1000000.0,
            "velocity_stale_timeout_sec": 0.5,
            # Drop redundant second /odometry publish (same stamp + twist as prior sample).
            "dedupe_same_stamp_twist": True,
            "dedupe_twist_epsilon": 1.0e-9,
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
    sensor_frame_arg = DeclareLaunchArgument(
        "sensor_frame",
        default_value="dvl_a50_link",
        description=(
            "TF child frame and DVL driver ``frame`` / odometry.child_frame_id "
            "(must match dvl_a50.yaml if you change the default)."
        ),
    )
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
        sensor_frame_arg,
        range_mode_arg,
        respawn_arg,
        respawn_delay_arg,
        configure_delay_arg,
        OpaqueFunction(function=_launch_setup),
    ])
