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
from datetime import datetime

import launch
import launch.events
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState


def generate_launch_description():
    # Keep rosbags out of the workspace root by default.
    default_bag_dir = "/bags"
    os.makedirs(default_bag_dir, exist_ok=True)
    default_bag_name = os.path.join(
        default_bag_dir, f"dvl_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )

    record_dvl_bag = LaunchConfiguration("record_dvl_bag")
    dvl_bag_name = LaunchConfiguration("dvl_bag_name")
    range_mode = LaunchConfiguration("range_mode")
    respawn = True
    respawn_delay = 2.0

    record_dvl_bag_arg = DeclareLaunchArgument(
        "record_dvl_bag",
        default_value="false",
        description="If true, start rosbag recording for DVL topics.",
    )
    dvl_bag_name_arg = DeclareLaunchArgument(
        "dvl_bag_name",
        default_value=default_bag_name,
        description="Output folder name for rosbag2 recording.",
    )
    range_mode_arg = DeclareLaunchArgument(
        "range_mode",
        default_value="auto",
        description=(
            "DVL range mode: auto, '=a', or 'a<=b' "
            "(examples: auto, =3, 2<=3)."
        ),
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
        parameters=[
            config,
            {"range_mode": range_mode},
        ],
        output="screen",
        respawn=respawn,
        respawn_delay=respawn_delay,
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
        respawn=respawn,
        respawn_delay=respawn_delay,
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
        respawn=respawn,
        respawn_delay=respawn_delay,
    )

    # Optional rosbag recorder for DVL integration/testing data
    dvl_rosbag_record = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            dvl_bag_name,
            "/sensors/dvl/velocity",
            "/sensors/dvl/dead_reckoning",
            "/sensors/dvl/odometry",
            "/sensors/dvl/odometry_cov",
            "/tf_static",
        ],
        condition=IfCondition(record_dvl_bag),
        output="screen",
        respawn=respawn,
        respawn_delay=respawn_delay,
    )

    return LaunchDescription([
        record_dvl_bag_arg,
        dvl_bag_name_arg,
        range_mode_arg,
        respawn_arg,
        respawn_delay_arg,
        dvl_node,
        on_process_start,
        on_inactive,
        on_activated,
        static_tf_base_to_dvl,
        covariance_node,
        dvl_rosbag_record,
    ])
