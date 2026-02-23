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

Note: After activation, pinging is still disabled by default to prevent
overheating when out of water. Call the /sensors/dvl_a50/enable service
to start receiving data once the DVL is submerged.
"""

import os

import launch
import launch.events
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
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

    # Lifecycle transitions: configure and activate the node on startup
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

    # Log confirmation when the node reaches active state
    on_activated = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=dvl_node,
            goal_state="active",
            entities=[
                LogInfo(msg="DVL-A50 reached the 'ACTIVE' state"),
            ],
        )
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
            "sigma_min": 0.01,             # min horizontal std dev [m/s]
            "sigma_scale": 0.0101,         # σ_xy = max(sigma_min, sigma_scale * |v|)
            "no_lock_variance": 1.0,       # variance when bottom lock lost [m²/s²]
            "angular_covariance": -1.0,    # -1 = not available (REP-135)
        }],
        output="screen",
    )

    return LaunchDescription([
        configure_event,
        activate_event,
        dvl_node,
        on_activated,
        covariance_node,
    ])
