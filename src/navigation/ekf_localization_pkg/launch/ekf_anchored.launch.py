"""
Live (online) version of the anchored localization pipeline.

Runs:
  ekf_local_node     →  /odometry/filtered/local              (DVL + IMU + pressure)
  odometry_validator →  /odometry/filtered/local_validated    (drops bad stamps)
  gnss_anchored_pose →  /odometry/filtered/global             (local + offset)

This replaces the global EKF + navsat_transform + gnss_datum_watchdog stack.
For offline bag replay, use offline_anchored_replay.launch.py instead — it
sets use_sim_time=true and otherwise has the same content.

Map frame is anchored to the first /fix that passes the h_acc gate. After that,
position is local-EKF dead-reckoning + a fixed translation. No Kalman fusion of
GPS thereafter (set reanchor_on_each_fix:=true to update on each fix instead).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("ekf_localization_pkg")
    default_local_params = Path(pkg_dir, "config", "ekf_local.yaml")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=str(default_local_params),
        description="Path to local EKF (ekf_local_node) parameters YAML.",
    )
    gps_fix_topic_arg = DeclareLaunchArgument(
        "gps_fix_topic",
        default_value="/gps/selected",
        description="NavSatFix topic that gnss_anchored_pose subscribes to.",
    )
    h_acc_topic_arg = DeclareLaunchArgument(
        "h_acc_topic",
        default_value="/ubx_nav_hp_pos_llh",
        description=(
            "UBX-NAV-HPPOSLLH topic for the anchor h_acc gate. Empty string "
            "disables the gate (anchor on first non-null-island status>=0 fix)."
        ),
    )
    h_acc_max_arg = DeclareLaunchArgument(
        "h_acc_max_m",
        default_value="0.5",
        description="Max horizontal accuracy (m) for the anchor fix.",
    )
    reanchor_arg = DeclareLaunchArgument(
        "reanchor_on_each_fix",
        default_value="false",
        description=(
            "If true, gnss_anchored_pose updates the offset on every valid GPS "
            "fix (causes position discontinuities). Default false matches the "
            "'set origin once on first fix, dead-reckon afterwards' intent."
        ),
    )

    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        remappings=[("odometry/filtered", "/odometry/filtered/local")],
    )

    odometry_validator_node = Node(
        package="ekf_localization_pkg",
        executable="odometry_validator",
        name="odometry_validator",
        output="screen",
        parameters=[{
            "input_topic": "/odometry/filtered/local",
            "output_topic": "/odometry/filtered/local_validated",
            "max_forward_jump_s": 60.0,
            "max_backward_jump_s": 1.0,
        }],
    )

    anchored_pose_node = Node(
        package="ekf_localization_pkg",
        executable="gnss_anchored_pose",
        name="gnss_anchored_pose",
        output="screen",
        parameters=[{
            "local_odom_topic": "/odometry/filtered/local_validated",
            "gps_topic": LaunchConfiguration("gps_fix_topic"),
            "global_odom_topic": "/odometry/filtered/global",
            "h_acc_topic": LaunchConfiguration("h_acc_topic"),
            "h_acc_max_m": LaunchConfiguration("h_acc_max_m"),
            "reanchor_on_each_fix": LaunchConfiguration("reanchor_on_each_fix"),
            "publish_tf": True,
            "map_frame": "map",
            "odom_frame": "odom",
        }],
    )

    return LaunchDescription([
        params_file_arg,
        gps_fix_topic_arg,
        h_acc_topic_arg,
        h_acc_max_arg,
        reanchor_arg,
        ekf_local_node,
        odometry_validator_node,
        anchored_pose_node,
    ])
