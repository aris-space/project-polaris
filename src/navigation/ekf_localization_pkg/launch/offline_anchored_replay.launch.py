"""
Offline replay using the local EKF + gnss_anchored_pose (no global EKF, no
navsat_transform, no gnss_datum_watchdog).

Pipeline:
  ekf_local_node    →  /odometry/filtered/local              (DVL+IMU+pressure)
  odometry_validator → /odometry/filtered/local_validated    (drops bad stamps)
  gnss_anchored_pose → /odometry/filtered/global             (local + offset)

The anchored pose node:
  - waits for the first valid /fix
  - records local-EKF position at that instant
  - publishes /odometry/filtered/global = local_pos + (datum - local_pos_at_anchor)
  - by default does NOT re-anchor on subsequent fixes

You run bag playback separately:

  ros2 launch ekf_localization_pkg offline_anchored_replay.launch.py
  ros2 bag play /path/to/bag --clock --topics ...

Set `gps_fix_topic` to whatever the bag actually contains
(`/gps/selected` is typical; `/fix` for older recordings).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetUseSimTime


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
    yaw_offset_arg = DeclareLaunchArgument(
        "yaw_offset_deg",
        default_value="0.0",
        description=(
            "Manual yaw correction (deg, CCW about +Z) applied to Odometry+NavSatFix."
        ),
    )
    antenna_offset_arg = DeclareLaunchArgument(
        "gps_antenna_offset_xyz",
        default_value="[0.0, 0.0, 0.0]",
        description=(
            "GPS antenna body-frame [x, y, z] (m). Compensates for lever-arm "
            "translation that appears when the AUV yaws on the spot."
        ),
    )
    imu_yaw_offset_arg = DeclareLaunchArgument(
        "imu_yaw_offset_deg",
        default_value="0.0",
        description="Pre-EKF yaw correction (deg, CCW about +Z) for imu_yaw_correction.",
    )
    ubx_pvt_topic_arg = DeclareLaunchArgument(
        "ubx_pvt_topic",
        default_value="/ubx_nav_pvt",
        description="UBX-NAV-PVT topic for head_mot-based yaw calibration.",
    )

    imu_yaw_correction_node = Node(
        package="ekf_localization_pkg",
        executable="imu_yaw_correction",
        name="imu_yaw_correction",
        output="screen",
        parameters=[{
            "yaw_offset_deg": LaunchConfiguration("imu_yaw_offset_deg"),
            "input_topic": "/imu/data",
            "output_topic": "/imu/data_corrected",
            "gps_topic": LaunchConfiguration("gps_fix_topic"),
            "ubx_pvt_topic": LaunchConfiguration("ubx_pvt_topic"),
        }],
    )

    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"imu0": "/imu/data_corrected"},
        ],
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
            "yaw_offset_deg": LaunchConfiguration("yaw_offset_deg"),
            "gps_antenna_offset_xyz": LaunchConfiguration("gps_antenna_offset_xyz"),
            "publish_tf": True,
            "map_frame": "map",
            "odom_frame": "odom",
        }],
    )

    return LaunchDescription([
        SetUseSimTime(True),
        params_file_arg,
        gps_fix_topic_arg,
        h_acc_topic_arg,
        h_acc_max_arg,
        reanchor_arg,
        yaw_offset_arg,
        antenna_offset_arg,
        imu_yaw_offset_arg,
        ubx_pvt_topic_arg,
        imu_yaw_correction_node,
        ekf_local_node,
        odometry_validator_node,
        anchored_pose_node,
    ])
