"""
Starts navsat_transform_node and (optionally) ekf_global_node with a precise
GPS datum written by gnss_datum_watchdog.

Not intended for manual invocation. Spawned automatically by the watchdog
once a quality-gated GNSS fix is available, so navsat_transform never starts
at null-island (0°, 0°).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("ekf_localization_pkg")
    default_navsat_params = Path(pkg_dir, "config", "navsat_transform.yaml")
    default_global_params = Path(pkg_dir, "config", "ekf_global.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "datum_yaml",
                description=(
                    "Path to a one-shot YAML providing "
                    "datum: [lat, lon, alt] for navsat_transform_node."
                ),
            ),
            DeclareLaunchArgument(
                "navsat_params_file",
                default_value=str(default_navsat_params),
                description="Base navsat_transform params (frequency, mag_declination, etc.).",
            ),
            DeclareLaunchArgument(
                "global_ekf_params_file",
                default_value=str(default_global_params),
                description="Global EKF params.",
            ),
            DeclareLaunchArgument(
                "gps_fix_topic",
                default_value="/gps/selected",
                description="GNSS NavSatFix topic remapped to navsat_transform gps/fix.",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/imu/data",
                description="IMU topic remapped to navsat_transform imu.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odometry/filtered/local",
                description="Local EKF odometry remapped to navsat_transform odometry/filtered.",
            ),
            DeclareLaunchArgument(
                "use_global_ekf",
                default_value="true",
                description="Set false to skip ekf_global_node (navsat-only mode).",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Forward use_sim_time from the parent launch (bag replay).",
            ),
            DeclareLaunchArgument(
                "global_odom_topic",
                default_value="/odometry/filtered/global",
                description="Global EKF topic consumed by global_ekf_to_navsatfix_node.",
            ),
            DeclareLaunchArgument(
                "datum_lat",
                description="Datum latitude (degrees) — forwarded from gnss_datum_watchdog.",
            ),
            DeclareLaunchArgument(
                "datum_lon",
                description="Datum longitude (degrees) — forwarded from gnss_datum_watchdog.",
            ),
            DeclareLaunchArgument(
                "datum_alt",
                description="Datum altitude (meters) — forwarded from gnss_datum_watchdog.",
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                output="screen",
                # Base config loaded first; datum_yaml loaded second so its
                # datum: [lat, lon, alt] entry overrides any placeholder in the base.
                parameters=[
                    LaunchConfiguration("navsat_params_file"),
                    LaunchConfiguration("datum_yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                remappings=[
                    ("gps/fix", LaunchConfiguration("gps_fix_topic")),
                    ("imu", LaunchConfiguration("imu_topic")),
                    ("odometry/filtered", LaunchConfiguration("odom_topic")),
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_global_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("global_ekf_params_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                remappings=[("odometry/filtered", "/odometry/filtered/global")],
                condition=IfCondition(LaunchConfiguration("use_global_ekf")),
            ),
            Node(
                package="ekf_localization_pkg",
                executable="global_ekf_to_navsatfix",
                name="global_ekf_to_navsatfix_node",
                output="screen",
                parameters=[{
                    "datum_lat": LaunchConfiguration("datum_lat"),
                    "datum_lon": LaunchConfiguration("datum_lon"),
                    "datum_alt": LaunchConfiguration("datum_alt"),
                    "global_odom_topic": LaunchConfiguration("global_odom_topic"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }],
                condition=IfCondition(LaunchConfiguration("use_global_ekf")),
            ),
        ]
    )
