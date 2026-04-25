from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("ekf_localization_pkg")
    default_params = Path(pkg_dir, "config", "ekf_local.yaml")
    default_navsat_params = Path(pkg_dir, "config", "navsat_transform.yaml")
    default_global_params = Path(pkg_dir, "config", "ekf_global.yaml")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=str(default_params),
        description="Path to local EKF (ekf_local_node) parameters YAML.",
    )
    use_navsat_arg = DeclareLaunchArgument(
        "use_navsat_transform",
        default_value="true",
        description=(
            "Enable gnss_datum_watchdog. When a quality GNSS fix arrives the watchdog "
            "spawns navsat_transform_node and (optionally) ekf_global_node."
        ),
    )
    navsat_params_file_arg = DeclareLaunchArgument(
        "navsat_params_file",
        default_value=str(default_navsat_params),
        description="Base navsat_transform params forwarded to the watchdog.",
    )
    use_global_ekf_arg = DeclareLaunchArgument(
        "use_global_ekf",
        default_value="true",
        description="Whether the watchdog should also launch ekf_global_node.",
    )
    global_params_file_arg = DeclareLaunchArgument(
        "global_params_file",
        default_value=str(default_global_params),
        description="Global EKF params forwarded to the watchdog.",
    )
    gps_fix_topic_arg = DeclareLaunchArgument(
        "gps_fix_topic",
        default_value="/fix",
        description="GNSS NavSatFix topic for the watchdog and navsat_transform.",
    )
    h_acc_topic_arg = DeclareLaunchArgument(
        "h_acc_topic",
        default_value="/ubx_nav_hp_pos_llh",
        description=(
            "UBX-NAV-HPPOSLLH topic for the h_acc quality gate. "
            "Pass an empty string to disable the gate."
        ),
    )
    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value="/imu/data",
        description="IMU topic forwarded to navsat_transform.",
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odometry/filtered/local",
        description="Local EKF odometry forwarded to navsat_transform.",
    )

    # Local EKF: fuses IMU + DVL + pressure. Starts immediately, no GPS needed.
    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        remappings=[("odometry/filtered", "/odometry/filtered/local")],
    )

    # Watchdog: waits for a quality GNSS fix, then spawns navsat_transform_node
    # and ekf_global_node via navsat_global_ekf.launch.py with the fix as datum.
    datum_watchdog_node = Node(
        package="ekf_localization_pkg",
        executable="gnss_datum_watchdog",
        name="gnss_datum_watchdog",
        output="screen",
        parameters=[{
            "fix_topic": LaunchConfiguration("gps_fix_topic"),
            "h_acc_topic": LaunchConfiguration("h_acc_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "navsat_params_file": LaunchConfiguration("navsat_params_file"),
            "global_ekf_params_file": LaunchConfiguration("global_params_file"),
            "use_global_ekf": LaunchConfiguration("use_global_ekf"),
        }],
        condition=IfCondition(LaunchConfiguration("use_navsat_transform")),
    )

    return LaunchDescription(
        [
            params_file_arg,
            use_navsat_arg,
            navsat_params_file_arg,
            use_global_ekf_arg,
            global_params_file_arg,
            gps_fix_topic_arg,
            h_acc_topic_arg,
            imu_topic_arg,
            odom_topic_arg,
            ekf_local_node,
            datum_watchdog_node,
        ]
    )
