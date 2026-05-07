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
    imu0_topic_arg = DeclareLaunchArgument(
        "imu0_topic",
        default_value="/imu/data",
        description=(
            "IMU topic the local EKF actually fuses. Defaults to /imu/data "
            "(raw). Override to /imu/data_corrected if imu_yaw_correction is "
            "running upstream so the local EKF sees the heading-calibrated "
            "stream. Sets ekf_local_node's imu0 parameter directly, "
            "overriding the value in ekf_local.yaml."
        ),
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odometry/filtered/local_validated",
        description=(
            "Local EKF odometry forwarded to navsat_transform and the global EKF. "
            "Defaults to the *_validated topic so consumers receive the validated "
            "stream from odometry_validator (filters out stamp anomalies during "
            "offline replay)."
        ),
    )

    # Local EKF: fuses IMU + DVL + pressure. Starts immediately, no GPS needed.
    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"imu0": LaunchConfiguration("imu0_topic")},
        ],
        remappings=[("odometry/filtered", "/odometry/filtered/local")],
    )

    # Validator: forwards /odometry/filtered/local → /odometry/filtered/local_validated
    # while dropping any message whose header.stamp jumps forward by more than
    # 60 s relative to the last accepted message. Workaround for a robot_localization
    # behaviour observed during offline replay where, when /clock is briefly
    # unavailable, the EKF publishes a single message with a wall-clock stamp
    # instead of sim-time, which produces a multi-day dt downstream and blows
    # up the global EKF's predict step.
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
            imu0_topic_arg,
            odom_topic_arg,
            ekf_local_node,
            odometry_validator_node,
            datum_watchdog_node,
        ]
    )
