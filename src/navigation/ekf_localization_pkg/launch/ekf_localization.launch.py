from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = Path(
        get_package_share_directory("ekf_localization_pkg"),
        "config",
        "ekf_local.yaml",
    )
    default_navsat_params = Path(
        get_package_share_directory("ekf_localization_pkg"),
        "config",
        "navsat_transform.yaml",
    )
    default_global_params = Path(
        get_package_share_directory("ekf_localization_pkg"),
        "config",
        "ekf_global.yaml",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=str(default_params),
        description="Path to robot_localization EKF parameters YAML.",
    )
    use_navsat_arg = DeclareLaunchArgument(
        "use_navsat_transform",
        default_value="true",
        description="Launch navsat_transform_node together with EKF.",
    )
    navsat_params_file_arg = DeclareLaunchArgument(
        "navsat_params_file",
        default_value=str(default_navsat_params),
        description="Path to navsat_transform_node parameters YAML.",
    )
    use_global_ekf_arg = DeclareLaunchArgument(
        "use_global_ekf",
        default_value="true",
        description="Launch global map-frame EKF node.",
    )
    global_params_file_arg = DeclareLaunchArgument(
        "global_params_file",
        default_value=str(default_global_params),
        description="Path to robot_localization global EKF parameters YAML.",
    )
    gps_fix_topic_arg = DeclareLaunchArgument(
        "gps_fix_topic",
        default_value="/fix",
        description="GNSS NavSatFix topic for navsat_transform_node.",
    )
    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value="/imu/data",
        description="IMU topic for navsat_transform_node.",
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odometry/filtered/local",
        description="EKF odometry topic for navsat_transform_node.",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        remappings=[("odometry/filtered", "/odometry/filtered/local")],
    )

    pressure_adapter_node = Node(
        package="ekf_localization_pkg",
        executable="pressure_z_ned_to_pose_node",
        name="pressure_z_ned_to_pose_node",
        output="screen",
        parameters=[
            {
                "input_topic": "/pixhawk/z_ned",
                "output_topic": "/sensors/pressure/pose_enu",
                "output_frame_id": "odom",
                "z_variance": 0.04,
                "unused_variance": 1000000.0,
            }
        ],
    )

    navsat_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[LaunchConfiguration("navsat_params_file")],
        remappings=[
            ("gps/fix", LaunchConfiguration("gps_fix_topic")),
            ("imu", LaunchConfiguration("imu_topic")),
            ("odometry/filtered", LaunchConfiguration("odom_topic")),
        ],
        condition=IfCondition(LaunchConfiguration("use_navsat_transform")),
    )

    ekf_global_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global_node",
        output="screen",
        parameters=[LaunchConfiguration("global_params_file")],
        remappings=[("odometry/filtered", "/odometry/filtered/global")],
        condition=IfCondition(LaunchConfiguration("use_global_ekf")),
    )

    return LaunchDescription(
        [
            params_file_arg,
            use_navsat_arg,
            navsat_params_file_arg,
            use_global_ekf_arg,
            global_params_file_arg,
            gps_fix_topic_arg,
            imu_topic_arg,
            odom_topic_arg,
            pressure_adapter_node,
            ekf_node,
            navsat_node,
            ekf_global_node,
        ]
    )
