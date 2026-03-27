from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = Path(
        get_package_share_directory("ekf_localization_pkg"),
        "config",
        "navsat_transform.yaml",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=str(default_params),
        description="Path to navsat_transform_node parameters YAML.",
    )
    gps_fix_topic_arg = DeclareLaunchArgument(
        "gps_fix_topic",
        default_value="/fix",
        description="GNSS NavSatFix topic.",
    )
    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value="/imu/data",
        description="IMU topic.",
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odometry/filtered/local",
        description="Filtered odometry topic from ekf_node.",
    )

    navsat_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        remappings=[
            ("gps/fix", LaunchConfiguration("gps_fix_topic")),
            ("imu", LaunchConfiguration("imu_topic")),
            ("odometry/filtered", LaunchConfiguration("odom_topic")),
        ],
    )

    return LaunchDescription(
        [params_file_arg, gps_fix_topic_arg, imu_topic_arg, odom_topic_arg, navsat_node]
    )
