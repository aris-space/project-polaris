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
        "ekf_local.yaml",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=str(default_params),
        description="Path to robot_localization EKF parameters YAML.",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
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

    return LaunchDescription([params_file_arg, pressure_adapter_node, ekf_node])
