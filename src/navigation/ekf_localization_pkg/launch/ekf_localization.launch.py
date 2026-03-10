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

    return LaunchDescription([params_file_arg, ekf_node])
