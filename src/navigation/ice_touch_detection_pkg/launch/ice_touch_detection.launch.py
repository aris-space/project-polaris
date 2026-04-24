from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_params = Path(
        get_package_share_directory("ice_touch_detection_pkg"),
        "config",
        "ice_touch_detection.yaml",
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=str(default_params),
        description="Path to ice touch detection parameters YAML.",
    )

    node = Node(
        package="ice_touch_detection_pkg",
        executable="ice_touch_detection_node",
        name="ice_touch_detection_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_file_arg, node])
