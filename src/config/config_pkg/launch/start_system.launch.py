import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    config_pkg_dir = get_package_share_directory("config_pkg")

    manual_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg_dir, "launch", "manual_control.launch.py")
        )
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg_dir, "launch", "sensors.launch.py")
        )
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[
            os.path.join(config_pkg_dir, "config", "foxglove.yaml"),
        ],
    )

    return LaunchDescription(
        [
            manual_control_launch,
            sensors_launch,
            foxglove_node,
        ]
    )
