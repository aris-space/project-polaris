import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_pkg_dir = get_package_share_directory("config_pkg")

    vehicle_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg_dir, "launch", "vehicle_control.launch.py")
        ),
        launch_arguments={
            "autonomy": LaunchConfiguration("autonomy"),
        }.items(),
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
            DeclareLaunchArgument(
                "autonomy",
                default_value="false",
                description="Launch the Nav2 autonomy stack iff argument true passed.",
            ),
            vehicle_control_launch,
            sensors_launch,
            foxglove_node,
        ]
    )
