from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    respawn = LaunchConfiguration("respawn")
    respawn_delay = LaunchConfiguration("respawn_delay")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "respawn",
                default_value="true",
                description="Automatically relaunch node if it exits/crashes.",
            ),
            DeclareLaunchArgument(
                "respawn_delay",
                default_value="2.0",
                description="Seconds to wait before restarting a crashed node.",
            ),
            Node(
                package="temperature_sensor_pkg",
                executable="temperature_sensor_node",
                name="temperature_sensor_node",
                output="screen",
                respawn=respawn,
                respawn_delay=respawn_delay,
            )
        ]
    )
