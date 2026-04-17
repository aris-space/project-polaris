from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    respawn = True
    respawn_delay = 2.0

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
                package="jetson_temperature",
                executable="jetson_temperature",
                name="jetson_temperature",
                output="screen",
                respawn=respawn,
                respawn_delay=respawn_delay,
            )
        ]
    )
