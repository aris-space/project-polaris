from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launches MAVLink bridge nodes with optional automatic respawn.
    """
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
                package="mavlink_bridge",
                executable="mavlink_publisher",
                name="mavlink_bridge_publisher",
                output="screen",
                respawn=respawn,
                respawn_delay=respawn_delay,
            ),
            Node(
                package="mavlink_bridge",
                executable="output_monitor",
                name="output_monitor",
                output="screen",
                respawn=respawn,
                respawn_delay=respawn_delay,
            ),
            Node(
                package="mavlink_bridge",
                executable="ros2_receiver",
                name="ros2_receiver",
                output="screen",
                respawn=respawn,
                respawn_delay=respawn_delay,
            ),
        ]
    )
