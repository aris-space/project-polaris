from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "respawn",
                default_value="true",
                description="Automatically relaunch the BMS node if it exits.",
            ),
            DeclareLaunchArgument(
                "respawn_delay",
                default_value="2.0",
                description="Seconds to wait before restarting the BMS node.",
            ),
            Node(
                package="bms_pkg",
                executable="bms_node",
                name="bms_node",
                output="screen",
                respawn=LaunchConfiguration("respawn"),
                respawn_delay=LaunchConfiguration("respawn_delay"),
            ),
        ]
    )
