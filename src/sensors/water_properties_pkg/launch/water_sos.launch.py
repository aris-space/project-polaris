from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_bool(raw: str) -> bool:
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context, *args, **kwargs):
    respawn = _parse_bool(LaunchConfiguration("respawn").perform(context))
    respawn_delay = float(LaunchConfiguration("respawn_delay").perform(context))

    return [
        Node(
            package="water_properties_pkg",
            executable="water_sos_node",
            name="water_sos_node",
            output="screen",
            respawn=respawn,
            respawn_delay=respawn_delay,
        )
    ]


def generate_launch_description():
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
            OpaqueFunction(function=_launch_setup),
        ]
    )
