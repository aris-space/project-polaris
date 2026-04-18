from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_bool(raw_value: str) -> bool:
    return raw_value.strip().lower() in {"1", "true", "yes", "on"}

def _launch_setup(context, *args, **kwargs):
    respawn = _parse_bool(LaunchConfiguration("respawn").perform(context))
    respawn_delay = float(LaunchConfiguration("respawn_delay").perform(context))

    return [
        Node(
            package="keller_26x_pkg",
            executable="keller_26x",
            name="keller_26x",
            output="screen",
            respawn=respawn,
            respawn_delay=respawn_delay,
        ),
        # Static base_link -> keller_pressure_link. Replace with measured mounting values.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_to_keller_pressure",
            arguments=[
                "--x", "0.005607",
                "--y", "-0.000299",
                "--z", "0.117520",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "base_link",
                "--child-frame-id", "keller_pressure_link",
            ],
            output="screen",
            respawn=respawn,
            respawn_delay=respawn_delay,
        ),
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