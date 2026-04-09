import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Full stack: MAVLink bridge first (serial + UDP + param sync), then mode control,
    then sensors and Foxglove. Splitting mavlink ahead of sensors/foxglove reduces
    launch-load contention for ros2_receiver and heartbeat publishing.
    """
    config_pkg_dir = get_package_share_directory("config_pkg")
    mavlink_bridge_pkg_dir = get_package_share_directory("mavlink_bridge")
    mode_control_pkg_dir = get_package_share_directory("mode_control_pkg")

    respawn = LaunchConfiguration("respawn")
    respawn_delay = LaunchConfiguration("respawn_delay")

    mavlink_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mavlink_bridge_pkg_dir, "launch", "mavlink_bridge.launch.py")
        ),
        launch_arguments={
            "respawn": respawn,
            "respawn_delay": respawn_delay,
        }.items(),
    )

    mode_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mode_control_pkg_dir, "launch", "launch_mode_control.launch.py")
        ),
        launch_arguments={
            "respawn": respawn,
            "respawn_delay": respawn_delay,
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
                "respawn",
                default_value="true",
                description="Automatically relaunch processes if they exit/crash.",
            ),
            DeclareLaunchArgument(
                "respawn_delay",
                default_value="2.0",
                description="Seconds to wait before restarting a crashed process.",
            ),
            mavlink_launch,
            mode_control_launch,
            sensors_launch,
            foxglove_node,
        ]
    )
