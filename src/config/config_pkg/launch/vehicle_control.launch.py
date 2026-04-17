import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from datetime import datetime
from config_pkg.constants import Logs


# TODO: IS respwan and such really used?

def generate_launch_description():
    # IncludeLaunchDescription arguments must remain launch substitutions/strings.
    respawn_arg_value = LaunchConfiguration("respawn")
    respawn_delay_arg_value = LaunchConfiguration("respawn_delay")
    autonomy_arg_value = LaunchConfiguration("autonomy")

    # Node action fields can safely use concrete python values.
    respawn = True
    respawn_delay = 2.0

    # 1. Find the path to the child package
    mode_control_pkg_dir = get_package_share_directory("mode_control_pkg")
    mavlink_bridge_pkg_dir = get_package_share_directory("mavlink_bridge")
    orca_bringup_dir = get_package_share_directory("orca_bringup")

    mode_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mode_control_pkg_dir, "launch", "launch_mode_control.launch.py"
            )
        ),
        launch_arguments={
            "respawn": respawn_arg_value,
            "respawn_delay": respawn_delay_arg_value,
        }.items(),
    )

    mavlink_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mavlink_bridge_pkg_dir, "launch", "mavlink_bridge.launch.py")
        ),
        launch_arguments={
            "respawn": respawn_arg_value,
            "respawn_delay": respawn_delay_arg_value,
        }.items(),
    )

    autonomy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orca_bringup_dir, "launch", "autonomy_launch.py")
        ),
        condition=IfCondition(autonomy_arg_value),
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
            DeclareLaunchArgument(
                "autonomy",
                default_value="false",
                description="Launch the Nav2 autonomy stack (inactive until activated).",
            ),
            mode_control_launch,
            mavlink_launch,
            autonomy_launch,
        ]
    )
