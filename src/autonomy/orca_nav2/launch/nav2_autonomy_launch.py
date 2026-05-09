"""
Launch file for the Nav2 3D Autonomy system.

Starts the Navigation2 servers configured for 3D AUV movement:
  - planner_server:     Uses StraightLinePlanner3D to generate 3D paths.
  - controller_server:  Uses PurePursuitController3D to follow paths.
                        Remaps /cmd_vel to /pixhawk/cmd_vel for Pixhawk integration.
  - lifecycle_manager:  The 'Boss' node that transitions the servers to the Active state.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Setup Paths
    orca_nav2_dir = get_package_share_directory('orca_nav2')
    default_params_file = os.path.join(orca_nav2_dir, 'config', 'nav2_autonomy.yaml')

    # 2. Launch Configurations (Matching your mode_control_pkg style)
    respawn = LaunchConfiguration("respawn")
    respawn_delay = LaunchConfiguration("respawn_delay")
    params_file = LaunchConfiguration("params_file")

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        respawn=respawn,
        respawn_delay=respawn_delay,
        parameters=[params_file],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        respawn=respawn,
        respawn_delay=respawn_delay,
        parameters=[params_file],
        remappings=[('/cmd_vel', '/pixhawk/cmd_vel')]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['planner_server', 'controller_server']
        }]
    )

    on_start_log = RegisterEventHandler(
        OnProcessStart(
            target_action=planner_server,
            on_start=[LogInfo(msg="[Nav2 Autonomy] Planner Server is starting up ...")]
        )
    )

    on_exit_shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=lifecycle_manager,
            on_exit=[
                LogInfo(msg="[FATAL] Lifecycle manager exited. Shutting down Autonomy stack for safety!"),
                EmitEvent(event=Shutdown(reason="Lifecycle manager lost"))
            ]
        )
    )

    return LaunchDescription([
        # Arguments
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
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Full path to the ROS2 parameters file to use.",
        ),
        planner_server,
        controller_server,
        lifecycle_manager,

        on_start_log,
        on_exit_shutdown

    ])