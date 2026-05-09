#!/usr/bin/env python3
"""
Nav2 autonomy stack: planner, controller, behaviors, BT navigator,
waypoint follower and lifecycle manager for 3D AUV mission execution.

Nodes start unconfigured (autostart=False). Activate on demand via:
  ros2 service call /lifecycle_manager_navigation/manage_nodes \
      nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"

Usage (via start_system):
  ros2 launch config_pkg start_system.launch.py autonomy:=true

Usage (standalone):
  ros2 launch orca_bringup autonomy_launch.py
  ros2 launch orca_bringup autonomy_launch.py bag:=True
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


# DONE: Delete this rosbag things or at least comment them out.

def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')

    # use_sim_time is ALWAYS False for hardware - not exposed as an arg
    # so it can never be accidentally set to True on the real vehicle.
    use_sim_time = 'False'

    nav2_bt_file = os.path.join(orca_bringup_dir, 'behavior_trees', 'orca4_bt.xml')
    nav2_params_file = os.path.join(orca_bringup_dir, 'params', 'nav2_params.yaml')

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'default_nav_to_pose_bt_xml': nav2_bt_file,
        },
        convert_types=True,
    )

    respawn = LaunchConfiguration('respawn')
    respawn_delay = LaunchConfiguration('respawn_delay')

    # DONE: Change odom_topic in nav2_params.yaml to /odometry/filtered/local
    # DONE: Is this tf_rempapping necessary? where is it used? NOT NECESSARY
    # tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    cont_remappings = [('/cmd_vel', '/pixhawk/cmd_vel')]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        respawn=respawn,
        respawn_delay=respawn_delay,
        parameters=[configured_nav2_params],
        remappings=cont_remappings,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        respawn=respawn,
        respawn_delay=respawn_delay,
        parameters=[configured_nav2_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        respawn=respawn,
        respawn_delay=respawn_delay,
        parameters=[configured_nav2_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn=respawn,
        respawn_delay=respawn_delay,
        parameters=[configured_nav2_params],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        respawn=respawn,
        respawn_delay=respawn_delay,
        parameters=[configured_nav2_params],
    )

    mission_waypoints_publisher = Node(
        package='orca_bringup',
        executable='mission_waypoints_publisher.py',
        name='mission_waypoints_publisher',
        output='screen',
        respawn=respawn,
        respawn_delay=respawn_delay,
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': False,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ],
            'bond_timeout': 15.0,
        }],
    )

    on_exit_shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=lifecycle_manager,
            on_exit=[
                LogInfo(msg='[FATAL] Lifecycle manager exited - shutting down autonomy stack.'),
                EmitEvent(event=Shutdown(reason='Lifecycle manager lost')),
            ],
        )
    )

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'bag',
        #     default_value='False',
        #     description='Record interesting topics to a rosbag?',
        # ),
        DeclareLaunchArgument(
            'respawn',
            default_value='true',
            description='Respawn Nav2 nodes on crash?',
        ),
        DeclareLaunchArgument(
            'respawn_delay',
            default_value='2.0',
            description='Seconds to wait before restarting a crashed node.',
        ),

        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'bag', 'record',
        #         '/pure_pursuit_cross_track_xy',
        #         '/pure_pursuit_vertical_error',
        #         '/pure_pursuit_yaw_error',
        #         '/pure_pursuit_closest_point_map',
        #         '/pure_pursuit_robot_pose_map',
        #         '/pure_pursuit_robot_twist',
        #         '/odom',
        #         '/pixhawk/attitude',
        #         '/pixhawk/battery',
        #     ],
        #     output='screen',
        #     condition=IfCondition(LaunchConfiguration('bag')),
        # ),

        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        mission_waypoints_publisher,
        lifecycle_manager,
        on_exit_shutdown,
    ])
