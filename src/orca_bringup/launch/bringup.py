#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2022 Clyde McQueen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Bring up all nodes

Use a modified navigation_launch.py that doesn't launch velocity_smoother.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')
    mavlink_bridge_dir = get_package_share_directory('mavlink_bridge')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    nav2_bt_file = os.path.join(orca_bringup_dir, 'behavior_trees', 'orca4_bt.xml')
    nav2_params_file = os.path.join(orca_bringup_dir, 'params', 'nav2_params.yaml')

    # Rewrite to add the full path
    # The rewriter will only rewrite existing keys
    param_sub = {
        'use_sim_time': use_sim_time,
        'default_nav_to_pose_bt_xml': nav2_bt_file
    }
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites=param_sub,
        convert_types=True)

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'base',
            default_value='True',
            description='Launch base controller?',
        ),

        DeclareLaunchArgument(
            'comms',
            default_value='True',
            description='Launch custom MAVLink bridge?',
        ),

        DeclareLaunchArgument(
            'comms_respawn',
            default_value='True',
            description='Respawn mavlink bridge nodes?',
        ),

        DeclareLaunchArgument(
            'enable_external_odom',
            default_value='True',
            description='Forward /odom to ArduPilot as MAVLink ODOMETRY (ros2_receiver).',
        ),

        DeclareLaunchArgument(
            'nav',
            default_value='True',
            description='Launch navigation?',
        ),

        DeclareLaunchArgument(
            'orca_params_file',
            default_value=os.path.join(orca_bringup_dir, 'params', 'orca_params.yaml'),
            description='Full path to the ROS2 parameters file to use for Orca nodes',
        ),

        # Launch custom MAVLink bridge for Pixhawk communications.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mavlink_bridge_dir, 'launch', 'mavlink_bridge.launch.py')),
            launch_arguments={
                'respawn': LaunchConfiguration('comms_respawn'),
                'respawn_delay': '2.0',
                'use_sim_time': use_sim_time,
                'enable_external_odom': LaunchConfiguration('enable_external_odom'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('comms')),
        ),

        # If base controller is disabled, keep map and odom aligned.
        ExecuteProcess(
            cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
                 '--frame-id', 'map',
                 '--child-frame-id', 'odom'],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('base')),
        ),

        # Include the rest of Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(orca_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'namespace': '',
                'use_sim_time': use_sim_time,
                'autostart': 'True',
                'params_file': configured_nav2_params,
                'use_composition': 'False',
                'use_respawn': 'False',
                'container_name': 'nav2_container',
            }.items(),
            condition=IfCondition(LaunchConfiguration('nav')),
        ),
    ])
