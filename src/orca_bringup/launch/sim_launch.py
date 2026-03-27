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
Launch a simulation.

Includes Gazebo, ArduSub, RViz, custom MAVLink bridge, and Nav2 nodes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')
    orca_description_dir = get_package_share_directory('orca_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    ardusub_params_file = os.path.join(orca_bringup_dir, 'cfg', 'sub.parm')
    orca_params_file = os.path.join(orca_bringup_dir, 'params', 'sim_orca_params.yaml')
    rosbag2_record_qos_file = os.path.join(orca_bringup_dir, 'params', 'rosbag2_record_qos.yaml')
    rviz_file = os.path.join(orca_bringup_dir, 'cfg', 'sim_launch.rviz')
    world_file = os.path.join(orca_description_dir, 'worlds', 'sand.world')
    
    # e.g. ros2 launch orca_bringup sim_launch.py gzclient:=False ardusub:=False

    return LaunchDescription([
        DeclareLaunchArgument(
            'ardusub',
            default_value='True',
            description='Launch ArduSUB with SIM_JSON?'
        ),

        DeclareLaunchArgument(
            'bag',
            default_value='False',
            description='Bag interesting topics?',
        ),

        DeclareLaunchArgument(
            'base',
            default_value='False',
            description='Launch base controller?',
        ),

        DeclareLaunchArgument(
            'gzclient',
            default_value='True',
            description='Launch Gazebo UI?'
        ),

        DeclareLaunchArgument(
            'comms',
            default_value='True',
            description='Launch custom MAVLink bridge?',
        ),

        DeclareLaunchArgument(
            'nav',
            default_value='True',
            description='Launch navigation?',
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='True',
            description='Launch rviz?',
        ),

        # Bag useful topics
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--qos-profile-overrides-path', rosbag2_record_qos_file,
                '--include-hidden-topics',
                '/cmd_vel',
                '/pixhawk/arm_cmd',
                '/pixhawk/cmd_vel',
                '/pixhawk/mode_cmd',
                '/model/orca4/odometry',
                '/pid_z',
                '/rosout',
                '/tf',
                '/tf_static',
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('bag')),
        ),

        # Launch rviz
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),

        # Launch ArduSub w/ SIM_JSON
        # -w: wipe eeprom
        # --home: start location (lat,lon,alt,yaw). Yaw is provided by Gazebo, so the start yaw value is ignored.
        # ardusub must be on the $PATH, see src/orca4/setup.bash
        # St. Moritz!
        ExecuteProcess(
            cmd=['ardusub', '-S', '-w', '-M', 'JSON', '--defaults', ardusub_params_file,
                 '-I0', '--home', '46.494536,9.856195,1822.0,0'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('ardusub')),
        ),

        # Launch Gazebo Sim
        # gz must be on the $PATH
        # libArduPilotPlugin.so must be on the GZ_SIM_SYSTEM_PLUGIN_PATH
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', '-r', world_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gzclient')),
        ),

        # Launch Gazebo Sim server-only
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', '-r', '-s', world_file],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('gzclient')),
        ),


        # Gazebo sim time -> ROS /clock (required when use_sim_time is true).
        # Without this, Nav2 lifecycle often stops after planner_server: behavior_server never
        # configures and /follow_waypoints has no server. Use GZ_TO_ROS only; see ros_gz_bridge README.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/model/orca4/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
            remappings=[
                ('/model/orca4/odometry', '/odom'),
            ],
            output='screen'
        ),

        Node(
            package='orca_base',
            executable='odom_to_path_node',
            output='screen'
        ),

        # In sim-only mode, publish odom -> base_link from Gazebo odometry.
        Node(
            package='orca_bringup',
            executable='odom_to_tf.py',
            parameters=[{
                'odom_topic': '/odom',
                'parent_frame_id': 'odom',
                'child_frame_id': 'base_link',
                'use_sim_time': use_sim_time,
            }],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('base')),
        ),

        # Delay bringup so /clock and /tf exist before lifecycle_manager autostart. Otherwise
        # behavior_server can fail configure while controller/planner already sit in inactive.
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(orca_bringup_dir, 'launch', 'bringup.py')),
                    launch_arguments={
                        'base': LaunchConfiguration('base'),
                        'comms': LaunchConfiguration('comms'),
                        'nav': LaunchConfiguration('nav'),
                        'use_sim_time': use_sim_time,
                        'orca_params_file': orca_params_file,
                    }.items(),
                ),
            ],
        ),
    ])

