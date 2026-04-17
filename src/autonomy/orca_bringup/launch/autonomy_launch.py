#!/usr/bin/env python3

# Copyright (c) ARIS Space — hardware bringup for Orca4 AUV.
#
# Drop-in replacement for sim_launch.py without any Gazebo, ros_gz_bridge,
# or use_sim_time dependencies.  Localization (map -> odom TF) must be
# provided externally — see the TODO block below.

"""
Hardware bringup: MAVLink bridge, base controller, Nav2, optional RViz / rosbag.

Prerequisites (must be running before or alongside this launch):
  1. A localization source that publishes the map -> odom TF continuously.
     This can be a SLAM node (ORB-SLAM2, RTAB-Map, …), a USBL/DVL-based
     dead-reckoning node, or any other source.  See the placeholder below.
  2. ArduSub running on the real Pixhawk, reachable via MAVLink.
     Set the connection URLs with environment variables BEFORE launching:
       export MAVLINK_PUBLISHER_URL="udp:192.168.2.2:14550"   # read from FC
       export MAVLINK_RECEIVER_URL="udp:192.168.2.2:14551"    # write to FC
     (Or use serial:/dev/ttyUSB0:115200 etc.)

Usage (via start_system):
  ros2 launch config_pkg start_system.launch.py autonomy:=true

Usage (standalone):
  ros2 launch orca_bringup autonomy_launch.py
  ros2 launch orca_bringup autonomy_launch.py rviz:=False bag:=True
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')

    # -------------------------------------------------------------------------
    # use_sim_time is ALWAYS False for hardware.
    # It is NOT exposed as a launch argument intentionally so it can never
    # be accidentally set to True when launching on the real vehicle.
    # -------------------------------------------------------------------------
    use_sim_time = 'False'

    nav2_bt_file = os.path.join(orca_bringup_dir, 'behavior_trees', 'orca4_bt.xml')
    nav2_params_file = os.path.join(orca_bringup_dir, 'params', 'nav2_params.yaml')

    # TODO: Rewrite nav2_params.yaml: inject use_sim_time=False and the BT path.
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={
            'use_sim_time': use_sim_time,
            'default_nav_to_pose_bt_xml': nav2_bt_file,
        },
        convert_types=True,
    )

    return LaunchDescription([

        # -----------------------------------------------------------------
        # Launch arguments
        # -----------------------------------------------------------------
        DeclareLaunchArgument(
            'bag',
            default_value='False',
            description='Record interesting topics to a rosbag?',
        ),


        # -----------------------------------------------------------------
        # Optional: rosbag recording (hardware-relevant topics only).
        # /ocean_current is intentionally removed — it does not exist on hardware.
        # -----------------------------------------------------------------
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '/pure_pursuit_cross_track_xy',
                '/pure_pursuit_vertical_error',
                '/pure_pursuit_yaw_error',
                '/pure_pursuit_closest_point_map',
                '/pure_pursuit_robot_pose_map',
                '/pure_pursuit_robot_twist',
                '/odom',
                '/pixhawk/attitude',
                '/pixhawk/battery',
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('bag')),
        ),

        # -----------------------------------------------------------------
        # Odometry path visualisation node.
        # -----------------------------------------------------------------
        # Node(
        #     package='orca_base',
        #     executable='odom_to_path_node',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': False,
        #         'max_poses': 5000,
        #     }],
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(orca_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'namespace': '',
                'use_sim_time': use_sim_time,
                'autostart': 'False',
                'params_file': configured_nav2_params,
                'use_composition': 'False',
                'use_respawn': 'True',
                'container_name': 'nav2_container',
            }.items(),
        ),
    ])
