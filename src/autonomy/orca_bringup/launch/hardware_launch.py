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

Usage:
  ros2 launch orca_bringup hardware_launch.py
  ros2 launch orca_bringup hardware_launch.py rviz:=False bag:=True
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
    mavlink_bridge_dir = get_package_share_directory('mavlink_bridge')

    # -------------------------------------------------------------------------
    # use_sim_time is ALWAYS False for hardware.
    # It is NOT exposed as a launch argument intentionally so it can never
    # be accidentally set to True when launching on the real vehicle.
    # -------------------------------------------------------------------------
    use_sim_time = 'False'

    nav2_bt_file = os.path.join(orca_bringup_dir, 'behavior_trees', 'orca4_bt.xml')
    nav2_params_file = os.path.join(orca_bringup_dir, 'params', 'nav2_params.yaml')
    rviz_file = os.path.join(orca_bringup_dir, 'cfg', 'sim_launch.rviz')

    # Rewrite nav2_params.yaml: inject use_sim_time=False and the BT path.
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

        DeclareLaunchArgument(
            'comms_respawn',
            default_value='True',
            description='Respawn MAVLink bridge nodes on crash?',
        ),

        DeclareLaunchArgument(
            'enable_external_odom',
            default_value='True',
            description='Forward /odom to ArduSub as MAVLink ODOMETRY (ros2_receiver).',
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='False',
            description='Launch RViz2?',
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
        # Optional: RViz2
        # -----------------------------------------------------------------
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),

        # -----------------------------------------------------------------
        # MAVLink bridge — talks to Pixhawk over MAVLink.
        # Connection URLs come from env vars (see file header).
        # use_sim_time is forwarded explicitly as False.
        # -----------------------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mavlink_bridge_dir, 'launch', 'mavlink_bridge.launch.py')
            ),
            launch_arguments={
                'respawn': LaunchConfiguration('comms_respawn'),
                'respawn_delay': '2.0',
                'use_sim_time': use_sim_time,
                'enable_external_odom': LaunchConfiguration('enable_external_odom'),
            }.items(),
        ),

        # -----------------------------------------------------------------
        # Odometry path visualisation node.
        # -----------------------------------------------------------------
        Node(
            package='orca_base',
            executable='odom_to_path_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'max_poses': 5000,
            }],
        ),

        # -----------------------------------------------------------------
        # TODO: LOCALIZATION NODE
        #
        # You MUST supply a node (or set of nodes) that continuously
        # broadcasts the map -> odom TF.  Without this the Nav2 lifecycle
        # manager cannot configure and all navigation will be blocked.
        #
        # The localization must keep running for the full mission duration —
        # the BT recovery is a Wait (drift/hover), not a re-localise, so
        # Nav2 will stall immediately if the TF chain breaks.
        #
        # Options (uncomment / replace with your actual package):
        #
        #   A) DVL-based dead reckoning + initial USBL fix:
        #      Node(package='your_dvl_localizer', executable='dvl_localizer', ...)
        #
        #   B) Visual SLAM (ORB-SLAM2, RTAB-Map, …):
        #      Node(package='rtabmap_ros', executable='rtabmap', ...)
        #
        #   C) Temporary static map -> odom TF for bench tests ONLY
        #      (vehicle stays at origin, navigation will NOT work in open water):
        #      ExecuteProcess(
        #          cmd=['/opt/ros/humble/lib/tf2_ros/static_transform_publisher',
        #               '--frame-id', 'map', '--child-frame-id', 'odom'],
        #          output='screen',
        #      ),
        # -----------------------------------------------------------------

        # -----------------------------------------------------------------
        # Base controller + Nav2 are delayed so the TF chain (map->odom->
        # base_link) is already publishing before the Nav2 lifecycle manager
        # autostarts.  Without the delay, behavior_server may reach the
        # configure phase before the TF is available and fail permanently.
        #
        # On hardware the delay also gives the MAVLink bridge time to
        # connect and receive the first heartbeat from ArduSub.
        # -----------------------------------------------------------------
        TimerAction(
            period=5.0,
            actions=[

                # TODO: base_controller
                #
                # base_controller is the node that:
                #   - subscribes  /cmd_vel, ArduSub EKF pose, SLAM pose
                #   - publishes   /odom, odom->base_link TF, RC overrides to ArduSub
                #   - always must be enabled on hardware
                #
                # The source exists at orca_base/src/base_controller.cpp but
                # is NOT compiled — it was removed from orca_base/CMakeLists.txt.
                # Before using this launch file you MUST re-add it:
                #
                #   ament_auto_add_executable(base_controller src/base_controller.cpp)
                #
                # Then rebuild orca_base and uncomment the Node below.
                #
                # Node(
                #     package='orca_base',
                #     executable='base_controller',
                #     output='screen',
                #     parameters=[{
                #         'use_sim_time': False,
                #     }],
                # ),

                # Nav2 stack: controller_server, planner_server, behavior_server,
                # bt_navigator, waypoint_follower, lifecycle_manager.
                # velocity_smoother is NOT included (does not work in 3D).
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(orca_bringup_dir, 'launch', 'navigation_launch.py')
                    ),
                    launch_arguments={
                        'namespace': '',
                        'use_sim_time': use_sim_time,
                        'autostart': 'True',
                        'params_file': configured_nav2_params,
                        'use_composition': 'False',
                        'use_respawn': 'False',
                        'container_name': 'nav2_container',
                    }.items(),
                ),
            ],
        ),
    ])
