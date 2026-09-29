#!/usr/bin/env python3
"""
Polaris SITL: Gazebo Harmonic + ArduSub SITL + the REAL autonomy stack.

    ros2 launch orca_sim_bringup sim_launch.py                 # headless, ground-truth odom
    ros2 launch orca_sim_bringup sim_launch.py gzclient:=True  # Gazebo GUI (native Linux only)

Then, exactly as on the vehicle:
    ros2 run autonomy_bringup_pkg nav2_activate

This file must never grow Nav2 nodes or Nav2 parameters of its own. It includes
autonomy_bringup_pkg/launch/autonomy.launch.py and mavlink_bridge.launch.py
unmodified; the only thing it changes about them is POLARIS_USE_SIM_TIME=1
(honoured only in the :sim image, see autonomy.launch.py) and the MAVLink
connection URLs (see mavlink_bridge/connection.py).

ground_truth:=True (Phase 2 default)
    Gazebo odometry stands in for the local EKF: odom_to_tf publishes TF
    odom->base_link and republishes the odometry on /odometry/filtered/local,
    and a static map->odom replaces the global EKF. Everything downstream of the
    EKF on the vehicle runs unchanged: Nav2 (odom_topic) and ros2_receiver, which
    forwards /odometry/filtered/local to ArduSub as MAVLink ODOMETRY (external
    nav, see cfg/sub.parm EK3_SRC1_*).

WARNING: the Polaris thruster geometry in orca_description is not yet
independently verified. Do not transfer any tuning from this sim to the vehicle
(docs/SIM_MERGE_PLAN.md Phase 4).

Derived from orca4 (Clyde McQueen, MIT) via project-polaris-simulation-personal.
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

# ArduSub SITL serves one MAVLink client per TCP serial port:
# SERIAL0 5760, SERIAL1 5762, SERIAL2 5763. Receiver, GCS heartbeat and
# publisher each need their own. Values already in the environment win.
_MAVLINK_ENV_DEFAULTS = {
    'MAVLINK_PUBLISHER_URL': 'tcp:127.0.0.1:5760',
    'MAVLINK_RECEIVER_URL': 'tcp:127.0.0.1:5762',
    'MAVLINK_GCS_URL': 'tcp:127.0.0.1:5763',
    'MAVLINK_CONNECT_RETRIES': '90',
    'MAVLINK_CONNECT_DELAY_SEC': '1.0',
}


def _ardusub_home(mission_origin_json: str) -> str:
    """lat,lon,alt,yaw for ``ardusub --home``, from the one mission origin file."""
    with open(mission_origin_json, encoding='utf-8') as f:
        o = json.load(f)
    return f"{float(o['lat'])},{float(o['lon'])},{float(o['alt'])},{float(o.get('yaw', 0.0))}"


def generate_launch_description():
    sim_bringup_dir = get_package_share_directory('orca_sim_bringup')
    orca_description_dir = get_package_share_directory('orca_description')
    autonomy_bringup_dir = get_package_share_directory('autonomy_bringup_pkg')
    mavlink_bridge_dir = get_package_share_directory('mavlink_bridge')

    ardusub_params_file = os.path.join(sim_bringup_dir, 'cfg', 'sub.parm')
    world_file = os.path.join(orca_description_dir, 'worlds', 'sand.world')
    ardusub_home = _ardusub_home(
        os.path.join(autonomy_bringup_dir, 'missions', 'default_mission_origin.json'))
    # ArduSub writes eeprom.bin and logs into its cwd; keep them out of the workspace.
    ardusub_cwd = os.path.join(os.path.expanduser('~'), '.ros', 'ardusub_sitl')
    os.makedirs(ardusub_cwd, exist_ok=True)

    ground_truth = LaunchConfiguration('ground_truth')

    return LaunchDescription([
        # Must come first: read by autonomy.launch.py when it is included below.
        SetEnvironmentVariable('POLARIS_USE_SIM_TIME', '1'),
        *[SetEnvironmentVariable(k, EnvironmentVariable(k, default_value=v))
          for k, v in _MAVLINK_ENV_DEFAULTS.items()],

        DeclareLaunchArgument(
            'gzclient', default_value='False',
            description='Launch the Gazebo GUI? Headless by default; GUI works on native Linux only.'),
        DeclareLaunchArgument(
            'ardusub', default_value='True',
            description='Launch ArduSub SITL?'),
        DeclareLaunchArgument(
            'nav', default_value='True',
            description='Launch the Nav2 stack via autonomy_bringup_pkg/autonomy.launch.py?'),
        DeclareLaunchArgument(
            'ground_truth', default_value='True',
            description='True: Gazebo odometry drives TF and /odometry/filtered/local. '
                        'False: the real EKF does (Phase 3).'),
        DeclareLaunchArgument(
            'gcs_url', default_value='udpclient:127.0.0.1:14550',
            description='ArduSub SITL SERIAL5 device for a human GCS (MAVProxy / QGroundControl).'),
        DeclareLaunchArgument(
            'foxglove', default_value='True',
            description='Launch foxglove_bridge on port 8765?'),

        # ArduSub SITL with the JSON physics backend (Gazebo ArduPilotPlugin on 9002).
        # -w wipes eeprom so sub.parm always applies; yaw in --home is ignored (Gazebo owns it).
        # SERIAL0-2 (TCP 5760/5762/5763) belong to mavlink_bridge; SERIAL5 sends to UDP 14550
        # for a human GCS (MAVProxy / QGroundControl), see cfg/sub.parm.
        ExecuteProcess(
            cmd=['ardusub', '-S', '-w', '-M', 'JSON', '--defaults', ardusub_params_file,
                 '-I0', '--home', ardusub_home,
                 '--serial5', LaunchConfiguration('gcs_url')],
            cwd=ardusub_cwd,
            output='screen',
            sigterm_timeout='15',
            sigkill_timeout='5',
            condition=IfCondition(LaunchConfiguration('ardusub')),
        ),

        # Gazebo Sim, headless unless gzclient:=True.
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', '-r', world_file],
            output='screen',
            sigterm_timeout='15',
            sigkill_timeout='5',
            condition=IfCondition(LaunchConfiguration('gzclient')),
        ),
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', '-r', '-s', world_file],
            output='screen',
            sigterm_timeout='15',
            sigkill_timeout='5',
            condition=UnlessCondition(LaunchConfiguration('gzclient')),
        ),

        # Gazebo -> ROS: sim clock, ground-truth odometry, ocean current.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/model/orca4/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/ocean_current@geometry_msgs/msg/Vector3]gz.msgs.Vector3d',
            ],
            remappings=[('/model/orca4/odometry', '/odom')],
            output='screen',
        ),

        # Ground-truth stand-in for ekf_local (odom->base_link + /odometry/filtered/local) ...
        Node(
            package='orca_sim_bringup',
            executable='odom_to_tf.py',
            parameters=[{
                'odom_topic': '/odom',
                'parent_frame_id': 'odom',
                'child_frame_id': 'base_link',
                'republish_topic': '/odometry/filtered/local',
                'use_sim_time': True,
            }],
            output='screen',
            condition=IfCondition(ground_truth),
        ),
        # ... and for ekf_global (map->odom). With ground_truth:=False the EKFs own both.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[{'use_sim_time': True}],
            output='screen',
            condition=IfCondition(ground_truth),
        ),

        # The vehicle's MAVLink bridge, unmodified.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mavlink_bridge_dir, 'launch', 'mavlink_bridge.launch.py')),
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'port': 8765, 'use_sim_time': True}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('foxglove')),
        ),

        # The vehicle's Nav2 stack, unmodified. Delayed so /clock and TF exist first.
        # Starts unconfigured (autostart False), exactly as on the vehicle.
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(autonomy_bringup_dir, 'launch', 'autonomy.launch.py')),
                    condition=IfCondition(LaunchConfiguration('nav')),
                ),
            ],
        ),
    ])
