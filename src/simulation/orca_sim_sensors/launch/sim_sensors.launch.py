#!/usr/bin/env python3
"""Synthetic sensors for the Polaris simulation, on the vehicle's raw topics.

    ros2 launch orca_sim_sensors sim_sensors.launch.py ice_layer:=true

Included by orca_sim_bringup/launch/sim_launch.py. Starts the synthetic DVL, IMU, GNSS and
SBL plus the base_link -> sensor static TFs the vehicle's driver launch files publish (values
copied from those files; keep in sync). The Bar30 is not here: it is ArduSub SITL's second
barometer (SIM_BAR2_* in orca_sim_bringup/cfg/sub.parm), reaching ROS through
mavlink_publisher as on the vehicle.

All nodes run on sim time. See ../README.md for what each sensor models.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

# (child frame, x, y, z, roll, pitch, yaw) relative to base_link, from the vehicle launch files.
_MOUNTS = [
    # xsens_mti_ros2_driver/launch/xsens_mti_node.launch.py
    ('imu_link', 0.049, -0.0088, 0.076, 0.0, 0.0, 0.0),
    # dvl_a50_pkg/launch/launch_dvl.launch.py
    ('dvl_a50_link', 0.7216, -0.000243, -0.075, 3.141592653589793, 0.0, -0.7853981633974483),
    # gnss_bringup_pkg/launch/launch_gnss_x20p.launch.py
    ('gnss_link', -0.0057, -0.00024, 0.174, 0.0, 0.0, 0.0),
    # uwgpsg2_ros2_interface/launch/start_waterlinked_interface_bttm_side.launch.py
    ('sbl_link', -0.614, -0.000086, 0.197, 0.0, 0.0, 0.0),
]


def _static_tf(child, x, y, z, roll, pitch, yaw):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'static_tf_base_to_{child}',
        arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                   '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                   '--frame-id', 'base_link', '--child-frame-id', child],
        output='screen',
    )


def generate_launch_description():
    ice_layer = ParameterValue(LaunchConfiguration('ice_layer'), value_type=bool)
    seed = ParameterValue(LaunchConfiguration('seed'), value_type=int)
    startup_rtk = ParameterValue(LaunchConfiguration('startup_rtk'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument(
            'ice_layer', default_value='false',
            description='GNSS under ice: each surfacing randomly gets RTK (late), float only, '
                        'or nothing. false: RTK within a few seconds of every surfacing.'),
        DeclareLaunchArgument(
            'startup_rtk', default_value='true',
            description='GNSS: guaranteed RTK at startup until gnss_datum_watchdog latches '
                        '/gnss_datum. Only meaningful when the watchdog runs (real EKF).'),
        DeclareLaunchArgument(
            'seed', default_value='-1',
            description='Random seed for all synthetic sensors (-1 = different every run).'),

        GroupAction([
            SetParameter('use_sim_time', True),
            *[_static_tf(*m) for m in _MOUNTS],
            Node(package='orca_sim_sensors', executable='sim_imu_node', name='sim_imu_node',
                 parameters=[{'seed': seed}], output='screen'),
            Node(package='orca_sim_sensors', executable='sim_dvl_node', name='sim_dvl_node',
                 parameters=[{'seed': seed}], output='screen'),
            Node(package='orca_sim_sensors', executable='sim_gnss_node', name='sim_gnss_node',
                 parameters=[{'seed': seed, 'ice_layer': ice_layer,
                              'startup_rtk_until_datum': startup_rtk}],
                 output='screen'),
            Node(package='orca_sim_sensors', executable='sim_sbl_node', name='sim_sbl_node',
                 parameters=[{'seed': seed}], output='screen'),
        ]),
    ])
