import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_pkg_dir = get_package_share_directory("config_pkg")

    use_navsat_transform_arg_value = LaunchConfiguration("use_navsat_transform")
    use_global_ekf_arg_value = LaunchConfiguration("use_global_ekf")
    gps_fix_topic_arg_value = LaunchConfiguration("gps_fix_topic")
    p_surface_pa_arg_value = LaunchConfiguration("p_surface_pa")

    vehicle_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg_dir, "launch", "vehicle_control.launch.py")
        ),
        launch_arguments={
            "autonomy": LaunchConfiguration("autonomy"),
        }.items(),
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg_dir, "launch", "sensors.launch.py")
        )
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg_dir, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_navsat_transform": use_navsat_transform_arg_value,
            "use_global_ekf": use_global_ekf_arg_value,
            "gps_fix_topic": gps_fix_topic_arg_value,
            "p_surface_pa": p_surface_pa_arg_value,
        }.items(),
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
                "autonomy",
                default_value="false",
                description="Launch the Nav2 autonomy stack iff argument true passed.",
            ),
            vehicle_control_launch,
            sensors_launch,
            navigation_launch,
            foxglove_node,
        ]
    )
