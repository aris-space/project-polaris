import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from config_pkg.constants import Comms, Logs


def generate_launch_description():
    config_pkg_dir = get_package_share_directory("config_pkg")

    manual_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg_dir, "launch", "manual_control.launch.py")
        )
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg_dir, "launch", "sensors.launch.py")
        )
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[
            os.path.join(config_pkg_dir, "config", "foxglove.yaml"),
            {"address": Comms.JETSON_IP_ADDRESS},
        ],
    )

    recorder_controller_node = Node(
        package="config_pkg",
        executable="recorder_controller",
        name="recorder_controller_node",
        parameters=[
            {"base_output_dir": Logs.ROSBAG_DIR},
            {"storage_id": "mcap"},
        ],
    )

    return LaunchDescription(
        [
            manual_control_launch,
            sensors_launch,
            foxglove_node,
            recorder_controller_node,
        ]
    )
