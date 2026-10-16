"""Calibrate the Waterlinked topside position from vehicle GNSS RTK (thesis) — normal/vehicle.

Heave the vehicle so its GNSS antenna is directly above the topside, then run:
    ros2 launch uwgpsg2_ros2_interface sbl_position_calibration.launch.py
    ros2 launch uwgpsg2_ros2_interface sbl_position_calibration.launch.py require_rtk_fixed:=false
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("uwgpsg2_ros2_interface"),
        "config", "sbl_position_calibration_params.yaml",
    )
    require_rtk_fixed = LaunchConfiguration("require_rtk_fixed")
    url = LaunchConfiguration("url")
    return LaunchDescription([
        DeclareLaunchArgument("require_rtk_fixed", default_value="true",
                              description="true = accept only RTK-FIXED; false = accept RTK-FLOAT too."),
        DeclareLaunchArgument("url", default_value="http://192.168.194.94",
                              description="Waterlinked UGPS box base URL."),
        Node(
            package="uwgpsg2_ros2_interface",
            executable="sbl_position_calibration",
            name="sbl_position_calibration",
            output="screen",
            emulate_tty=True,
            parameters=[
                params,
                {
                    "source": "gnss",
                    "require_rtk_fixed": ParameterValue(require_rtk_fixed, value_type=bool),
                    "waterlinked_url": ParameterValue(url, value_type=str),
                },
            ],
        ),
    ])
