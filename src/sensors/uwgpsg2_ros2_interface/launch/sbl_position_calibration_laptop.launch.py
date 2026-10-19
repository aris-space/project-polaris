"""Calibrate the Waterlinked topside position — LAPTOP test with a SIMULATED lat/lon (thesis).

No GNSS: PUTs a simulated position to config/generic.static_lat/static_lon and verifies via
/position/master. Confirms the write path from the laptop.

    ros2 launch uwgpsg2_ros2_interface sbl_position_calibration_laptop.launch.py sim_lat:=47.3210 sim_lon:=8.5540
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
    sim_lat = LaunchConfiguration("sim_lat")
    sim_lon = LaunchConfiguration("sim_lon")
    url = LaunchConfiguration("url")
    return LaunchDescription([
        DeclareLaunchArgument("sim_lat", default_value="47.3210",
                              description="Simulated topside latitude [deg]."),
        DeclareLaunchArgument("sim_lon", default_value="8.5540",
                              description="Simulated topside longitude [deg]."),
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
                    "source": "simulated",
                    "sim_lat": ParameterValue(sim_lat, value_type=float),
                    "sim_lon": ParameterValue(sim_lon, value_type=float),
                    "waterlinked_url": ParameterValue(url, value_type=str),
                },
            ],
        ),
    ])
