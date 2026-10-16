"""Launch the SBL topside-heading auto-calibration node (thesis).

Loads config/sbl_heading_calibration_params.yaml and runs the one-shot calibration node. Drive a
straight forward leg right after launching. See docs / THESIS_CONTEXT for the procedure.

  ros2 launch uwgpsg2_ros2_interface sbl_heading_calibration.launch.py
  # dry-run (compute, do not write):  ... apply:=false
  # probe box schema only:            ... probe:=true
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
        "config",
        "sbl_heading_calibration_params.yaml",
    )

    apply_arg = LaunchConfiguration("apply")
    probe_arg = LaunchConfiguration("probe")

    return LaunchDescription([
        DeclareLaunchArgument("apply", default_value="true",
                              description="Write the corrected heading to the box ('false' = dry-run)."),
        DeclareLaunchArgument("probe", default_value="false",
                              description="Only dump config/generic + position/master, then exit."),
        Node(
            package="uwgpsg2_ros2_interface",
            executable="sbl_heading_calibration",
            name="sbl_heading_calibration",
            output="screen",
            emulate_tty=True,
            parameters=[
                params,
                {
                    "apply": ParameterValue(apply_arg, value_type=bool),
                    "probe": ParameterValue(probe_arg, value_type=bool),
                },
            ],
        ),
    ])
