"""SBL quick check — live sanity report of the raw SBL ranges (thesis lake test). LAPTOP.

Only listens to topics; run it in parallel to the stations, floating stops and missions.

    ros2 launch uwgpsg2_ros2_interface sbl_quick_check.launch.py label:=station1_top
    ros2 launch uwgpsg2_ros2_interface sbl_quick_check.launch.py label:=float1_under reference:=station1_top

Change the label / reference while it runs:
    ros2 param set /sbl_quick_check label station2_top
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("label", default_value="",
                              description="Name of what is recorded now (window saved as <label>.json)."),
        DeclareLaunchArgument("reference", default_value="",
                              description="Label of an earlier window to compare with."),
        DeclareLaunchArgument("window_s", default_value="30.0", description="Report interval [s]."),
        DeclareLaunchArgument("out_dir", default_value="~/sbl_quick_check",
                              description="Folder for <label>.json and log.jsonl."),
        Node(
            package="uwgpsg2_ros2_interface",
            executable="sbl_quick_check",
            name="sbl_quick_check",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "label": ParameterValue(LaunchConfiguration("label"), value_type=str),
                "reference": ParameterValue(LaunchConfiguration("reference"), value_type=str),
                "window_s": ParameterValue(LaunchConfiguration("window_s"), value_type=float),
                "out_dir": ParameterValue(LaunchConfiguration("out_dir"), value_type=str),
            }],
        ),
    ])
