"""Laptop-only test of the Waterlinked heading API (thesis).

No sensors — just GET the current heading, optionally PUT a new static_orientation, and GET to verify.
Use it to confirm the write path works from your laptop over Wi-Fi before relying on the full
sbl_heading_calibration node.

    ros2 launch uwgpsg2_ros2_interface sbl_heading_set.launch.py                  # read-only probe
    ros2 launch uwgpsg2_ros2_interface sbl_heading_set.launch.py heading:=123.4   # set + verify
    ros2 launch uwgpsg2_ros2_interface sbl_heading_set.launch.py heading:=90 url:=http://192.168.194.94
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    heading = LaunchConfiguration("heading")
    url = LaunchConfiguration("url")

    return LaunchDescription([
        DeclareLaunchArgument("heading", default_value="-1.0",
                              description="Heading to write [deg CW from N]. Negative = read-only probe."),
        DeclareLaunchArgument("url", default_value="http://192.168.194.94",
                              description="Waterlinked UGPS box base URL."),
        Node(
            package="uwgpsg2_ros2_interface",
            executable="sbl_heading_set",
            name="sbl_heading_set",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "heading_deg": ParameterValue(heading, value_type=float),
                "waterlinked_url": ParameterValue(url, value_type=str),
            }],
        ),
    ])
