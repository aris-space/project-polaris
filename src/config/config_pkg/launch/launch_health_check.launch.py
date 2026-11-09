from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Health check service (call /health_check from the Foxglove Service Call panel)
        Node(
            package="config_pkg",
            executable="health_check",
            name="polaris_health_check",
            arguments=["--serve"],
            output="screen",
        ),
    ])
