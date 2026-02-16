from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # 1. Your Mode Control Node
            Node(
                package="mode_control_pkg",
                executable="mode_control_node",
                name="mode_control_node",
                output="screen",
            ),
        ]
    )
