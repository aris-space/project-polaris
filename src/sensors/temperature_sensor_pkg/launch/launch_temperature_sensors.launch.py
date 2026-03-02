from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="temperature_sensor_pkg",
                executable="temperature_sensor_node",
                name="temperature_sensor_node",
                output="screen",
            )
        ]
    )
