from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ice_estimates",
            executable="archimedes_touch",
            name="ice_estimation",
            output="screen",
        ),
    ])
