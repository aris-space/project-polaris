from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launches the two different nodes: the mavlink to ros translator and the output monitor
    """
    return LaunchDescription(
        [
            Node(
                package="mavlink_bridge",
                executable="mavlink_publisher",
                name="mavlink_bridge_publisher",
                output="screen",
            ),
            Node(
                package="mavlink_bridge",
                executable="output_monitor",
                name="output_monitor",
                output="screen",
            ),
        ]
    )
