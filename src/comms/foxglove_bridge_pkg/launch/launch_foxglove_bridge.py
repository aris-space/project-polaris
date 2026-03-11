from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Foxglove Bridge (Talks to the foxglove studio)
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                parameters=[
                    {
                        "port": 8765,
                        "address": "0.0.0.0",
                        "capabilities": ["clientPublish", "connectionGraph", "assets"],
                        "send_buffer_limit": 10000000,
                    }
                ],
            )
        ]
    )
