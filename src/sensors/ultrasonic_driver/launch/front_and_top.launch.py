from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from config_pkg.constants import Ports

def generate_launch_description():
    respawn = True
    respawn_delay = 2.0

    # 1. Define the sensors we want to launch
    sensors = [
        {'name': 'front_ultrasonic_sensor', 'ns': 'front', 'publish_frequency_hz': 20.0, 'port': Ports.FRONT_ULTRASONIC_PORT},
        {'name': 'top_ultrasonic_sensor',   'ns': 'top', 'publish_frequency_hz': 10.0, 'port': Ports.TOP_ULTRASONIC_PORT}
    ]

    nodes = []
    nodes.append(
        DeclareLaunchArgument(
            "respawn",
            default_value="true",
            description="Automatically relaunch node if it exits/crashes.",
        )
    )
    nodes.append(
        DeclareLaunchArgument(
            "respawn_delay",
            default_value="2.0",
            description="Seconds to wait before restarting a crashed node.",
        )
    )

    for sensor in sensors:
        nodes.append(
            Node(
                package='ultrasonic_driver',
                executable='ultrasonic_node',
                name=sensor['name'],
                namespace=sensor['ns'],
                output='screen',
                respawn=respawn,
                respawn_delay=respawn_delay,
                parameters=[
                    {'serial_device': sensor['port']},
                    {'publish_frequency_hz': sensor['publish_frequency_hz']},   
                ],
            )
        )

    return LaunchDescription(nodes)