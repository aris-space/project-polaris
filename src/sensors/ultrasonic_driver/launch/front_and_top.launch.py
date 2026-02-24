from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from config_pkg.constants import Ports

def generate_launch_description():
    # 1. Define the shared serial device configuration
    serial_device = LaunchConfiguration('serial_device')

    # 2. Define the sensors we want to launch
    sensors = [
        {'name': 'front_sensor', 'ns': 'front', 'port': Ports.FRONT_ULTRASONIC_PORT},
        {'name': 'top_sensor',   'ns': 'top',   'port': Ports.TOP_ULTRASONIC_PORT}
    ]

    nodes = []

    for sensor in sensors:
        nodes.append(
            Node(
                package='ultrasonic_driver',
                executable='ultrasonic_node',
                name=sensor['name'],
                namespace=sensor['ns'],
                output='screen',
                parameters=[
                    {'serial_device': sensor['port']},
                ],
            )
        )

    return LaunchDescription(nodes)