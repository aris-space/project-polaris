from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from config_pkg.constants import Ports


def generate_launch_description():
    serial_device = LaunchConfiguration("serial_device")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="default",
                description="Namespace for the ultrasonic node",
            ),
            DeclareLaunchArgument(
                "serial_device",
                default_value=Ports.FRONT_ULTRASONIC_PORT,
                description="Serial device path for the ultrasonic sensor",
            ),
            Node(
                package="ultrasonic_driver",
                executable="ultrasonic_node",
                name="ultrasonic_sensor_node",
                namespace="front",
                output="screen",
                parameters=[
                    {"serial_device": serial_device},
                    {"publish_frequency_hz": 20.0},
                ],
            ),
        ]
    )
