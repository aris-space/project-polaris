from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    serial_device = LaunchConfiguration('serial_device')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='ultrasonic_front',
            description='Namespace for the ultrasonic node'
        ),
        DeclareLaunchArgument(
            'serial_device',
            default_value='/dev/ttyUSB0',
            description='Serial device path for the ultrasonic sensor'
        ),
        Node(
            package='ultrasonic_driver',
            executable='ultrasonic_node',
            name='ultrasonic_sensor_node',
            namespace=namespace,
            output='screen',
            parameters=[
                {'serial_device': serial_device},
            ],
        ),
    ])
