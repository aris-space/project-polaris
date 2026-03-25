from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    keller_26x_node = Node(
        package='keller_26x_pkg',
        executable='keller_26x_node',
        name='keller_26x_pressure',
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        keller_26x_node,
    ])