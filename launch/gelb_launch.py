from launch import LaunchDescription
from launch_ros.actions import Node

''' Launch file to start the thrust control node and joystick node single motor thrust control forward and backwards. '''


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pt_gelb_pkg",
            executable="thrust_control_fwbw",
            name="thrust_control_fwbw",
            output="screen",
        ),
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
        ),
    ])