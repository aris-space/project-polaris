from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic",
                default_value="/imu/data",
                description="sensor_msgs/Imu subscription (remap as needed).",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/sensors/imu/orientation_rpy",
                description="geometry_msgs/Vector3Stamped: x=roll, y=pitch, z=yaw (rad).",
            ),
            Node(
                package="imu_orientation_pkg",
                executable="imu_to_rpy_node",
                name="imu_to_rpy_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": ParameterValue(
                            LaunchConfiguration("input_topic"), value_type=str
                        ),
                        "output_topic": ParameterValue(
                            LaunchConfiguration("output_topic"), value_type=str
                        ),
                    }
                ],
            ),
        ]
    )
