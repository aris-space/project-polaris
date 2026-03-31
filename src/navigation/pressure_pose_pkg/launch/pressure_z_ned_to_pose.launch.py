from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "p_surface_pa",
                default_value="101325.0",
                description=(
                    "Absolute pressure at the water surface (Pa). Read from BlueOS/QGC at the "
                    "surface (same units as sensor_msgs/FluidPressure). 1 hPa = 100 Pa."
                ),
            ),
            Node(
                package="pressure_pose_pkg",
                executable="pressure_z_ned_to_pose_node",
                name="pressure_z_ned_to_pose_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/pixhawk/scaled_pressure",
                        "output_topic": "/sensors/pressure/pose_enu",
                        "output_frame_id": "odom",
                        "z_variance": 0.04,
                        "unused_variance": 1000000.0,
                        "water_density_kg_m3": 1000.0,
                        "gravity_m_s2": 9.80665,
                        "p_surface_pa": ParameterValue(
                            LaunchConfiguration("p_surface_pa"), value_type=float
                        ),
                        "fluid_pressure_is_gauge": False,
                        "sensor_z_offset_m": 0.0,
                    }
                ],
            ),
        ]
    )
