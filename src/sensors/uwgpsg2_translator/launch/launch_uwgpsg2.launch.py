from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    respawn = LaunchConfiguration("respawn")
    respawn_delay = LaunchConfiguration("respawn_delay")
    max_horizontal_accuracy_m = LaunchConfiguration("max_horizontal_accuracy_m")
    ubx_nav_hp_pos_llh_topic = LaunchConfiguration("ubx_nav_hp_pos_llh_topic")

    # Get the path to the waterlinked launch file
    waterlinked_launch_file = os.path.join(
        get_package_share_directory("uwgpsg2_ros2_interface"),
        "launch",
        "start_waterlinked_interface_bttm_side.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "respawn",
                default_value="true",
                description="Automatically relaunch node if it exits/crashes.",
            ),
            DeclareLaunchArgument(
                "respawn_delay",
                default_value="2.0",
                description="Seconds to wait before restarting a crashed node.",
            ),
            DeclareLaunchArgument(
                "max_horizontal_accuracy_m",
                default_value="4.0",
                description="Publish /fix to /gps/selected only if horizontal accuracy (m) is at most this.",
            ),
            DeclareLaunchArgument(
                "ubx_nav_hp_pos_llh_topic",
                default_value="/ubx_nav_hp_pos_llh",
                description=(
                    "UBXNavHPPosLLH topic for receiver h_acc (set '' to disable). "
                    "Must match ublox stack namespace (Polaris gnss launch uses default namespace '')."
                ),
            ),
            # Include the waterlinked interface launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(waterlinked_launch_file),
                launch_arguments={
                    "respawn": respawn,
                    "respawn_delay": respawn_delay,
                }.items(),
            ),
            Node(
                package="uwgpsg2_translator",
                executable="to_navsatfix_translator",
                name="to_navsatfix_translator",
                output="screen",
                respawn=respawn,
                respawn_delay=respawn_delay,
            ),
            Node(
                package="uwgpsg2_translator",
                executable="selector",
                name="selector",
                output="screen",
                respawn=respawn,
                respawn_delay=respawn_delay,
                parameters=[
                    {
                        "max_horizontal_accuracy_m": ParameterValue(
                            max_horizontal_accuracy_m, value_type=float
                        ),
                        "ubx_nav_hp_pos_llh_topic": ubx_nav_hp_pos_llh_topic,
                    }
                ],
            ),
        ]
    )
