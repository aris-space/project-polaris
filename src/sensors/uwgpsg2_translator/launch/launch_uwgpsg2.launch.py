from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    respawn = LaunchConfiguration("respawn")
    respawn_delay = LaunchConfiguration("respawn_delay")

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
            ),
        ]
    )
