import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.conditions import IfCondition


def generate_launch_description():
    ld = LaunchDescription()

    waterlinked_node_params = os.path.join(
        get_package_share_directory("uwgpsg2_ros2_interface"),
        "config",
        "waterlinked_node_params.yaml",
    )

    waterlinked_localization_node = Node(
        namespace="waterlinked/ugps",
        name="waterlinked_localization_node",
        package="uwgpsg2_ros2_interface",
        executable="uwgpsg2_ros2_interface",
        output="screen",
        emulate_tty=True,
        parameters=[waterlinked_node_params],
        # remappings=gstreamer_remappings,
        remappings=[
            (
                "fix",
                "/waterlinked_ugps/fix",
            ),
            ("navrelposned", "/waterlinked_ugps/navrelposned"),
            (
                "locator_position_relative_wrt_topside",
                "/waterlinked_ugps/locator_position_relative_wrt_topside",
            ),
            ("locator_position_global", "/waterlinked_ugps/locator_position_global"),
            (
                "locator_position_topside_ned",
                "/waterlinked_ugps/locator_position_topside_ned",
            ),
        ],
        arguments=[],
    )

    ###################################################################

    ld.add_action(waterlinked_localization_node)
    return ld
