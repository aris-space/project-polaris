"""WaterLinked UGPS G2 (SBL) bottom-side interface.

Static TF ``base_link`` -> ``sbl_link`` matches CAD (CENTER_OF_MASS_LINK -> SBL_LINK);
assumes ``base_link`` coincides with center of mass (same as IMU/DVL/GNSS).
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_nodes(context):
    respawn = LaunchConfiguration("respawn")
    # ExecuteProcess expects a numeric delay, so resolve LaunchConfiguration here.
    respawn_delay = float(LaunchConfiguration("respawn_delay").perform(context))

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
                "locator_acoustic_quality",
                "/waterlinked_ugps/locator_acoustic_quality",
            ),
            (
                "locator_position_topside_ned",
                "/waterlinked_ugps/locator_position_topside_ned",
            ),
        ],
        arguments=[],
        respawn=respawn,
        respawn_delay=respawn_delay,
    )

    # CAD: translation CENTER_OF_MASS_LINK -> SBL_LINK (Waterlinked), no rotation (m).
    static_tf_base_to_sbl = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_sbl",
        arguments=[
            "--x",
            "-0.544225",
            "--y",
            "-0.000086",
            "--z",
            "0.203593",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "sbl_link",
        ],
        output="screen",
        respawn=respawn,
        respawn_delay=respawn_delay,
    )

    return [waterlinked_localization_node, static_tf_base_to_sbl]


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "respawn",
            default_value="true",
            description="Automatically relaunch nodes if they exit/crashes.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "respawn_delay",
            default_value="2.0",
            description="Seconds to wait before restarting a crashed node.",
        )
    )

    ld.add_action(OpaqueFunction(function=_build_nodes))
    return ld
