import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from config_pkg.constants import Logs


def create_rosbag_record(context, default_bag_prefix):
    bag_name = LaunchConfiguration("bag_name").perform(context).strip()
    exclude_regex = LaunchConfiguration("exclude_regex").perform(context).strip()
    timestamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    bag_base_name = (
        f"{bag_name}_{timestamp}" if bag_name else f"{default_bag_prefix}_{timestamp}"
    )
    bag_path = os.path.join(Logs.ROSBAG_DIR, bag_base_name)

    cmd = ["ros2", "bag", "record", "-a", "-s", "mcap", "-o", bag_path]
    if exclude_regex:
        cmd.extend(["--exclude", exclude_regex])

    return [
        ExecuteProcess(
            cmd=cmd,
            output="screen",
            # Avoid restarting recorder during shutdown and allow flush/finalization.
            respawn=False,
            sigterm_timeout="10",
            sigkill_timeout="10",
        )
    ]


def generate_launch_description():
    rosbag_record = OpaqueFunction(
        function=lambda context: create_rosbag_record(context, "bag_pool_test")
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_name",
                default_value="",
                description=(
                    "Optional rosbag base name. The launch system appends _YYYY_MM_DD-HH_MM_SS."
                ),
            ),
            DeclareLaunchArgument(
                "exclude_regex",
                default_value=".*/(image(_raw)?|image_raw|compressed|theora|depth/image.*|camera/image.*).*",
                description="Regex of topics to exclude from recording (defaults to camera/image feeds).",
            ),
            rosbag_record,
        ]
    )
