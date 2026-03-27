import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from config_pkg.constants import Logs


def create_rosbag_record(context, default_bag_prefix):
    bag_name = LaunchConfiguration("bag_name").perform(context).strip()
    timestamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    bag_base_name = (
        f"{bag_name}_{timestamp}" if bag_name else f"{default_bag_prefix}_{timestamp}"
    )
    bag_path = os.path.join(Logs.ROSBAG_DIR, bag_base_name)

    return [
        ExecuteProcess(
            cmd=["ros2", "bag", "record", "-a", "-s", "mcap", "-o", bag_path],
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
            rosbag_record,
        ]
    )
