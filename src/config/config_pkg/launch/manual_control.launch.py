import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from datetime import datetime
from config_pkg.constants import Logs


def generate_launch_description():

    # 1. Find the path to the child package
    mode_control_pkg_dir = get_package_share_directory("mode_control_pkg")
    mavlink_bridge_pkg_dir = get_package_share_directory("mavlink_bridge")
    gnss_bringup_pkg_dir = get_package_share_directory("gnss_bringup_pkg")

    mode_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mode_control_pkg_dir, "launch", "launch_mode_control.launch.py"
            )
        )
    )

    mavlink_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mavlink_bridge_pkg_dir, "launch", "mavlink_bridge.launch.py")
        )
    )

    gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gnss_bringup_pkg_dir, "launch", "launch_gnss_x20p.launch.py")
        )
    )

    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge_node",
        output="screen",
    )


    timestamp = datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
    bag_path = os.path.join(Logs.ROSBAG_DIR, f"bag_{timestamp}")
    
    rosbag_record = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-a", "-s", "mcap", "-o", bag_path],
        output="screen",
    )

    return LaunchDescription(
        [
            mode_control_launch,
            mavlink_launch,
            gnss_launch,
            foxglove_bridge_node,
            rosbag_record,
        ]
    )
