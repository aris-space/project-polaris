import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    package_dir = get_package_share_directory("gnss_bringup_pkg")
    default_config = os.path.join(package_dir, "config", "ublox_x20p_rover.yaml")

    gnss_config = LaunchConfiguration("gnss_config")
    log_level = LaunchConfiguration("log_level")
    namespace = LaunchConfiguration("namespace")

    use_ntrip = LaunchConfiguration("use_ntrip")
    ntrip_use_https = LaunchConfiguration("ntrip_use_https")
    ntrip_host = LaunchConfiguration("ntrip_host")
    ntrip_port = LaunchConfiguration("ntrip_port")
    ntrip_mountpoint = LaunchConfiguration("ntrip_mountpoint")
    ntrip_username = LaunchConfiguration("ntrip_username")
    ntrip_password = LaunchConfiguration("ntrip_password")
    ntrip_maxage_conn = LaunchConfiguration("ntrip_maxage_conn")

    gnss_container = ComposableNodeContainer(
        name="ublox_dgnss_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        composable_node_descriptions=[
            ComposableNode(
                package="ublox_dgnss_node",
                plugin="ublox_dgnss::UbloxDGNSSNode",
                name="ublox_dgnss",
                namespace=namespace,
                parameters=[gnss_config],
            ),
            ComposableNode(
                package="ublox_nav_sat_fix_hp_node",
                plugin="ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode",
                name="ublox_nav_sat_fix_hp",
                namespace=namespace,
            ),
        ],
    )

    ntrip_container = ComposableNodeContainer(
        name="ntrip_client_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        condition=IfCondition(use_ntrip),
        arguments=["--ros-args", "--log-level", log_level],
        composable_node_descriptions=[
            ComposableNode(
                package="ntrip_client_node",
                plugin="ublox_dgnss::NTRIPClientNode",
                name="ntrip_client",
                namespace=namespace,
                parameters=[
                    {
                        "use_https": ntrip_use_https,
                        "host": ntrip_host,
                        "port": ntrip_port,
                        "mountpoint": ntrip_mountpoint,
                        "username": ntrip_username,
                        "password": ntrip_password,
                        "maxage_conn": ntrip_maxage_conn,
                    }
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gnss_config", default_value=default_config),
            DeclareLaunchArgument("log_level", default_value="INFO"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_ntrip", default_value="false"),
            DeclareLaunchArgument("ntrip_use_https", default_value="true"),
            DeclareLaunchArgument("ntrip_host", default_value=""),
            DeclareLaunchArgument("ntrip_port", default_value="443"),
            DeclareLaunchArgument("ntrip_mountpoint", default_value=""),
            DeclareLaunchArgument(
                "ntrip_username",
                default_value=EnvironmentVariable(
                    "NTRIP_USERNAME", default_value=""
                ),
            ),
            DeclareLaunchArgument(
                "ntrip_password",
                default_value=EnvironmentVariable(
                    "NTRIP_PASSWORD", default_value=""
                ),
            ),
            DeclareLaunchArgument("ntrip_maxage_conn", default_value="30"),
            gnss_container,
            ntrip_container,
        ]
    )
