import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from config_pkg.constants import Comms


def generate_launch_description():
    config_pkg_dir = get_package_share_directory("config_pkg")

    gnss_respawn_arg_value = LaunchConfiguration("gnss_respawn")
    ultrasonic_respawn_arg_value = LaunchConfiguration("ultrasonic_respawn")
    temperature_respawn_arg_value = LaunchConfiguration("temperature_respawn")
    xsens_respawn_arg_value = LaunchConfiguration("xsens_respawn")
    dvl_respawn_arg_value = LaunchConfiguration("dvl_respawn")
    usb_cam_respawn_arg_value = LaunchConfiguration("usb_cam_respawn")
    ping_sonar_respawn_arg_value = LaunchConfiguration("ping_sonar_respawn")
    jetson_temperature_respawn_arg_value = LaunchConfiguration(
        "jetson_temperature_respawn"
    )
    respawn_delay_arg_value = LaunchConfiguration("respawn_delay")
    use_ntrip_arg_value = LaunchConfiguration("use_ntrip")
    ntrip_use_https_arg_value = LaunchConfiguration("ntrip_use_https")
    ntrip_host_arg_value = LaunchConfiguration("ntrip_host")
    ntrip_port_arg_value = LaunchConfiguration("ntrip_port")
    ntrip_mountpoint_arg_value = LaunchConfiguration("ntrip_mountpoint")
    ntrip_version_arg_value = LaunchConfiguration("ntrip_version")
    ntrip_username_arg_value = LaunchConfiguration("ntrip_username")
    ntrip_password_arg_value = LaunchConfiguration("ntrip_password")

    manual_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg_dir, "launch", "manual_control.launch.py")
        )
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg_dir, "launch", "sensors.launch.py")
        ),
        launch_arguments={
            "gnss_respawn": gnss_respawn_arg_value,
            "ultrasonic_respawn": ultrasonic_respawn_arg_value,
            "temperature_respawn": temperature_respawn_arg_value,
            "xsens_respawn": xsens_respawn_arg_value,
            "dvl_respawn": dvl_respawn_arg_value,
            "usb_cam_respawn": usb_cam_respawn_arg_value,
            "ping_sonar_respawn": ping_sonar_respawn_arg_value,
            "jetson_temperature_respawn": jetson_temperature_respawn_arg_value,
            "respawn_delay": respawn_delay_arg_value,
            "use_ntrip": use_ntrip_arg_value,
            "ntrip_use_https": ntrip_use_https_arg_value,
            "ntrip_host": ntrip_host_arg_value,
            "ntrip_port": ntrip_port_arg_value,
            "ntrip_mountpoint": ntrip_mountpoint_arg_value,
            "ntrip_version": ntrip_version_arg_value,
            "ntrip_username": ntrip_username_arg_value,
            "ntrip_password": ntrip_password_arg_value,
        }.items(),
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[
            os.path.join(config_pkg_dir, "config", "foxglove.yaml"),
            {"address": Comms.JETSON_IP_ADDRESS},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gnss_respawn",
                default_value="true",
                description="Respawn GNSS bringup nodes if they exit/crash.",
            ),
            DeclareLaunchArgument(
                "ultrasonic_respawn",
                default_value="true",
                description="Respawn ultrasonic nodes if they exit/crash.",
            ),
            DeclareLaunchArgument(
                "temperature_respawn",
                default_value="true",
                description="Respawn temperature sensor nodes if they exit/crash.",
            ),
            DeclareLaunchArgument(
                "xsens_respawn",
                default_value="true",
                description="Respawn Xsens IMU nodes if they exit/crash.",
            ),
            DeclareLaunchArgument(
                "dvl_respawn",
                default_value="true",
                description="Respawn DVL nodes if they exit/crash.",
            ),
            DeclareLaunchArgument(
                "usb_cam_respawn",
                default_value="false",
                description="Respawn USB camera nodes if they exit/crash.",
            ),
            DeclareLaunchArgument(
                "ping_sonar_respawn",
                default_value="true",
                description="Respawn ping sonar node if it exits/crashes.",
            ),
            DeclareLaunchArgument(
                "jetson_temperature_respawn",
                default_value="true",
                description="Respawn Jetson temperature node if it exits/crashes.",
            ),
            DeclareLaunchArgument(
                "respawn_delay",
                default_value="2.0",
                description="Seconds to wait before restarting a crashed process.",
            ),
            DeclareLaunchArgument(
                "use_ntrip",
                default_value=EnvironmentVariable("USE_NTRIP", default_value="true"),
                description="Enable NTRIP client inside GNSS bringup.",
            ),
            DeclareLaunchArgument(
                "ntrip_use_https",
                default_value=EnvironmentVariable("NTRIP_USE_HTTPS", default_value="false"),
            ),
            DeclareLaunchArgument(
                "ntrip_host",
                default_value=EnvironmentVariable("NTRIP_HOST", default_value="www.swipos.ch"),
            ),
            DeclareLaunchArgument(
                "ntrip_port",
                default_value=EnvironmentVariable("NTRIP_PORT", default_value="2101"),
            ),
            DeclareLaunchArgument(
                "ntrip_mountpoint",
                default_value=EnvironmentVariable(
                    "NTRIP_MOUNTPOINT", default_value="MSM_GISGEO_LV95LHN95"
                ),
            ),
            DeclareLaunchArgument(
                "ntrip_version",
                default_value=EnvironmentVariable("NTRIP_VERSION", default_value="Ntrip/1.0"),
            ),
            DeclareLaunchArgument(
                "ntrip_username",
                default_value=EnvironmentVariable("NTRIP_USERNAME", default_value=""),
            ),
            DeclareLaunchArgument(
                "ntrip_password",
                default_value=EnvironmentVariable("NTRIP_PASSWORD", default_value=""),
            ),
            manual_control_launch,
            sensors_launch,
            foxglove_node,
        ]
    )
