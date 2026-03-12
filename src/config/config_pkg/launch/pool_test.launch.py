import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from datetime import datetime
from config_pkg.constants import Logs


def generate_launch_description():
    # IncludeLaunchDescription arguments must remain launch substitutions/strings.
    respawn_arg_value = LaunchConfiguration("respawn", default="true")
    respawn_delay_arg_value = LaunchConfiguration("respawn_delay", default="2.0")
    # use_ntrip_arg_value = LaunchConfiguration("use_ntrip")
    # ntrip_use_https_arg_value = LaunchConfiguration("ntrip_use_https")
    # ntrip_host_arg_value = LaunchConfiguration("ntrip_host")
    # ntrip_port_arg_value = LaunchConfiguration("ntrip_port")
    # ntrip_mountpoint_arg_value = LaunchConfiguration("ntrip_mountpoint")
    # ntrip_version_arg_value = LaunchConfiguration("ntrip_version")
    # ntrip_username_arg_value = LaunchConfiguration("ntrip_username")
    # ntrip_password_arg_value = LaunchConfiguration("ntrip_password")

    # Reuse launch args so local actions honor CLI overrides.
    respawn = respawn_arg_value
    respawn_delay = respawn_delay_arg_value

    # 1. Find the path to the child package
    mode_control_pkg_dir = get_package_share_directory("mode_control_pkg")
    mavlink_bridge_pkg_dir = get_package_share_directory("mavlink_bridge")
    # gnss_bringup_pkg_dir = get_package_share_directory("gnss_bringup_pkg")
    ultrasonic_driver_pkg_dir = get_package_share_directory("ultrasonic_driver")
    temperature_sensor_pkg_dir = get_package_share_directory("temperature_sensor_pkg")
    xsens_mti_pkg_dir = get_package_share_directory("xsens_mti_ros2_driver")
    dvl_a50_pkg_dir = get_package_share_directory("dvl_a50_pkg")
    usb_cam_pkg_dir = get_package_share_directory("usb_cam_pkg")

    mode_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mode_control_pkg_dir, "launch", "launch_mode_control.launch.py"
            )
        ),
        launch_arguments={
            "respawn": respawn_arg_value,
            "respawn_delay": respawn_delay_arg_value,
        }.items(),
    )

    mavlink_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mavlink_bridge_pkg_dir, "launch", "mavlink_bridge.launch.py")
        ),
        launch_arguments={
            "respawn": respawn_arg_value,
            "respawn_delay": respawn_delay_arg_value,
        }.items(),
    )

    # gnss_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(gnss_bringup_pkg_dir, "launch", "launch_gnss_x20p.launch.py")
    #     ),
    #     launch_arguments={
    #         "respawn": respawn_arg_value,
    #         "respawn_delay": respawn_delay_arg_value,
    #         "use_ntrip": use_ntrip_arg_value,
    #         "ntrip_use_https": ntrip_use_https_arg_value,
    #         "ntrip_host": ntrip_host_arg_value,
    #         "ntrip_port": ntrip_port_arg_value,
    #         "ntrip_mountpoint": ntrip_mountpoint_arg_value,
    #         "ntrip_version": ntrip_version_arg_value,
    #         "ntrip_username": ntrip_username_arg_value,
    #         "ntrip_password": ntrip_password_arg_value,
    #     }.items(),
    # )

    ultrasonic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ultrasonic_driver_pkg_dir, "launch", "front_and_top.launch.py")
        ),
        launch_arguments={
            "respawn": respawn_arg_value,
            "respawn_delay": respawn_delay_arg_value,
        }.items(),
    )

    temperature_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                temperature_sensor_pkg_dir,
                "launch",
                "launch_temperature_sensors.launch.py",
            )
        ),
        launch_arguments={
            "respawn": respawn_arg_value,
            "respawn_delay": respawn_delay_arg_value,
        }.items(),
    )

    xsens_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xsens_mti_pkg_dir, "launch", "xsens_mti_node.launch.py")
        ),
        launch_arguments={
            "respawn": respawn_arg_value,
            "respawn_delay": respawn_delay_arg_value,
        }.items(),
    )

    dvl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dvl_a50_pkg_dir, "launch", "launch_dvl.py")
        ),
        launch_arguments={
            "respawn": respawn_arg_value,
            "respawn_delay": respawn_delay_arg_value,
        }.items(),
    )

    usb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(usb_cam_pkg_dir, "launch", "launch_cameras.py")
        ),
        launch_arguments={
            "respawn": respawn_arg_value,
            "respawn_delay": respawn_delay_arg_value,
        }.items(),
    )

    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge_node",
        output="screen",
        respawn=respawn,
        respawn_delay=respawn_delay,
    )

    timestamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    bag_path = os.path.join(Logs.ROSBAG_DIR, f"bag_pool_test_{timestamp}")

    rosbag_record = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-a", "-s", "mcap", "-o", bag_path],
        output="screen",
        respawn=respawn,
        respawn_delay=respawn_delay,
    )

    ping_sonar_node = Node(
        package="ping_sonar",
        executable="ice_measurement",
        name="ice_measurement_publisher",
        output="screen",
        respawn=respawn,
        respawn_delay=respawn_delay,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "respawn",
                default_value="true",
                description="Automatically relaunch processes if they exit/crash.",
            ),
            DeclareLaunchArgument(
                "respawn_delay",
                default_value="2.0",
                description="Seconds to wait before restarting a crashed process.",
            ),
            # DeclareLaunchArgument(
            #     "use_ntrip",
            #     default_value=EnvironmentVariable("USE_NTRIP", default_value="true"),
            #     description="Enable NTRIP client inside GNSS bringup.",
            # ),
            # DeclareLaunchArgument(
            #     "ntrip_use_https",
            #     default_value=EnvironmentVariable("NTRIP_USE_HTTPS", default_value="false"),
            # ),
            # DeclareLaunchArgument(
            #     "ntrip_host",
            #     default_value=EnvironmentVariable("NTRIP_HOST", default_value="www.swipos.ch"),
            # ),
            # DeclareLaunchArgument(
            #     "ntrip_port",
            #     default_value=EnvironmentVariable("NTRIP_PORT", default_value="2101"),
            # ),
            # DeclareLaunchArgument(
            #     "ntrip_mountpoint",
            #     default_value=EnvironmentVariable(
            #         "NTRIP_MOUNTPOINT", default_value="MSM_GISGEO_LV95LHN95"
            #     ),
            # ),
            # DeclareLaunchArgument(
            #     "ntrip_version",
            #     default_value=EnvironmentVariable("NTRIP_VERSION", default_value="Ntrip/1.0"),
            # ),
            # DeclareLaunchArgument(
            #     "ntrip_username",
            #     default_value=EnvironmentVariable("NTRIP_USERNAME", default_value=""),
            # ),
            # DeclareLaunchArgument(
            #     "ntrip_password",
            #     default_value=EnvironmentVariable("NTRIP_PASSWORD", default_value=""),
            # ),
            mode_control_launch,
            mavlink_launch,
            # gnss_launch,
            ultrasonic_launch,
            temperature_launch,
            xsens_launch,
            dvl_launch,
            usb_cam_launch,
            ping_sonar_node,
            foxglove_bridge_node,
            rosbag_record,
        ]
    )
