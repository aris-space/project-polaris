import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # IncludeLaunchDescription arguments must remain launch substitutions/strings.
    gnss_respawn_arg_value = LaunchConfiguration("gnss_respawn", default="true")
    ultrasonic_respawn_arg_value = LaunchConfiguration(
        "ultrasonic_respawn", default="false"
    )
    temperature_respawn_arg_value = LaunchConfiguration(
        "temperature_respawn", default="true"
    )
    xsens_respawn_arg_value = LaunchConfiguration("xsens_respawn", default="true")
    dvl_respawn_arg_value = LaunchConfiguration("dvl_respawn", default="true")
    keller_respawn_arg_value = LaunchConfiguration("keller_respawn", default="true")
    usb_cam_respawn_arg_value = LaunchConfiguration("usb_cam_respawn", default="false")
    uwgpsg2_respawn_arg_value = LaunchConfiguration("uwgpsg2_respawn", default="true")
    ping_sonar_respawn_arg_value = LaunchConfiguration(
        "ping_sonar_respawn", default="true"
    )
    jetson_temperature_respawn_arg_value = LaunchConfiguration(
        "jetson_temperature_respawn", default="true"
    )
    enable_water_sos_arg_value = LaunchConfiguration("enable_water_sos", default="false")
    water_sos_respawn_arg_value = LaunchConfiguration(
        "water_sos_respawn", default="true"
    )
    respawn_delay_arg_value = LaunchConfiguration("respawn_delay", default="2.0")
    use_ntrip_arg_value = LaunchConfiguration("use_ntrip")
    ntrip_use_https_arg_value = LaunchConfiguration("ntrip_use_https")
    ntrip_host_arg_value = LaunchConfiguration("ntrip_host")
    ntrip_port_arg_value = LaunchConfiguration("ntrip_port")
    ntrip_mountpoint_arg_value = LaunchConfiguration("ntrip_mountpoint")
    ntrip_version_arg_value = LaunchConfiguration("ntrip_version")
    ntrip_username_arg_value = LaunchConfiguration("ntrip_username")
    ntrip_password_arg_value = LaunchConfiguration("ntrip_password")

    # Reuse launch args so local actions honor CLI overrides.
    ping_sonar_respawn = ping_sonar_respawn_arg_value
    jetson_temperature_respawn = jetson_temperature_respawn_arg_value
    respawn_delay = respawn_delay_arg_value

    # 1. Find the path to the child package
    gnss_bringup_pkg_dir = get_package_share_directory("gnss_bringup_pkg")
    ultrasonic_driver_pkg_dir = get_package_share_directory("ultrasonic_driver")
    temperature_sensor_pkg_dir = get_package_share_directory("temperature_sensor_pkg")
    xsens_mti_pkg_dir = get_package_share_directory("xsens_mti_ros2_driver")
    dvl_a50_pkg_dir = get_package_share_directory("dvl_a50_pkg")
    keller_26x_pkg_dir = get_package_share_directory("keller_26x_pkg")
    usb_cam_pkg_dir = get_package_share_directory("usb_cam_pkg")
    uwgpsg2_translator_pkg_dir = get_package_share_directory("uwgpsg2_translator")
    water_properties_pkg_dir = get_package_share_directory("water_properties_pkg")

    gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gnss_bringup_pkg_dir, "launch", "launch_gnss_x20p.launch.py")
        ),
        launch_arguments={
            "respawn": gnss_respawn_arg_value,
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

    ultrasonic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ultrasonic_driver_pkg_dir, "launch", "front_and_top.launch.py")
        ),
        launch_arguments={
            "respawn": ultrasonic_respawn_arg_value,
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
            "respawn": temperature_respawn_arg_value,
            "respawn_delay": respawn_delay_arg_value,
        }.items(),
    )

    xsens_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xsens_mti_pkg_dir, "launch", "xsens_mti_node.launch.py")
        ),
        launch_arguments={
            "respawn": xsens_respawn_arg_value,
            "respawn_delay": respawn_delay_arg_value,
        }.items(),
    )

    dvl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dvl_a50_pkg_dir, "launch", "launch_dvl.launch.py")
        ),
        launch_arguments={
            "respawn": dvl_respawn_arg_value,
            "respawn_delay": respawn_delay_arg_value,
        }.items(),
    )

    keller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(keller_26x_pkg_dir, "launch", "keller_26x.launch.py")
        ),
        launch_arguments={
            "respawn": keller_respawn_arg_value,
            "respawn_delay": respawn_delay_arg_value,
        }.items(),
    )

    usb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(usb_cam_pkg_dir, "launch", "launch_cameras.launch.py")
        ),
        launch_arguments={
            "respawn": usb_cam_respawn_arg_value,
            "respawn_delay": respawn_delay_arg_value,
        }.items(),
    )

    uwgpsg2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(uwgpsg2_translator_pkg_dir, "launch", "launch_uwgpsg2.launch.py")
        ),
        launch_arguments={
            "respawn": uwgpsg2_respawn_arg_value,
            "respawn_delay": respawn_delay_arg_value,
        }.items(),
    )

    water_sos_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(water_properties_pkg_dir, "launch", "water_sos.launch.py")
        ),
        condition=IfCondition(enable_water_sos_arg_value),
        launch_arguments={
            "respawn": water_sos_respawn_arg_value,
            "respawn_delay": respawn_delay_arg_value,
        }.items(),
    )

    ping_sonar_node = Node(
        package="ping_sonar",
        executable="ice_measurement",
        name="ice_measurement_publisher",
        output="screen",
        respawn=ping_sonar_respawn,
        respawn_delay=respawn_delay,
    )

    jetson_temperature_node = Node(
        package="jetson_temperature",
        executable="jetson_temperature",
        name="jetson_temperature_monitor_node",
        output="screen",
        respawn=jetson_temperature_respawn,
        respawn_delay=respawn_delay,
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
                "keller_respawn",
                default_value="true",
                description="Respawn Keller pressure sensor nodes if they exit/crash.",
            ),
            DeclareLaunchArgument(
                "usb_cam_respawn",
                default_value="false",
                description="Respawn USB camera nodes if they exit/crash.",
            ),
            DeclareLaunchArgument(
                "uwgpsg2_respawn",
                default_value="true",
                description="Respawn UGPS G2 nodes if they exit/crash.",
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
                "enable_water_sos",
                default_value="false",
                description="Launch the water speed-of-sound node.",
            ),
            DeclareLaunchArgument(
                "water_sos_respawn",
                default_value="true",
                description="Respawn water speed-of-sound node if it exits/crashes.",
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
                default_value=EnvironmentVariable(
                    "NTRIP_USE_HTTPS", default_value="false"
                ),
            ),
            DeclareLaunchArgument(
                "ntrip_host",
                default_value=EnvironmentVariable(
                    "NTRIP_HOST", default_value="www.swipos.ch"
                ),
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
                default_value=EnvironmentVariable(
                    "NTRIP_VERSION", default_value="Ntrip/1.0"
                ),
            ),
            DeclareLaunchArgument(
                "ntrip_username",
                default_value=EnvironmentVariable("NTRIP_USERNAME", default_value=""),
            ),
            DeclareLaunchArgument(
                "ntrip_password",
                default_value=EnvironmentVariable("NTRIP_PASSWORD", default_value=""),
            ),
            gnss_launch,
            ultrasonic_launch,
            temperature_launch,
            xsens_launch,
            dvl_launch,
            keller_launch,
            usb_cam_launch,
            uwgpsg2_launch,
            water_sos_launch,
            ping_sonar_node,
            jetson_temperature_node,
        ]
    )
