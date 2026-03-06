import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import datetime
from config_pkg.constants import Logs


def generate_launch_description():
    # IncludeLaunchDescription arguments must remain launch substitutions/strings.
    respawn_arg_value = LaunchConfiguration("respawn")
    respawn_delay_arg_value = LaunchConfiguration("respawn_delay")

    # Node action fields can safely use concrete python values.
    respawn = True
    respawn_delay = 2.0

    # 1. Find the path to the child package
    mode_control_pkg_dir = get_package_share_directory("mode_control_pkg")
    mavlink_bridge_pkg_dir = get_package_share_directory("mavlink_bridge")
    gnss_bringup_pkg_dir = get_package_share_directory("gnss_bringup_pkg")
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

    gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gnss_bringup_pkg_dir, "launch", "launch_gnss_x20p.launch.py")
        ),
        launch_arguments={
            "respawn": respawn_arg_value,
            "respawn_delay": respawn_delay_arg_value,
        }.items(),
    )

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
                temperature_sensor_pkg_dir, "launch", "launch_temperature_sensors.launch.py"
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


    timestamp = datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
    bag_path = os.path.join(Logs.ROSBAG_DIR, f"bag_{timestamp}")
    
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
            mode_control_launch,
            mavlink_launch,
            gnss_launch,
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
