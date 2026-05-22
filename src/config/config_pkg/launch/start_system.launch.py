import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from config_pkg.constants import Comms, Logs


def generate_launch_description():
    config_pkg_dir = get_package_share_directory("config_pkg")
    ice_estimates_pkg_dir = get_package_share_directory("ice_estimates")

    use_gnss_datum_watchdog_arg_value = LaunchConfiguration("use_gnss_datum_watchdog")
    use_global_ekf_arg_value = LaunchConfiguration("use_global_ekf")
    gps_fix_topic_arg_value = LaunchConfiguration("gps_fix_topic")
    p_surface_pa_arg_value = LaunchConfiguration("p_surface_pa")
    start_ekf_arg_value = LaunchConfiguration("start_ekf")

    manual_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg_dir, "launch", "manual_control.launch.py")
        )
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg_dir, "launch", "sensors.launch.py")
        )
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(config_pkg_dir, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_gnss_datum_watchdog": use_gnss_datum_watchdog_arg_value,
            "use_global_ekf": use_global_ekf_arg_value,
            "gps_fix_topic": gps_fix_topic_arg_value,
            "p_surface_pa": p_surface_pa_arg_value,
            "start_ekf": start_ekf_arg_value,
        }.items(),
    )

    archimedes_measurement_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ice_estimates_pkg_dir, "launch", "launch_archimedes_measurement.launch.py")
        )
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[
            os.path.join(config_pkg_dir, "config", "foxglove.yaml"),
        ],
    )

    recorder_controller_node = Node(
        package="config_pkg",
        executable="recorder_controller",
        name="recorder_controller_node",
        parameters=[
            {"base_output_dir": Logs.ROSBAG_DIR},
            {"storage_id": "mcap"},
        ],
    )

    recorder_controller_node = Node(
        package="config_pkg",
        executable="recorder_controller",
        name="recorder_controller_node",
        parameters=[
            {"base_output_dir": Logs.ROSBAG_DIR},
            {"storage_id": "mcap"},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_gnss_datum_watchdog",
                default_value="true",
                description=(
                    "Enable gnss_datum_watchdog: gates and republishes GPS, then "
                    "activates the in-place global EKF stack on first quality lock."
                ),
            ),
            DeclareLaunchArgument(
                "use_global_ekf",
                default_value="true",
                description="Launch global map-frame EKF node.",
            ),
            DeclareLaunchArgument(
                "gps_fix_topic",
                default_value="/gps/selected",
                description=(
                    "NavSatFix topic remapped to navsat_transform gps/fix (selector: GNSS + SBL). "
                    "Override e.g. /fix if your stack publishes fixes only there."
                ),
            ),
            DeclareLaunchArgument(
                "p_surface_pa",
                default_value="101325.0",
                description=(
                    "Surface reference pressure (Pa) for depth from absolute pressure. "
                    "Override with the value read in BlueOS/QGC at the surface (1 hPa = 100 Pa)."
                ),
            ),
            DeclareLaunchArgument(
                "start_ekf",
                default_value="false",
                description=(
                    "Launch the localization EKF stack with the rest of the system. "
                    "Default false so the IMU (VRU mode) and pressure can settle on land with "
                    "the boat aligned to true East before the EKF starts in water. "
                    "Launch the EKF separately once in water with: "
                    "`ros2 launch ekf_localization_pkg ekf_localization.launch.py "
                    "gps_fix_topic:=/gps/selected`."
                ),
            ),
            manual_control_launch,
            sensors_launch,
            navigation_launch,
            archimedes_measurement_launch,
            foxglove_node,
            recorder_controller_node,
        ]
    )
