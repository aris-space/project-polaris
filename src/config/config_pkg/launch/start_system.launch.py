import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_pkg_dir = get_package_share_directory("config_pkg")

    use_navsat_transform_arg_value = LaunchConfiguration("use_navsat_transform")
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
            "use_navsat_transform": use_navsat_transform_arg_value,
            "use_global_ekf": use_global_ekf_arg_value,
            "gps_fix_topic": gps_fix_topic_arg_value,
            "p_surface_pa": p_surface_pa_arg_value,
            "start_ekf": start_ekf_arg_value,
        }.items(),
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[
            os.path.join(config_pkg_dir, "config", "foxglove.yaml"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_navsat_transform",
                default_value="true",
                description=(
                    "Launch navsat_transform_node. When true, publishes /odometry/gps for ekf_global."
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
            foxglove_node,
        ]
    )
