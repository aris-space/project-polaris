import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_navsat_transform_arg_value = LaunchConfiguration("use_navsat_transform")
    use_global_ekf_arg_value = LaunchConfiguration("use_global_ekf")
    gps_fix_topic_arg_value = LaunchConfiguration("gps_fix_topic")
    p_surface_pa_arg_value = LaunchConfiguration("p_surface_pa")

    ekf_localization_pkg_dir = get_package_share_directory("ekf_localization_pkg")
    pressure_pose_pkg_dir = get_package_share_directory("pressure_pose_pkg")

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ekf_localization_pkg_dir, "launch", "ekf_localization.launch.py"
            )
        ),
        launch_arguments={
            "use_navsat_transform": use_navsat_transform_arg_value,
            "use_global_ekf": use_global_ekf_arg_value,
            "gps_fix_topic": gps_fix_topic_arg_value,
        }.items(),
    )

    pressure_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pressure_pose_pkg_dir, "launch", "pressure_z_ned_to_pose.launch.py"
            )
        ),
        launch_arguments={
            "p_surface_pa": p_surface_pa_arg_value,
        }.items(),
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
            localization_launch,
            pressure_launch,
        ]
    )
