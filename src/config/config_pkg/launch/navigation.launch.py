import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_gnss_datum_watchdog_arg_value = LaunchConfiguration("use_gnss_datum_watchdog")
    use_global_ekf_arg_value = LaunchConfiguration("use_global_ekf")
    gps_fix_topic_arg_value = LaunchConfiguration("gps_fix_topic")
    p_surface_pa_arg_value = LaunchConfiguration("p_surface_pa")
    start_ekf_arg_value = LaunchConfiguration("start_ekf")

    ekf_localization_pkg_dir = get_package_share_directory("ekf_localization_pkg")
    pressure_pose_pkg_dir = get_package_share_directory("pressure_pose_pkg")
    measurement_points_pkg_dir = get_package_share_directory("measurement_points_pkg")

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ekf_localization_pkg_dir, "launch", "ekf_localization.launch.py"
            )
        ),
        launch_arguments={
            "use_gnss_datum_watchdog": use_gnss_datum_watchdog_arg_value,
            "use_global_ekf": use_global_ekf_arg_value,
            "gps_fix_topic": gps_fix_topic_arg_value,
        }.items(),
        condition=IfCondition(start_ekf_arg_value),
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

    measurement_points_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                measurement_points_pkg_dir, "launch", "launch_measurement_points.launch.py"
            )
        ),
        launch_arguments={
            "respawn": LaunchConfiguration("measurement_points_respawn"),
            "respawn_delay": LaunchConfiguration("respawn_delay"),
        }.items(),
    )

    ice_touch_detection_node = Node(
        package="ice_touch_detection_pkg",
        executable="ice_touch_detection_node",
        name="ice_touch_detection_node",
        parameters=[
            # geometry
            {"tower_height_m": 0.148},
            {"tower_horizontal_offset_m": 0.650},
            {"pressure_sensor_offset_m": 0.136},
            # geometric detection
            {"touch_tolerance_m": 0.02},
            {"max_valid_angle_deg": 45.0},
            # ultrasonic validity
            {"ultrasonic_zero_window": 15},
            {"ultrasonic_zero_ratio_threshold": 0.8},
            # pressure
            {"water_density_kgm3": 1000.0},
            {"ice_thickness_m": 0.0},
            {"pressure_near_surface_pa": 3000.0},
            {"pressure_fallback_pa": 1800.0},
            # IMU collision detection
            {"use_imu_collision": True},
            {"imu_collision_window": 30},
            {"imu_collision_min_samples": 10},
            {"imu_collision_threshold_ms2": 3.5},
            # debounce
            {"confirm_count": 3},
            {"clear_count": 5},
            {"publish_rate_hz": 10.0},
            # topics
            {"ultrasonic_topic": "/top/ultrasonic/distance"},
            {"odometry_topic": "/odometry/filtered/local"},
            {"acceleration_topic": "/imu/acceleration"},
            {"pressure_topic": "/sensors/keller26x/gauge_pressure"},
            {"output_topic": "/ice_touch_detection/touching"},
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
                "measurement_points_respawn",
                default_value="true",
                description="Respawn measurement points node if it exits/crashes.",
            ),
            DeclareLaunchArgument(
                "respawn_delay",
                default_value="2.0",
                description="Seconds to wait before restarting a crashed process.",
            ),
            DeclareLaunchArgument(
                "start_ekf",
                default_value="false",
                description=(
                    "Launch the localization EKF stack (local + watchdog + global). "
                    "Default off so the IMU/pressure can settle on land with the boat aligned to "
                    "true East before the EKF starts in water. Launch the EKF separately with "
                    "`ros2 launch ekf_localization_pkg ekf_localization.launch.py`."
                ),
            ),
            localization_launch,
            pressure_launch,
            measurement_points_launch,
            ice_touch_detection_node,
        ]
    )
