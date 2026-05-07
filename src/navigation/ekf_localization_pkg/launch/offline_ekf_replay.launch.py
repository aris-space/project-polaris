"""
Offline EKF + navsat + global EKF against a **played** rosbag (MCAP).

Use this when mission bags have sensor streams but no (or wrong) on-robot filter
recording — e.g. `use_navsat_transform:=false` on the vehicle.

**You run bag playback separately** (so you can pick `--topics` / exclusions):

1. Terminal A — filter stack (this launch):
   ros2 launch ekf_localization_pkg offline_ekf_replay.launch.py

2. Terminal B — time source + sensor data:
   ros2 bag play /path/to/bag --clock

**Clock:** `--clock` publishes `/clock`; this launch sets `use_sim_time` so EKF and
navsat advance in lockstep with the bag.

**Bags that already contain `/tf` and `/odometry/filtered/*` from an old run:**
Replay only *inputs* so you do not get duplicate publishers, e.g.:

  ros2 bag play /path/to/bag --clock --topics \\
    /imu/data /sensors/dvl/odometry_cov /sensors/pressure/pose_enu \\
    /tf_static /gps/selected /waterlinked_ugps/navsatfix ...

(Adjust the list to match what that bag actually has; always include `/tf_static`
if the bag has it.)

**NavSatFix source:** Default `gps_fix_topic` is `/gps/selected` (selector output).
Override to `/fix` if your replay graph provides only that.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetUseSimTime


def generate_launch_description():
    ekf_localization_pkg_dir = get_package_share_directory("ekf_localization_pkg")
    ekf_launch = Path(ekf_localization_pkg_dir, "launch", "ekf_localization.launch.py")

    return LaunchDescription(
        [
            SetUseSimTime(True),
            DeclareLaunchArgument(
                "use_navsat_transform",
                default_value="true",
                description="Enable gnss_datum_watchdog (which spawns navsat_transform + global EKF).",
            ),
            DeclareLaunchArgument(
                "use_global_ekf",
                default_value="true",
                description="Whether the watchdog should also launch ekf_global_node.",
            ),
            DeclareLaunchArgument(
                "gps_fix_topic",
                default_value="/gps/selected",
                description="NavSatFix topic for gnss_datum_watchdog.",
            ),
            DeclareLaunchArgument(
                "h_acc_topic",
                default_value="",
                description=(
                    "UBX-NAV-HPPOSLLH topic for the h_acc quality gate. "
                    "Empty (default) disables the gate for offline replay — "
                    "GPS status + coordinate check is sufficient for bags."
                ),
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/imu/data",
                description="IMU topic forwarded to navsat_transform.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odometry/filtered/local",
                description="Local odometry topic forwarded to navsat_transform.",
            ),
            DeclareLaunchArgument(
                "diag_output_dir",
                default_value="/tmp/ekf_diag",
                description=(
                    "Directory for ekf_offline_diagnostic CSV. Use a per-run "
                    "subdir (e.g. diagnosis/global_ekf_residual/rate_2.0/) so "
                    "compare_diag_runs.py can plot multiple runs side by side."
                ),
            ),
            DeclareLaunchArgument(
                "imu_yaw_offset_deg",
                default_value="0.0",
                description=(
                    "Pre-EKF yaw correction (deg, CCW about +Z) applied by "
                    "imu_yaw_correction before the local EKF consumes IMU. "
                    "Use a non-zero value to compensate the boat's IMU heading "
                    "bias on a recorded bag (e.g. -140 for "
                    "zermatt_rectangle_01_2026_04_29). 0 = passthrough."
                ),
            ),
            DeclareLaunchArgument(
                "ubx_pvt_topic",
                default_value="/ubx_nav_pvt",
                description="UBX-NAV-PVT topic for head_mot-based yaw calibration.",
            ),
            # Pre-EKF heading correction. Subscribes to /imu/data (raw, from
            # bag), applies the constant yaw_offset_deg, republishes on
            # /imu/data_corrected. The local EKF (via imu0_topic override
            # below) consumes the corrected stream so the entire global-EKF
            # stack — local EKF, navsat_transform with use_odometry_yaw,
            # and ekf_global_node — sees a heading aligned with actual ENU.
            Node(
                package="ekf_localization_pkg",
                executable="imu_yaw_correction",
                name="imu_yaw_correction",
                output="screen",
                parameters=[{
                    "yaw_offset_deg": LaunchConfiguration("imu_yaw_offset_deg"),
                    "input_topic": "/imu/data",
                    "output_topic": "/imu/data_corrected",
                    "gps_topic": LaunchConfiguration("gps_fix_topic"),
                    "ubx_pvt_topic": LaunchConfiguration("ubx_pvt_topic"),
                }],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([str(ekf_launch)]),
                launch_arguments={
                    "use_navsat_transform": LaunchConfiguration("use_navsat_transform"),
                    "use_global_ekf": LaunchConfiguration("use_global_ekf"),
                    "gps_fix_topic": LaunchConfiguration("gps_fix_topic"),
                    "h_acc_topic": LaunchConfiguration("h_acc_topic"),
                    # navsat_transform_node still wants raw /imu/data — its
                    # use_odometry_yaw=true mode pulls yaw from the local
                    # EKF's filtered odometry, not directly from the IMU
                    # topic, so the corrected stream isn't needed here.
                    "imu_topic": LaunchConfiguration("imu_topic"),
                    # Local EKF MUST see the corrected stream so the rest of
                    # the stack inherits the calibrated heading.
                    "imu0_topic": "/imu/data_corrected",
                    "odom_topic": LaunchConfiguration("odom_topic"),
                }.items(),
            ),
            Node(
                package="ekf_localization_pkg",
                executable="ekf_offline_diagnostic",
                name="ekf_offline_diagnostic",
                output="screen",
                parameters=[{
                    "output_dir": LaunchConfiguration("diag_output_dir"),
                    "global_odom_topic": "/odometry/filtered/global",
                    "local_odom_topic": "/odometry/filtered/local_validated",
                    "gps_odom_topic": "/odometry/gps",
                    "sbl_topic": "/waterlinked_ugps/navsatfix",
                    # Use the global track's own NavSatFix as the diagnostic
                    # datum so SBL is projected with the same lat/lon the
                    # algorithm anchored on. Falls back to /gps/validated if
                    # no NavSatFix has appeared within ~5 s.
                    "datum_navsatfix_topic": "/gps/filtered",
                    "datum_topic_fallback": "/gps/validated",
                }],
            ),
        ]
    )
