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
from launch_ros.actions import SetUseSimTime


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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([str(ekf_launch)]),
                launch_arguments={
                    "use_navsat_transform": LaunchConfiguration("use_navsat_transform"),
                    "use_global_ekf": LaunchConfiguration("use_global_ekf"),
                    "gps_fix_topic": LaunchConfiguration("gps_fix_topic"),
                    "h_acc_topic": LaunchConfiguration("h_acc_topic"),
                    "imu_topic": LaunchConfiguration("imu_topic"),
                    "odom_topic": LaunchConfiguration("odom_topic"),
                }.items(),
            ),
        ]
    )
