"""
Live (online) version of the anchored localization pipeline.

Runs:
  ekf_local_node     →  /odometry/filtered/local              (DVL + IMU + pressure)
  odometry_validator →  /odometry/filtered/local_validated    (drops bad stamps)
  gnss_anchored_pose →  /odometry/filtered/global             (local + offset)

This replaces the global EKF + navsat_transform + gnss_datum_watchdog stack.
For offline bag replay, use offline_anchored_replay.launch.py instead — it
sets use_sim_time=true and otherwise has the same content.

Map frame is anchored to the first /fix that passes the h_acc gate. After that,
position is local-EKF dead-reckoning + a fixed translation. No Kalman fusion of
GPS thereafter (set reanchor_on_each_fix:=true to update on each fix instead).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("ekf_localization_pkg")
    default_local_params = Path(pkg_dir, "config", "ekf_local.yaml")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=str(default_local_params),
        description="Path to local EKF (ekf_local_node) parameters YAML.",
    )
    gps_fix_topic_arg = DeclareLaunchArgument(
        "gps_fix_topic",
        default_value="/gps/selected",
        description="NavSatFix topic that gnss_anchored_pose subscribes to.",
    )
    h_acc_topic_arg = DeclareLaunchArgument(
        "h_acc_topic",
        default_value="/ubx_nav_hp_pos_llh",
        description=(
            "UBX-NAV-HPPOSLLH topic for the anchor h_acc gate. Empty string "
            "disables the gate (anchor on first non-null-island status>=0 fix)."
        ),
    )
    h_acc_max_arg = DeclareLaunchArgument(
        "h_acc_max_m",
        default_value="0.5",
        description="Max horizontal accuracy (m) for the anchor fix.",
    )
    reanchor_arg = DeclareLaunchArgument(
        "reanchor_on_each_fix",
        default_value="false",
        description=(
            "If true, gnss_anchored_pose updates the offset on every valid GPS "
            "fix (causes position discontinuities). Default false matches the "
            "'set origin once on first fix, dead-reckon afterwards' intent."
        ),
    )
    yaw_offset_arg = DeclareLaunchArgument(
        "yaw_offset_deg",
        default_value="0.0",
        description=(
            "Manual yaw correction (degrees, CCW about +Z) applied to the "
            "Odometry+NavSatFix output to cancel a constant IMU heading bias. "
            "Positive rotates the dead-reckoning track CCW. TF tree is not "
            "rotated."
        ),
    )
    antenna_offset_arg = DeclareLaunchArgument(
        "gps_antenna_offset_xyz",
        default_value="[0.0, 0.0, 0.0]",
        description=(
            "GPS antenna body-frame [x, y, z] in metres relative to the AUV "
            "reference frame the local EKF tracks. When non-zero, the node "
            "compensates for the antenna arcing around the AUV centre during "
            "yaw rotations so /gps/filtered/global stays aligned with raw /fix."
        ),
    )
    imu_yaw_offset_arg = DeclareLaunchArgument(
        "imu_yaw_offset_deg",
        default_value="0.0",
        description=(
            "Pre-EKF yaw correction (deg, CCW about +Z) applied by "
            "imu_yaw_correction to /imu/data before the local EKF sees it. "
            "Trigger /imu_yaw_correction/calibrate_yaw_offset to set this "
            "automatically from GNSS heading after a forward-driving maneuver."
        ),
    )
    ubx_pvt_topic_arg = DeclareLaunchArgument(
        "ubx_pvt_topic",
        default_value="/ubx_nav_pvt",
        description=(
            "UBX-NAV-PVT topic for head_mot-based yaw calibration. Empty "
            "string disables the head_mot path; only two-fix bearing fallback."
        ),
    )
    use_thruster_fallback_arg = DeclareLaunchArgument(
        "use_thruster_fallback",
        default_value="true",
        description=(
            "Launch thruster_velocity_estimator. Activates only when DVL has "
            "been absent for dvl_timeout_s (default 3 s)."
        ),
    )

    # Pre-EKF: rotate /imu/data by yaw_offset_deg, republish on /imu/data_corrected.
    # The local EKF then sees the calibrated heading directly, so /odometry/filtered/local
    # and the local TF tree are correct without any post-EKF fix-up.
    imu_yaw_correction_node = Node(
        package="ekf_localization_pkg",
        executable="imu_yaw_correction",
        name="imu_yaw_correction",
        output="screen",
        parameters=[{
            "yaw_offset_deg": LaunchConfiguration("imu_yaw_offset_deg"),
            "input_topic": "/imu/data",
            "output_topic": "/imu/data_corrected",
            # Raw /fix, not the gated /gps/selected: yaw calibration needs
            # GNSS available before the selector's IMU yaw stability gate
            # latches (the gate's whole purpose is to protect the EKF map
            # datum from a moving platform — opposite of this use case).
            "gps_topic": "/fix",
            "ubx_pvt_topic": LaunchConfiguration("ubx_pvt_topic"),
        }],
    )

    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
        remappings=[("odometry/filtered", "/odometry/filtered/local")],
    )

    odometry_validator_node = Node(
        package="ekf_localization_pkg",
        executable="odometry_validator",
        name="odometry_validator",
        output="screen",
        parameters=[{
            "input_topic": "/odometry/filtered/local",
            "output_topic": "/odometry/filtered/local_validated",
            "max_forward_jump_s": 60.0,
            "max_backward_jump_s": 1.0,
        }],
    )

    # Thruster fallback: estimates surge velocity from PWM when DVL is absent.
    # Silent during normal DVL operation; activates after dvl_timeout_s (default 3 s).
    thruster_fallback_node = Node(
        package="ekf_localization_pkg",
        executable="thruster_velocity_estimator",
        name="thruster_velocity_estimator",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_thruster_fallback")),
    )

    anchored_pose_node = Node(
        package="ekf_localization_pkg",
        executable="gnss_anchored_pose",
        name="gnss_anchored_pose",
        output="screen",
        parameters=[{
            "local_odom_topic": "/odometry/filtered/local_validated",
            # Raw /fix, not the gated /gps/selected: the calibrate_yaw_offset
            # service needs GNSS before the selector's IMU yaw stability gate
            # opens (same reason imu_yaw_correction uses /fix directly).
            # Anchor quality is still protected by the h_acc gate in _try_anchor.
            "gps_topic": "/fix",
            "global_odom_topic": "/odometry/filtered/global",
            "h_acc_topic": LaunchConfiguration("h_acc_topic"),
            "h_acc_max_m": LaunchConfiguration("h_acc_max_m"),
            "reanchor_on_each_fix": LaunchConfiguration("reanchor_on_each_fix"),
            "yaw_offset_deg": LaunchConfiguration("yaw_offset_deg"),
            "gps_antenna_offset_xyz": LaunchConfiguration("gps_antenna_offset_xyz"),
            "publish_tf": True,
            "map_frame": "map",
            "odom_frame": "odom",
        }],
    )

    return LaunchDescription([
        params_file_arg,
        gps_fix_topic_arg,
        h_acc_topic_arg,
        h_acc_max_arg,
        reanchor_arg,
        yaw_offset_arg,
        antenna_offset_arg,
        imu_yaw_offset_arg,
        ubx_pvt_topic_arg,
        use_thruster_fallback_arg,
        imu_yaw_correction_node,
        ekf_local_node,
        odometry_validator_node,
        thruster_fallback_node,
        anchored_pose_node,
    ])
