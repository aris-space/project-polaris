"""
Offline replay using the local EKF + gnss_anchored_pose (no global EKF, no
navsat_transform, no gnss_datum_watchdog).

Pipeline:
  ekf_local_node    →  /odometry/filtered/local              (DVL+IMU+pressure)
  odometry_validator → /odometry/filtered/local_validated    (drops bad stamps)
  gnss_anchored_pose → /odometry/filtered/global             (local + offset)

The anchored pose node:
  - waits for the first valid /fix
  - records local-EKF position at that instant
  - publishes /odometry/filtered/global = local_pos + (datum - local_pos_at_anchor)
  - by default does NOT re-anchor on subsequent fixes

You run bag playback separately:

  ros2 launch ekf_localization_pkg offline_anchored_replay.launch.py
  ros2 bag play /path/to/bag --clock --topics ...

Set `gps_fix_topic` to whatever the bag actually contains
(`/gps/selected` is typical; `/fix` for older recordings).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.descriptions import ParameterValue


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
            "Manual yaw correction (deg, CCW about +Z) applied to Odometry+NavSatFix."
        ),
    )
    antenna_offset_arg = DeclareLaunchArgument(
        "gps_antenna_offset_xyz",
        default_value="[0.0, 0.0, 0.0]",
        description=(
            "GPS antenna body-frame [x, y, z] (m). Compensates for lever-arm "
            "translation that appears when the AUV yaws on the spot."
        ),
    )
    imu_yaw_offset_arg = DeclareLaunchArgument(
        "imu_yaw_offset_deg",
        default_value="0.0",
        description=(
            "Pre-EKF yaw correction (deg, CCW about +Z) for imu_yaw_correction. "
            "Ignored if use_imu_yaw_correction:=false."
        ),
    )
    ubx_pvt_topic_arg = DeclareLaunchArgument(
        "ubx_pvt_topic",
        default_value="/ubx_nav_pvt",
        description="UBX-NAV-PVT topic for head_mot-based yaw calibration.",
    )
    use_imu_yaw_correction_arg = DeclareLaunchArgument(
        "use_imu_yaw_correction",
        default_value="true",
        description=(
            "Spin up the imu_yaw_correction node. Set to false when replaying "
            "a bag that ALREADY contains /imu/data_corrected from the live "
            "stack (post-2026-05-07 bags) — otherwise two publishers on the "
            "same topic confuse the local EKF. When false, the bag's "
            "/imu/data_corrected is replayed directly and carries the live "
            "head_mot-based calibration."
        ),
    )

    # Pre-EKF heading correction. Started only when the bag does NOT already
    # contain /imu/data_corrected (pre-2026-05-07 bags). For live-stack-
    # recorded bags, the recorded /imu/data_corrected already carries the
    # in-mission head_mot calibration; running this node would create a
    # conflicting second publisher on the same topic.
    imu_yaw_correction_node = Node(
        package="ekf_localization_pkg",
        executable="imu_yaw_correction",
        name="imu_yaw_correction",
        output="screen",
        parameters=[{
            # Force float — passing "imu_yaw_offset_deg:=-140" on the command
            # line arrives as int and rclpy rejects it (yaw_offset_deg is
            # declared DOUBLE inside the node).
            "yaw_offset_deg": ParameterValue(
                LaunchConfiguration("imu_yaw_offset_deg"),
                value_type=float,
            ),
            "input_topic": "/imu/data",
            "output_topic": "/imu/data_corrected",
            "gps_topic": LaunchConfiguration("gps_fix_topic"),
            "ubx_pvt_topic": LaunchConfiguration("ubx_pvt_topic"),
        }],
        condition=IfCondition(LaunchConfiguration("use_imu_yaw_correction")),
    )

    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"imu0": "/imu/data_corrected"},
        ],
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

    anchored_pose_node = Node(
        package="ekf_localization_pkg",
        executable="gnss_anchored_pose",
        name="gnss_anchored_pose",
        output="screen",
        parameters=[{
            "local_odom_topic": "/odometry/filtered/local_validated",
            "gps_topic": LaunchConfiguration("gps_fix_topic"),
            "global_odom_topic": "/odometry/filtered/global",
            "h_acc_topic": LaunchConfiguration("h_acc_topic"),
            "h_acc_max_m": ParameterValue(
                LaunchConfiguration("h_acc_max_m"), value_type=float),
            "reanchor_on_each_fix": ParameterValue(
                LaunchConfiguration("reanchor_on_each_fix"), value_type=bool),
            "yaw_offset_deg": ParameterValue(
                LaunchConfiguration("yaw_offset_deg"), value_type=float),
            "gps_antenna_offset_xyz": LaunchConfiguration("gps_antenna_offset_xyz"),
            "publish_tf": True,
            "map_frame": "map",
            "odom_frame": "odom",
        }],
    )

    diag_output_dir_arg = DeclareLaunchArgument(
        "diag_output_dir",
        default_value="/tmp/ekf_diag",
        description=(
            "Directory for ekf_offline_diagnostic CSV. Use a per-run subdir "
            "(e.g. diagnosis/global_ekf_residual/anchored_rate_2.0/) so "
            "compare_diag_runs.py can plot multiple runs side by side."
        ),
    )

    # gnss_anchored_pose does not produce /odometry/gps; the node leaves the
    # corresponding columns empty. SBL comparison via /waterlinked_ugps/navsatfix
    # still works, and the parity test against the live recording's
    # /gps/filtered/global drives the output of this run.
    #
    # Datum source: gnss_anchored_pose publishes /gps/filtered/global as
    # NavSatFix. Its first message lat/lon equals the anchor datum by
    # construction (map-frame origin = datum). Using this as the diagnostic
    # datum guarantees that SBL is projected with the SAME datum the
    # algorithm anchored on — without this, the diagnostic locks at the
    # first /gps/selected (no h_acc gate) while gnss_anchored_pose waits
    # for h_acc<=0.5m, the AUV moves between those two moments, and the
    # whole SBL track is offset by that displacement (~9 m on rect_01).
    diagnostic_node = Node(
        package="ekf_localization_pkg",
        executable="ekf_offline_diagnostic",
        name="ekf_offline_diagnostic",
        output="screen",
        parameters=[{
            "output_dir": LaunchConfiguration("diag_output_dir"),
            "global_odom_topic": "/odometry/filtered/global",
            "local_odom_topic": "/odometry/filtered/local_validated",
            "gps_odom_topic": "/odometry/gps_unused",
            "sbl_topic": "/waterlinked_ugps/navsatfix",
            "datum_navsatfix_topic": "/gps/filtered/global",
            "datum_topic_fallback": LaunchConfiguration("gps_fix_topic"),
        }],
    )

    return LaunchDescription([
        SetUseSimTime(True),
        params_file_arg,
        gps_fix_topic_arg,
        h_acc_topic_arg,
        h_acc_max_arg,
        reanchor_arg,
        yaw_offset_arg,
        antenna_offset_arg,
        imu_yaw_offset_arg,
        ubx_pvt_topic_arg,
        use_imu_yaw_correction_arg,
        diag_output_dir_arg,
        imu_yaw_correction_node,
        ekf_local_node,
        odometry_validator_node,
        anchored_pose_node,
        diagnostic_node,
    ])
