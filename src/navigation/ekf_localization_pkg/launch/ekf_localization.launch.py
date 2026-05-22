from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition  # noqa: F401
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("ekf_localization_pkg")
    default_params = Path(pkg_dir, "config", "ekf_local.yaml")
    default_global_params = Path(pkg_dir, "config", "ekf_global.yaml")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=str(default_params),
        description="Path to local EKF (ekf_local_node) parameters YAML.",
    )
    use_gnss_datum_watchdog_arg = DeclareLaunchArgument(
        "use_gnss_datum_watchdog",
        default_value="true",
        description=(
            "Enable gnss_datum_watchdog. When a quality GNSS fix arrives the watchdog "
            "captures the datum and activates the dormant in-place global-EKF stack."
        ),
    )
    use_global_ekf_arg = DeclareLaunchArgument(
        "use_global_ekf",
        default_value="true",
        description="Whether the watchdog should also launch ekf_global_node.",
    )
    global_params_file_arg = DeclareLaunchArgument(
        "global_params_file",
        default_value=str(default_global_params),
        description="Global EKF params forwarded to the watchdog.",
    )
    gps_fix_topic_arg = DeclareLaunchArgument(
        "gps_fix_topic",
        default_value="/fix",
        description="GNSS NavSatFix topic for the watchdog and navsat_transform.",
    )
    h_acc_topic_arg = DeclareLaunchArgument(
        "h_acc_topic",
        default_value="/ubx_nav_hp_pos_llh",
        description=(
            "UBX-NAV-HPPOSLLH topic for the h_acc quality gate. "
            "Pass an empty string to disable the gate."
        ),
    )
    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value="/imu/data_corrected",
        description=(
            "IMU topic forwarded to navsat_transform. Defaults to the head_mot-"
            "calibrated stream so navsat's yaw reference matches what the "
            "local and global EKFs are running on."
        ),
    )
    imu0_topic_arg = DeclareLaunchArgument(
        "imu0_topic",
        default_value="/imu/data_corrected",
        description=(
            "IMU topic the local EKF actually fuses. Defaults to "
            "/imu/data_corrected (head_mot-calibrated) so the entire stack "
            "(local EKF, navsat_transform, ekf_global_node) runs off a "
            "single consistent heading frame. Sets ekf_local_node's imu0 "
            "parameter directly, overriding the value in ekf_local.yaml."
        ),
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odometry/filtered/local_validated",
        description=(
            "Local EKF odometry forwarded to navsat_transform and the global EKF. "
            "Defaults to the *_validated topic so consumers receive the validated "
            "stream from odometry_validator (filters out stamp anomalies during "
            "offline replay)."
        ),
    )
    use_thruster_fallback_arg = DeclareLaunchArgument(
        "use_thruster_fallback",
        default_value="true",
        description=(
            "Launch thruster_velocity_estimator. Publishes body-frame velocity from "
            "surge PWM when DVL lock is absent, giving the local EKF a velocity "
            "anchor instead of IMU-only dead-reckoning."
        ),
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description=(
            "Wire use_sim_time into every node in this included launch. "
            "MUST be set to true for offline-bag replay; otherwise the watchdog "
            "and friends timestamp outgoing messages with wall clock, which "
            "(a) makes set_pose's header.stamp ~days into the bag's future and "
            "(b) cascades that bug to the watchdog's spawned navsat_transform + "
            "ekf_global subprocess. SetUseSimTime in the parent launch does "
            "NOT propagate into IncludeLaunchDescription'd children — has to "
            "be passed explicitly."
        ),
    )

    # Local EKF: fuses IMU + DVL + pressure. Starts immediately, no GPS needed.
    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"imu0": LaunchConfiguration("imu0_topic"),
             "use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        # Remap set_pose to the node-name-prefixed topic. Without this,
        # robot_localization's relative `set_pose` resolves to root /set_pose,
        # so any /set_pose publication would reset BOTH the local and global
        # EKFs (corrupting whichever wasn't intended). The watchdog only
        # targets the global EKF; this remap ensures /set_pose stays inert
        # for the local one.
        remappings=[
            ("odometry/filtered", "/odometry/filtered/local"),
            ("set_pose", "/ekf_local_node/set_pose"),
        ],
    )

    # Validator: forwards /odometry/filtered/local → /odometry/filtered/local_validated
    # while dropping any message whose header.stamp jumps forward by more than
    # 60 s relative to the last accepted message. Workaround for a robot_localization
    # behaviour observed during offline replay where, when /clock is briefly
    # unavailable, the EKF publishes a single message with a wall-clock stamp
    # instead of sim-time, which produces a multi-day dt downstream and blows
    # up the global EKF's predict step.
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
            "use_sim_time": LaunchConfiguration("use_sim_time"),
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

    # Watchdog: waits for a quality GNSS fix, then activates the dormant
    # global-EKF stack by pushing the datum and local-EKF anchor to the
    # in-place nodes via SetParameters.
    datum_watchdog_node = Node(
        package="ekf_localization_pkg",
        executable="gnss_datum_watchdog",
        name="gnss_datum_watchdog",
        output="screen",
        parameters=[{
            "fix_topic": LaunchConfiguration("gps_fix_topic"),
            "h_acc_topic": LaunchConfiguration("h_acc_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "use_global_ekf": LaunchConfiguration("use_global_ekf"),
            # CRITICAL: the watchdog uses self.get_clock() to stamp the
            # bootstrap set_pose message — if use_sim_time is False under
            # offline replay, that stamp is wall-clock-now, which can be
            # days into the bag's future. robot_localization then rejects
            # every subsequent measurement as "preceded the most recent
            # pose reset" and freezes. Empirically observed on grid_02:
            # bootstrap stamped May 9 22:00 (wall) while bag was May 7
            # 12:25 — 215 000 s gap, all bag measurements rejected.
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        condition=IfCondition(LaunchConfiguration("use_gnss_datum_watchdog")),
    )

    # ── IN-PLACE global-stack nodes ──────────────────────────────────────
    # Start in dormant mode at T=0 (no datum yet). The watchdog calls
    # set_parameters on each of them when GNSS lock fires, flipping them
    # to active mode. ~6 s warmup (single DDS discovery at launch).
    static_map_odom_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_odom_bootstrap",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("use_global_ekf")),
    )

    # GPS velocity-consistency filter: drops fixes that imply impossible
    # boat velocities (typical RTK-glitch failure mode where a few samples
    # report kilometres of jump while h_acc still claims fix-quality). Sits
    # between the watchdog and gps_to_map_position. Without this, tight
    # cov_floor settings on the EKF let glitched fixes pull state several
    # metres before the next good fix snaps it back — visible as
    # "trajectory jumps to middle and back" in the overlay plot.
    gps_velocity_filter_node = Node(
        package="ekf_localization_pkg",
        executable="gps_velocity_filter",
        name="gps_velocity_filter",
        output="screen",
        parameters=[{
            "input_topic": "/gps/validated",
            "output_topic": "/gps/validated_filtered",
            "max_velocity_m_s": 2.0,
            "max_time_gap_s": 5.0,
            "min_dt_s": 0.01,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        condition=IfCondition(LaunchConfiguration("use_global_ekf")),
    )

    gps_to_map_position_node = Node(
        package="ekf_localization_pkg",
        executable="gps_to_map_position",
        name="gps_to_map_position",
        output="screen",
        parameters=[{
            "input_topic": "/gps/validated_filtered",
            "output_topic": "/odometry/gps_map",
            "output_frame_id": "map",
            # null-island default — watchdog overrides after lock.
            "datum_lat": 0.0,
            "datum_lon": 0.0,
            "datum_alt": 0.0,
            # 0.10 m² ≡ ~32 cm 1σ. Geometric midpoint between 0.04
            # (aggressive tracking, but exposes RTK glitches as visible
            # 1.5 m "jump to middle" artifacts) and 0.25 (clean
            # trajectory, but p95 vs /fix ~70 cm). With 0.10, K_steady
            # is ~0.5 — state still snaps reasonably to GPS but doesn't
            # chase short-lived outliers as hard. RTK glitches are still
            # gated upstream by the watchdog's h_acc check (≤ 0.5 m),
            # the velocity filter (≤ 3 m/s apparent), and additionally
            # rejected by the EKF's odom1_pose_rejection_threshold: 5σ.
            "min_pos_cov_m2": 0.10,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        condition=IfCondition(LaunchConfiguration("use_global_ekf")),
    )

    pressure_pose_frame_fix_node = Node(
        package="ekf_localization_pkg",
        executable="pressure_pose_frame_fix",
        name="pressure_pose_frame_fix",
        output="screen",
        parameters=[{
            "input_topic": "/sensors/pressure/pose_enu",
            "output_topic": "/sensors/pressure/pose_enu_map",
            "output_frame_id": "map",
            "local_anchor_z": 0.0,  # watchdog overrides on lock
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        condition=IfCondition(LaunchConfiguration("use_global_ekf")),
    )

    ekf_global_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global_node",
        output="screen",
        parameters=[
            LaunchConfiguration("global_params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("odometry/filtered", "/odometry/filtered/global"),
            ("set_pose", "/ekf_global_node/set_pose"),
        ],
        condition=IfCondition(LaunchConfiguration("use_global_ekf")),
    )

    global_ekf_to_navsatfix_node = Node(
        package="ekf_localization_pkg",
        executable="global_ekf_to_navsatfix",
        name="global_ekf_to_navsatfix_node",
        output="screen",
        parameters=[{
            "datum_lat": 0.0,  # watchdog overrides on lock
            "datum_lon": 0.0,
            "datum_alt": 0.0,
            "map_yaw_offset_rad": 0.0,
            "global_odom_topic": "/odometry/filtered/global",
            "output_topic": "/gps/filtered/global",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
        condition=IfCondition(LaunchConfiguration("use_global_ekf")),
    )

    # EKF stack health diagnostics — publishes on /diagnostics_software so the
    # operator (Foxglove diagnostic panel) can see whether the local/global
    # EKFs, map→odom TF, GPS pipeline, and divergence-vs-GPS are healthy.
    # Pattern matches src/autonomy/.../nav2_lifecycle_diagnostics.py
    # (parent + per-subsystem children).
    ekf_diagnostics_node = Node(
        package="ekf_localization_pkg",
        executable="ekf_diagnostics",
        name="ekf_diagnostics",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription(
        [
            params_file_arg,
            use_gnss_datum_watchdog_arg,
            use_global_ekf_arg,
            global_params_file_arg,
            gps_fix_topic_arg,
            h_acc_topic_arg,
            imu_topic_arg,
            imu0_topic_arg,
            odom_topic_arg,
            use_thruster_fallback_arg,
            use_sim_time_arg,
            ekf_local_node,
            odometry_validator_node,
            thruster_fallback_node,
            datum_watchdog_node,
            # Global stack — start dormant; watchdog activates on lock.
            static_map_odom_node,
            gps_velocity_filter_node,
            gps_to_map_position_node,
            pressure_pose_frame_fix_node,
            ekf_global_node,
            global_ekf_to_navsatfix_node,
            ekf_diagnostics_node,
        ]
    )
