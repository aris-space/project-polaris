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

**IMU yaw correction:** Pre-2026-05-07 bags do not contain `/imu/data_corrected`,
so the local EKF needs `imu_yaw_correction` to apply a manually-tuned
`imu_yaw_offset_deg` and republish the calibrated stream.

Bags recorded on or after 2026-05-07 (after `imu_yaw_correction` was promoted
to the live bringup) DO contain `/imu/data_corrected` from the live system,
already calibrated by the on-board service-triggered head_mot calibration.
For those bags, set `use_imu_yaw_correction:=false` so the offline launch does
NOT spin up its own `imu_yaw_correction` node — otherwise you have TWO
publishers on `/imu/data_corrected` (the bag and the offline node), the local
EKF sees a mix, and the heading prior is wrong.

The `use_imu_yaw_correction` arg defaults to `true` for backwards compatibility
with old bags. Pass `use_imu_yaw_correction:=false` for any post-2026-05-07
live-stack-recorded bag.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.descriptions import ParameterValue


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
                default_value="/imu/data_corrected",
                description=(
                    "IMU topic forwarded to navsat_transform. Calibrated "
                    "stream so navsat's yaw reference matches the rest of "
                    "the stack."
                ),
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
                "record_outputs",
                default_value="false",
                description=(
                    "If true, also start a ros2 bag record process inside "
                    "this launch (so it inherits the launch's DDS context — "
                    "spawning ros2 bag record from a separate bash session "
                    "under WSL2 / docker desktop fails to receive messages "
                    "despite topic discovery working). Output goes to "
                    "<diag_output_dir>/replay_outputs/."
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
                    "zermatt_rectangle_01_2026_04_29). 0 = passthrough. "
                    "Ignored if use_imu_yaw_correction:=false."
                ),
            ),
            DeclareLaunchArgument(
                "ubx_pvt_topic",
                default_value="/ubx_nav_pvt",
                description="UBX-NAV-PVT topic for head_mot-based yaw calibration.",
            ),
            DeclareLaunchArgument(
                "use_imu_yaw_correction",
                default_value="true",
                description=(
                    "Spin up the imu_yaw_correction node. Set to false when "
                    "replaying a bag that ALREADY contains /imu/data_corrected "
                    "from the live stack (post-2026-05-07 bags) — otherwise "
                    "two publishers on the same topic confuse the local EKF. "
                    "When false, the bag's /imu/data_corrected is replayed "
                    "directly and carries the live head_mot calibration."
                ),
            ),
            # ── Anchored shadow ─────────────────────────────────────────────
            # When true, also runs gnss_anchored_pose alongside the global EKF
            # stack. Both consume the SHARED /odometry/filtered/local_validated
            # (one local EKF instance is enough). The anchored node's outputs
            # are renamed so they don't clash with the global EKF stack's:
            #   /odometry/filtered/global         ← ekf_global_node
            #   /odometry/filtered/global_anchored ← gnss_anchored_pose
            #   /gps/filtered/global              ← global_ekf_to_navsatfix
            #   /gps/filtered/global_anchored     ← gnss_anchored_pose
            # The anchored node has publish_tf=false in this mode so it does
            # not fight ekf_global_node over the map→odom transform.
            DeclareLaunchArgument(
                "use_anchored_shadow",
                default_value="false",
                description=(
                    "Run gnss_anchored_pose in parallel with the global EKF "
                    "stack, publishing to /odometry/filtered/global_anchored "
                    "and /gps/filtered/global_anchored. Useful for direct "
                    "comparison of the global EKF's output against the "
                    "anchored DR baseline that the live system was validated "
                    "with. publish_tf is forced false on the anchored node "
                    "to avoid map→odom TF conflicts with ekf_global_node."
                ),
            ),
            DeclareLaunchArgument(
                "anchored_h_acc_max_m",
                default_value="0.5",
                description=(
                    "h_acc gate (m) for the anchored shadow's anchor lock. "
                    "Tighten to 0.05 to require RTK Fixed for the anchor."
                ),
            ),
            DeclareLaunchArgument(
                "anchored_h_acc_topic",
                default_value="/ubx_nav_hp_pos_llh",
                description=(
                    "UBX-NAV-HPPOSLLH topic for the anchored shadow's h_acc "
                    "gate. Empty string disables the gate."
                ),
            ),
            DeclareLaunchArgument(
                "anchored_yaw_offset_deg",
                default_value="0.0",
                description=(
                    "Manual yaw correction (deg, CCW about +Z) applied by "
                    "gnss_anchored_pose to its output Odometry+NavSatFix. "
                    "Defaults to 0 because /imu/data_corrected from the bag "
                    "already carries the live head_mot calibration."
                ),
            ),
            DeclareLaunchArgument(
                "anchored_gps_antenna_offset_xyz",
                default_value="[0.0, 0.0, 0.0]",
                description=(
                    "GPS antenna body-frame [x, y, z] (m) for the anchored "
                    "shadow's lever-arm compensation."
                ),
            ),
            # Pre-EKF heading correction. Subscribes to /imu/data (raw, from
            # bag), applies the constant yaw_offset_deg, republishes on
            # /imu/data_corrected. The local EKF (via imu0_topic override
            # below) consumes the corrected stream so the entire global-EKF
            # stack — local EKF, navsat_transform with use_odometry_yaw,
            # and ekf_global_node — sees a heading aligned with actual ENU.
            #
            # Only started when use_imu_yaw_correction:=true. For bags that
            # already contain /imu/data_corrected from the live system,
            # disable this so we don't fight the bag's own publisher.
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
                condition=IfCondition(LaunchConfiguration("use_imu_yaw_correction")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([str(ekf_launch)]),
                launch_arguments={
                    "use_navsat_transform": LaunchConfiguration("use_navsat_transform"),
                    "use_global_ekf": LaunchConfiguration("use_global_ekf"),
                    "gps_fix_topic": LaunchConfiguration("gps_fix_topic"),
                    "h_acc_topic": LaunchConfiguration("h_acc_topic"),
                    # Both navsat_transform and the local EKF run off the
                    # head_mot-calibrated /imu/data_corrected so the entire
                    # stack lives in a single rotational frame. (Earlier
                    # comment claimed navsat could keep raw /imu/data because
                    # use_odometry_yaw=true pulls yaw from filtered odom,
                    # but inconsistent IMU sources between local EKF and
                    # navsat were still showing up as a slow GPS-vs-state
                    # rotational mismatch — so we align them explicitly.)
                    "imu_topic": LaunchConfiguration("imu_topic"),
                    "imu0_topic": "/imu/data_corrected",
                    "odom_topic": LaunchConfiguration("odom_topic"),
                    # SetUseSimTime(True) at the top of THIS launch does NOT
                    # propagate into included launches. Pass it explicitly so
                    # the watchdog (and friends) timestamp set_pose / TF
                    # broadcasts in bag time, not wall time. Wall-time stamps
                    # on set_pose make robot_localization reject every
                    # subsequent bag-time-stamped measurement as "preceded
                    # the most recent pose reset."
                    "use_sim_time": "true",
                }.items(),
            ),
            # Independent rate probe — minimal callback (just counts), so
            # we can tell whether the EKF's true publish rate matches what
            # the diag CSV captures. If probe says 10 Hz and diag says
            # 2 Hz, the diag is the bottleneck. If both say 2 Hz, the
            # EKF is actually publishing slowly.
            Node(
                package="ekf_localization_pkg",
                executable="topic_rate_probe",
                name="ekf_global_rate_probe",
                output="screen",
                parameters=[{
                    "topic": "/odometry/filtered/global",
                    "log_period_s": 5.0,
                }],
            ),
            # TF rate probe — autonomy mostly consumes map->odom via TF
            # lookups, not via /odometry/filtered/global topic. If TF is
            # at the configured EKF frequency while the topic is gated at
            # GPS rate, autonomy gets full rate without further work.
            Node(
                package="ekf_localization_pkg",
                executable="tf_rate_probe",
                name="map_odom_tf_rate_probe",
                output="screen",
                parameters=[{
                    "parent_frame_id": "map",
                    "child_frame_id": "odom",
                    "log_period_s": 5.0,
                }],
            ),
            # Local EKF rate probe — what's the actual local-publish rate?
            # Should be near 30 Hz sim-time after the sensor_timeout +
            # predict_to_current_time tweak in ekf_local.yaml. If it stays
            # at ~13 Hz, the throttle is CPU/executor and we need to
            # throttle IMU input or use a multi-threaded executor.
            Node(
                package="ekf_localization_pkg",
                executable="topic_rate_probe",
                name="ekf_local_rate_probe",
                output="screen",
                parameters=[{
                    "topic": "/odometry/filtered/local",
                    "log_period_s": 5.0,
                }],
            ),
            Node(
                package="ekf_localization_pkg",
                executable="tf_rate_probe",
                name="odom_base_link_tf_rate_probe",
                output="screen",
                parameters=[{
                    "parent_frame_id": "odom",
                    "child_frame_id": "base_link",
                    "log_period_s": 5.0,
                }],
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
                    # /odometry/gps_floored carries the post-anchor-subtraction,
                    # map-frame, cov-floored GPS — i.e. exactly what the global
                    # EKF processes. Innovation columns in the diag CSV reflect
                    # the EKF's actual innovations rather than raw navsat
                    # output (which is in odom frame).
                    "gps_odom_topic": "/odometry/gps_floored",
                    "sbl_topic": "/waterlinked_ugps/navsatfix",
                    # Use the global track's own NavSatFix as the diagnostic
                    # datum so SBL is projected with the same lat/lon the
                    # algorithm anchored on. Falls back to /gps/validated if
                    # no NavSatFix has appeared within ~5 s.
                    # The primary datum source — must be the global EKF's
                    # own NavSatFix output (from global_ekf_to_navsatfix),
                    # NOT navsat_transform's /gps/filtered. The two have
                    # different lat/lon (navsat's is its own filtered
                    # estimate; the global EKF's is the actual fused
                    # state). Using the wrong one gave the diag a
                    # subtly-different datum than the algorithm anchored
                    # on, which produced spurious SBL-error offsets.
                    "datum_navsatfix_topic": "/gps/filtered/global",
                    "datum_topic_fallback": "/gps/validated",
                }],
            ),
            # ── Anchored-shadow gnss_anchored_pose node (only when
            # use_anchored_shadow:=true). Renamed outputs and publish_tf=false
            # to coexist with the global EKF stack without conflicts.
            # In-launch bag recorder. Spawned inside the launch so it shares
            # the DDS context with the publishers (the same WSL2 / docker
            # desktop pathology that broke `ros2 topic hz` from a side
            # shell also breaks bag record from a side shell — discovery
            # succeeds but message exchange fails). Off by default; turn
            # on with record_outputs:=true. Output: <diag_output_dir>/
            # replay_outputs/. If that directory already exists ros2 bag
            # record will error — delete it (or use a fresh diag_output_dir)
            # before re-running.
            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "record",
                    # MCAP matches the project's standard bag format —
                    # everything in recordings/rosbags/ is already mcap,
                    # so downstream scripts (rosbags AnyReader, overlay
                    # tools) work without storage-plugin gymnastics.
                    "--storage", "mcap",
                    # Disable compression: mcap's default zstd compression
                    # is CPU-intensive and on this Jetson under offline
                    # replay it eats enough CPU that the pressure→z fusion
                    # in the global EKF lags, producing a z-drift to ~-16 m
                    # by end of bag (v19 grid_02 vs v18 db3 which was +0.21 m
                    # on the same run). `fastwrite` mode writes uncompressed
                    # mcap — slightly larger files but zero CPU overhead.
                    "--storage-preset-profile", "fastwrite",
                    "-o", PathJoinSubstitution([
                        LaunchConfiguration("diag_output_dir"),
                        "replay_outputs",
                    ]),
                    # Raw sensors (replayed from input bag)
                    "/imu/data",
                    "/sensors/dvl/odometry_cov",
                    "/sensors/pressure/pose_enu",
                    "/gps/selected",
                    "/ubx_nav_hp_pos_llh",
                    "/waterlinked_ugps/navsatfix",
                    # Live-derived intermediates (from this launch's nodes)
                    "/imu/data_corrected",
                    "/gps/validated",
                    "/odometry/gps",
                    "/odometry/gps_floored",
                    "/odometry/gps_map",
                    "/sensors/pressure/pose_enu_map",
                    # Three localization outputs
                    "/odometry/filtered/local",
                    "/odometry/filtered/local_validated",
                    "/odometry/filtered/global",
                    "/odometry/filtered/global_anchored",
                    # NavSatFix forms of global outputs. Two distinct topics:
                    #   /gps/filtered          — published by navsat_transform_node
                    #                            (publish_filtered_gps: true), reflects
                    #                            navsat's idea of the boat's lat/lon
                    #                            given its current filtered odom input.
                    #   /gps/filtered/global   — published by global_ekf_to_navsatfix,
                    #                            directly converts the global EKF's
                    #                            /odometry/filtered/global to lat/lon.
                    #                            This is THE global EKF's NavSatFix
                    #                            output and is what downstream consumers
                    #                            should subscribe to for the GPS-fused
                    #                            position estimate.
                    "/gps/filtered",
                    "/gps/filtered/global",
                    "/gps/filtered/global_anchored",
                    # TF
                    "/tf",
                    "/tf_static",
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("record_outputs")),
            ),
            Node(
                package="ekf_localization_pkg",
                executable="gnss_anchored_pose",
                name="gnss_anchored_pose_shadow",
                output="screen",
                parameters=[{
                    "local_odom_topic": "/odometry/filtered/local_validated",
                    "gps_topic": LaunchConfiguration("gps_fix_topic"),
                    "global_odom_topic": "/odometry/filtered/global_anchored",
                    "global_navsatfix_topic": "/gps/filtered/global_anchored",
                    "h_acc_topic": LaunchConfiguration("anchored_h_acc_topic"),
                    "h_acc_max_m": ParameterValue(
                        LaunchConfiguration("anchored_h_acc_max_m"),
                        value_type=float,
                    ),
                    "reanchor_on_each_fix": False,
                    "yaw_offset_deg": ParameterValue(
                        LaunchConfiguration("anchored_yaw_offset_deg"),
                        value_type=float,
                    ),
                    "gps_antenna_offset_xyz": LaunchConfiguration(
                        "anchored_gps_antenna_offset_xyz"
                    ),
                    # CRITICAL: do not publish map→odom TF — that's
                    # ekf_global_node's job in this stack. Two publishers on
                    # the same TF parent→child corrupt the buffer.
                    "publish_tf": False,
                    "map_frame": "map",
                    "odom_frame": "odom",
                }],
                condition=IfCondition(LaunchConfiguration("use_anchored_shadow")),
            ),
        ]
    )
