"""
Starts navsat_transform_node and (optionally) ekf_global_node with a precise
GPS datum written by gnss_datum_watchdog.

Not intended for manual invocation. Spawned automatically by the watchdog
once a quality-gated GNSS fix is available, so navsat_transform never starts
at null-island (0°, 0°).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory("ekf_localization_pkg")
    default_navsat_params = Path(pkg_dir, "config", "navsat_transform.yaml")
    default_global_params = Path(pkg_dir, "config", "ekf_global.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "datum_yaml",
                description=(
                    "Path to a one-shot YAML providing "
                    "datum: [lat, lon, alt] for navsat_transform_node."
                ),
            ),
            DeclareLaunchArgument(
                "navsat_params_file",
                default_value=str(default_navsat_params),
                description="Base navsat_transform params (frequency, mag_declination, etc.).",
            ),
            DeclareLaunchArgument(
                "global_ekf_params_file",
                default_value=str(default_global_params),
                description="Global EKF params.",
            ),
            DeclareLaunchArgument(
                "gps_fix_topic",
                default_value="/gps/selected",
                description="GNSS NavSatFix topic remapped to navsat_transform gps/fix.",
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value="/imu/data_corrected",
                description="IMU topic remapped to navsat_transform imu.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odometry/filtered/local",
                description="Local EKF odometry remapped to navsat_transform odometry/filtered.",
            ),
            DeclareLaunchArgument(
                "use_global_ekf",
                default_value="true",
                description="Set false to skip ekf_global_node (navsat-only mode).",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Forward use_sim_time from the parent launch (bag replay).",
            ),
            DeclareLaunchArgument(
                "global_odom_topic",
                default_value="/odometry/filtered/global",
                description="Global EKF topic consumed by global_ekf_to_navsatfix_node.",
            ),
            DeclareLaunchArgument(
                "datum_lat",
                description="Datum latitude (degrees) — forwarded from gnss_datum_watchdog.",
            ),
            DeclareLaunchArgument(
                "datum_lon",
                description="Datum longitude (degrees) — forwarded from gnss_datum_watchdog.",
            ),
            DeclareLaunchArgument(
                "datum_alt",
                description="Datum altitude (meters) — forwarded from gnss_datum_watchdog.",
            ),
            DeclareLaunchArgument(
                "local_anchor_x",
                default_value="0.0",
                description=(
                    "Local-EKF x position at GNSS-lock — subtracted from "
                    "/odometry/gps to convert from navsat's odom frame to "
                    "map frame inside gps_odom_cov_floor."
                ),
            ),
            DeclareLaunchArgument(
                "local_anchor_y",
                default_value="0.0",
                description="Local-EKF y position at GNSS-lock.",
            ),
            DeclareLaunchArgument(
                "local_anchor_z",
                default_value="0.0",
                description="Local-EKF z position at GNSS-lock.",
            ),
            DeclareLaunchArgument(
                "local_anchor_yaw",
                default_value="0.0",
                description=(
                    "Local-EKF yaw (radians) at GNSS-lock — forwarded as "
                    "map_yaw_offset_rad to global_ekf_to_navsatfix so that "
                    "the back-projection rotates state.(x, y) from the map "
                    "frame (rotated by navsat_transform's lock-time yaw) "
                    "into UTM ENU before adding to the datum. Without this "
                    "the published /gps/filtered/global is offset from /fix "
                    "by a heading-dependent term that scales with distance "
                    "from datum."
                ),
            ),
            # /clock-propagation grace period.
            #
            # This launch file is spawned as a fresh subprocess by
            # gnss_datum_watchdog._spawn_navsat_and_global once a valid GNSS fix
            # arrives. The new process re-discovers topics and re-subscribes to
            # /clock from scratch. Between subprocess start and the first /clock
            # message arriving, rclcpp::Time::now() returns wall-clock even
            # though use_sim_time=true is set. Any node that publishes during
            # that window stamps its first message(s) with wall-clock, and the
            # global EKF then processes those messages with dt = wall_clock −
            # sim_time ≈ 3 days for offline-replay of recent bags — its predict
            # step integrates velocity over the bogus dt and position explodes.
            #
            # ALL three nodes (navsat_transform, ekf_global, global_ekf_to_navsatfix)
            # must be inside the timer. Earlier we left navsat_transform_node at
            # top level "because it has no integrated state" — but its first
            # /odometry/gps message inherits the wall-clock stamp, the EKF
            # consumes it later, and the dt blow-up still happens. Confirmed
            # in the debug log: the largest predict-step delta in a run was
            # 298 174 s, exactly matching wall-clock − bag-time, and that
            # single bad measurement produced a state jump from <1 km to 49 km
            # of horizontal position and 263 km of z (z is unsensored, so it
            # could only have grown from vz × dt integration).
            TimerAction(
                period=10.0,
                actions=[
                    # Bootstrap identity map->odom so the global EKF can
                    # transform incoming pose measurements on the very first
                    # tick. Without this, the first odom0_pose message arrives
                    # before ekf_global_node has published its own map->odom,
                    # the TF lookup fails, the pose is dropped, and the filter
                    # initialises off odom0_twist alone — leaving state yaw
                    # stuck at 0 while local yaw is ~2.4 rad. Subsequent pose
                    # messages then get rotated through that wrong TF and the
                    # state never recovers. Once ekf_global_node starts
                    # publishing its own map->odom, that broadcast wins because
                    # it carries the latest stamp; this static publisher is
                    # only load-bearing during the startup window.
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        name="map_odom_bootstrap",
                        output="screen",
                        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                    ),
                    Node(
                        package="robot_localization",
                        executable="navsat_transform_node",
                        name="navsat_transform_node",
                        output="screen",
                        # Base config loaded first; datum_yaml loaded second so its
                        # datum: [lat, lon, alt] entry overrides any placeholder in the base.
                        parameters=[
                            LaunchConfiguration("navsat_params_file"),
                            LaunchConfiguration("datum_yaml"),
                            {"use_sim_time": LaunchConfiguration("use_sim_time")},
                        ],
                        remappings=[
                            ("gps/fix", LaunchConfiguration("gps_fix_topic")),
                            ("imu", LaunchConfiguration("imu_topic")),
                            ("odometry/filtered", LaunchConfiguration("odom_topic")),
                        ],
                    ),
                    # Floor /odometry/gps's pose-covariance diagonal so the
                    # global EKF's first GPS update doesn't blow up due to
                    # numerical instability with R ≈ 4e-5 m² (RTK Fixed
                    # noise floor). Republishes on /odometry/gps_floored.
                    # ekf_global.yaml's odom1 must point at the floored
                    # topic for this to take effect.
                    # pressure_pose_frame_fix: convert
                    # /sensors/pressure/pose_enu (odom frame) to map frame
                    # for the global EKF. Same TF-feedback-loop fix
                    # mechanism as gps_odom_cov_floor; see that node's
                    # docstring or the v20 entry in EKF_RESEARCH_NOTES.md.
                    Node(
                        package="ekf_localization_pkg",
                        executable="pressure_pose_frame_fix",
                        name="pressure_pose_frame_fix",
                        output="screen",
                        parameters=[{
                            "input_topic": "/sensors/pressure/pose_enu",
                            "output_topic": "/sensors/pressure/pose_enu_map",
                            "output_frame_id": "map",
                            "local_anchor_z": ParameterValue(
                                LaunchConfiguration("local_anchor_z"),
                                value_type=float,
                            ),
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                        }],
                        condition=IfCondition(LaunchConfiguration("use_global_ekf")),
                    ),
                    # gps_to_map_position: project /gps/validated directly to
                    # true-ENU map-frame Odometry on /odometry/gps_map.
                    # Replaces the navsat_transform → gps_odom_cov_floor
                    # path for the global EKF's GPS input. See node
                    # docstring or ekf_global.yaml comment on odom1.
                    Node(
                        package="ekf_localization_pkg",
                        executable="gps_to_map_position",
                        name="gps_to_map_position",
                        output="screen",
                        parameters=[{
                            "input_topic": LaunchConfiguration("gps_fix_topic"),
                            "output_topic": "/odometry/gps_map",
                            "output_frame_id": "map",
                            "datum_lat": ParameterValue(
                                LaunchConfiguration("datum_lat"),
                                value_type=float,
                            ),
                            "datum_lon": ParameterValue(
                                LaunchConfiguration("datum_lon"),
                                value_type=float,
                            ),
                            "datum_alt": ParameterValue(
                                LaunchConfiguration("datum_alt"),
                                value_type=float,
                            ),
                            "min_pos_cov_m2": 0.25,
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                        }],
                        condition=IfCondition(LaunchConfiguration("use_global_ekf")),
                    ),
                    # gps_odom_cov_floor: legacy path. Kept running for
                    # backward compatibility (publishes /odometry/gps_floored)
                    # but the EKF no longer subscribes here — see
                    # ekf_global.yaml odom1 which now points at
                    # /odometry/gps_map from gps_to_map_position above.
                    # Safe to disable in production once the new path
                    # is fully validated.
                    Node(
                        package="ekf_localization_pkg",
                        executable="gps_odom_cov_floor",
                        name="gps_odom_cov_floor",
                        output="screen",
                        parameters=[{
                            "input_topic": "/odometry/gps",
                            "output_topic": "/odometry/gps_floored",
                            # Relabel frame_id "odom" -> "map" AND subtract
                            # local_anchor to break the navsat-via-TF
                            # feedback loop. See gps_odom_cov_floor.py
                            # _on_odom for the mechanism.
                            "output_frame_id": "map",
                            "local_anchor_x": ParameterValue(
                                LaunchConfiguration("local_anchor_x"),
                                value_type=float,
                            ),
                            "local_anchor_y": ParameterValue(
                                LaunchConfiguration("local_anchor_y"),
                                value_type=float,
                            ),
                            "local_anchor_z": ParameterValue(
                                LaunchConfiguration("local_anchor_z"),
                                value_type=float,
                            ),
                            # 0.25 m² ≡ 50 cm 1σ. Earlier value 0.01 (10 cm
                            # 1σ) was too aggressive: at P_xx ≈ 2 m² the
                            # post-update P collapses to ≈ 0.005 m² which
                            # then takes a full Q*dt ≈ 1 s to grow back —
                            # but at the EKF's observed ~1 Hz cycle, that's
                            # one whole cycle of state collapsing into a
                            # tiny ball where even small cross-covariance
                            # bleed into unsensored states (vx, ax) gets
                            # amplified by K ≈ 1. The looser floor keeps
                            # the post-update P at ≈ 0.22 m² (K ≈ 0.89,
                            # still strong pull), which is far more robust
                            # to the cross-covariance amplification mode
                            # documented in the v7/v8 grid_02 explosions.
                            "min_pos_cov_m2": 0.25,
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                        }],
                        condition=IfCondition(LaunchConfiguration("use_global_ekf")),
                    ),
                    Node(
                        package="robot_localization",
                        executable="ekf_node",
                        name="ekf_global_node",
                        output="screen",
                        parameters=[
                            LaunchConfiguration("global_ekf_params_file"),
                            {"use_sim_time": LaunchConfiguration("use_sim_time")},
                        ],
                        # set_pose is declared by robot_localization as a relative
                        # name; without a namespace it resolves to root /set_pose,
                        # which would make EVERY ekf_node in the graph subscribe
                        # to the same global topic. Remap it so this EKF listens
                        # on its own node-name-prefixed topic, matching what
                        # gnss_datum_watchdog publishes to. Without this remap,
                        # the bootstrap message goes to /ekf_global_node/set_pose
                        # while the EKF listens on /set_pose — they never connect
                        # and the EKF cold-starts.
                        remappings=[
                            ("odometry/filtered", "/odometry/filtered/global"),
                            ("set_pose", "/ekf_global_node/set_pose"),
                        ],
                        condition=IfCondition(LaunchConfiguration("use_global_ekf")),
                    ),
                    Node(
                        package="ekf_localization_pkg",
                        executable="global_ekf_to_navsatfix",
                        name="global_ekf_to_navsatfix_node",
                        output="screen",
                        parameters=[{
                            "datum_lat": LaunchConfiguration("datum_lat"),
                            "datum_lon": LaunchConfiguration("datum_lon"),
                            "datum_alt": LaunchConfiguration("datum_alt"),
                            # 0.0 — state is now in true ENU map frame
                            # (the EKF subscribes to /odometry/gps_map from
                            # gps_to_map_position which projects directly
                            # via pyproj, no navsat rotation). The
                            # back-projection is therefore a pure
                            # datum + state addition with no rotation.
                            # The watchdog still captures local_anchor_yaw
                            # for diagnostics but it's not used here.
                            "map_yaw_offset_rad": 0.0,
                            "global_odom_topic": LaunchConfiguration("global_odom_topic"),
                            "use_sim_time": LaunchConfiguration("use_sim_time"),
                        }],
                        condition=IfCondition(LaunchConfiguration("use_global_ekf")),
                    ),
                ],
            ),
        ]
    )
