import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    package_dir = get_package_share_directory("gnss_bringup_pkg")
    default_config = os.path.join(package_dir, "config", "ublox_x20p_rover.yaml")

    gnss_config = LaunchConfiguration("gnss_config")
    log_level = LaunchConfiguration("log_level")
    namespace = LaunchConfiguration("namespace")

    use_ntrip = LaunchConfiguration("use_ntrip")
    ntrip_use_https = LaunchConfiguration("ntrip_use_https")
    ntrip_host = LaunchConfiguration("ntrip_host")
    ntrip_port = LaunchConfiguration("ntrip_port")
    ntrip_mountpoint = LaunchConfiguration("ntrip_mountpoint")
    ntrip_username = LaunchConfiguration("ntrip_username")
    ntrip_password = LaunchConfiguration("ntrip_password")
    ntrip_version = LaunchConfiguration("ntrip_version")
    respawn = LaunchConfiguration("respawn")
    respawn_delay = LaunchConfiguration("respawn_delay")

    gnss_container = ComposableNodeContainer(
        name="ublox_dgnss_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        respawn=respawn,
        respawn_delay=respawn_delay,
        arguments=["--ros-args", "--log-level", log_level],
        composable_node_descriptions=[
            ComposableNode(
                package="ublox_dgnss_node",
                plugin="ublox_dgnss::UbloxDGNSSNode",
                name="ublox_dgnss",
                namespace=namespace,
                parameters=[gnss_config],
                remappings=[
                    # Keep only required UBX topics visible; move others to hidden topics.
                    ("ubx_nav_clock", "/_ublox_hidden/ubx_nav_clock"),
                    ("ubx_nav_dop", "/_ublox_hidden/ubx_nav_dop"),
                    ("ubx_nav_eoe", "/_ublox_hidden/ubx_nav_eoe"),
                    ("ubx_nav_hp_pos_ecef", "/_ublox_hidden/ubx_nav_hp_pos_ecef"),
                    ("ubx_nav_odo", "/_ublox_hidden/ubx_nav_odo"),
                    ("ubx_nav_orb", "/_ublox_hidden/ubx_nav_orb"),
                    ("ubx_nav_sig", "/_ublox_hidden/ubx_nav_sig"),
                    ("ubx_nav_pos_ecef", "/_ublox_hidden/ubx_nav_pos_ecef"),
                    ("ubx_nav_pos_llh", "/_ublox_hidden/ubx_nav_pos_llh"),
                    ("ubx_nav_rel_pos_ned", "/_ublox_hidden/ubx_nav_rel_pos_ned"),
                    ("ubx_nav_svin", "/_ublox_hidden/ubx_nav_svin"),
                    ("ubx_nav_time_utc", "/_ublox_hidden/ubx_nav_time_utc"),
                    ("ubx_nav_vel_ecef", "/_ublox_hidden/ubx_nav_vel_ecef"),
                    ("ubx_nav_vel_ned", "/_ublox_hidden/ubx_nav_vel_ned"),
                    ("ubx_rxm_cor", "/_ublox_hidden/ubx_rxm_cor"),
                    ("ubx_rxm_measx", "/_ublox_hidden/ubx_rxm_measx"),
                    ("ubx_rxm_rawx", "/_ublox_hidden/ubx_rxm_rawx"),
                    ("ubx_rxm_spartn", "/_ublox_hidden/ubx_rxm_spartn"),
                    ("ubx_rxm_spartnkey", "/_ublox_hidden/ubx_rxm_spartnkey"),
                    ("ubx_esf_status", "/_ublox_hidden/ubx_esf_status"),
                    ("ubx_esf_meas", "/_ublox_hidden/ubx_esf_meas"),
                    ("ubx_esf_meas_to_device", "/_ublox_hidden/ubx_esf_meas_to_device"),
                    ("ubx_mon_comms", "/_ublox_hidden/ubx_mon_comms"),
                    ("ubx_sec_sig", "/_ublox_hidden/ubx_sec_sig"),
                    ("ubx_sec_sig_log", "/_ublox_hidden/ubx_sec_sig_log"),
                ],
            ),
            ComposableNode(
                package="ublox_nav_sat_fix_hp_node",
                plugin="ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode",
                name="ublox_nav_sat_fix_hp",
                namespace=namespace,
            ),
        ],
    )

    # Static base_link -> gnss_link (must match FRAME_ID in gnss_config, e.g. ublox_x20p_rover.yaml).
    # CAD: translation CENTER_OF_MASS_LINK -> GNSS_LINK, no rotation (meters, vehicle frame).
    # Assumes base_link coincides with center-of-mass (same convention as IMU/DVL static TFs).
    static_tf_base_to_gnss = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_gnss",
        arguments=[
            "--x",
            "-0.048",
            "--y",
            "-0.002",
            "--z",
            "0.224",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "gnss_link",
        ],
        output="screen",
        respawn=respawn,
        respawn_delay=respawn_delay,
    )

    # Use LORD ntrip_client here because it can consume /fix and generate/sent GGA
    # upstream to VRS casters such as SWIPOS.
    ntrip_node = GroupAction(
        condition=IfCondition(use_ntrip),
        actions=[
            Node(
                package="gnss_bringup_pkg",
                executable="fix_qos_bridge",
                name="fix_qos_bridge",
                output="screen",
                respawn=respawn,
                respawn_delay=respawn_delay,
            ),
            PushRosNamespace("ntrip_client"),
            Node(
                package="ntrip_client",
                executable="ntrip_ros.py",
                name="ntrip_client",
                output="screen",
                respawn=respawn,
                respawn_delay=respawn_delay,
                parameters=[
                    {
                        "host": ntrip_host,
                        "port": ntrip_port,
                        "mountpoint": ntrip_mountpoint,
                        "authenticate": True,
                        "username": ntrip_username,
                        "password": ntrip_password,
                        "ssl": ntrip_use_https,
                        "ntrip_version": ntrip_version,
                        "rtcm_message_package": "rtcm_msgs",
                    }
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gnss_config", default_value=default_config),
            DeclareLaunchArgument("log_level", default_value="INFO"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_ntrip", default_value="false"),
            DeclareLaunchArgument(
                "ntrip_use_https",
                default_value=EnvironmentVariable(
                    "NTRIP_USE_HTTPS", default_value="false"
                ),
            ),
            DeclareLaunchArgument(
                "ntrip_host",
                default_value=EnvironmentVariable(
                    "NTRIP_HOST", default_value="www.swipos.ch"
                ),
            ),
            DeclareLaunchArgument(
                "ntrip_port",
                default_value=EnvironmentVariable("NTRIP_PORT", default_value="2101"),
            ),
            DeclareLaunchArgument(
                "ntrip_mountpoint",
                default_value=EnvironmentVariable(
                    "NTRIP_MOUNTPOINT", default_value="MSM_GISGEO_LV95LHN95"
                ),
            ),
            DeclareLaunchArgument("ntrip_version", default_value="Ntrip/1.0"),
            DeclareLaunchArgument(
                "respawn",
                default_value="true",
                description="Automatically relaunch node if it exits/crashes.",
            ),
            DeclareLaunchArgument(
                "respawn_delay",
                default_value="2.0",
                description="Seconds to wait before restarting a crashed node.",
            ),
            DeclareLaunchArgument(
                "ntrip_username",
                default_value=EnvironmentVariable(
                    "NTRIP_USERNAME", default_value=""
                ),
            ),
            DeclareLaunchArgument(
                "ntrip_password",
                default_value=EnvironmentVariable(
                    "NTRIP_PASSWORD", default_value=""
                ),
            ),
            gnss_container,
            static_tf_base_to_gnss,
            ntrip_node,
        ]
    )
