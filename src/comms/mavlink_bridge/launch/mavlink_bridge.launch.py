from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#simulation:
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Launches MAVLink bridge nodes with optional automatic respawn.
    """
    respawn = True
    respawn_delay = 2.0
    #sim
    enable_external_odom = LaunchConfiguration("enable_external_odom")
    use_sim_time = LaunchConfiguration("use_sim_time")
    common_params = {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
    }
    #


    return LaunchDescription(
        [
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
            #sim:
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Forward to all mavlink_bridge nodes.",
            ),
            DeclareLaunchArgument(
                "enable_external_odom",
                default_value="true",
                description="If true, ros2_receiver forwards /odom (or external_odom_topic) as MAVLink ODOMETRY.",
            ),
            #
            Node(
                package="mavlink_bridge",
                executable="mavlink_publisher",
                name="mavlink_bridge_publisher",
                output="screen",
                respawn=respawn,
                respawn_delay=respawn_delay,
            ),
            Node(
                package="mavlink_bridge",
                executable="ros2_receiver",
                name="ros2_receiver",
                output="screen",
                respawn=respawn,
                respawn_delay=2.0,
                #
                parameters=[common_params],
            ),
            Node(
                package="mavlink_bridge",
                executable="battery_tracker",
                name="battery_tracker",
                output="screen",
                respawn=respawn,
                respawn_delay=2.0,
                #
                parameters=[
                    common_params,
                    {
                        "enable_external_odom": ParameterValue(
                            enable_external_odom, value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
