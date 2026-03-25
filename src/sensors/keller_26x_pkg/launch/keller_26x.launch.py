import os

import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, LifecycleTransition, Node

_KELLER_LIFECYCLE_NODE_NAME = "/sensors/keller_26x"

def _parse_bool(raw_value: str) -> bool:
    return raw_value.strip().lower() in {"1", "true", "yes", "on"}

def _launch_setup(context, *args, **kwargs):
    # Resolve substitutions to concrete Python types for ExecuteLocal internals.
    #range_mode = LaunchConfiguration("range_mode")
    respawn = _parse_bool(LaunchConfiguration("respawn").perform(context))
    respawn_delay = float(LaunchConfiguration("respawn_delay").perform(context))
    configure_delay = float(LaunchConfiguration("configure_delay_sec").perform(context))

    config = os.path.join(
        get_package_share_directory("keller_26x_pkg"),
        "config",
        "keller_26x.yaml",
    )

    keller_26x_node = LifecycleNode(
        namespace="sensors",
        package='keller_26x_pkg',
        executable='keller_26x_node',
        name='keller_26x',
        parameters=[config],
        output="screen",
        respawn=respawn,
        respawn_delay=respawn_delay,
    )

    # Chain configure → activate using the same matchers as upstream launch_ros (start_state +
    # goal_state per transition). Manual OnStateTransition on goal_state alone can miss events
    # or misfire depending on rmw / TransitionEvent labeling.
    auto_lifecycle = LifecycleTransition(
        lifecycle_node_names=[_KELLER_LIFECYCLE_NODE_NAME],
        transition_ids=[
            lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        ],
    )

    # Defer: DDS + ~/change_state must exist before the first transition; Jetson/Docker often
    # needs several seconds. "Node not found" from ros2 lifecycle on the host usually means
    # wrong ROS_DOMAIN_ID vs docker-compose or querying before the node appears.
    on_process_start = RegisterEventHandler(
        OnProcessStart(
            target_action=keller_26x_node,
            on_start=[
                LogInfo(
                    msg=(
                        f"[launch_keller_26x] Auto lifecycle for {_KELLER_LIFECYCLE_NODE_NAME} "
                        f"starts in {configure_delay:g}s (configure then activate)."
                    )
                ),
                TimerAction(period=configure_delay, actions=[auto_lifecycle]),
            ],
        )
    )

    # Static Transform: parent must match your robot base; child must match driver ``frame`` /
    # odometry.child_frame_id (see sensor_frame launch argument).
    # CAD: CENTER_OF_MASS_LINK -> DVL_LINK [m]; extrinsic RPY (rad): pi, 0, -pi/4.
    """
    static_tf_base_to_dvl = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_keller",
        arguments=[
            "--x", "-0.736",
            "--y", "0.000403",
            "--z", "0.068",
            "--roll", "3.141592653589793",
            "--pitch", "0.0",
            "--yaw", "-0.7853981633974483",
            "--frame-id", "base_link",
            "--child-frame-id", sensor_frame,
        ],
        output="screen",
        respawn=respawn,
        respawn_delay=respawn_delay,
    )
    """

    return [
        keller_26x_node,
        on_process_start,
        # static_tf_base_to_dvl,
    ]


def generate_launch_description():

    respawn_arg = DeclareLaunchArgument(
        "respawn",
        default_value="true",
        description="Automatically relaunch node if it exits/crashes.",
    )
    respawn_delay_arg = DeclareLaunchArgument(
        "respawn_delay",
        default_value="2.0",
        description="Seconds to wait before restarting a crashed node.",
    )
    configure_delay_arg = DeclareLaunchArgument(
        "configure_delay_sec",
        default_value="8.0",
        description=(
            "Wait after Keller_26x process starts before auto configure+activate "
            "(increase on Jetson/Docker if ~/change_state is not ready yet)."
        ),
    )
    

    return LaunchDescription([
        respawn_arg,
        respawn_delay_arg,
        configure_delay_arg,
        OpaqueFunction(function=_launch_setup),
    ])