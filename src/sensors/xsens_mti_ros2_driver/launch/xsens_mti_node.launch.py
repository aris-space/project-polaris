from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    respawn = True
    respawn_delay = 2.0

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "respawn",
            default_value="true",
            description="Automatically relaunch node if it exits/crashes.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "respawn_delay",
            default_value="2.0",
            description="Seconds to wait before restarting a crashed node.",
        )
    )

    # Set environment variables to control logging behavior
    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_USE_STDOUT", "1"))
    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))

    parameters_file_path = Path(
        get_package_share_directory("xsens_mti_ros2_driver"),
        "param",
        "xsens_mti_node.yaml",
    )
    xsens_mti_node = Node(
        package="xsens_mti_ros2_driver",
        executable="xsens_mti_node",
        name="xsens_mti_node",
        output="screen",
        parameters=[parameters_file_path],
        arguments=[],
        respawn=respawn,
        respawn_delay=respawn_delay,
    )
    ld.add_action(xsens_mti_node)

    # Static base_link -> imu_link (mounting only). Keep xsens pub_transform: false so the driver
    # does not also publish world -> imu_link.
    # DONE: Replace translation/rotation with measured IMU mounting values. From base_link TO imu_link
    static_tf_base_to_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_imu",
        arguments=[
            "--x",
            "0.049",
            "--y",
            "-0.0088",
            "--z",
            "0.076",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "imu_link",
        ],
        output="screen",
        respawn=respawn,
        respawn_delay=respawn_delay,
    )
    ld.add_action(static_tf_base_to_imu)

    return ld
