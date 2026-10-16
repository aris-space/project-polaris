"""
Offline replay of adaptive_thruster_estimator against a rosbag (MCAP).

The node learns the thruster model (k, b) from DVL + servo data and, when
force_publish is true, also publishes /sensors/thruster/odometry_cov while
DVL is healthy — allowing direct comparison of the RLS estimate vs DVL.

**Important — wall-clock DVL timeout:**
The DVL absence timer uses steady_clock (wall clock), NOT sim time.
Play the bag at 1x speed (no --rate flag) for correct timeout behaviour.

**Usage:**

1. Terminal A — estimator node (this launch):
   ros2 launch adaptive_thruster_estimator offline_thruster_replay.launch.py \\
     force_publish:=true

2. Terminal B — bag playback (1x speed):
   ros2 bag play /ros2_ws/recordings/zermatt_grid_01_2026_04_30-12_04_16/ \\
     --clock \\
     --topics /pixhawk/servo_output_raw /sensors/dvl/velocity /sensors/dvl/odometry_cov

3. Terminal C — observe output:
   ros2 topic echo /sensors/thruster/odometry_cov
   ros2 topic echo /adaptive_thruster_estimator/diagnostics
   # diagnostics: [k, b, P00, P11, trace(P), dvl_age_s]
   # k and b update live as the bag plays — P shrinks as confidence grows
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("force_publish", default_value="false",
                              description="Publish thruster odometry even while DVL is healthy (testing only)"),
        DeclareLaunchArgument("lambda",        default_value="0.995",
                              description="RLS forgetting factor"),
        DeclareLaunchArgument("init_k",        default_value="0.780",
                              description="Prior gain coefficient (m/s)"),
        DeclareLaunchArgument("init_b",        default_value="0.964",
                              description="Prior power-law exponent"),
        DeclareLaunchArgument("surge_channel", default_value="0",
                              description="0-based index into servo_output_raw array"),
        DeclareLaunchArgument("pwm_neutral",   default_value="1500",
                              description="PWM microseconds for zero thrust"),
        DeclareLaunchArgument("pwm_range",     default_value="500",
                              description="Half-range used to normalise PWM to ±1"),

        Node(
            package="adaptive_thruster_estimator",
            executable="adaptive_thruster_estimator",
            name="adaptive_thruster_estimator",
            output="screen",
            parameters=[{
                "force_publish": LaunchConfiguration("force_publish"),
                "lambda":        LaunchConfiguration("lambda"),
                "init_k":        LaunchConfiguration("init_k"),
                "init_b":        LaunchConfiguration("init_b"),
                "surge_channel": LaunchConfiguration("surge_channel"),
                "pwm_neutral":   LaunchConfiguration("pwm_neutral"),
                "pwm_range":     LaunchConfiguration("pwm_range"),
            }],
        ),
    ])
