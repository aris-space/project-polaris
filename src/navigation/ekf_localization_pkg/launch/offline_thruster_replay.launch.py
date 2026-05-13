"""
Offline replay of thruster_velocity_estimator against a rosbag (MCAP).

The node publishes body-frame velocity estimates (/sensors/thruster/odometry_cov)
derived from servo PWM — activated only when DVL has been absent for dvl_timeout_s
seconds (default 3 s).

**Important — wall-clock DVL timeout:**
The DVL absence timer uses time.monotonic() (wall clock), NOT sim time.
Play the bag at 1x speed (no --rate flag) for correct dropout detection.
Faster playback will compress the apparent gap between DVL messages and
the node will not trigger fallback when it should.

**Usage:**

1. Terminal A — estimator node (this launch):
   ros2 launch ekf_localization_pkg offline_thruster_replay.launch.py

   Optional overrides:
     surge_a:=0.780 surge_b:=0.964   # model coefficients
     dvl_timeout_s:=3.0               # seconds without DVL before activating
     surge_channel:=0                 # servo array index (0-based)
     force_publish:=true              # publish even while DVL is healthy (for testing)

2. Terminal B — bag playback (1x speed, provide /clock):
   ros2 bag play /ros2_ws/recordings/zermatt_grid_01_2026_04_30-12_04_16/ \\
     --clock \\
     --topics /pixhawk/servo_output_raw /sensors/dvl/velocity /sensors/dvl/odometry_cov

   Add other bags or adjust --topics as needed. Always include /tf_static if
   the bag has it and downstream nodes need transforms.

3. Terminal C — observe output:
   ros2 topic echo /sensors/thruster/odometry_cov
   # or with rqt_plot: rqt_plot /sensors/thruster/odometry_cov/twist/twist/linear/x

**Interpreting output:**
- Node logs "activating thruster velocity fallback" when DVL dropout starts.
- Node logs "DVL recovered" when DVL messages resume.
- With force_publish:=true, output is published continuously alongside DVL.
- No output published while DVL is present (EKF uses DVL directly).
- vx in the Odometry twist is the surge velocity estimate in m/s.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetUseSimTime


def generate_launch_description():
    return LaunchDescription(
        [
            SetUseSimTime(True),

            DeclareLaunchArgument("surge_a", default_value="0.780",
                                  description="Model coefficient a (vx = a·sign(u)·|u|^b)"),
            DeclareLaunchArgument("surge_b", default_value="0.964",
                                  description="Model exponent b"),
            DeclareLaunchArgument("surge_channel", default_value="0",
                                  description="0-based index into servo_output_raw array"),
            DeclareLaunchArgument("pwm_neutral", default_value="1500",
                                  description="PWM microseconds for zero thrust"),
            DeclareLaunchArgument("pwm_range", default_value="500",
                                  description="Half-range used to normalise PWM to ±1"),
            DeclareLaunchArgument("dvl_timeout_s", default_value="3.0",
                                  description="Seconds without DVL before activating fallback"),
            DeclareLaunchArgument("surge_deadzone", default_value="0.01",
                                  description="|u| below which vx is forced to 0"),
            DeclareLaunchArgument("force_publish", default_value="false",
                                  description="Publish thruster odometry even while DVL is healthy (testing only)"),

            Node(
                package="ekf_localization_pkg",
                executable="thruster_velocity_estimator",
                name="thruster_velocity_estimator",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "surge_a": LaunchConfiguration("surge_a"),
                    "surge_b": LaunchConfiguration("surge_b"),
                    "surge_channel": LaunchConfiguration("surge_channel"),
                    "pwm_neutral": LaunchConfiguration("pwm_neutral"),
                    "pwm_range": LaunchConfiguration("pwm_range"),
                    "dvl_timeout_s": LaunchConfiguration("dvl_timeout_s"),
                    "surge_deadzone": LaunchConfiguration("surge_deadzone"),
                    "force_publish": LaunchConfiguration("force_publish"),
                    # Topic defaults match the bags
                    "servo_topic": "/pixhawk/servo_output_raw",
                    "dvl_topic": "/sensors/dvl/velocity",
                    "output_topic": "/sensors/thruster/odometry_cov",
                }],
            ),
        ]
    )
