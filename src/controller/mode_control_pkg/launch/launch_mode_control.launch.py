"""
Launch file for the mode control system.

Starts all mode-related nodes:
  - mode_control_node:                  Listens to joystick mode-switch inputs and publishes
                                        the active mode on 'current_mode'. Also sends the
                                        corresponding Pixhawk flight mode via 'pixhawk/mode_cmd'.
  - manual_control_node:                6DOF joystick control (active in 'manual_control' mode).
  - manual_altitude_hold_control_node:  4DOF joystick control with depth hold
                                        (active in 'manual_depth_hold' mode).
    - step_inputs_mode:                   PID tuning step commands with submodes
                                            attitude/depthhold/poshold.
  - emergency_stop_mode_node:           Sends neutral commands to stop all thrusters
                                        (active in 'emergency_stop' mode).

All mode nodes publish on the shared 'pixhawk/manual_control' topic, but only the
node matching the current mode will actually send data at any given time.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    respawn = True
    respawn_delay = 2.0

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
            Node(
                package="mode_control_pkg",
                executable="joy_handler_node",
                name="joy_handler_node",
                output="screen",
                respawn=respawn,
                respawn_delay=respawn_delay,
                parameters=[
                    {
                        "keyboard_source_frame_id": "keyboard",
                        "controller_source_frame_id": "controller",
                        "mode_only_source_frame_id": "mode",
                    }
                ],
            ),
            Node(
                package="mode_control_pkg",
                executable="mode_control_node",
                name="mode_control_node",
                output="screen",
                respawn=respawn,
                respawn_delay=respawn_delay,
            ),
            Node(
                package="mode_control_pkg",
                executable="manual_control_node",
                name="manual_control_node",
                output="screen",
                respawn=respawn,
                respawn_delay=respawn_delay,
                parameters=[
                    {
                        "keyboard_source_frame_id": "keyboard",
                        "controller_source_frame_id": "controller",
                        "controller_gain_x": 1000.0,
                        "controller_gain_y": 500.0,
                        "controller_gain_z": 500.0,
                        "controller_gain_r": 500.0,
                        "controller_gain_s": 300.0,
                        "controller_gain_t": 300.0,
                        "keyboard_gain_x": 1000.0,
                        "keyboard_gain_y": 500.0,
                        "keyboard_gain_z": 500.0,
                        "keyboard_gain_r": 500.0,
                        "keyboard_gain_s": 300.0,
                        "keyboard_gain_t": 300.0,
                        "keyboard_x_single_press_gain": 500.0,
                        "keyboard_x_double_press_gain": 1000.0,
                        "keyboard_x_double_press_window_s": 0.2,
                    }
                ],
            ),
            Node(
                package="mode_control_pkg",
                executable="manual_altitude_hold_control_node",
                name="manual_altitude_hold_control_node",
                output="screen",
                respawn=respawn,
                respawn_delay=respawn_delay,
                parameters=[
                    {
                        "keyboard_source_frame_id": "keyboard",
                        "controller_source_frame_id": "controller",
                        "controller_gain_x": 1000.0,
                        "controller_gain_y": 500.0,
                        "controller_gain_z": 500.0,
                        "controller_gain_r": 500.0,
                        "keyboard_gain_x": 1000.0,
                        "keyboard_gain_y": 500.0,
                        "keyboard_gain_z": 500.0,
                        "keyboard_gain_r": 500.0,
                        "keyboard_x_single_press_gain": 500.0,
                        "keyboard_x_double_press_gain": 1000.0,
                        "keyboard_x_double_press_window_s": 0.2,
                    }
                ],
            ),
            Node(
                package="mode_control_pkg",
                executable="step_inputs_mode",
                name="step_inputs_mode",
                output="screen",
                respawn=respawn,
                respawn_delay=respawn_delay,
                parameters=[
                    {
                        "/tuning/active/attitude": False,
                        "/tuning/active/depthhold": False,
                        "/tuning/active/poshold": False,
                        "tuning/target/attitude_roll_deg": 0.0,
                        "tuning/target/attitude_pitch_deg": 0.0,
                        "tuning/target/attitude_yaw_deg": 0.0,
                        "tuning/target/depthhold_z_m": -1.0,
                        "tuning/target/poshold_x_m": 0.0,
                        "tuning/target/poshold_y_m": 0.0,
                        "tuning/target/poshold_z_m": -1.0,
                    }
                ],
            ),
            Node(
                package="mode_control_pkg",
                executable="emergency_stop_mode_node",
                name="emergency_stop_mode_node",
                output="screen",
                respawn=respawn,
                respawn_delay=respawn_delay,
            ),
            Node(
                package="mode_control_pkg",
                executable="collision_avoidance_node",
                name="collision_avoidance_node",
                output="screen",
                respawn=respawn,
                respawn_delay=respawn_delay,
            ),
        ]
    )
