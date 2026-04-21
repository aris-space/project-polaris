"""
Launch file for the mode control system.

Starts all mode-related nodes:
  - mode_control_node:                  Translates joystick safety+button combos into mode
                                        changes (MANUAL/ALT_HOLD/STABILIZE/GUIDED) and
                                        arm/disarm commands. Publishes to both
                                        /mode_control/current_mode and /pixhawk/mode_cmd.
  - manual_control_node:                6DOF joystick control (active in MANUAL/STABILIZE).
  - manual_altitude_hold_control_node:  4DOF joystick control with depth hold (active in ALT_HOLD).
  - collision_avoidance_node:           Monitors front ultrasonic distance; disarms and resets to
                                        MANUAL when too close.

All control nodes publish on the shared /pixhawk/manual_control topic, but only the
node matching the current mode will actually send data at any given time.
Emergency stop is handled by disarm (L3 or R3) — there is no dedicated emergency_stop mode.
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
                executable="collision_avoidance_node",
                name="collision_avoidance_node",
                output="screen",
                respawn=respawn,
                respawn_delay=respawn_delay,
            ),
        ]
    )
