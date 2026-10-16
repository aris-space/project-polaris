"""Laptop-only SBL raw-collection mission (thesis: tightly-coupled SBL particle filter).

Launches ONLY the Waterlinked UGPS G2 driver (in raw-publishing mode) plus a ``ros2 bag record``,
so a first-week data-collection test can run on a Windows laptop in a devcontainer talking to the
UGPS box over HTTP — no onboard computer, no ublox, no static TF. It records the additive raw
per-receiver ranges (``sbl_raw``), the latched receiver geometry (``sbl_receiver_config``) and the
Waterlinked-solved position topics for comparison.

Launch args:
  bag_dir   : directory to write the .mcap bag into (a timestamped subfolder is created). Default:
              $SBL_BAG_DIR if set, else ./recordings (gitignored). Use an absolute path to be safe.
  record    : "true"/"false" — set "false" to run the driver without recording (e.g. to eyeball
              topics with `ros2 topic echo`).
  bag_all   : "true" records all topics (-a); "false" records only the SBL + solved-position topics.

See docs/SBL_DRIVER_CHANGE_MISSION.md for how to run and verify this.
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NS = "/waterlinked_ugps"
SBL_TOPICS = [
    f"{NS}/sbl_raw",
    f"{NS}/sbl_receiver_config",
    f"{NS}/locator_position_global",
    f"{NS}/locator_acoustic_quality",
    f"{NS}/locator_position_relative_wrt_topside",
    f"{NS}/locator_position_topside_ned",
    f"{NS}/antenna_marker",
]


def _build_nodes(context):
    record = LaunchConfiguration("record").perform(context).lower() in ("1", "true", "yes")
    bag_all = LaunchConfiguration("bag_all").perform(context).lower() in ("1", "true", "yes")
    bag_dir = LaunchConfiguration("bag_dir").perform(context)
    if not bag_dir:
        bag_dir = os.environ.get("SBL_BAG_DIR", os.path.join(os.getcwd(), "recordings"))

    params = os.path.join(
        get_package_share_directory("uwgpsg2_ros2_interface"),
        "config",
        "waterlinked_laptop_sbl_params.yaml",
    )

    # Driver node: raw-publishing mode. Namespace + name must match the params file's keys.
    # Only the SBL/solved-position outputs are remapped to /waterlinked_ugps/*; the fix/navrelposned
    # inputs are intentionally NOT wired (no ublox on the laptop).
    driver = Node(
        namespace="waterlinked/ugps",
        name="waterlinked_localization_node",
        package="uwgpsg2_ros2_interface",
        executable="uwgpsg2_ros2_interface",
        output="screen",
        emulate_tty=True,
        parameters=[params],
        remappings=[
            ("sbl_raw", f"{NS}/sbl_raw"),
            ("sbl_receiver_config", f"{NS}/sbl_receiver_config"),
            ("locator_position_relative_wrt_topside", f"{NS}/locator_position_relative_wrt_topside"),
            ("locator_position_global", f"{NS}/locator_position_global"),
            ("locator_acoustic_quality", f"{NS}/locator_acoustic_quality"),
            ("locator_position_topside_ned", f"{NS}/locator_position_topside_ned"),
        ],
    )

    actions = [driver]

    if record:
        stamp = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
        bag_path = os.path.join(bag_dir, f"sbl_raw_{stamp}")
        os.makedirs(bag_dir, exist_ok=True)
        cmd = ["ros2", "bag", "record", "-s", "mcap", "-o", bag_path]
        cmd += ["-a"] if bag_all else SBL_TOPICS
        actions.append(
            ExecuteProcess(cmd=cmd, output="screen", emulate_tty=True)
        )
        print(f"[sbl_laptop_record] recording -> {bag_path}  (all={bag_all})")
    else:
        print("[sbl_laptop_record] recording disabled (record:=false)")

    return actions


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("bag_dir", default_value="",
                                        description="Output dir for the .mcap bag (timestamped subfolder). Default ./recordings or $SBL_BAG_DIR."))
    ld.add_action(DeclareLaunchArgument("record", default_value="true",
                                        description="Record a rosbag ('true'/'false')."))
    ld.add_action(DeclareLaunchArgument("bag_all", default_value="false",
                                        description="'true' records all topics; 'false' records only the SBL + solved-position topics."))
    ld.add_action(OpaqueFunction(function=_build_nodes))
    return ld
