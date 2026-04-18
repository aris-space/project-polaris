from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from config_pkg.constants import Ports

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
            # Node(
            #     package="usb_cam",
            #     executable="usb_cam_node_exe",
            #     name="camera_front",
            #     namespace="front",
            #     respawn=respawn,
            #     respawn_delay=respawn_delay,
            #     parameters=[
            #         {
            #             "video_device": Ports.USB_CAM_FRONT_PORT,
            #             "pixel_format": "mjpeg2rgb",  # The format that fixed the crash
            #             "image_width": 640,
            #             "image_height": 480,
            #         }
            #     ],
            # ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="camera_tube",
                namespace="tube",
                respawn=respawn,
                respawn_delay=respawn_delay,
                parameters=[
                    {
                        "video_device": '/dev/video0',
                        "pixel_format": "mjpeg2rgb",
                        "image_width": 640,
                        "image_height": 480,
                    }
                ],
            ),
        ]
    )
