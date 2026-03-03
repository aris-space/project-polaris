from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="camera_front",
                namespace="front",
                parameters=[
                    {
                        "video_device": "/dev/cam_front",
                        "pixel_format": "mjpeg2rgb",  # The format that fixed the crash
                        "image_width": 640,
                        "image_height": 480,
                    }
                ],
            ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="camera_tube",
                namespace="tube",
                parameters=[
                    {
                        "video_device": "/dev/cam_tube",
                        "pixel_format": "mjpeg2rgb",
                        "image_width": 640,
                        "image_height": 480,
                    }
                ],
            ),
        ]
    )
