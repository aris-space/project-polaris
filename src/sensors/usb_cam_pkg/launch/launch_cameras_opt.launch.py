import os
from launch import LaunchDescription
from launch_ros.actions import Node
from config_pkg.constants import Ports


def generate_launch_description():
    # Helper to build the GStreamer string for Jetson Hardware Acceleration
    def get_gst_config_tube(device_path):
        return (
            f"v4l2src device={device_path} do-timestamp=true ! "
            "image/jpeg, width=1280, height=720 ! "
            "nvv4l2decoder mjpeg=1 ! "
            "nvvidconv ! "
            "video/x-raw, format=BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=RGB"
        )

    def get_gst_config_front(device_path):
        return (
            f"v4l2src device={device_path} do-timestamp=true ! "
            "image/jpeg, width=1920, height=1080 ! "
            "nvv4l2decoder mjpeg=1 ! "
            "nvvidconv ! "
            "video/x-raw, format=BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=RGB"
        )

    # Camera 1 Node
    cam_front = Node(
        package="gscam",
        executable="gscam_node",
        name="gscam_front",
        namespace="front",
        parameters=[
            {
                "gscam_config": get_gst_config_front(Ports.USB_CAM_FRONT_PORT),
                "camera_name": "front_camera",
                "frame_id": "camera_front_link",
                "image_encoding": "rgb8",
                "sync_sink": False,  # Helps with FPS stability
            }
        ],
    )

    # Camera 2 Node
    cam_tube = Node(
        package="gscam",
        executable="gscam_node",
        name="gscam_tube",
        namespace="tube",
        parameters=[
            {
                "gscam_config": get_gst_config_tube(Ports.USB_CAM_TUBE_PORT),
                "camera_name": "tube_camera",
                "frame_id": "camera_tube_link",
                "image_encoding": "rgb8",
                "sync_sink": False,
            }
        ],
    )

    return LaunchDescription(
        [
            cam_front  ,
            cam_tube
        ]
    )
