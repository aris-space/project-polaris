import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Helper to build the GStreamer string for Jetson Hardware Acceleration
    def get_gst_config(device_path):
        return (
            f"v4l2src device={device_path} do-timestamp=true ! "
            "image/jpeg, width=1280, height=720, framerate=30/1 ! "
            "nvv4l2decoder mjpeg=1 ! "
            "nvvidconv ! "
            "video/x-raw, format=BGRx ! "
            "videoconvert"
        )

    # Camera 1 Node
    cam_left = Node(
        package='gscam2',
        executable='gscam_main',
        name='gscam_front',
        namespace='front',
        parameters=[{
            'gscam_config': get_gst_config('/dev/video4'),
            'camera_name': 'front_camera',
            'frame_id': 'camera_front_link',
            'image_encoding': 'rgb8',
            'sync_sink': False # Helps with FPS stability
        }]
    )

    # Camera 2 Node
    cam_right = Node(
        package='gscam2',
        executable='gscam_main',
        name='gscam_top',
        namespace='top',
        parameters=[{
            'gscam_config': get_gst_config('/dev/video0'),
            'camera_name': 'top_camera',
            'frame_id': 'camera_top_link',
            'image_encoding': 'rgb8',
            'sync_sink': False
        }]
    )

    return LaunchDescription([
        cam_left,
        cam_right
    ])