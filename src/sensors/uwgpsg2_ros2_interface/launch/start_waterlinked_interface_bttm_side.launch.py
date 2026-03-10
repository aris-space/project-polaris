import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.conditions import IfCondition


def generate_launch_description():
    ld = LaunchDescription()
      
    
    waterlinked_node_params = os.path.join(
        get_package_share_directory('uwgpsg2_ros2_interface'),
        'config',
        'waterlinked_node_params.yaml'
        )
        
    #gstreamer_remappings = [('image_raw', 'camera_img_raw_topside_stream'),
    #              ('camera_info', 'camera_calibration_params_topside_stream')]
    
    waterlinked_localization_node = Node (
            namespace='korkyra/uwgps',
            name='waterlinked_localization_node',
            package='uwgpsg2_ros2_interface',
            executable='uwgpsg2_ros2_interface',
            output='screen',
            emulate_tty=True,
            parameters= [waterlinked_node_params],
            #remappings=gstreamer_remappings,
            remappings =[   ('fix', '/korkyra/fix'), # ? this maybe needs to be changed to /korkyra/fix ?
                            ('navrelposned', '/korkyra/navrelposned'),
                            ('locator_position_relative_wrt_topside', 'locator_position_relative_wrt_topside'),
                            ('locator_position_global', 'locator_position_global'),
                            ('locator_position_topside_ned', 'locator_position_topside_ned')
                        ],
            arguments=[]    
        )
       
    ###################################################################
    
    #pkg_dir = get_package_share_directory('blueye_ros2_interface')    
    #launch_top_side = IncludeLaunchDescription( \
    #    PythonLaunchDescriptionSource( pkg_dir + \
    #    '/launch/start_blueye_interface_top_side.launch.py'))   
    #ld.add_action(launch_top_side)
    
    ld.add_action(waterlinked_localization_node)    
    return ld

