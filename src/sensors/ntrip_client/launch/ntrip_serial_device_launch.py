from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
      return LaunchDescription([
          # Declare arguments with default values
          DeclareLaunchArgument('namespace',             default_value='/'),
          DeclareLaunchArgument('node_name',             default_value='ntrip_serial_device'),
          DeclareLaunchArgument('debug',                 default_value='false'),
          DeclareLaunchArgument('port',                  default_value='/dev/ttyACM0'),
          DeclareLaunchArgument('baudrate',              default_value='115200'),
          DeclareLaunchArgument('rtcm_message_package',  default_value='rtcm_msgs'),

          # Pass an environment variable to the node
          SetEnvironmentVariable(name='NTRIP_CLIENT_DEBUG', value=LaunchConfiguration('debug')),

          # ******************************************************************
          # NTRIP Client Node
          # ******************************************************************
          Node(
                name=LaunchConfiguration('node_name'),
                namespace=LaunchConfiguration('namespace'),
                package='ntrip_client',
                executable='ntrip_serial_device_ros.py',
                parameters=[
                  {
                    # Required parameters used to connect to the NTRIP device
                    'port': LaunchConfiguration('port'),
                    'baudrate': LaunchConfiguration('baudrate'),

                    # Not sure if this will be looked at by other ndoes, but this frame ID will be added to the RTCM messages published by this node
                    'rtcm_frame_id': 'odom',

                    # Optional parameters that will allow for longer or shorter NMEA messages. Standard max length for NMEA is 82
                    'nmea_max_length': 128,
                    'nmea_min_length': 3,

                    # Use this parameter to change the type of RTCM message published by the node. Defaults to "mavros_msgs", but we also support "rtcm_msgs"
                    'rtcm_message_package': LaunchConfiguration('rtcm_message_package'),

                    # Will affect how many times the node will attempt to reconnect before exiting, and how long it will wait in between attempts when a reconnect occurs
                    'reconnect_attempt_max': 10,
                    'reconnect_attempt_wait_seconds': 5,
                  }
                ],
                # Uncomment the following section and replace "/gx5/nmea/sentence" with the topic you are sending NMEA on if it is not the one we requested
                #remappings=[
                #  ("nmea", "/gx5/nmea/sentence")
                #],
          )
      ])
