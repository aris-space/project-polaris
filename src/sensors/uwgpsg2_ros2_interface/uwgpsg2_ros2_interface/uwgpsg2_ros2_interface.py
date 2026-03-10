#!/usr/bin/env python3

from scipy import rand
import rclpy
from rclpy.node import Node

#from std_msgs.msg import Int32
#from std_msgs.msg import Float32
#from std_msgs.msg import String
#from std_msgs.msg import Bool
#from sensor_msgs.msg import Image
#from sensor_msgs.msg import Joy
#from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geographic_msgs.msg import GeoPointStamped
from auv_msgs.msg import NavigationStatus
from ublox_msgs.msg import NavRELPOSNED9

import math
import sys
#import os
#import inputs

import requests
#import argparse
#import json
import time
import math
import requests
from requests.structures import CaseInsensitiveDict
from termcolor import colored
#import random
from scipy.spatial.transform import Rotation as rotation
import pymap3d
import numpy as np


# sys.path.append('~/waterlinked-uwgpsg2-ros2-pkg/uwgpsg2_ros2_interface/uwgpsg2_ros2_interface/examples')
#from getposition import get_data, get_acoustic_position, get_global_position
#from . import get_data, get_acoustic_position, get_global_position

class WaterLinkedUWGPSG2Interface(Node):

    # WaterLinked API methods
    def get_data(self, url):
        try:
            r = requests.get(url)
        except requests.exceptions.RequestException as exc:
            print(colored("Exception occured {}".format(exc), "red"))
            return None

        if r.status_code != requests.codes.ok:
            print("Got error {}: {}. Problems getting relative/global Locator position from the topside".format(r.status_code, r.text))
            return None

        return r.json()

    def get_acoustic_position(self, base_url):
        return self.get_data("{}/api/v1/position/acoustic/filtered".format(base_url))
             
    def get_global_position(self, base_url):
        return self.get_data("{}/api/v1/position/global".format(base_url))

    # ROS-related methods
    def declare_node_parameters(self):
        #self.declare_parameter('use_gamepad', False)
        #self.declare_parameter('camera_bitrate_min', 1000000)
        #self.declare_parameter('camera_resolution_values', [480, 720, 1080])
        #self.declare_parameter('velocity_force_min', -1.0)
        print("Declaring ROS parameters")
        # ROS params
        self.declare_parameter('ros_rate', 2.0)
        self.declare_parameter('ros_rate_topside', 10.0)
        self.declare_parameter('waterlinked_url', '')
        self.declare_parameter('use_ros_based_frame_transform', True)
        self.declare_parameter('waterlinked_api_external_master_path', '')
        self.declare_parameter('wl_api_use_external_gps_fixed', False)
        self.declare_parameter('external_gps_fixed_lat', 0.0)
        self.declare_parameter('external_gps_fixed_lon', 0.0)
        self.declare_parameter('wl_api_use_external_gps_measurements', False)
        # self.declare_parameter('external_gps_measurements_topic', '')
        # self.declare_parameter('external_heading_measurements_topic', '')
        # self.declare_parameter('external_ned_measurements_topic', '')
        # self.declare_parameter('external_map_origin_measurements_topic', '')
        self.declare_parameter('wl_api_use_external_heading_fixed', False)
        self.declare_parameter('external_heading_fixed_value', 0.0)
        self.declare_parameter(
            'wl_api_use_external_heading_measurements', False)
        # self.declare_parameter('external_imu_measurements_topic', '')
        # self.declare_parameter( 'external_navigation_status_measurements_topic', '')
        self.declare_parameter(
            'use_ros_based_locator_relative_position', False)
        # self.declare_parameter('external_locator_relative_position_topic', '')

    def get_ros_params(self):
        # Setting ROS parameters
        # self.IS_SIMULATION = self.get_parameter(
        #    'is_simulation').get_parameter_value().bool_value
        # self.CAMERA_BITRATE_MIN = self.get_parameter(
        #    'camera_bitrate_min').get_parameter_value().integer_value
        # self.CAMERA_FRAMERATE_VALUES = self.get_parameter(
        #    'camera_framerate_values').get_parameter_value().integer_array_value
        print("Getting ROS parameters.")
        self.RATE = self.get_parameter(
            'ros_rate').get_parameter_value().double_value
        self.RATE_TOPSIDE = self.get_parameter(
            'ros_rate_topside').get_parameter_value().double_value
        self.WATERLINKED_URL = self.get_parameter(
            'waterlinked_url').get_parameter_value().string_value
        self.USE_ROS_BASED_FRAME_TRANSFORM = self.get_parameter(
            'use_ros_based_frame_transform').get_parameter_value().bool_value
        self.WATERLINKED_API_EXTERNAL_MASTER_PATH = self.get_parameter(
            'waterlinked_api_external_master_path').get_parameter_value().string_value
        self.WL_API_USE_EXTERNAL_GPS_FIXED = self.get_parameter(
            'wl_api_use_external_gps_fixed').get_parameter_value().bool_value
        self.EXTERNAL_GPS_FIXED_LAT_VALUE = self.get_parameter(
            'external_gps_fixed_lat').get_parameter_value().double_value
        self.EXTERNAL_GPS_FIXED_LON_VALUE = self.get_parameter(
            'external_gps_fixed_lon').get_parameter_value().double_value
        self.WL_API_USE_EXTERNAL_GPS_MEASUREMENTS = self.get_parameter(
            'wl_api_use_external_gps_measurements').get_parameter_value().bool_value
        # self.EXTERNAL_GPS_MEASUREMENTS_TOPIC = self.get_parameter(
        #     'external_gps_measurements_topic').get_parameter_value().string_value
        # self.EXTERNAL_HEADING_MEASUREMENTS_TOPIC = self.get_parameter(
        #     'external_heading_measurements_topic').get_parameter_value().string_value    
        # self.EXTERNAL_NED_MEASUREMENTS_TOPIC = self.get_parameter(
        #     'external_ned_measurements_topic').get_parameter_value().string_value
        # self.EXTERNAL_MAP_ORIGIN_MEASUREMENTS_TOPIC = self.get_parameter(
        #     'external_map_origin_measurements_topic').get_parameter_value().string_value
        self.WL_API_USE_EXTERNAL_HEADING_FIXED = self.get_parameter(
            'wl_api_use_external_heading_fixed').get_parameter_value().bool_value
        self.EXTERNAL_HEADING_FIXED_VALUE = self.get_parameter(
            'external_heading_fixed_value').get_parameter_value().double_value
        self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS = self.get_parameter(
            'wl_api_use_external_heading_measurements').get_parameter_value().bool_value
        # self.EXTERNAL_IMU_MEASUREMENTS_TOPIC = self.get_parameter(
        #     'external_imu_measurements_topic').get_parameter_value().string_value
        # self.EXTERNAL_NAVIGATION_STATUS_MEASUREMENTS_TOPIC = self.get_parameter(
        #     'external_navigation_status_measurements_topic').get_parameter_value().string_value
        self.USE_ROS_BASED_LOCATOR_RELATIVE_POSITION = self.get_parameter(
            'use_ros_based_locator_relative_position').get_parameter_value().bool_value
        # self.EXTERNAL_LOCATOR_RELATIVE_POSITION_TOPIC = self.get_parameter(
        #     'external_locator_relative_position_topic').get_parameter_value().string_value

        # print(self.WATERLINKED_URL)

    def set_ros_params(self):
        return

    def initialize_timer(self):
        print("Initializing timer")
        self.timer_period = 1.0/self.RATE
        self.timer = self.create_timer(
            self.timer_period, self.timer_callback)

    def timer_callback(self):
        self.set_ros_params()
        if not self.USE_ROS_BASED_FRAME_TRANSFORM:
            self.get_waterlinked_measuremets_global()
        if not self.USE_ROS_BASED_LOCATOR_RELATIVE_POSITION:
            self.get_waterlinked_measuremets_relative()    
        
        self.transform_relative_to_ned_position()
        self.transform_ned_to_global_position()
        self.publish_all_waterlinked_variables()    

        """msg = NavSatFix()
        msg.latitude = 43.0 + rand()*2.0
        msg.longitude = 16.0  + rand()*3.0
        self.fake_gps_pub.publish(msg)
        msg = NavRELPOSNED9()
        msg.rel_pos_heading = int((45+rand())*100000)
        self.fake_heading_pub.publish(msg)"""

    def get_waterlinked_measuremets_relative(self):
        time_ = time.time()
        time_nanosec, time_sec = math.modf(time_)
        self.locator_rel_pos_time_new = time_sec + time_nanosec
        
        data = self.get_acoustic_position(self.WATERLINKED_URL)
        if data:
            self.locator_wrt_base_relative_x = data["x"]
            self.locator_wrt_base_relative_y = data["y"]
            self.locator_wrt_base_relative_z = data["z"]

    def get_waterlinked_measuremets_global(self):
        pos = self.get_global_position(self.WATERLINKED_URL)
        if (pos and not self.USE_ROS_BASED_FRAME_TRANSFORM):
            self.locator_global_lat = pos["lat"]
            self.locator_global_lon = pos["lon"]

    def transform_relative_to_ned_position(self):
        if self.USE_ROS_BASED_FRAME_TRANSFORM:
            if (hasattr(self, 'topside_external_pos_north') and hasattr(self, 'topside_external_pos_east') and
                hasattr(self, 'topside_external_pos_down') and hasattr(self, 'topside_external_heading_rad') and
                hasattr(self, 'topside_external_pitch_rad') and hasattr(self, 'topside_external_roll_rad') and
                hasattr(self, 'locator_wrt_base_relative_x') and hasattr(self, 'locator_wrt_base_relative_y') and
                    hasattr(self, 'locator_wrt_base_relative_z')):
                topside_pos_ned = np.array([self.topside_external_pos_north,
                                            self.topside_external_pos_east,
                                            self.topside_external_pos_down])

                #quat = [x, y, z, w]
                #R = rotation.from_quat(quat)
                euler = np.array([self.topside_external_heading_rad,  # CHECK !
                                self.topside_external_pitch_rad, self.topside_external_roll_rad])
                R = rotation.from_euler('zyx', euler, degrees=False)
                pos_relative = np.array([self.locator_wrt_base_relative_x,
                                        self.locator_wrt_base_relative_y,
                                        self.locator_wrt_base_relative_z])
                self.locator_pos_ned = R.apply(pos_relative) + topside_pos_ned

            else:
                print(
                    colored("Transformation to topside NED frame lacking arguments!", "red"))

    def transform_ned_to_global_position(self):
        if self.USE_ROS_BASED_FRAME_TRANSFORM:
            if (hasattr(self, 'locator_pos_ned') and hasattr(self, 'topside_external_origin_lat') and
                    hasattr(self, 'topside_external_origin_lon') and hasattr(self, 'topside_external_origin_h')):
                n = self.locator_pos_ned[0]
                e = self.locator_pos_ned[1]
                d = self.locator_pos_ned[2]
                lat0 = self.topside_external_origin_lat
                lon0 = self.topside_external_origin_lon
                h0 = self.topside_external_origin_h
                lat, lon, h = pymap3d.ned2geodetic(
                    n, e, d, lat0, lon0, h0, ell=None, deg=True)
                self.locator_global_lat = lat
                self.locator_global_lon = lon
            else:
                print(colored(
                    "Transformation from topside NED frame to WGS84 frame lacking arguments!", "red"))

    def pub_locator_wrt_base_relative_pos(self):
        if hasattr(self, 'locator_wrt_base_relative_x') and not self.USE_ROS_BASED_LOCATOR_RELATIVE_POSITION:
            msg = Vector3Stamped()
            time_ = time.time()
            time_nanosec, time_sec = math.modf(time_)
            time_sec = int(time_sec)
            time_nanosec = int(1e9*time_nanosec)
            msg.header.stamp.sec = time_sec
            msg.header.stamp.nanosec = time_nanosec
            msg.vector.x = float(self.locator_wrt_base_relative_x)
            msg.vector.y = float(self.locator_wrt_base_relative_y)
            msg.vector.z = float(self.locator_wrt_base_relative_z)
            self.pos_relative_wrt_topside.publish(msg)

    def pub_locator_global_pos(self):
        if hasattr(self, 'locator_global_lat'):
            msg = GeoPointStamped()
            time_ = time.time()
            time_nanosec, time_sec = math.modf(time_)
            time_sec = int(time_sec)
            time_nanosec = int(1e9*time_nanosec)  # +2**32
            msg.header.stamp.sec = time_sec
            msg.header.stamp.nanosec = time_nanosec
            msg.position.latitude = float(self.locator_global_lat)
            msg.position.longitude = float(self.locator_global_lon)
            # or -self.locator_wrt_base_relative_z
            msg.position.altitude = -float(self.locator_wrt_base_relative_z)
            self.gps_pub.publish(msg)

    def pub_locator_pos_ned(self):
        if hasattr(self, 'locator_pos_ned'):
            msg = Vector3Stamped()
            time_ = time.time()
            time_nanosec, time_sec = math.modf(time_)
            time_sec = int(time_sec)
            time_nanosec = int(1e9*time_nanosec)
            msg.header.stamp.sec = time_sec
            msg.header.stamp.nanosec = time_nanosec
            msg.vector.x = self.locator_pos_ned[0]
            msg.vector.y = self.locator_pos_ned[1]
            msg.vector.z = self.locator_pos_ned[2]
            self.ned_pub.publish(msg)

    def publish_all_waterlinked_variables(self):
        time_ = time.time()
        time_nanosec, time_sec = math.modf(time_)
        self.time_pub = time_sec + time_nanosec
        #if hasattr(self, 'topside_pos_time_new') and hasattr(self, 'locator_rel_pos_time_new'):
        if hasattr(self, 'locator_rel_pos_time_new'):
            #delta_time_topside = self.time_pub - self.topside_pos_time_new
            delta_time_locator = self.time_pub - self.locator_rel_pos_time_new
            #print(delta_time_topside)
            #if (delta_time_topside<=3/self.RATE_TOPSIDE and delta_time_locator <= 3/self.RATE):
            #if (delta_time_topside<=3/self.RATE_TOPSIDE and delta_time_locator <= 3/self.RATE):    
            if (delta_time_locator <= 3/self.RATE):     
                self.pub_locator_wrt_base_relative_pos()
                self.pub_locator_pos_ned()
                self.pub_locator_global_pos()
            #if delta_time_topside>3/self.RATE_TOPSIDE:
            #    print(colored("Topside NED/GPS data timedout!", "red"))
            if delta_time_locator>3/self.RATE:
                print(colored("Locator data timedout!", "red"))

    def euler_to_quaternion(self, roll, pitch, yaw):  # yaw (Z), pitch (Y), roll (X)
        # Abbreviations for the various angular functions
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return x, y, z, w

    def quaternion2euler(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

    def sendHttpPutRequestToTopsideAsExternalMaster(self):
        url = self.WATERLINKED_URL + self.WATERLINKED_API_EXTERNAL_MASTER_PATH
        headers = CaseInsensitiveDict()
        headers["accept"] = "application/vnd.waterlinked.operation_response+json"
        headers["Content-Type"] = "application/json"
        data = dict(lat=self.topside_external_lat_deg_dec,
                    lon=self.topside_external_lon_deg_dec,
                    orientation=self.topside_external_heading_deg
                    )
        try:            
            resp = requests.put(url, json=data, timeout=1.0/self.RATE)
        except requests.exceptions.RequestException as exc:
            print(colored("Exception occured {}".format(exc), "red"))

    def external_ned_measurements_callback(self, msg):
        """
        # Assuming Vector3Stamped NED coordinates msgs
        self.topside_external_pos_north = 0.0
        self.topside_external_pos_east = 0.0
        self.topside_external_pos_down = 0.0 
        """
        return 0

    def external_map_origin_measurements_callback(self, msg):
        """
        # Assuming GeoPointStamped NED origin msgs
        self.topside_external_origin_lat = 0.0
        self.topside_external_origin_lon = 0.0
        self.topside_external_origin_h = 0.0 
        """
        return 0

    def external_gps_measurements_callback(self, msg):
        # Parse topside's lat-lon coordinates 
        self.topside_external_lat_deg_dec = msg.latitude
        self.topside_external_lon_deg_dec = msg.longitude     

        if (self.WL_API_USE_EXTERNAL_GPS_MEASUREMENTS and self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS
                and hasattr(self, 'topside_external_lat_deg_dec') 
                and hasattr(self, 'topside_external_lon_deg_dec') and hasattr(self, 'topside_external_heading_deg')):
            self.sendHttpPutRequestToTopsideAsExternalMaster() 

    def external_heading_measurements_callback(self, msg):
        # Parse topside's heading
        self.topside_external_heading_deg = msg.rel_pos_heading/100000.0; # degrees

        if (self.WL_API_USE_EXTERNAL_GPS_MEASUREMENTS and self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS
                and hasattr(self, 'topside_external_lat_deg_dec') 
                and hasattr(self, 'topside_external_lon_deg_dec') and hasattr(self, 'topside_external_heading_deg')):
            self.sendHttpPutRequestToTopsideAsExternalMaster() 

    def external_navigation_status_measurements_callback(self, msg):
        time_ = time.time()
        time_nanosec, time_sec = math.modf(time_)
        self.topside_pos_time_new = time_sec + time_nanosec
                
        # Assuming NavigationStatus msgs
        # Parse local NED origin lat-lon coordinates
        self.topside_external_origin_lat = msg.origin.latitude
        self.topside_external_origin_lon = msg.origin.longitude
        self.topside_external_origin_h = msg.origin.altitude

        # Parse topside's lat-lon coordinates
        self.topside_external_lat_deg_dec = msg.global_position.latitude
        self.topside_external_lon_deg_dec = msg.global_position.longitude

        # Parse topside's NED frame coordinates
        self.topside_external_pos_north = msg.position.north
        self.topside_external_pos_east = msg.position.east
        self.topside_external_pos_down = msg.position.depth

        # Orientation and orientation rate are in radians and radians/sec using RPY
        self.topside_external_roll_rad = msg.orientation.x
        self.topside_external_pitch_rad = msg.orientation.y
        self.topside_external_heading_rad = msg.orientation.z

        self.topside_external_heading_deg = self.topside_external_roll_rad * \
            180.0/math.pi  # degrees
        if (self.WL_API_USE_EXTERNAL_GPS_MEASUREMENTS and self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS
                and not self.USE_ROS_BASED_FRAME_TRANSFORM):
            self.sendHttpPutRequestToTopsideAsExternalMaster()        

    def external_locator_relative_position_callback(self, msg):
        time_ = time.time()
        time_nanosec, time_sec = math.modf(time_)
        self.locator_rel_pos_time_new = time_sec + time_nanosec
        
        self.locator_wrt_base_relative_x = msg.vector.x
        self.locator_wrt_base_relative_y = msg.vector.y
        self.locator_wrt_base_relative_z = msg.vector.z

    def initialize_subscribers(self):
        print("Initializing ROS subscribers")
        if (self.USE_ROS_BASED_FRAME_TRANSFORM or
                (self.WL_API_USE_EXTERNAL_GPS_MEASUREMENTS and self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS)):
            """self.create_subscription(
                NavSatFix, self.EXTERNAL_GPS_MEASUREMENTS_TOPIC, self.external_gps_measurements_callback, 10)                
            self.create_subscription(
                Vector3Stamped, self.EXTERNAL_NED_MEASUREMENTS_TOPIC, self.external_ned_measurements_callback, 10)
            self.create_subscription(
                GeoPointStamped, self.EXTERNAL_MAP_ORIGIN_MEASUREMENTS_TOPIC, self.external_map_origin_measurements_callback, 10)"""
            # Subscribers to external ublox GPS and heading measurements
            self.create_subscription(
                NavSatFix, 'fix', self.external_gps_measurements_callback, 10)
            self.create_subscription(
                NavRELPOSNED9, 'navrelposned', self.external_heading_measurements_callback, 10)

            #self.create_subscription(
            #    NavigationStatus, self.EXTERNAL_NAVIGATION_STATUS_MEASUREMENTS_TOPIC, self.external_navigation_status_measurements_callback, 10)

        # if (self.USE_ROS_BASED_FRAME_TRANSFORM or self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS):
        #    self.create_subscription(
        #        Imu, self.EXTERNAL_IMU_MEASUREMENTS_TOPIC, self.external_imu_measurements_callback, 10)

        if (self.USE_ROS_BASED_LOCATOR_RELATIVE_POSITION):
            self.create_subscription(
                Vector3Stamped, 'locator_position_relative_wrt_topside', self.external_locator_relative_position_callback, 10)

    def initialize_publishers(self):
        print("Initializing ROS publishers")
        if not self.USE_ROS_BASED_LOCATOR_RELATIVE_POSITION:
            self.pos_relative_wrt_topside = self.create_publisher(
                Vector3Stamped, "locator_position_relative_wrt_topside", 10)
        self.gps_pub = self.create_publisher(
            GeoPointStamped, "locator_position_global", 10)
        self.ned_pub = self.create_publisher(
            Vector3Stamped, "locator_position_topside_ned", 10)

        """self.fake_gps_pub = self.create_publisher(NavSatFix, "/fix", 10)
        self.fake_heading_pub = self.create_publisher(NavRELPOSNED9, "/navrelposned", 10)"""

    def run_tests(self):
        # Test external GPS+heading measurements HTTP request sending
        if (self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS or
                self.WL_API_USE_EXTERNAL_GPS_MEASUREMENTS):
            print("Testing WaterLinked API external GPS+heading PUT requests")
            for iter in range(5):
                self.topside_external_lat_deg_dec = iter*10
                self.topside_external_lon_deg_dec_dec = iter*10
                self.topside_external_heading_deg = iter*360/5
                url = self.WATERLINKED_URL + self.WATERLINKED_API_EXTERNAL_MASTER_PATH
                headers = CaseInsensitiveDict()
                headers["accept"] = "application/vnd.waterlinked.operation_response+json"
                headers["Content-Type"] = "application/json"
                data = dict(lat=self.topside_external_lat_deg_dec,
                            lon=self.topside_external_lon_deg_dec_dec, orientation=self.topside_external_heading_deg)
                print(data)
                try:
                    resp = requests.put(url, json=data, timeout=1.0/self.RATE)
                    print(resp.status_code)
                    print(resp.reason)
                except requests.exceptions.RequestException as exc:
                    print(colored("Exception occured {}".format(exc), "red"))
                time.sleep(2)

        # Test frame transforms
        n = 10000.0*np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        e = n
        d = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        r = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        p = r
        y = np.array([0.0, math.pi/4.0, math.pi/2.0, math.pi, math.pi*5/4.0, math.pi *
                      3/2.0, 0.0, math.pi/4.0, math.pi/2.0, math.pi, math.pi*5/4.0, math.pi*3/2.0])
        lat0 = np.array([43.0, 43.0, 43.0, 43.0, 43.0, 43.0,
                         43.0, 43.0, 43.0, 43.0, 43.0, 43.0])
        lon0 = lat0/43.0*16.0
        x_rel = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                          1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        y_rel = x_rel
        z_rel = x_rel*5.0
        for iter in range(len(n)):
            self.topside_external_pos_north = n[iter]
            self.topside_external_pos_east = e[iter]
            self.topside_external_pos_down = d[iter]

            self.topside_external_roll_rad = r[iter]
            self.topside_external_pitch_rad = p[iter]
            self.topside_external_heading_rad = y[iter]

            self.topside_external_origin_lat = lat0[iter]
            self.topside_external_origin_lon = lon0[iter]
            self.topside_external_origin_h = 0.0

            self.locator_wrt_base_relative_x = x_rel[iter]
            self.locator_wrt_base_relative_y = y_rel[iter]
            self.locator_wrt_base_relative_z = z_rel[iter]

            self.transform_relative_to_ned_position()
            self.transform_ned_to_global_position()
            print(iter+1)
            print(self.locator_pos_ned)
            print(self.locator_global_lat)
            print(self.locator_global_lon)

    def initialize_properties(self):
        # Check the validity of params
        if (not self.USE_ROS_BASED_FRAME_TRANSFORM):
            if not (self.WL_API_USE_EXTERNAL_GPS_FIXED ^ self.WL_API_USE_EXTERNAL_GPS_MEASUREMENTS):
                print(colored("WL_API_USE_EXTERNAL_GPS_FIXED and WL_API_USE_EXTERNAL_GPS_MEASUREMENTS must not have the same value!", "red"))

            if not (self.WL_API_USE_EXTERNAL_HEADING_FIXED ^ self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS):
                print(colored(
                    "WL_API_USE_EXTERNAL_HEADING_FIXED and USE_EXTERNAL_HEADING_ASV_MEASUREMENTS must not have the same value!", "red"))

            if (self.WL_API_USE_EXTERNAL_GPS_FIXED and self.WL_API_USE_EXTERNAL_HEADING_FIXED):
                self.topside_external_lat_deg_dec = self.EXTERNAL_GPS_FIXED_LAT_VALUE
                self.topside_external_lon_deg_dec_dec = self.EXTERNAL_GPS_FIXED_LON_VALUE
                self.topside_external_heading_deg = self.EXTERNAL_HEADING_FIXED_VALUE
                self.sendHttpPutRequestToTopsideAsExternalMaster()

        if (self.USE_ROS_BASED_FRAME_TRANSFORM and
            (self.WL_API_USE_EXTERNAL_GPS_FIXED or self.WL_API_USE_EXTERNAL_GPS_MEASUREMENTS or
             self.WL_API_USE_EXTERNAL_HEADING_FIXED or self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS)):
            print(colored(
                "When USE_ROS_BASED_FRAME_TRANSFORM is True then all other WL_API* booleans must be False!", "red"))

        if (not (self.USE_ROS_BASED_FRAME_TRANSFORM or
                 self.WL_API_USE_EXTERNAL_GPS_FIXED or self.WL_API_USE_EXTERNAL_GPS_MEASUREMENTS or
                 self.WL_API_USE_EXTERNAL_HEADING_FIXED or self.WL_API_USE_EXTERNAL_HEADING_MEASUREMENTS)):
            print(colored(
                "Source of external IMU/GPS measurements must be used, either ROS or WL_API_FIXED/MEASUREMENTS!", "red"))

    def __init__(self):
        print("Initializing WaterLinkedUWGPSG2Interface class instance.")
        super().__init__('uwgpsg2_interface')
        self.declare_node_parameters()
        self.get_ros_params()
        self.initialize_properties()
        self.initialize_timer()
        self.initialize_subscribers()
        self.initialize_publishers()

        debug = False
        if debug:
            self.run_tests()

def main(args=None):
    print("Started")
    rclpy.init(args=args)

    try:
        interface = WaterLinkedUWGPSG2Interface()
        rclpy.spin(interface)
        # interface.run()
    except:
        print(colored("Exception caught!", "red"))
        e = sys.exc_info()[0]
        write_to_page("<p>Error: %s</p>" % e)
        pass

    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
