import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
import numpy as np


class UltraIceEstimation(Node):

    def __init__(self):
        super().__init__('ultra_ice_estimation')
        
        # Subscriber to mission data
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/ultrasonic_raw_data',
            self.listener_callback,
            10
        )

        # Publisher for ice thickness
        self.publisher_ = self.create_publisher(
            Float64,
            '/ice_ultra_thickness',
            10
        )

        self.get_logger().info("Ice Ultra Thickness Node Started")
    
    def  ice_ultra_thickness(self, t1, t2, salz, temp ,pitch, roll):
        delta_t = t2-t1
        c_ice = 3500.0 + (temp*0.6)-(salz*0.3)
        T =(c_ice*delta_t)/2.0
        T_corr= T * np.cos(np.deg2rad(pitch)) * np.cos(np.deg2rad(roll))

        return float(T_corr)


    def listener_callback(self, msg):

        # Extract data in correct order
        t1 = msg.data[0]
        t2 = msg.data[1]
        temp = msg.data[2]
        salz = msg.data[3]
        roll = msg.data[4]
        pitch = msg.data[5]

        # Compute thickness
        thickness = self.ice_ultra_thickness(t1, t2, salz, temp, pitch, roll)

        # Publish result
        out_msg = Float64()
        out_msg.data = thickness
        self.publisher_.publish(out_msg)

        # Ergebnis veröffentlichen
        res_msg = Float64()
        res_msg.data = thickness
        self.publisher_.publish(res_msg)

        self.get_logger().info(f'Berechnete Dicke: {thickness:.4f} m (dt: {(t2-t1):.6f}s)')

def main(args=None):
    rclpy.init(args=args)
    node = UltraIceEstimation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()