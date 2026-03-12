import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import numpy as np


class IceEstimation(Node):

    def __init__(self):
        super().__init__('ice_estimation')
        
        # Subscriber to mission data
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/mission_data',
            self.listener_callback,
            10
        )

        # Publisher for ice thickness
        self.publisher_ = self.create_publisher(
            Float32,
            '/ice_thickness',
            10
        )

        self.get_logger().info("Ice Thickness Node Started")
    
    def  ice_thickness(self, omega, pressure,  pitch, roll):
        
        rho_s = 300.0
        rho_water = 1000.0
        rho_ice = 917.0
        g = 9.81
        h_s = 0.2
        P_ext = 0.0

        # Convert pressure from bar to Pa
        P = (pressure - 1.0) * 100000.0

        # Water column height
        v = P / (rho_water * g)

        sensor_offset = 0.05
        v_druck = v - sensor_offset

        # Correct omega for pitch & roll
        omega_corr = omega * np.cos(np.deg2rad(pitch)) * np.cos(np.deg2rad(roll))

        # Validity mask
        if abs(pitch) > 25 or abs(roll) > 10:
            return float('nan')

        # Thickness formula
        T = (1.0 / rho_ice) * (
            ( v_druck- omega_corr) * rho_water
            - h_s * rho_s
            - P_ext / g
        )

        return float(T)



    def listener_callback(self, msg):

        # Extract data in correct order
        x = msg.data[0]
        y = msg.data[1]
        pressure = msg.data[2]
        omega = msg.data[3]
        roll = msg.data[4]
        pitch = msg.data[5]

        # Compute thickness
        thickness = self.ice_thickness(
            omega,
            pressure,
            pitch,
            roll
        )

        # Publish result
        out_msg = Float32()
        out_msg.data = thickness
        self.publisher_.publish(out_msg)


        self.get_logger().info(
            f"x={x:.1f}, y={y:.1f} -> Ice Thickness = {thickness:.3f} m"
        )

def main(args=None):
    rclpy.init(args=args)
    node = IceEstimation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()