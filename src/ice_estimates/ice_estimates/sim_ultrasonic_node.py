import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import random
import time

class SimulationUltrasonicNode(Node):

    def __init__(self):
        super().__init__('simulation_ultrasonic_node')

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/ultrasonic_raw_data',
            10
        )
        
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('Ultrasonic Raw Data Simulation gestartet...')

    def timer_callback(self):

        t1 = time.time() 
        delay = 0.00028 + random.uniform(-0.00005, 0.00005)
        t2 = t1 + delay
        temperatur = -15.0 + random.uniform(-2.0, 2.0) 
        salzgehalt = 5.0 + random.uniform(-0.5, 0.5)
        roll = random.uniform(-1.2,1.2)
        pitch = random.uniform(-0.8,0.8)

        msg = Float64MultiArray()
        msg.data = [
            float(t1),          
            float(t2),          
            float(temperatur),  
            float(salzgehalt),
            float(roll),
            float(pitch)  
        ]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Sende Rohdaten: t1={t1:.4f}, t2={t2:.4f}, Temp={temperatur:.1f}, Salz={salzgehalt:.1f}')

def main(args=None):
    rclpy.init(args=args)
    node = SimulationUltrasonicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()