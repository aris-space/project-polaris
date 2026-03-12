import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray 
from std_msgs.msg import Float64
import random

class SubmarineValues(Node):

    def __init__(self):
        super().__init__('submarine_values')

        self.pressure_sub = self.create_subscription(
            Float64,
            '/pixhawk/scaled_pressure', 
            self.pressure_callback,
            10
        )


        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/mission_data',
            10
        )

        self.timer = self.create_timer(0.5, self.timer_callback)  # 2 Hz
        self.t = 0.0
        self.hz = 2.0
        self.area_size = 100.0
        self.lane_width =5.0
        self.x= 0.0
        self.y =0.0
        self.direction = 1.0      # 1 = move right, -1 = move left
        self.speed = 0.5        # meters per timer tick
        self.current_pressure_pa = 101325.0  # Standaconcordwert (1 atm in Pascal)
    
    def pressure_callback(self, msg):
        self.current_pressure_pa = msg.data

    def timer_callback(self):

        if self.direction == 1:
            self.x += self.speed
            if self.x >= self.area_size:
                self.x = self.area_size
                self.y += self.lane_width
                self.direction = -1
        else:
            self.x -= self.speed
            if self.x <= 0.0:
                self.x = 0.0
                self.y += self.lane_width
                self.direction = 1

        # Stop mission if area fully covered
        if self.y > self.area_size:
            self.get_logger().info("Mission completed.")
            return

        roll = random.uniform(-1.2,1.2)
        pitch = random.uniform(-0.8,0.8)
        omega= 0.6 + random.uniform(-0.02,0.02)
        pressure = self.current_pressure_pa / 100000.0

        msg = Float64MultiArray()
        msg.data = [
            float(self.x),
            float(self.y),
            float(pressure),
            float(omega),
            float(roll),
            float(pitch)
        ]

        self.publisher_.publish(msg)
        self.t += 1.0 / self.hz

def main(args=None):
    rclpy.init(args=args)
    node = SubmarineValues()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

