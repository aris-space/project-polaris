import rclpy
from rclpy.node import Node # rclpy = ROS client library for python --> script becomes part of the ROS network

class SemiraNode(Node):
    def __init__(self):
        super().__init__('semira_node')
        self.get_logger().info(" The queen's ROS node started yeahhh")

def main(args= None):
    rclpy.init(args=args)
    node= SemiraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()   

