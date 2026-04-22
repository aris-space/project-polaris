import rclpy
from rclpy.node import Node
from dalybms import DalyBMS


class BMSNode(Node):
    def __init__(self):
        super().__init__("bms_node")
        self.get_logger().info("BMS node started")
        self.bms = DalyBMS() # check device address
        self.bms.connect("dev/bms")
        



def main(args=None):
    rclpy.init(args=args)
    node = BMSNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
