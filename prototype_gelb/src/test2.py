import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def joy_callback(self, msg):
        print(f"Read r2 value: {msg.axes[5]}")
        print("")
        print(f"Joystick inputs: {msg.axes}")

def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
