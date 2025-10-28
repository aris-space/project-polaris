import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.joy_subscriber = self.create_subscription(
            String,
            '/test_topic',
            self.test_callback,
            10
        )

    def test_callback(self, msg):
        print(f"Received message: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

