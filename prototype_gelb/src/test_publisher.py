import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.joy_publisher = self.create_publisher(String,'/test_topic',10)
        # publish every 1.0 second
        self.timer = self.create_timer(1.0, self._timer_callback)

    def _timer_callback(self):
        self.publish_message('hello')

    def publish_message(self, msg):
        self.joy_publisher.publish(String(data=msg))


def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
