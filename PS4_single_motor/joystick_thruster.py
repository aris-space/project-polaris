import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32


'''Example of a simple skeleton ROS2 node'''
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')

'''Example of running the simple skeleton ROS2 node'''
def main_example(args=None):
    rclpy.init(args=args)   # Initialize ROS2
    node = MyNode()         # Create an instance of MyNode
    rclpy.spin(node)        # Keep the node running
    node.destroy_node()     # Clean up the node, when done
    rclpy.shutdown()        # Shutdown ROS2


class PS4toThrust(Node):
    '''
    A Class for creating a ROS2 node that converts PS4 controller R2 axis input to thrust commands.
    It subscribes to the '/joy' topic to receive joystick inputs and publishes
    the thrust commands to the '/thrust_command' topic.
    '''

    def __init__(self):
        super().__init__('ps4_to_thrust')
        self.thrust_publisher = self.create_publisher(Float32, '/thrust_command', 10)
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def joy_callback(self, msg):

        R2_input = msg.axes[5]                            # R2 axis value
        thrust_value = ((-R2_input + 1) / 2) * 100        # Map R2 input to thrust value [0,100]
        msg_out = Float32()                               # Create empty Float32 message
        msg_out.data = thrust_value                       # Assign mapped thrust value to  empty message
        self.thrust_publisher.publish(msg_out)            # Publish the thrust command



def main(args=None):
    '''Main function to run the PS4 to Thrust ROS2 node'''
    rclpy.init(args=args)               # Initialize ROS2
    ps4_to_thrust_node = PS4toThrust()  # Create an instance of PS4toThrust
    rclpy.spin(ps4_to_thrust_node)      # Keep the node running
    ps4_to_thrust_node.destroy_node()   # Clean up the node, when done
    rclpy.shutdown()                    # Shutdown ROS2


if __name__ == '__main__':
    main()
