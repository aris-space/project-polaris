import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mavros_msgs.msg import OverrideRCIn


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


class PS4toPWM(Node):
    '''
    A Class for creating a ROS2 node that converts PS4 controller R2 axis input to PWM signals.
    It subscribes to the '/joy' topic to receive joystick inputs and publishes
    the PWM signals to the '/mavros/rc/override' topic.
    '''

    def __init__(self):
        super().__init__('ps4_to_mavros')
        self.thrust_publisher = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def pwm_mapper(self, thrust):
        '''Maps thrust value [1,-1] (value that /joy publishes) to PWM range [1000,1900]'''
        min_pwm = 1000
        max_pwm = 1900
        pwm = int((thrust + 1) / 2 * (max_pwm - min_pwm) + min_pwm) 
        return pwm

    def joy_callback(self, msg):

        R2_joy_input = msg.axes[5]                            # R2 axis value
        pwm = self.pwm_mapper(-R2_joy_input)                    # Map R2 input to PWM value [1000,1900]

        msg_out = OverrideRCIn()                            # Create empty OverrideRCIn message
        msg_out.channels = [0] * 8                          # Initialize all channels to zero
        msg_out.channels[0] = pwm                           # Assign mapped thrust value to empty message
        msg_out.channels[1] = pwm                             # Set the PWM value for the 2nd channel
        msg_out.channels[2] = pwm                             # Set the PWM value for the 3rd channel
        msg_out.channels[3] = pwm                             # Set the PWM value for the 4th channel
        msg_out.channels[4] = pwm                             # Set the PWM value for the 5th channel
        msg_out.channels[5] = pwm                             # Set the PWM value for the 6th channel
        msg_out.channels[6] = pwm                             # Set the PWM value for the 7th channel
        msg_out.channels[7] = pwm                             # Set the PWM value for the 8th channel
        
        self.thrust_publisher.publish(msg_out)              # Publish the thrust command



def main(args=None):
    '''Main function to run the PS4 to PWM ROS2 node'''
    rclpy.init(args=args)               # Initialize ROS2
    ps4_to_pwm_node = PS4toPWM()  # Create an instance of PS4toPWM
    rclpy.spin(ps4_to_pwm_node)      # Keep the node running
    ps4_to_pwm_node.destroy_node()   # Clean up the node, when done
    rclpy.shutdown()                    # Shutdown ROS2


if __name__ == '__main__':
    main()
