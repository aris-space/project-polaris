import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mavros_msgs.msg import OverrideRCIn

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
        '''Maps thrust value [1,-1] (value that /joy publishes) to PWM range [1000,2000]'''
        min_pwm = 1000
        max_pwm = 1900 
        pwm = int((thrust + 1) / 2 * (max_pwm - min_pwm) + min_pwm) 
        return pwm

    def joy_callback(self, msg):

        r2_joy_input = msg.axes[5]                            # R2 axis value
        # Invert the R2 axis input
        pwm = self.pwm_mapper(-r2_joy_input)                    # Map R2 input to PWM value [1000,2000]

        msg_out = OverrideRCIn()                            # Create empty OverrideRCIn message

        msg_out.channels[0] = pwm                             # Set the PWM value for the 1st channel
        msg_out.channels[2] = 0                             # Set the PWM value for the 3


        # Set all channels to the same PWM value:
        # msg_out.channels = [pwm] * 8

        
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
