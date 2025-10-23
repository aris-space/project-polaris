import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from mavros_msgs.msg import OverrideRCIn

class ThrustToPixhawk(Node):

    '''
    A Class for creating a ROS2 node that converts thrust commands to Pixhawk actuator control messages.
    It subscribes to the '/thrust_command' topic to receive thrust inputs and publishes
    the actuator control messages to the '/mavros/actuator_control' topic.
    '''

    def __init__(self):
        super().__init__('thrust_to_pixhawk')
        self.actuator_publisher = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.thrust_subscription = self.create_subscription(Float32, '/thrust_command', self.thrust_callback, 10)

    def thrust_callback(self, msg):
        thrust_value = msg.data
        # Map thrust to PWM (assumes thrust in 0.0..1.0). Adjust ranges if different.
        min_pwm = 800
        max_pwm = 1900
        normalized = max(0.0, min(1.0, float(thrust_value)))
        pwm = int(min_pwm + normalized * (max_pwm - min_pwm))
        rc_msg = OverrideRCIn()                 # Create empty OverrideRCIn message
        rc_msg.channels = [0] * 8
        rc_msg.channels[4] = pwm                 # Set the PWM value for the 5th channel
        rc_msg.channels[5] = pwm                 # Set the PWM value for the 6th channel
        rc_msg.channels[6] = pwm                 # Set the PWM value for the 7th channel
        rc_msg.channels[7] = pwm                 # Set the PWM value for the 8th channel
        self.actuator_publisher.publish(rc_msg)     # Publish the actuator control message

def main():
    '''Main function to run the Thrust to Pixhawk ROS2 node'''
    rclpy.init()                                # Initialize ROS2
    thrust_to_pixhawk_node = ThrustToPixhawk()  # Create an instance of ThrustToPixhawk
    rclpy.spin(thrust_to_pixhawk_node)          # Keep the node running
    thrust_to_pixhawk_node.destroy_node()       # Clean up the node, when done
    rclpy.shutdown()                            # Shutdown ROS2

if __name__ == '__main__':
    main()