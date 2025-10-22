import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from mavros_msgs.msg import ActuatorControl

class ThrustToPixhawk(Node):

    '''
    A Class for creating a ROS2 node that converts thrust commands to Pixhawk actuator control messages.
    It subscribes to the '/thrust_command' topic to receive thrust inputs and publishes
    the actuator control messages to the '/mavros/actuator_control' topic.
    '''

    def __init__(self):
        super().__init__('thrust_to_pixhawk')
        self.actuator_publisher = self.create_publisher(ActuatorControl, '/mavros/actuator_control', 10)
        self.thrust_subscription = self.create_subscription(Float32, '/thrust_command', self.thrust_callback, 10)

    def thrust_callback(self, msg):
        thrust_value = msg.data                     # Received thrust value
        actuator_msg = ActuatorControl()                 # Create empty ActuatorControl message
        actuator_msg.group_mix = 0                        # Set group mix (0 for main motors)
        actuator_msg.controls[0] = thrust_value           # Set msg value to thrust value
        self.actuator_publisher.publish(actuator_msg)     # Publish the actuator control message

def main():
    '''Main function to run the Thrust to Pixhawk ROS2 node'''
    rclpy.init()                                # Initialize ROS2
    thrust_to_pixhawk_node = ThrustToPixhawk()  # Create an instance of ThrustToPixhawk
    rclpy.spin(thrust_to_pixhawk_node)          # Keep the node running
    thrust_to_pixhawk_node.destroy_node()       # Clean up the node, when done
    rclpy.shutdown()                            # Shutdown ROS2

if __name__ == '__main__':
    main()