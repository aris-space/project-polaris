import rclpy
import pigpio
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

'''
ROS2 Node to convert joystick inputs to PWM signals and writing to GPIO Pin using pigpio.
COMMENTS:
- Ensure pigpio library is installed: `pip install pigpio`
- pigpio daemon must be running: `sudo pigpiod`
- Pigpio can only be installed on Raspberry Pi.
'''

class PWMNode(Node):
    def __init__(self):
        super().__init__('pwm_node')
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Could not connect to pigpio daemon!")
            return
        self.pin = 18  # GPIO pin
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def set_pwm(self, r2_value):
        min_pwm = 1000  # Minimum pulse width in microseconds
        max_pwm = 1900  # Maximum pulse width in microseconds
        pwm = int((r2_value + 1) / 2 * (max_pwm - min_pwm) + min_pwm)  # Convert r2-input to pwm range
        self.pi.set_servo_pulsewidth(self.pin, pwm)

    '''
    With duty cycle and frequency [Hz], we can calculate the max pulse width in microseconds.
    Thus for a frequency of 50Hz, the max PWM is ~2000 microseconds at 50Hz.
    '''

    def joy_callback(self, msg):
        # Process joystick input and convert to PWM signals
        self.get_logger().info(f"Joystick axes: {msg.axes}")
        r2_value = - msg.axes[5] # Axis 5 for r2 signal, inverted since r2 is negative when pressed
        self.set_pwm(r2_value)

    def destroy_node(self):
        ''' Make sure to stop pigpio daemon on node destruction '''
        try:
            self.pi.set_servo_pulsewidth(self.pin, 0)  # stop pulses
            self.pi.stop()
            return
        except Exception:
            pass
        super().destroy_node()
        

def main(args=None):
    rclpy.init(args=args)
    pwm_node = PWMNode()
    rclpy.spin(pwm_node)
    pwm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 