import rclpy
import pigpio
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from config.config import Config

'''
ROS2 Node to convert joystick inputs to PWM signals and writing to GPIO Pin using pigpio.
Extended to support bidirectional control with safety button X.
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

    def set_pwm(self, r2_value, l2_value, safety_button):

        min_pwm = Config.get_pwm_min()  # Minimum pulse width in microseconds
        init_pwm = Config.get_pwm_init()  # Neutral pulse width in microseconds
        max_pwm = Config.get_pwm_max()  # Maximum pulse width in microseconds

        # Normalize trigger inputs from [-1,1] to [0,1]
        r_normalized = max(0.0, min(1.0, (r2_value + 1.0) / 2.0)) # Normalize r2 input to [0,1]
        l_normalized = max(0.0, min(1.0, (l2_value + 1.0) / 2.0)) # Normalize l2 input to [0,1]

        # net is positive when R2 > L2 (forward), negative when L2 > R2 (reverse)
        net = r_normalized - l_normalized

        if safety_button == 0:
            pwm = init_pwm  # Safety button not pressed, set to neutral
            print("Safety button not pressed, setting PWM to neutral. To throttle, press and hold X button.")
        else:
            # Use asymmetric scaling around init_pwm:
            # - For forward (net >= 0) scale up to max_pwm using (max_pwm - init_pwm)
            # - For reverse (net < 0) scale down toward min_pwm using (init_pwm - min_pwm)
            if net >= 0:
                pwm = int(init_pwm + net * (max_pwm - init_pwm))
            else:
                pwm = int(init_pwm + net * (init_pwm - min_pwm))

            # Clamp into valid range
            pwm = max(min_pwm, min(max_pwm, pwm))
            print(f"Setting PWM to {pwm} based on joystick input (net={net}).")
        self.pi.set_servo_pulsewidth(self.pin, pwm)

    def joy_callback(self, msg):
        # Process joystick input and convert to PWM signals
        self.get_logger().info(f"Joystick axes: {msg.axes}")

        r2_axis = Config.get_joy_r2_axis()
        l2_axis = Config.get_joy_l2_axis()
        x_button = Config.get_joy_x_button()
        
        #read out ps 4 controller values
        r2_value = - msg.axes[r2_axis] # Axis 5 for r2 signal, inverted since r2 is negative when pressed
        l2_value = - msg.axes[l2_axis] # Axis 4 for l2 signal, inverted since l2 is negative when pressed
        safety_button = msg.buttons[x_button]  # X button as safety button
        self.set_pwm(r2_value, l2_value, safety_button)

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