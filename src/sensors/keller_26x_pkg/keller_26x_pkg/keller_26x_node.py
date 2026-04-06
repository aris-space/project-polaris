"""
TODO: Add dependencies!!!!!!
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure

from config_pkg.constants import Ports

from keller_protocol import keller_protocol as kp
# use pip install keller-protocol
# https://github.com/KELLERAGfuerDruckmesstechnik/keller_protocol_python


# TODO: Make the udev rule if not done just use: /dev/ttyUSB0 and to check ls /dev/ttyUSB*
# On Windows it is like "COM3" e.g.
class Keller26xNode(Node):

    def __init__(self):
        super().__init__('keller_26x_pressure')

        self.bus = kp.KellerProtocol(port=Ports.KELLER_SENSOR, baud_rate=9600, timeout=0.3, echo=False)
        self.address = 1
        self.p1_Pa = 0.0
        self.serial_number = None
        self.f73_channels = {
            "CH0": 0,
            "P1": 1,
            "P2": 2,
            "T": 3,
            "TOB1": 4,
            "TOB2": 5,
            "ConTc": 10,
            "ConRaw": 11,
        }

        self.init_f48()

        self.pub = self.create_publisher(
            FluidPressure, 
            'sensors/keller26x/pressure', 
            10,
        )

        timer_period = 0.5  # TODO: How high needed?
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def init_f48(self):
        """
        To be able to communicate with the transmitter you will have to use F48 first to initialize.
        """
        self.bus.f48(self.address)

    def measure_p1(self) -> float:
        """Get pressure P1

        :return: pressure
        """
        pressure = self.bus.f73(self.address, self.f73_channels["P1"])
        return pressure
    
    
    def timer_callback(self):
        msg_P = FluidPressure()
        p1_bar = self.measure_p1()
        self.p1_Pa = p1_bar * 100000.0
        msg_P.fluid_pressure = self.p1_Pa

        self.pub.publish(msg_P)
        self.get_logger().info(
            f"keller_pressure={self.p1_Pa}", throttle_duration_sec=5.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = Keller26xNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()