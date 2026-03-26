"""
TODO: Add dependencies!!!!!!
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from keller_protocol import keller_protocol as kp
# use pip install keller-protocol
# https://github.com/KELLERAGfuerDruckmesstechnik/keller_protocol_python



class Keller26xNode(Node):

    def __init__(self):
        super().__init__('keller_26x_pressure')

        self.bus = kp.KellerProtocol(port="COM17", baud_rate=9600, timeout=0.3, echo=True)
        self.address = 1
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

        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        
    def init_f48(self) -> str:
        """Initialise and release"""
        answer = self.bus.f48(self.address)
        print(f" Init of Device Address: {self.address} with Firmware: {answer}")

    def get_serial(self) -> int:
        """Get Serial Number from X-Line

        :returns Serial Number
        """
        self.serial_number = self.bus.f69(self.address)
        return self.serial_number

    def get_address(self) -> int:
        return self.address

    def set_address(self, new_address: int) -> int:
        """Change the Device address. -> Has to be unique on the RS485 bus

        :param new_address: New address of the Device
        :return: If successful return new_address otherwise old address and throw exception
        """
        self.address = self.bus.f66(self.address, new_address)
        return self.address

    def measure_tob1(self) -> float:
        """Get temperature TOB1

        :return: temperature
        """
        temperature = self.bus.f73(self.address, self.f73_channels["TOB1"])
        return temperature

    def measure_p1(self) -> float:
        """Get pressure P1

        :return: pressure
        """
        pressure = self.bus.f73(self.address, self.f73_channels["P1"])
        return pressure
    
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = Keller26xNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()