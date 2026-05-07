import rclpy
from rclpy.node import Node
import serial
import time

class UARTDebugNode(Node):
    def __init__(self):
        super().__init__('uart_debug_node')
        self.device = '/dev/ttyTHS1'
        self.baud = 115200
        
        try:
            self.ser = serial.Serial(
                port=self.device,
                baudrate=self.baud,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            self.get_logger().info(f'Opened {self.device} at {self.baud} baud.')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            return

        self.timer = self.create_timer(1.0, self.send_test_data)

    def send_test_data(self):
        test_str = f"HEARTBEAT_TEST_{time.time()}\n"
        try:
            self.ser.write(test_str.encode('utf-8'))
            self.get_logger().info(f'Sent: {test_str.strip()}')
            
            # Try to read any response back
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode('utf-8', errors='replace').strip()
                self.get_logger().info(f'Received: {response}')
        except Exception as e:
            self.get_logger().error(f'Error during UART comms: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = UARTDebugNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()