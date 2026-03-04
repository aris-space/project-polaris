import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from config_pkg.constants import Ports

DISCONNECTED_TEMP = -127.00  # DallasTemperature returns this for disconnected sensors


class Temperature_sensor(Node):
    def __init__(self):
        super().__init__("temperature_sensor_node")
        self.publisher_ = self.create_publisher(
            Float32MultiArray, "/temperature_sensors", 10
        )

        self.diagnostic_publisher_ = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        
        self.warn_level = 30
        self.error_level = 50


        self.get_logger().info(
            "Temperature Sensor Node started. warn_level=%.1f°C, error_level=%.1f°C"
            % (self.warn_level, self.error_level)
        )

        self.serial = serial.Serial(
            port=Ports.ARDUINO_PORT,
            baudrate=9600,
            timeout=0,
        )

        self.get_logger().info(f"Successfully opened serial port {Ports.ARDUINO_PORT}")

        self.timer = self.create_timer(0.01, self.timer_callback)
        self.buffer = bytearray()
        self.max_buffer_size = 1024  # Maximum buffer size to prevent overflow

    def timer_callback(self):

        n = self.serial.in_waiting
        if n > 0:
            self.buffer.extend(self.serial.read(n))

        while b"\n" in self.buffer:
            line, _, rest = self.buffer.partition(b"\n")
            self.buffer = bytearray(rest)

            try:
                line = line.decode("utf-8")
            except UnicodeDecodeError:
                self.get_logger().warn(f"Invalid UTF-8 line dropped: {line!r}")
                continue

            if not line.startswith("DATA,"):
                # Ignore non-data lines safely
                continue

            payload = line[5:]  # remove "DATA,"

            try:
                values = [
                    float(x.strip()) for x in payload.split(",") if x.strip() != ""
                ]
            except ValueError:
                self.get_logger().warn(f"Malformed DATA line dropped: {line!r}")
                continue

            if not values:
                continue


            msg = Float32MultiArray()
            msg.data = values
            self.publisher_.publish(msg)
            self.get_logger().info(
                "Temperatures [°C]: [%s]"
                % ", ".join(f"{v:.2f}" for v in values)
            )


            for sensor_i in range(len(values)):
                if values[sensor_i] > 26: #TODO: Might need to be adapted
                    self.get_logger().warning(f"Sensor {sensor_i} is hot: {values[sensor_i]}°C") #TODO: Add which sensor_i corresponds to which position in the Hardware
                    
            status = DiagnosticStatus()
            status.name = "Temperature Sensors"
            status.hardware_id = Ports.ARDUINO_PORT
            status.values = [
                KeyValue(key=f"sensor_{i}", value=f"{v:.2f}")
                for i, v in enumerate(values)
            ]

            level = DiagnosticStatus.OK
            message = "OK"
            
            for i, v in enumerate(values):
                if v <= DISCONNECTED_TEMP:
                    level = DiagnosticStatus.ERROR
                    message = f"Sensor {i} disconnected (-127°C)"
                    
                elif v >= self.error_level:
                    level = DiagnosticStatus.ERROR
                    message = f"Sensor {i} temperature >= {self.error_level}°C"
                    
                elif v >= self.warn_level and level < DiagnosticStatus.WARN:
                    level = DiagnosticStatus.WARN
                    message = f"Sensor {i} temperature >= {self.warn_level}°C"

            status.level = level
            status.message = message

            diag_msg = DiagnosticArray()
            diag_msg.header.stamp = self.get_clock().now().to_msg()
            diag_msg.status.append(status)
            self.diagnostic_publisher_.publish(diag_msg)





def main():
    rclpy.init()
    temperature_sensor_node = Temperature_sensor()
    rclpy.spin(temperature_sensor_node)
    temperature_sensor_node.destroy_node()
    rclpy.shutdown()
