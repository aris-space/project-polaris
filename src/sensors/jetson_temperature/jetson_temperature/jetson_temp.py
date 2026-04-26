import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class JetsonTemperature(Node):
    def __init__(self):
        super().__init__("jetson_temperature")
        self.publisher = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("Jetson Temperature Node started ")

    
    
    def read_thermal_zone(self, zone_number):
        """Safely reads a thermal zone file and returns the temperature in Celsius."""
        try:
            with open(f'/sys/class/thermal/thermal_zone{zone_number}/temp', 'r') as f:
                return float(f.read().strip()) / 1000.0
        except (IOError, ValueError):
            return None

    
    def timer_callback(self):
        
        cpu_temp = self.read_thermal_zone(0)
        #gpu_temp = self.read_thermal_zone(1)
        tj_temp = self.read_thermal_zone(8)
        
        temperatures = {
            'cpu': cpu_temp,
            #'gpu': gpu_temp,
            'junction': tj_temp,
        }

        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "Jetson: Temperatures"
        status.level = DiagnosticStatus.OK
        status.message = "OK"

        for name, temp in temperatures.items():
            if temp is None:
                continue
            status.values.append(KeyValue(key=f"{name}_C", value=f"{temp:.2f}"))
            if temp > 90:
                status.level = DiagnosticStatus.ERROR
                status.message = f"{name}: {temp:.1f}°C is dangerously high"
            elif temp > 80 and status.level != DiagnosticStatus.ERROR:
                status.level = DiagnosticStatus.WARN
                status.message = f"{name}: {temp:.1f}°C is too high"

        diag_msg.status.append(status)
        self.publisher.publish(diag_msg)



def main(args=None):
    rclpy.init(args=args)
    node = JetsonTemperature()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()