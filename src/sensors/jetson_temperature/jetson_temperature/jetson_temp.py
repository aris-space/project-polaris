from jtop import jtop
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class JetsonTemperature(Node):
    def __init__(self):
        super().__init__("jetson_temperature")
        self.publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.jtop = jtop()
        self.jtop.start()
        self.get_logger().info("Jetson Temperature Node started")

    def timer_callback(self):
        
                    
        cpu_temp = self.jtop.temperature['CPU']
        junction_temp = self.jtop.temperature['tj']
        board_temp = self.jtop.temperature['Tboard']
        
        temperatures_dict = {
            'CPU temperature': cpu_temp,
            'Junction temperature': junction_temp,
            'Board temperature': board_temp
        }

        diag_msg = DiagnosticArray()

        

        for i, temp in temperatures_dict.items():

            if temp is None:
                continue

            level = DiagnosticStatus.OK
            message = "OK"
            
            if temp > 90:
                level = DiagnosticStatus.ERROR
                message = f"{i}: {temp}°C is dangerously high"
                
            elif temp > 80:
                level = DiagnosticStatus.WARN
                message = f"{i}: {temp}°C is too high"
                
            
            status = DiagnosticStatus()
            status.name = i
            status.level = level
            status.values = [KeyValue(key=i, value=str(temp))]    
            status.message = message
            
            diag_msg.status.append(status)

    
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(diag_msg)



def main(args=None):
    rclpy.init(args=args)
    jetson_temperature = JetsonTemperature()
    rclpy.spin(jetson_temperature)
    jetson_temperature.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()