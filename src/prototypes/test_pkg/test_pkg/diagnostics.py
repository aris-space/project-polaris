import rclpy
from rclpy.node import Node
import diagnostic_updater
from diagnostic_msgs.msg import DiagnosticStatus

class RobotHealthNode(Node):
    def __init__(self):
        super().__init__('robot_health_node')

        # 1. Create the Updater
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID("Main_Chassis_v2")

        # 2. Add a diagnostic task (Name, Function)
        self.updater.add("Temperature Sensor", self.check_temp)

    def check_temp(self, stat):
        # Logic to check your "sensor"
        current_temp = 35.5 
        
        if current_temp < 40.0:
            stat.summary(DiagnosticStatus.OK, "Temperature is normal")
        elif current_temp < 50.0:
            stat.summary(DiagnosticStatus.WARN, "Temperature is rising")
        else:
            stat.summary(DiagnosticStatus.ERROR, "Overheating!")

        # Add key-value pairs for more detail
        stat.add("Current Temp (C)", str(current_temp))
        stat.add("Fan Speed", "High")
        
        return stat

def main():
    rclpy.init()
    node = RobotHealthNode()
    rclpy.spin(node)
    rclpy.shutdown()