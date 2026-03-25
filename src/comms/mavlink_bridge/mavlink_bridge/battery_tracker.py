from rclpy.node import Node
from std_msgs.msg import Float32
import yaml
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Trigger
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
class BatteryTracker(Node):
    def __init__(self):
        super().__init__("battery_tracker")

        self.battery_consumed_subscriber = self.create_subscription(
            Float32, "/pixhawk/battery_consumed", self.battery_consumed_callback, 10)
        
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10)

        self.remaining = 1.0
        self.last_raw_consumed = 0.0   # last raw msg.data from Pixhawk
        self.zero_point = 0.0          # accumulated mAh from before current Pixhawk session
        self.battery_consumed = 0.0    # total = zero_point + raw
        self.remaining = 1.0
        self.startup_msgs_remaining = 5  # ignore reset detection for first N messages after node start

        self.reset_service = self.create_service(Trigger, "/reset_battery_tracker", self.reset_battery_tracker_callback)
        
        self.declare_parameter("battery_capacity", 10000.0)
        self.battery_capacity = self.get_parameter("battery_capacity").value
        
        self.add_on_set_parameters_callback(self.set_battery_capacity_callback)
        try:
            data = yaml.safe_load(open("/home/polaris5/battery_state.yaml"))
            self.zero_point = data["zero_point"]
            self.last_raw_consumed = data.get("last_raw_consumed", 0.0)
        except (FileNotFoundError, TypeError, KeyError):
            self.zero_point = 0.0
            self.last_raw_consumed = 0.0
        
        self.remaining_publisher = self.create_publisher(
            Float32, "/pixhawk/battery_remaining", 10)
        
        self.timer = self.create_timer(1.0, self.publish_remaining_callback)
        
    def set_battery_capacity_callback(self, params):
        for param in params:
            if param.name == "battery_capacity":
                self.battery_capacity = float(param.value)
        return SetParametersResult(successful=True)

    def publish_remaining_callback(self):
        remaining_msg = Float32()
        remaining_msg.data = (1.0 - self.battery_consumed / self.battery_capacity) * 100
        self.remaining_publisher.publish(remaining_msg)
        
    def save_to_yaml(self):
        with open("/home/polaris5/battery_state.yaml", "w") as f:
            yaml.safe_dump({
                "zero_point": self.zero_point,
                "last_raw_consumed": self.last_raw_consumed,
            }, f)
        
    def battery_consumed_callback(self, msg):
        raw = float(msg.data)

        # Skip reset detection during startup to ignore brief zero/junk readings on reconnect
        if self.startup_msgs_remaining > 0:
            self.startup_msgs_remaining -= 1
        elif self.last_raw_consumed - raw > 500:
            # Pixhawk reset detected: raw counter dropped significantly
            self.zero_point = self.battery_consumed  # preserve running total

        self.last_raw_consumed = raw
        self.battery_consumed = raw + self.zero_point

        self.save_to_yaml()

        self.remaining = (self.battery_capacity - self.battery_consumed) / self.battery_capacity

        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "Battery"
        status.level = DiagnosticStatus.OK
        status.message = "OK"
        status.values = [
            KeyValue(key="battery_remaining", value=f"{(self.remaining * 100):.2f} %"),
            KeyValue(key="battery_consumed_mah", value=f"{self.battery_consumed:.0f} mAh"),
        ]
        
        if self.remaining < 0.2:
            status.level = DiagnosticStatus.ERROR
            status.message = f"Battery is critically low {self.remaining:.2f}%, GET TO A SAFE LANDING ASAP!!"
        elif self.remaining < 0.35:
            status.level = DiagnosticStatus.WARN
            status.message = f"Battery is very low: {self.remaining:.2f}%"
        
        
        diag_msg.status.append(status)
        


        self.diagnostic_publisher.publish(diag_msg)
    
    def reset_battery_tracker_callback(self, req, res):
        self.zero_point = 0.0
        self.last_raw_consumed = 0.0
        self.battery_consumed = 0.0
        self.startup_msgs_remaining = 5
        self.save_to_yaml()
        res.success = True
        res.message = "Battery tracker reset"
        return res

def main(args=None):
    rclpy.init(args=args)
    node = BatteryTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()