from rclpy.node import Node
from std_msgs.msg import Float32
import yaml
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Trigger
import rclpy
class BatteryTracker(Node):
    def __init__(self):
        super().__init__("battery_tracker")

        self.battery_consumed_subscriber = self.create_subscription(
            Float32, "/pixhawk/battery_consumed", self.battery_consumed_callback, 10)
        
        self.last_battery_consumed = 0
        self.zero_point = 0
        self.battery_consumed = 0.0

        self.reset_service = self.create_service(Trigger, "reset_battery_tracker", self.reset_battery_tracker_callback)
        
        self.declare_parameter("battery_capacity", 10000.0)
        self.battery_capacity = self.get_parameter("battery_capacity").value
        
        self.add_on_set_parameters_callback(self.set_battery_capacity_callback)
        try:
            self.zero_point = yaml.safe_load(open("/home/polaris5/battery_state.yaml"))["global_consumed_mah"]
        except (FileNotFoundError, TypeError, KeyError):
            self.zero_point = 0.0
        
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
            yaml.safe_dump({"global_consumed_mah": self.battery_consumed}, f)
        
    def battery_consumed_callback(self, msg):
        
        self.battery_consumed = msg.data + self.zero_point
        
        if self.battery_consumed < self.last_battery_consumed:
            self.zero_point = self.last_battery_consumed
            
        self.last_battery_consumed = self.battery_consumed

        self.save_to_yaml()


    def reset_battery_tracker_callback(self, req):
        self.zero_point = 0
        self.last_battery_consumed = 0
        self.battery_consumed = 0
        self.save_to_yaml()
        return Trigger.Response(success=True, message="Battery tracker reset")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()