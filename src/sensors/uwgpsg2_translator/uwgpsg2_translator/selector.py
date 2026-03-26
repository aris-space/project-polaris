import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class Selector(Node):
    def __init__(self):
        super().__init__("selector")
        self.get_logger().info("Selector node initialized")
        
        self.ekf_gps_publisher = self.create_publisher(NavSatFix, "/gps/selected", 10)
        self.ekf_gps_subscriber = self.create_subscription(NavSatFix, "/fix", self.ekf_gps_callback, 10)
        self.sbl_gps_subscriber = self.create_subscription(NavSatFix, "/waterlinked_ugps/navsatfix", self.sbl_gps_callback, 10)

        self.last_gps_time = None
        self.gps_timeout_seconds = 2.0

    def ekf_gps_callback(self, msg):
        # Update the timestamp of the last received GPS message
        self.last_gps_time = self.get_clock().now()
        
        # GPS is currently active and fresh, so publish it immediately
        self.ekf_gps_publisher.publish(msg)
        
    def sbl_gps_callback(self, msg):
        # If we have never received a GPS message, publish SBL
        if self.last_gps_time is None:
            self.ekf_gps_publisher.publish(msg)
            return
            
        # Calculate time since last GPS message
        duration_since_gps = (self.get_clock().now() - self.last_gps_time).nanoseconds / 1e9
        
        # If the GPS is stale (> 2 seconds), fallback to SBL
        if duration_since_gps > self.gps_timeout_seconds:
            self.ekf_gps_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    selector = Selector()
    rclpy.spin(selector)
    selector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()