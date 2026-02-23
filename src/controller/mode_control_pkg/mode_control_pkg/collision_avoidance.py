import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool


class CollisionAvoidanceNode(Node):
    def __init__(self):
        super().__init__("collision_avoidance_node")

        self.trigger_distance = 0.5  # Distance threshold for triggering emergency stop
        self.rearm_delay = (
            10.0  # Time in seconds to wait before reactivating after an emergency stop
        )
        self.checking = (
            True  # Flag to indicate if we are currently checking for distance
        )
        self.rearm_distance = (
            0.7  # Distance threshold for rearming the system after an emergency stop
        )
        self.rearm_timer = None
        self.trigger_time = None
        self.distance = float("inf")  # Initialize distance to infinity

        self.get_logger().info("CollisionAvoidanceNode: Node has been initialized")

        self.distance_subscriber = self.create_subscription(
            Float32, "front/ultrasonic/distance", self.distance_cb, 10
        )

        self.mode_subscriber = self.create_subscription(
            String, "/mode_control/current_mode", self.mode_cb, 10
        )

        self.checking_subscriber = self.create_subscription(
            Bool, "/obstacle_avoidance/checking", self.checking_cb, 10
        )

        self.pixhawk_mode_publisher = self.create_publisher(
            String, "/pixhawk/mode_cmd", 10
        )

        self.mode_publisher = self.create_publisher(
            String, "/mode_control/current_mode", 10
        )

        self.current_mode = ""
        self.pixhawk_mode = ""

    def mode_cb(self, msg):
        """
        Called when a new mode is published by mode_control_node. This is important because otherwise if the mode is changed after the collision avoidance
        node is triggered, it wont trigger the emergency stop mode again if not updated.
        """
        self.current_mode = msg.data

    def checking_cb(self, msg):

        self.checking = msg.data

        if self.checking == False:
            self.get_logger().info("Collision avoidance system deactivated.")

    def publish_mode(self, mode: str):
        mode_msg = String()
        mode_msg.data = mode
        self.mode_publisher.publish(mode_msg)
        self.get_logger().info(f"Mode changed! New Mode: {mode}")

    def publish_pixhawk_mode(self, mode: str):
        pixhawk_mode_msg = String()
        pixhawk_mode_msg.data = mode
        self.pixhawk_mode_publisher.publish(pixhawk_mode_msg)

    def rearm_callback(self):
        time_elapsed = (self.get_clock().now() - self.trigger_time).nanoseconds / 1e9
        if time_elapsed >= self.rearm_delay:
            if self.distance >= self.rearm_distance:
                self.checking = True
                self.rearm_timer.cancel()
                self.rearm_timer = None
                self.get_logger().info("Collision avoidance system rearmed.")

    def distance_cb(self, msg):
        self.distance = msg.data
        # self.get_logger().info(f'Distance: {self.distance}')

        if self.distance < self.trigger_distance and self.checking:
            # Trigger emergency stop: same as mode_control_node
            if self.current_mode != "emergency_stop":
                self.current_mode = "emergency_stop"
                self.pixhawk_mode = "MANUAL"
                self.publish_mode("emergency_stop")
                self.publish_pixhawk_mode("MANUAL")

            if self.rearm_timer is None:
                self.trigger_time = self.get_clock().now()
                self.rearm_timer = self.create_timer(0.5, self.rearm_callback)
