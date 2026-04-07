import rclpy
from rclpy import parameter_service
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from collections import deque
from rcl_interfaces.msg import SetParametersResult


class CollisionAvoidanceNode(Node):
    def __init__(self):
        super().__init__("collision_avoidance_node")

        #self.trigger_distance = 0.05  # Distance threshold for triggering emergency stop
        self.checking = False  # Set via /collision_avoidance/checking (operator controlled)
        self.manual_mode_published = False  # Flag to track if manual mode has been published
        self.distance = float("inf")  # Initialize distance to infinity
        self.distance_averager = deque(maxlen=5)

        self.declare_parameter("min_distance", 0.5)
        self.min_distance = float(self.get_parameter("min_distance").value)
        self.add_on_set_parameters_callback(self.min_distance_callback)


        self.get_logger().info("CollisionAvoidanceNode: Node has been initialized")

        self.distance_subscriber = self.create_subscription(
            Float32, "front/ultrasonic/distance", self.distance_cb, 10
        )

        self.mode_subscriber = self.create_subscription(
            String, "/mode_control/current_mode", self.mode_cb, 10
        )

        self.checking_subscriber = self.create_subscription(
            Bool, "/collision_avoidance/checking", self.checking_cb, 10
        )

        self.pixhawk_mode_publisher = self.create_publisher(
            String, "/pixhawk/mode_cmd", 10
        )

        self.mode_publisher = self.create_publisher(
            String, "/mode_control/current_mode", 10
        )

        self.current_mode = ""
        self.pixhawk_mode = ""

    
    def min_distance_callback(self, params):
        """
        Called when the min_distance parameter is changed.
        """
        for param in params:
            if param.name == "min_distance":
                self.min_distance = float(param.value)
                self.get_logger().info(f"Min distance changed! New Min Distance: {self.min_distance}")
                return SetParametersResult(successful=True)
        
        return SetParametersResult(successful=False)
    
    def mode_cb(self, msg):
        """
        Called when a new mode is published by mode_control_node. This is important because otherwise if the mode is changed after the collision avoidance
        node is triggered, it wont trigger the emergency stop mode again if not updated.
        """
        self.current_mode = msg.data

    def checking_cb(self, msg):
        """
        Operator controls checking via /collision_avoidance/checking.
        True = collision avoidance active. False = disabled, publish manual so operator can drive.
        """
        self.checking = bool(msg.data)
        if not self.checking:
            self.get_logger().info("Collision avoidance disabled by operator.")
            self.publish_mode("manual_control")
            self.publish_pixhawk_mode("MANUAL")
            self.current_mode = "manual_control"
            self.pixhawk_mode = "MANUAL"
            self.manual_mode_published = True
        else:
            self.get_logger().info("Collision avoidance enabled by operator.")
            self.manual_mode_published = False

    def publish_mode(self, mode: str):
        mode_msg = String()
        mode_msg.data = mode
        self.mode_publisher.publish(mode_msg)
        self.get_logger().info(f"Mode changed! New Mode: {mode}")

    def publish_pixhawk_mode(self, mode: str):
        pixhawk_mode_msg = String()
        pixhawk_mode_msg.data = mode
        self.pixhawk_mode_publisher.publish(pixhawk_mode_msg)

    def distance_cb(self, msg):
        self.distance = msg.data
        self.distance_averager.append(self.distance)
        self.distance = sum(self.distance_averager) / len(self.distance_averager)

        # When operator has disabled (checking=false), ensure manual mode for driving
        if not self.checking and not self.manual_mode_published:
            self.publish_mode("manual_control")
            self.publish_pixhawk_mode("MANUAL")
            self.manual_mode_published = True
        elif self.checking:
            self.manual_mode_published = False

        # Trigger emergency when too close and we are checking
        if (
            self.distance < self.min_distance
            and len(self.distance_averager) == self.distance_averager.maxlen
            and self.checking
        ):

            # Trigger emergency stop: same as mode_control_node
            if self.current_mode != "emergency_stop":
                self.current_mode = "emergency_stop"
                self.pixhawk_mode = "MANUAL"
                self.publish_mode("emergency_stop")
                self.publish_pixhawk_mode(
                    "MANUAL"
                )  # TODO: once we are able to implement POS_HOLD, switch to that mode


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
