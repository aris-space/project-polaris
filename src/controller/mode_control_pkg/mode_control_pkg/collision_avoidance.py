import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from collections import deque
from rcl_interfaces.msg import SetParametersResult


class CollisionAvoidanceNode(Node):
    def __init__(self):
        super().__init__("collision_avoidance_node")

        self.checking = False  # Set via /collision_avoidance/checking (operator controlled)
        self.manual_mode_published = False
        self.collision_triggered = False  # True after a collision disarm until checking is re-enabled
        self.distance = float("inf")
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

        self.pixhawk_mode_publisher = self.create_publisher(String, "/pixhawk/mode_cmd", 10)
        self.mode_publisher = self.create_publisher(String, "/mode_control/current_mode", 10)
        self.arm_cmd_publisher = self.create_publisher(Bool, "/pixhawk/arm_cmd", 10)

        self.current_mode = ""

    def min_distance_callback(self, params):
        for param in params:
            if param.name == "min_distance":
                self.min_distance = float(param.value)
                self.get_logger().info(f"Min distance updated: {self.min_distance}")
                return SetParametersResult(successful=True)
        return SetParametersResult(successful=False)

    def mode_cb(self, msg):
        self.current_mode = msg.data

    def checking_cb(self, msg):
        """
        Operator controls checking via /collision_avoidance/checking.
        True = collision avoidance active. False = disabled, restore MANUAL so operator can drive.
        """
        self.checking = bool(msg.data)
        if not self.checking:
            self.get_logger().info("Collision avoidance disabled by operator.")
            self._publish_mode("MANUAL")
            self.current_mode = "MANUAL"
            self.manual_mode_published = True
            self.collision_triggered = False
        else:
            self.get_logger().info("Collision avoidance enabled by operator.")
            self.manual_mode_published = False

    def _publish_mode(self, mode: str):
        mode_msg = String()
        mode_msg.data = mode
        self.mode_publisher.publish(mode_msg)
        self.pixhawk_mode_publisher.publish(mode_msg)
        self.get_logger().info(f"Mode → {mode}")

    def _disarm(self):
        msg = Bool()
        msg.data = False
        self.arm_cmd_publisher.publish(msg)
        self.get_logger().info("Collision avoidance: disarm sent")

    def distance_cb(self, msg):
        self.distance = msg.data
        self.distance_averager.append(self.distance)
        self.distance = sum(self.distance_averager) / len(self.distance_averager)

        # When operator has disabled, ensure MANUAL mode for driving
        if not self.checking and not self.manual_mode_published:
            self._publish_mode("MANUAL")
            self.manual_mode_published = True
        elif self.checking:
            self.manual_mode_published = False

        # Trigger disarm + MANUAL when too close (once per enable cycle)
        if (
            self.checking
            and not self.collision_triggered
            and len(self.distance_averager) == self.distance_averager.maxlen
            and self.distance < self.min_distance
        ):
            self.collision_triggered = True
            self._disarm()
            self._publish_mode("MANUAL")  # TODO: switch to POS_HOLD once available


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
