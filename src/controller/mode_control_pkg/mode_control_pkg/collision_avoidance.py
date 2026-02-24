import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from collections import deque


class CollisionAvoidanceNode(Node):
    def __init__(self):
        super().__init__("collision_avoidance_node")

        self.trigger_distance = 0.05  # Distance threshold for triggering emergency stop
        self.rearm_delay = (
            10.0  # Time in seconds to wait before reactivating after an emergency stop
        )
        self.checking = (
            True  # Flag to indicate if we are currently checking for distance
        )
        self.manual_mode_published = (
            False  # Flag to track if manual mode has been published
        )
        self.rearm_distance = (
            0.07  # Distance threshold for rearming the system after an emergency stop
        )
        self.rearm_timer = None
        self.trigger_time = None
        self.distance = float("inf")  # Initialize distance to infinity
        self._rearm_last_log_time = 0.0  # For throttled debug logging

        self.distance_averager = deque(maxlen=5)

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

    def mode_cb(self, msg):
        """
        Called when a new mode is published by mode_control_node. This is important because otherwise if the mode is changed after the collision avoidance
        node is triggered, it wont trigger the emergency stop mode again if not updated.
        """
        self.current_mode = msg.data

    def checking_cb(self, msg):
        """
        Operator publishes False to get manual control (move vehicle away from obstacle).
        Operator does not publish True - rearming is done internally when safe.
        """
        if msg.data is False:
            self.checking = False
            self.get_logger().info("Collision avoidance disabled by operator.")
            self.publish_mode("manual_control")
            self.publish_pixhawk_mode("MANUAL")
            self.current_mode = "manual_control"
            self.pixhawk_mode = "MANUAL"
            self.manual_mode_published = True

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
        """
        Runs every 0.2s after emergency. Rearms only when:
        - 10 seconds have elapsed since emergency, AND
        - Distance >= rearm_distance (safe zone).
        Stays in manual until safe; never auto-rearms in dangerous area.
        Once rearmed (checking=True), emergency will trigger again when close.
        """
        if self.trigger_time is None:
            return

        time_elapsed = (self.get_clock().now() - self.trigger_time).nanoseconds / 1e9
        if time_elapsed < self.rearm_delay:
            return

        if self.distance < self.rearm_distance:
            # Still in dangerous area - keep timer running (throttled debug)
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._rearm_last_log_time >= 5.0:
                self.get_logger().info(
                    f"Rearm waiting: need distance>={self.rearm_distance:.2f}m "
                    f"(current={self.distance:.3f}m), elapsed={time_elapsed:.1f}s"
                )
                self._rearm_last_log_time = now
            return

        # Safe distance reached - rearm collision avoidance.
        # checking=True so emergency can trigger again when close.
        self.checking = True
        self.trigger_time = None
        if self.rearm_timer is not None:
            self.rearm_timer.cancel()
            self.rearm_timer = None
        self.get_logger().info(
            f"Safe distance reached. Collision avoidance rearmed (checking=True). "
            f"Emergency will trigger again when distance < {self.trigger_distance:.2f} m."
        )

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
            self.distance < self.trigger_distance
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

            if self.rearm_timer is None:
                self.trigger_time = self.get_clock().now()
                self.rearm_timer = self.create_timer(0.2, self.rearm_callback)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
