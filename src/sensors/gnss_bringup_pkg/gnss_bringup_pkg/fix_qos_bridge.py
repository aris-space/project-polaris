from rclpy.node import Node
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix


class FixQosBridge(Node):
    """Bridge /fix (best-effort) to /ntrip_client/fix (reliable)."""

    def __init__(self):
        super().__init__("fix_qos_bridge")

        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(NavSatFix, "/ntrip_client/fix", reliable_qos)
        self._sub = self.create_subscription(
            NavSatFix, "/fix", self._on_fix, qos_profile_sensor_data
        )

    def _on_fix(self, msg: NavSatFix):
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FixQosBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

