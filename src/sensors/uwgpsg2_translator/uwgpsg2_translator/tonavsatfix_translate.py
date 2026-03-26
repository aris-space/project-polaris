import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geographic_msgs.msg import GeoPointStamped


class ToNavSatFixTranslator(Node):
    def __init__(self):
        super().__init__("to_navsatfix_translator")
        self.sub = self.create_subscription(
            GeoPointStamped,
            "/waterlinked_ugps/locator_position_global",
            self.on_geopointstamped,
            10,
        )
        self.pub = self.create_publisher(
            NavSatFix,
            "/waterlinked_ugps/navsatfix",
            10,
        )
        self.get_logger().info("ToNavSatFixTranslator started")

    def on_geopointstamped(self, msg: GeoPointStamped):
        out = NavSatFix()
        out.header = msg.header
        out.latitude = msg.position.latitude
        out.longitude = msg.position.longitude
        out.altitude = msg.position.altitude

        # Mark as valid satellite-like fix for downstream consumers
        out.status.status = NavSatStatus.STATUS_FIX
        out.status.service = NavSatStatus.SERVICE_GPS

        # We do not provide covariance yet (TODO: add covariance)
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ToNavSatFixTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()