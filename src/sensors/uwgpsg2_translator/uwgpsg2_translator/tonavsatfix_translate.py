import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geographic_msgs.msg import GeoPointStamped


class ToNavSatFixTranslator(Node):
    def __init__(self):
        super().__init__("to_navsatfix_translator")
        # Water Linked node publishes GeoPointStamped with default depth-10 (RELIABLE).
        self.sub = self.create_subscription(
            GeoPointStamped,
            "/waterlinked_ugps/locator_position_global",
            self.on_geopointstamped,
            10,
        )
        # BEST_EFFORT so selector (and navsat_transform gps/fix) SensorDataQoS subscribers match.
        self.pub = self.create_publisher(
            NavSatFix,
            "/waterlinked_ugps/navsatfix",
            qos_profile_sensor_data,
        )
        self.get_logger().info("ToNavSatFixTranslator started")

    def on_geopointstamped(self, msg: GeoPointStamped):
        out = NavSatFix()
        out.header = msg.header
        out.header.frame_id = "sbl_link" #Verify this is the correct frame_id
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